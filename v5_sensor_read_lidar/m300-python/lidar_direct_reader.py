import socket
import struct
import math
import threading
import time
import numpy as np

class LidarDirectReader:
    def __init__(self, ip="0.0.0.0", port=6669):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, self.port))
        self.sock.settimeout(1.0)
        
        self.running = False
        self.thread = None
        
        # 缓存最新数据
        self.lock = threading.Lock()
        
        # IMU 数据 [gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z]
        self.latest_imu = {
            "gyro": [0.0, 0.0, 0.0],
            "acc": [0.0, 0.0, 0.0],
            "timestamp": 0
        }
        self.imu_update_count = 0
        
        # 点云数据缓存 (这里仅缓存最新的一帧数据，或者按需扩展)
        # 为避免内存爆炸，我们限制缓存大小
        self.latest_pointcloud = []
        self.pc_timestamp = 0

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.thread.start()
        print(f"[LidarDirectReader] Started listening on UDP {self.port}")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        self.sock.close()
        print("[LidarDirectReader] Stopped.")

    def _recv_loop(self):
        # 预编译 struct 格式以加速解析
        # PC header: version(B), length(H), time_interval(H), dot_num(H), udp_cnt(H), frame_cnt(B), data_type(B), time_type(B), rsvd(12s), crc32(I), timestamp(Q)
        pc_header_fmt = struct.Struct('< B H H H H B B B 12s I Q')
        # PC point: depth+theta_hi(I), theta_lo+phi(I), reflectivity(B), tag(B)
        pc_point_fmt = struct.Struct('< I I B B')
        
        # IMU: Header(B), Accel_X(h), Accel_Y(h), Accel_Z(h), Gyro_X(h), Gyro_Y(h), Gyro_Z(h), T(b), TS(H), timestamp(Q)
        imu_fmt = struct.Struct('< B h h h h h h b H Q')
        
        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[LidarDirectReader] Recv error: {e}")
                continue
                
            data_len = len(data)
            
            # 判断是否为 IMU 包 (33 字节, 头标志为 0xfa 0x88)
            if data_len == 33 and data[0] == 0xfa and data[1] == 0x88:
                # TransBuf: code(H), len(H), idx(H), pad(H), data[TRANS_BLOCK]
                # 实际 IMU 数据从偏移量 8 (头) + 1 (IMU的1字节填充?) = 9 开始
                # 根据 C++ 驱动: IIM42652_FIFO_PACKET_16_ST *imu_stmp = (IIM42652_FIFO_PACKET_16_ST *)(trans->data + 1);
                imu_data_bytes = data[9:33]
                parsed = imu_fmt.unpack(imu_data_bytes)
                
                acc_x_raw, acc_y_raw, acc_z_raw = parsed[1:4]
                gyro_x_raw, gyro_y_raw, gyro_z_raw = parsed[4:7]
                timestamp = parsed[9]
                
                # C++ 驱动的换算系数
                # dat2->gyro_x = imu_stmp->Gyro_X * 4000.0 / 0x10000 * M_PI / 180
                gyro_scale = 4000.0 / 65536.0 * math.pi / 180.0
                acc_scale = 4.0 / 65536.0
                
                with self.lock:
                    self.latest_imu["gyro"] = [
                        gyro_x_raw * gyro_scale,
                        gyro_y_raw * gyro_scale,
                        gyro_z_raw * gyro_scale
                    ]
                    self.latest_imu["acc"] = [
                        acc_x_raw * acc_scale,
                        acc_y_raw * acc_scale,
                        acc_z_raw * acc_scale
                    ]
                    self.latest_imu["timestamp"] = timestamp
                    self.imu_update_count += 1
                    
            # 判断是否为点云包 (1316 字节, 头标志版本为 0 或 1)
            elif data_len == 1316 and (data[0] == 0x00 or data[0] == 0x01):
                # 解析包头 (36 bytes)
                header = pc_header_fmt.unpack(data[:36])
                dot_num = header[3]
                data_type = header[6]
                timestamp = header[10]
                
                # C++ 驱动实际上并不检查 data_type，只要包头是 0 或 1 均认为是点云
                points = []
                offset = 36
                # 遍历解析每个点
                for _ in range(dot_num):
                    pt_data = pc_point_fmt.unpack(data[offset:offset+10])
                    val1, val2, reflectivity, tag = pt_data
                    
                    # 位运算拆解
                    depth = val1 & 0xFFFFFF
                    theta_hi = (val1 >> 24) & 0xFF
                    theta_lo = val2 & 0xFFF
                    phi = (val2 >> 12) & 0xFFFFF
                    
                    theta = (theta_hi << 12) | theta_lo
                    
                    # 坐标换算 (极坐标 -> 笛卡尔坐标)
                    ang = (90000 - theta) * math.pi / 180000.0
                    depth_m = depth / 1000.0
                    
                    r = depth_m * math.cos(ang)
                    z = depth_m * math.sin(ang)
                    
                    ang_phi = phi * math.pi / 180000.0
                    x = math.cos(ang_phi) * r
                    y = math.sin(ang_phi) * r
                    
                    points.append([x, y, z, reflectivity])
                    offset += 10
                    
                with self.lock:
                    self.latest_pointcloud.extend(points)
                    # 防止缓存过大，保留最近的 15000 个点（约一圈数据）
                    if len(self.latest_pointcloud) > 15000:
                        self.latest_pointcloud = self.latest_pointcloud[-15000:]
                    self.pc_timestamp = timestamp

    def get_latest_imu(self):
        """获取最新的 IMU 数据"""
        with self.lock:
            return dict(self.latest_imu)

    def get_latest_pointcloud(self, clear=True):
        """获取点云数据。clear=True 表示获取后清空缓存，类似于提取一帧"""
        with self.lock:
            pc = list(self.latest_pointcloud)
            if clear:
                self.latest_pointcloud.clear()
            return pc

if __name__ == "__main__":
    reader = LidarDirectReader()
    reader.start()
    
    try:
        while True:
            time.sleep(1.0)
            imu = reader.get_latest_imu()
            pc = reader.get_latest_pointcloud(clear=True)
            print(f"IMU: GyroZ={imu['gyro'][2]:.4f} rad/s, AccZ={imu['acc'][2]:.4f} g | IMU Hz: {reader.imu_update_count} | PointCloud size: {len(pc)}")
            reader.imu_update_count = 0
    except KeyboardInterrupt:
        reader.stop()
