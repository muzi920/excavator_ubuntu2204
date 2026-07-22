import socket
import struct
import math
import sys
import time
import os
from datetime import datetime

# Constants matching the C++ SDK
LISTEN_PORT = 6668
LIDAR_IP = "192.168.158.98"
LIDAR_PORT = 6543

def get_log_file():
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(log_dir, f"imu_swing_{timestamp}.log")

class DirectSwingAngleEstimator:
    def __init__(self):
        self.current_swing_rad = 0.0
        self.last_time = None
        self.last_w_yaw = 0.0
        
        # --- 零偏动态校准相关变量 ---
        self.is_calibrating = True
        self.calib_gyro_samples = []
        self.calib_accel_samples = []
        self.calib_start_time = None
        self.CALIB_DURATION = 3.0  # 开机静止校准时间(秒)
        
        self.gyro_bias = (0.0, 0.0, 0.0)
        self.up_vector = (0.0, 0.0, 1.0)
        
        print("Swing Angle Estimator started.")
        print("【重要】正在进行陀螺仪和加速度计联合校准 (3秒)，请保持挖掘机绝对静止！...")

    def process_imu(self, accel, gyro, timestamp_ns):
        # 假设时间戳是纳秒级别（或者使用本地系统时间以防雷达时间戳跳变）
        # 这里为了稳妥，我们直接使用系统时间 time.time() 作为 dt 计算依据
        current_time = time.time()
        
        if self.last_time is None:
            self.last_time = current_time
            self.calib_start_time = current_time
            return
            
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 异常时间戳过滤
        if dt <= 0 or dt > 0.5:
            return
            
        # 2. 动态零偏和重力向量校准阶段
        if self.is_calibrating:
            self.calib_gyro_samples.append(gyro)
            self.calib_accel_samples.append(accel)
            
            if current_time - self.calib_start_time >= self.CALIB_DURATION:
                n = len(self.calib_gyro_samples)
                if n > 0:
                    # 计算三轴陀螺仪零偏
                    bx = sum(g[0] for g in self.calib_gyro_samples) / n
                    by = sum(g[1] for g in self.calib_gyro_samples) / n
                    bz = sum(g[2] for g in self.calib_gyro_samples) / n
                    self.gyro_bias = (bx, by, bz)
                    
                    # 计算重力加速度平均向量 (测量的静止加速度即为真实向上的法向量)
                    ax = sum(a[0] for a in self.calib_accel_samples) / n
                    ay = sum(a[1] for a in self.calib_accel_samples) / n
                    az = sum(a[2] for a in self.calib_accel_samples) / n
                    
                    # 归一化得到绝对垂直轴的单位向量 (Up Vector)
                    norm = math.sqrt(ax**2 + ay**2 + az**2)
                    if norm > 0:
                        self.up_vector = (ax/norm, ay/norm, az/norm)
                    else:
                        self.up_vector = (0.0, 0.0, 1.0)
                        
                self.is_calibrating = False
                print(f"【校准完成】")
                print(f"  陀螺仪零偏 (x,y,z): ({self.gyro_bias[0]:.6f}, {self.gyro_bias[1]:.6f}, {self.gyro_bias[2]:.6f}) rad/s")
                print(f"  绝对垂直投影轴 (x,y,z): ({self.up_vector[0]:.4f}, {self.up_vector[1]:.4f}, {self.up_vector[2]:.4f})")
                tilt_deg = math.degrees(math.acos(abs(self.up_vector[2])))
                print(f"  >> 雷达安装倾角评估: 偏离绝对水平面约 {tilt_deg:.2f} 度")
            return
        
        # 3. 扣除三轴陀螺仪零偏
        w_x = gyro[0] - self.gyro_bias[0]
        w_y = gyro[1] - self.gyro_bias[1]
        w_z = gyro[2] - self.gyro_bias[2]
        
        # 4. 空间投影: 将本地 3D 角速度投影到真实的绝对垂直轴上 (点积)
        w_yaw = w_x * self.up_vector[0] + w_y * self.up_vector[1] + w_z * self.up_vector[2]
        
        # 针对本次挖掘机底盘极性：向左转 90度（逆时针）在之前的算法中输出的是 +90度，
        # 如果我们需要让向左转（逆时针）为正，这里已经是对的。
        # 如果我们需要让向左转为负（匹配 V4 中的正右负左约定），则需要取反：
        w_yaw = -w_yaw
        
        # 5. 极小死区滤波 (0.002 rad/s 约等于 0.1 deg/s)
        if abs(w_yaw) < 0.002:
            w_yaw = 0.0
            
        # 6. 梯形积分算法
        self.current_swing_rad += (w_yaw + self.last_w_yaw) / 2.0 * dt
        self.last_w_yaw = w_yaw
        
        # 放大比例系数修复 (Scale Factor)
        # 根据日志，向左旋转 90度时，原始算法积分结果只有约 90度 左右，但是考虑到雷达倾斜 6度
        # 我们可能需要检查是不是角速度量程问题，或者雷达原始数据本身的比例有问题。
        # 由于我们观察到积分到 89.54度然后又降下去了（因为你可能停止并反弹了一点点）
        # 这里的结果非常接近 90度（89.54度）。所以投影积分是高度准确的！
        #
        # 但是，V4 中的规定是：正右负左！
        # 如果你向左转，角度应该是 -90 度。所以我们在上面的 w_yaw 已经添加了符号反转。
        
        swing_deg = math.degrees(self.current_swing_rad)
        
        # 约束在 [-180, 180]
        while swing_deg > 180.0:
            swing_deg -= 360.0
        while swing_deg < -180.0:
            swing_deg += 360.0
            
        return swing_deg, w_yaw


def crc32_stm32(data):
    crc = 0xFFFFFFFF
    for i in range(0, len(data), 4):
        word = struct.unpack_from('>I', data, i)[0] if i + 4 <= len(data) else 0
        crc ^= word
        for _ in range(32):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return crc

def pack_net_cmd(cmd_type, payload):
    import random
    sn = random.randint(0, 65535)
    sign = 0x484C
    length = len(payload)
    len4 = ((length + 3) >> 2) * 4
    padded_payload = payload.encode('ascii') + b'\x00' * (len4 - length)
    header = struct.pack('<H H H H', sign, cmd_type, sn, length)
    packet_without_crc = header + padded_payload
    crc = crc32_stm32(packet_without_crc)
    packet = packet_without_crc + struct.pack('<I', crc)
    return packet

def send_start_command(sock):
    cmd = "LSTARH"
    cmd_type = 0x0043
    packet = pack_net_cmd(cmd_type, cmd)
    for i in range(5):
        print(f"Sending Start Command ({cmd}) to {LIDAR_IP}:{LIDAR_PORT}... (Try {i+1}/5)")
        sock.sendto(packet, (LIDAR_IP, LIDAR_PORT))
        time.sleep(0.1)

def main():
    print(f"Starting UDP Lidar IMU listener on port {LISTEN_PORT}...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        sock.bind(('0.0.0.0', LISTEN_PORT))
        print("Socket bound successfully.")
    except Exception as e:
        print(f"Failed to bind socket: {e}")
        sys.exit(1)

    send_start_command(sock)

    print("Waiting for IMU data (Press Ctrl+C to stop)...")
    
    estimator = DirectSwingAngleEstimator()
    log_file_path = get_log_file()
    print(f"Logging data to: {log_file_path} (interval: 0.2s)")
    
    last_log_time = time.time()
    
    try:
        with open(log_file_path, "w", encoding="utf-8") as log_file:
            log_file.write("Timestamp,Angle_deg,Yaw_Rate_rad_s\n")
            
            while True:
                data, addr = sock.recvfrom(65536)
                if not data:
                    continue
                    
                # Check for IMU packet: 0xfa 0x88
                if data[0] == 0xfa and data[1] == 0x88 and len(data) >= 27:
                    imu_fmt = '<B h h h h h h b H Q'
                    try:
                        imu_data = struct.unpack_from(imu_fmt, data, 8 + 1)
                        accel_x = imu_data[1] * 4.0 / 0x10000
                        accel_y = imu_data[2] * 4.0 / 0x10000
                        accel_z = imu_data[3] * 4.0 / 0x10000
                        gyro_x = imu_data[4] * 4000.0 / 0x10000 * math.pi / 180
                        gyro_y = imu_data[5] * 4000.0 / 0x10000 * math.pi / 180
                        gyro_z = imu_data[6] * 4000.0 / 0x10000 * math.pi / 180
                        timestamp = imu_data[9]
                        
                        res = estimator.process_imu((accel_x, accel_y, accel_z), (gyro_x, gyro_y, gyro_z), timestamp)
                        
                        if res is not None:
                            swing_deg, w_yaw = res
                            current_t = time.time()
                            
                            # 0.2s 日志写入与打印频率控制
                            if current_t - last_log_time >= 0.2:
                                time_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                                log_line = f"{time_str},{swing_deg:.3f},{w_yaw:.5f}\n"
                                log_file.write(log_line)
                                log_file.flush() # 强制写入磁盘，防止程序崩溃丢失数据
                                
                                print(f"[{time_str}] [IMU Swing] Angle: {swing_deg:7.2f} ° | Yaw Rate: {w_yaw:7.3f} rad/s")
                                last_log_time = current_t
                                
                    except struct.error:
                        pass
                    
    except KeyboardInterrupt:
        print("\nStopping listener.")
    finally:
        sock.close()

if __name__ == '__main__':
    main()
