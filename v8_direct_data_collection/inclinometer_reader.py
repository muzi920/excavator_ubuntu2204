import os
import sys
import time
import threading

# 引入 WIT 官方底层驱动 (假设相对路径或绝对路径)
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "v3_sensor_read_wit", "WitStandardModbus_WT901C485-main", "Python", "Python-SDK-WT901C485_new"))
import device_model

class InclinometerReader:
    def __init__(self, ports=None, baud=230400):
        if ports is None:
            self.ports = [
                "/dev/ttyUSB_Sensor1",
                "/dev/ttyUSB_Sensor2",
                "/dev/ttyUSB_Sensor3",
                "/dev/ttyUSB_Sensor4"
            ]
        else:
            self.ports = ports
            
        self.baud = baud
        self.addrLis = [0x50, 0x51, 0x52, 0x53]
        self.devices = []
        
        # 用于存储最新的绝对角度数据
        self.latest_angles = {
            0x50: 0.0, # 铲斗
            0x51: 0.0, # 小臂
            0x52: 0.0, # 大臂
            0x53: 0.0  # 回转
        }
        
        # 记录每个传感器初始的绝对角度
        self.init_angles = {
            0x50: None,
            0x51: None,
            0x52: None,
            0x53: None
        }
        
        self.lock = threading.Lock()

    def _create_callback(self, port_name):
        def updateData(DeviceModel):
            for addr in self.addrLis:
                data = DeviceModel.deviceData.get(addr, {})
                if data and "AngX" in data:
                    ang_x = data.get("AngX", 0.0)
                    with self.lock:
                        if self.init_angles[addr] is None:
                            self.init_angles[addr] = ang_x
                        self.latest_angles[addr] = ang_x
                    # 成功读取并处理后，再清理该地址的缓存
                    DeviceModel.deviceData[addr].clear()
        return updateData

    def start(self):
        for port in self.ports:
            try:
                device = device_model.DeviceModel(port, port, self.baud, self.addrLis, self._create_callback(port))
                device.openDevice()
                device.startLoopRead()
                self.devices.append(device)
                print(f"[InclinometerReader] 成功连接并启动轮询: {port}")
            except Exception as e:
                print(f"[InclinometerReader] 初始化失败 {port}: {e}")

    def get_relative_angles(self):
        """
        获取大臂、小臂、铲斗相对夹角
        返回: (boom_rel, arm_rel, bucket_rel)
        """
        with self.lock:
            # 只有当所有传感器都获取到初始值时，才认为数据有效
            if None in self.init_angles.values():
                return None, None, None
                
            # 相对角度 = 子部件角度 - 父部件角度
            bucket_rel = self.latest_angles[0x50] - self.latest_angles[0x51]
            arm_rel = self.latest_angles[0x51] - self.latest_angles[0x52]
            boom_rel = self.latest_angles[0x52] - self.latest_angles[0x53]
            
            return boom_rel, arm_rel, bucket_rel

    def stop(self):
        print("[InclinometerReader] 正在关闭传感器...")
        for device in self.devices:
            device.stopLoopRead()
        time.sleep(0.5)
        for device in self.devices:
            device.isOpen = False
            device.closeDevice()
        print("[InclinometerReader] 已完全关闭.")
