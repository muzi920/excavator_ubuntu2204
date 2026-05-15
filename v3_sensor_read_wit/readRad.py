import os
import sys
import time

# 将官方 SDK 路径加入环境变量以便引用
sys.path.append(os.path.join(os.path.dirname(__file__), "WitStandardModbus_WT901C485-main", "Python", "Python-SDK-WT901C485_new"))
import device_model

def create_callback(sensor_name):
    def updateData(DeviceModel):
        addr = DeviceModel.addrLis[0]
        data = DeviceModel.deviceData.get(addr, {})
        if data:
            roll = data.get("AngX", 0)
            pitch = data.get("AngY", 0)
            print(f"[{sensor_name}] 角度: Roll={roll:7.3f}° Pitch={pitch:7.3f}°")
    return updateData

if __name__ == "__main__":
    addrLis = [0x50]
    baud = 230400
    
    configs = [
        ("大臂", "COM11"),
        ("小臂", "COM8"),
        ("铲斗", "COM7"),
        ("回转", "COM12"),
    ]
    
    devices = []
    
    for name, port in configs:
        try:
            device = device_model.DeviceModel(name, port, baud, addrLis, create_callback(name))
            device.openDevice()
            device.startLoopRead()
            devices.append(device)
            print(f"[{name}] {port} 初始化成功")
        except Exception as e:
            print(f"[{name}] {port} 初始化失败: {e}")
            
    print("开始读取数据... (Ctrl+C 退出)")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("停止读取")
    finally:
        for device in devices:
            device.stopLoopRead()
            
        # 稍微等待一下让读线程退出
        time.sleep(0.5)
        
        for device in devices:
            # 标记关闭，防止线程还在读
            device.isOpen = False
            device.closeDevice()
