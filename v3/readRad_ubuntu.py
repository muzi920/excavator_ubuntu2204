import os
import sys
import time

# 将官方 SDK 路径加入环境变量以便引用
sys.path.append(os.path.join(os.path.dirname(__file__), "WitStandardModbus_WT901C485-main", "Python", "Python-SDK-WT901C485_new"))
import device_model

id_to_name = {
    0x50: "铲斗",
    0x51: "小臂",
    0x52: "大臂",
    0x53: "回转"
}

# 用于记录上一次打印的时间，限制过高的异常打印频率（避免解析错乱时的数据粘连/时间错误）
last_printed_time = {
    0x50: 0,
    0x51: 0,
    0x52: 0,
    0x53: 0
}

def create_callback(port_name):
    def updateData(DeviceModel):
        current_time = time.time()
        for addr, name in id_to_name.items():
            data = DeviceModel.deviceData.get(addr, {})
            # 确保需要的数据都已读到
            if data and "AccX" in data and "AngX" in data:
                acc_x = data.get("AccX", 0)
                acc_y = data.get("AccY", 0)
                ang_x = data.get("AngX", 0)
                ang_y = data.get("AngY", 0)
                
                # 筛选机制：
                # 避免时间错误/粘连数据：限制最高打印频率（例如两次打印间隔 > 0.02秒，即最高50Hz），保证取到的是最新有效数据
                if (current_time - last_printed_time[addr]) > 0.02:
                    # 打印筛选后的数据，去掉端口信息
                    print(f"[{name}] AccX={acc_x:7.3f} AccY={acc_y:7.3f} AngX={ang_x:7.3f}° AngY={ang_y:7.3f}°")
                    
                    # 更新状态
                    last_printed_time[addr] = current_time
                
                # 清除已读数据，确保下一次回调拿到的一定是最新读取进来的数据，而不是历史缓存
                DeviceModel.deviceData[addr].clear()
    return updateData

if __name__ == "__main__":
    # 需要轮询的传感器ID列表
    addrLis = [0x50, 0x51, 0x52, 0x53]
    baud = 230400
    
    # 使用 udev 规则绑定的固定端口名称
    # 这样可以避免全局扫描 (glob) 误将轮询指令发给挖掘机控制器
    ports = [
        "/dev/ttyUSB_Sensor1",
        "/dev/ttyUSB_Sensor2",
        "/dev/ttyUSB_Sensor3",
        "/dev/ttyUSB_Sensor4",
    ]
    
    devices = []
    
    for port in ports:
        try:
            device = device_model.DeviceModel(port, port, baud, addrLis, create_callback(port))
            device.openDevice()
            device.startLoopRead()
            devices.append(device)
            print(f"[{port}] 初始化成功，正在探测传感器...")
        except Exception as e:
            print(f"[{port}] 初始化失败: {e}")
            
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
