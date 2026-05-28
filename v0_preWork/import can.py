import can
import time
import threading

def receive_messages(bus):
    """接收并解析 CAN 消息的独立线程"""
    print("开始接收 CAN 消息...")
    try:
        while True:
            # 阻塞接收，超时时间 1 秒
            msg = bus.recv(1) 
            if msg is not None:
                parse_message(msg)
    except KeyboardInterrupt:
        pass

def parse_message(msg):
    """
    自定义协议解析函数
    假设我们的协议规则如下：
    - ID 0x100 ~ 0x10F: 传感器数据 (Node ID 为 ID - 0x100)
        - Byte 0: 温度值 (摄氏度)
        - Byte 1: 湿度值 (%)
    - ID 0x200 ~ 0x20F: 控制指令 (Node ID 为 ID - 0x200)
        - Byte 0: 动作 (1=启动, 0=停止)
    """
    if 0x100 <= msg.arbitration_id <= 0x10F:
        node_id = msg.arbitration_id - 0x100
        temp = msg.data[0]
        humidity = msg.data[1]
        print(f"[传感器解析] 节点 {node_id} | 温度: {temp}°C | 湿度: {humidity}%")
        
    elif 0x200 <= msg.arbitration_id <= 0x20F:
        node_id = msg.arbitration_id - 0x200
        action = "启动" if msg.data[0] == 1 else "停止"
        print(f"[控制端解析] 节点 {node_id} | 动作: {action}")
        
    else:
        print(f"[未知协议帧] ID: {hex(msg.arbitration_id)} | 数据: {msg.data.hex()}")

def send_messages(bus):
    """模拟发送 CAN 消息 (封包过程)"""
    # 1. 构造传感器数据 (Node 1, 温度 25度, 湿度 60%)
    msg_sensor = can.Message(
        arbitration_id=0x101, 
        data=[25, 60, 0, 0, 0, 0, 0, 0], 
        is_extended_id=False
    )
    
    # 2. 构造控制指令 (Node 2, 启动)
    msg_control = can.Message(
        arbitration_id=0x202, 
        data=[1, 0, 0, 0, 0, 0, 0, 0], 
        is_extended_id=False
    )

    time.sleep(0.5)
    print("\n---> 发送消息: 传感器节点 1 数据")
    bus.send(msg_sensor)
    
    time.sleep(0.5)
    print("---> 发送消息: 控制节点 2 启动指令")
    bus.send(msg_control)

if __name__ == "__main__":
    # 使用虚拟 CAN 总线进行测试 (跨平台，无需真实硬件)
    try:
        # 在 Windows 等平台上可以使用 'virtual' 接口模拟收发
        bus = can.interface.Bus(bustype='virtual', channel='vcan0', bitrate=500000)
        
        # 启动后台接收线程
        rx_thread = threading.Thread(target=receive_messages, args=(bus,))
        rx_thread.daemon = True
        rx_thread.start()

        # 主线程发送测试消息
        send_messages(bus)
        
        time.sleep(1) # 等待接收线程处理完毕
        bus.shutdown()
        print("\n测试完成，总线已关闭。")
        
    except Exception as e:
        print(f"CAN 总线初始化失败: {e}")