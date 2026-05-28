import can
import time
import threading

# ⚠️ 警告：本文件名为 can.py。如果在同一目录下运行此脚本，
# Python 的 `import can` 会尝试导入本文件自身，而不是 `python-can` 库，
# 从而导致 "partially initialized module 'can' has no attribute 'Message'" 错误。
# 建议在实际运行前将本文件重命名为其他名称（例如 `can_test.py` 或 `main.py`）。

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
    根据《数字量输入输出系列使用手册(CAN版)》解析协议：
    标准帧 ID = 功能码(Bit 10~8) + 地址码(Bit 7~0)
    扩展帧 ID = 任意(Bit 28~24) + 0xAA(Bit 23~16) + 功能码(Bit 15~8) + 地址码(Bit 7~0)
    """
    if msg.is_extended_id:
        func_code = (msg.arbitration_id >> 8) & 0xFF
        address = msg.arbitration_id & 0xFF
    else:
        func_code = (msg.arbitration_id >> 8) & 0x07
        address = msg.arbitration_id & 0xFF

    if func_code == 0x01:
        print(f"[解析-写继电器] 地址: {address} | 返回状态: {msg.data.hex()}")
        
    elif func_code == 0x02:
        print(f"[解析-读继电器] 地址: {address}")
        for i in range(min(6, len(msg.data))):
            print(f"  通道 {i*8 + 1:02d}~{i*8 + 8:02d} 状态: {bin(msg.data[i])}")
            
    elif func_code == 0x03:
        print(f"[解析-读输入口] 地址: {address}")
        for i in range(min(6, len(msg.data))):
            print(f"  输入 {i*8 + 1:02d}~{i*8 + 8:02d} 状态: {bin(msg.data[i])}")
            
    elif func_code == 0x04:
        if len(msg.data) > 0:
            sub_func = msg.data[0]
            if sub_func == 0xA1:
                print(f"[解析-参数设置] 读取到地址码: {msg.data[1]}")
            elif sub_func == 0xA2:
                print(f"[解析-参数设置] 读取到波特率码: {msg.data[1]}")
            elif sub_func == 0xB1:
                print(f"[解析-参数设置] 写入地址码成功: {msg.data[1]}")
            elif sub_func == 0xB2:
                print(f"[解析-参数设置] 写入波特率码成功: {msg.data[1]}")
            elif sub_func == 0xB3:
                interval = (msg.data[1] << 8) | msg.data[2]
                print(f"[解析-参数设置] 写入主动上传间隔成功: {interval} ms")
    elif func_code == 0x05:
        # 假设 0x05 是模拟量操作的功能码
        if len(msg.data) >= 4:
            analog1 = (msg.data[0] << 8) | msg.data[1]
            analog2 = (msg.data[2] << 8) | msg.data[3]
            print(f"[解析-模拟量] 地址: {address} | 通道1: {analog1}, 通道2: {analog2}")
    else:
        print(f"[未知协议帧] ID: {hex(msg.arbitration_id)} | 数据: {msg.data.hex()}")

# ================= 新增：封装好的发送函数 =================

def set_12_relays(bus, address, relay_states):
    """
    控制 12 个开关量 (继电器)
    :param bus: CAN 总线对象
    :param address: 设备地址 (1-255)
    :param relay_states: 一个包含 12 个布尔值或 0/1 的列表，对应 1-12 路开关状态
    """
    if len(relay_states) != 12:
        print("警告：开关量状态列表长度应为 12")
        return
    
    # 将 12 个状态打包成 2 个字节
    byte1 = 0 # 控制 1~8 路
    for i in range(8):
        if relay_states[i]:
            byte1 |= (1 << i)
            
    byte2 = 0 # 控制 9~12 路
    for i in range(8, 12):
        if relay_states[i]:
            byte2 |= (1 << (i - 8))
            
    # 调用原有的 set_relay 发送指令 (0x01 功能码)
    set_relay(bus, address, [byte1, byte2, 0x00, 0x00, 0x00, 0x00])


def set_2_analogs(bus, address, analog1, analog2):
    """
    控制 2 个模拟量输出 (假设协议：每个通道占 2 字节，功能码 0x05)
    注：由于提供的 PDF 手册仅包含数字量，此处为您提供一个最常见的通用 CAN 模拟量输出协议框架。
    请根据您的《模拟量输出系列使用手册(CAN版)》对功能码和数据格式进行调整。
    :param analog1: 通道 1 模拟量值 (例如 0~4095 对应 0-10V)
    :param analog2: 通道 2 模拟量值
    """
    # 假设功能码为 0x05 专门用于写模拟量
    func_code = 0x05 
    
    # 假设数据格式：通道1高字节，通道1低字节，通道2高字节，通道2低字节
    data = [
        (analog1 >> 8) & 0xFF, analog1 & 0xFF,
        (analog2 >> 8) & 0xFF, analog2 & 0xFF,
        0x00, 0x00, 0x00, 0x00
    ]
    
    msg = can.Message(
        arbitration_id=(func_code << 8) | address,
        data=data,
        is_extended_id=False
    )
    print(f"发送: 设置地址 {address} 模拟量 -> 通道1:{analog1}, 通道2:{analog2}")
    bus.send(msg)


def set_relay(bus, address, relays_status):
    """
    功能码 0x01: 设置继电器状态
    :param bus: CAN 总线对象
    :param address: 设备地址 (1-255)
    :param relays_status: 继电器状态列表，最多6个字节，每个字节控制8路。
                          例如 [0xFF] 表示1-8路全开，[0x0F, 0x00] 表示1-4路开。
    """
    data = [0x00] * 8
    for i in range(min(len(relays_status), 6)):
        data[i] = relays_status[i]
        
    msg = can.Message(
        arbitration_id=(0x01 << 8) | address,
        data=data,
        is_extended_id=False
    )
    print(f"发送: 设置地址 {address} 继电器状态 -> {data}")
    bus.send(msg)

def get_relay_status(bus, address):
    """
    功能码 0x02: 读继电器状态
    """
    msg = can.Message(
        arbitration_id=(0x02 << 8) | address,
        data=[0x00] * 8,
        is_extended_id=False
    )
    print(f"发送: 读取地址 {address} 继电器状态")
    bus.send(msg)

def get_input_status(bus, address):
    """
    功能码 0x03: 读输入口状态
    """
    msg = can.Message(
        arbitration_id=(0x03 << 8) | address,
        data=[0x00] * 8,
        is_extended_id=False
    )
    print(f"发送: 读取地址 {address} 输入口状态")
    bus.send(msg)

def get_device_address(bus):
    """
    功能码 0x04: 读设备地址 (子功能码 0xA1)
    """
    # 广播或发送给任意地址
    msg = can.Message(
        arbitration_id=(0x04 << 8) | 0x01, # 使用默认地址1发读指令
        data=[0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        is_extended_id=False
    )
    print("发送: 读取设备地址")
    bus.send(msg)

def set_device_address(bus, current_address, new_address):
    """
    功能码 0x04: 设置设备地址 (子功能码 0xB1)
    """
    msg = can.Message(
        arbitration_id=(0x04 << 8) | current_address,
        data=[0xB1, new_address, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        is_extended_id=False
    )
    print(f"发送: 将设备地址 {current_address} 修改为 {new_address}")
    bus.send(msg)

def send_messages(bus):
    """测试用例"""
    address = 1 # 默认设备地址为1
    
    time.sleep(0.5)
    # 1. 批量控制 12 个开关量 (例如：打开第1、2、12路，其余关闭)
    # 列表对应第 1~12 路状态，True 为开，False 为关
    states = [True, True, False, False, False, False, False, False, False, False, False, True]
    print("\n---> 发送消息: 设置 12 个开关量状态")
    set_12_relays(bus, address, states)

    time.sleep(0.5)
    # 2. 控制 2 个模拟量 (例如通道1输出1000，通道2输出2000)
    print("\n---> 发送消息: 设置 2 个模拟量")
    set_2_analogs(bus, address, 1000, 2000)
    
    time.sleep(0.5)
    # 3. 读取继电器状态
    print("\n---> 发送消息: 读取开关量(继电器)状态")
    get_relay_status(bus, address)

    time.sleep(0.5)
    # 4. 读取设备地址
    print("\n---> 发送消息: 读取设备地址")
    get_device_address(bus)


if __name__ == "__main__":
    # 使用虚拟 CAN 总线进行测试
    try:
        # 在 Windows 等平台上可以使用 'virtual' 接口模拟收发
        # 注意：在测试真实设备时，需修改 bustype 和 channel
        bus = can.interface.Bus(bustype='virtual', channel='vcan0', bitrate=250000)
        
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
