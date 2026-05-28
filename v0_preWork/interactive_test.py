import can
import time
import threading

# 从您之前的文件中导入我们写好的控制函数
# 注意：这里假设您的原始文件已重命名为 can_controller.py 
# 或者我们可以直接在这里把需要的函数复制过来，方便单独运行这个测试脚本。

# ================= 复制过来的控制函数 =================
def set_relay(bus, address, relays_status):
    data = [0x00] * 8
    for i in range(min(len(relays_status), 6)):
        data[i] = relays_status[i]
    msg = can.Message(arbitration_id=(0x01 << 8) | address, data=data, is_extended_id=False)
    bus.send(msg)

def set_12_relays(bus, address, relay_states):
    if len(relay_states) != 12:
        print("警告：开关量状态列表长度应为 12")
        return
    byte1 = sum((1 << i) for i in range(8) if relay_states[i])
    byte2 = sum((1 << (i - 8)) for i in range(8, 12) if relay_states[i])
    set_relay(bus, address, [byte1, byte2, 0x00, 0x00, 0x00, 0x00])
    print(f"[控制端] -> 发送 12 路开关量设置指令 (Byte1:{hex(byte1)}, Byte2:{hex(byte2)})")

def set_2_analogs(bus, address, analog1, analog2):
    func_code = 0x05 
    data = [
        (analog1 >> 8) & 0xFF, analog1 & 0xFF,
        (analog2 >> 8) & 0xFF, analog2 & 0xFF,
        0x00, 0x00, 0x00, 0x00
    ]
    msg = can.Message(arbitration_id=(func_code << 8) | address, data=data, is_extended_id=False)
    bus.send(msg)
    print(f"[控制端] -> 发送 2 路模拟量设置指令 (通道1:{analog1}, 通道2:{analog2})")

def get_relay_status(bus, address):
    msg = can.Message(arbitration_id=(0x02 << 8) | address, data=[0x00] * 8, is_extended_id=False)
    bus.send(msg)
    print(f"[控制端] -> 发送读取继电器状态指令")

# ================= 交互式调试流程 =================

def interactive_test(bus, address=1):
    print("\n" + "="*50)
    print("CAN 设备交互式调试工具已启动")
    print("="*50)
    print("可用指令列表:")
    print("  1. 测试单个继电器开关 (探索哪个通道对应哪个实际设备)")
    print("  2. 测试批量继电器开关")
    print("  3. 测试模拟量输出")
    print("  4. 读取当前继电器状态")
    print("  q. 退出调试")
    print("="*50)

    while True:
        try:
            choice = input("\n请输入测试指令编号: ").strip()
            
            if choice == 'q':
                break
                
            elif choice == '1':
                print("\n-- 单通道继电器测试 --")
                channel = int(input("请输入要测试的通道号 (1-12): "))
                if 1 <= channel <= 12:
                    action = input("输入动作 (1=开, 0=关): ")
                    # 构造12路状态
                    states = [False] * 12
                    states[channel - 1] = (action == '1')
                    print(f"正在尝试将通道 {channel} 设置为 {'开启' if action=='1' else '关闭'}...")
                    set_12_relays(bus, address, states)
                else:
                    print("无效的通道号！")

            elif choice == '2':
                print("\n-- 批量继电器测试 --")
                states_str = input("请输入12个0或1 (例如: 110000000001 代表开1,2,12): ")
                if len(states_str) == 12 and all(c in '01' for c in states_str):
                    states = [c == '1' for c in states_str]
                    set_12_relays(bus, address, states)
                else:
                    print("输入格式错误，必须是12位0或1。")

            elif choice == '3':
                print("\n-- 模拟量输出测试 --")
                try:
                    val1 = int(input("请输入通道 1 模拟量值 (0-4095): "))
                    val2 = int(input("请输入通道 2 模拟量值 (0-4095): "))
                    set_2_analogs(bus, address, val1, val2)
                except ValueError:
                    print("请输入有效的数字！")

            elif choice == '4':
                print("\n-- 读取状态 --")
                get_relay_status(bus, address)

            else:
                print("未知指令，请重新输入。")
                
            # 给模拟设备一点时间响应
            time.sleep(0.5)
            
        except Exception as e:
            print(f"发生错误: {e}")

# ================= 接收线程 (用于打印回复) =================
def receive_thread(bus):
    while True:
        msg = bus.recv(1)
        if msg is not None:
            # 简单解析收到的回复
            if not msg.is_extended_id:
                func_code = (msg.arbitration_id >> 8) & 0x07
                if func_code == 0x02: # 读状态的回复
                    print(f"\n[控制端-接收] 继电器状态返回: Byte1={bin(msg.data[0])}, Byte2={bin(msg.data[1])}")

if __name__ == "__main__":
    try:
        bus = can.interface.Bus(bustype='virtual', channel='vcan0', bitrate=115200)
        
        # 启动接收线程
        rx = threading.Thread(target=receive_thread, args=(bus,))
        rx.daemon = True
        rx.start()
        
        # 启动交互式测试
        interactive_test(bus)
        
    finally:
        if 'bus' in locals():
            bus.shutdown()
