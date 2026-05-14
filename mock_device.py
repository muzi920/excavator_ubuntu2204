import can
import time
import threading

# 这是一个模拟设备的脚本（作为服务端/被控端）
# 它可以监听 CAN 总线，接收控制指令并返回模拟的状态数据，
# 帮助你在没有真实硬件的情况下，测试你的 can.py 控制代码是否正确。

class MockDevice:
    def __init__(self, address=1):
        self.address = address
        # 模拟 12 路继电器的状态 (默认全关)
        self.relays = [False] * 12
        # 模拟 2 路模拟量的状态 (默认 0)
        self.analogs = [0, 0]
        # 模拟 48 路输入口的状态 (用于功能码 0x03)
        self.inputs = [False] * 48

    def process_message(self, msg, bus):
        """解析接收到的指令，并返回对应的响应"""
        # 判断是否发给本设备 (标准帧: Bit 10~8 功能码, Bit 7~0 地址)
        if msg.is_extended_id:
            func_code = (msg.arbitration_id >> 8) & 0xFF
            target_addr = msg.arbitration_id & 0xFF
        else:
            func_code = (msg.arbitration_id >> 8) & 0x07
            target_addr = msg.arbitration_id & 0xFF

        # 如果地址不匹配且不是广播地址(假设0是广播，这里简化处理只响应自己地址)
        if target_addr != self.address and target_addr != 0x01: # 之前的获取地址是往0x01发的
            return

        print(f"\n[模拟设备] 收到指令 -> 功能码: {hex(func_code)}, 数据: {msg.data.hex()}")

        # 构造响应消息的基础信息
        reply_id = (func_code << 8) | self.address
        reply_data = [0x00] * 8

        if func_code == 0x01:
            # 写继电器状态
            # 将收到的字节还原为 12 路继电器状态
            if len(msg.data) >= 2:
                for i in range(8):
                    self.relays[i] = bool((msg.data[0] >> i) & 1)
                for i in range(4):
                    self.relays[8+i] = bool((msg.data[1] >> i) & 1)
            
            print(f"  [动作] 更新继电器状态为: {self.relays}")
            # 返回当前状态作为响应 (回显)
            reply_data[0] = msg.data[0]
            reply_data[1] = msg.data[1]
            self._send_reply(bus, reply_id, reply_data)

        elif func_code == 0x02:
            # 读继电器状态
            byte1 = sum((1 << i) for i in range(8) if self.relays[i])
            byte2 = sum((1 << i) for i in range(4) if self.relays[8+i])
            reply_data[0] = byte1
            reply_data[1] = byte2
            print(f"  [动作] 返回继电器状态: {self.relays}")
            self._send_reply(bus, reply_id, reply_data)

        elif func_code == 0x03:
            # 读输入口状态
            # 这里简单返回全 0 或者预设一些触发状态
            reply_data[0] = 0x03 # 假设第 1,2 通道被触发
            print("  [动作] 返回输入口状态")
            self._send_reply(bus, reply_id, reply_data)

        elif func_code == 0x04:
            # 参数设置/读取
            if len(msg.data) > 0:
                sub_func = msg.data[0]
                reply_data[0] = sub_func
                if sub_func == 0xA1: # 读地址
                    reply_data[1] = self.address
                    print(f"  [动作] 返回设备地址: {self.address}")
                elif sub_func == 0xA2: # 读波特率
                    reply_data[1] = 0x07 # 默认 250kbps
                    print("  [动作] 返回波特率: 250kbps")
                # ... 其他参数设置忽略 ...
                self._send_reply(bus, reply_id, reply_data)

        elif func_code == 0x05:
            # 模拟量控制
            if len(msg.data) >= 4:
                self.analogs[0] = (msg.data[0] << 8) | msg.data[1]
                self.analogs[1] = (msg.data[2] << 8) | msg.data[3]
            print(f"  [动作] 更新模拟量状态为: 通道1={self.analogs[0]}, 通道2={self.analogs[1]}")
            # 回显数据
            reply_data[0:4] = msg.data[0:4]
            self._send_reply(bus, reply_id, reply_data)

    def _send_reply(self, bus, arb_id, data):
        """发送响应消息"""
        reply_msg = can.Message(
            arbitration_id=arb_id,
            data=data,
            is_extended_id=False
        )
        time.sleep(0.1) # 模拟处理延迟
        bus.send(reply_msg)
        print(f"  [回复] ID: {hex(arb_id)}, 数据: {bytes(data).hex()}")

def run_mock_device():
    print("启动模拟硬件设备... (按 Ctrl+C 退出)")
    try:
        # 使用虚拟总线监听，必须和控制端保持一致
        bus = can.interface.Bus(bustype='virtual', channel='vcan0', bitrate=250000)
        device = MockDevice(address=1)
        
        while True:
            msg = bus.recv(1)
            if msg is not None:
                # 只处理控制端发来的消息 (简单过滤一下：如果是自己发出去的回显就不处理了)
                # 因为虚拟总线上收发都在一起，我们需要区分指令和回复
                # 通常控制指令是我们要处理的
                # 这里为了简单，只处理数据不全为我们自己发送的回复的情况
                # 真实情况中，上位机和下位机有明确的主从关系
                device.process_message(msg, bus)
                
    except KeyboardInterrupt:
        print("\n模拟设备已关闭。")
    finally:
        if 'bus' in locals():
            bus.shutdown()

if __name__ == "__main__":
    run_mock_device()
