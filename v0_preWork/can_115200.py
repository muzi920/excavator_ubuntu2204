import serial
import time
import struct
import threading

class ZSCanConverter:
    """
    中盛科技 ZS-USB-CAN 转换器串口通信协议实现
    """
    def __init__(self, port="COM3", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None

    def open(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            print(f"[INFO] 串口已打开: {self.port} at {self.baudrate} bps")
            return True
        except Exception as e:
            print(f"[ERROR] 打开串口失败: {e}")
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[INFO] 串口关闭")

    def _send_command(self, id_bytes, data_bytes, func_code):
        if len(id_bytes) != 4 or len(data_bytes) != 8:
            print("[ERROR] 数据长度错误")
            return
        
        # 协议规定：报文总长度 13 字节
        frame = bytearray(13)
        frame[0:4] = id_bytes      # 字节 1-4: ID区
        frame[4:12] = data_bytes   # 字节 5-12: 数据区
        frame[12] = func_code      # 字节 13: 功能码
        
        if self.ser and self.ser.is_open:
            self.ser.write(frame)
            print(f"[TX] 发送: {frame.hex(' ').upper()}")
        else:
            print("[ERROR] 串口未打开")

    def send_can_frame(self, can_id, data, is_extended=False):
        """
        发送CAN报文 (透传, 0x00)
        """
        if len(data) > 8:
            print("[ERROR] CAN数据不能超过8字节")
            return
            
        # 补齐8字节数据
        data_bytes = bytearray(data)
        data_bytes.extend(b'\x00' * (8 - len(data)))
        
        if is_extended:
            # 扩展帧：原始CAN报文中的帧ID整体左移3位，后将第4个字节第二个比特位置1
            shifted_id = (can_id << 3) & 0xFFFFFFFF
            id_bytes = bytearray(struct.pack('>I', shifted_id))
            # 第二个比特位置1，通常指 bit1 (0x02)。若实际设备不响应，可尝试改为 bit2 (0x04)
            id_bytes[3] |= 0x02  
        else:
            # 标准帧：原始CAN报文中的帧ID整体左移21位
            shifted_id = (can_id << 21) & 0xFFFFFFFF
            id_bytes = bytearray(struct.pack('>I', shifted_id))
            
        self._send_command(id_bytes, data_bytes, 0x00)

    def set_serial_baudrate(self, baud_code):
        """
        设置串口波特率 (0x01)
        baud_code 见手册表2.1 (如 0x07 为 115200)
        """
        # 参数设置/读取指令中，ID固定为 'Z' 'S' 'K' 'J' -> 0x5A 0x53 0x4B 0x4A
        id_bytes = bytes([0x5A, 0x53, 0x4B, 0x4A])
        data_bytes = bytearray(8)
        data_bytes[0] = baud_code
        self._send_command(id_bytes, data_bytes, 0x01)

    def read_serial_baudrate(self):
        """读取串口波特率 (0x02)"""
        id_bytes = bytes([0x5A, 0x53, 0x4B, 0x4A])
        data_bytes = bytearray(8)
        self._send_command(id_bytes, data_bytes, 0x02)

    def set_can_baudrate(self, baud_code):
        """
        设置CAN波特率 (0x03)
        baud_code 见手册表2.2 (如 0x09 为 500kbps)
        """
        id_bytes = bytes([0x5A, 0x53, 0x4B, 0x4A])
        data_bytes = bytearray(8)
        data_bytes[0] = baud_code
        self._send_command(id_bytes, data_bytes, 0x03)

    def read_can_baudrate(self):
        """读取CAN波特率 (0x04)"""
        id_bytes = bytes([0x5A, 0x53, 0x4B, 0x4A])
        data_bytes = bytearray(8)
        self._send_command(id_bytes, data_bytes, 0x04)

    def read_data(self):
        """读取返回数据，解析CAN标准帧并打印"""
        if self.ser and self.ser.in_waiting >= 13:
            # 读取13字节的一帧数据
            data = self.ser.read(13)
            
            # 协议规定：报文总长度 13 字节
            # 解析功能码
            func_code = data[12]
            
            if func_code == 0x00:
                # 透传数据
                id_bytes = data[0:4]
                can_id_shifted = struct.unpack('>I', id_bytes)[0]
                
                # 判断是否为扩展帧（检查第4字节的第2位是否为1）
                is_ext = (id_bytes[3] & 0x02) != 0
                
                if not is_ext:
                    # 标准帧: 原始CAN报文中的帧ID整体左移21位
                    can_id = (can_id_shifted >> 21) & 0x7FF
                    can_data = data[4:12]
                    print(f"[RX] 标准帧 | ID: 0x{can_id:03X} | 数据: {can_data.hex(' ').upper()}")
                else:
                    # 扩展帧: 原始CAN报文中的帧ID整体左移3位
                    can_id = (can_id_shifted >> 3) & 0x1FFFFFFF
                    can_data = data[4:12]
                    print(f"[RX] 扩展帧 | ID: 0x{can_id:08X} | 数据: {can_data.hex(' ').upper()}")
            else:
                print(f"[RX] 响应帧 | 功能码: 0x{func_code:02X} | 原始数据: {data.hex(' ').upper()}")
            
            return data
        return None

def input_thread(converter):
    """
    独立线程：用于接收用户的键盘输入并发送对应的CAN指令。
    输入格式示例: 00 00 01 03 04 00 00 00 00 00 00 00
    """
    print("\n[INFO] 控制台输入已启动。")
    print("您可以输入 12 字节的十六进制指令 (以空格分隔)，例如:")
    print("00 00 01 03 04 00 00 00 00 00 00 00")
    print("系统将自动发送该指令，并在 1 秒后自动发送对应的停止指令 (将第 5 字节设为 00)\n")
    
    while True:
        try:
            user_input = input()
            if not user_input.strip():
                continue
                
            parts = user_input.strip().split()
            if len(parts) != 12:
                print(f"[ERROR] 输入格式错误，需要 12 个字节，当前输入了 {len(parts)} 个。")
                continue
                
            # 解析输入的 12 字节
            bytes_list = [int(x, 16) for x in parts]
            
            # 提取 CAN ID (前 4 字节) 和 数据 (后 8 字节)
            can_id = (bytes_list[0] << 24) | (bytes_list[1] << 16) | (bytes_list[2] << 8) | bytes_list[3]
            data = bytes_list[4:12]
            
            print(f"\n[TX] 发送用户运动指令 -> ID: 0x{can_id:04X}, 数据: {[f'0x{b:02X}' for b in data]}")
            converter.send_can_frame(can_id=can_id, data=data, is_extended=False)
            
            # 等待 1 秒
            time.sleep(3)
            
            # 自动生成停止指令 (将所有数据字节清零，实现真正的全0停止指令)
            stop_data = [0x00] * 8
            
            print(f"[TX] 发送自动停止指令 -> ID: 0x{can_id:04X}, 数据: {[f'0x{b:02X}' for b in stop_data]}")
            converter.send_can_frame(can_id=can_id, data=stop_data, is_extended=False)
            
        except Exception as e:
            print(f"[ERROR] 解析输入失败: {e}")

def run_sequence_commands(converter):
    """
    顺序执行指令序列。
    您可以将想要执行的指令（12字节十六进制字符串）添加到 commands 列表中。
    每条指令执行 1 秒后自动停止（发送对应全0指令）。
    """

    commands_wheel = [
        "00 00 01 04 13 88 13 88 00 00 00 00",
        "00 00 01 04 00 00 00 00 00 00 00 00"
    ]

    wheel_parr1 = commands_wheel[0].strip().split()
    wheel_parr2 = commands_wheel[1].strip().split()
    bytes_list = [int(x, 16) for x in wheel_parr1]
    can_id = (bytes_list[0] << 24) | (bytes_list[1] << 16) | (bytes_list[2] << 8) | bytes_list[3]
    data_wheel = bytes_list[4:12]
    
    print(f"\n[序列执行] 发送运动指令 -> ID: 0x{can_id:04X}, 数据: {[f'0x{b:02X}' for b in data_wheel]}")
    converter.send_can_frame(can_id=can_id, data=data_wheel, is_extended=False)
            
    command_boom = [
        "00 00 01 02 01 00 00 00 00 00 00 00",
        "00 00 01 02 02 00 00 00 00 00 00 00"
    ]

    for boom_control in command_boom:
        if not boom_control.strip() or boom_control.startswith("#"):
            continue

        try:
            parts = boom_control.strip().split()
            if len(parts) != 12:
                print(f"[ERROR] 指令格式错误，跳过: {cmd_str}")
                continue
                
            bytes_list = [int(x, 16) for x in parts]
            can_id = (bytes_list[0] << 24) | (bytes_list[1] << 16) | (bytes_list[2] << 8) | bytes_list[3]
            data = bytes_list[4:12]
            
            print(f"\n[序列执行] 发送运动指令 -> ID: 0x{can_id:04X}, 数据: {[f'0x{b:02X}' for b in data]}")
            converter.send_can_frame(can_id=can_id, data=data, is_extended=False)
            
            # 维持运行 1 秒
            time.sleep(0.5)
            
            # 发送停止指令
            stop_data = [0x00] * 8
            print(f"[序列执行] 发送停止指令 -> ID: 0x{can_id:04X}, 数据: {[f'0x{b:02X}' for b in stop_data]}")
            converter.send_can_frame(can_id=can_id, data=stop_data, is_extended=False)
            
            # 停止后强制停顿 0.5 秒，再执行下一条指令
            time.sleep(1)
            
        except Exception as e:
            print(f"[ERROR] 执行序列指令异常: {e}, 指令: {cmd_str}")

    commands = [
        # 请在这里添加您的指令，例如:
        # "00 00 01 04 00 00 13 88 00 00 00 00",
        # "00 00 01 04 00 00 00 00 03 E8 00 00",

        "00 00 01 03 01 00 00 00 00 00 00 00",
        "00 00 01 03 02 00 00 00 00 00 00 00",
        "00 00 01 03 04 00 00 00 00 00 00 00",
        "00 00 01 03 08 00 00 00 00 00 00 00",
        "00 00 01 03 06 00 00 00 00 00 00 00",
        "00 00 01 03 09 00 00 00 00 00 00 00",
        "00 00 01 03 0A 00 00 00 00 00 00 00",
        "00 00 01 03 05 00 00 00 00 00 00 00",

        "00 00 01 04 00 00 00 00 00 00 00 00"

        "00 00 01 01 01 00 00 00 00 00 00 00",
        "00 00 01 01 02 00 00 00 00 00 00 00",

        "00 00 01 02 04 00 00 00 00 00 00 00",
        "00 00 01 02 08 00 00 00 00 00 00 00",
        
        "00 00 01 01 04 00 00 00 00 00 00 00",
        "00 00 01 01 08 00 00 00 00 00 00 00",

    ]

    for cmd_str in commands:
        if not cmd_str.strip() or cmd_str.startswith("#"):
            continue
            
        try:
            parts = cmd_str.strip().split()
            if len(parts) != 12:
                print(f"[ERROR] 指令格式错误，跳过: {cmd_str}")
                continue
                
            bytes_list = [int(x, 16) for x in parts]
            can_id = (bytes_list[0] << 24) | (bytes_list[1] << 16) | (bytes_list[2] << 8) | bytes_list[3]
            data = bytes_list[4:12]
            
            print(f"\n[序列执行] 发送运动指令 -> ID: 0x{can_id:04X}, 数据: {[f'0x{b:02X}' for b in data]}")
            converter.send_can_frame(can_id=can_id, data=data, is_extended=False)
            
            # 维持运行 1 秒
            time.sleep(1.0)
            
            # 发送停止指令
            stop_data = [0x00] * 8
            print(f"[序列执行] 发送停止指令 -> ID: 0x{can_id:04X}, 数据: {[f'0x{b:02X}' for b in stop_data]}")
            converter.send_can_frame(can_id=can_id, data=stop_data, is_extended=False)
            
            # 停止后强制停顿 0.5 秒，再执行下一条指令
            time.sleep(0.5)
            
        except Exception as e:
            print(f"[ERROR] 执行序列指令异常: {e}, 指令: {cmd_str}")

def main():
    # 采用的串口是COM3，波特率115200
    converter = ZSCanConverter(port="COM3", baudrate=115200)
    if not converter.open():
        return

    try:
        print("\n--- 发送建立连接指令 ---")
        # 建立连接的指令只需要发布一次，ID: 0x0303
        converter.send_can_frame(can_id=0x0303, data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended=False)
        time.sleep(0.5)

        # 启动独立输入线程 (手动交互)
        t_input = threading.Thread(target=input_thread, args=(converter,), daemon=True)
        t_input.start()

        # 启动自动序列执行线程 (自动执行指令列表)
        t_sequence = threading.Thread(target=run_sequence_commands, args=(converter,), daemon=True)
        t_sequence.start()

        print("\n--- 监听总线数据 (仅读取继电器数据，不发送控制指令) ---")
        while True:
            # 持续读取串口返回的数据
            converter.read_data()
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[INFO] 用户中断")
    finally:
        converter.close()

if __name__ == "__main__":
    main()