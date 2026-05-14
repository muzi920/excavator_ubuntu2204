import struct
import time
from typing import Dict, Iterable, List, Optional

import serial


class ZSCanTransport:
    """
    中盛科技 ZS-USB-CAN 转换器串口通信传输层协议封装。
    负责建立串口连接，并将高层数据打包成 13 字节协议帧发送给 CAN 转换器。
    
    13字节协议格式：
    - [0:4]   : 4 字节的 CAN ID 区 (经过位移编码)
    - [4:12]  : 8 字节的数据区 (不足 8 字节补 0)
    - [12]    : 1 字节的功能码 (默认为 0x00 透传)
    """

    def __init__(self, port: str = "COM5", baudrate: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None

    def open(self) -> bool:
        """打开串口并初始化配置"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
            )
            print(f"[INFO] 串口已打开: {self.port} @ {self.baudrate} bps")
            return True
        except Exception as exc:
            print(f"[ERROR] 无法打开串口: {exc}")
            self.ser = None
            return False

    def close(self) -> None:
        """安全关闭串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[INFO] 串口已关闭")

    def handshake(self, delay_s: float = 0.5) -> None:
        """
        发送握手帧。
        根据原有脚本逻辑，启动时需要向 0x0303 发送一帧全0数据，以建立正常通信。
        """
        self.send_can_frame(0x0303, [0x00] * 8)
        time.sleep(delay_s)

    @staticmethod
    def _encode_can_id(can_id: int, is_extended: bool) -> bytes:
        """
        根据中盛科技手册规则，对原始 CAN ID 进行移位编码：
        - 扩展帧：CAN ID 左移 3 位，并将第 4 字节的第 2 位（bit1, 0x02）置 1。
        - 标准帧：CAN ID 左移 21 位。
        """
        if is_extended:
            shifted = (can_id << 3) & 0xFFFFFFFF
            id_bytes = bytearray(struct.pack(">I", shifted))
            id_bytes[3] |= 0x02
            return bytes(id_bytes)

        shifted = (can_id << 21) & 0xFFFFFFFF
        return struct.pack(">I", shifted)

    def send_can_frame(self, can_id: int, data: Iterable[int], is_extended: bool = False, func_code: int = 0x00) -> bytes:
        """
        发送标准 CAN 数据帧。
        :param can_id: 目标 CAN ID
        :param data: 发送的数据序列（最大 8 字节）
        :param is_extended: 是否为扩展帧
        :param func_code: 功能码，0x00 代表 CAN 透传
        :return: 发送的 13 字节完整报文
        """
        data_list = list(data)
        if len(data_list) > 8:
            raise ValueError("CAN 数据长度不能超过 8 字节")

        # 补齐 8 字节数据区
        payload = bytearray(data_list)
        payload.extend(b"\x00" * (8 - len(payload)))

        # 组装 13 字节的帧
        frame = bytearray(13)
        frame[0:4] = self._encode_can_id(can_id, is_extended)
        frame[4:12] = payload
        frame[12] = func_code

        if not self.ser or not self.ser.is_open:
            raise RuntimeError("串口未打开，无法发送数据")

        self.ser.write(frame)
        print(f"[TX] ID=0x{can_id:04X} DATA={payload.hex(' ').upper()} FRAME={frame.hex(' ').upper()}")
        return bytes(frame)

    def send_raw_12byte_command(self, hex_command: str, is_extended: bool = False) -> bytes:
        """
        兼容原始文本形式的 12 字节控制指令发送（例如 control.txt 中的行）。
        格式例如："00 00 01 03 06 00 00 00 00 00 00 00"
        """
        # 清理注释 (# 和 //)
        clean = hex_command.split("#", 1)[0]
        clean = clean.split("//", 1)[0].strip()
        parts = clean.split()
        if len(parts) != 12:
            raise ValueError(f"预期输入 12 字节的十六进制字符串，但获得了 {len(parts)} 个: {hex_command}")

        values = [int(part, 16) for part in parts]
        # 前 4 个字节直接组合为 can_id
        can_id = (values[0] << 24) | (values[1] << 16) | (values[2] << 8) | values[3]
        # 后 8 个字节为数据区
        return self.send_can_frame(can_id, values[4:12], is_extended=is_extended)

    def read_frame(self) -> Optional[Dict[str, object]]:
        """
        从串口读取一帧 13 字节回包并解析。
        """
        if not self.ser or not self.ser.is_open or self.ser.in_waiting < 13:
            return None

        data = self.ser.read(13)
        func_code = data[12]
        id_bytes = data[0:4]
        shifted_id = struct.unpack(">I", id_bytes)[0]
        is_extended = (id_bytes[3] & 0x02) != 0

        # 逆向解析移位后的 CAN ID
        if is_extended:
            can_id = (shifted_id >> 3) & 0x1FFFFFFF
        else:
            can_id = (shifted_id >> 21) & 0x7FF

        result = {
            "can_id": can_id,
            "data": bytes(data[4:12]),
            "func_code": func_code,
            "is_extended": is_extended,
            "raw_frame": bytes(data),
        }

        frame_type = "EXT" if is_extended else "STD"
        print(f"[RX] {frame_type} ID=0x{can_id:X} FUNC=0x{func_code:02X} DATA={data[4:12].hex(' ').upper()}")
        return result


class ExcavatorController:
    """
    基于 control.txt 和底层协议封装的高级语义控制器。
    负责将底盘行走、机械臂动作和模拟量速度转换为实际的 CAN 数据帧。
    """

    # ==== 模块/继电器对应的 CAN ID 定义 ====
    ID_ARM_SWING = 0x0101    # 第一继电器模块：小臂/回转
    ID_BOOM_BUCKET = 0x0102  # 第二继电器模块：大臂/铲斗
    ID_CHASSIS = 0x0103      # 第三继电器模块：底盘电机控制
    ID_ANALOG = 0x0104       # 模拟量控制器 (速度/电压控制)

    # 模拟量有效范围 (毫伏)
    MIN_ANALOG_MV = 0
    MAX_ANALOG_MV = 5000

    # ==== 底盘动作控制位 (对应 0x0103 数据区第 1 字节) ====
    CHASSIS_BITS = {
        "left_backward": 0x01,   # 左侧后退
        "left_forward": 0x02,    # 左侧前进
        "right_forward": 0x04,   # 右侧前进
        "right_backward": 0x08,  # 右侧后退
        "forward": 0x06,         # 同时前进 (0x02 | 0x04)
        "backward": 0x09,        # 同时后退 (0x01 | 0x08)
        "turn_left": 0x05,       # 左转 (原 0x0A，现对调)
        "turn_right": 0x0A,      # 右转 (原 0x05，现对调)
        "stop": 0x00,            # 停止
    }

    # ==== 大臂与铲斗动作控制位 (对应 0x0102 数据区第 1 字节) ====
    BOOM_BUCKET_BITS = {
        "boom_down": 0x01,       # 大臂下
        "boom_up": 0x02,         # 大臂上
        "bucket_in": 0x04,       # 铲斗回拉
        "bucket_out": 0x08,      # 铲斗外推
        "stop": 0x00,            # 停止
    }

    # ==== 小臂与回转动作控制位 (对应 0x0101 数据区第 1 字节) ====
    ARM_SWING_BITS = {
        "arm_pull": 0x01,        # 小臂回拉
        "arm_push": 0x02,        # 小臂前推
        "swing_right": 0x04,     # 回转右转
        "swing_left": 0x08,      # 回转左转
        "stop": 0x00,            # 停止
    }

    # 兼容直接发送 12 字节原始指令的预设字典
    RAW_COMMANDS = {
        "left_backward": "00 00 01 03 01 00 00 00 00 00 00 00",
        "left_forward": "00 00 01 03 02 00 00 00 00 00 00 00",
        "right_forward": "00 00 01 03 04 00 00 00 00 00 00 00",
        "right_backward": "00 00 01 03 08 00 00 00 00 00 00 00",

        "forward": "00 00 01 03 06 00 00 00 00 00 00 00",
        "backward": "00 00 01 03 09 00 00 00 00 00 00 00",

        "turn_left": "00 00 01 03 0A 00 00 00 00 00 00 00",
        "turn_right": "00 00 01 03 05 00 00 00 00 00 00 00",
        
        "boom_down": "00 00 01 02 01 00 00 00 00 00 00 00",
        "boom_up": "00 00 01 02 02 00 00 00 00 00 00 00",
        "bucket_in": "00 00 01 02 04 00 00 00 00 00 00 00",
        "bucket_out": "00 00 01 02 08 00 00 00 00 00 00 00",
        "arm_pull": "00 00 01 01 01 00 00 00 00 00 00 00",
        "arm_push": "00 00 01 01 02 00 00 00 00 00 00 00",
        "swing_right": "00 00 01 01 04 00 00 00 00 00 00 00",
        "swing_left": "00 00 01 01 08 00 00 00 00 00 00 00",
    }

    def __init__(self, transport: ZSCanTransport):
        self.transport = transport
        # 缓存当前的模拟量，防止在只设置单/双侧履带时将其它通道(如液压)误清零
        self._current_ch1_mv = 0
        self._current_ch2_mv = 0
        self._current_ch3_mv = 0

    def connect(self, do_handshake: bool = True) -> bool:
        """初始化并建立连接"""
        if not self.transport.open():
            return False
        if do_handshake:
            self.transport.handshake()
        return True

    def close(self) -> None:
        """关闭连接"""
        self.transport.close()

    @staticmethod
    def _check_mv(value: int) -> int:
        """校验模拟量(毫伏)是否在 0~5000 安全范围内"""
        if not ExcavatorController.MIN_ANALOG_MV <= value <= ExcavatorController.MAX_ANALOG_MV:
            raise ValueError(
                f"模拟量超出范围: {value}, 预期应在 {ExcavatorController.MIN_ANALOG_MV}..{ExcavatorController.MAX_ANALOG_MV} 之间"
            )
        return value

    @staticmethod
    def _u16_bytes(value: int) -> List[int]:
        """
        将 16 位整数转换为 [高位, 低位] 的双字节列表。
        中盛科技的模拟量通道，输入 1000 即下发 0x03E8，输入 5000 即下发 0x1388。
        """
        checked = ExcavatorController._check_mv(value)
        return [(checked >> 8) & 0xFF, checked & 0xFF]

    def send_named_raw(self, name: str) -> bytes:
        """根据 RAW_COMMANDS 字典的名称发送原始 12 字节指令"""
        if name not in self.RAW_COMMANDS:
            raise KeyError(f"未知的指令名称: {name}")
        return self.transport.send_raw_12byte_command(self.RAW_COMMANDS[name])

    def _send_single_byte_action(self, can_id: int, action_code: int) -> bytes:
        """发送单字节控制的继电器动作 (第 1 字节为动作码，其余 7 字节为 0)"""
        return self.transport.send_can_frame(can_id, [action_code] + [0x00] * 7)

    def set_analog(self, ch1_mv: Optional[int] = None, ch2_mv: Optional[int] = None, ch3_mv: Optional[int] = None) -> bytes:
        """
        设置模拟量通道 (例如调整左右轮的速度，量程为 0-5000 毫伏)。
        根据 control.txt 的注释映射关系：
        - ch1 (左轮): 占用数据区第 1-2 字节
        - ch2 (右轮): 占用数据区第 3-4 字节
        - ch3 (液压/备用): 占用数据区第 5-6 字节
        如果参数为 None，则保留上次设定的值（防止被意外清零，比如单独驱动某侧履带时）。
        """
        if ch1_mv is not None:
            self._current_ch1_mv = ch1_mv
        if ch2_mv is not None:
            self._current_ch2_mv = ch2_mv
        if ch3_mv is not None:
            self._current_ch3_mv = ch3_mv

        data = [0x00] * 8
        data[0:2] = self._u16_bytes(self._current_ch1_mv)
        data[2:4] = self._u16_bytes(self._current_ch2_mv)
        data[4:6] = self._u16_bytes(self._current_ch3_mv)

        return self.transport.send_can_frame(self.ID_ANALOG, data)

    # ==========================
    # ===== 停止与急停动作 =====
    # ==========================

    def stop_analog(self) -> bytes:
        """停止模拟量输出 (所有通道归零)"""
        self._current_ch1_mv = 0
        self._current_ch2_mv = 0
        self._current_ch3_mv = 0
        return self.set_analog(0, 0, 0)

    def stop_chassis(self) -> bytes:
        """停止底盘继电器动作"""
        return self._send_single_byte_action(self.ID_CHASSIS, self.CHASSIS_BITS["stop"])

    def stop_boom_bucket(self) -> bytes:
        """停止大臂与铲斗的继电器动作"""
        return self._send_single_byte_action(self.ID_BOOM_BUCKET, self.BOOM_BUCKET_BITS["stop"])

    def stop_arm_swing(self) -> bytes:
        """停止小臂与回转的继电器动作"""
        return self._send_single_byte_action(self.ID_ARM_SWING, self.ARM_SWING_BITS["stop"])

    def stop_all(self) -> None:
        """紧急停止挖掘机的所有动作 (包括底盘、大臂、小臂和速度模拟量)"""
        self.stop_chassis()
        self.stop_boom_bucket()
        self.stop_arm_swing()
        self.stop_analog()

    # ==========================
    # ===== 底盘行走控制接口 =====
    # ==========================

    def drive_forward(self, left_mv: int, right_mv: int) -> None:
        """控制双侧履带前进，需同时给定左右侧的速度(0-5000mV)"""
        self.set_analog(ch1_mv=right_mv, ch2_mv=left_mv)
        self._send_single_byte_action(self.ID_CHASSIS, self.CHASSIS_BITS["forward"])

    def drive_backward(self, left_mv: int, right_mv: int) -> None:
        """控制双侧履带后退"""
        self.set_analog(ch1_mv=right_mv, ch2_mv=left_mv)
        self._send_single_byte_action(self.ID_CHASSIS, self.CHASSIS_BITS["backward"])

    def turn_left(self, left_mv: int, right_mv: int) -> None:
        """机身左转"""
        self.set_analog(ch1_mv=right_mv, ch2_mv=left_mv)
        self._send_single_byte_action(self.ID_CHASSIS, self.CHASSIS_BITS["turn_left"])

    def turn_right(self, left_mv: int, right_mv: int) -> None:
        """机身右转"""
        self.set_analog(ch1_mv=right_mv, ch2_mv=left_mv)
        self._send_single_byte_action(self.ID_CHASSIS, self.CHASSIS_BITS["turn_right"])

    def left_track_forward(self, mv: int) -> None:
        """仅左侧履带前进"""
        self.set_analog(ch2_mv=mv)
        self._send_single_byte_action(self.ID_CHASSIS, self.CHASSIS_BITS["left_forward"])

    def left_track_backward(self, mv: int) -> None:
        """仅左侧履带后退"""
        self.set_analog(ch2_mv=mv)
        self._send_single_byte_action(self.ID_CHASSIS, self.CHASSIS_BITS["left_backward"])

    def right_track_forward(self, mv: int) -> None:
        """仅右侧履带前进"""
        self.set_analog(ch1_mv=mv)
        self._send_single_byte_action(self.ID_CHASSIS, self.CHASSIS_BITS["right_forward"])

    def right_track_backward(self, mv: int) -> None:
        """仅右侧履带后退"""
        self.set_analog(ch1_mv=mv)
        self._send_single_byte_action(self.ID_CHASSIS, self.CHASSIS_BITS["right_backward"])

    # ==========================
    # ===== 大臂与铲斗控制 =====
    # ==========================

    def boom_up(self) -> bytes:
        """大臂抬起"""
        return self._send_single_byte_action(self.ID_BOOM_BUCKET, self.BOOM_BUCKET_BITS["boom_up"])

    def boom_down(self) -> bytes:
        """大臂放下"""
        return self._send_single_byte_action(self.ID_BOOM_BUCKET, self.BOOM_BUCKET_BITS["boom_down"])

    def bucket_in(self) -> bytes:
        """铲斗回拉"""
        return self._send_single_byte_action(self.ID_BOOM_BUCKET, self.BOOM_BUCKET_BITS["bucket_in"])

    def bucket_out(self) -> bytes:
        """铲斗外推"""
        return self._send_single_byte_action(self.ID_BOOM_BUCKET, self.BOOM_BUCKET_BITS["bucket_out"])

    # ==========================
    # ===== 小臂与回转控制 =====
    # ==========================

    def arm_push(self) -> bytes:
        """小臂前推"""
        return self._send_single_byte_action(self.ID_ARM_SWING, self.ARM_SWING_BITS["arm_push"])

    def arm_pull(self) -> bytes:
        """小臂回拉"""
        return self._send_single_byte_action(self.ID_ARM_SWING, self.ARM_SWING_BITS["arm_pull"])

    def swing_left(self) -> bytes:
        """机身向左回转"""
        return self._send_single_byte_action(self.ID_ARM_SWING, self.ARM_SWING_BITS["swing_left"])

    def swing_right(self) -> bytes:
        """机身向右回转"""
        return self._send_single_byte_action(self.ID_ARM_SWING, self.ARM_SWING_BITS["swing_right"])

    # ==========================
    # ======= 高级辅助函数 ======
    # ==========================

    def run_for(self, start_fn, stop_fn, duration_s: float) -> None:
        """
        组合动作辅助函数：执行指定的开始动作，维持指定时间后自动停止。
        :param start_fn: 开始动作的函数，如 controller.boom_up
        :param stop_fn: 结束动作的函数，如 controller.stop_boom_bucket
        :param duration_s: 持续时间(秒)
        """
        start_fn()
        time.sleep(duration_s)
        stop_fn()

# def build_controller(port: str = "COM3", baudrate: int = 115200, timeout: float = 1.0) -> ExcavatorController:
def build_controller(port: str = "/dev/ttyUSB_Controller", baudrate: int = 115200, timeout: float = 1.0) -> ExcavatorController:
    """
    实例化并返回一个配置好的挖掘机控制器对象。
    :param port: 串口端口名称 (Windows 下通常为 COM3, Linux 下此处使用 udev 规则绑定的 /dev/ttyUSB_Controller)
    :param baudrate: 串口波特率 (默认为 115200)
    """
    transport = ZSCanTransport(port=port, baudrate=baudrate, timeout=timeout)
    return ExcavatorController(transport)


def interactive_shell(controller: ExcavatorController):
    """
    提供一个交互式的命令行控制台，允许用户输入数字执行预设动作或输入原始十六进制指令。
    """
    current_duration = 1.0  # 默认动作持续时间 (秒)

    def do_action(action_name: str, action_func):
        print(f">>> 发送动作: {action_name} {current_duration} 秒 (同时发送模拟量 5000 (0x1388) 提供最大液压/速度)")
        # 1. 设定三个模拟量为最大 5000 即 0x1388 提供动力/流量
        controller.set_analog(ch1_mv=5000, ch2_mv=5000, ch3_mv=5000)
        # 2. 触发继电器动作
        action_func()
        # 3. 维持指定的时间
        time.sleep(current_duration)
        # 4. 全停保护
        controller.stop_all()

    while True:
        print("\n===============================")
        print("      ZS 挖掘机交互式控制台")
        print("===============================")
        print(f"当前动作持续时间: {current_duration} 秒")
        print("-------------------------------")
        print("[0]  退出控制台")
        print("[1]  停止所有动作 (急停)")
        print("[T]  设置动作持续时间")
        print("\n--- 模拟量手动控制 ---")
        print("[2]  手动设置模拟量 (通道 1/2/3, 输入 0-5000)")
        print("\n--- 底盘单侧独立控制 ---")
        print("[3]  左履带 前进")
        print("[4]  左履带 后退")
        print("[5]  右履带 前进")
        print("[6]  右履带 后退")
        print("\n--- 底盘双侧组合控制 ---")
        print("[7]  双侧 前进")
        print("[8]  双侧 后退")
        print("[9]  机身 左转 (左后右前)")
        print("[10] 机身 右转 (左前右后)")
        print("\n--- 大臂/铲斗 ---")
        print("[11] 大臂 抬起 (上)")
        print("[12] 大臂 落下 (下)")
        print("[13] 铲斗 回拉 (收斗)")
        print("[14] 铲斗 外推 (翻斗)")
        print("\n--- 小臂/回转 ---")
        print("[15] 小臂 回拉 (收臂)")
        print("[16] 小臂 前推 (伸臂)")
        print("[17] 回转 左转")
        print("[18] 回转 右转")
        print("\n--- 原始指令 ---")
        print("[99] 手动输入 12 字节十六进制指令")
        print("-------------------------------")

        try:
            choice = input("\n请选择要执行的操作序号: ").strip()
            
            if choice == "0":
                print("退出交互控制台...")
                break
            elif choice.upper() == "T":
                new_time = input("请输入新的动作持续时间(秒，例如 0.5 或 2): ").strip()
                try:
                    val = float(new_time)
                    if val > 0:
                        current_duration = val
                        print(f">>> 动作时间已更新为: {current_duration} 秒")
                    else:
                        print("时间必须大于 0")
                except ValueError:
                    print("无效的时间输入")
            elif choice == "1":
                print(">>> 停止所有动作")
                controller.stop_all()
            elif choice == "2":
                print("\n--- 手动设置模拟量 ---")
                ch = input("请输入要控制的通道号 (1=右履带, 2=左履带, 3=液压/备用, 回车取消): ").strip()
                if ch not in ["1", "2", "3"]:
                    print("已取消或输入无效通道")
                    continue
                val = input("请输入模拟量值 (0-5000，5000满量程): ").strip()
                try:
                    val_int = int(val)
                    if ch == "1":
                        controller.set_analog(ch1_mv=val_int)
                    elif ch == "2":
                        controller.set_analog(ch2_mv=val_int)
                    elif ch == "3":
                        controller.set_analog(ch3_mv=val_int)
                    print(f">>> 已向通道 {ch} 发送模拟量值: {val_int}")
                except ValueError:
                    print("无效的模拟量值")
            elif choice == "3":
                do_action("左履带 前进", lambda: controller.left_track_forward(1000))
            elif choice == "4":
                do_action("左履带 后退", lambda: controller.left_track_backward(1000))
            elif choice == "5":
                do_action("右履带 前进", lambda: controller.right_track_forward(1000))
            elif choice == "6":
                do_action("右履带 后退", lambda: controller.right_track_backward(1000))
            elif choice == "7":
                do_action("双侧 前进", lambda: controller.drive_forward(1000, 1000))
            elif choice == "8":
                do_action("双侧 后退", lambda: controller.drive_backward(1000, 1000))
            elif choice == "9":
                do_action("机身 左转", lambda: controller.turn_left(1000, 1000))
            elif choice == "10":
                do_action("机身 右转", lambda: controller.turn_right(1000, 1000))
            elif choice == "11":
                do_action("大臂 抬起", controller.boom_up)
            elif choice == "12":
                do_action("大臂 落下", controller.boom_down)
            elif choice == "13":
                do_action("铲斗 回拉", controller.bucket_in)
            elif choice == "14":
                do_action("铲斗 外推", controller.bucket_out)
            elif choice == "15":
                do_action("小臂 回拉", controller.arm_pull)
            elif choice == "16":
                do_action("小臂 前推", controller.arm_push)
            elif choice == "17":
                do_action("回转 左转", controller.swing_left)
            elif choice == "18":
                do_action("回转 右转", controller.swing_right)
            elif choice == "99":
                hex_str = input("请输入 12 字节指令 (如: 00 00 01 03 06 00 00 00 00 00 00 00): ").strip()
                try:
                    controller.transport.send_raw_12byte_command(hex_str)
                except Exception as e:
                    print(f"指令发送失败: {e}")
            elif len(choice.split()) == 12:
                # 猜测用户直接输入了 12 字节的十六进制字符串
                try:
                    controller.transport.send_raw_12byte_command(choice)
                except Exception as e:
                    print(f"指令发送失败: {e}")
            else:
                print("无效的选择，请重新输入。")
                
        except KeyboardInterrupt:
            print("\n检测到 Ctrl+C，停止动作并退出...")
            controller.stop_all()
            break
        except Exception as e:
            print(f"执行出错: {e}")


if __name__ == "__main__":
    controller = build_controller()
    if controller.connect():
        try:
            interactive_shell(controller)
        finally:
            controller.close()
