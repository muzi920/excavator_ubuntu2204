# ZS 挖掘机控制接口 (v1 调试版)

本项目是一个针对中盛科技“数字量/模拟量输出系列(CAN版)”设备的控制库，用于驱动挖掘机底盘行走、大臂、铲斗、小臂及回转等动作。目前包含 Python 与 C++ 两个同步版本的实现。本阶段所有代码统一存放在 `v1` 文件夹下，待实物测试通过后，再迁移为 ROS2 接口。

---

## 1. 核心设计说明

该控制库主要分为两层：
1. **传输层 (ZSCanTransport)**：处理串口连接、13 字节协议封装（CAN ID、8字节数据区、1字节功能码）以及底层的读写收发。
2. **控制语义层 (ExcavatorController)**：将 `control.txt` 中的 12 字节控制指令解析并封装成具有明确语义的函数（如 `drive_forward`, `boom_up` 等），并自动处理模拟量（速度）的参数传递。

### 1.1 代码结构与调用关系图

以下是脚本中各个类与函数的作用及其层级调用关系的脑图结构：

```text
zs_excavator_controller.py
│
├── build_controller()
│   └── 作用: 工厂函数，实例化传输层和控制层，返回可直接使用的 ExcavatorController 对象。
│
├── class ZSCanTransport
│   │   作用: 底层通信类，负责串口打通与 13 字节中盛协议帧的封装与解析。
│   ├── open() / close()          : 串口打开与关闭。
│   ├── handshake()               : 发送 0x0303 握手帧。
│   ├── _encode_can_id()          : 内部方法，对 CAN ID 进行移位编码 (区分标准/扩展帧)。
│   ├── send_can_frame()          : 核心发送方法，组装 13 字节帧下发到串口。
│   ├── send_raw_12byte_command() : 兼容方法，将 12 字节十六进制字符串转为 CAN 帧下发。
│   └── read_frame()              : 从串口读取并逆向解析回包帧。
│
├── class ExcavatorController
│   │   作用: 高级控制语义层，基于 ZSCanTransport，将动作翻译为具体的硬件指令。
│   ├── connect() / close()       : 调用传输层的 open/close，并在 connect 时自动握手。
│   │
│   ├── 内部工具方法:
│   │   ├── _check_mv()           : 校验模拟量输入是否在 0~5000 范围内。
│   │   ├── _u16_bytes()          : 将输入的 0~5000 值拆分为高低双字节列表，转为 16 进制。
│   │   └── _send_single_byte_action() : 发送继电器动作的核心封装 (1个动作码 + 7个0x00)。
│   │
│   ├── 模拟量控制 (ID: 0x0104):
│   │   ├── set_analog()          : 控制 3 个通道的模拟量输出 (0-5000对应 0x0000-0x1388)。
│   │   └── stop_analog()         : 模拟量通道清零。
│   │
│   ├── 底盘控制 (ID: 0x0103):
│   │   ├── drive_forward() / drive_backward()  : 双侧履带同步动作 (底层先调用 set_analog, 再触发动作)。
│   │   ├── turn_left() / turn_right()          : 双侧差速转向。
│   │   ├── left_track_forward/backward()       : 左履带单侧动作。
│   │   ├── right_track_forward/backward()      : 右履带单侧动作。
│   │   └── stop_chassis()                      : 停止底盘动作。
│   │
│   ├── 大臂与铲斗 (ID: 0x0102):
│   │   ├── boom_up() / boom_down()             : 大臂起降。
│   │   ├── bucket_in() / bucket_out()          : 铲斗收放。
│   │   └── stop_boom_bucket()                  : 停止大臂与铲斗。
│   │
│   ├── 小臂与回转 (ID: 0x0101):
│   │   ├── arm_push() / arm_pull()             : 小臂伸缩。
│   │   ├── swing_left() / swing_right()        : 机身回转。
│   │   └── stop_arm_swing()                    : 停止小臂与回转。
│   │
│   └── 组合与安全方法:
│       ├── stop_all()                          : 紧急停止 (调用所有 stop 方法清零)。
│       ├── run_for()                           : 定时动作辅助函数 (执行动作 -> sleep -> 停止)。
│       └── send_named_raw()                    : 按 control.txt 字典名发送原始指令 (如 "forward")。
│
└── interactive_shell()
    └── 作用: 终端交互菜单，循环接收用户数字输入，自动组装动作和满载模拟量下发测试。
```

---

## 2. 快速使用 (Python)

### 2.1 交互式控制台 (GUI)

你可以直接在终端中运行带有界面的主程序进行全功能的实时测试：

```bash
python zs_excavator_gui.py
```

**GUI 功能亮点**：
- **实时模拟量控制**：顶部设有 3 个滑动条控制 CH1 (左履带)、CH2 (右履带)、CH3 (液压) 的数值 (默认 2000)。滑动或输入回车后立即下发。
- **按压控制 (点动)**：无论是点击界面按钮，还是按下快捷键，机器都会**按下执行、松开停止**。
- **状态监控**：底部会实时蓝色字体显示当前发送给底层的三个模拟量通道实际数值，防止误操作。

**键盘快捷键映射**：
- **底盘行走**：`W` (前), `S` (后), `A` (左转), `D` (右转)
- **履带独立**：`Q` (左前), `Z` (左后), `E` (右前), `C` (右后)
- **大臂/铲斗**：小键盘 `8` (大臂上), `2` (大臂下), `4` (铲斗收), `6` (铲斗放)
- **小臂/回转**：`I` (小臂收), `M` (小臂伸), `J` (回转左), `L` (回转右)
- **急停保护**：`空格键 (Space)` 立即停止所有继电器动作。

---

### 2.2 代码中调用示例

如果你需要在自己的脚本中调用：

```python
import time
from zs_excavator_controller import build_controller

# 1. 实例化并连接 (请根据实际情况修改端口，如 COM3 或 /dev/ttyUSB0)
controller = build_controller(port="COM3", baudrate=115200)

if controller.connect():
    try:
        print("连接成功，开始执行动作...")
        
        # 2. 控制底盘前进 (左右轮分别给定 5000mV 模拟量)
        controller.drive_forward(left_mv=5000, right_mv=5000)
        time.sleep(1.0)
        
        # 3. 停止所有动作
        controller.stop_all()

        # 4. 控制大臂抬起
        controller.boom_up()
        time.sleep(0.5)
        controller.stop_boom_bucket()
        
    finally:
        # 5. 确保在退出前关闭串口连接
        controller.close()
```

---

## 3. 控制接口参考字典

### 3.1 建立与断开连接
- `connect(do_handshake=True)`: 打开串口连接。如果 `do_handshake` 为 True，将自动发送一次建连握手指令 (ID: 0x0303)。
- `close()`: 安全关闭串口。

### 3.2 紧急停止
- `stop_all()`: 停止挖掘机的所有动作（包括底盘、大臂、小臂，并将模拟量归零）。
- `stop_chassis()`: 仅停止底盘行走继电器。
- `stop_boom_bucket()`: 仅停止大臂与铲斗继电器。
- `stop_arm_swing()`: 仅停止小臂与回转继电器。
- `stop_analog()`: 仅停止模拟量（左右轮速度设为 0）。

### 3.3 底盘行走 (需传入左右轮速度 0~5000mV)
*注：传入参数 `left_mv` 和 `right_mv` 分别对应左侧 (CH1) 和右侧 (CH2) 履带的驱动速度。单侧控制不会影响未被调用通道的数值。*
- `drive_forward(left_mv, right_mv)`: 双侧履带同时前进。
- `drive_backward(left_mv, right_mv)`: 双侧履带同时后退。
- `turn_left(left_mv, right_mv)`: 左转（左前右后 / 右侧前进）。
- `turn_right(left_mv, right_mv)`: 右转（右前左后 / 左侧前进）。
- `left_track_forward(mv)`: 仅左侧履带前进。
- `left_track_backward(mv)`: 仅左侧履带后退。
- `right_track_forward(mv)`: 仅右侧履带前进。
- `right_track_backward(mv)`: 仅右侧履带后退。

### 3.4 大臂与铲斗
- `boom_up()`: 大臂抬起。
- `boom_down()`: 大臂放下。
- `bucket_in()`: 铲斗回拉。
- `bucket_out()`: 铲斗外推。

### 3.5 小臂与回转
- `arm_push()`: 小臂前推。
- `arm_pull()`: 小臂回拉。
- `swing_left()`: 机身向左回转。
- `swing_right()`: 机身向右回转。

### 3.6 高级/调试接口
- `set_analog(ch1_mv=None, ch2_mv=None, ch3_mv=None)`: 直接设置通道1(左履带)、通道2(右履带)、通道3(液压)的模拟量。如果只传入部分通道参数，其余通道将保持上次设定的缓存值不变。
- `stop_analog()`: 清零所有三个通道的模拟量输出。
- `send_named_raw(name)`: 直接发送 `control.txt` 中的 12 字节原始十六进制指令（例如：`send_named_raw("forward")`）。
- `run_for(start_fn, stop_fn, duration_s)`: 执行某个动作，维持指定时间后自动调用停止函数。

---

## 4. C++ 版本说明

`zs_excavator_controller.cpp` 提供了与 Python 完全一致的类结构和函数命名（采用驼峰命名法）。在编译和运行时：
1. 它使用了 Windows 的 `HANDLE` API 进行串口通信（`<windows.h>`）。
2. 在迁移到 ROS2 (Linux 平台) 之前，它是作为 Windows 下联调逻辑同步的基准版本。
3. `main()` 函数已被宏 `BUILD_ZS_CONTROLLER_DEMO` 包裹，若需编译测试可直接开启此宏。
