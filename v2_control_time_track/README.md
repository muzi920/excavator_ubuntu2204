# ZS 挖掘机自动剧本控制 (v2)

在 `v1` 版本提供了交互式和点动控制的基础上，`v2` 文件夹提供了一个**动作调度器**和**基于 JSON 剧本的时间开环控制系统**。它的核心目标是让你能够像写**剧本**一样，按时间轴预先编排好一系列的挖掘机动作，并引入了柔性液压控制以消除机械僵硬感。

---

## 1. 核心特性与文件结构

- **`action_gui.py`**: 一个基于 Tkinter 的交互式控制界面。不仅可以手动点击按钮控制挖掘机，还可以**录制**你的动作流程（包含时间与液压参数），并一键导出为 JSON 剧本文件。
- **`run_json_script.py`**: JSON 剧本解析执行器。支持读取导出的 JSON 文件，按时间顺序自动执行一系列控制指令。
- **`preset_working_script.json` / `smooth_working_script.json`**: JSON 剧本文件示例。剧本中详细定义了每个动作的名称、持续时间、三路液压模拟量大小，以及柔性控制（三次样条平滑）的加减速时间。

### 1.1 柔性控制 (Smooth Control)
在实际挖土测试中，突变的液压流量会导致挖掘机动作一顿一顿的僵硬感。因此我们在 V2 剧本中引入了柔性控制参数：
- **`ramp_up_s` (加速时间)**：动作开始时，液压模拟量会在该时间内从 0 平滑增加到目标值。
- **`ramp_down_s` (减速时间)**：动作快结束时，液压模拟量会在该时间内从目标值平滑降至最低值（保留20%防卡死）。
系统底层使用了**三次样条插值 (Cubic Spline Interpolation)** 计算平滑曲线：`s = 3*tau^2 - 2*tau^3`。

---

## 2. 如何使用

### 2.1 通过 GUI 手动控制与录制剧本

确保 CAN 转接板正确连接到电脑后，运行：
```bash
python3 action_gui.py
```
- 在界面上方可以调整 CH1, CH2, CH3（液压）的模拟量。
- 在界面中间可以设置柔性控制的“加速时间”和“减速时间”（默认 0.2 秒）。
- 点击下方的**“🔴 开始录制剧本”**，随后你点击的每一次动作（如“大臂抬起 1.5 秒”）都会被记录在后台。
- 完成动作编排后，点击**“💾 保存为 JSON 剧本”**，即可将录制的流程保存为文件。

### 2.2 运行 JSON 剧本文件

录制好 JSON 剧本（如 `smooth_working_script.json`）后，通过命令行工具自动执行：

```bash
# 运行指定的开环剧本（默认执行 1 次）
python3 run_json_script.py --json smooth_working_script.json

# 循环执行剧本 5 次
python3 run_json_script.py --json smooth_working_script.json --times 5

# 指定其他串口并运行剧本
python3 run_json_script.py --json smooth_working_script.json --port /dev/ttyUSB_Controller --times 3
```

> **注意**：由于时间开环控制在挖掘机实际带载工作时，会因阻力不同导致物理行程产生误差（例如同样的 2 秒，空载能抬起 30 度，满载可能只抬起 15 度）。因此，如果需要极高精度的物理寻的，请前往 **`v4_control_closed`** 使用基于传感器的角度闭环控制方案。

---

## 3. 旧版 Python 脚本调度器

*注：以下为早期的纯代码编写剧本方式，目前更推荐使用上方的 JSON GUI 录制方案。*

你可以直接运行 `action_scheduler.py` 文件：

```bash
python3 action_scheduler.py
```

启动后，程序会提示你选择模式：

```text
请选择模式 [1] 运行预设自动剧本 [2] 交互式手动控制:
```

### 2.1 交互式手动控制 (模式 2)

当你选择 `2` 后，会进入终端的交互界面，允许你**实时输入**要执行的动作、执行时间和推力大小，而不需要预先写在代码里。

操作流程如下：
1. 终端会打印出一个带有编号的动作列表（如 `[1] 双侧前进`，`[9] 大臂抬起` 等）。
2. 输入你想测试的**动作序号**。
3. 输入动作**持续时间**（比如 `1.5`，单位秒）。直接回车默认为 1.0 秒。
4. 输入动作**推力**（即模拟量，`0-5000`）。直接回车默认为 2000。
5. 程序立刻执行，时间到了之后自动停止。然后你可以继续输入下一个动作序号。随时按 `Ctrl+C` 或者直接回车即可退出。

### 2.2 运行预设自动剧本 (模式 1)

如果你想让挖掘机自动按照一套固定流程走，你可以选择 `1`。
默认情况下，它会执行 `main()` 函数里写好的一套演示动作。

#### 编写你自己的剧本

打开 `action_scheduler.py`，定位到 `main()` 函数中的 `choice == "1"` 分支。你可以通过调用以下两个核心方法来编排动作：

#### `scheduler.wait(duration_s)`
- **作用**：不做任何动作，纯粹等待指定的时间（秒）。
- **示例**：`scheduler.wait(1.0)` 等待 1 秒。

#### `scheduler.run_action(action_name, action_func, duration_s, ch1_mv, ch2_mv, ch3_mv)`
- **作用**：执行一个指定的动作，并在达到指定时间后自动停止。
- **参数**：
  - `action_name`: 字符串，用于在控制台打印日志（如 `"大臂抬起"`）。
  - `action_func`: 动作的执行函数（如 `scheduler.controller.boom_up`）。
  - `duration_s`: 动作持续的时间，单位为秒（如 `2.5`）。
  - `ch1_mv` / `ch2_mv` / `ch3_mv`: (可选) 动作期间对应的通道推力，范围 `0-5000`。默认均为 `2000`。

---

## 3. 常见动作代码示例

下面是一些常用动作在剧本中的写法参考：

### 3.1 机械臂动作（无需额外传参给函数）

```python
# 大臂抬起 2.5 秒，给出 5000mV 满量程推力
scheduler.run_action(
    "大臂抬起", 
    scheduler.controller.boom_up, 
    duration_s=2.5, 
    ch1_mv=5000, ch2_mv=5000, ch3_mv=5000
)

# 铲斗外推 1.2 秒，推力 4000mV
scheduler.run_action(
    "铲斗外推", 
    scheduler.controller.bucket_out, 
    duration_s=1.2, 
    ch1_mv=4000, ch2_mv=4000, ch3_mv=4000
)
```

### 3.2 底盘动作（函数本身需要传参，需使用 lambda 包装）

因为底盘控制的函数（如 `drive_forward`）本身需要传入左右履带的速度参数，所以我们需要用 `lambda` 将其包装成一个无参函数传递给调度器：

```python
# 底盘双侧前进 1.5 秒，推力设为 3000mV
scheduler.run_action(
    "双侧前进", 
    lambda: scheduler.controller.drive_forward(3000, 3000), 
    duration_s=1.5, 
    ch1_mv=3000, ch2_mv=3000, ch3_mv=3000
)

# 机身向左转 1 秒，推力 2000mV
scheduler.run_action(
    "机身左转", 
    lambda: scheduler.controller.turn_left(2000, 2000), 
    duration_s=1.0, 
    ch1_mv=2000, ch2_mv=2000, ch3_mv=2000
)
```

## 4. 注意事项
1. 运行前请确保脚本中的 `port="COM3"` 与你实际插上的串口一致。
2. 调度器会在整个剧本运行结束（或中途异常退出）时，在 `finally` 块中调用 `stop_all()` 进行最终的安全断电保护。