# 挖掘机 V2 时间开环控制模块 (v2_control_time_track)

本目录包含了基于**时间开环**的挖掘机控制程序。它通过发送动作指令（如“大臂抬起”）并持续指定的时间，结合平滑的三次样条曲线进行液压流量的柔性加减速控制，从而实现对挖掘机的程序化作业控制。

> **注意**: V2 是开环时间控制，不会实时读取倾角传感器来判断是否到达位置。如果需要高精度的角度闭环控制，请移步至 `v4_control_closed` 目录。

## 核心特性
- **柔性控制 (Ramp-up / Ramp-down)**: 避免了直接拉满液压或瞬间切断液压带来的机械冲击（水锤效应）。所有动作在起步时平滑增加液压流量，停止前平滑减小液压流量。
- **剧本示教与执行**: 支持通过 GUI 面板进行动作录制，并将动作序列保存为 JSON 剧本文件。
- **脚本化运行**: 提供专用的脚本执行器，支持按 JSON 文件中的步骤依次执行动作。

---

## 文件与脚本说明

### 1. `action_scheduler.py` (底层时间调度器)
- **作用**: 核心调度逻辑。它封装了 V1 的 CAN 通信模块（`zs_excavator_controller.py`），提供 `run_action` 方法。
- **机制**: 在指定的 `duration_s` 时间内，根据 `ramp_up_s` 和 `ramp_down_s` 参数，计算当前时刻的缩放比例（Scale），并将缩放后的 `ch1`, `ch2`, `ch3` 模拟量通过 CAN 实时下发给执行机构。

### 2. `action_gui.py` (V2 时间控制与录制面板)
- **作用**: 提供了一个图形化界面（GUI），用于手动测试挖掘机各个关节的动作，并支持“示教录制”。
- **功能**:
  - 可以实时调整 CH1, CH2, CH3 以及加减速时间。
  - 点击“开始录制剧本”后，所有手动触发的动作及参数都会被记录。
  - 支持将录制好的动作序列保存为 JSON 文件。
- **运行**:
  ```bash
  python3 src/shandong/v2_control_time_track/action_gui.py
  ```

### 3. `run_json_script.py` (JSON 剧本执行器)
- **作用**: 用于在终端命令行中自动执行预先录制好的 JSON 剧本文件。
- **运行**:
  ```bash
  python3 src/shandong/v2_control_time_track/run_json_script.py --json <你的剧本文件.json>
  ```

### 4. 示例 JSON 剧本文件
- `preset_working_script.json`: 预设的工作剧本示例。
- `script_demo.json`: 用于测试基本动作流的 Demo 剧本。
- `smooth_working_script.json`: 重点展示了柔性加减速配置的平滑动作剧本（例如：大臂落下时较长的 `ramp_down_s` 配置，以减小顿挫感）。