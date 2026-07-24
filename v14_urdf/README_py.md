# v14_urdf Python 脚本详细说明

本文档详细列出 `v14_urdf` 目录下每个 Python 脚本的功能、类、以及每个函数/方法的作用。
目标是让你在后续统一接口或重构时，能快速定位每个脚本的职责。

---

## 文件总览

| 脚本文件 | 角色 | 是否可独立运行 |
|---|---|---|
| `ros_joint_bridge.py` | 底层 ROS 2 `/joint_states` 收发桥接 | 否（被其他脚本导入） |
| `sim_angle_controller.py` | 仿真版角度控制器（v4 语义） | 否（被其他脚本导入） |
| `script_replay.py` | JSON 剧本回放引擎 | 否（被其他脚本导入） |
| `point_to_dig_dump_trajectory.py` | 单点挖掘-卸料轨迹生成器 | 是（CLI） |
| `terminal_stepper.py` | 终端步进执行器 | 是（CLI） |
| `replay_json_script.py` | JSON 剧本自动回放入口 | 是（CLI） |
| `v4_urdf_sim_gui.py` | 仿真版 Tkinter GUI | 是（GUI） |

**依赖关系：**

```text
v4_urdf_sim_gui.py
  ├── ros_joint_bridge.py
  ├── sim_angle_controller.py
  └── script_replay.py
         └── ros_joint_bridge.py

replay_json_script.py
  ├── ros_joint_bridge.py
  └── script_replay.py

terminal_stepper.py
  ├── ros_joint_bridge.py
  └── sim_angle_controller.py

point_to_dig_dump_trajectory.py
  └── v10_cailbration_arm/inverse_kinematics.py
```

---

## 1. ros_joint_bridge.py

**角色：** ROS 2 `/joint_states` 的收发桥接层。所有需要与 URDF 交互的脚本都通过它来发布和读取关节状态。

**核心类：** `RosJointBridge`

```text
输入：v4 风格的关节角度（deg，字典形式）
输出：sensor_msgs/msg/JointState（rad），发布到 /joint_states
```

### 类：RosJointBridge

| 方法 | 参数 | 返回值 | 作用 |
|---|---|---|---|
| `__init__(node_name)` | `node_name`: ROS 节点名，默认 `"v14_v4_joint_state_bridge"` | 无 | 初始化 ROS 2 节点，创建 `/joint_states` 的 Publisher 和 Subscriber，启动后台 spin 线程 |
| `_deg_to_rad(value)` | `value`: 角度值（度） | 浮点数（弧度） | 静态方法，度转弧度 |
| `_rad_to_deg(value)` | `value`: 弧度值 | 浮点数（度） | 静态方法，弧度转度 |
| `_spin()` | 无 | 无 | 后台线程，持续调用 `rclpy.spin_once()` 处理 ROS 2 回调 |
| `_on_joint_state(msg)` | `msg`: `JointState` 消息 | 无 | 回调函数，收到 `/joint_states` 时缓存最新关节状态和时间戳 |
| `close()` | 无 | 无 | 关闭 ROS 节点，停止 spin 线程，释放资源 |
| `get_v4_angles_from_joint_states_deg()` | 无 | `dict` 或 `None` | 读取当前缓存的关节状态，转换为 v4 风格的角度字典：`{"ts", "swing_yaw", "boom_swing", "arm_boom", "bucket_arm"}`，单位为度 |
| `publish_v4_targets_deg(**kwargs)` | `kwargs`: 关节名到目标角度的映射（度） | `bool` | 把 v4 风格的目标角度（度）转换为 `JointState`（弧度），发布到 `/joint_states`。支持部分更新，未指定的关节保持上次值 |

**关节名映射：**

| v4 语义名 | URDF 关节名 |
|---|---|
| `swing_yaw` | `swing_joint` |
| `boom_swing` | `boom_joint` |
| `arm_boom` | `arm_joint` |
| `bucket_arm` | `bucket_joint` |

---

## 2. sim_angle_controller.py

**角色：** 仿真版角度控制器。模拟 `v4_control_closed/angle_controller.py` 的控制接口，但不下发硬件指令，只通过 `RosJointBridge` 发布到 `/joint_states`。

**核心类：** `SimAngleController`

```text
输入：v4 语义的关节名 + 目标角度
输出：通过 RosJointBridge 发布 /joint_states
```

### 类：SimAngleController

| 方法 | 参数 | 返回值 | 作用 |
|---|---|---|---|
| `__init__(ros_bridge)` | `ros_bridge`: `RosJointBridge` 实例 | 无 | 初始化控制器，设置关节限位，创建日志文件 |
| `log_msg(msg, also_print)` | `msg`: 日志内容；`also_print`: 是否同时打印到终端 | 无 | 写入带时间戳的日志，同时可选打印到终端 |
| `update_sensor_data(sensor_data)` | `sensor_data`: 传感器数据字典 | 无 | 更新当前传感器数据缓存（仿真中实际未使用硬件传感器数据） |
| `stop_all()` | 无 | 无 | 仿真模式下仅打印日志，不下发硬件急停 |
| `move_joint_to_angle(...)` | `joint_name`: 关节名；`target_angle`: 目标角度（度）；其他参数在仿真中被忽略 | 无 | 核心方法。将目标角度钳位到关节限位内，然后通过 `ros_bridge.publish_v4_targets_deg()` 发布到 `/joint_states` |

**关节限位：**

| 关节 | 最小值（度） | 最大值（度） |
|---|---|---|
| `boom_swing` | -5.0 | 55.0 |
| `arm_boom` | -5.0 | 95.0 |
| `bucket_arm` | -95.0 | 20.0 |
| `swing_yaw` | -180.0 | 180.0 |

---

## 3. script_replay.py

**角色：** JSON 剧本回放引擎。支持两种回放模式：时间推进模式和角度反馈模式。被 `replay_json_script.py` 和 `v4_urdf_sim_gui.py` 共用。

**核心类：** `JsonScriptReplayer`

```text
输入：JSON 剧本（script 数组），通过 RosJointBridge 发布
输出：按剧本顺序逐步骤驱动 /joint_states
```

### 类：JsonScriptReplayer

| 方法 | 参数 | 返回值 | 作用 |
|---|---|---|---|
| `__init__(ros_bridge, ...)` | `ros_bridge`: 桥接实例；`status_callback`: 状态回调；其余为反馈模式参数 | 无 | 初始化回放器，设置反馈模式参数（速度、容差、频率等） |
| `is_running()` | 无 | `bool` | 当前是否有剧本正在执行 |
| `load_script(script_path)` | `script_path`: JSON 文件路径 | `list` | 加载 JSON 剧本，支持顶层为数组或 `{script: [...]}` 格式 |
| `stop()` | 无 | 无 | 请求停止当前回放 |
| `wait(timeout)` | `timeout`: 超时时间 | 无 | 等待回放线程结束 |
| `start(script, script_path, daemon)` | `script`: 步骤列表；`script_path`: 文件路径；`daemon`: 是否守护线程 | `Thread` | 启动回放线程 |
| `_notify(info)` | `info`: 状态字典 | 无 | 调用用户注册的 `status_callback`，汇报回放进度 |
| `_estimate_duration(...)` | `joint_name`, `current_angle`, `target_angle`, `step` | `float`（秒） | 估算单步执行时长（时间推进模式用） |
| `_feedback_speed(joint_name)` | `joint_name`: 关节名 | `float`（度/秒） | 返回反馈模式下该关节的运动速度 |
| `_run_step_feedback(...)` | `step`, `current_state`, `step_index`, `total_steps`, `start_time` | `(current_state, reached, timed_out)` | 反馈模式的单步执行。按速度插值推进，或持续发布目标角度等待反馈到位 |
| `_get_current_state()` | 无 | `dict` | 从 `/joint_states` 读取当前四个关节角度（度） |
| `_publish_joint_value(joint_name, value)` | `joint_name`: 关节名；`value`: 目标角度（度） | 无 | 发布单个关节的目标值 |
| `_run_script(script)` | `script`: 步骤列表 | 无 | 主回放循环。先估算总时长，然后按步骤逐一执行（时间模式或反馈模式） |

**回放模式对比：**

| 模式 | 适用场景 | 推进方式 |
|---|---|---|
| 时间推进（默认） | 快速预览 | 按帧插值，每步按固定 FPS 30Hz 发布 |
| 反馈模式（`--feedback`） | 精确验证 | 到达目标角度（容差内）才进入下一步 |

---

## 4. point_to_dig_dump_trajectory.py

**角色：** 从三维挖掘点和卸料点生成可回放的关节轨迹 JSON 文件。调用 `v10_cailbration_arm/inverse_kinematics.py` 的逆解器。

**核心类：** `DigDumpPlanner`

```text
输入：挖掘点 (x,y,z)、卸料点 (x,y,z)
输出：{metadata, script} JSON 文件，共 14 步
```

### 类：DigDumpPlanner

| 方法 | 参数 | 返回值 | 作用 |
|---|---|---|---|
| `__init__()` | 无 | 无 | 初始化逆解器 `ExcavatorIK()`，设置铲斗长度和关节限位 |
| `_radius_and_yaw(point)` | `point`: `(x, y, z)` | `(radius, yaw_deg, z)` | 静态方法。把三维点转为柱坐标 `(r, yaw, z)` |
| `_within_limits(result)` | `result`: 逆解结果字典 | `bool` | 检查逆解结果是否在关节限位内（回转角除外） |
| `_clamp(joint_name, value)` | `joint_name`: 关节名；`value`: 角度值 | `float` | 将角度值钳位到该关节的限位范围内 |
| `_search_pose(radius, z, angle_candidates, ...)` | `radius`, `z`: 目标点坐标；`angle_candidates`: 铲斗姿态角候选列表 | `list[(score, bucket_abs, result)]` | 搜索可行的铲斗姿态解，按评分排序返回所有可行解 |
| `solve_dig_pose(dig_point)` | `dig_point`: `(x, y, z)` | `dict` | 求解挖掘点的最优姿态。搜索半开斗（-70 到 -20 度）下探的解 |
| `solve_dump_pose(dump_point)` | `dump_point`: `(x, y, z)` | `dict` | 求解卸料点的最优空中卸料姿态。搜索多个安全高度（0.35~0.55m）和铲斗角度 |
| `_step(step_id, joint, description, target_val, ...)` | 步骤参数 | `dict` | 静态方法。构建单步 JSON 字典 |
| `plan(dig_point, dump_point)` | 挖掘点、卸料点的 `(x,y,z)` | `dict` | 核心方法。完整规划挖掘到卸料的 14 步轨迹，返回 `{metadata, script}` |

### main() 函数

命令行入口，接收 `--dig-x/y/z` 和 `--dump-x/y/z` 参数，输出 JSON 到 `--output` 指定路径。

**生成的 14 步轨迹：**

| 步骤 | 关节 | 描述 | 阶段 |
|---|---|---|---|
| 1 | `swing_yaw` | 对准挖掘点（回转） | 初始化 |
| 2 | `bucket_arm` | 挖掘预备-半开斗 | 初始化 |
| 3 | `boom_swing` | 挖掘预备-大臂下探 | 初始化 |
| 4 | `arm_boom` | 挖掘预备-小臂下探 | 初始化 |
| 5 | `bucket_arm` | 收斗取料 | 挖掘 |
| 6 | `arm_boom` | 回拉小臂抬料 | 挖掘 |
| 7 | `boom_swing` | 抬大臂离开挖掘点 | 挖掘 |
| 8 | `arm_boom` | 运输姿态-收小臂 | 运输 |
| 9 | `bucket_arm` | 运输姿态-稳料 | 运输 |
| 10 | `swing_yaw` | 回转到卸料点 | 回转 |
| 11 | `boom_swing` | 卸料预备-大臂 | 卸料 |
| 12 | `arm_boom` | 卸料预备-小臂 | 卸料 |
| 13 | `bucket_arm` | 卸料预备-铲斗半开 | 卸料 |
| 14 | `bucket_arm` | 打开铲斗卸料 | 卸料 |

---

## 5. terminal_stepper.py

**角色：** 终端交互式步进执行器。加载 JSON 剧本后提供 REPL 风格命令行，支持逐步执行、跳步、连续执行等操作。适合无 GUI 环境下精细调试。

**核心类：** `TerminalStepper`

```text
输入：JSON 剧本文件路径
交互：终端 REPL 命令行
输出：通过 SimAngleController + RosJointBridge 发布 /joint_states
```

### 类：TerminalStepper

| 方法 | 参数 | 返回值 | 作用 |
|---|---|---|---|
| `__init__(script_path)` | `script_path`: JSON 剧本路径 | 无 | 加载剧本，初始化桥接和控制器 |
| `close()` | 无 | 无 | 关闭 ROS 桥接，释放资源 |
| `get_angles()` | 无 | `dict` | 读取当前四个关节角度（度） |
| `print_header()` | 无 | 无 | 打印启动信息（脚本路径、步数、挖掘/卸料点摘要） |
| `print_help()` | 无 | 无 | 打印所有可用命令 |
| `print_angles()` | 无 | 无 | 打印当前 `/joint_states` 中的四个关节角度 |
| `print_current_step()` | 无 | 无 | 打印当前选中的步骤信息 |
| `list_steps()` | 无 | 无 | 列出全部步骤，当前选中步骤用 `->` 标记 |
| `execute_step(index)` | `index`: 步骤索引（从 0 开始） | `bool` | 执行指定步骤，通过 `SimAngleController.move_joint_to_angle()` 发布目标角度 |
| `run_next()` | 无 | 无 | 执行下一步 |
| `run_prev()` | 无 | 无 | 执行上一步 |
| `goto(index_1_based)` | `index_1_based`: 1-based 步骤编号 | 无 | 跳转到指定步骤（不执行），用于 `goto N` 命令 |
| `reload()` | 无 | 无 | 重新读取 JSON 文件，重置步骤索引 |
| `run_all_from_current()` | 无 | 无 | 从当前步骤开始连续执行到末尾 |
| `repl()` | 无 | 无 | 主 REPL 循环，读取终端输入并分发命令 |

### main() 函数

命令行入口，接收可选的 `script_path` 参数（默认为 `json/generated_dig_dump_trajectory.json`）。

**支持的 REPL 命令：**

| 命令 | 作用 |
|---|---|
| `help` | 显示帮助 |
| `list` | 列出全部步骤 |
| `show` | 显示当前选中步骤详情 |
| `angles` | 显示当前 `/joint_states` 角度 |
| `next` | 选择下一步并执行 |
| `prev` | 选择上一步并执行 |
| `goto N` | 选中第 N 步（不执行） |
| `run` | 执行当前选中步骤 |
| `runall` | 从当前步骤执行到末尾 |
| `reload` | 重新读取 JSON 文件 |
| `reset` | 重置到第 1 步 |
| `quit` / `exit` | 退出 |

---

## 6. replay_json_script.py

**角色：** JSON 剧本自动回放的 CLI 入口。支持时间推进模式和角度反馈模式，是 `script_replay.py` 的命令行封装。

```text
输入：JSON 剧本文件路径 + 回放参数
输出：自动按顺序执行全部步骤，驱动 /joint_states
```

### main() 函数

| 参数 | 类型 | 默认值 | 作用 |
|---|---|---|---|
| `script_path` | positional | 必填 | 要回放的 JSON 剧本路径 |
| `--background` | flag | `False` | 后台运行，持续保持进程 |
| `--feedback` | flag | `False` | 启用角度反馈模式（到位再下一步） |
| `--tolerance-deg` | float | `1.5` | 反馈模式到位容差（度） |
| `--feedback-publish-mode` | choice | `"interpolate"` | 反馈发布方式：`interpolate`（速度插值）或 `target`（持续发目标） |
| `--joint-speed-deg-s` | float | `12.0` | 反馈模式非回转关节速度（度/秒） |
| `--swing-speed-deg-s` | float | `30.0` | 反馈模式回转关节速度（度/秒） |
| `--fps` | float | `30.0` | 反馈模式发布频率（Hz） |
| `--max-step-s` | float | `30.0` | 单步最大等待时间（秒） |
| `--min-step-s` | float | `0.0` | 单步最短时间（秒），用于放慢观察 |
| `--dwell-s` | float | `0.05` | 每步结束后的停顿（秒） |

**典型用法：**

```bash
# 时间推进模式（快速预览）
/usr/bin/python3 src/shandong/v14_urdf/replay_json_script.py \
  src/shandong/v14_urdf/json/auto_dig_imu_30_loops.json

# 反馈模式（精确验证）
/usr/bin/python3 src/shandong/v14_urdf/replay_json_script.py \
  src/shandong/v14_urdf/mode1/real_pcd/output/base_link_0000_mode1_scene_v2/scene_mode1_task_plan.json \
  --feedback \
  --feedback-publish-mode interpolate \
  --joint-speed-deg-s 6 \
  --swing-speed-deg-s 15 \
  --min-step-s 1.0 \
  --dwell-s 0.4
```

---

## 7. v4_urdf_sim_gui.py

**角色：** 仿真版 Tkinter GUI。提供完整的图形界面，包含关节状态显示、目标角度控制、JSON 剧本录制/回放、手动步进验证等功能。是 `v4_control_closed/closed_loop_gui_imu.py` 的仿真替代品。

```text
输入：用户 GUI 操作
输出：通过 RosJointBridge + SimAngleController 发布 /joint_states
```

### 类：V4UrdfSimGUI

| 方法 | 参数 | 返回值 | 作用 |
|---|---|---|---|
| `__init__(root)` | `root`: Tkinter 根窗口 | 无 | 初始化 GUI，创建 ROS 桥接、角度控制器、剧本回放器，构建界面 |
| `_build_ui()` | 无 | 无 | 构建全部 GUI 组件：关节状态区、保留 v4 参数区、闭环控制区、录制区、剧本回放区、手动步进区 |
| `_toggle_recording()` | 无 | 无 | 开始/停止录制剧本 |
| `_save_script()` | 无 | 无 | 将录制的剧本保存为 JSON 文件 |
| `_load_script()` | 无 | 无 | 从文件对话框加载 JSON 剧本 |
| `_start_loaded_script()` | 无 | 无 | 启动已加载剧本的自动回放 |
| `_stop_loaded_script()` | 无 | 无 | 停止剧本回放 |
| `_refresh_steps_list()` | 无 | 无 | 刷新步骤列表框内容 |
| `_update_selected_step_label()` | 无 | 无 | 更新当前选中步骤的标签文字 |
| `_on_step_select(_event)` | 无 | 无 | 列表框选中事件回调 |
| `_on_step_double_click(_event)` | 无 | 无 | 列表框双击事件回调，执行选中步骤 |
| `_select_prev_step()` | 无 | 无 | 选中上一步 |
| `_select_next_step()` | 无 | 无 | 选中下一步 |
| `_reset_step_selection()` | 无 | 无 | 重置步骤选择到第 1 步 |
| `_execute_selected_step()` | 无 | 无 | 执行当前选中的单个步骤 |
| `_on_replay_status(info)` | `info`: 状态字典 | 无 | 剧本回放状态回调，更新 GUI 状态标签 |
| `_record_current_angle(joint_name, label_text, target_var, is_init)` | 关节信息 | 无 | 把当前角度记录到录制列表 |
| `_handle_move(joint_name, label_text, target_val)` | 关节名、标签、目标值 | 无 | 执行关节移动，如果正在录制则同时记录 |
| `_create_ctrl_row(parent, row, label_text, joint_name, target_var, entry_label)` | GUI 参数 | 无 | 创建一行关节控制组件（输入框 + 移动按钮 + 记录按钮） |
| `_update_loop()` | 无 | 无 | 50ms 定时循环，从 `/joint_states` 读取角度并刷新 GUI 显示 |
| `on_closing()` | 无 | 无 | 窗口关闭回调，停止所有控制、回放，关闭 ROS 桥接，退出进程 |

---

## 脚本间数据流

```text
                    ┌─────────────────────┐
                    │  point_to_dig_dump_  │
                    │  trajectory.py       │
                    │  (v10 IK 求解)       │
                    └─────────┬───────────┘
                              │ 输出 JSON
                              v
                    ┌─────────────────────┐
                    │  generated_dig_dump_ │
                    │  trajectory.json     │
                    └─────────┬───────────┘
                              │
              ┌───────────────┼───────────────┐
              v               v               v
   ┌──────────────┐ ┌──────────────┐ ┌──────────────┐
   │ terminal_    │ │ replay_json_ │ │ v4_urdf_sim_ │
   │ stepper.py   │ │ script.py    │ │ gui.py       │
   │ (REPL 交互)  │ │ (自动回放)   │ │ (GUI 操作)   │
   └──────┬───────┘ └──────┬───────┘ └──────┬───────┘
          │                │                │
          v                v                v
   ┌──────────────────────────────────────────────┐
   │         sim_angle_controller.py              │
   │         (关节限位 + 目标角度发布)             │
   └──────────────────┬───────────────────────────┘
                      │
                      v
   ┌──────────────────────────────────────────────┐
   │         ros_joint_bridge.py                  │
   │         (deg -> rad, 发布 /joint_states)     │
   └──────────────────┬───────────────────────────┘
                      │
                      v
   ┌──────────────────────────────────────────────┐
   │         /joint_states (ROS 2 Topic)          │
   └──────────────────┬───────────────────────────┘
                      │
                      v
   ┌──────────────────────────────────────────────┐
   │         robot_state_publisher                │
   │         (计算 TF)                            │
   └──────────────────┬───────────────────────────┘
                      │
                      v
   ┌──────────────────────────────────────────────┐
   │         RViz2                                │
   │         (URDF 模型可视化)                    │
   └──────────────────────────────────────────────┘
```

---

## 运行环境要求

所有 ROS 2 Python 脚本必须使用系统 Python 和 ROS 2 环境：

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 <script.py>
```

不要使用 Conda 或其他非系统 Python，否则 `rclpy` 会因为版本不匹配而失败。
