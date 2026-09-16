# V11 多模态挖掘机端到端数据集采集系统

本目录（V11）是 V4（闭环控制）、V5（雷达与IMU读取）和 V10（运动学与3D可视化）的集大成者。
它的核心目标是：**提供一键式的数据集采集功能**，在执行自动闭环挖掘动作（复现 JSON 剧本）或手动遥控操作时，**高频、严格时间戳对齐地记录挖掘机的所有内外状态**，最终生成标准的自动驾驶/具身智能数据集格式，并支持执行过程的 **3D 实时可视化与自动 GIF 留档**。

## 核心功能特色

1. **统一时间轴 (Time Sync)**：所有数据流（图像、点云、本体传感器、控制指令）均采用系统统一时间戳 (`time.time()`) 记录，确保离线训练时各模态数据的绝对对齐。
2. **遥控与自动化双模式支持**：
   - 遥控模式：直接点击“启动端到端数据采集”，手动遥控挖掘机，此时系统记录真实物理动作。由于不经由 CAN 卡下发流量，默认记录液压推力 `ch3 = 3000`。
   - 自动模式：选择指定 JSON 剧本一键复现，系统自动下发指令并精准记录。
3. **实时 3D 数字孪生与动图留档**：剧本运行期间，界面将弹出挖掘机姿态的实时 3D 可视化（俯视图与侧面切视图）。运行结束后，系统将在后台自动将本次运动轨迹保存为 `*_realtime.gif`，方便数据集验证与溯源。

## 包含的数据模态与存储格式

每次启动录制，系统会在 `src/shandong/data/` 目录下生成一个以 `v11_时间戳` 命名的数据集文件夹（例如 `v11_20260526_183000/`）。
各子目录及文件格式如下：

### 1. 视觉感知 (Vision)
- 存储路径：`cam_net_1/`, `cam_net_2/`, `cam_hikvision/`
- 存储格式：`.jpg` 图片，文件名为采集时的绝对时间戳（如 `16212345.123.jpg`）。
- 机制：由独立守护线程异步拉取 RTSP 流，降帧（~10Hz）并应用 `[cv2.IMWRITE_JPEG_QUALITY, 85]` 轻量级压缩后异步落盘，防止阻塞主控线程。

### 2. 激光雷达点云 (Lidar PointCloud)
- 存储路径：`pointclouds/`
- 存储格式：`.npy` (Numpy 数组二进制文件)，文件名为绝对时间戳（如 `16212345.123.npy`）。
- 格式说明：Nx3 的浮点数组，每一行代表一个点云的 `(X, Y, Z)` 空间物理坐标（单位：米）。
- 机制：抽样保存（默认10帧报文保存1帧），避免磁盘 IO 爆炸，并通过独立线程异步落盘。

### 3. 本体感知与硬件状态 (Proprioception)
- 存储路径：`sensor_states.csv`
- 存储格式：CSV 表格
- 字段说明：
  - `timestamp`: 绝对时间戳
  - `boom_pitch`: 大臂与水平面夹角（基于倾角传感器）
  - `arm_pitch`: 小臂与大臂夹角（基于倾角传感器）
  - `bucket_pitch`: 铲斗与小臂夹角（基于倾角传感器）
  - `swing_yaw`: 挖掘机回转绝对偏航角（基于雷达 IMU 空间积分）
  - `yaw_rate`: 实时回转角速度 (rad/s)

### 4. 控制指令 (Control Commands)
- 存储路径：`control_commands.csv`
- 存储格式：CSV 表格
- 字段说明：
  - `timestamp`: 绝对时间戳
  - `ch1`: 比例阀 1 的推力输出（mV）
  - `ch2`: 比例阀 2 的推力输出（mV）
  - `ch3`: 比例阀主液压输出（mV，遥控模式默认记录 3000）

### 5. 可视化轨迹动图 (GIF)
- 存储路径：录制状态下保存在 `src/shandong/data/v11_时间戳/`，非录制状态下保存在 `src/shandong/json/`
- 存储格式：`.gif`
- 机制：在 JSON 剧本复现时，自动截取真实传感器角度帧生成 3D 俯视/侧视图动画。

## 运行方式

本系统提供了两种运行版本：**ROS2 发布版本**（推荐，支持 RViz 实时查看）和 **纯 Python 本地版本**。

### 方式一：运行 ROS2 版本 (推荐)

此版本会在后台发布相机图像、关节状态和雷达点云到 ROS 网络，支持你通过 `rosbag` 录制或使用 RViz 实时观察环境。

```bash
# 1. 进入工作区并 source ROS2 与当前工程环境 (必须)
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 2. 启动 GUI 与 ROS 节点
python3 src/shandong/v11_multimodal_dataset_collection/ros2_multimodal_gui.py
```

**ROS2 版本特有的话题说明：**
- `/lidar/points` (Frame: `base_link`): 未进行重力与抗旋补偿的原始点云，随车体转动。**这是默认的点云话题**。
- `/lidar/points_odom` (Frame: `odom`): 经过 IMU 抗旋补偿后的点云。当挖掘机转动时，周围环境（树木、墙壁）在 RViz 中会保持静止，适合需要全局静止坐标系的场景。
- `/excavator/joint_states`: 实时发布的挖掘机 4 个关节角度。
- `/camera_hik/image_raw` 等: 实时发布的摄像头图像。

### 方式二：运行纯 Python 本地版本

此版本不依赖 ROS 环境，主要用于快速离线数据集采集（直接存为 `.npy` 和 `.jpg`）。

```bash
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v11_multimodal_dataset_collection
python3 multimodal_gui.py
```

---

### 操作说明
- **【🚀 启动端到端数据采集】**：开启所有传感器落盘（图像、点云、状态）。
- **【📂 选择并执行 JSON 剧本】**（默认路径 `src/shandong/json/`）：一键复现闭环动作。
- **【🔴 开始手动录制】**：传统录制模式。点击后，只有当您在界面上手动点击“开始移动”或“记录当前角度”按钮时，该动作才会被记录入剧本。
- **【🔴 开始自动提取】**：智能录制模式。点击后，您可以自由遥控挖掘机进行挖掘作业。
  - 系统会在后台以 20Hz 的频率记录所有关节的实时角度。
  - 当您点击 **【⏹ 停止自动提取】** 时，系统会使用**滑动窗口方差算法**自动过滤掉您操作过程中的手抖与微小停顿，精准提取出每一个“稳态运动终点”，并自动拼接生成一份 JSON 剧本。
  - 生成后会弹窗提示您将这份剧本保存到本地。

---

## 额外工具：从 ROS Bag 提取 JSON 剧本

如果您在操作时并没有使用 GUI 的实时录制功能，而是使用了 `rosbag record` 录制了全量数据，您可以事后使用提供的独立脚本将录制好的 Bag 文件转换为 JSON 剧本文件。

这个脚本（`bag_to_json.py`）同样内置了防抖与动作提取算法，它会分析 `/excavator/joint_states` 话题，并将运动终点提取出来。

**使用方法：**

```bash
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v11_multimodal_dataset_collection

python3 bag_to_json.py \
  --bag /path/to/your/rosbag_dir \
  --out my_script.json \
  --threshold 1.0 \
  --steady_time 1.0 \
  --min_move 2.0
```

**参数说明：**
- `--bag`: 必填，你录制的 rosbag 目录路径。
- `--out`: 必填，提取后生成的 `.json` 剧本存放路径。
- `--threshold`: 判定为运动的最小角度极差（默认 1.0 度）。
- `--steady_time`: 关节保持平稳多少秒后认为一个动作结束（默认 1.0 秒）。
- `--min_move`: 动作结束时的角度与初始角度的差值，必须大于此值才会被记录，用来过滤操作时的原地轻微抖动（默认 2.0 度）。

生成 JSON 后，您可以使用任意文本编辑器打开它，根据需要手动修改或微调角度（`target_val`）及加减速时间（`ramp_up_s`）。

---

# 6. 新增：V11 ROS Topic 控制桥（独立模块，零修改原程序）

## 6.1 功能定位

为了让 V11 数据集采集系统能够与 **V15_action_task（通用挖掘机控制库）** 以及 **RViz URDF（describe_60FED_calibrated）** 的指令体系完全打通，我们在本目录**新增了 3 个完全独立的文件，不修改 multimodal_gui.py、camera_all.launch.py、setup.py、package.xml 等任何原 V11 程序：**

| 文件 | 作用 |
|---|---|
| [v11_ros_topic_controller.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v11_multimodal_dataset_collection/v11_ros_topic_controller.py) | **控制桥主程序**：独立 ROS 2 Node，订阅 2 条标准控制话题、复用 V4 AngleController 液压闭环、复用 V10 ExcavatorIK 做笛卡尔空间点逆解。 |
| [launch/v11_ros_control_only.launch.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v11_multimodal_dataset_collection/launch/v11_ros_control_only.launch.py) | **Launch A：只启控制桥**（不启相机）。用于 v15 ↔ v11 ↔ RViz URDF 纯指令联调、快速验证 topic 通断。 |
| [launch/v11_cameras_with_ros_control.launch.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v11_multimodal_dataset_collection/launch/v11_cameras_with_ros_control.launch.py) | **Launch B：相机 + 控制桥一并启动**。include 原生 `camera_all.launch.py`（hik/net1/net2 URL 参数原样透传），再追加启动控制桥，用于**同时采集图像+执行 ROS 下发的动作**（端到端数据集生成的典型用法）。 |

## 6.2 设计原则

- **零依赖新增**：3 个文件都是新增，V11 原有程序 1 行不动。未来 V11 GUI / launch 升级不会被 ROS 控制功能污染。
- **指令 100% 对齐 V15**：话题名、关节 4 元素顺序、弧度/米单位、方向语义，**与 V15_main_node、V15_ex16 的指令完全一致**，`ros2 topic pub` 命令复制粘贴即可复用。
- **依赖完全复用 V11 原有**：不新增任何 pip / apt 包；复用 V11 已经 import 的 v4 AngleController（CAN 闭环液压）+ v10 ExcavatorKinematics/IK（运动学）；rclpy 走系统 ROS 2 Humble 默认 Python 3.10。
- **Offline 模式**：`--offline` 时不连接 CAN 串口，只打印下发的目标角度，方便 SSH 无头环境只验证消息格式、联调 V15 的 topic relay，不影响真实液压系统安全。

---

## 6.3 ROS 2 话题接口规范（严格对齐 V15）

### 输入话题（V11 订阅，外部 V15 / RViz GUI / 你自己的节点下发）

| # | 话题名 | 消息类型 | 单位约定 | 格式约束 | 对应 V15 等价话题 |
|---|---|---|---|---|---|
| I1 | **`/state_joint`** | `sensor_msgs/msg/JointState` | position 数组 = **弧度**（rad） | - `name` 固定顺序 = `["swing_joint", "boom_joint", "arm_joint", "bucket_joint"]`（推荐写法，最稳妥）<br>- 或 `name` 空/省略，但 `position` 长度=4 → 按上述顺序解析（Fallback 兼容 rospub 简写格式）<br>- `header.frame_id` = `base_link`（建议写，用于 TF 对齐） | `/v15/cmd/joint_states`（直接 `ros2 run topic_tools relay` 即可桥接） |
| I2 | **`/target_point`** | `geometry_msgs/msg/PointStamped` | point (x,y,z) = **米**（m） | - `header.frame_id = "base_link"`（必须写正确，空字符串会被直接忽略）<br>- x=前（挖掘机前方 +X）、y=左（+Y）、z=上（+Z）<br>- bucket_guess_deg 默认=-45°（挖掘姿态下的铲斗绝对倾角），启动时可用参数 `--default-bucket-guess-deg` 覆盖 | `/v15/cmd/target_point`（直接 relay 即可桥接） |

### 输出话题（V11 发布，调试/闭环反馈用，非必须订阅）

| # | 话题名 | 消息类型 | 说明 |
|---|---|---|---|
| O1 | `/v11/current_state` | `sensor_msgs/msg/JointState` | 最近一次下发到 v4 AngleController 的**目标角度（度）**。用于「不连真车 offline 时」也能通过 topic echo 看到下发值是否按预期变化。注意：这里是**度**不是弧度，仅用于调试，**不是标准物理反馈**。真实车反馈请接 V11 自带的 4 路 WT901C485 倾角 → `/excavator/joint_states`。 |

### 关节映射 + 方向语义（与 V15 / describe_60FED_calibrated URDF 完全一致）

| URDF JointState name（输入 I1 的 name） | position[i] 对应语义（度/弧度均遵循） | 正方向语义 | 负方向语义 |
|---|---|---|---|
| `swing_joint` (position[0]) | `swing_yaw` 回转偏航 | **+rad / +deg → 从上往下看顺时针 (CW)** | -rad / -deg → 逆时针（CCW） |
| `boom_joint` (position[1]) | `boom_swing` 大臂-回转基座夹角 | **+rad / +deg → 大臂向下压** | -rad / -deg → 大臂向上抬 |
| `arm_joint` (position[2]) | `arm_boom` 小臂-大臂夹角 | **+rad / +deg → 小臂向内收** | -rad / -deg → 小臂向外推 |
| `bucket_joint` (position[3]) | `bucket_arm` 铲斗-小臂夹角 | **+rad / +deg → 铲斗向内卷铲（装料）** | -rad / -deg → 铲斗向外打开（卸料） |

> 💡 **方向核对锚点**：`ros2 topic pub` 发 `position=[+0.5236, 0.0873, 1.0472, 0.1745]`（对应 [+30°, +5°, +60°, +10°]）→ 真车/URDF 必须是「顺时针转 30°、大臂压 5°、小臂收 60°、铲斗卷 10°」。若方向相反，只需要在 `v11_ros_topic_controller.py` 的 `_dispatch_one_joint` 里在对应 sem 处加一个负号即可（不影响其他逻辑）。

---

# 7. 标准启动步骤 & 3 终端验证 Checklist

## 7.0 启动前置（必须先做 2 件事）

1. **Python 解释器切到 ROS Humble 配套的系统 Python 3.10**（千万不要用 conda/miniconda，否则 `import rclpy` 直接找不到 `_rclpy_pybind11.cpython-310`）：
   ```bash
   export PATH="/usr/bin:$PATH"; hash -r
   python3 --version    # 期望输出：Python 3.10.x
   which python3        # 期望输出：/usr/bin/python3
   ```
2. **先一键杀死所有遗留 ROS / v11 / v15 进程**（避免双节点静默冲突 —— 「topic 收到回调但值不变」的头号凶手）：
   ```bash
   pkill -9 -f "v11_ros_topic_controller" ; sleep 0.3
   pkill -9 -f "v15_" ; sleep 0.3
   pkill -9 -f "rviz2" ; pkill -9 -f "robot_state_publisher" ; pkill -9 -f "joint_state_publisher" ; sleep 0.3
   ps -ef | grep -E "v11|v15_|rviz|robot_state|joint_state" | grep -v grep
   # ✅ 期望：最后一行输出为空（干净）
   ```

## 7.1 用法 A1：独立脚本 offline 联调（最快，推荐先用这个）

不启 launch、不启相机、不连 CAN，只用独立脚本验证「你发的 ROS topic → V11 能解析 → 目标角度计算对不对」。**SSH 无头环境下必过的第一关。**

```bash
# ── 终端 1（控制桥）────────────────────────────────────────
source /opt/ros/humble/setup.bash
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v11_multimodal_dataset_collection
export PATH="/usr/bin:$PATH"; hash -r
python3 v11_ros_topic_controller.py --offline
# ✅ 启动锚点（出现这 3 行就进入 OK 状态）：
#   [v11_bridge] offline 模式：不连接 CAN 串口，只打印日志
#   [v11_bridge] 启动完成: offline=True  hw=N/A  ik=OK
#   [v11_bridge] 订阅 2 条（与 v15 指令完全兼容）：① /state_joint ... ② /target_point ...
```

```bash
# ── 终端 2（你自己的控制端：rospub / V15 / GUI）─────────────
source /opt/ros/humble/setup.bash

# ── 验证 I1 state_joint（弧度 4 关节 → 离线日志应该精准打印 30°、-10°、60°、10°）
ros2 topic pub -1 /state_joint sensor_msgs/msg/JointState \
  "{header: {frame_id: base_link},
    name: ['swing_joint','boom_joint','arm_joint','bucket_joint'],
    position: [0.5236, -0.1745, 1.0472, 0.1745]}"
# ✅ 终端 1 立刻出现 4 行：[v11_bridge][offline] -> swing_yaw = 30.00 deg / boom_swing = -10.00 / arm_boom = 60.00 / bucket_arm = 10.00

# ── 验证 I2 target_point（x=1.2m 左偏 y=0.3m 高 z=0.5m → swing 必须≈atan(0.3/1.2)=14.04° 这是校验 IK 的锚点）
ros2 topic pub -1 /target_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: base_link},
    point: {x: 1.2, y: 0.3, z: 0.5}}"
# ✅ 终端 1 立刻出现：[v11_bridge][/target_point] ... IK OK  下发 → sw= 14.04° ...
```

**Offline 模式 3 项必过 Checklist**：
- [ ] 发 `/state_joint` → 4 个离线日志角度和你弧度→度换算值误差 ≤0.01°
- [ ] 发 (1.2, 0.3, 0.5) `/target_point` → swing ≈ 14.04°
- [ ] 发 (1.2, 0.0, 0.8) `/target_point` → swing ≈ 0.00°，IK solve 不打印"无解"

## 7.2 用法 A2：Launch A — 只启 ROS 控制桥（与 RViz URDF 联调）

想和 describe_60FED_calibrated.urdf 的 RViz 联动，但不启相机时用：

```bash
source /opt/ros/humble/setup.bash
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
source install/setup.bash
ros2 launch v11_multimodal_dataset_collection v11_ros_control_only.launch.py \
    offline:=true \
    default_bucket_guess_deg:=-45.0
```

## 7.3 用法 A3：Launch B — 相机 + 控制桥（端到端数据集生产）

这是「**一边 ROS 指令下发动作、一边 3 路 RTSP 图像落盘、严格时间戳对齐**」的标准生产模式：

```bash
source /opt/ros/humble/setup.bash
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
source install/setup.bash

# 真车生产（offline=false → 真连 CAN 串口，下发液压指令）
ros2 launch v11_multimodal_dataset_collection v11_cameras_with_ros_control.launch.py \
    offline:=false \
    can_port:=/dev/ttyUSB_Controller \
    can_baud:=115200 \
    default_bucket_guess_deg:=-60.0

# ★ 只想先验证 topic 通断、不动液压：把 offline 改回 true 即可，其它参数不变
```

**相机 RTSP URL 覆盖**（camera_all.launch.py 的 hik_url / net1_url 直接透传）：
```bash
ros2 launch v11_multimodal_dataset_collection v11_cameras_with_ros_control.launch.py \
    offline:=true \
    hik_url:="rtsp://admin:YourPwd@192.168.1.101:554/Streaming/Channels/101" \
    net1_url:="rtsp://admin:YourPwd@192.168.1.102:554/stream"
```

## 7.4 真车启动 Checklist（Offline 过了再开 offline=false）

- [ ] 终端 1 启动后 `CAN 控制串口` /v4 AngleController 打印 `CAN=OK`（否则会自动降级 offline）
- [ ] `/dev/ttyUSB_Controller` 权限是 `dialout` 用户组可用（`sudo chmod 666 /dev/ttyUSB_Controller` 先兜底验证）
- [ ] 手动发一个小角度（swing +5° / boom -3°）→ 真车液压响应 ≤500ms 开始动作
- [ ] `ros2 topic echo /v11/current_state` → position 数值变化时间戳跟随你 rospub 时间戳（证明下发链路真实生效）

---

# 8. V15 双向对接 & 指令对照表（零代码改动，只用 topic_tools relay）

## 8.1 背景

V15_action_task（通用控制库）暴露的控制话题是 `/v15/cmd/joint_states`、`/v15/cmd/target_point`；V11 桥监听的是 `/state_joint`、`/target_point`。**两边指令格式、关节顺序、单位、方向 100% 一致，只有话题名不同** → 用 `ros2 run topic_tools relay` 两条命令直接桥通，不需要写任何额外代码。

## 8.2 标准联调 4 终端拓扑

```
   ┌────────────────────────────────────────────┐
   │ 终端 A：RViz URDF + robot_state_publisher │
   │   ros2 launch describe_60FED display...   │
   └────────────────────┬───────────────────────┘
                        │/joint_states ← pub
   ┌────────────────────▼───────────────────────┐
   │ 终端 B：V15_main_node                     │
   │ pub →  /v15/cmd/joint_states              │
   │ pub →  /v15/cmd/target_point              │
   └────────────────────┬───────────────────────┘
                        │ relay + relay
   ┌────────────────────▼───────────────────────┐
   │ 终端 C：topic_tools relay（两行即可）      │
   │   relay /v15/cmd/joint_states → /state_joint
   │   relay /v15/cmd/target_point → /target_point
   └────────────────────┬───────────────────────┘
                        │
   ┌────────────────────▼───────────────────────┐
   │ 终端 D：V11 ROS 控制桥（offline/真车）     │
   │  sub →  /state_joint                       │
   │  sub →  /target_point                      │
   │  → V4 AngleController 液压闭环 真车执行     │
   └────────────────────────────────────────────┘
```

终端 C 的两条 relay 命令：
```bash
source /opt/ros/humble/setup.bash
ros2 run topic_tools relay /v15/cmd/joint_states /state_joint &
ros2 run topic_tools relay /v15/cmd/target_point  /target_point  &
# 现在你在 V15 那边发的所有单点、手动关节指令，V11 全按同样的数值和语义执行
```

## 8.3 指令对照表（复制粘贴级复用）

| 想做的事 | V15 单测命令（原封不动） | V11 单测命令（V15 没启动时直接用） |
|---|---|---|
| 回转 +30°（CW）<br>大臂抬 -10°<br>小臂收 60°<br>铲斗卷 10° | `ros2 topic pub -1 /v15/cmd/joint_states ...`（V15 文档里你已经在用的那条） | `ros2 topic pub -1 /state_joint sensor_msgs/msg/JointState "{... name:['swing_joint','boom_joint','arm_joint','bucket_joint'], position:[0.5236,-0.1745,1.0472,0.1745]}"`（和左侧那条除了话题名完全一致） |
| 铲尖移动到 前 1.2m / 高 0.8m 的笛卡尔空间点（挖掘机正前方平面内） | `ros2 topic pub -1 /v15/cmd/target_point ... point:{x:1.2,y:0.0,z:0.8}` | `ros2 topic pub -1 /target_point geometry_msgs/msg/PointStamped "{header:{frame_id:base_link}, point:{x:1.2,y:0.0,z:0.8}}"`（同样是 frame_id=base_link、单位米） |
| 铲尖移动到 前 0.9m / 偏左 0.3m / 低 0.3m 的空间点 | V15 同 | 同上改数值 → V11 会自动算出 swing≈18.43° |
| 不可达 3.0m 远点 → 验证安全边界（不动作） | V15 同 → 由 V15 WorkspaceChecker 拦截 | V11 会打印 `IK 无解` 且不调用 AngleController 下发（不动真车） |

---

# 9. 常见问题 FAQ（调试优先按顺序排查）

## Q1. `ros2 topic pub -1` 你看到它 publish #1 了，但 V11 终端没有任何「[cmd/recv] 或 [/target_point] 收到」的日志？

这是**你以为发了但订阅回调没被触发**的典型静默症状（90% 用户会踩）。按顺序跑这 5 条 10 秒定位：
```bash
# ① 先确认「谁在订阅 /state_joint」？必须 Subscriber count=1，节点是 v11_ros_topic_controller
ros2 topic info /state_joint -v

# ② 再确认 V11 进程活着？
ps -ef | grep v11_ros_topic | grep -v grep

# ③ 确认没有两个 v11_ros_topic_controller（双节点抢 DDS！头号凶手）
ros2 node list | grep v11     # 期望：只有一行 /v11_ros_topic_controller

# ④ 你发的 frame_id=base_link 了吗？（空字符串会被 PointStamped 默默丢弃）
ros2 topic echo /state_joint --once | head -n 15

# ⑤ 终极验证：用 ros2 topic pub --rate 10 连续发 10Hz 冲一下，看是不是 -1 one-shot 被 DDS 可靠 qos 丢了
ros2 topic pub --rate 10 -t 3 /state_joint sensor_msgs/msg/JointState \
  "{header: {frame_id: base_link},
    name: ['swing_joint','boom_joint','arm_joint','bucket_joint'],
    position: [0.1745, 0.0, 1.0472, 0.1745]}"
```

## Q2. V11 日志打印了目标角度，但真车 URDF / 液压系统不动？

90% 是「offline 忘记改成 false」或「CAN 串口权限/波特率不匹配」。
```bash
# 先看 v11 启动日志首 10 行：期望 hw=OK offline=false
grep -E "启动完成|CAN|offline" /tmp/v11_bridge.log 2>/dev/null
# - hw=OK + offline=false → 说明 V4 连了，不动是液压侧机械安全锁未解除
# - hw=N/A + offline=true → 忘记关 offline 了，重开 launch 加 offline:=false
```

## Q3. 角度数值正确（日志打印 30°），但真车动的是相反方向？

V10 / V4 的历史角度符号定义 + describe_60FED_calibrated.urdf 约定都对齐了标准方向语义，但如果你的 WT901C485 倾角 4 路 CAN 地址映射或安装方向与默认机型不同，**只需要在 `v11_ros_topic_controller.py._dispatch_one_joint()` 里对应 sem 加一行正负号取反即可**：
```python
# 例：某台特定机型的 swing 恰好要反向：
if sem == "swing_yaw":
    target_deg = -float(target_deg)
```
其他 3 个关节同理。**不要去改动 V4 AngleController 或 V10 IK 的内部符号**，否则会影响 JSON 剧本复现和方向核对锚点测试。

## Q4. bag_to_json.py 生成的动作剧本，和 ROS topic 控制桥能一起用吗？

完全可以。两种用法互补：
- **数据采集阶段（V11 原生闭环 + 4 路 IMU + 3 路相机）**：用 bag_to_json.py 把 ROSBag 里 `/excavator/joint_states`（真车物理反馈）提取成 JSON 剧本 — 这是「示教」。
- **复现/批量生成阶段**：把 JSON 剧本读出来，再由你自己的调度节点逐段计算成 `/state_joint`（弧度）或 `/target_point`（米）下发给 V11 控制桥 — 这是「V11 硬件执行」+ V11 相机同步落盘。
- 如此，**示教用 GUI + 真车反馈，复用用 ROS 指令 + 控制桥**，两边完全解耦，V11 原有的数据采集流程不会被破坏。

## Q5. 双节点冲突（ros2 node list 里出现两个相同的 /v15_urdf_controller 或 /v11_ros_topic_controller）怎么办？

立刻执行「7.0 启动前置」的 `pkill -9` 清理命令，然后重开所有终端；如果反复出现，请检查：
- 有没有在两个终端里分别 launch 了两次；
- 有没有使用 `ros2 launch` + 同时手动 `python3 v11_ros_topic_controller.py` 并行启动；
- SSH 连接重连后旧进程未被杀的情况（建议 `tmux` 或 `screen` 管理常驻节点）。