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