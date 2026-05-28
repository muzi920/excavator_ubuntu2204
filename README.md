# 挖掘机控制与多传感器采集系统

本仓库用于模型挖掘机的底层控制、时间脚本控制、倾角传感器采集、闭环角度控制、M300 雷达接入、多路摄像头读取、ROS 2 坐标系标定与端到端多模态数据集采集。

当前仓库已经从早期的 `v1`、`v2`、`v3` 顺序试验目录，整理为按功能命名的目录结构，便于长期维护和现场调试。

## 项目概览
- `v1_control_base`：中盛 CAN 控制板底层控制与点动 GUI
- `v2_control_time_track`：基于时间的动作调度与剧本控制
- `v3_sensor_read_wit`：WIT 倾角传感器读取、串口识别与 ROS 2 发布
- `v4_control_closed`：基于倾角反馈的闭环角度控制与 JSON 剧本生成
- `v5_sensor_read_lidar`：M300 雷达读取、点云与 IMU 处理、TF 标定
- `v6_sensor_read_camera`：USB/RTSP 摄像头读取与 ROS 2 图像发布
- `v7_lerobot_dataset`：基于 ROS 2 的 LeRobot 数据集采集测试
- `v8_direct_data_collection`：纯 Python 底层无延迟直连 LeRobot 采集架构
- `v9_nav`：Navigation2 路径规划与导航
- `v10_cailbration`：正运动学、3D 可视化、雷达 IMU 回转偏航角融合
- `v11_multimodal_dataset_collection`：**最新主推版本**，提供端到端数据采集、剧本一键复现、实时 3D 轨迹留档。
- `launch`：多传感器启动脚本与静态 TF 统一管理
- `json`：所有自动化控制 JSON 剧本的统一存放目录。
- `data`：所有多模态数据集（视觉、点云、状态、指令、动图）的统一归档目录。

## 推荐调试流程
1. 先在 `v1_control_base` 确认底盘、大臂、小臂、铲斗、回转的底层 CAN 控制正常。
2. 在 `v3_sensor_read_wit` 中完成倾角传感器串口识别、角度读取与极限工况测量。
3. 根据测得的关节活动范围，在 `v4_control_closed` 中进行闭环角度控制调试，并录制/生成自动挖掘的 JSON 剧本。
4. 在 `v5_sensor_read_lidar` 中完成 M300 雷达接入、倒装修正与 `map -> base_link` 标定。
5. 在 `v6_sensor_read_camera` 中接入海康或普通网络摄像头，并在 `launch/sensors_tf.launch.py` 中维护相机相对于 `base_link` 的静态外参。
6. 在 `v11_multimodal_dataset_collection` 中，使用综合 GUI 执行自动化闭环剧本，同时记录高频对齐的多模态数据集。
7. 最后通过 `launch` 中的脚本统一启动感知链路（如需 ROS 2 支持）。

## 核心目录详细说明

### `v11_multimodal_dataset_collection`：端到端多模态数据集采集 (最新核心)
这是整个工程的集大成者，融合了底层控制、闭环寻的、雷达直连、视频流拉取和运动学可视化。
- 提供一键启动的多模态数据录制（支持遥控采集与剧本自动复现采集）。
- 严格基于时间戳（`time.time()`）对齐所有模态数据。
- 剧本复现时，支持实时双视图 3D 可视化，并在执行完毕后自动生成 `.gif` 动图归档至数据集中。

### `v10_cailbration`：正运动学与 3D 可视化
- `kinematics.py`：挖掘机正向运动学核心模型，计算大臂、小臂、铲斗 2D 坐标。
- `animate_trajectory_3d.py`：基于正运动学模型和雷达回转数据，将 JSON 剧本或实时运行轨迹渲染成直观的 3D 动图。

### `v9_nav`：Navigation2 路径规划与导航
该目录用于接入 ROS 2 Nav2 导航栈，实现挖掘机底盘的自主移动与路径规划。

### `v1_control_base`：底层控制与实时交互
该目录直接面向中盛 ZS-USB-CAN 转接板和继电器/模拟量控制模块。

- `zs_excavator_controller.py`
  - 封装底层 13 字节协议、握手、CAN ID 编码和动作语义接口。
  - 提供底盘行走、大臂、小臂、铲斗、回转以及模拟量通道控制。
- `zs_excavator_gui.py`
  - 基于 `Tkinter` 的实时点动控制界面。
  - 支持滑条调整模拟量、键盘快捷键控制和急停。
- `zs_excavator_controller.cpp`
  - 与 Python 控制逻辑对应的 C++ 版本，便于 Windows 侧联调。

启动方式：

```bash
python3 v1_control_base/zs_excavator_gui.py
```

### `v2_control_time_track`：时间调度与动作剧本
该目录在底层控制之上增加时间维度，用于实现连续动作编排。

- `action_scheduler.py`
  - 负责动作定时执行、自动停止和多动作串联。
  - 适合编写“抬臂 1.2s -> 收斗 0.8s -> 回转 1.5s”这类时间脚本。
- `action_gui.py`
  - 图形界面版本的时间控制工具。
  - 可做单步测试，也可按预设顺序逐步执行一整套动作流程。

启动方式：

```bash
python3 v2_control_time_track/action_gui.py
```

### `v3_sensor_read_wit`：倾角传感器采集与 ROS 2 发布
该目录主要处理 WIT-Motion 系列倾角传感器的数据读取、串口识别和相对角度计算。

- `readRad_ubuntu.py`
  - Ubuntu 下的传感器读取主脚本。
  - 支持多串口轮询、根据传感器 ID 区分关节，并适配现场的固定软链接串口方案。
- `ros2_readRad_pub.py`
  - 将倾角数据发布为 ROS 2 Topic。
  - 保留原始角度/加速度数据，同时额外发布关节相对角度。
- `sensor_port_mapper_gui.py`
  - 串口识别工具，用于排查 USB 重新插拔后端口号变化的问题。
- `sensor_action_gui.py`
  - 动作控制与传感器实时监控结合的 GUI。
  - 可记录极限状态并计算各关节活动范围。

当前约定的关节传感器 ID：
- `0x50`：铲斗
- `0x51`：小臂
- `0x52`：大臂
- `0x53`：回转参考

启动方式示例：

```bash
python3 v3_sensor_read_wit/readRad_ubuntu.py
python3 v3_sensor_read_wit/ros2_readRad_pub.py
```

### `v4_control_closed`：闭环角度控制
该目录将底层控制与倾角反馈结合，实现机械臂关节的自动寻的，并负责控制剧本的生成与解析。

- `angle_controller.py`
  - 闭环控制核心。
  - 根据目标角度和当前反馈角度决定运动方向，并在进入容差或越过目标时主动刹停。
  - 已加入提前量补偿、符号翻转防越界和不同关节独立停止指令适配。
- `closed_loop_gui.py`
  - 闭环控制调试界面。
  - 支持输入目标角度并观察各关节实时状态。
- `generate_*.py`
  - 各类自动化剧本生成工具（如 30 轮扇形扫掠挖掘），生成的 `.json` 文件将统一保存在根目录的 `json/` 文件夹下。

当前控制逻辑中的关键经验：
- 铲斗、小臂、大臂的相对角度控制已适配传感器安装方向反向问题。
- 回转已从倾角闭环改为按时间控制，因为当前传感器无法直接提供可靠的 Z 轴回转角反馈。

启动方式：

```bash
python3 v4_control_closed/closed_loop_gui.py
```

### `v5_sensor_read_lidar`：M300 雷达接入与 TF 标定
该目录用于 M300 雷达驱动测试、点云读取与坐标系标定。完全抛弃 ROS2 机制，通过 Python直接监听雷达 UDP 端口（6543/6668）。

- `m300-main`
  - 官方驱动和 SDK 源码。
- `lidar_direct_reader.py`、`lidar_viewer.py`
  - 点云读取与独立测试脚本。
- `imu_direct_swing_estimator.py`
  - 解析高频 IMU 报文，结合静止加速度计进行 3D 空间重力投影，完美解决雷达非水平安装导致的回转误差。
- `tf_calibration_gui.py`
  - 雷达 TF 可视化标定工具。
  - 用于动态调整 `map -> base_link` 的平移和姿态参数。
- `tf_calibration_record.txt`
  - 标定记录文件，可将最终参数复制到 `launch/sensors_tf.launch.py`。
- `sensor_calibration_guide.md`
  - 雷达与多传感器坐标系整理说明。

这里有两个重要约定：
- 雷达点云的 `frame_id` 保持为 `map`，不直接改驱动内部输出。如果需要在 `base_link` 坐标系下使用，请使用 `pointcloud_transformer.py` 进行动态转换。
- 回转角度的获取：由于雷达未发布直接四元数，当前通过订阅雷达的 `/imu` 角速度并在 Python 节点中进行**动态零偏校准**与**梯形积分**获取。
- 为避免 Rviz2 中出现 `timestamp earlier than all the data in the transform cache`，标定和最终使用阶段以静态 TF 为主。

### `v6_sensor_read_camera`：多摄像头读取与图像发布
该目录负责 USB 摄像头、海康摄像头和普通 RTSP 网络摄像头的接入。

- `read_hikvision_cam.py`
  - 海康 RTSP 拉流测试。
- `read_network_cam.py`
  - 普通网络摄像头拉流测试。
- `read_usb_cam.py`
  - USB 摄像头测试。
- `ros2_hikvision_cam_pub.py`、`ros2_network_cam_pub.py`、`ros2_usb_cam_pub.py`
  - 单路图像 ROS 2 发布节点。
- `ros2_net_cams_pub.py`
  - 多路网络摄像头联合发布节点。
- `ros2_all_cams_pub.py`
  - 综合相机发布节点。

已处理的现场问题：
- RTSP 拉流时通过 OpenCV/FFmpeg 参数优化，降低卡死、花屏和高延迟问题。
- 在 `launch/sensors_tf.launch.py` 中维护普通网络摄像头 `.102`、`.103` 以及海康相机相对于 `base_link` 的静态外参。

注意：
- 如果希望在 Rviz2 中把图像按相机视角投影到 3D 场景，而不是只看 2D 图像窗口，除了发布 `Image` 之外，还需要同步发布 `CameraInfo`。

### `v7_lerobot_dataset`：基于 ROS 2 的 LeRobot 数据集采集测试
该目录是构建模仿学习（Imitation Learning）数据集的早期版本，主要基于 ROS 2 话题进行订阅和同步。
- `lerobot_data_collector.py`
  - 使用 HuggingFace `LeRobotDataset` API 创建本地数据集。
  - 通过 ROS 2 订阅图像、角度等话题。
  - 已废弃：因发现 ROS 2 消息排队带来的通信延迟过高，无法满足精确控制对齐要求。

### `v8_direct_data_collection`：纯 Python 底层直连采集架构 (推荐)
该目录彻底抛弃 ROS 2 消息传递机制，直接通过 Python 底层协议读取硬件，实现**零延迟多模态数据采集**，完美对齐 `LeRobot`。

- `inclinometer_reader.py`
  - 使用独立线程和 Modbus RTU 协议直接轮询 WIT 倾角传感器，解析原始串口十六进制数据，实时计算各关节相对角。
- `camera_reader.py`
  - 使用独立守护线程和 `cv2.VideoCapture.grab()` 机制强制清空 FFmpeg RTSP 流缓冲，确保主线程每次拉取都是最新的一帧画面，消灭拉流延迟。
- `lidar_reader.py`
  - 纯 Python 编写的雷达 UDP 协议（端口 6668）解包工具。
  - **IMU 处理**：解析 `0xFA 0x88` 包头，在后台 200Hz 高频实现去死区、零偏校准和回转梯形积分。
  - **点云处理**：解析位域数据并使用 `numpy` 矩阵运算，实现 `map -> base_link` 齐次矩阵逆变换，并滤除超出 `(-5m, 5m)` 范围的无效点云。
- `lerobot_direct_collector.py`
  - 采集入口。主循环严格卡死在 10Hz，每隔 100ms 通过线程锁 `lock` 瞬间抓取所有硬件模块的**最新鲜数据**。
  - 将图像、点云 (padding 至 15000)、角度状态压入 LeRobot 数据集，并通过终端命令支持 `start/stop` 分段录制。

启动方式：

```bash
python3 v8_direct_data_collection/lerobot_direct_collector.py
```

### `launch`：统一启动与坐标系管理
该目录是 ROS 2 启动入口，负责把雷达、相机、IMU/倾角传感器与静态 TF 整理到统一坐标树中。

- `sensors_tf.launch.py`
  - 维护 `map -> base_link -> camera_frame/...` 的静态 TF 关系。
  - 当前已包含 M300 雷达、海康摄像头、普通网络摄像头的标定参数。
- `all_sensors.launch.py`
  - 用于统一启动多传感器节点。

TF 结构约定：

```text
map
└── base_link
    ├── network_cam_frame
    ├── network_cam2_frame
    └── hikvision_cam_frame
```

## 2. V11 多模态数据集采集架构演进 (ROS 2)

在 V11 版本中，系统架构已全面演进为 **基于 ROS 2 标准 Topic 的发布-订阅模式**。Python 核心采集脚本（`v11_multimodal_dataset_collection/ros2_multimodal_gui.py`）不再直接执行高频写盘操作，而是将所有传感器数据打上系统绝对时间戳后封装为标准的 ROS 2 消息实时发布，用户需利用 `rosbag2` 统一录制。

**当前可用的话题 (Topics) 列表及详情**：

| Topic 名称 | 消息类型 (ROS 2) | 发布频率 | 包含数据与详情说明 |
| :--- | :--- | :--- | :--- |
| `/camera_hik/image_raw` | `sensor_msgs/Image` | ~10Hz | 海康相机（主视）实时画面流。BGR8 编码。 |
| `/camera1/image_raw` | `sensor_msgs/Image` | ~10Hz | 网络相机 1 实时画面流。BGR8 编码。 |
| `/camera2/image_raw` | `sensor_msgs/Image` | ~10Hz | 网络相机 2 实时画面流。BGR8 编码。 |
| `/lidar/points` | `sensor_msgs/PointCloud2` | 10Hz | 聚合后的三维激光雷达点云数据。包含解析完成的 `x`, `y`, `z` 浮点坐标。 |
| `/excavator/joint_states` | `sensor_msgs/JointState` | 20Hz | 挖掘机本体四个关节的实时绝对物理夹角，单位为 **弧度(Radians)**。<br>数组 `position` 的排列顺序为：<br>`[0] boom_joint`: 大臂与回转夹角<br>`[1] arm_joint`: 小臂与大臂夹角<br>`[2] bucket_joint`: 铲斗与小臂夹角<br>`[3] swing_joint`: 绝对偏航角（由雷达IMU解算） |

**如何录制数据（一键 Bag）：**
可通过运行 `launch/launch_gui.py` 打开简易面板，一键启动 `ros2 bag record` 录制以上所有话题，录制完毕后的包默认保存在 `src/bag/` 目录下。

---

## 3. 常见问题排查 (Troubleshooting)

### Q1: 关闭脚本后再次启动，提示串口被占用？
这是因为上一次关闭时，Python 脚本或 ROS 2 Launch 节点没有被彻底释放。
**解决办法**: 运行以下命令强制杀掉残留的 Python 进程，释放 `/dev/ttyUSB*`：
```bash
killall -9 python3
```

### Q2: 运行 `ros2 topic list` 发现有很多旧话题（如 `/imu/arm_ang_x`）残留在列表里？
即使你杀死了所有节点，ROS 2 底层的 FastDDS 守护进程依然会缓存这些话题的路由信息，导致你看到所谓的“幽灵话题”。
**解决办法**: 在你当前查看 topic 的终端中，执行以下命令重启守护进程清理缓存：
```bash
ros2 daemon stop
ros2 daemon start
```

### Q3: 使用 `launch_gui.py` 时，按了停止却总是自动重启？
由于 ROS 2 Launch 的进程保护和 Respawn 机制，单次 `Ctrl+C` 往往只会被拦截。在 V11 的 `launch_gui.py` 中我们已经通过**连续发送 SIGINT 和 SIGTERM 信号**彻底解决了这个问题，确保子节点完全退出。

---

## 4. 与现场部署相关的补充说明

### 1. 串口稳定绑定
由于多个 USB 转串口设备会在拔插后造成 `/dev/ttyUSB*` 乱序，本项目已经实践过两种思路：

- 传感器脚本内部按 Modbus ID 轮询识别
- 使用 `udev` 根据物理 USB 端口拓扑绑定固定设备名

现场长期使用时，推荐优先采用 `udev` 固定软链接。

### 2. 相对角度计算
当前相对角度主要关注绕 X 轴的关节关系，典型定义为：
- 铲斗角 = 铲斗相对小臂
- 小臂角 = 小臂相对大臂
- 大臂角 = 大臂相对回转参考

这些相对值在 ROS 2 话题中已与原始角度分开发布，便于后续控制和记录。

### 3. Rviz2 显示原则
- 点云能否从 `base_link` 视角正常看到，关键在于 `map` 与 `base_link` 的 TF 是否正确连接。
- 普通图像若要参与 3D 投影，需要 `Image + CameraInfo + 正确的 camera frame TF` 三者同时存在。

## 快速启动参考

### 启动端到端采集与闭环测试 (主推)
```bash
python3 v11_multimodal_dataset_collection/multimodal_gui.py
```

### 仅测试底层点动控制
```bash
python3 v1_control_base/zs_excavator_gui.py
```

### 仅测试倾角传感器读取
```bash
python3 v3_sensor_read_wit/readRad_ubuntu.py
```

### 启动闭环控制界面
```bash
python3 v4_control_closed/closed_loop_gui.py
```

### 启动雷达与多传感器 TF
```bash
ros2 launch shandong launch/sensors_tf.launch.py
```

## 环境依赖

常用 Python 依赖：

```bash
pip install pyserial opencv-python numpy matplotlib
```

可能会用到的系统或 Python 组件包括：
- `pyserial` (用于传感器串口和 CAN 通信)
- `opencv-python` (用于拉取 RTSP 流和保存图像)
- `numpy` (用于雷达点云矩阵运算与保存)
- `matplotlib` (用于 3D 实时可视化与生成 GIF)
- `tkinter` (用于 GUI 界面)
- `rclpy`
- `tf2_ros`
- `sensor_msgs`
- `geometry_msgs`

## 相关补充文档
- `camera_direct_connect_ubuntu.md`：摄像头直连测试说明
- `switch_network_setup.md`：网络交换机与 IP 环境配置
- `device_network_inventory.md`：设备网络信息记录
- `readme_ubuntu.md`：Ubuntu 环境下的补充说明

## 说明
- 当前 Git 仓库根目录位于本目录，即 `shandong_ws/src/shandong`。
- 部分脚本保留了历史试验路径和演进痕迹，使用时建议优先参考各功能目录下最新的 `README.md`。
