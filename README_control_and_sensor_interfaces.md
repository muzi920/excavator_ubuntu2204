# v1-v14 控制接口与传感器接入接口说明

这份文档专门回答两个问题：

1. `v1-v14` 每个版本的控制接口是什么，怎么使用。
2. `v1-v14` 每个版本的传感器接入接口是什么，怎么使用。

你后续如果要统一控制接口，这份文档可以直接作为接口梳理底稿。  
你后续如果要把传感器接口逐步替换成 C++ 版本，这份文档也会明确哪些版本已经接近这种
架构，哪些版本还停留在 Python 直连或 ROS 订阅阶段。

## 总体结论

从项目演进来看，`src/shandong` 里的接口可以粗分成三条主线：

1. **控制接口主线**
   - `v1 -> v2 -> v4 -> v12 -> v13 -> v14`
2. **传感器接入主线**
   - `v3 -> v5 -> v6 -> v11 -> v12 -> v13`
3. **几何与规划主线**
   - `v10 -> v14`

如果你后续要“统一控制接口”，建议把控制层统一成“关节语义 + 目标角 + 执行状态”。

如果你后续要“统一传感器接口，并改成 C++ 版本”，建议把传感器层统一成“标准 ROS 2
Topic + 标准消息结构”，而把底层串口、UDP、相机拉流都收敛进 C++ 节点。

## 推荐统一接口目标

### 控制接口统一目标

建议后续统一为一套抽象控制接口：

```text
输入:
  joint_name
  target_value
  control_mode
  tolerance
  speed / ramp / hydraulic

输出:
  current_value
  error
  is_reached
  status
```

对于挖掘机当前工程语义，最合适的统一关节名就是：

```text
boom_swing
arm_boom
bucket_arm
swing_yaw
```

这是当前 `v4`、`v10`、`v12`、`v14` 已经事实统一的关节控制语义。

### 传感器接口统一目标

建议后续统一为一套 ROS 2 话题接口：

```text
/excavator/inclinometer/raw
/excavator/inclinometer/group
/excavator/lidar/points
/excavator/lidar/imu
/excavator/joint_states
/excavator/camera/<name>/image_raw
```

也就是：

- 串口倾角传感器统一成一类节点
- 雷达点云和雷达 IMU 统一成一类节点
- 相机统一成一类节点
- 机械臂关节状态统一成一类节点

这正好符合你后续“传感器接口改成 C++ 版本”的目标，因为：

- 倾角串口读取适合 C++ 节点长期运行
- 雷达 UDP 解包与点云变换更适合 C++
- RTSP/USB 相机采集更适合 C++
- 上层 Python 可以只保留 GUI、轨迹规划和实验逻辑

## 接口分层建议

后续如果正式统一，建议按下面四层拆分：

1. **驱动层**
   - 串口、CAN、UDP、RTSP、USB
2. **传感器节点层**
   - 倾角、雷达 IMU、点云、图像
3. **控制执行层**
   - 接收目标角或 JSON 命令，输出执行状态
4. **规划与仿真层**
   - IK、任务规划、URDF、RViz、数据采集

其中：

- 驱动层和传感器节点层优先改为 C++
- 控制执行层可保留 Python 或逐步迁到 C++
- 规划与仿真层继续保留 Python 更灵活

## v1-v14 版本接口总表

| 版本 | 主要定位 | 控制接口 | 传感器接口 | 当前推荐用途 |
| --- | --- | --- | --- | --- |
| `v1` | 底层 CAN 控制 | Python 直连 CAN | 无独立传感器接口 | 验证底层动作 |
| `v2` | 时间开环剧本 | 时间调度 + JSON | 无闭环传感器 | 早期开环脚本 |
| `v3` | 倾角采集 | 辅助调试 | Python 串口 + ROS 2 发布 | 倾角数据接入 |
| `v4` | 闭环角度控制 | 目标角闭环 + JSON | Python 串口 + 雷达 IMU UDP | 实机控制主线 |
| `v5` | 雷达/IMU 接入 | 不直接控车 | Python UDP 雷达/IMU | 回转估计与点云 |
| `v6` | 相机接入 | 无 | Python/OpenCV + ROS 2 图像 | 摄像头接入 |
| `v7` | ROS 订阅式数据集 | 无实机控制 | ROS 2 话题订阅 | 早期数据集验证 |
| `v8` | 直连式数据集 | 无实机控制 | Python 直连多传感器 | 低延迟采集 |
| `v9` | Nav2 预研 | 规划中的底盘控制 | 规划中的导航传感器 | 导航预留 |
| `v10` | FK/IK/工作空间 | 几何逆解接口 | 不直连硬件 | 数学模型核心 |
| `v11` | 多模态采集系统 | 复用 v4 控制 | Python 传感器接入 + ROS 2 发布 | 集成采集 |
| `v12` | 混合架构系统 | ROS 2 控制节点 | C++ 高速传感器节点 + Python GUI | 向统一接口过渡 |
| `v13` | ROS 2 标准化实机节点 | ROS 2 控制消息 | ROS 2 传感器节点 | 标准化接口主线 |
| `v14` | URDF 仿真与规划 | `/joint_states` + 规划 JSON | 默认不接真实传感器 | 仿真验证与规划 |

## 各版本接口说明与用法

下面按版本单独说明“控制接口是什么、传感器接口是什么、怎么用”。

## v1_control_base

`v1` 是最底层的 CAN 控制版本。它不做闭环、不做传感器融合，重点是把控制器协议打通。

### 控制接口

核心控制接口在：

- `v1_control_base/zs_excavator_controller.py`

主要类：

- `ZSCanTransport`
- `ExcavatorController`

主要控制语义：

- 底盘前进/后退
- 大臂上/下
- 小臂拉/推
- 铲斗收/放
- 回转左/右
- `CH1/CH2/CH3` 模拟量输出

### 传感器接口

`v1` 没有独立传感器接入层，只接控制器串口：

```text
/dev/ttyUSB_Controller
```

### 用法

直接运行 GUI：

```bash
python3 src/shandong/v1_control_base/zs_excavator_gui.py
```

适合场景：

- 验证 CAN 通信
- 验证动作语义
- 验证比例阀输出

## v2_control_time_track

`v2` 在 `v1` 的基础上加入了时间调度，把动作编排成持续时间脚本。

### 控制接口

核心接口在：

- `v2_control_time_track/action_scheduler.py`
- `v2_control_time_track/run_json_script.py`

控制方式是：

```text
动作名 + duration_s + ramp_up_s + ramp_down_s + ch3_mv
```

它仍然是开环控制，不依赖实时角度反馈。

### 传感器接口

`v2` 没有真正闭环传感器接口。

### 用法

运行 GUI：

```bash
python3 src/shandong/v2_control_time_track/action_gui.py
```

运行 JSON 剧本：

```bash
python3 src/shandong/v2_control_time_track/run_json_script.py --json path/to/script.json
```

适合场景：

- 编排时间开环动作
- 快速回放简单剧本

## v3_sensor_read_wit

`v3` 的重点是把倾角传感器变成一个稳定的数据接口。

### 控制接口

`v3` 不提供完整实机控制接口，只提供“角度读数 + 调试联动界面”。

### 传感器接口

核心接口在：

- `v3_sensor_read_wit/readRad_ubuntu.py`
- `v3_sensor_read_wit/ros2_readRad_pub.py`

接入对象：

```text
/dev/ttyUSB_Sensor1
/dev/ttyUSB_Sensor2
/dev/ttyUSB_Sensor3
/dev/ttyUSB_Sensor4
```

当前约定 ID：

- `0x50`：铲斗
- `0x51`：小臂
- `0x52`：大臂
- `0x53`：回转参考

输出形式：

- 终端打印
- ROS 2 Topics

### 用法

读取串口：

```bash
python3 src/shandong/v3_sensor_read_wit/readRad_ubuntu.py
```

发布 ROS 2 话题：

```bash
python3 src/shandong/v3_sensor_read_wit/ros2_readRad_pub.py
```

适合场景：

- 倾角传感器联调
- 串口识别
- 关节相对角发布

## v4_control_closed

`v4` 是整个项目当前实机控制语义的核心来源。后续所有仿真和规划基本都在复用它的关节
语义。

### 控制接口

核心接口在：

- `v4_control_closed/angle_controller.py`
- `v4_control_closed/closed_loop_gui_imu.py`
- `v4_control_closed/run_closed_loop_script.py`

统一关节语义：

```text
boom_swing
arm_boom
bucket_arm
swing_yaw
```

控制方式：

- 目标角闭环
- 容差判断
- 柔性起停
- 卡死补偿
- JSON 剧本执行

### 传感器接口

传感器输入包括两类：

1. 倾角传感器：

```text
/dev/ttyUSB_Sensor1~4
```

2. 雷达 IMU UDP：

```text
UDP 6668
```

### 用法

启动最新闭环 GUI：

```bash
python3 src/shandong/v4_control_closed/closed_loop_gui_imu.py
```

执行闭环剧本：

```bash
python3 src/shandong/v4_control_closed/run_closed_loop_script.py --json path/to/script.json
```

适合场景：

- 实机闭环控制
- 单步动作调试
- 剧本录制与复现

## v5_sensor_read_lidar

`v5` 的重点是雷达点云和雷达 IMU 接入，不直接负责机械控制。

### 控制接口

`v5` 不直接提供实机控制接口，主要输出：

- 点云
- 回转角
- IMU 角速度/加速度

### 传感器接口

核心接口在：

- `v5_sensor_read_lidar/lidar_direct_reader.py`
- `v5_sensor_read_lidar/imu_direct_swing_estimator.py`

输入对象：

- 雷达 UDP 点云包
- 雷达 IMU 包

当前关键接口：

```text
UDP 6668
雷达设备 192.168.158.98:6543
```

### 用法

运行回转角估计：

```bash
python3 src/shandong/v5_sensor_read_lidar/imu_direct_swing_estimator.py
```

运行点云读取：

```bash
python3 src/shandong/v5_sensor_read_lidar/lidar_direct_reader.py
```

适合场景：

- 雷达驱动联调
- IMU 预积分
- 点云读取与标定

## v6_sensor_read_camera

`v6` 负责统一摄像头接入，不参与控制。

### 控制接口

`v6` 无机械控制接口。

### 传感器接口

核心接口在：

- `v6_sensor_read_camera/ros2_all_cams_pub.py`

输入来源：

- USB 摄像头
- 普通 RTSP
- 海康 RTSP

输出：

- `sensor_msgs/Image`

### 用法

统一发布多路图像：

```bash
python3 src/shandong/v6_sensor_read_camera/ros2_all_cams_pub.py
```

适合场景：

- 摄像头接入
- ROS 2 图像话题发布

## v7_lerobot_dataset

`v7` 是早期的 ROS 2 订阅式数据集采集版本。

### 控制接口

`v7` 不做实机控制，动作字段在数据集中是占位意义更强。

### 传感器接口

它不直连硬件，而是通过 ROS 2 订阅已有话题：

- 图像
- 角度
- 回转角

### 用法

```bash
python3 src/shandong/v7_lerobot_dataset/lerobot_data_collector.py
```

适合场景：

- 早期 LeRobot 数据格式验证

注意：

- 该方案后续已被认为延迟偏高，不是当前推荐主线。

## v8_direct_data_collection

`v8` 是 `v7` 的低延迟替代版，直接绕过 ROS 2，所有传感器 Python 直连。

### 控制接口

`v8` 不直接控车，重点是“同步记录”。

### 传感器接口

核心模块：

- `inclinometer_reader.py`
- `lidar_reader.py`
- `camera_reader.py`
- `lerobot_direct_collector.py`

输入对象：

- 4 路倾角串口
- 雷达 UDP
- 摄像头 RTSP/USB

### 用法

```bash
python3 src/shandong/v8_direct_data_collection/lerobot_direct_collector.py
```

适合场景：

- 低延迟多模态直连采集
- LeRobot 数据集录制

## v9_nav

`v9` 目前更像导航接口预留层，不是主线控制版本。

### 控制接口

规划目标是：

- 用 `cmd_vel` 一类接口控制底盘
- 接入 Nav2

### 传感器接口

规划目标是接：

- 雷达
- 底盘状态
- 回转角

### 用法

当前以文档占位为主，后续如果继续推进，需要补完整 launch、参数与底盘驱动封装。

## v10_cailbration_arm

`v10` 是几何模型核心，不直接接硬件，不直接控车。

### 控制接口

它输出的是：

- FK 结果
- IK 结果
- 工作空间
- 动画

最关键的接口是：

- `ExcavatorKinematics`
- `ExcavatorIK`

### 传感器接口

不直接接硬件，但接收：

- 倾角绝对读数
- 或目标点 `(X, Z)` 与铲斗姿态角

### 用法

测试逆解：

```bash
python3 src/shandong/v10_cailbration_arm/inverse_kinematics.py
```

测试工作空间：

```bash
python3 src/shandong/v10_cailbration_arm/workspace_analyzer.py
```

适合场景：

- 机械臂几何推导
- 铲斗逆解
- 工作空间分析

## v11_multimodal_dataset_collection

`v11` 是端到端多模态集成采集版本。

### 控制接口

控制层复用：

- `v1` 控制器
- `v4` 闭环控制语义

同时发布 ROS 2 话题，形成统一数据流。

### 传感器接口

接入对象：

- 相机
- 雷达点云
- 雷达 IMU
- 倾角状态

输出话题包括：

- `/camera*/image_raw`
- `/lidar/points`
- `/lidar/points_odom`
- `/lidar/elevation_map`
- `/excavator/joint_states`

### 用法

```bash
python3 src/shandong/v11_multimodal_dataset_collection/multimodal_gui.py
```

或使用 ROS 版本主脚本。

适合场景：

- 端到端多模态数据集采集
- 采集与可视化一体化验证

## v12_multimodal_hybrid_architecture

`v12` 是非常接近你后续目标的版本，因为它已经开始把传感器高频接口下沉到 C++。

### 控制接口

控制层接口已经 ROS 化，主要包括：

- `/excavator/target_joint_angles_deg`
- `/excavator/target_ch3_mv`
- `/excavator/joint_command_json`

由执行节点消费这些控制命令。

### 传感器接口

这版已经开始把高频传感器做成 C++ ROS 节点，包括：

- `imu_sensor_node.cpp`
- `lidar_processor_node.cpp`
- `rtsp_camera_node.cpp`

这正是你后续“传感器接口统一成 C++”最值得继续沿用的方向。

### 用法

统一启动：

```bash
ros2 launch v12_multimodal_hybrid_architecture v12_launch.py
```

适合场景：

- 高性能多传感器系统
- 作为未来统一传感器接口的直接参考版本

## v13_excavator_ros

`v13` 是当前最像“标准化 ROS 2 实机接口层”的版本。

### 控制接口

控制接口已经标准化成 ROS 2 消息：

- 订阅：`/v13/controller/cmd`
- 发布：`/v13/controller/status`

### 传感器接口

传感器接口也已经标准化：

- `/v13/inclinometer/raw`
- `/v13/inclinometer/group`
- `/v13/lidar/points`
- `/v13/lidar/imu`
- `/v13/robot/joint_state`

### 用法

```bash
ros2 launch v13_excavator_ros v13_bringup.launch.py
```

适合场景：

- 统一实机 ROS 2 接口
- 后续做接口标准化最值得参考的版本

## v14_urdf

`v14` 的重点不是实机传感器接入，而是把控制语义和几何规划接到 URDF 仿真上。

### 控制接口

核心接口：

- `/joint_states`
- `replay_json_script.py`
- `point_to_dig_dump_trajectory.py`
- `mode1_task_planner.py`

### 传感器接口

默认不直接接真实传感器。

当前主要输入是：

- 规划点 `(x, y, z)`
- JSON 剧本
- 或来自仿真模型的关节状态

### 用法

启动 URDF：

```bash
ros2 launch describe_60FED display.launch.py headless:=true
```

回放 JSON：

```bash
python3 src/shandong/v14_urdf/replay_json_script.py path/to/task.json --feedback
```

生成单点轨迹：

```bash
python3 src/shandong/v14_urdf/point_to_dig_dump_trajectory.py \
  --dig-x ... --dig-y ... --dig-z ... \
  --dump-x ... --dump-y ... --dump-z ...
```

适合场景：

- URDF 仿真验证
- 单点/多点规划
- 控制语义验证

## 后续统一接口建议

如果你后续真的要统一成一套接口，我建议按下面思路收口。

### 1. 控制接口统一建议

保留当前已经稳定的控制语义：

```text
boom_swing
arm_boom
bucket_arm
swing_yaw
```

统一控制消息可以设计成：

```text
joint_name
target_value
control_mode
tolerance
speed
aux_output
timestamp
```

这样就能同时兼容：

- `v4` 闭环角控制
- `v12/v13` ROS 控制消息
- `v14` URDF JointState 仿真

### 2. 传感器接口统一建议

传感器层建议统一成 C++ ROS 2 节点，按以下类别拆分：

1. 倾角节点
2. 雷达 IMU 节点
3. 点云节点
4. 相机节点
5. 关节状态融合节点

统一输出标准消息，不让上层再关心：

- 串口怎么读
- UDP 怎么解包
- RTSP 怎么拉流

### 3. 当前最值得保留的两版

如果只从“后续统一接口”的角度看，最值得保留的参考主线其实是：

- `v4_control_closed`
  - 保留控制语义和闭环逻辑
- `v12_multimodal_hybrid_architecture`
  - 保留 C++ 传感器节点架构
- `v13_excavator_ros`
  - 保留 ROS 2 标准化接口命名
- `v14_urdf`
  - 保留仿真与规划接口

也就是说，你后续统一接口最自然的组合应该是：

```text
v4 的控制语义
+ v12/v13 的 ROS 2 传感器接口
+ v14 的仿真与规划接口
```

## 推荐阅读顺序

如果你接下来马上要做“统一控制接口 + 传感器接口改 C++”，建议按下面顺序读：

1. `v4_control_closed/README.md`
2. `v12_multimodal_hybrid_architecture/HANDOVER_SUMMARY.md`
3. `v13_excavator_ros/README.md`
4. `v14_urdf/README.md`

这样可以最快把：

- 实机控制语义
- C++ 传感器节点
- ROS 2 标准接口
- URDF 仿真接口

串成一条统一路线。
