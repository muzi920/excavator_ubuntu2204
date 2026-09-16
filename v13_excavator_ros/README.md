# v13 excavator ros

这个 `README` 说明 `v13_excavator_ros` 包里每个 ROS2 程序对应的硬件，
以及你在联调和测试时该看哪个话题、哪个端口、哪个状态输出。

`v13_excavator_ros` 当前包含 4 个直接连接硬件的 C++ 节点、1 个聚合主节
点，以及 1 个后续 Python 扩展入口。你可以单独启动每个节点做测试，也可
以用 launch 文件一起启动。

<!-- prettier-ignore -->
> [!IMPORTANT]
> 当前 `lidar_reader` 和 `lidar_imu_reader` 都连接同一台雷达设备，默认都使用
> UDP 监听端口 `6668` 和设备地址 `192.168.158.99:6543`。如果现场设备不支持
> 两个进程同时接收同一份 UDP 数据，你需要把它们改成单接收节点，再由内部转发。

## 程序与硬件对应关系

下面这张表是测试时最重要的映射表。你可以直接按这个表确认每个程序连接的是
哪个传感器或控制器。

| ROS2 可执行程序 | ROS 节点名 | 对应硬件 | 默认连接参数 | 主要输出 |
| --- | --- | --- | --- | --- |
| `inclinometer_reader` | `v13_inclinometer_reader` | 4 路倾角传感器 | `/dev/ttyUSB_Sensor1`~`4`，`230400` 波特率 | `/v13/inclinometer/raw`，`/v13/inclinometer/group` |
| `lidar_reader` | `v13_lidar_reader` | 雷达点云 | 监听 `6668`，设备 `192.168.158.99:6543` | `/v13/lidar/points` |
| `lidar_imu_reader` | `v13_lidar_imu_reader` | 雷达内置 IMU / 回转 IMU 数据 | 监听 `6668`，设备 `192.168.158.99:6543` | `/v13/lidar/imu` |
| `controller_node` | `v13_controller_node` | 挖机控制器 / USB-CAN 控制串口 | `/dev/ttyUSB_Controller`，`115200` 波特率 | `/v13/controller/status` |
| `v13_main_node` | `v13_main_node` | 不直接连接硬件 | 订阅倾角组和雷达 IMU 输出 | `/v13/robot/joint_state`，`/v13/system/summary` |
| `v13_python_bridge.py` | `v13_python_bridge` | 不直接连接硬件 | 无默认硬件连接 | 预留给 Python 调用 |

## 倾角传感器对应关系

`inclinometer_reader` 会一次轮询 4 个串口设备，并按参数文件里的顺序给它们命名。
默认配置如下：

| 串口 | 传感器 ID | 逻辑名称 | 说明 |
| --- | --- | --- | --- |
| `/dev/ttyUSB_Sensor1` | `0x50` | `bucket` | 铲斗倾角传感器 |
| `/dev/ttyUSB_Sensor2` | `0x51` | `arm` | 小臂倾角传感器 |
| `/dev/ttyUSB_Sensor3` | `0x52` | `boom` | 大臂倾角传感器 |
| `/dev/ttyUSB_Sensor4` | `0x53` | `swing` | 回转位置上的倾角传感器 |

这些配置来自 `config/v13_topics.yaml`。如果现场接线顺序和这里不同，你需要改
这个 YAML，而不是改测试命令。

## 每个程序测试时看什么

这一节按“你启动一个程序后，该看什么结果”来组织，适合现场快速联调。

## 关节相关 topic 速查

如果你当前主要关心“机械臂三个相对角”和“雷达 IMU 回转角”，先看这一节最直
接。当前实现里，这些数据不是分别拆成 4 个独立 topic，而是放在 3 个核心
topic 里。

| 你要看的数据 | topic | 消息类型 | 关键字段 |
| --- | --- | --- | --- |
| 4 路倾角原始值 | `/v13/inclinometer/raw` | `v13_excavator_ros/msg/Inclinometer` | `roll_deg`、`pitch_deg`、`yaw_deg` |
| 机械臂 3 个相对角和 4 路预处理角度 | `/v13/inclinometer/group` | `v13_excavator_ros/msg/InclinometerGroup` | `bucket_arm_deg`、`arm_boom_deg`、`boom_swing_deg` |
| 雷达 IMU 预处理后的回转数据 | `/v13/lidar/imu` | `v13_excavator_ros/msg/LidarImu` | `swing_deg`、`yaw_rate`、`calibrated` |
| 融合后的 4 维关节参数 | `/v13/robot/joint_state` | `v13_excavator_ros/msg/RobotJointState` | `bucket_arm_deg`、`arm_boom_deg`、`boom_swing_deg`、`swing_yaw_deg` |

如果你想快速验证这 3 个核心输出，直接运行：

```bash
ros2 topic echo /v13/inclinometer/group
ros2 topic echo /v13/lidar/imu
ros2 topic echo /v13/robot/joint_state
```

### `inclinometer_reader`

这个节点负责连接 4 个倾角传感器，读取 Modbus 数据，并发布统一的
`Inclinometer` 消息。同时，它会在完成初始化预处理后，再发布一份组合结果
消息，包含 4 路修正后的倾角和 3 个机械臂相对角。

启动命令：

```bash
ros2 run v13_excavator_ros inclinometer_reader
```

测试时重点看这些内容：

- 串口是否存在：`/dev/ttyUSB_Sensor1` 到 `/dev/ttyUSB_Sensor4`
- 状态话题：`/v13/inclinometer/status`
- 原始数据话题：`/v13/inclinometer/raw`
- 预处理组合话题：`/v13/inclinometer/group`

查看原始数据：

```bash
ros2 topic echo /v13/inclinometer/raw
```

你会看到类似下面的字段：

```text
sensor_name: bucket
sensor_id: 80
roll_deg: ...
pitch_deg: ...
yaw_deg: ...
```

查看三个机械臂相对角和 4 路预处理后的倾角：

```bash
ros2 topic echo /v13/inclinometer/group
```

你会看到这些关键字段：

- `bucket_pitch_deg`
- `arm_pitch_deg`
- `boom_pitch_deg`
- `swing_pitch_deg`
- `bucket_arm_deg`
- `arm_boom_deg`
- `boom_swing_deg`

<!-- prettier-ignore -->
> [!NOTE]
> `inclinometer_reader` 启动后会先做 `init_sample_count` 次静态初始化，先估计
> 每路传感器的零位偏置。初始化完成前，`/v13/inclinometer/raw` 会有数据，但
> `/v13/inclinometer/group` 可能暂时还没有输出。

### `lidar_reader`

这个节点负责连接雷达，发送启动命令，并把收到的 UDP 点云解析后发布为
`PointCloud2`。

启动命令：

```bash
ros2 run v13_excavator_ros lidar_reader
```

测试时重点看这些内容：

- 本机 UDP 监听端口：`6668`
- 雷达设备地址：`192.168.158.99:6543`
- 状态话题：`/v13/lidar/status`
- 点云话题：`/v13/lidar/points`

查看点云头信息：

```bash
ros2 topic echo /v13/lidar/points --once
```

如果节点工作正常，你至少会看到 `width`、`fields`、`data` 等 `PointCloud2`
字段。

### `lidar_imu_reader`

这个节点同样连接那台雷达，但它只处理雷达里的 IMU 数据，并输出回转角和
偏航角速度。

启动命令：

```bash
ros2 run v13_excavator_ros lidar_imu_reader
```

测试时重点看这些内容：

- 本机 UDP 监听端口：`6668`
- 雷达设备地址：`192.168.158.99:6543`
- 状态话题：`/v13/lidar_imu/status`
- IMU 话题：`/v13/lidar/imu`

查看数据：

```bash
ros2 topic echo /v13/lidar/imu
```

你会看到这些关键字段：

- `accel_x`, `accel_y`, `accel_z`
- `gyro_x`, `gyro_y`, `gyro_z`
- `swing_deg`
- `yaw_rate`
- `calibrated`
- `sensor_timestamp_ns`

<!-- prettier-ignore -->
> [!NOTE]
> 这个节点启动后的前 3 秒会做零偏和重力方向校准。测试时你必须让挖机保持静止，
> 否则 `swing_deg` 和 `yaw_rate` 的结果会不稳定。

### `controller_node`

这个节点负责通过 USB-CAN 控制器下发动作命令。它连接的不是传感器，而是控制
器串口。

启动命令：

```bash
ros2 run v13_excavator_ros controller_node
```

测试时重点看这些内容：

- 控制器串口：`/dev/ttyUSB_Controller`
- 波特率：`115200`
- 状态话题：`/v13/controller/status`
- 控制命令输入话题：`/v13/controller/cmd`

查看状态：

```bash
ros2 topic echo /v13/controller/status
```

发送一条测试控制命令：

```bash
ros2 topic pub --once /v13/controller/cmd \
  v13_excavator_ros/msg/ControllerCommand \
  "{motion_name: forward, ch1_mv: 1000, ch2_mv: 1000, ch3_mv: 3000, emergency_stop: false}"
```

常用 `motion_name` 包括：

- `forward`
- `backward`
- `turn_left`
- `turn_right`
- `boom_up`
- `boom_down`
- `arm_push`
- `arm_pull`
- `bucket_in`
- `bucket_out`
- `swing_left`
- `swing_right`
- `stop`

### `v13_main_node`

这个节点不直接连接任何传感器或控制器。它的作用是订阅倾角预处理结果和雷达
IMU 预处理结果，把它们按时间戳做同步判断，然后融合成 `robot` 的 4 维关
节参数。

启动命令：

```bash
ros2 run v13_excavator_ros v13_main_node
```

测试时重点看这些内容：

- 融合关节话题：`/v13/robot/joint_state`
- 汇总字符串话题：`/v13/system/summary`
- 订阅来源：
  - `/v13/inclinometer/group`
  - `/v13/lidar/imu`

查看融合后的 4 维关节参数：

```bash
ros2 topic echo /v13/robot/joint_state
```

你会看到这些关键字段：

- `bucket_arm_deg`
- `arm_boom_deg`
- `boom_swing_deg`
- `swing_yaw_deg`
- `yaw_rate`
- `sync_delta_sec`

查看汇总结果：

```bash
ros2 topic echo /v13/system/summary
```

<!-- prettier-ignore -->
> [!NOTE]
> 当前实现会发布融合后的 `RobotJointState`，但还没有拆成
> `/v13/joint/bucket_arm`、`/v13/joint/arm_boom` 这种单关节独立 topic。
> 如果你需要这种更直观的测试方式，可以继续在 `v13_main_node` 上追加。

## 一起启动

如果你要做整包联调，可以先构建再用 launch 一次性启动全部节点。

```bash
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
colcon build --packages-select v13_excavator_ros
source install/setup.bash
ros2 launch v13_excavator_ros v13_bringup.launch.py
```

这个 launch 默认会启动：

1. `v13_inclinometer_reader`
2. `v13_lidar_reader`
3. `v13_lidar_imu_reader`
4. `v13_controller_node`
5. `v13_main_node`

## 参数文件位置

如果你要改测试环境里的硬件映射、IP、端口或串口名，直接改这个文件：

- `config/v13_topics.yaml`

你最常改的参数一般是：

- 倾角传感器串口名 `serial_ports`
- 倾角传感器逻辑名 `sensor_names`
- 雷达 IP `lidar_ip`
- 雷达端口 `lidar_port`
- 控制器串口 `device`

## 快速排查

如果你测试时发现某个程序没数据，按这个顺序排查最省时间。

1. 确认节点是否启动成功。
2. 确认设备文件是否存在，例如 `/dev/ttyUSB_Controller`。
3. 确认参数文件里的串口名、IP 和端口是否和现场一致。
4. 确认状态话题里是否有失败信息。
5. 再确认数据话题是否真的没有消息。

常用命令：

```bash
ros2 node list
ros2 topic list
ros2 topic echo /v13/inclinometer/status
ros2 topic echo /v13/lidar/status
ros2 topic echo /v13/lidar_imu/status
ros2 topic echo /v13/controller/status
```

## 下一步

这份 README 现在适合做硬件联调和节点归属确认。后续如果你要，我可以继续补两
类文档：

- 一份“现场测试流程”，按上电、启动、回显、动作验证来写
- 一份“topic / msg 接口手册”，方便后续 Python 或上位机直接接入
