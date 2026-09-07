# v15 关节指令库 · 传感器接口 · 外参 · 温漂 · Examples 规范（v2 对齐 v0~v14 真实硬件）

> **v2 Changelog（首次 reject 后重写）**：按用户 feedback `参考 v0~v14 现有传感器方案` 要求，FR-3 倾角改回真实 WT901C485 MODBUS 地址 0x50~53 + Float64MultiArray 话题；新增 FR-5 外参（static_transform_publisher 6 参数）出厂默认值对齐 sensors_tf.launch.py；新增 FR-6 倾角温漂/零偏（alpha=0.98 互补滤波 + 50 帧启动校准）；新增 FR-7 独立 TF 预标定 examples 脚本（对标 tf_calibration_gui.py，非每次运行）。新增 AC-8/9/10 三条 rule 验收。

## Problem

v15_action_task 库当前存在以下 7 个结构性缺口（用户 4 条主需求 + 3 条 feedback 补充项），导致上层业务代码无法标准化接入：

1. **关节限位未形成 spec 契约**：虽然 `config/default_config.yaml` 已定义 `joint_limits` 节并由 `JointLimitsConfig` 加载，但缺乏在 spec 层面的统一三层契约说明，需要正式纳入。
2. **作业指令层缺失单一关节布尔+角度指令**：当前仅有 StepBuilder 剧本式原语，缺少面向"单次直接调用"的 `arm_move(bool up_or_down, float angle)` 风格 4 关节单一指令函数。
3. **传感器 ROS 订阅接口完全缺失**（用户主需求第 3 条）：当前仅覆盖关节控制的 `/joint_states`，完全没有倾角（4 路 WT901C485）、雷达（5 路：1 矩阵固态 + 2 环视 360 + 2 单线）、相机（≥5 路：海康主 + 网络 + 俯视）的 ROS 订阅封装；且必须与 v0~v14 代码 **零 import 依赖**，v15 纯独立。
4. **测试脚本混入正式代码**：缺少独立 examples 目录，验证脚本散落在 `action_library/` 等源码树中。
5. **传感器外参无法在 config 中统一管理**（用户 feedback 补充项 3）：v0~v14 时代使用 `launch/sensors_tf.launch.py` 硬编码 6 参数 static_transform_publisher，无法 YAML 化修改，必须加入 config 并保留出厂默认值（含雷达倒装 roll=3.0316rad 真实倒装值）。
6. **倾角温漂与零偏处理缺失**（用户 feedback 补充项 2）：v11/v5 都重复实现了 TiltCompensator + 3s 静止校准，属于横切关注点，必须在 v15 SensorManager 内部统一内置。
7. **雷达/相机 TF 预标定工具无独立脚本**（用户 feedback 补充项 1）：v5 只有 Tkinter GUI `tf_calibration_gui.py`，缺少命令行版和 GUI 输出 → v15 YAML 的一键转写脚本，且两者必须是 examples 级独立工具，不混入每次启动的正式代码。

---

## Users

| 角色 | 描述 |
|---|---|
| 业务应用开发者 | 调用 v15_action_task 开发挖掘/卸料/装车等作业脚本，需要 arm_move 类便捷函数直接驱动单关节 |
| 感知算法工程师 | 订阅倾角/雷达/相机数据流（14 路 +），完成避障、地形识别、姿态估计 |
| 系统集成/标定工程师 | 换车/换传感器安装位置后，独立运行 examples TF 标定脚本（不跑主程序），输出 YAML 外参覆盖出厂值 |
| 库维护者 | 通过统一配置文件 + examples 验证新功能、做回归测试 |

---

## Goals

1. （FR-1）将已实现的关节限位配置正式纳入规范，明确 **Config → URDFController → Action Library** 三层限位裁剪契约。
2. （FR-2）在 `action_library/primitives/` 新增 8 个语义作业指令函数：4 个基础布尔+增量（`arm_move` 等）+ 4 个扩展便利函数（`swing_move_to`、`boom_lift` 等），方向语义严格统一表。
3. （FR-3）新增独立 `sensors/` 子包，提供 **4 倾角 + 5 雷达 + ≥5 相机** 共 ≥14 路传感器 ROS 订阅统一封装；全部话题/消息类型/QoS/frame 走 config，且 **倾角话题名+数据类型 1:1 对齐 v12 inclinometer_sensor_bridge 的真实 Float64MultiArray**；雷达对齐 PaceCat M300 原始 PointCloud2。
4. （FR-5）config 新增 `extrinsics:` 顶层节，每台传感器外参 6 参数（x/y/z m + yaw/pitch/roll rad）**严格对齐 static_transform_publisher 参数顺序**，lidar_front 出厂默认值 = sensors_tf.launch.py 真实倒装值 `x=-0.5500 y=-0.2000 z=1.2712 yaw=0.0532 pitch=0.0349 roll=3.0316`。
5. （FR-6）config 新增 `tilt_compensation:` 顶层节（alpha=0.98, calib_count=50, gyro_deadzone=0.002rad/s, auto_calibrate=True）；SensorManager 每路 tilt 内置 `TiltCompensator`，启动时静止 50 帧自动扣零偏，读数返回 `compensated_angle_deg` 字段。
6. （FR-7）新增 2 个独立 examples 脚本：① ex11 无 GUI TF 命令行标定（对标 v5 GUI 工具，输出 tf_record.txt）；② ex12 把 GUI 输出的 `tf_calibration_record.txt` 一键转写为 v15 `extrinsics_<chassis_sn>.yaml`，可直接 load_config。
7. （FR-4）顶层 `examples/` 目录（12 脚本 + 辅助）独立存在，不混入 `v15_action_task/` 包源码树。

---

## Non-Goals

1. 不修改现有 StepBuilder 剧本系统（`move_joint_step` 等保持不动）。
2. 不实现传感器算法处理（点云融合/检测/分割、相机解码 H.264、IMU 预积分多传感器融合），仅做订阅缓存 + latest 查询 API；SensorManager 回调内 ≤1ms。
3. 不改动 v14 URDF `/joint_states` 协议。
4. 不改动 FK/IK 实现细节。
5. 不新增 GUI 工具（标定 ex11 用命令行滑条，不依赖 Tkinter，兼容 SSH 无头环境）。
6. 不重复实现 WIT 倾角 MODBUS 驱动硬件层（v12 bridge 已跑在独立节点，v15 只订阅其 ROS 话题；AC-6 零 import 约束保证）。
7. 不实现点云 TF 外参变换（v5 pointcloud_transformer 模式由用户业务层可选调用 tf2_ros.do_transform_cloud，非 v15 标准库范围；extrinsics 提供 6 参数 + 4×4 matrix 两种输出即可）。

---

## Functional Requirements

### FR-1 关节限位三层契约纳入

- **FR-1.1** 规范明确 4 语义关节限位默认值（**与已实现 default_config.yaml 完全一致，不修改任何数值**）：
  - `swing_yaw`: [-180.0°, +180.0°]
  - `boom_swing`: [-5.0°, +55.0°]
  - `arm_boom`: [0.0°, +130.0°]
  - `bucket_arm`: [-95.0°, +45.0°]
- **FR-1.2** 三层限位契约（任何运动指令必须逐级经过，可 bypass 但默认全启用）：
  1. **Config 层（纯函数）**：`V15Config.limits.clamp_pose(pose_dict) -> pose_dict` 裁剪、`check(pose_dict) -> List[str]` 校验。
  2. **URDFController 层（默认开启）**：构造时注入 `joint_limits=`，`set_joint/set_pose` 发布前 `_clamp_pose()` 自动裁剪；可通过 `clamp=False` 临时绕过。
  3. **Action Library 层（FR-2 函数必须走）**：新增的 8 个语义指令函数在发布前调用 Config 层 `clamp_joint/pose` 裁剪，超限不越界。

### FR-2 语义作业指令库（arm_move 风格）

在 `action_library/primitives/semantic_moves.py` 新增 8 函数，返回值统一复用 `motion.cartesian_mover.MoveResult`（`__bool__()` 返回 success，支持 `if arm_move(...):` 简写）。

#### FR-2.1 基础 4 个布尔+增量函数（用户 `arm_move(bool, angle)` 风格严格对齐）

| 函数签名 | 语义 | 布尔=True 方向 | 角度单位 |
|---|---|---|---|
| `swing_move(ctl, right_or_left: bool, angle_deg: float, *, blocking: bool=True, tolerance_deg: float=1.0, timeout_s: float=3.0) -> MoveResult` | 回转单一增量 | True=顺时针（俯视 +Z 右手定则，swing_yaw **正**增量） | 度 |
| `boom_move(ctl, up_or_down: bool, angle_deg: float, *, ...) -> MoveResult` | 大臂单一增量 | True=抬起（boom_swing **负**增量，向 -5° 方向） | 度 |
| `arm_move(ctl, up_or_down: bool, angle_deg: float, *, ...) -> MoveResult` | 小臂单一增量 | True=收回（arm_boom **正**增量，向 +130° 方向） | 度 |
| `bucket_move(ctl, up_or_down: bool, angle_deg: float, *, ...) -> MoveResult` | 铲斗单一增量 | True=收斗（bucket_arm **正**增量，向 +45° 方向） | 度 |

#### FR-2.2 扩展 4 个便利函数

- `swing_move_to(ctl, abs_yaw_deg, ...)` → 转到绝对 yaw（不是增量）
- `boom_lift(mover, delta_z_m, *, keep_xy=True, ...)` → 保持铲尖 XY 不动抬升 Δz（走 CartesianMover + IK，需 mover 对象，非 ctl 增量）
- `arm_extend(mover, delta_x_m, *, keep_yz=True, ...)` → 保持 YZ 不动沿 X 方向伸臂
- `bucket_tilt_to(ctl, abs_bucket_tip_deg, ...)` → 铲斗齿尖绝对倾角（需要 FK 反解 bucket_arm，不是 bucket_arm 直接设）

#### FR-2.3 每个函数统一 5 步契约（FR-1.2 第 3 层）

1. 读取 `ctl.get_pose_or_default()` 当前反馈角；
2. 按布尔方向 ± angle_deg 计算目标角；
3. **Config 层裁剪**：目标角经 `cfg.limits.clamp_joint(joint_name, raw_target)` 或等价 `clamp_pose` 裁到限位内；
4. `ctl.set_joint(joint_name, target_deg)` 发布到 Adapter；
5. `blocking=True` 时轮询 `ctl.is_at_pose(tolerance_deg)`，≤timeout_s 到位则 `MoveResult.success=True`，否则 False；`blocking=False` 直接发布立即返回 success=True（视为投递成功）。

### FR-3 传感器 ROS 2 订阅统一接口（对齐 v0~v14 真实硬件方案）

新增 `v15_action_task/sensors/` 子包。**硬约束：该子包任何文件不准 import v0~v14（AC-6 grep 空匹配保证）。**

#### FR-3.1 Config 节（default_config.yaml 新增 `sensors:` 顶层节，结构对齐真实 v12/v5/v6 硬件）

```yaml
# ============================================================
# 7. 传感器配置（与 v12 inclinometer_sensor_bridge / v5 lidar / v11 cam 1:1 对齐）
# ============================================================
sensors:
  # --- 7.1 4 路倾角（维特智能 WT901C485，MODBUS 地址对齐 v12 addr_to_index）---
  #   v12 真实话题：/excavator/inclinometer_pitch_deg  (Float64MultiArray, 4 路：[bucket,arm,boom,swing])
  #   v12 真实 MODBUS 地址：0x50=bucket / 0x51=arm / 0x52=boom / 0x53=swing
  tilt_sensors:
    - id: tilt_bucket          # 0x50 → bucket 倾角（铲斗连杆绝对角）
      type: inclinometer_wt901c485
      modbus_addr: "0x50"
      semantic_joint: bucket_arm
      topic: "/excavator/inclinometer_pitch_deg"   # v12 bridge 真实话题名
      array_index: 0           # 在 Float64MultiArray.data[] 中的索引（对齐地址升序）
      msg_type: "std_msgs/msg/Float64MultiArray"
      frame_id: "bucket_link"
      qos_depth: 10
      enabled: true
    - id: tilt_arm             # 0x51 → arm 小臂
      modbus_addr: "0x51"
      semantic_joint: arm_boom
      topic: "/excavator/inclinometer_pitch_deg"
      array_index: 1
      msg_type: "std_msgs/msg/Float64MultiArray"
      frame_id: "arm_link"
      qos_depth: 10
      enabled: true
    - id: tilt_boom            # 0x52 → boom 大臂
      modbus_addr: "0x52"
      semantic_joint: boom_swing
      topic: "/excavator/inclinometer_pitch_deg"
      array_index: 2
      msg_type: "std_msgs/msg/Float64MultiArray"
      frame_id: "boom_link"
      qos_depth: 10
      enabled: true
    - id: tilt_swing           # 0x53 → swing 回转底盘
      modbus_addr: "0x53"
      semantic_joint: swing_yaw
      topic: "/excavator/inclinometer_pitch_deg"
      array_index: 3
      msg_type: "std_msgs/msg/Float64MultiArray"
      frame_id: "swing_base_link"
      qos_depth: 10
      enabled: true

  # --- 7.2 最多 5 路雷达（1 矩阵固态主雷达 + 2 环视 360 + 2 单线补盲）---
  #   PaceCat M300-E 单线真实话题：/pointcloud  (PointCloud2, 帧=lidar_front_link)
  lidars:
    - id: lidar_matrix_front          # ① 矩阵式固态雷达（主雷达，前向）
      type: lidar_solid_state_matrix
      topic: "/sensors/lidar/matrix_front/scan"
      msg_type: "sensor_msgs/msg/PointCloud2"
      frame_id: "lidar_matrix_front_link"
      qos_depth: 10
      enabled: false                   # 默认关，装机后开
    - id: lidar_360_left               # ② 环视 360 左
      type: lidar_360_hemisphere
      topic: "/sensors/lidar/360_left/scan"
      msg_type: "sensor_msgs/msg/PointCloud2"
      frame_id: "lidar_360_left_link"
      qos_depth: 10
      enabled: false
    - id: lidar_360_right              # ③ 环视 360 右
      type: lidar_360_hemisphere
      topic: "/sensors/lidar/360_right/scan"
      msg_type: "sensor_msgs/msg/PointCloud2"
      frame_id: "lidar_360_right_link"
      qos_depth: 10
      enabled: false
    - id: lidar_single_rear            # ④ 单线补盲 后向
      type: lidar_single_line
      topic: "/pointcloud"             # ← v5 PaceCat M300 真实原始话题（默认开，主单线）
      msg_type: "sensor_msgs/msg/PointCloud2"
      frame_id: "lidar_single_rear_link"
      qos_depth: 10
      enabled: true
    - id: lidar_single_bucket          # ⑤ 单线补盲 斗齿前向
      type: lidar_single_line
      topic: "/sensors/lidar/single_bucket/scan"
      msg_type: "sensor_msgs/msg/PointCloud2"
      frame_id: "lidar_single_bucket_link"
      qos_depth: 10
      enabled: false

  # --- 7.3 ≥5 路相机视频流（对齐 v11 hikvision + v6 network_cam 真实 TF）---
  #   真实消息：sensor_msgs/msg/Image /sensors/cam/<id>/image_raw
  cameras:
    - id: cam_front_main               # ① 海康主相机（v11 hikvision_cam_node）
      type: camera_hikvision_usb
      topic: "/sensors/camera/front_main/image_raw"
      info_topic: "/sensors/camera/front_main/camera_info"  # 空=不订阅
      msg_type: "sensor_msgs/msg/Image"
      frame_id: "hikvision_cam_frame"
      qos_depth: 5
      enabled: true
    - id: cam_left                     # ② 网络左相机
      type: camera_rtsp_network
      topic: "/sensors/camera/left/image_raw"
      info_topic: ""
      msg_type: "sensor_msgs/msg/Image"
      frame_id: "network_cam_left_frame"
      qos_depth: 5
      enabled: true
    - id: cam_right                    # ③ 网络右相机
      type: camera_rtsp_network
      topic: "/sensors/camera/right/image_raw"
      info_topic: ""
      msg_type: "sensor_msgs/msg/Image"
      frame_id: "network_cam_right_frame"
      qos_depth: 5
      enabled: true
    - id: cam_rear                     # ④ 后向环视
      type: camera_rtsp_network
      topic: "/sensors/camera/rear/image_raw"
      info_topic: ""
      msg_type: "sensor_msgs/msg/Image"
      frame_id: "cam_rear_link"
      qos_depth: 5
      enabled: false
    - id: cam_bucket_view              # ⑤ 斗齿作业视角（第 5 路，满足"5 个以上"）
      type: camera_fisheye_usb
      topic: "/sensors/camera/bucket_view/image_raw"
      info_topic: ""
      msg_type: "sensor_msgs/msg/Image"
      frame_id: "cam_bucket_view_link"
      qos_depth: 5
      enabled: false
    - id: cam_arm_overlook             # ⑥ 臂架俯视（可选第 6 路）
      type: camera_fisheye_usb
      topic: "/sensors/camera/arm_overlook/image_raw"
      info_topic: ""
      msg_type: "sensor_msgs/msg/Image"
      frame_id: "cam_arm_overlook_link"
      qos_depth: 5
      enabled: false
```

#### FR-3.2 Config 新增 dataclass

`config/loader.py` 新增：`SingleSensorConfig`（通用 12 字段，含 tilt 专属 modbus_addr/array_index/semantic_joint、camera 专属 info_topic）+ `SensorsConfig`（聚合 tilt_sensors: Dict[id, SingleSensorConfig] / lidars / cameras）。

#### FR-3.3 SensorHub 统一接口（Adapter 模式，对齐 RosV14Adapter 生命周期）

- `SensorManager.from_config(cfg: V15Config, *, ros_environment: Literal["auto","force_ros","force_mock"] = "auto")` → 自动检测 rclpy，无 ROS 退回 Mock。
- 生命周期：`open()` 启动订阅 + TiltCompensator 启动校准；`close()` 停止销毁；支持 `with`。
- 查询 API（业务层唯一调用）：
  - `get_tilt(sensor_id: str) -> Optional[TiltReading]` ／ `all_tilt() -> Dict`
  - `get_lidar(id) -> Optional[LidarReading]` ／ `all_lidar()`
  - `get_camera(id) -> Optional[CameraReading]` ／ `all_camera()`
  - `list_tilt_ids()` 等枚举
- Reading 数据类（sensors/base.py，零 ROS 依赖）：
  - **TiltReading**（FR-6 温漂输出）：`id, ts_s, is_valid, raw_angle_deg, compensated_angle_deg, temperature_c_or_none, roll_deg_or_none, pitch_deg_or_none, calibration_done, calibration_remaining`
  - **LidarReading**：`id, ts_s, is_valid, points_count, width_or_none, height_or_none, raw_ref_or_none`
  - **CameraReading**：`id, ts_s, is_valid, width_px, height_px, encoding, seq_or_none, raw_ref_or_none`
- 回调仅做 3 件事（NFR-2.1 ≤1ms）：拿 stamp → 转 ts_s → 存基本元信息；**不保存大对象（raw_ref 默认 None，可单独开开关）**。
- ROS 动态 import：无 rclpy 环境 `import v15_action_task.sensors` 不崩，`is_ros_backend()` 返回 False + reason 字符串。

### FR-5 外参 Extrinsics 配置（用户 feedback：加入 config 可修改）

config 新增 `extrinsics:` 顶层节，**严格对齐 static_transform_publisher 6 参数顺序（x y z yaw pitch roll）**，单位米/弧度，同时可选支持 4×4 matrix / quaternion（两种可互转，单一源为 6 参数）。

```yaml
# ============================================================
# 8. 传感器外参（相对于 base_link；对齐 v12 launch/sensors_tf.launch.py 出厂值）
#   格式：static_transform_publisher 6 参数 = [x_m, y_m, z_m, yaw_rad, pitch_rad, roll_rad, parent, child]
# ============================================================
extrinsics:
  # 出厂默认值 = sensors_tf.launch.py lidar_front 真实值（倒装）
  - sensor_id: lidar_single_rear
    parent_frame: "base_link"
    child_frame: "lidar_single_rear_link"
    x_m: -0.5500
    y_m: -0.2000
    z_m:  1.2712
    yaw_rad:   0.0532
    pitch_rad: 0.0349
    roll_rad:  3.0316          # ← 真实倒装值 π 量级，必须原样保留出厂默认
    quaternion_xyzw_or_none: null     # 可选：不填则从 rpy 推导
    matrix_4x4_or_none: null          # 可选：不填则从 xyz+q 推导
    note: "PaceCat M300-E 单线，出厂标定，倒装。换安装位置后跑 examples/ex11+ex12 生成新值覆盖。"

  # 海康主相机（真实值，sensors_tf.launch.py hikvision）
  - sensor_id: cam_front_main
    parent_frame: "base_link"
    child_frame: "hikvision_cam_frame"
    x_m:  0.4539
    y_m:  0.1532
    z_m:  1.5246
    yaw_rad:   0.0
    pitch_rad: 1.1519         # ≈ 66° 下俯
    roll_rad:  0.0
    note: "v11 海康主相机，出厂值"

  # 网络左相机（真实值 sensors_tf.launch.py network_cam_frame）
  - sensor_id: cam_left
    parent_frame: "base_link"
    child_frame: "network_cam_left_frame"
    x_m:  0.4239
    y_m: -0.1768
    z_m:  1.4246
    yaw_rad:   0.0
    pitch_rad: 0.9250         # ≈ 53° 下俯
    roll_rad:  0.0

  # 其余 2 雷达 + 3 相机：出厂设 0 0 0 0 0 0（未装机，标定时填）
  - sensor_id: lidar_matrix_front
    parent_frame: "base_link"
    child_frame: "lidar_matrix_front_link"
    x_m: 0.0; y_m: 0.0; z_m: 0.0; yaw_rad: 0.0; pitch_rad: 0.0; roll_rad: 0.0
    enabled: false
  # ...（lidar_360_left/right / lidar_single_bucket / cam_right / cam_rear / cam_bucket_view / cam_arm_overlook 共 10 条，id 与 sensors.节 1:1 对应）
```

Loader 新增 `ExtrinsicsEntry` + `ExtrinsicsConfig` dataclass，暴露便捷 API：
- `extrinsics.get(sensor_id) -> Optional[ExtrinsicsEntry]`
- `extrinsics.to_static_transform_publisher_args(sensor_id) -> List[str]` → 6 参数字符串列表，可直接喂 static_transform_publisher CLI。

### FR-6 倾角温漂/零偏处理（用户 feedback：订阅后的温漂处理）

config 新增 `tilt_compensation:` 顶层节，SensorManager 每路 tilt 内置 `TiltCompensator` 实例。

```yaml
# ============================================================
# 9. 倾角温漂补偿（对齐 v11 imu_preintegration.py TiltCompensator + v5 swing 估计 gyro bias）
# ============================================================
tilt_compensation:
  alpha: 0.98                 # 互补滤波权重（gyro 积分 0.98 + accel 绝对方向 0.02）；v11 默认值
  calib_count: 50             # 启动静止校准帧数（= 约 3s @ 17Hz；v11 CALIB_COUNT=50）
  gyro_deadzone_rad_s: 0.002  # 陀螺仪死区，低于该值视为静止；v5 imu_direct_swing 默认 0.002
  auto_calibrate_on_open: true  # SensorManager.open() 时自动启动 50 帧校准；可关（重标定则关→开）
  use_relative_subtraction: true  # True=子部件角-父部件角=相对关节角（bucket - arm - boom - swing），对齐 v8 inclinometer_reader init_angles
```

SensorManager 行为：
1. `open()` 时若 `auto_calibrate_on_open=True` → 每路 TiltCompensator 进入 `calibrating` 状态；
2. 前 `calib_count` 帧视为静止 → 累计每路 tilt 的 raw_angle_deg 均值作为 `zero_bias_deg`；
3. `calib_count` 帧后 `is_calibrated=True`，输出 `compensated_angle_deg = raw_angle_deg - zero_bias_deg`；
4. 若 `use_relative_subtraction=True`，额外在 Reading 上提供 `relative_angle_deg = compensated(child) - compensated(parent)`（父子关系 = tilt_bucket ← tilt_arm ← tilt_boom ← tilt_swing 链条，与 v8 一致）。

### FR-7 独立 TF 预标定 examples 脚本（用户 feedback：独立、非每次运行）

**定位**：只有物理换车、换传感器安装位置时**手动**运行，不进入任何正式 launch 文件；出厂默认值（FR-5）覆盖 90% 场景，不需要跑。

新增两个独立脚本，全放在 `examples/`：

| # | 脚本名 | 对标 v5 工具 | 输入 | 输出 | 运行时机 |
|---|---|---|---|---|---|
| 11 | `ex11_sensor_tf_calibration_cmdline.py` | `tf_calibration_gui.py`（无 GUI，纯命令行 6 滑条，SSH 无头兼容） | ① 指定 sensor_id（如 lidar_single_rear）② RViz 或终端手动对准 | `tf_calibration_record_<sensor_id>_<ts>.txt`，内容一行：`x y z yaw pitch roll parent child` 8 字段（与 GUI 输出格式完全兼容，static_transform_publisher 6 参数 + parent/child） | 装机、拆修、传感器移位后 |
| 12 | `ex12_sensor_tf_record_to_yaml.py` | （新，GUI → v15 YAML 转写工具） | ① ex11 / GUI 输出的 `tf_calibration_record.txt` ② 指定 chassis_sn（选填） | `extrinsics_<chassis_sn>.yaml`，内容是 v15 FR-5 `extrinsics:` 节完整 YAML，可直接 `load_config(extrinsics_override=...)` 或 merge 到整机 YAML | ex11 跑完 / GUI 保存 tf_calibration_record.txt 后 |

约束（FR-7.1 ~ FR-7.3）：
- FR-7.1 两脚本不 import v0~v14（AC-6 同样适用于 examples，grep 空匹配）。
- FR-7.2 两脚本不 import `v15_action_task.control_core`（只依赖 config/loader 解析 YAML + 纯 Python 命令行 argparse），避免拖入 rclpy。
- FR-7.3 ex12 输出的 YAML 必须能被 `load_config(path)` 作为外参加载，且 `extrinsics.get(sensor_id)` 6 参数与输入 record.txt 逐字段误差 ≤1e-4。

### FR-4 Examples 独立目录

- **路径**：`v15_action_task/examples/`（包内子包，colcon 随包分发，OQ-5 决策）。
- **约束**：`v15_action_task/` 包源码树外（`config/control_core/motion/kinematics/action_library/sensors/` 内）不准新增任何 `ex_*.py`。
- **脚本清单**（12 脚本 + 2 辅助）：

| # | 文件名 | 功能 | ROS 依赖 |
|---|---|---|---|
| 01 | `ex01_joint_limits_3layer_contract.py` | FR-1 三层契约演示 + 自断言 | 零 |
| 02 | `ex02_four_single_joint_commands.py` | FR-2.1 4 基础布尔+增量函数 + 方向语义自断言 PASS/FAIL | 零 |
| 03 | `ex03_semantic_primitives_all_8.py` | FR-2.2 全部 8 函数 + MoveResult 误差打印 | 零 |
| 04 | `ex04_sensor_manager_mock_14routes.py` | FR-3 Mock 模式 14+ 路枚举 + get_* API 演示 | 零 |
| 05 | `ex05_sensor_manager_ros_end_to_end.py` | FR-3 ROS 模式 30s 频率打印（无 rclpy → 打印跳过 exit 0） | 可选 |
| 06 | `ex06_from_config_full_pipeline.py` | 4 需求综合：from_config(init_sensors=True) → arm_move + get_tilt 联调 | 零 |
| 07 | `ex07_kinematics_closed_loop.py` | FK→IK→FK 20 组随机闭环 0mm 误差 | 零 |
| 08 | `ex08_cartesian_mover_basic.py` | CartesianMover move / move_with_bucket | 零 |
| 09 | `ex09_action_library_task_builders.py` | build_single_dig_dump_task 等 JSON dump | 零 |
| 10 | `ex10_custom_config_long_bucket_demo.py` | L_bucket=0.50m → FK 铲尖 Δz=240.00mm 物理验证 | 零 |
| 11 | `ex11_sensor_tf_calibration_cmdline.py` | FR-7.1 无 GUI TF 6 参数命令行标定（对标 GUI） | 可选 RViz |
| 12 | `ex12_sensor_tf_record_to_yaml.py` | FR-7.2 tf_calibration_record.txt → v15 extrinsics YAML | 零 |
| - | `_util_import_helper.py` | 公共 sys.path 插入 + 零依赖导入兜底 | 辅助 |
| - | `_run_all_checks.py` | Task 6 回归聚合脚本（25+ TR 自动打分） | 辅助 |

---

## Non-Functional Requirements

### NFR-1 接口稳定性
- NFR-1.1 旧 API 零 break：`URDFController` / `MockAdapter` / `CartesianMover` / `from_config(adapter_backend="mock")` 全部兼容。
- NFR-1.2 限位默认值零修改（AC-5）。

### NFR-2 性能
- NFR-2.1 SensorHub 回调 ≤1ms（只缓存元信息，不保留大对象默认）。
- NFR-2.2 单一关节函数调用 ≤1ms（不含等待到位轮询）。
- NFR-2.3 TiltCompensator 每帧补偿 ≤0.1ms（纯算术，不分配大对象）。

### NFR-3 可配置性
- NFR-3.1 所有话题 / QoS / frame / 外参 6 参数 / 温漂 α、calib_count → 100% YAML 化，零硬编码。
- NFR-3.2 三级兜底延续：sensors/extrinsics/tilt_compensation 三节 YAML ↔ JSON ↔ BUILTIN dict 1:1 等价。

### NFR-4 可测试性
- NFR-4.1 MockSensorManager 零 ROS 可跑 11/12 examples（仅 ex05/ex11 可选 ROS）。
- NFR-4.2 每个 example 独立 `python3 examples/exXX_*.py` exit 0（除故意打印 FAIL exit 1）。

### NFR-5 分层解耦
- NFR-5.1 sensors/ 不 import control_core/motion/action_library（横切关注点独立，可单独部署到感知节点）。
- NFR-5.2 FR-7 两个标定脚本只依赖 config/loader + 标准库，不拖入控制链。

---

## Constraints

1. **C-1 真实话题/数据类型硬约束**：
   - 倾角必须订阅 `std_msgs/msg/Float64MultiArray /excavator/inclinometer_pitch_deg`（v12 bridge 真实格式）；不准改用 Imu 或自定义消息。
   - 雷达必须订阅 `sensor_msgs/msg/PointCloud2`（PaceCat M300 真实格式）。
   - 相机必须订阅 `sensor_msgs/msg/Image`（v6/v11 真实格式）。
2. **C-2 倾角地址映射硬约束**：4 路 tilt 的 `modbus_addr` / `array_index` 必须是 `0x50/0 → bucket`、`0x51/1 → arm`、`0x52/2 → boom`、`0x53/3 → swing`（AC-9 验证）。
3. **C-3 外参 6 参数顺序硬约束**：必须 `x y z yaw pitch roll`（米 + 弧度），与 static_transform_publisher CLI 参数 1:1（AC-8 验证 lidar_front 出厂值）。
4. **C-4 限位值硬约束**：joint_limits 4 关节默认值完全不动（AC-5）。
5. **C-5 零依赖硬约束**：v15 任何代码 / examples（除明确注释）不准 `import shandong.v0|v1[0-4]` 或引用其字符串（AC-6 grep 空匹配）。
6. **C-6 Ubuntu 默认 Python 3.10 零外部依赖**：无 PyYAML 回落到 JSON/BUILTIN dict；无 rclpy 回落到 Mock；examples 10/12 exit 0。

---

## Assumptions

1. **A-1 v12 inclinometer_sensor_bridge 已在独立节点运行**：v15 SensorManager 只订阅其 ROS 话题（不实现 MODBUS 驱动），ZeroMQ/串口驱动层不进入 v15。
2. **A-2 PaceCat M300 ROS 2 driver 独立运行**：发布 `/pointcloud` PointCloud2；v15 不 re-export pacecat_m300_driver。
3. **A-3 相机独立节点（海康 USB / RTSP network）已发布 Image**：v15 不驱动硬件，只订阅。
4. **A-4 语义关节顺序**：维持 `SEMANTIC_JOINT_ORDER = ["swing_yaw", "boom_swing", "arm_boom", "bucket_arm"]`。
5. **A-5 FR-7 标定由人工确认对准**：ex11 是手动滑条，不做自动特征配准；自动点云外参标定属于 Non-Goals。
6. **A-6 tf_calibration_record.txt 每行格式**：`x y z yaw_rad pitch_rad roll_rad parent_frame child_frame`（8 空格分隔字段，与 v5 GUI 输出完全一致）。

---

## Open Questions（首次 5 OQ 已在 tasks.md 决策日志解答，剩余未决 2 条，本次默认决策可推翻）

1. **OQ-6** 子部件-父部件相对角（FR-6 `use_relative_subtraction=True`）是在 SensorManager 内部计算还是交给业务层？→ 本次默认：**SensorManager 内置**（v8/v12 都重复实现，横切统一）。
2. **OQ-7** 雷达点云外参变换（lookup_transform + do_transform_cloud）是否作为 SensorManager 可选 `transform_to(target_frame, lidar_id)` 便利 API？→ 本次默认：**不提供**（Non-Goals，用户业务层自行 tf2_ros，避免引入 tf2 硬依赖拖垮零依赖设计；仅在 ExtrinsicsConfig 暴露 `to_4x4_matrix()` 与 `to_static_transform_publisher_args()` 两个纯函数输出）。

---

## Acceptance Criteria（10 条，新增 AC-8/9/10）

### AC-1 关节限位三层契约可追溯
- **Type**: rule
- **Pass**: FR-1.2 三层代码路径 1:1 存在：
  - Config 层 `V15Config.limits.clamp_pose/check`（config/loader.py）
  - URDFController `_clamp_pose`（control_core/urdf_controller.py）
  - Action Library FR-2 8 函数发布前裁剪
- **Evidence**: 代码 diff + ex01 运行 PASS。

### AC-2 4 关节函数 rubric
- **Type**: rubric 0-2
  - 0: 缺失或签名不一致
  - 1: 4 基础函数存在但限位裁剪/阻塞逻辑不完整
  - 2: 8 函数（4+4）签名与 FR-2 表完全一致；FR-2.3 五步契约全满足；返回 MoveResult 支持 `__bool__`
- **Pass Threshold**: ≥2
- **Evidence**: ex02 / ex03 运行日志。

### AC-3 传感器订阅接口 rubric
- **Type**: rubric 0-2
  - 0: sensors 缺失或 import v0~v14
  - 1: 基本功能但 14 路不全或 config 不完整
  - 2: default_config.yaml sensors/extrinsics/tilt_compensation 三节齐全；SensorsConfig + SensorManager 覆盖 4+5+≥5 路；Mock 后端存在；Reading 含 TiltReading.compensated_angle_deg；动态 import rclpy 零崩
- **Pass Threshold**: ≥2
- **Evidence**: ex04 / ex05 日志。

### AC-4 Examples 独立目录结构
- **Type**: rule
- **Pass**: examples/ 下 12 脚本 + 2 辅助；examples 外无 ex_*.py；ex01/ex02/ex03/ex04/ex06/ex07/ex08/ex09/ex10/ex12 **零 ROS 环境下 exit 0**；ex11 无 GUI 依赖（SSH 无头可跑）。
- **Evidence**: ls + `for f in ex0* ex10 ex12; do python3 $f; done` 退出码。

### AC-5 限位默认值一致性
- **Type**: rule
- **Pass**: default_config.yaml joint_limits 四关节与当前值完全一致（不增不减不改）。
- **Evidence**: git diff 空。

### AC-6 v0~v14 零依赖
- **Type**: rule
- **Pass**: `grep -En "from|import" v15_action_task/ examples/ -r | grep -E "v0|v1[0-4]|shandong\\.v[^15]" | grep -v v15_action_task` → 空输出 0 行。
- **Evidence**: grep 结果。

### AC-7 方向语义自洽
- **Type**: rule
- **Pass**: ex02 内部自断言：boom_move(True,10) → 目标 -10°；arm_move(True,10) → +10°；bucket_move(True,10) → +10°；swing_move(True,10) → +10°。
- **Evidence**: ex02 输出 `方向语义 4/4 PASS`。

### AC-8 外参出厂默认值一致性（用户 feedback 硬验证）
- **Type**: rule
- **Pass**: `load_default_config().extrinsics.get("lidar_single_rear")` 的 6 参数 == sensors_tf.launch.py 真实值（浮点误差 ≤1e-4）：
  - x=-0.5500, y=-0.2000, z=1.2712, yaw=0.0532, pitch=0.0349, roll=3.0316
- **Evidence**: python3 -c "assert abs(load_default_config().extrinsics.get('lidar_single_rear').roll_rad-3.0316)<1e-4" exit 0。

### AC-9 倾角地址/语义映射一致性（用户 feedback 硬验证）
- **Type**: rule
- **Pass**: `load_default_config().sensors.tilt_sensors` 4 条严格：
  - `id=tilt_bucket → modbus_addr="0x50" → array_index=0 → semantic_joint=bucket_arm`
  - `tilt_arm → 0x51 → 1 → arm_boom`
  - `tilt_boom → 0x52 → 2 → boom_swing`
  - `tilt_swing → 0x53 → 3 → swing_yaw`
- **Evidence**: python3 -c "断言 4 条" exit 0。

### AC-10 独立标定脚本闭环（用户 feedback 硬验证）
- **Type**: rule
- **Pass**: 三步脚本闭环：
  1. 手写 `tf_calibration_record_test.txt` 内容一行：`-0.5500 -0.2000 1.2712 0.0532 0.0349 3.0316 base_link lidar_single_rear_link`
  2. `python3 examples/ex12_sensor_tf_record_to_yaml.py --input tf_calibration_record_test.txt --sn TEST01 --out extrinsics_TEST01.yaml`
  3. `cfg = load_config(extrinsics_override="extrinsics_TEST01.yaml"); assert abs(cfg.extrinsics.get("lidar_single_rear").roll_rad - 3.0316) < 1e-4` 成功
- **Evidence**: 三步命令全 exit 0。
