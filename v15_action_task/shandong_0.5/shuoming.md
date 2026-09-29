# V15 shandong_0.5 

**项目名称**: shandong_ws 挖掘机自动化控制  
**版本**: shandong_0.5（基于 v15_action_task）  

---

## 一、系统架构总览

系统分为 **4 层**，自上而下：

```
ROS2 话题层 (7 个订阅话题)
       │
  ┌────┴────────────────────────────┐
  │  main.py  ——  V15MainNode       │  ← 节点层（回调分发 + 防并发）
  └────┬────────────────────────────┘
       │
  ┌────┴────────────────────────────┐
  │  control_lib/                   │  ← 算法层（3 个控制库）
  │  ├─ dig_control.py   阶段控制    │
  │  ├─ point_control.py  点位控制   │
  │  └─ angle_control.py  关节控制   │
  └────┬────────────────────────────┘
       │
  ┌────┴────────────────────────────┐
  │  v11_control/                   │  ← 硬件桥接层
  │  └─ v11_sensor_bridge.py        │
  └─────────────────────────────────┘
       │
  V11 GUI (CAN 控制器 + 传感器 + 液压闭环)
```

---

## 二、ROS 话题 → 回调函数 → 底层函数 完整映射

### 2.1 话题 /excavator/target_pose — 角度控制

| 层级 | 脚本 | 行号 | 函数/方法 | 功能说明 |
|:---|:---|:---:|:---|:---|
| 节点订阅 | [main.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L97) | L97 | `create_subscription(JointState, '/excavator/target_pose', ...)` | 订阅目标关节角 |
| 回调 | [main.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L324-L343) | L324 | `V15MainNode.target_pose_cb(msg)` | 解析 JointState 中 name[] + position[]（度），映射为 4 语义字典，开线程执行 |
| 适配 | [main.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L44-L47) | L44 | `V11ControllerAdapter.set_pose(pose_deg)` | 将语义角度字典转为 ROS JointState 弧度消息并发布到 V11 |
| 核心执行 | [angle_control.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/angle_control.py#L183-L238) | L183 | `JointAngleController.move_pose_serial(target_pose_deg)` | 按 swing→boom→arm→bucket 顺序逐关节串行运动，每步只调 `move_joint` |
| 单关节执行 | [angle_control.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/angle_control.py#L70-L178) | L70 | `JointAngleController.move_joint(joint_name, target_deg)` | 读当前 4 关节 → 改 1 格 → clamp_pose 限位夹紧 → set_single_joint 下发 → 轮询 is_at_pose 到位 |

---

### 2.2 话题 /excavator/target_point — 空间点控制

| 层级 | 脚本 | 行号 | 函数/方法 | 功能说明 |
|:---|:---|:---:|:---|:---|
| 节点订阅 | [main.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L93) | L93 | `create_subscription(PointStamped, '/excavator/target_point', ...)` | 订阅目标点 (x,y,z) |
| 回调 | [main.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L303-L322) | L303 | `V15MainNode.target_point_cb(msg)` | 提取 xyz，WorkspaceChecker 预检可达性，不可达直接拒绝，可达则开线程调 point_mover |
| 核心执行 | [point_control.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/point_control.py#L156-L270) | L156 | `SingleJointPointMover.move_to_point_serial(xyz)` | 5 步流程：WS 预检 → auto-closest 替换 → IK 搜索铲斗角 → clamp_pose 限位 → 调 `move_pose_serial` 4 段串行 → FK 闭环校验 ≤5cm |

---

### 2.3 话题 /excavator/cmd_dig_point — 阶段 1 挖掘

| 层级 | 脚本 | 行号 | 函数/方法 | 功能说明 |
|:---|:---|:---:|:---|:---|
| 节点订阅 | [main.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L94) | L94 | `create_subscription(PointStamped, '/excavator/cmd_dig_point', ...)` | 订阅挖掘目标点 |
| 回调 | [main.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L110-L131) | L110 | `V15MainNode.cmd_dig_cb(msg)` | 提取 xyz，WS 预检可达性，调 `dig_ctrl.plan_and_execute_dig` |
| 核心执行 | [dig_control.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L267-L375) | L267 | `DigPhase1Controller.plan_and_execute_dig(xyz)` | **7 步挖掘剧本**：① WS 预检 ② IK 求解预位+目标 ③ 抬臂到安全高度 1.2m ④ 预位对准 ⑤ 下探到挖掘深度 ⑥ 收斗 75% ⑦ 收小臂垂直地面 → 收斗 100% |
| 辅助执行 | [dig_control.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L241-L265) | L241 | `DigPhase1Controller._run_sequence(sequence)` | 通用序列执行器：遍历 [(joint, target_deg, desc)] 列表，逐个调 `angle_ctrl.move_joint` |
| 限位钳制 | [dig_control.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L85-L97) | L85 | `DigPhase1Controller._clamp(joint, value_deg)` | 按关节限位钳制角度，防止超范围指令下发 |

---

### 2.4 话题 /excavator/cmd_transit_yaw — 阶段 2 运输回转

| 层级 | 脚本 | 行号 | 函数/方法 | 功能说明 |
|:---|:---|:---:|:---|:---|
| 节点订阅 | [main.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L95) | L95 | `create_subscription(PointStamped, '/excavator/cmd_transit_yaw', ...)` | 订阅回转角度 |
| 回调 | [main.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L133-L158) | L133 | `V15MainNode.cmd_transit_cb(msg)` | 从 point.x 提取回转角度（度），point.y/z 忽略，开线程调 `dig_ctrl.plan_and_execute_transit` |
| 核心执行 | [dig_control.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L382-L422) | L382 | `DigPhase1Controller.plan_and_execute_transit(yaw_deg)` | **3 步剧本**：① 抬大臂到 0°（最高点）② 收小臂到 85°（稳料姿态）③ 回转 swing_yaw 到目标角度 |

---

### 2.5 话题 /excavator/cmd_dump_point — 阶段 3 卸料

| 层级 | 脚本 | 行号 | 函数/方法 | 功能说明 |
|:---|:---|:---:|:---|:---|
| 节点订阅 | [main.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L96) | L96 | `create_subscription(PointStamped, '/excavator/cmd_dump_point', ...)` | 订阅卸料目标点 |
| 回调 | [main.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L160-L188) | L160 | `V15MainNode.cmd_dump_cb(msg)` | 提取 xyz，**boom 强制 0° 不动**，开线程调 `dig_ctrl.plan_and_execute_dump` |
| 核心执行 | [dig_control.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L424-L573) | L424 | `DigPhase1Controller.plan_and_execute_dump(dump_xyz)` | **4 步剧本**：① z≥0.3m 兜底 ② WS 预检 + IK 分段搜索 boom 最高解 ③ 回转对齐 → 小臂外推 45% → 铲斗完全打开 -95° ④ 开斗卸料 → 回正准备位 |

---

### 2.6 话题 /excavator/cmd_full_cycle — 全流程 A（挖掘→回转→卸料→回正）

| 层级 | 脚本 | 行号 | 函数/方法 | 功能说明 |
|:---|:---|:---:|:---|:---|
| 节点订阅 | [main.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L102-L103) | L102 | `create_subscription(Float64MultiArray, '/excavator/cmd_full_cycle', ...)` | 订阅 6 元素数组 [dig_xyz(3) + dump_xyz(3)] |
| 回调 | [main.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L193-L247) | L193 | `V15MainNode.cmd_full_cycle_cb(msg)` | 解析 6 个 float，dump_z<0.5 强制=0.5，开线程调 `dig_ctrl.execute_full_cycle_dig_dump` |
| 核心执行 | [dig_control.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L576-L667) | L576 | `DigPhase1Controller.execute_full_cycle_dig_dump(dig_xyz, dump_xyz)` | **4 阶段串起**：① `plan_and_execute_dig` ② 自动算 swing=atan2(dy,dx) + `plan_and_execute_transit` ③ `plan_and_execute_dump` ④ 回正（boom→0° / arm→80° / bucket→-30° / swing→0°） |

---

### 2.7 话题 /excavator/cmd_dig_transit — 全流程 B（挖掘→回转→自动推卸料→回正）

| 层级 | 脚本 | 行号 | 函数/方法 | 功能说明 |
|:---|:---|:---:|:---|:---|
| 节点订阅 | [main.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L104-L105) | L104 | `create_subscription(Float64MultiArray, '/excavator/cmd_dig_transit', ...)` | 订阅 4 元素数组 [dig_xyz(3) + transit_yaw_deg] |
| 回调 | [main.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L253-L301) | L253 | `V15MainNode.cmd_dig_transit_cb(msg)` | 解析 4 个 float，开线程调 `dig_ctrl.execute_dig_and_transit` |
| 核心执行 | [dig_control.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L669-L740) | L669 | `DigPhase1Controller.execute_dig_and_transit(dig_xyz, yaw_deg)` | **4 阶段串起**：① `plan_and_execute_dig` ② `plan_and_execute_transit(yaw)` ③ 自动推 dump_xyz=(1.2·cosyaw, 1.2·sinyaw, ≥0.5m) + `plan_and_execute_dump` ④ 回正 |

---

## 三、底层控制库函数索引

### 3.1 库 1：angle_control.py — 关节角度控制

| 行号 | 函数/方法 | 功能说明 |
|:---:|:---|:---|
| [L42](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/angle_control.py#L42) | `class JointAngleController` | 单关节安全控制器，包装 adapter 实现「任意时刻只动 1 个关节」 |
| [L70](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/angle_control.py#L70-L178) | `move_joint(joint_name, target_deg, ...)` | **核心原子操作**：读当前 4 关节 → 改 1 格 → clamp 限位 → 单关节下发 → 轮询到位（容差 5°/超时 25~30s） |
| [L183](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/angle_control.py#L183-L238) | `move_pose_serial(target_pose_deg, ...)` | 将 4 关节目标按 swing→boom→arm→bucket 顺序逐个调 `move_joint`，stop_on_first_fail |
| [L243](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/angle_control.py#L243-L260) | `_get_current_pose()` | 从 adapter 读当前 4 关节角度（度），兼容多种 ctl 接口 |

### 3.2 库 2：point_control.py — 空间点位控制

| 行号 | 函数/方法 | 功能说明 |
|:---:|:---|:---|
| [L74](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/point_control.py#L74) | `class SingleJointPointMover` | Point→IK→4 段串行执行器 |
| [L113](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/point_control.py#L113-L148) | `from_config_default()` | 从 default_config.yaml 加载 FK/IK/WS/限位，推荐构建方式 |
| [L150](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/point_control.py#L150-L154) | `bind_controller(angle_ctrl)` | 绑定库 1 的 JointAngleController，作为底层执行器 |
| [L156](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/point_control.py#L156-L270) | `move_to_point_serial(xyz, ...)` | **核心流程**：WS 5 层预检 → closest 替换 → IK 自动搜索铲斗角(-95°,45°,N=33) → clamp 限位 → `move_pose_serial` → FK 闭环 ≤5cm |

### 3.3 库 3：dig_control.py — 阶段控制

| 行号 | 函数/方法 | 功能说明 |
|:---:|:---|:---|
| [L37](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L37) | `class DigPhase1Controller` | 挖掘/运输/卸料全流程控制器，依赖库 1 + v15 IK/WS |
| [L85](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L85-L97) | `_clamp(joint, value_deg)` | 按关节限位钳制角度（swing±180 / boom 0~48 / arm 0~130 / bucket -95~45） |
| [L114](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L114-L236) | `set_arm_perpendicular(timeout_s)` | 传感器闭环：步进 ±5° 调小臂到对地垂直（abs_pitch≈90°），收敛阈值 ±5° |
| [L241](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L241-L265) | `_run_sequence(sequence)` | 通用序列执行器：[(joint, target, desc)] 逐个调 `angle_ctrl.move_joint` |
| [L267](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L267-L375) | `plan_and_execute_dig(xyz)` | **阶段 1**：7 步挖掘剧本（WS 预检→IK→抬臂→预位→下探→收斗75%→收臂→收斗100%） |
| [L382](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L382-L422) | `plan_and_execute_transit(yaw_deg)` | **阶段 2**：3 步运输剧本（抬大臂 0°→收小臂 85°稳料→回转） |
| [L424](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L424-L573) | `plan_and_execute_dump(dump_xyz)` | **阶段 3**：4 步卸料剧本（boom 锁 0°→IK 分段搜索→回转对齐→外推小臂→开斗 -95°） |
| [L576](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L576-L667) | `execute_full_cycle_dig_dump(dig, dump)` | **全流程 A**：挖掘→自动回转 atan2(dy,dx)→卸料→回正 |
| [L669](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L669-L740) | `execute_dig_and_transit(dig, yaw_deg)` | **全流程 B**：挖掘→回转→自动推卸料点(1.2m×cos/sin)→卸料→回正 |

---

## 四、硬件桥接层函数索引

### 4.1 v11_control/v11_sensor_bridge.py

| 行号 | 函数/方法 | 功能说明 |
|:---:|:---|:---|
| [L160](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/v11_control/v11_sensor_bridge.py#L160) | `class V11SensorBridge` | ROS2 传感器桥接器，订阅 v11 传感器数据 + 提供反向控制 msg factory |
| [L345](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/v11_control/v11_sensor_bridge.py#L345-L370) | `get_current_sensor_data(blocking, timeout_s)` | 获取 V4 14 路倾角传感器最新数据帧（大臂/小臂/铲斗绝对 pitch + IMU yaw） |
| [L466](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/v11_control/v11_sensor_bridge.py#L466-L505) | `_on_joint_states(msg)` | 回调 /excavator/joint_states（20Hz），name 对齐 boom/arm/bucket/swing → 弧度转度 → 缓存 |
| [L507](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/v11_control/v11_sensor_bridge.py#L507-L532) | `_on_sensor_data(msg)` | 回调 /excavator/sensor_data（Float64MultiArray，V4 14 路绝对倾角） |
| [L534](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/v11_control/v11_sensor_bridge.py#L534-L570) | `_on_lidar_points(msg)` | 回调 /lidar/points（10Hz），PointCloud2 16B 步长解包为 Nx3 float32 xyz + Nx3 uint8 RGB |

### 4.2 main.py — V11ControllerAdapter

| 行号 | 函数/方法 | 功能说明 |
|:---:|:---|:---|
| [L26](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L26) | `class V11ControllerAdapter` | V11 硬件控制适配器，桥接 sensor_bridge + JointAngleController |
| [L32](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L32-L42) | `get_pose_or_default()` | 从 V11 传感器读当前 4 关节角度（度），无数据则返回默认值 |
| [L44](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L44-L47) | `set_pose(pose_deg)` | 语义角度字典→ROS JointState 弧度→发布 cmd/joint_states→V11 CAN 执行 |
| [L49](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L49-L65) | `set_single_joint(joint_name, target_deg)` | **严格单一动作**：只下发 1 个关节角度（其他 3 个不动），防止复合动作 |
| [L67](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L67-L72) | `get_current_sensor_data(...)` | 桥接 V11SensorBridge 的 V4 14 路绝对倾角传感器数据 |

---

## 五、关节限位与物理量程

| 关节 | 最小值 | 最大值 | 方向语义 | 对应 URDF |
|:---|:---:|:---:|:---|:---|
| swing_yaw | -180° | +180° | + = 顺时针（从上往下看） | swing_joint |
| boom_swing | **0°** | **48°** | 0=最高点（完全抬起）；48=最低点（压地面） | boom_joint |
| arm_boom | 0° | 130° | 0=伸直；增大=小臂内收 | arm_joint |
| bucket_arm | -95° | +45° | - = 向外开斗卸料；+ = 向内卷斗装料 | bucket_joint |

**60FED 连杆**: 大臂=0.88m, 小臂=0.44m, 铲斗=0.26m  
**物理极限**: 最远 1.765m / 最高 1.915m / 最低下挖 -0.261m

---

## 六、话题接口汇总表

| 话题 | 消息类型 | 回调函数 | 核心执行函数 | 安全约束 |
|:---|:---|:---|:---|:---|
| `/excavator/target_pose` | JointState | `target_pose_cb` | `move_pose_serial` | 逐关节串行，每步 clamp 限位 |
| `/excavator/target_point` | PointStamped | `target_point_cb` | `move_to_point_serial` | WS 预检 + FK 闭环 ≤5cm |
| `/excavator/cmd_dig_point` | PointStamped | `cmd_dig_cb` | `plan_and_execute_dig` | WS 预检 + 7 步剧本 |
| `/excavator/cmd_transit_yaw` | PointStamped | `cmd_transit_cb` | `plan_and_execute_transit` | 3 步：抬臂→稳料→回转 |
| `/excavator/cmd_dump_point` | PointStamped | `cmd_dump_cb` | `plan_and_execute_dump` | boom 锁 0° + z≥0.3m |
| `/excavator/cmd_full_cycle` | Float64MultiArray(6) | `cmd_full_cycle_cb` | `execute_full_cycle_dig_dump` | 4 阶段串起 + dump_z 下限 |
| `/excavator/cmd_dig_transit` | Float64MultiArray(4) | `cmd_dig_transit_cb` | `execute_dig_and_transit` | 4 阶段串起 + 自动推卸料点 |