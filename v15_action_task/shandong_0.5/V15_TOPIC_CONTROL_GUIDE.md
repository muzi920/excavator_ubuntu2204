# V15 实体挖掘机 ROS 2 话题控制测试指南

本文档说明了如何通过 ROS 2 话题发布指令，来测试 `shandong_0.5` 目录下的真机控制节点 (`main.py`)。
该节点在底层实现了 **单关节串行安全控制**（即：动作依次执行，不会出现多个关节同时运动的复合动作），以确保挖掘机实体的安全。

---

## 1. 系统启动顺序

在发送控制指令前，请确保系统已按以下顺序启动：

1. **启动 V11 多模态 GUI (底层驱动)**
   负责连接 CAN 控制器和传感器，并监听 `cmd/joint_states`。
   ```bash
   cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
   source /opt/ros/humble/setup.bash
   python3 src/shandong/v11_multimodal_dataset_collection/ros2_multimodal_gui.py
   ```

2. **启动 V15 主控制节点 (算法与指令转换层)**
   负责运行运动学/逆运动学(FK/IK)、可达域检查，并接收外部话题指令。
   ```bash
   cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   python3 src/shandong/v15_action_task/shandong_0.5/main.py
   ```

---

## 2. 运行状态 topic

主控制节点会对外发布当前执行状态，方便你在上位机、调度脚本或
测试终端中判断设备是否空闲。只要节点接受到任意动作类指令并开始
执行，状态就会切换到 `working`；动作结束后会恢复为 `idle`。

- **发布话题**: `/excavator/state`
- **消息类型**: `std_msgs/msg/String`
- **状态说明**:
  - `idle`: 空闲状态，可以接收新的动作指令。
  - `working`: 正在执行动作，新的动作类 topic 会被拒绝。

动作类 topic 包括：
- `/excavator/target_pose`
- `/excavator/target_point`
- `/excavator/cmd_dig_point`
- `/excavator/cmd_transit_yaw`
- `/excavator/cmd_dump_point`
- `/excavator/cmd_full_cycle`
- `/excavator/cmd_dig_transit`
- `/perception/soil_highest_points`

### 📝 测试指令示例

**示例 1：监听当前状态**
```bash
ros2 topic echo /excavator/state
```

**示例 2：先发动作，再观察状态切换**
```bash
ros2 topic pub --once /excavator/cmd_dig_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'base_link'}, point: {x: 1.2, y: 0.0, z: 0.0}}"
```

预期现象：
1. 节点空闲时，`/excavator/state` 发布 `idle`。
2. 接受动作指令后，`/excavator/state` 立即切换为 `working`。
3. 若此时再次发送新的动作类指令，主节点会拒绝执行，并在日志中打印
   当前忙碌状态提示。
4. 当前动作结束后，`/excavator/state` 恢复为 `idle`。

---

## 3. 角度控制 (Joint Pose Control)

直接给定挖掘机四个关节的目标角度。节点接收到后，会按照 `回转 -> 大臂 -> 小臂 -> 铲斗` 的安全顺序，逐个单关节移动到目标位置。

- **订阅话题**: `/excavator/target_pose`
- **消息类型**: `sensor_msgs/msg/JointState`
- **参数说明**:
  - `name`: 关节名称数组，支持 `['boom_joint', 'arm_joint', 'bucket_joint', 'swing_joint']`。
  - `position`: 目标角度数组，**单位为度 (Degrees)**。

### 📝 测试指令示例

**示例 1：将小臂和大臂都移动到 90 度**
```bash
ros2 topic pub --once /excavator/target_pose sensor_msgs/msg/JointState "{name: ['boom_joint', 'arm_joint'], position: [90.0, 90.0]}"
```

**示例 2：控制所有四个关节**
*(大臂90度, 小臂90度, 铲斗90度, 回转回正到0度)*
```bash
ros2 topic pub --once /excavator/target_pose sensor_msgs/msg/JointState "{name: ['boom_joint', 'arm_joint', 'bucket_joint', 'swing_joint'], position: [90.0, 90.0, 90.0, 0.0]}"
```

---

## 4. 空间点控制 (Spatial Point Control) - 移动至指定点

给定挖掘机铲尖在 `base_link` (车体中心) 坐标系下的目标 `(x, y, z)` 空间坐标。
此命令仅计算一个可行姿态并控制铲尖**移动到该点**，**不**包含挖掘动作序列。
如果该点超出挖掘机的工作范围（见文末运动包络参数），系统将拒绝执行并打印 `error` 报错。

- **订阅话题**: `/excavator/target_point`
- **消息类型**: `geometry_msgs/msg/PointStamped`
- **参数说明**:
  - `header.frame_id`: 推荐写 `'base_link'`。
  - `point.x`: 挖掘机正前方的距离（米）。
  - `point.y`: 挖掘机左右的距离（米，左正右负）。
  - `point.z`: 挖掘机上下高度（米，以履带接地或基座为准，根据实际标定）。

### 📝 测试指令示例

**示例 1：控制铲尖移动到正前方 1.0米，高度 0.5米 的位置**
```bash
ros2 topic pub --once /excavator/target_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'base_link'}, point: {x: 1.0, y: 0.0, z: 0.5}}"
```

---

## 5. 自动挖掘动作序列 (Automatic Digging Sequence) - 阶段 1

给定一个挖掘目标点 `(x, y, z)`，系统将自动规划并执行完整的**单点挖掘动作序列**（阶段1）。
内部流程：
1. **预检与逆解**：校验可达域（超出范围自动找最近点），并在半开斗姿态下搜索最优逆解。
2. **高空越障**：在进行任何回转或下探前，**首先计算并提臂将铲斗抬升至 1m 高度**，避免刮蹭地面或越障。
3. **初始化对准**：回转对准目标 → 铲斗半开 → 小臂/大臂下探。
4. **挖掘取料**：收斗取料 → 小臂辅助收料。
（注：第一阶段到此结束。抬大臂、回转、卸料等动作属于第二、第三阶段功能）

**注意**：为保障硬件安全，以上所有动作均被严格拆解为**单关节串行**执行，绝不会并发。

- **订阅话题**: `/excavator/cmd_dig_point`
- **消息类型**: `geometry_msgs/msg/PointStamped`

### 📝 测试指令示例

**示例 1：在正前方 1.2米，高度 0.0米（地面）处执行挖掘动作**
```bash
ros2 topic pub --once /excavator/cmd_dig_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'base_link'}, point: {x: 1.2, y: 0.0, z: 0.0}}"
```

**示例 2：在侧方 (前方 0.8米，左侧 0.5米，高度 0.0米) 处执行挖掘动作**
```bash
ros2 topic pub --once /excavator/cmd_dig_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'base_link'}, point: {x: 0.8, y: 0.5, z: 0.0}}"
```

---

## 6. 感知挖掘点输入（Soil Highest Points）

这是主控制节点对感知模块暴露的自动挖掘入口。当感知/识别算法选出一个最终
挖掘点后，直接把该点发布到 `/perception/soil_highest_points`；主控拿到后
会按与 `/excavator/cmd_dig_point` 完全一致的流程走阶段一挖掘，包含：
忙闲状态互斥、可达域预检、`plan_and_execute_dig(...)` 七步挖掘剧本。

- **订阅话题**: `/perception/soil_highest_points`
- **兼容消息类型**：主控制节点对同 topic 同时订阅以下 3 种常见上游格式，
  以避免“感知侧发了但主控没回调”的静默失配问题。你任选其一即可。

  1. `geometry_msgs/msg/PointStamped`（推荐，带 frame_id）
     - `header.frame_id`: 感知源坐标系名。当前实现直接把
       `point.x/y/z` 当作 `base_link` 系使用；如需 TF 转换请在感知侧先转。
     - `point.x`: 挖掘点正前方距离（米）。
     - `point.y`: 挖掘点左右距离（米，左正右负）。
     - `point.z`: 挖掘点高度（米）。

  2. `geometry_msgs/msg/Point`（裸点，不带 header）
     - `x`, `y`, `z`: 与上面 `PointStamped.point.*` 语义完全一致。

  3. `std_msgs/msg/Float64MultiArray`（感知端最通用的数组格式）
     - `data` 长度必须 ≥ 3。前 3 个元素依次视为 `x, y, z`（米），多余元素忽略。

> 设计约定：该 topic 代表“感知最终选定的一个挖掘点”，而不是点云簇。
> 如果你的感知输出是 `sensor_msgs/msg/PointCloud2`（料堆最高点集合），
> 推荐在感知节点先选单点（取质心 / Z 最大 / 按距离选）后再以
> `PointStamped` / `Point` / `Float64MultiArray` 三种之一发布到本 topic。

### 🛠️ 常见排障：topic 存在但主控没有反应

如果 `ros2 topic list | grep soil_highest_points` 能看到 topic，但主控
`main.py` 没有任何 `[SoilHighest]` 开头的日志，**按顺序检查**：

1. **必须用 ROS Humble 对应的系统 Python 启动主控**：
   - 正确：`/usr/bin/python3` → `3.10.x`
   - 错误：`~/miniconda3/bin/python3` → `3.14.x`（conda 环境会导致
     rclpy / geometry_msgs / cv_bridge 在运行时出现反序列化、回调不触发、
     numpy ABI 不兼容等各种“看起来没反应”的问题）。
   - 启动前务必确保：
     ```bash
     export ROS_DOMAIN_ID=0
     source /opt/ros/humble/setup.bash
     which python3    # 应输出 /usr/bin/python3
     python3 --version   # 应输出 Python 3.10.x
     ```

2. **消息类型必须匹配**：在你真机上执行
   ```bash
   export ROS_DOMAIN_ID=0
   source /opt/ros/humble/setup.bash
   ros2 topic info -v /perception/soil_highest_points
   ```
   查看 `Publisher` 真正发布的类型。主控节点启动时也会把自己所有已注册
   subscription 的 topic + 消息类型打印到日志（`=== 已注册订阅清单 ===`）。
   两边类型不一致就会静默不回调。如果你确认上游是第 4 种 msg（例如
   `PointCloud2` 或自定义 msg），告诉我真实类型，我补对应的解码回调。

3. **`/excavator/state == working` 时本来就会拒绝新感知点**：
   - 感知端“持续发布”的情况下，第一帧点被接受后主控切 `working`。
   - 之后的若干帧在动作结束前都会被 busy 机制跳过，并打印节流警告：
     `[SoilHighest][...] 忙碌（state='working'），已跳过本次感知挖掘点 ...`
   - 这是期望行为。不是“没反应”。如果你希望改成“每次新来点就打断重跑”
     或“来新排队取最后一帧”，告诉我策略，我再加一个可切换的抢占模式。

### 📝 测试指令示例

**示例 1：PointStamped（推荐）**
```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
ros2 topic pub --once /perception/soil_highest_points geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'soil_perception'}, point: {x: 1.28, y: 0.0, z: 0.0}}"
```

**示例 2：裸 Point**
```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
ros2 topic pub --once /perception/soil_highest_points geometry_msgs/msg/Point \
  "{x: 1.28, y: 0.0, z: 0.0}"
```

**示例 3：Float64MultiArray([x,y,z])**
```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
ros2 topic pub --once /perception/soil_highest_points std_msgs/msg/Float64MultiArray \
  "{data: [1.28, 0.0, 0.0]}"
```

预期行为（3 个示例一致）：
1. 主控日志打印 `[SoilHighest][PointStamped|Point|Float64MultiArray] 收到感知挖掘点 ...`
2. 若当前状态为 `idle`，`/excavator/state` 切换为 `working` 并开始挖掘。
3. 阶段一挖掘完成后，`/excavator/state` 自动恢复为 `idle`。
4. 若在挖掘中再次发布该 topic，会看到显式的 busy 跳过日志（约 2 秒一条），
   不启动新挖掘，也不会把上一动作打断。

---

## 7. 运输与回转序列 (Transit Sequence) - 阶段 2

此指令用于挖掘完成后的物料转移阶段。
系统将自动执行：
1. **抬起大臂**：将大臂抬到最高点 **0°**（用户标定：boom_swing=0°=完全抬起，48°=压到地面最低）。
2. **回转**：在最高点 0° 下回转至指定的卸料角度（Swing Yaw）。
3. ★ 本阶段**不包含卸料**，卸料属于阶段 3。

**注意**：大臂/小臂/铲斗的稳料动作在此阶段默认**关闭跳过**（为了单独测试阶段二时不误导）。如果后续需要复合运输姿态可以再开启。

- **订阅话题**: `/excavator/cmd_transit_yaw`
- **消息类型**: `geometry_msgs/msg/PointStamped`
- **参数说明**:
  - `point.x`: 目标回转角度（度，Degrees）。为了消息格式统一，复用了坐标格式，**只需填写 X 值**。正数=从上往下看顺时针(CW)，负数=逆时针(CCW)。
  - `point.y / point.z`: 忽略（填 0 即可）。

### 📝 测试指令示例

**示例 1：抬大臂到 0° 最高，然后回转 +30°（顺时针右前方）**
```bash
ros2 topic pub --once /excavator/cmd_transit_yaw geometry_msgs/msg/PointStamped "{header: {frame_id: 'base_link'}, point: {x: 30.0, y: 0.0, z: 0.0}}"
```

**示例 2：抬大臂到 0° 最高，然后回转 -30°（逆时针左前方）**
```bash
ros2 topic pub --once /excavator/cmd_transit_yaw geometry_msgs/msg/PointStamped "{header: {frame_id: 'base_link'}, point: {x: -30.0, y: 0.0, z: 0.0}}"
```

---

## 8. 卸料点控制 (Dump Sequence) - 阶段 3

此指令用于到达卸料区域后，将铲尖对准卸料点并打开铲斗倒料。

**★ 与阶段 1 挖掘点严格区分（Topic 不同，约束不同）：**
- 阶段 1 挖掘点 → `/excavator/cmd_dig_point`，大臂会下探地面。
- 阶段 3 卸料点 → `/excavator/cmd_dump_point`，**大臂强制锁死在 0° 最高点（无论你传什么 z 值，内部都保持 boom_swing = 0° 不动）**。

### 执行流程（4 步，全部单关节串行）
1. **保证大臂 0°**：若当前 boom≠0°，先抬到最高点（安全越障）。
2. **回转对齐**：根据卸料点 (x,y) 的平面投影，计算 swing_yaw = atan2(y, x) 并回转。
3. **小臂+铲斗到点**：boom 固定 0°，计算 IK 使铲尖运动到 (x, y, max(z用户, 安全高度0.3m))。
   - 若 boom=0° 固定下该点仍不可达 → 直接报错返回，不动任何关节。
4. **大角度开斗卸料**：bucket_arm → -85°，把料倒出。

- **订阅话题**: `/excavator/cmd_dump_point`
- **消息类型**: `geometry_msgs/msg/PointStamped`
- **参数说明**:
  - `header.frame_id`: `base_link`。
  - `point.x`: 卸料点 X（正前方距离，米）。
  - `point.y`: 卸料点 Y（左正右负，米）。
  - `point.z`: 卸料点 Z（高度，米）。★ 仅作参考，boom 强制=0° 不动；若 z<0.3m 会被抬升到 0.3m 防止砸地。

### 📝 测试指令示例

**示例 1：在正前方 1.0米，高度 1.0米 处卸料**
```bash
ros2 topic pub --once /excavator/cmd_dump_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'base_link'}, point: {x: 1.0, y: 0.0, z: 1.0}}"
```

**示例 2：在右前方 1.2米，高度 0.5米 处卸料（故意给 z 很低）**
```bash
# 注意：z=0.2m 会被内部抬到 0.3m（安全高度），但大臂仍然保持 0° 不动，只用小臂/铲斗补偿
ros2 topic pub --once /excavator/cmd_dump_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'base_link'}, point: {x: 1.2, y: -0.4, z: 0.2}}"
```

---

## 8. 高级全流程组合（高级 API）

前面的 §3/4/5/6/7 都是「单阶段话题」，每次只触发一个阶段。当你
已经调好了单阶段的参数，想让整机自动运行一个完整循环时，直接用
下面两个「组合话题」即可。**一次发布 = 自动串起多个阶段**，中间
失败会立即中止。

---

### 7.1 类型 A：全流程闭环（挖掘 → 回转 → 卸料 → 回正）

一次发布，自动执行 4 大步：
1. **挖掘**（调用 cmd_dig_point 相同逻辑）
2. **自动回转**（根据卸料点 (x,y) 平面方向算 swing_yaw = atan2(y, x)，然后走阶段 2）
3. **卸料**（阶段 3，boom 取最高可达姿态）
4. **回正**（下一目标点准备位：boom→0° 最高，arm→80° 微收，bucket→-30° 半开，swing→0° 正前方）

- **订阅话题**: `/excavator/cmd_full_cycle`
- **消息类型**: `std_msgs/msg/Float64MultiArray`
- **参数顺序**（`data` 数组，大小必须 = 6）：

| index | 名称 | 含义 | 单位 |
|:---:|:---|:---|:---:|
| 0 | `dig_x`   | 挖掘点 X（正前方） | 米 |
| 1 | `dig_y`   | 挖掘点 Y（左正右负） | 米 |
| 2 | `dig_z`   | 挖掘点 Z（高度） | 米 |
| 3 | `dump_x`  | 卸料点 X | 米 |
| 4 | `dump_y`  | 卸料点 Y | 米 |
| 5 | `dump_z`  | 卸料点 Z（**用户指定规则**：`<1.0m → 强制 1.0m`；`≥1.0m → 原值`） | 米 |

#### 📝 测试指令示例

**示例 1：dig=(1.0, 0, 0.1)m 正前下挖，dump=(1.2, -0.4, 0.5)m 右前卸料（z=0.5 < 1 → 被强制=1.0）**
```bash
ros2 topic pub --once /excavator/cmd_full_cycle std_msgs/msg/Float64MultiArray \
  "{layout: {dim: [{label: 'dig_dump', size: 6, stride: 6}], data_offset: 0}, \
    data: [1.0, 0.0, 0.1,  1.2, -0.4, 0.5]}"
```
日志里会打印：
```
[FullCycle A] dump_z 规则判断: 原始 z=0.500m  →  使用 z=1.000m  (规则: z<1→1.0, z>=1→原值)
```

**示例 2：dig=(1.2, 0.3, 0.1)m 左前下挖，dump=(0.8, -0.6, 1.5)m 右前高卸料（z=1.5≥1，保留 1.5）**
```bash
ros2 topic pub --once /excavator/cmd_full_cycle std_msgs/msg/Float64MultiArray \
  "{layout: {dim: [{label: 'dig_dump', size: 6, stride: 6}], data_offset: 0}, \
    data: [1.2, 0.3, 0.1,  0.8, -0.6, 1.5]}"
```

---

### 7.2 类型 B：完整闭环（挖掘 → 回转指定角度 → 自动推卸料点 → 回正）

**用户输入只给：挖掘点 (x,y,z) + 一个回转角度。**  
卸料点 **不需要用户输入**——系统在回转到指定角度后，**自动按「该角度正前方 1.2m + 高度 ≥1.0m」推导一个卸料点**并卸料，最后自动回正，整个流程**一次发布一步到位不用分步**。

一次发布，自动执行 4 大步：
1. **挖掘**（同阶段 1）
2. **运输回转**（同阶段 2，旋转到你指定的角度）
3. **自动推卸料点并卸料**（阶段3）
   - 推导公式：`dump_x = 1.2 * cos(yaw)`，`dump_y = 1.2 * sin(yaw)`，`dump_z = max(1.0, 用户设置下限)`
4. **回正**（下一目标点准备位：boom→0° 最高 / arm→80° / bucket→-30° / swing→0°）

- **订阅话题**: `/excavator/cmd_dig_transit`
- **消息类型**: `std_msgs/msg/Float64MultiArray`
- **参数顺序**（`data` 数组，大小必须 = 4）：

| index | 名称 | 含义 | 单位 |
|:---:|:---|:---|:---:|
| 0 | `dig_x`           | 挖掘点 X | 米 |
| 1 | `dig_y`           | 挖掘点 Y | 米 |
| 2 | `dig_z`           | 挖掘点 Z | 米 |
| 3 | `transit_yaw_deg` | 回转角度（正=顺时针从上往下看） | 度 |

#### 📝 测试指令示例

**示例 1：dig=(1.0, 0, 0.1)m，挖完后回转 -45°（左前方）→ 自动在 -45°正前 1.2m / 高 1.0m 处卸料 → 回正**
```bash
ros2 topic pub --once /excavator/cmd_dig_transit std_msgs/msg/Float64MultiArray \
  "{layout: {dim: [{label: 'dig_transit', size: 4, stride: 4}], data_offset: 0}, \
    data: [1.0, 0.0, 0.1, -45.0]}"
```

日志关键信息（观察卸料推导）：
```
[FULL-CYCLE B] === [3/4] 阶段 3：自动推卸料点卸料
            自动推导: swing=-45.0° → 正前 1.20m × 高度 1.00m
            得到 dump_xyz = ( 0.85, -0.85, 1.00) m  ===
```

---

## 9. 附录：60FED 实体模型运动包络范围 & 关节限位（最新版）

### 关节限位（★ 用户现场标定，覆盖 v10 默认值）
| 关节名 (semantic) | 最小值 | 最大值 | 方向语义说明 |
|:---|:---:|:---:|:---|
| `swing_yaw`  | -180° | +180° | 从上往下看，正=顺时针(CW) |
| `boom_swing` | **0°** | **48°** | **0°=大臂最高点(完全抬起)；48°=大臂最低点(压地面)** |
| `arm_boom`   |   0°  | 130°  | 0°=小臂与大臂伸直；增大=小臂向内收 |
| `bucket_arm` | -95°  | +45°  | 负值=向外开斗卸料；正值=向内卷斗装料 |

### 绝对物理极限量程（60FED 连杆标定计算值）
根据 `default_config.yaml` 的连杆长度 (大臂=0.88m, 小臂=0.44m, 铲斗=0.26m) 及限位配置计算，该型挖掘机在 `base_link` (回转中心, X向前方, Z向垂直上方) 坐标系下：

- **最远可达距离 (Max Reach X)**: `1.765 米` (此时高度为 `0.383 m`)
- **最近可达距离 (Min Reach X)**: `-0.018 米` (小臂折叠贴紧车身)
- **最高举升高度 (Max Height Z)**: `1.915 米` (此时前方距离为 `0.276 m`)
- **最大下挖深度 (Max Depth Z)**: `-0.261 米` (此时前方距离为 `1.138 m`)
- **★ 阶段 3 卸料（boom=0° 固定）臂展**: 约 `0.6m ~ 1.5m`（平面距离 d），高度约 `0.8m ~ 1.6m`（具体随小臂伸屈变化）。

*注意：实际在控制中，系统会在极值边缘保留约 `0.02m` 的缓冲安全余量(margin)。如果指令点位于上述包络范围外，系统将直接拦截拒绝执行。*

---

## 12. 运行机制与注意事项

1. **防并发保护**: 
   如果你在某个指令还未执行完毕时发送了新指令，节点会保持
   `/excavator/state = working`，并拒绝新的动作类 topic。终端日志会
   打印类似 `当前状态为 working，拒绝新指令` 的提示。这是为了防止动作
   冲突导致机械臂失控。
2. **串行单关节**: 
   在执行任何控制时，你会看到机器先旋转、停顿，然后动大臂、停顿，再动小臂... 绝对不会有两个关节同时运动。
3. **容差过滤**: 
   底层已经设置了过滤机制，只有目标角度与当前角度差值大于 `1.0°` 时才会真正触发 CAN 指令下发，避免微小抖动引发的无效动作。
