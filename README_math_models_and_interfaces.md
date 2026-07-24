# 公式、数学模型与接口说明

这份文档汇总 `src/shandong` 目录下和后续方案设计最相关的四类内容：

1. 倾角传感器与雷达 IMU 的预积分处理，以及积分漂移修正方法。
2. 挖掘机铲斗轨迹规划、逆向运动学求解，以及从挖掘点生成轨迹的数学模型。
3. 多点轨迹的实现方法与任务拼接逻辑。
4. `v14_urdf` 的 ROS 接口与 `v4`、`v7`、`v10`、`v11`、`v12` 的区别。

这份文档的定位不是“操作手册”，而是后续做 `mode2`、多点规划、接口重构时可直接
引用的理论和架构说明。

## 0. 课题定义与技术路线

这一节把前面的公式、接口和代码整理成更接近“课题说明”的表述。这样你后续无论是做
`mode2`、写阶段总结，还是继续拆分子方向，都可以直接从这一节往下展开。

### 0.1 课题背景

当前项目的真实目标不是单一的“让挖掘机动起来”，而是要建立一条完整链路：

```text
传感器姿态估计
-> 点云/工作区域几何表达
-> 单点与多点轨迹规划
-> URDF / RViz 仿真验证
-> 实机控制语义对接
```

这条链路的难点在于，系统同时包含：

- 实机控制
- 传感器异步采集
- 几何建模
- 逆向运动学
- 多点任务调度
- ROS 2 / URDF 仿真接口

因此这个课题本质上不是单纯的控制问题，也不是单纯的感知问题，而是一个“感知-几何-
规划-执行”统一闭环问题。

### 0.2 课题目标

如果把整个 `src/shandong` 当前的工作收敛成一个统一课题，可以表述为：

> 构建面向挖掘机自动作业的多传感器感知、姿态补偿、几何建模、逆向运动学求解与多点
> 轨迹规划一体化方法，并通过 URDF 与 ROS 2 接口完成仿真验证及实机接口映射。

进一步拆开，当前课题的直接目标包括：

1. 稳定解算车体姿态和回转角，抑制 IMU / 雷达安装误差带来的累计漂移。
2. 建立可复用的机械臂几何模型，把空间目标点转成可执行的关节目标。
3. 建立从单点挖掘到多点连续挖掘的统一轨迹生成框架。
4. 建立从实机控制语义到 URDF 仿真接口的统一映射层。
5. 为后续 `mode2` 的密集工作区域生成和人工点选提供数学模型基础。

### 0.3 课题核心问题

目前这个课题最关键的研究问题可以归纳为四个主问题。

#### 问题一：倾斜安装与高频异步条件下的姿态稳定估计

在真实挖掘机上：

- 雷达通常不是完美水平安装
- 车体在作业过程中会晃动
- IMU 是高频数据，点云和 GUI/ROS 是低频消费者

因此需要解决：

```text
如何在高低频异步、存在安装误差和零偏漂移的条件下，稳定恢复 roll/pitch/yaw？
```

#### 问题二：从三维作业点到机械臂关节角的可达求解

给定点云中一个目标点 `(x, y, z)`，系统必须先确定：

- 回转朝向是否合理
- 半径是否可达
- 高度是否可达
- 铲斗姿态应如何约束

因此需要解决：

```text
如何把三维目标点转成满足机械结构和控制语义约束的关节角解？
```

#### 问题三：从单点动作到多点连续作业

单点可达不代表多点连续任务可执行。多点任务还会额外涉及：

- 初始化与归位拆分
- 点位排序
- 单点失败跳过
- 过渡姿态设计

因此需要解决：

```text
如何把一批候选作业点自动组织成连续、可执行、可验证的多点任务脚本？
```

#### 问题四：实机接口、仿真接口与几何模型接口的统一

当前项目跨越了多个版本：

- `v4` 偏实机闭环控制
- `v10` 偏几何建模
- `v11/v12` 偏多模态感知
- `v14` 偏 URDF 仿真与任务规划

因此需要解决：

```text
如何用统一接口把实机控制语义、几何模型和 ROS/URDF 仿真连接起来？
```

### 0.4 课题技术路线

当前仓库已经走出来的一条比较清晰的技术路线如下：

```text
IMU / 倾角传感器
-> 姿态补偿与回转角估计
-> 点云 / 工作区域几何表达
-> 目标点提取或公式生成
-> 单点逆解与轨迹搜索
-> 多点任务拼接
-> URDF / RViz 验证
-> 回接实机控制语义
```

如果进一步写成分层结构，可以分成五层：

1. **感知层**
   - 倾角传感器、雷达 IMU、点云、相机
2. **状态估计层**
   - `roll/pitch/yaw` 解算、零偏校准、重力补偿
3. **几何建模层**
   - FK / IK、工作空间、作业区域、圆环/半球壳模型
4. **规划层**
   - 单点挖掘轨迹、多点任务生成、候选点筛选
5. **接口验证层**
   - `v14_urdf`、`/joint_states`、RViz、JSON 剧本回放

### 0.5 当前仓库中的课题分工映射

当前目录结构其实已经自然对应了几个子课题：

| 子课题 | 主要目录 | 当前定位 |
| --- | --- | --- |
| 姿态估计与 IMU 预积分 | `v5_sensor_read_lidar`, `v11_multimodal_dataset_collection`, `v12_multimodal_hybrid_architecture` | 感知与状态估计 |
| 机械臂几何建模与逆解 | `v10_cailbration_arm` | 数学模型核心 |
| 单点轨迹与流程语义 | `v14_urdf/point_to_dig_dump_trajectory.py` | 单点作业规划 |
| 多点轨迹与区域模式 | `v14_urdf/multi_dig`, `v14_urdf/mode1` | 多点规划与区域任务 |
| 仿真接口与语义映射 | `v14_urdf` | ROS 2 / URDF 验证层 |
| 实机闭环控制接口 | `v4_control_closed` | 实机执行语义源头 |

### 0.6 课题的阶段性成果

从当前代码状态看，这个课题已经完成的阶段性成果包括：

1. 已形成稳定的实机关节控制语义：
   - `boom_swing`
   - `arm_boom`
   - `bucket_arm`
   - `swing_yaw`
2. 已形成倾角与雷达 IMU 融合的姿态估计链路。
3. 已形成可复用的二维 FK / IK 模型。
4. 已形成从单个挖掘点、单个卸料点自动生成动作轨迹的能力。
5. 已形成从候选点集合自动生成多点任务脚本的能力。
6. 已形成 `v14_urdf` 仿真接口，可在 RViz 中完成动作语义验证。
7. 已形成 `mode1` 的工作区域点云与约束 JSON，可作为后续 `mode2` 的几何基准。

### 0.7 对 mode2 的直接意义

如果把后续工作落到 `mode2`，那么这份课题文档对你最直接的价值有三点：

1. 你已经有稳定的姿态补偿和回转角估计基础，不需要从 IMU 开始重写。
2. 你已经有成熟的单点 IK 和动作语义，不需要重新设计单次挖掘流程。
3. 你已经有工作区域几何和圆环/半球壳生成思路，`mode2` 可以直接把重点放在：
   - 点的生成
   - 点的筛选
   - 点的排序
   - 多点连续执行

换句话说，`mode2` 更像是这个总课题在“几何生成 + 候选点组织 + 多点任务”方向上的
下一阶段，而不是一个完全独立的新系统。

## 1. 倾角传感器预积分与 IMU 漂移修正

这一部分对应当前仓库里的两条主线：

- `v5_sensor_read_lidar/imu_direct_swing_estimator.py`
- `v11_multimodal_dataset_collection/templates/imu_preintegration.py`

项目里的重点不是完整惯导导航解算，而是“在挖掘机车体晃动、雷达倾斜安装、原地回转”
条件下，稳定解算：

- `roll`
- `pitch`
- `swing_yaw`

### 1.1 观测模型

记：

- 陀螺仪测量值：`omega_m`
- 加速度计测量值：`a_m`
- 陀螺仪零偏：`b_g`
- 加速度计零偏：`b_a`
- 噪声：`n_g`, `n_a`
- 真实角速度：`omega`
- 重力向量：`g`

则标准传感器模型可以写成：

```text
omega_m = omega + b_g + n_g
a_m = R^T (a + g) + b_a + n_a
```

在本项目中，真正长期可观测、且工程上最关键的是：

- 由重力恢复 `roll/pitch`
- 由回转角速度积分得到 `yaw`

### 1.2 倾角的互补滤波

当前 `TiltCompensator` 采用的是互补滤波，而不是完整 EKF。

先用加速度计恢复几何姿态：

```text
roll_acc  = atan2(a_y, a_z)
pitch_acc = atan2(-a_x, sqrt(a_y^2 + a_z^2))
```

再用陀螺仪积分提供高频变化：

```text
roll_gyro(k)  = roll(k-1)  + omega_x * dt
pitch_gyro(k) = pitch(k-1) + omega_y * dt
```

最后做互补滤波融合：

```text
roll(k)  = alpha * roll_gyro(k)  + (1 - alpha) * roll_acc(k)
pitch(k) = alpha * pitch_gyro(k) + (1 - alpha) * pitch_acc(k)
```

其中 `alpha` 在现有实现里默认取 `0.98`，含义是：

- 高频变化主要相信陀螺仪
- 低频绝对方向主要相信加速度计

### 1.3 开机零偏校准

项目里最关键的一步是“开机静止联合校准”。系统在启动后的静止窗口内累计若干帧
加速度测量，估计安装误差零点：

```text
roll0  = mean(roll_acc_i)
pitch0 = mean(pitch_acc_i)
```

后续真正用于 ROS / 点云纠正的姿态不是绝对姿态，而是相对初始水平面的姿态：

```text
roll_rel  = roll  - roll0
pitch_rel = pitch - pitch0
```

这样可以消除：

- 传感器安装倾斜
- 车体初始停放不水平
- 雷达装配固定误差

### 1.4 回转 yaw 的预积分

单纯对 `gyro_z` 积分在本项目里是不够的，因为雷达通常不是完美水平安装。现有系统
的核心思路是：

1. 开机静止时，用加速度平均值得到雷达坐标系下的“向上向量” `u`。
2. 每时刻把三维角速度 `omega = [omega_x, omega_y, omega_z]^T` 投影到 `u` 上。

真实用于积分的回转角速度写成：

```text
omega_yaw = omega · u
```

然后再用积分恢复回转角：

```text
yaw(k) = yaw(k-1) + omega_yaw * dt
```

当前实现里还叠加了：

- 零偏校准
- 小角速度死区
- 梯形积分

如果写成更稳的离散形式：

```text
yaw(k) = yaw(k-1) + 0.5 * (omega_yaw(k-1) + omega_yaw(k)) * dt
```

### 1.5 本项目里“尺度漂移”的准确理解

如果按严格 SLAM 术语，“尺度漂移”通常指地图尺度不稳定。但在本项目现有代码语境里，
更准确的问题其实是：

- 陀螺仪积分漂移
- 雷达倾斜安装导致的 yaw 投影误差
- 零偏累计误差

当前仓库里已经采用的修正方法是：

1. **静止零偏校准**
   - 开机保持静止，估计 `b_g` 与初始重力方向。
2. **重力方向投影**
   - 不直接积分 `gyro_z`，而是积分 `omega · u`。
3. **互补滤波**
   - 用加速度长期纠正 `roll/pitch`。
4. **外部 yaw 输入**
   - 在 `TiltCompensator` 中，`yaw` 设计为外部输入，因为单靠陀螺仪积分会漂移。
5. **高频内部积分、低频对外发布**
   - 内部 100Hz~200Hz 更新，外部 ROS / GUI 低频读取，避免消费者阻塞反过来放大漂移。

### 1.6 这一部分对后续规划的意义

对后续轨迹规划与点云处理来说，IMU 预积分不是独立目标，它主要服务于三件事：

- 恢复稳定的 `swing_yaw`
- 纠正点云相对于地平面的 `roll/pitch`
- 为 `base_link` 下的工作区域 / 点云 ROI 提供稳定姿态基准

## 2. 铲斗轨迹规划、逆解与挖掘轨迹模型

这一部分的核心实现位于：

- `v10_cailbration_arm/inverse_kinematics.py`
- `v14_urdf/point_to_dig_dump_trajectory.py`

项目当前采用的是：

- 三维点先转成 `yaw + (r, z)` 问题
- 再用二维平面逆解求 `boom_swing / arm_boom / bucket_arm`

### 2.1 三维点到回转角与工作平面的转换

给定目标点：

```text
P = (x, y, z)
```

先求回转角：

```text
swing_yaw = atan2(y, x)
```

再把问题投影到工作平面，定义：

```text
r = sqrt(x^2 + y^2)
```

于是三维问题被转成二维目标：

```text
(r, z)
```

这一步的物理意义是：

- `yaw` 决定上车回转朝向
- `(r, z)` 决定大臂、小臂、铲斗在竖直剖面内如何运动

### 2.2 二维逆向运动学模型

现有模型把机械臂视作：

- 大臂等效连杆：`L_boom`
- 小臂连杆：`L_arm`
- 铲斗连杆：`L_bucket`

并且底座相对于回转中心存在偏移：

```text
offset_x, offset_z
```

如果给定铲斗绝对姿态角 `theta3`，铲尖目标点是 `(target_x, target_z)`，那么先扣除铲斗
长度，得到腕点：

```text
x_wrist = (target_x - offset_x) - L_bucket * cos(theta3)
z_wrist = (target_z - offset_z) - L_bucket * sin(theta3)
```

定义腕点距离：

```text
d = sqrt(x_wrist^2 + z_wrist^2)
```

可达条件为：

```text
|L_boom - L_arm| <= d <= L_boom + L_arm
```

### 2.3 两连杆逆解推导

记：

```text
alpha = atan2(z_wrist, x_wrist)
```

根据余弦定理：

```text
cos(gamma) = (L_boom^2 + d^2 - L_arm^2) / (2 * L_boom * d)
gamma = acos(cos(gamma))
```

采用当前项目里的“肘部朝上”解：

```text
theta1 = alpha + gamma
```

大臂末端坐标：

```text
x_elbow = L_boom * cos(theta1)
z_elbow = L_boom * sin(theta1)
```

于是小臂绝对角为：

```text
theta2 = atan2(z_wrist - z_elbow, x_wrist - x_elbow)
```

到这里，几何空间里三个绝对角已经确定：

- `theta1`：大臂绝对角
- `theta2`：小臂绝对角
- `theta3`：铲斗绝对角

### 2.4 从几何角回到 V4 控制语义

现有系统并不是直接下发绝对几何角，而是回到 `v4` 的控制语义。根据已标定的映射关系：

```text
abs_boom_L2_deg = theta1_deg - beta_deg
sensor_boom_deg = 40.9 - abs_boom_L2_deg
sensor_arm_deg = 19.6 - theta2_deg
sensor_bucket_deg = theta3_deg + 56.2
```

最终 V4 剧本里的相对角写成：

```text
boom_swing = sensor_boom_deg
arm_boom   = sensor_arm_deg - sensor_boom_deg
bucket_arm = sensor_bucket_deg - sensor_arm_deg
```

这就是为什么 `v14_urdf` 虽然在做 URDF 仿真，但其底层几何求解仍直接复用 `v10` 的
逆解模型。

### 2.5 从挖掘点生成挖掘轨迹

单个挖掘点并不会直接生成一个终点，而是生成一个阶段化轨迹。当前 `DigDumpPlanner`
 采用的流程是：

1. 对准挖掘点：`swing_yaw -> dig_yaw`
2. 挖掘预备：半开斗、大臂下探、小臂下探
3. 收斗取料：`bucket_arm -> 0`
4. 抬臂离坑：抬大臂、收小臂、稳料
5. 回转到卸料点
6. 卸料预备
7. 打开铲斗卸料：`bucket_arm -> -90`

如果用集合符号写，一次单点循环可以抽象为：

```text
Cycle(P_dig, P_dump)
= Align(P_dig)
+ Entry(P_dig)
+ Scoop(P_dig)
+ Lift()
+ SwingTo(P_dump)
+ Dump(P_dump)
```

### 2.6 轨迹生成中的约束

当前实现中并不是“只要 IK 有解就采用”，还同时施加了：

- 关节角限位
- 铲斗绝对姿态候选集合
- 卸料安全高度候选
- 偏好分数函数

因此它本质上是：

```text
argmin score(q)
subject to
  IK(q, target) feasible
  joint_limits satisfied
  preferred_bucket_angle satisfied
  preferred_dump_height satisfied
```

这也是当前轨迹生成器更偏“工程可执行搜索”，而不是严格解析最优控制的原因。

## 3. 多点轨迹的实现方法

这一部分的主线在：

- `v4_control_closed/generate_imu_auto_dig.py`
- `v14_urdf/multi_dig/`
- `v14_urdf/mode1/`
- `v14_urdf/mode1/mode1_task_planner.py`

### 3.1 从单点循环到多点循环

多点轨迹的关键不是把所有点直接串起来，而是把任务分成三段：

```text
Task = Init + Sum(Cycle_i + Transit_i) + Home
```

也就是：

1. `init_segment`
2. `cycle_segment_1 ... cycle_segment_n`
3. `home_segment`

这样设计的好处是：

- 初始化只做一次
- 每个挖掘点只做一次局部循环
- 所有点结束后才执行最终归位

### 3.2 候选点的来源

当前项目里，多点候选点主要有两种来源：

1. **预设区域模式**
   - 在矩形框或工作区域网格内采样
2. **点云模式**
   - 从真实点云、ROI、匹配结果或作业区域点中提取候选点

形式上可以把候选点集合记为：

```text
S = {p_1, p_2, ..., p_n}
```

每个点都带：

- `x`
- `y`
- `z`
- `radius`
- `yaw_deg`

### 3.3 候选点排序

当前 `mode1` 中已经有牛耕式排序（`boustrophedon`）逻辑。其目的不是数学最优，而是：

- 减少回转来回跳跃
- 尽量让邻近点连续执行
- 让挖掘路径更像人工连续扫掘

抽象写法可以表示为：

```text
ordered(S) = sort_by_row_then_snake(S)
```

如果后续切到 `mode2`，你依然可以复用这个排序思路。

### 3.4 多点任务拼接

对每一个候选点 `p_i`：

```text
cycle_i = plan_single_cycle(p_i, dump_point)
```

然后做总拼接：

```text
task_script
= init_pose
+ cycle_1
+ transit_1
+ cycle_2
+ transit_2
+ ...
+ cycle_n
+ home_pose
```

如果某个点逆解失败或超限，则跳过：

```text
if IK(p_i) == None:
    skip(p_i)
```

因此当前多点规划器不是“强行规划所有点”，而是：

- 读取一批候选点
- 逐点尝试生成单点循环
- 成功的点进入总任务
- 失败的点写入 `skipped_candidates`

### 3.5 多点规划的工程原则

当前仓库里已经验证出的原则是：

- 初始化与归位不参与挖掘循环
- 单点规划先正确，再做多点拼接
- 点云候选点宁可少一点，也不要把机械臂干扰点或漂浮点带入循环
- 规划阶段允许跳过个别失败点，不能因为单点失败导致全局任务不可执行

## 4. `v14_urdf` 的 ROS 接口与 `v4/v7/v10/v11/v12` 的区别

这里把 `v14_urdf` 作为当前“仿真与规划接口”基准版本，对比其与前几个版本的区别。

### 4.1 总体对比

| 版本 | 主要职责 | 接口形态 | 是否以 ROS 2 为主 | 核心输入输出 |
| --- | --- | --- | --- | --- |
| `v4_control_closed` | 实机闭环控制与 JSON 剧本执行 | Python 类 + 串口/UDP + JSON | 否 | 目标角、传感器角、CAN 输出 |
| `v7_lerobot_dataset` | 早期 ROS 订阅式数据集采集 | ROS 2 Topic 订阅 | 是 | 图像 / 角度 Topic -> 数据集 |
| `v10_cailbration_arm` | FK / IK / 工作空间几何分析 | 纯 Python API | 否 | `(X,Z)` <-> 关节角 |
| `v11_multimodal_dataset_collection` | 多模态 ROS 2 发布与采集 | 标准 ROS 2 Topic 发布 | 是 | 图像、点云、JointState |
| `v12_multimodal_hybrid_architecture` | C++/Python 混合高频系统 | ROS 2 节点 + Topic + Launch | 是 | 高速感知/控制 Topic |
| `v14_urdf` | URDF 仿真、`/joint_states` 桥接、点到轨迹规划 | ROS 2 + URDF + JSON | 是 | JointState、规划 JSON、RViz |

### 4.2 `v14_urdf` 的接口特点

`v14_urdf` 与前几个版本最大的不同，是它把“控制语义”和“真实硬件接口”解耦了。

当前 `v14_urdf` 的核心接口有三类：

1. **URDF 显示接口**
   - `ros2 launch describe_60FED display.launch.py`
2. **JointState 驱动接口**
   - 通过 `/joint_states` 驱动 RViz 中的机械臂模型
3. **规划与剧本接口**
   - `replay_json_script.py`
   - `point_to_dig_dump_trajectory.py`
   - `mode1_task_planner.py`

它的作用不是直接控制液压，而是：

- 复用 `v4` 的关节语义
- 复用 `v10` 的 IK
- 在 ROS 2 / RViz 中做动作验证和任务规划

### 4.3 与 `v4_control_closed` 的区别

`v4` 是实机控制版本，`v14` 是仿真桥接版本。

**`v4` 的接口特点**

- 直接对接 `/dev/ttyUSB_Sensor*`
- 直接对接 CAN 控制器
- 直接对接雷达 IMU UDP
- 输出是液压电压、继电器动作和真实机械臂运动

**`v14` 的接口特点**

- 不直连 CAN 和倾角传感器
- 不下发真实液压命令
- 只通过 `/joint_states` 驱动 URDF
- 主要用于动作语义验证、轨迹规划验证、工作区域验证

可以理解为：

```text
v4 = 实机执行层
v14 = 仿真验证层
```

### 4.4 与 `v7_lerobot_dataset` 的区别

`v7` 是早期数据采集验证版，重点是“订阅已有 ROS Topic 生成 LeRobot 数据集”。

它的接口核心是订阅：

- 图像 Topic
- 倾角 / 回转 Topic

而 `v14` 不以数据集采集为目标，重点是：

- 生成轨迹
- 回放轨迹
- 在 RViz 中验证几何与动作

因此两者差异是：

```text
v7 = 数据记录接口
v14 = 仿真规划接口
```

### 4.5 与 `v10_cailbration_arm` 的区别

`v10` 是几何与运动学版本，不提供 ROS 接口主线。

它主要提供：

- 正向运动学
- 逆向运动学
- 工作空间分析
- 轨迹动画

而 `v14` 的很多规划能力都建立在 `v10` 的几何模型之上。

可以理解为：

```text
v10 = 数学模型层
v14 = ROS 仿真与任务层
```

### 4.6 与 `v11_multimodal_dataset_collection` 的区别

`v11` 是 ROS 2 多模态发布版本，重点是统一发布：

- `/camera*/image_raw`
- `/lidar/points`
- `/excavator/joint_states`

它的定位是“采集与可视化”。

而 `v14` 的定位是“规划与回放”。

两者都用 ROS 2，但接口重心不同：

- `v11` 偏感知数据接口
- `v14` 偏机械臂轨迹接口

### 4.7 与 `v12_multimodal_hybrid_architecture` 的区别

`v12` 是高频混合架构版本，接口特点是：

- C++ 节点负责高频 I/O 与点云处理
- Python GUI 只做上层订阅
- 存在更完整的目标命令 Topic 和状态 Topic

相比之下，`v14` 更轻量，也更专注：

- 不追求高频传感器吞吐
- 不追求混合架构
- 重点是把已有控制语义映射到 URDF 和 RViz

因此：

```text
v12 = 高性能系统接口层
v14 = 轻量仿真规划接口层
```

## 5. 推荐阅读路径

如果你后续继续做理论设计，建议按下面顺序阅读：

1. `v11_multimodal_dataset_collection/docs/IMU_PointCloud_Architecture.md`
2. `v11_multimodal_dataset_collection/templates/imu_preintegration.py`
3. `v10_cailbration_arm/readme_arm.md`
4. `v10_cailbration_arm/inverse_kinematics.py`
5. `v14_urdf/point_to_dig_dump_trajectory.py`
6. `v14_urdf/mode1/README.md`
7. `v14_urdf/mode1/mode1_task_planner.py`

## 6. 下一步建议

如果你要继续往 `mode2` 推进，这份文档最直接的用途有两个：

1. 把“单点逆解”替换成“公式生成候选点 + 多点拼接”。
2. 把 `v14_urdf` 作为仿真验证层，继续复用 `v10` 的几何模型和 `v4` 的控制语义。
