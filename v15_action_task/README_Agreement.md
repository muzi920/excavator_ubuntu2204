# v15_action_task — Interface Agreement (接口协议说明)

<!-- prettier-ignore -->
> [!IMPORTANT]
> 本文档是 v15 对外发布的**正式接口协议（Agreement）**。所有在表 1 / 表 2
> 中标注为 **「STABLE」** 的接口，保证语义、参数顺序、返回字段、ROS 话题
> 名 / msg 类型 / 单位 在后续 v15 的小版本中**不会破坏兼容性**。
> 标注为 **「PROVISIONAL」** 的接口在未来可以扩展（只加字段，不减 / 不改）。
> 所有在表中未列出的私有实现细节（以下划线开头的变量、内部类等），业务层
> **禁止直接依赖**。

本文档分四部分说明：

1. 接口总览：所有对外可用的符号，分类为「控制输出」 vs 「传感器输入」 vs
   「通用工具」；
2. 控制输出协议（下到 URDF / 真机）：关节级 / 笛卡尔级 / 语义级 / 剧本级
   四层 API；
3. 传感器输入协议（从 ROS 2 话题采集进来）：4 倾角 + 5 雷达 + ≥5 相机 三
   类消息格式、v15 内部统一的 Reading 数据结构、温漂补偿流程；
4. 新程序 3 种接入模板：分别对应「最小可用」「完整工具链」「自定义机型配
   置」，直接复制改参数即可。

---

## 1. 接口符号总览（3 大类）

所有 STABLE 符号都可以通过 `from v15_action_task import XXX` 直接导入，
不需要再写 `import v15_action_task.xxx.yyy` 形式。

| 类别 | 接口方向 | 稳定度 | 符号 / 函数签名 | 章节 |
|------|---------|--------|----------------|------|
| **A. 控制输出（从程序 → 挖掘机）** | OUTPUT（发布） | STABLE | `from_config(*, adapter_backend, use_config_limits, init_sensors, sensor_ros_mode) -> dict` | §2.1 |
| 同上 | OUTPUT | STABLE | `URDFController`（`set_joint / set_pose / get_pose / is_at_pose`） | §2.2 |
| 同上 | OUTPUT | STABLE | `CartesianMover`（`move / move_with_bucket`） + `MoveResult`（含 `__bool__()`） | §2.3 |
| 同上 | OUTPUT | STABLE | 8 个语义函数：`swing_move / boom_move / arm_move / bucket_move / swing_move_to / boom_lift / arm_extend / bucket_tilt_to`（统一返回 `MoveResult`） | §2.4 |
| 同上 | OUTPUT | STABLE | `build_single_dig_dump_task / build_single_dig_dump_script / build_multi_dig_task / build_multi_dig_cycles` | §2.5 |
| **B. 传感器输入（ROS 2 话题 → 程序）** | INPUT（订阅） | STABLE | `SensorManager.from_config(cfg, ros_environment="auto"/"force_ros"/"force_mock")` + `open/close / __enter__/__exit__` | §3.1 |
| 同上 | INPUT | STABLE | `SensorManager.get_tilt(id) -> TiltReading / .all_tilt() / list_tilt_ids()` | §3.2 |
| 同上 | INPUT | STABLE | `SensorManager.get_lidar(id) -> LidarReading / .all_lidar() / list_lidar_ids()` | §3.3 |
| 同上 | INPUT | STABLE | `SensorManager.get_camera(id) -> CameraReading / .all_camera() / list_camera_ids()` | §3.4 |
| 同上 | INPUT（内部纯算术，稳定） | STABLE | `TiltCompensator.update(raw_angle_deg, *, gyro_rad_s_or_none, dt_s) -> (compensated_deg, done_bool, remaining_frames)` | §3.2.3 |
| **C. 通用工具 / 配置（读写）** | 读写双向 | STABLE | `load_config(path_yaml_or_json) -> V15Config` + `load_default_config() -> V15Config` + `V15Config.from_dict(d) -> V15Config` | §4 |
| 同上 | 读写双向 | STABLE | `ForwardKinematics / InverseKinematics / LinkParams / DEFAULT_PARAMS / FKSolution / IKSolution` | §4 |
| 同上 | 仅读 | PROVISIONAL | `SingleSensorConfig / SensorsConfig / ExtrinsicsEntry / ExtrinsicsConfig / TiltCompensationConfig`（允许追加字段） | §3 |
| 同上 | 仅读 | STABLE | `RosV14Adapter.from_config(cfg)` + `MockAdapter()` | §2.1 |

---

## 2. 控制输出协议（OUTPUT：程序 → 挖掘机）

控制输出层分 4 层：你可以从任一层进入，下层不会破坏上层的返回契约。
最推荐的入口是 `from_config()`（第 1 层），它帮你把所有工具一次性组装完毕。

### 2.1 顶层工厂入口：`from_config()`（STABLE）

> **一句话**：一行代码拿到 `controller / fk / ik / mover / sensor_manager / config`
> 7 件套。底层 Adapter 可以在 Mock（本地调试）和 ROS（RViz / 真机）之间
> **零改动切换**。

函数签名：

```python
from v15_action_task import from_config

ctx = from_config(
    cfg=None,                             # None=默认 60FED / str 路径 / V15Config / dict
    *,
    adapter_backend: str = "mock",        # "mock" | "ros" | "v14" (后两者等价)
    start_adapter: bool = False,          # True=立刻 adapter.open()，ros 后端常用
    use_config_limits: bool = True,       # True=自动用 YAML 关节限位 3 层裁剪(AC-1)
    init_sensors: bool = False,           # True=同步构造 SensorManager 并 open
    sensor_ros_mode: Literal[             # "auto"=检测 rclpy / "force_ros" / "force_mock"
        "auto","force_ros","force_mock"
    ] = "auto",
)
```

返回 `dict` 的 7 个 key（全部 STABLE，顺序不保证）：

| key | 类型 | 说明 |
|-----|------|------|
| `"config"` | `V15Config` | 完整配置对象；可继续分块 `build_controller / build_kinematics / build_mover` |
| `"controller"` | `URDFController` | 已装配好 Adapter + 限位裁剪；未 enter 上下文，需 `with ctx["controller"] as ctl:` |
| `"adapter"` | `MockAdapter \| RosV14Adapter` | 底层通信；`start_adapter=True` 时已调 `open()` |
| `"fk"` / `"ik"` | `ForwardKinematics / InverseKinematics` | 连杆几何来自配置；`cfg = ctx["config"]` 时，FK 结果真实对应配置中的 L1/L2/L_bucket |
| `"mover"` | `CartesianMover` | 笛卡尔运动执行器；容差 / 超时 / 铲斗搜索样本数均来自配置 `motion_defaults` |
| `"sensor_manager"` | `SensorManager \| None` | `init_sensors=True` 时为实例（已调 `open()`），`False` 时为 `None` — 保证旧代码零 break |

<details>
<summary>两个 Adapter 的 ROS 2 话题协议（与 v14 URDF 1:1 对齐）</summary>

`RosV14Adapter` 底层严格使用以下话题协议（可从 `config.ros_protocol`
覆盖）：

- **发布 OUTPUT**：`/joint_states`，msg 类型 `sensor_msgs/msg/JointState`，
  QoS depth = `config.ros.qos_depth`（默认 10），frame_id =
  `config.ros.frame_id`（默认 `"base_link"`）。
- `msg.name` 顺序必须是：
  `["swing_joint","boom_joint","arm_joint","bucket_joint"]`（URDF 原关节）。
- `msg.position` 单位为**弧度**，自动由 v15 内部 `deg_to_rad()` 换算；
  上层 API 统一入**度**。
- **订阅（作为反馈）**：同话题 `/joint_states`（双向半双工）。
- `RosV14Adapter.from_config(cfg)` 自动把 `cfg.mapping` 应用到全局
  `SEMANTIC_TO_URDF` 映射，配置里写新的映射名即可适配其他 URDF。

</details>

### 2.2 关节级控制：`URDFController`（STABLE，所有上层都调用它）

> 关节级 API 是 v15 对外的最低控制层：你传入**语义名** + **度**，
> 它帮你做限位裁剪、URDF 名映射、度 → 弧度换算、`/joint_states` 发布。

**稳定度：STABLE。所有公开方法签名：**

```python
from v15_action_task import URDFController

# 构造（from_config 会自动替你调）
ctl = URDFController(adapter, *, joint_limits=None, clamp=True)
ctl.open();  ctl.close();
with ctl: ...   # 上下文管理器，自动 open/close

# 发布（OUTPUT）
ctl.set_joint(semantic_joint: str, value_deg: float,
              *, frame_id="base_link", clamp: bool | None = None) -> bool
ctl.set_pose(pose_deg: Dict[str, float],
             *, frame_id="base_link", clamp: bool | None = None) -> bool

# 读反馈（INPUT，由 Adapter 订阅缓存）
ctl.get_pose()         -> Dict[str,float] | None  # 非阻塞，无反馈返回 None
ctl.get_pose_blocking(timeout_s=5.0) -> Dict[str,float] | None  # 等首帧反馈
ctl.get_pose_or_default()           -> Dict[str,float]  # 无反馈返回全 0（安全默认）
ctl.is_at_pose(target_deg: Dict[str,float], tolerance_deg=1.0, joints: list[str] | None = None) -> bool
```

**4 个语义关节名（STABLE，与 URDF 映射在 `config.joint_mapping`）：**

| 语义名（joint API 入参） | 默认 URDF 关节 | 单位 | 正方向约定（STABLE，禁止修改） |
|--------------------------|----------------|------|-------------------------------|
| `swing_yaw`              | `swing_joint`  | deg  | 俯视下 **顺时针 +**（车舱右转） |
| `boom_swing`             | `boom_joint`   | deg  | 大臂 **向下落 +** / 向上抬 **-** |
| `arm_boom`               | `arm_joint`    | deg  | 小臂 **收回 +**（铲尖近机身）/ 伸出 **-** |
| `bucket_arm`             | `bucket_joint` | deg  | **收斗/卷铲 +** / 外翻卸料 **-** |

> 🔺 重要：方向约定 4 条是 **STABLE 契约**。任何业务代码都不要自己再写
> `±angle` 去"修正方向"，否则语义层（§2.4）会做两次叠加导致动作相反。

### 2.3 末端笛卡尔运动：`CartesianMover` + `MoveResult`（STABLE）

> 直接按铲尖 `(x, y, z)` 米做控制，内部自动用 `InverseKinematics`
> 解算 4 关节角、自动搜索铲斗候选角、发布、等待到位。`MoveResult` 是
> 所有运动指令（语义 / 笛卡尔 / 剧本）统一使用的返回结构体。

```python
from v15_action_task import CartesianMover, MoveResult, move_to_cartesian

# 推荐从 from_config() 直接拿 ctx["mover"] 而不是手动造
mover: CartesianMover = ctx["mover"]

r1: MoveResult = mover.move(x_m, y_m, z_m, *, blocking=True, tolerance_deg=1.0, timeout_s=3.0)
r2: MoveResult = mover.move_with_bucket(x_m, y_m, z_m, bucket_angle_deg, *, blocking=True)

# MoveResult 所有字段（STABLE，允许未来追加 — 但已列出字段绝不删减）
r1.success                   # bool（__bool__() 就是这个值 → 可直接写 `if r1:`）
r1.reached_pose_deg          # Dict[str,float] | None — 最终关节角（度）
r1.final_tip_xyz             # Tuple[float,float,float] | None — 到位后 FK 铲尖（米）
r1.ik_solution               # IKSolution | None
r1.target_xyz                # Tuple[float,float,float] — 本次目标（米）
r1.target_bucket_angle_deg   # float | None — 本次指定铲斗角（度）；None = 自动搜索
r1.waited_s                  # float — 实际等待秒数（非阻塞 =0.0）
r1.reason                    # str — 失败原因简述；成功时 = "" 或 "ok"
```

坐标约定（STABLE，与 FK/IK 数学完全一致）：

- **X 轴**：前方（从 `base_link` 指向挖掘机前方履带方向）
- **Y 轴**：左方（驾驶员视角左手侧）
- **Z 轴**：上方（垂直地面向上）
- **单位**：3 轴一律 **米**；`bucket_angle_deg` 铲斗绝对几何角，**向上为正 /
  水平为 0**。挖掘典型区间：`-60° ~ -20°`（下挖）；卸料典型区间：`-10° ~ +20°`。

### 2.4 语义作业指令库：8 个布尔方向 + 绝对角 / 笛卡尔便利函数（STABLE）

> 这是用户在阶段 1 第 2 条规格里明确点名的接口风格（例：`arm_move(bool
> up_or_down, float angle)`）。8 个函数全部复用 §2.3 的 `MoveResult`，
> 并且所有方向语义在 `examples/ex02_semantic_direction_AC7_self_assert.py`
> 中做了 8/8 自断言验收（AC-7）。

函数一览（均从 `from v15_action_task import ...` 可直接导入）：

```python
from v15_action_task import (
    swing_move, boom_move, arm_move, bucket_move,     # (A) 4 基础：布尔方向增量
    swing_move_to, boom_lift, arm_extend, bucket_tilt_to,  # (B) 4 扩展：绝对 / 笛卡尔
)

# ── (A) 4 基础增量 ── 方向约定表已在 AC-7 中固化
# swing:  right_or_left=True → swing_yaw  += angle（顺时针 +）
# boom:   up_or_down=True   → boom_swing -= angle（抬起 = 负）
# arm:    up_or_down=True   → arm_boom   += angle（收回 = 正）
# bucket: up_or_down=True   → bucket_arm += angle（收斗 = 正）
r = swing_move (ctl, right_or_left: bool, angle_deg: float,
                *, blocking=True, tolerance_deg=1.0, timeout_s=3.0, cfg=None) -> MoveResult
r = boom_move  (ctl, up_or_down: bool,     angle_deg: float,
                *, blocking=True, tolerance_deg=1.0, timeout_s=3.0, cfg=None) -> MoveResult
r = arm_move   (ctl, up_or_down: bool,     angle_deg: float,
                *, blocking=True, tolerance_deg=1.0, timeout_s=3.0, cfg=None) -> MoveResult
r = bucket_move(ctl, up_or_down: bool,     angle_deg: float,
                *, blocking=True, tolerance_deg=1.0, timeout_s=3.0, cfg=None) -> MoveResult

# ── (B) 4 扩展便利 ──
r = swing_move_to(ctl, abs_yaw_deg: float,  # 回转绝对值；自动判方向
                  *, blocking=True, tolerance_deg=1.0, timeout_s=3.0, cfg=None) -> MoveResult
r = boom_lift  (ctx["mover"], delta_z_m: float,   # 笛卡尔：抬 / 降 z
                *, keep_xy=True, tolerance_deg=1.0, timeout_s=3.0) -> MoveResult
r = arm_extend (ctx["mover"], delta_x_m: float,   # 笛卡尔：伸 / 收 x
                *, keep_yz=True, tolerance_deg=1.0, timeout_s=3.0) -> MoveResult
r = bucket_tilt_to(ctl, abs_bucket_tip_deg: float, # 铲尖绝对倾角
                   *, blocking=True, tolerance_deg=1.0, timeout_s=3.0, cfg=None) -> MoveResult
```

**关于 `cfg=None` 参数**：显式传 `cfg = ctx["config"]` 可强制走
`FR-1.2 第 3 层`（Config 层 clamp_pose）；不传时按 **传 cfg →
ctl.adapter.config → load_default_config()** 三级兜底自动解析，最终都会
把超限角度在发布前裁剪到限位内。

### 2.5 高层剧本构建：`action_library` 的 Task/Steps 生成器（PROVISIONAL）

> 这一层是把挖掘—回转—卸料的 20+ 步组合动作写成一键生成器，属于业务
> 模板层，参数仍可增删，因此标 PROVISIONAL。返回值的两个核心键
> `"script"` 与 `target_deg` 的格式稳定。

```python
from v15_action_task.action_library import (
    StepBuilder, INIT_POSE, HOME_POSE, CYCLE_TRANSIT_POSE,
    build_single_dig_dump_script,   # 返回 script: List[dict]，每个 dict 带 {"target_deg"/"duration_s"/"label"}
    build_single_dig_dump_task,     # 带 meta 包装（包含初始/循环姿态）
    build_multi_dig_task,           # 多挖掘点循环
    build_multi_dig_cycles,         # 指定循环次数
)

# 快速跑剧本：
for step in build_single_dig_dump_script(dig_x=1.0, dig_z=-0.25, dump_yaw_deg=60.0):
    ok = ctl.set_pose(step["target_deg"])
    time.sleep(step["duration_s"])
```

---

## 3. 传感器输入协议（INPUT：ROS 2 话题 → v15 内部 Reading 结构体）

传感器层的设计哲学（STABLE 设计原则）：

1. **v15 代码本身绝不 import shandong.v0~v14 的任何模块**（AC-6，
   两次 grep 0 行）。你现有的 v12 桥接节点 / v3 WIT 倾角驱动节点 /
   PaceCat 雷达驱动节点，继续在外部独立运行、发布原始话题即可，v15
   的 `SensorManager` 只负责「订阅 + 拆包 + 补偿 + 缓存」。
2. **缺少 rclpy 也不会崩**。`SensorManager` 在 import 时对 rclpy 做
   `try/except ImportError` 检查；`sensor_ros_mode="auto"` 时会自动
   回退到 Mock 模式，`get_*()` 合法但 `is_valid=False`。
3. **配置驱动**。所有话题名、msg 类型、外参 6 参数、温漂参数、父子
   链相对角算法，全部可从 `default_config.yaml` / 自定义机型 JSON
   修改，无需改 Python。

### 3.1 `SensorManager` 生命周期与创建（STABLE）

创建方式（推荐走 `from_config(init_sensors=True, sensor_ros_mode=...)`，
也可单独构建）：

```python
from v15_action_task import SensorManager, load_default_config

cfg = load_default_config()
smgr = SensorManager.from_config(cfg, ros_environment="auto")
                          # "auto"     → 自动检测 rclpy
                          # "force_ros"→ 必须用 ROS；不可用则打日志 + 降级 Mock
                          # "force_mock" → 纯模拟（不订阅任何话题）
smgr.open()
# ... 正常业务 ...
smgr.close()

# 更推荐：上下文管理器（和 URDFController 一样）
with SensorManager.from_config(cfg) as smgr:
    ...
```

### 3.2 4 路倾角（维特 WT901C485 MODBUS ×4）

> 对齐 v12 `inclinometer_sensor_bridge` 的真实输出：一台桥接节点把 4
> 台 WT901C485 (地址 0x50~0x53) 合并成 **1 条 `Float64MultiArray`**。
> v15 在配置里声明 `array_index`，按索引拆包。

#### 3.2.1 原始 ROS 2 输入协议（与 v12 bridge 对齐）

| 项目 | 值 |
|------|----|
| msg 类型 | `std_msgs/msg/Float64MultiArray` |
| 默认 topic | `/excavator/inclinometer_pitch_deg`（4 路都一样，合并） |
| 默认 QoS | depth = 10（可改 per-sensor `qos_depth`） |
| data[i] 单位 | **度（deg）** — 绝对姿态角（Pitch 主方向），桥接直接透传 WT901 `AngX` |
| 拆包规则 (AC-9) | `tilt_bucket ↔ data[0] ↔ addr 0x50`；`tilt_arm ↔ data[1] ↔ 0x51`；`tilt_boom ↔ data[2] ↔ 0x52`；`tilt_swing ↔ data[3] ↔ 0x53` |
| 语义父子链（for 相对角） | `bucket → arm → boom → swing(根)`，`relative = child − parent` |

#### 3.2.2 v15 内部统一 `TiltReading`（STABLE）

所有字段均暴露为属性，可直接读：

```python
from v15_action_task import TiltReading

r: TiltReading | None = smgr.get_tilt("tilt_bucket")
r.id                    # str: "tilt_bucket"（稳定字符串 ID）
r.ts_s                  # float: 时间戳（秒）；优先取 msg.header.stamp，否则取 time.time()
r.is_valid              # bool: 本帧是否有效（Mock / 未收到前 =False）
r.raw_angle_deg         # float | None: 从 Float64MultiArray data[idx] 直接拿的值（度）
r.compensated_angle_deg # float | None: 扣零偏 + 可选 gyro 融合后的值（度）——业务层用这个！
r.temperature_c_or_none # float | None: 预留（桥接暂未透传温度字段）
r.roll_deg_or_none      # float | None: 预留（Roll）
r.pitch_deg_or_none     # float | None: 预留（Pitch，通常与 compensated 相同）
r.calibration_done      # bool: SensorManager.open() 后是否已累积够 calib_count 帧静止校准
r.calibration_remaining # int: 剩余需要多少帧校准完成（0 = 已 done）
r.relative_angle_deg    # float | None: 相对父关节的角度差（消除机体安装零偏）；swing 无父 = 自身 compensated
```

批量查询：`smgr.all_tilt() -> Dict[str, TiltReading]`；`smgr.list_tilt_ids() -> List[str]`。

#### 3.2.3 `TiltCompensator.update()` 纯算术流程（STABLE，不依赖 ROS）

> 业务层也可在**离线回放 / 算法调试**里**单独**使用这个类，完全不 new
> 任何 ROS 对象。对齐 v8 + v11 + v5 的综合方案：零偏校准（50 帧均值）
> + 陀螺仪互补滤波（alpha=0.98，IMU 标准值）+ gyro 死区 + 父子链相对相减。

```python
from v15_action_task import TiltCompensator, load_default_config

comp = TiltCompensator(load_default_config().tilt_compensation)
# 循环每一帧：
compensated_deg, calibration_done, remaining = comp.update(
    raw_angle_deg,                              # 原始输入（度）
    gyro_rad_s_or_none=gyro_rad,                # 可选：角速度（弧度/秒），v12 暂未接 =None
    dt_s=0.05,                                  # 可选：帧间隔（秒），gyro 积分用
)
# ① 校准阶段（前 calib_count 帧，默认 50）：
#     calibration_done=False，remaining 递减， compensated=中间占位值
# ② 校准完成后（calibration_done=True，remaining=0）：
#     compensated_deg = raw_angle_deg - zero_bias_deg
#     若传 gyro + dt + 有 last_compensated，则额外做 alpha 融合：
#        predicted = last_compensated_deg + deg(gyro_rad)*dt
#        fused = alpha*raw_comp + (1-alpha)*predicted
#        |gyro_rad| < gyro_deadzone_rad_s (默认 0.002) 时 gyro_deg_s 归零，消除静止噪声
```

### 3.3 最多 5 路雷达（PointCloud2）

> 默认启用 PaceCat M300-E 单线后向雷达（`lidar_single_rear`），其余
> 1 矩阵固态 + 2 环视 360 + 另一单线在配置里 `enabled=false`。可在
> `default_config.yaml` 里把 `enabled` 改为 True 并改 topic / frame_id /
> qos_depth，v15 会自动创建对应订阅，不改代码。

#### 3.3.1 原始 ROS 2 输入协议

| 项目 | 默认值（lidar_single_rear） | 其他雷达 |
|------|---------------------------|---------|
| msg 类型 | `sensor_msgs/msg/PointCloud2` | 同 |
| topic | `/pointcloud`（PaceCat 驱动真实话题） | 每雷达独立，可在配置覆盖 |
| frame_id | `lidar_single_rear_link` | 各自配置 |
| QoS depth | 10 | 可改 per-sensor |
| points_count | `msg.width * msg.height`（对单线通常 height=1，width=N） | 同 |

#### 3.3.2 v15 内部统一 `LidarReading`（STABLE）

```python
r = smgr.get_lidar("lidar_single_rear")
r.id, r.ts_s, r.is_valid,         # 与 TiltReading 语义相同
r.points_count,                   # int: width * height 估算点数
r.width_or_none,                  # int | None: PointCloud2 msg.width
r.height_or_none,                 # int | None: PointCloud2 msg.height
r.raw_ref_or_none,                # Any | None: 原始 msg 引用（ROS 模式）；Mock=None
```

批量：`smgr.all_lidar() / list_lidar_ids()`（默认返回 5 个 ID，不管启用与否，启用的才会 `is_valid=True`）。

### 3.4 ≥5 路相机视频流（Image + 可选 CameraInfo）

> 默认启用 `cam_front_main`（海康） + `cam_left`（网络相机 103） +
> `cam_right`（102），再加 `cam_rear / bucket_view / arm_overlook` 共
> 6 路占位。前主相机额外有 `info_topic` 订阅到 CameraInfo（可做去畸变 /
> PnP）。

#### 3.4.1 原始 ROS 2 输入协议

| 项目 | 默认值（cam_front_main） |
|------|---------------------------|
| Image msg 类型 | `sensor_msgs/msg/Image` |
| CameraInfo msg 类型 | `sensor_msgs/msg/CameraInfo`（如果 info_topic 非空） |
| Image topic | `/sensors/camera/front_main/image_raw` |
| Info topic（非空才有订阅） | `/sensors/camera/front_main/camera_info` |
| frame_id | `cam_front_main_link` |
| QoS depth | 10（可 per-sensor 改） |
| width / height / encoding | 直接透传 msg.width / msg.height / msg.encoding（例如 `"bgr8"`） |

#### 3.4.2 v15 内部统一 `CameraReading`（STABLE）

```python
r = smgr.get_camera("cam_front_main")
r.id, r.ts_s, r.is_valid,
r.width_px, r.height_px,            # int, int: 分辨率
r.encoding,                         # str: "bgr8" / "rgb8" / "mono8" / ...
r.seq_or_none,                      # int | None: msg.header.seq 若有
r.raw_ref_or_none,                  # Any | None: 原始 msg 引用（Mock=None）
```

批量：`smgr.all_camera() / list_camera_ids()`（默认返回 6 个 ID）。

### 3.5 外参与标定流程（PROVISIONAL，字段格式稳定）

`load_config(path)` 返回的 `cfg.extrinsics: ExtrinsicsConfig`
是传感器相对于 `base_link` 的 6-DOF 外参。默认条目对齐
`launch/sensors_tf.launch.py` 的出厂倒装值，可通过
`examples/ex11_sensor_tf_calibration_cmdline.py`（命令行标定器，SSH
无头） + `examples/ex12_sensor_tf_record_to_yaml.py`（record.txt → 兼容
JSON）的工具链覆盖。

```python
from v15_action_task import load_default_config, ExtrinsicsEntry

cfg = load_default_config()
ext: ExtrinsicsEntry | None = cfg.extrinsics.get("lidar_single_rear")
# 稳定字段
ext.sensor_id, ext.parent_frame, ext.child_frame,
ext.x_m, ext.y_m, ext.z_m,           # 米
ext.yaw_rad, ext.pitch_rad, ext.roll_rad,   # 弧度；顺序与 static_transform_publisher 一致
ext.enabled, ext.note,               # bool, str
# 纯函数方法（不依赖 tf2，纯 math）
ext.to_xyzrpy()                      # -> Tuple[6]: (x,y,z,yaw,pitch,roll)
ext.to_static_transform_publisher_args() # -> List[str]: ["-0.55","-0.2","1.2712","0.0532","0.0349","3.0316"]（空格分隔）
ext.to_4x4_matrix()                  # -> List[List[float]]: 标准 4×4 SE3
cfg.extrinsics.to_launch_static_tf_nodes_yaml()  # -> str：可直接复制进 launch static_transform_publisher CLI 行
```

<!-- prettier-ignore -->
> [!NOTE]
> AC-8（硬验收）已固化：`lidar_single_rear` 的
> `(x,y,z,yaw,pitch,roll) = (-0.5500, -0.2000, 1.2712, 0.0532, 0.0349, 3.0316)` 与
> 真实 launch 倒装值误差 < 1e-4。如果你在现场做了新标定，生成的新 JSON
> 通过 `load_config(new_json_path)` 读进来，上面的 3 个转换方法会自动使用
> 标定值，不需要改业务代码。

---

## 4. 新程序接入模板（3 种）

把下面任一模板复制到你的脚本，**按注释改 2~3 个参数**即可接入 v15。
所有模板都满足「Ubuntu 裸系统无 PyYAML / 无 rclpy 也不会崩」的契约。

### 模板 A：最小可用（Mock 后端，0 依赖，零配置）

```python
from v15_action_task import from_config, boom_move, arm_move

# 1) 一行拿工具链（Mock 后端，传感器默认不启）
ctx = from_config(adapter_backend="mock")
cfg = ctx["config"]

# 2) 运动（方向语义见 §2.4）
with ctx["controller"] as ctl:
    r1 = boom_move(ctl, up_or_down=True, angle_deg=10.0, cfg=cfg)    # 抬大臂 10°
    r2 = arm_move (ctl, up_or_down=False, angle_deg=5.0, cfg=cfg)   # 伸小臂 5°
    if r1 and r2:
        print("两段动作均到位，共等待", r1.waited_s + r2.waited_s, "s")

    # 或者走笛卡尔：下挖 1 m 前方 -0.2 m 深，铲斗角 -60°
    r = ctx["mover"].move_with_bucket(1.0, 0.0, -0.2, -60.0)
    print("下挖到位？", bool(r), "末端误差 mm",
          None if not r.final_tip_xyz else
          ((r.final_tip_xyz[0]-r.target_xyz[0])**2 +
           (r.final_tip_xyz[1]-r.target_xyz[1])**2 +
           (r.final_tip_xyz[2]-r.target_xyz[2])**2)**0.5 * 1000)
```

### 模板 B：完整工具链（ROS 2 后端 + 传感器 + 限位裁剪）

```python
from v15_action_task import from_config, load_config

# ① 加载自定义机型配置（YAML 优先，无 PyYAML 会自动找同名 .json，
#    再失败就 BUILTIN dict — 三种都能跑，放心写路径）
cfg = load_config("/opt/v15_configs/extrinsics_EXC-车间-007.json")  # 或 xxx.yaml

# ② 启动：RosV14Adapter 立刻 open()；传感器 auto 模式；限位裁剪自动开启
ctx = from_config(cfg,
                  adapter_backend="ros",   start_adapter=True,
                  init_sensors=True,       sensor_ros_mode="auto")

# ③ 控制 + 传感器 同时使用
with ctx["controller"] as ctl, ctx["sensor_manager"] as smgr:
    import time
    # 先等倾角校准（默认 50 帧 ≈ 3 s；用 calibration_done 轮询不阻塞）
    for _ in range(600):
        swing = smgr.get_tilt("tilt_swing")
        if swing and swing.calibration_done: break
        time.sleep(0.01)
    # 边做动作 边实时读倾角反馈
    r = from_config.__module__   # 占位，写法举例：
    # from v15_action_task import swing_move_to
    # r = swing_move_to(ctl, abs_yaw_deg=45.0, cfg=cfg)
    # bucket_comp = smgr.get_tilt("tilt_bucket").relative_angle_deg  # 父子链消除机体零偏
```

### 模板 C：先改连杆 / 限位，再跑（不用改 YAML，纯 dict 加载）

```python
from v15_action_task import V15Config, from_config

# ① 用 dict 覆盖你要改的项，剩下的用 BUILTIN 默认补齐（空 dict 也合法）
partial_cfg = {
    "model_name": "60FED_加长小臂 +240mm",
    "link_geometry": {"L2": 0.840},                              # L2 加长 +240 mm（其余不变）
    "joint_limits":  {"boom_swing": {"min_deg": -5, "max_deg": 40}},  # 改大臂上限
    "extrinsics": [                                               # 新标定值（ex12 导出来的）
        {"sensor_id": "lidar_single_rear", "parent_frame": "base_link", "child_frame": "lidar_single_rear_link",
         "x_m": -0.549, "y_m": -0.199, "z_m": 1.271, "yaw_rad": 0.053, "pitch_rad": 0.035, "roll_rad": 3.032,
         "enabled": True, "note": "from ex11 / ex12 现场标定"}
    ],
}
# ② V15Config.from_dict 自动为缺失的节（sensors / tilt_compensation / ...）
#    生成默认值 → 最终还是一份完整 9 节的 V15Config
cfg = V15Config.from_dict(partial_cfg)
ctx = from_config(cfg, adapter_backend="mock")
print("机型：", ctx["config"].model_name)
print("L2：", ctx["config"].link.L2, "m")  # 验证：=0.840
# 其余就和模板 A / B 一样跑了……
```

---

## 5. Next steps

- 想快速跑 33 条回归：进入 [examples/README.md](./examples/README.md) 的
  §3，执行 `python3 _run_all_checks.py`。
- 需要把 v15 接到真实液压 / 485 / CAN 硬件：参考 v15 主 README 的
  `§9. 扩展：接真实硬件`，按模板写一个 `HardwareAdapter` 放进
  `control_core/` 目录，`from_config(adapter_backend="mock")` 的路径
  全部不变，只需要把 `adapter_backend` 改成自定义后端即可（或在
  `from_config` 外面自己 `URDFController(HardwareAdapter(...))` 造）。
- 需要新增一种传感器（如第 6 路相机、第 7 路雷达）：不要改
  `SensorManager` 源码。直接在 `default_config.yaml` 对应的
  `sensors.cameras / sensors.lidars` 下追加条目并改
  `enabled: true`，`SensorsConfig.by_id / list_*_ids / ExtrinsicsEntry`
  的反射会自动把它拉进订阅与外参系统。
