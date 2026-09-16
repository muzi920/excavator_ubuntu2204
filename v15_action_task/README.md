# v15_action_task — 通用挖掘机独立控制库 (v15 Standard Library)

> **定位**：完全自洽、可独立迁移到任何 Python/ROS2 项目的挖掘机控制标准库。
> **控制协议**：与 `shandong_v14_urdf` 标定版 URDF 完全一致（`/joint_states` 话题），可直接驱动 URDF 在 RViz2 中运动。
> **零外部依赖**：不再引用 `shandong/v10_cailbration_arm/` 或 `shandong/v14_urdf/` 下的任何 Python 文件，连杆参数 / FK / IK 全部本地化。
> **标准目标**：后续所有挖掘机项目 / 任务的运动控制接口，**统一以 v15_action_task 的 API 为准**。

---

## 目录

- [1. 关键特性](#1-关键特性)
- [2. 文件结构](#2-文件结构)
- [3. 统一标准接口 (5 层 API)](#3-统一标准接口-5-层-api)
  - [3.1 顶层入口](#31-顶层入口)
  - [3.2 关节级控制 (control_core)](#32-关节级控制-control_core)
  - [3.3 运动学 (kinematics)](#33-运动学-kinematics)
  - [3.4 末端笛卡尔运动 (motion)](#34-末端笛卡尔运动-motion)
  - [3.5 动作库 (action_library, 可选)](#35-动作库-action_library-可选)
  - [3.6 配置层 (config)](#36-配置层-config)
- [4. 配置层详解 (config)](#4-配置层详解-config)
  - [4.1 from_config() 标准用法](#41-from_config-标准用法)
  - [4.2 default_config.yaml 6 大类字段](#42-default_configyaml-6-大类字段)
  - [4.3 三级兜底加载机制 (零依赖可用)](#43-三级兜底加载机制-零依赖可用)
  - [4.4 自定义机型配置示例](#44-自定义机型配置示例)
  - [4.5 限位裁剪深度集成](#45-限位裁剪深度集成)
- [5. 快速使用示例](#5-快速使用示例)
- [6. 在 RViz2 中实际跑通 (SSH 无头模式)](#6-在-rviz2-中实际跑通-ssh-无头模式)
- [7. 与 v10 的数学差异说明（必知）](#7-与-v10-的数学差异说明必知)
- [8. 可达域 x/y/z 可执行区间说明（默认 60FED 机型）](#8-可达域-xyz-可执行区间说明默认-60fed-机型)
- [9. 验证结果](#9-验证结果)
- [10. 扩展：接真实硬件](#10-扩展接真实硬件)
- [11. 标准控制协议](#11-标准控制协议与-v14-urdf-完全对齐不可更改)

---

## 1. 关键特性

- ✅ **4 自由度挖掘机运动学**：回转 (`swing_yaw`)、大臂 (`boom_swing`)、小臂 (`arm_boom`)、铲斗 (`bucket_arm`)
- ✅ **正向/逆向运动学 100% 数学自洽**：FK→IK→FK 闭环空间误差 **0.00000 mm**
- ✅ **铲斗角度自动搜索**：未指定铲斗角时，扫描 `-70°~+10°` 自动选最优解
- ✅ **Adapter 模式，后端可热插拔**：`RosV14Adapter`（真实 ROS 话题）/ `MockAdapter`（无 ROS 调试）/ 未来硬件串口
- ✅ **语义接口统一**：代码里只用 `swing_yaw / boom_swing / arm_boom / bucket_arm`（度），与 URDF 关节名的映射由 Adapter 自动处理
- ✅ **末端笛卡尔一行到位**：给铲尖 (x, y, z) → 内部 IK → 发布 → 轮询到位 → 返回实际到达位姿
- ✅ **4 层动作库**：`utils → primitives → composites → tasks`，一键生成挖掘-回转-卸料 JSON 剧本
- ✅ **YAML 配置驱动 (6 大类参数)**：连杆几何、关节限位、ROS 协议、关节映射、运动默认值、标准姿态 —— 全部抽离到 `config/default_config.yaml`，改机型不用改代码
- ✅ **三级兜底零依赖**：YAML (PyYAML) → JSON (标准库) → 内置 Python dict，完全干净的 Python 3.10+ 环境也能 100% 工作
- ✅ **from_config() 一键构建**：一行拿到 `{config, controller, adapter, fk, ik, mover}` 完整工具链，减少 80% 样板代码
- ✅ **限位裁剪深度集成**：URDFController 发布前自动按 YAML 限位裁剪，避免超限指令打到物理机器
- ✅ **100% 向后兼容**：所有不使用 config 层的旧代码（直接实例化 URDFController / CartesianMover）完全可用，零改动
- ✅ **可达域 5 层组合过滤**：根据臂长自动确定空间可达域（swing 限位 / ground 穿透 / transit 最小高度 / 腕点三角不等式 / 奇异边界 margin），可达域外自动拦截并给出最近可达点
- ✅ **关节空间双策略轨迹规划**：LERP（线性等分，液压步长场景）/ Trapezoidal（梯形速度曲线，加速度限幅场景）可切换；单关节运动过程轨迹与完整轨迹统一数据结构
- ✅ **三标准控制接口**：接口1=单点笛卡尔到位（含完整+单关节双轨迹）；接口2=挖掘→卸料转运（抬臂 up_m + 回转 swing_deg）；接口3=卸料过程（半开→小臂半推→全开+抖斗）

---

## 2. 文件结构

```
v15_action_task/
│
├── __init__.py                  ★ 顶层统一入口：from v15_action_task import X
├── drive_v14_in_rviz.py         ★ RViz2 端到端 Demo（6 段挖掘动作）
│
├── control_core/                🔧 控制协议层（与 v14 URDF 对齐）
│   ├── __init__.py              统一导出
│   ├── types.py                 语义关节顺序、URDF 映射、单位转换、限位宏
│   ├── adapter_base.py          ControlAdapter 抽象基类（所有后端必须实现它）
│   ├── mock_adapter.py          MockAdapter：内存后端，cmd→feedback 零延迟同步
│   ├── ros_v14_adapter.py       RosV14Adapter：ROS 2 Humble /joint_states 发布+订阅
│   │                              (QoS 10, frame_id="base_link", 首发同步反馈)
│   ├── urdf_controller.py       URDFController：统一对外 API（set_joint / set_pose / get_pose / is_at_pose）
│   └── demo_usage.py            control_core 自检测试脚本
│
├── kinematics/                  📐 本地化运动学（不引用 shandong/v10_*）
│   ├── __init__.py              统一导出
│   ├── link_params.py           LinkParams dataclass + DEFAULT_PARAMS（等效大臂 0.8799 m / beta=16.626°
│   │                              offset_x=0.25 offset_z=0.40 L1=0.35 L2=0.60 L_arm=0.44 L_bucket=0.26
│   │                              传感器偏置: boom=40.9° arm=19.6° bucket=-56.2°）
│   ├── forward.py               ForwardKinematics.solve() → FKSolution（4 关键点 XZ/3D 坐标）
│   └── inverse.py               InverseKinematics
│                                  - solve_bucket_pose(x,y,z, bucket_angle) → IKSolution
│                                  - search_bucket_angle(x,y,z, range, N) → 自动搜索最优铲斗角
│
├── motion/                      🚀 末端笛卡尔高层运动（★ 一行到位）
│   ├── __init__.py              统一导出
│   ├── cartesian_mover.py       - CartesianMover 类（面向对象，含 move_to_point 接口1）
│   │                              - move_to_cartesian() 纯函数
│   │                              - MoveResult dataclass（成功标志、实际到达关节/末端、等待时间、失败原因、轨迹字段）
│   ├── workspace.py             - WorkspaceChecker：5 层可达域过滤 + 最近可达点收缩
│   │                              - ReachabilityReport：success/reasons/closest_point
│   └── trajectory.py            - TrajectoryPlanner：双策略（LERP/Trapezoidal）关节空间轨迹
│                                  - JointTrajectoryPoint：单关节时间戳+角位置+速度数据点
│
├── config/                      ⚙️ YAML/JSON 配置层（★ 改机型不用改代码）
│   ├── __init__.py              导出 8 个 dataclass + load_config / load_default_config / BUILTIN_DEFAULT_CONFIG_DICT
│   ├── loader.py                - V15Config 聚合类（build_controller / build_kinematics / build_mover）
│                                  - 3 级兜底（YAML→JSON→内置 dict），真正零依赖
│                                  - JointMappingConfig / JointLimitsConfig / LinkGeometryConfig / RosProtocolConfig / MotionDefaultsConfig / StandardPosesConfig
│   └── default_config.yaml      ★ 默认 60FED 机型配置（6 大类，与硬编码数值 1:1 对齐，用户可直接 copy 改）
│
└── action_library/              📜 可选 4 层动作库（IK 也已本地化，不再依赖 v10）
    ├── __init__.py              统一导出
    ├── verify_library.py        动作库自检脚本（11/11 验证通过）
    │
    ├── utils/                   工具层
    │   ├── __init__.py
    │   ├── joint_limits.py      JOINT_LIMITS、clamp_pose、check_pose_limits、default_pose_deg
    │   ├── step_builder.py      StepBuilder：统一的动作步编号/构造器
    │   └── ik_wrapper.py        IKSolver（动作库友好的 IK 封装，内部调用本地 kinematics）
    │
    ├── primitives/              原语层
    │   ├── __init__.py
    │   ├── joint_motion.py      move_joint_step / move_joint_steps_independent
    │   └── bucket_control.py    close_bucket / half_open_for_dig / full_open_for_dump
    │
    ├── composites/              组合层
    │   ├── __init__.py
    │   ├── standard_poses.py    INIT / CYCLE_TRANSIT / HOME 标准姿态 + 到达函数
    │   ├── arm_motion.py        dig_entry_sequence / dump_release_sequence
    │   ├── swing_motion.py      align_swing / align_swing_to_point
    │   └── dig_dump_actions.py  - CompositeActionResult（聚合 step_results）
    │                              - move_to_dump_transit() 【接口2】抬臂+回转转运
    │                              - dump_material()        【接口3】卸料+抖斗过程
    │
    └── tasks/                   剧本层
        ├── __init__.py
        ├── single_dig_dump.py   build_single_dig_dump_task() / build_single_dig_dump_script()
        └── multi_dig_cycle.py   build_multi_dig_task() / build_multi_dig_cycles()
```

---

## 3. 统一标准接口 (5 层 API)

### 3.1 顶层入口

**其他项目一律使用这一个入口，不要单独 import 子包：**

```python
from v15_action_task import (
    # ── Control (control_core) ──────────────────────
    URDFController,        # 统一控制器（with 上下文）
    ControlAdapter,        # 后端抽象基类（自定义后端时继承）
    MockAdapter,           # 无 ROS 调试
    RosV14Adapter,         # 真实 ROS 2 v14 URDF 协议
    # 类型与工具
    SEMANTIC_JOINT_ORDER,  # ["swing_yaw","boom_swing","arm_boom","bucket_arm"]
    SEMANTIC_TO_URDF,      # {"swing_yaw":"swing_joint", ...}
    URDF_JOINT_ORDER,      # ["swing_joint","boom_joint","arm_joint","bucket_joint"]
    DEFAULT_FRAME_ID,      # "base_link"
    deg_to_rad, rad_to_deg,
    default_pose_deg,      # 全 0 姿态
    # ── Kinematics (kinematics) ────────────────────
    LinkParams,            # 连杆参数 dataclass（可 UI 层替换）
    DEFAULT_PARAMS,        # 默认挖掘机连杆 + 标定参数
    FKSolution,            # FK 结果 dataclass
    ForwardKinematics,     # FK 求解器
    IKSolution,            # IK 结果 dataclass (as_pose() 直接喂给 set_pose)
    InverseKinematics,     # IK 求解器（solve_bucket_pose + search_bucket_angle）
    # ── Motion (motion) ────────────────────────────
    CartesianMover,        # 面向对象封装（可保存默认容差/超时）
    MoveResult,            # 移动结果（success / reached_pose_deg / final_tip_xyz / ...）
    move_to_cartesian,     # 纯函数式：一行到位
    # ── Action Library (可选) ──────────────────────
    # from v15_action_task.action_library import StepBuilder, build_single_dig_dump_task
)
```

### 3.2 关节级控制 (control_core)

```python
from v15_action_task import URDFController, RosV14Adapter, MockAdapter

# --- 有 ROS：驱动真实 /joint_states (与 v14 URDF 对齐) ---
with URDFController(RosV14Adapter()) as ctl:
    ctl.set_joint("swing_yaw", 15.0)              # 单关节（度）
    ctl.set_pose({"boom_swing": 30, "arm_boom": 50})  # 多个关节同时
    current = ctl.get_pose_blocking(timeout_s=2.0)     # 阻塞等反馈
    if ctl.is_at_pose(target, tolerance_deg=1.0):
        print("到位")

# --- 无 ROS：MockAdapter 调试（零延迟同步）---
with URDFController(MockAdapter()) as ctl:
    ctl.set_pose({"boom_swing": 20, "arm_boom": 40, "bucket_arm": -70})
    print(ctl.get_pose_or_default())
```

### 3.3 运动学 (kinematics)

```python
from v15_action_task import ForwardKinematics, InverseKinematics

fk = ForwardKinematics()
ik = InverseKinematics()

# FK: 4 个语义关节 → 铲尖 3D + 所有关键点
fk_sol = fk.solve(boom_swing_deg=20, arm_boom_deg=40, bucket_arm_deg=-70, swing_yaw_deg=10)
x, y, z = fk_sol.bucket_tip_3d       # (1.46, 0.25, 0.46) m
bucket_abs_angle = fk_sol.abs_bucket_deg  # 铲斗绝对几何角（向上为正）

# IK 指定铲斗角：铲尖 3D + 铲斗绝对角 → 4 个语义关节
ik_sol = ik.solve_bucket_pose(1.0, 0.0, -0.2, bucket_abs_angle_deg=-60.0)
pose_cmd = ik_sol.as_pose()         # {"swing_yaw":0.0, "boom_swing":..., ...}

# IK 自动搜索铲斗角（未知铲斗角，选最平稳的解）
ik_sol = ik.search_bucket_angle(0.9, 0.2, 0.0)   # bucket_range 默认 (-70°,+10°) 17 个候选
```

### 3.4 末端笛卡尔运动 (motion)

```python
from v15_action_task import URDFController, MockAdapter, InverseKinematics
from v15_action_task import CartesianMover, move_to_cartesian

ik = InverseKinematics()

# --- 方式 A：纯函数式 ---
with URDFController(MockAdapter()) as ctl:
    result = move_to_cartesian(
        ctl, ik,
        x=1.0, y=0.0, z=-0.2,
        bucket_angle_deg=-60.0,    # None = 自动搜索
        blocking=True, tolerance_deg=1.0, timeout_s=3.0,
    )
    if result:
        print(f"成功到位！铲尖实际到达 {result.final_tip_xyz}")
        print(f"关节指令 {result.reached_pose_deg}")
    else:
        print(f"失败原因：{result.reason}")

# --- 方式 B：面向对象（容差/超时可统一配置）---
with URDFController(MockAdapter()) as ctl:
    mover = CartesianMover(
        ctl, ik,
        default_tolerance_deg=2.0,
        default_timeout_s=5.0,
        default_bucket_range=(-60.0, +10.0),
    )
    # ① 下挖点
    r1 = mover.move_with_bucket(1.0, 0.0, -0.25, -60.0)
    # ② 提斗（自动搜索铲斗角）
    r2 = mover.move(0.9, 0.0, 0.15)
    # ③ 左转 30° 卸料
    r3 = mover.move_with_bucket(0.95, +0.55, 0.05, +10.0)
    # 读取当前实时位置和铲尖
    print(f"当前关节 {mover.current_pose()}")
    print(f"当前铲尖 {mover.current_tip()}")

    # ── 方式 C：可达域预检（避免发出不可达指令）──
    from v15_action_task import WorkspaceChecker
    ws = WorkspaceChecker.from_config(ctx["config"])
    rep = ws.check_point_reachable((1.2, 0.0, 0.8), mode="transit")
    if rep.success:
        r = mover.move_to_point((1.2, 0.0, 0.8), mode="transit")
        # r.trajectory_plan 是完整轨迹（List[PoseDeg]，可回放）
        # r.joint_trajectories 是 4 关节各自的单关节过程轨迹（时间戳+角度+速度）
    else:
        print(f"不可达，原因: {rep.reasons}，最近可达点: {rep.closest_point}")

    # ── 方式 D：手动生成轨迹（双策略切换）──
    from v15_action_task import TrajectoryPlanner
    tp = TrajectoryPlanner.from_config(ctx["config"], workspace_checker=ws, ik=ctx["ik"])
    plan1 = tp.plan_segment(
        target_xyz=(1.0, 0.0, 0.2),
        start_pose_deg=ctx["controller"].get_pose_or_default(),
        strategy="lerp",             # 线性等分：液压步长场景
        bucket_angle_deg=None,
        mode="transit",
    )
    plan2 = tp.plan_segment(
        target_xyz=(1.2, 0.3, 0.5),
        start_pose_deg=plan1.final_pose_deg,
        strategy="trapezoidal",      # 梯形速度：加速度限幅场景
        mode="transit",
    )
    # 每个 waypoint 直接喂 ctl.set_pose(wp, blocking=False)，步长=tp.step_duration_s
```

### 3.5 动作库 (action_library, 可选)

```python
from v15_action_task.action_library import build_single_dig_dump_task, StepBuilder

# 一键生成挖掘-回转-卸料 的 20+ 步标准剧本
task = build_single_dig_dump_task(
    dig_x=1.0, dig_z=-0.25,         # 挖掘点
    dump_yaw_deg=60.0,              # 卸料回转角
    swing_speed_deg_s=15.0,         # 回转速度
)
# task["script"] → 每步 target_deg、duration_s 可直接步进下发给 ctl.set_pose()
```

### 3.6 配置层 (config)

> **推荐所有新项目使用**：把所有硬编码参数（连杆/限位/ROS 话题/映射…）抽到一个 YAML 里，改机型**不用改 Python 代码**。

```python
from v15_action_task import from_config, load_config, load_default_config, V15Config

# ── ① 一行拿到完整工具链（最常用）──
ctx = from_config()                        # 默认 60FED + Mock 后端
# ctx = from_config("/my_model.yaml",      # 自定义 YAML
#                   adapter_backend="ros", start_adapter=True,
#                   init_sensors=True, sensor_ros_mode="auto")
with ctx["controller"] as ctl:
    ctx["mover"].move(1.0, 0.0, -0.2)      # 一行笛卡尔到位

# 返回值 dict 的 9 个 key (init_sensors/init_workspace/init_trajectory_planner 默认 False，旧 API 零 break):
#   config              → V15Config 对象（可继续 build_* 分块构造）
#   controller          → URDFController 实例（已接好 adapter + 限位裁剪）
#   adapter             → MockAdapter / RosV14Adapter
#   fk                  → ForwardKinematics（连杆来自 YAML）
#   ik                  → InverseKinematics
#   mover               → CartesianMover（容差/超时/铲斗搜索参数均来自 YAML）
#   sensor_manager      → SensorManager (可选) 或 None
#   workspace_checker   → WorkspaceChecker (可选，init_workspace=True 时构造) 或 None
#   trajectory_planner  → TrajectoryPlanner (可选，init_trajectory_planner=True 时构造) 或 None

# ── ② 单独加载配置，分块构造（灵活扩展）──
cfg = load_default_config()                # 加载包内 default_config.yaml
# cfg = load_config("/my_model.yaml")      # 加载自定义文件
# cfg = V15Config.from_dict({...})         # 直接传 Python dict
fk, ik = cfg.build_kinematics()            # 单独造 FK/IK
adapter = RosV14Adapter.from_config(cfg)   # 单独造 ROS Adapter
ctl = cfg.build_controller(adapter)        # 单独造 Controller（默认带限位裁剪）
```

### 3.7 作业指令库（语义动作 8 指令）

> 用户指定接口：`arm_move(bool up_or_down, float angle_deg)` 风格的布尔方向 + 角度增量 API。所有 8 函数统一返回 `MoveResult`，直接复用 `motion` 层已定义的结果类型。

```python
from v15_action_task import (
    from_config,
    swing_move, boom_move, arm_move, bucket_move,      # ① 四个基础增量：布尔方向语义 + 角度 (deg)
    swing_move_to, boom_lift, arm_extend, bucket_tilt_to,  # ② 四个扩展：绝对/笛卡尔便利函数
)

ctx = from_config(adapter_backend="mock")
cfg = ctx["config"]
with ctx["controller"] as ctl:
    # =============== ① 基础增量：布尔方向语义表 ===============
    # swing  right_or_left=True  → swing_yaw += 正 = 顺时针 (CW)
    # boom   up_or_down=True    → boom_swing -= 负 = 向上抬起 (boom lift)
    # arm    up_or_down=True    → arm_boom += 正 = 小臂向机身收回 (arm retract)
    # bucket up_or_down=True    → bucket_arm += 正 = 收斗/卷铲 (bucket curl)
    r = swing_move (ctl, right_or_left=True,  angle_deg=10.0, cfg=cfg)   # 顺时针转 10°
    r = boom_move  (ctl, up_or_down=True,    angle_deg=5.0,  cfg=cfg)   # 抬大臂 5°
    r = arm_move   (ctl, up_or_down=True,    angle_deg=15.0, cfg=cfg)   # 收小臂 15°
    r = bucket_move(ctl, up_or_down=False,   angle_deg=30.0, cfg=cfg)   # 铲斗外扬 30° (dump)
    if r:
        print("到位成功，等待时间", r.waited_s, "s")

    # =============== ② 扩展便利函数 ===============
    r = swing_move_to(ctl, abs_yaw_deg=45.0, cfg=cfg)                    # 回转直接到 45°
    r = boom_lift(ctx["mover"], delta_z_m=+0.10, keep_xy=True)           # 笛卡尔：保持XY 抬末端Z 10cm
    r = arm_extend(ctx["mover"], delta_x_m=+0.05, keep_yz=True)          # 笛卡尔：保持YZ 伸末端X 5cm
    r = bucket_tilt_to(ctl, abs_bucket_tip_deg=-45.0, cfg=cfg)           # 铲齿尖绝对倾角 45° 外翻
```

### 3.8 传感器订阅接口 (sensors)

> **v15 独立零依赖**：不 import 任何 v0~v14 代码；rclpy 用 try/except 在运行时动态导入，缺失时自动降级 Mock，不崩溃。配置层 (§4.7) 已经把 4 倾角 + 5 雷达 + ≥5 相机的话题、msg 类型、modbus_addr、外参 6 参数、温漂参数 全部 YAML 化。

```python
from v15_action_task import from_config

ctx = from_config(
    adapter_backend="mock",
    init_sensors=True,         # 打开 SensorManager（默认 False 省资源）
    sensor_ros_mode="auto",    # "auto"=有 rclpy 就开 ROS 订阅；"force_mock"=纯模拟；"force_ros"=强制 ROS
)
cfg = ctx["config"]

# ============== 4 路倾角 + 温漂补偿 ==============
#   cfg 对应硬件：维特 WT901C485 ×4 (地址 0x50/51/52/53)
#   桥接节点已由 v12 inclinometer_sensor_bridge 发布单一话题 Float64MultiArray /excavator/inclinometer_pitch_deg
#   配置层 sensors.tilt_sensors 声明 array_index=0/1/2/3 拆分；TiltCompensator 自动：
#     (a) 启动 50 帧静止零偏校准 → Reading.calibration_done + calibration_remaining
#     (b) 零偏扣减 → Reading.compensated_angle_deg
#     (c) 可选 alpha=0.98 陀螺仪互补滤波（若 gyro 话题接入）
#     (d) 父子链相对角相减 → Reading.relative_angle_deg (tilt_bucket = bucket - arm)
with ctx["sensor_manager"] as smgr:
    import time
    time.sleep(0.2)  # ROS 模式下等待前几帧进来 / Mock 模式瞬间完成
    for tid in smgr.list_tilt_ids():
        r = smgr.get_tilt(tid)
        if r and r.calibration_done:
            print(f"[tilt {tid}]  原始={r.raw_angle_deg:6.2f}°  补偿={r.compensated_angle_deg:6.2f}°  相对={r.relative_angle_deg}")

# ============== 5 路雷达 ==============
#   cfg.sensors.lidars: lidar_matrix / lidar_360_left / lidar_360_right / lidar_single_rear / lidar_single_front
#   出厂 PaceCat M300-E 单线 (lidar_single_rear) 默认 enabled=true，msg=PointCloud2 topic=/pointcloud
#   传感器外参 (§4.8): ExtrinsicsEntry.to_static_transform_publisher_args() 直接复用 v5 TF 格式
for lid in smgr.list_lidar_ids():
    r = smgr.get_lidar(lid)
    if r and r.is_valid:
        ext = cfg.extrinsics.get(lid)  # ExtrinsicsEntry: 6-DOF + to_4x4_matrix()
        print(f"[lidar {lid}]  点数={r.points_count} 外参 x={ext.x_m:.3f} y={ext.y_m:.3f} z={ext.z_m:.3f}")

# ============== ≥5 路相机视频流 ==============
for cid in smgr.list_camera_ids():
    r = smgr.get_camera(cid)
    if r and r.is_valid:
        print(f"[camera {cid}]  {r.width_px}×{r.height_px} {r.encoding}")
```

### 3.9 独立 examples 脚本目录

> 所有测试/演示/标定脚本**一律放在 `v15_action_task/examples/` 下**，正式代码 (`control_core / sensors / action_library / config / ...`) 目录里不会出现任何 `exNN_*.py`，保证发布时干干净净。运行方式：

```bash
cd v15_action_task/examples/

# ============= ① 作业/配置/运动 核心演示 =============
python3 ex01_joint_limits_three_layer_clamp.py   # 三层限位夹紧：Config→URDFController→动作库
python3 ex02_semantic_direction_AC7_self_assert.py  # 【AC-7 硬验收】8 方向语义断言，全部 PASS 才 exit 0
python3 ex03_eight_semantic_commands.py          # 8 指令表：4 增量 + 4 扩展，MoveResult 表
python3 ex07_fk_ik_closed_loop.py                # FK→IK→FK 闭环，4 关节累计误差 <1° 才算 PASS
python3 ex08_cartesian_mover_demo.py             # move_with_bucket 一行到位 + bool() 简写法
python3 ex09_task_builders_action_library.py     # build_single_dig_dump_script 纯构建演示
python3 ex10_physical_240mm_delta_L.py           # 【硬核】连杆改 +240mm，FK 末端增量=240mm 证明配置真实生效

# ============= ② 传感器接口 =============
python3 ex04_sensors_mock_14_channels.py         # 4 倾角 + 5 雷达 + 6 相机 = 15 路全枚举
python3 ex05_sensors_ros_optional.py             # 无 rclpy 的机器会打印 SKIP 并 exit 0 (无 ROS 不崩溃)
python3 ex06_full_pipeline_sensors_motion.py     # 控制器 + 传感器 完整管道冒烟

# ============= ③ 雷达预标定 (用户反馈：独立、不每次都跑) =============
# Step A：现场 SSH 无头 命令行滑条标定 (对标 v5 tf_calibration_gui.py)
#   支持命令：x+ x- y+ y- z+ z+ rz+/- ry+/- rx+/- show save quit
#   默认初始值 = sensors_tf.launch.py 出厂倒装值 (roll=3.0316 rad)
python3 ex11_sensor_tf_calibration_cmdline.py \
    --sensor-id lidar_single_rear --parent base_link --child lidar_single_rear_link \
    --default-x -0.5500 --default-y -0.2000 --default-z 1.2712 \
    --default-ry 0.0532  --default-pp 0.0349 --default-rr 3.0316 \
    --output /tmp/tf_calibration_record.txt
#   ↑ 进入交互 cmd> 提示符；回车 save → 一行 8 tokens：x y z yaw_rad pitch_rad roll_rad parent_frame child_frame

# Step B：record.txt → v15 配置 (JSON/纯 dict，无 PyYAML 也能加载 = load_config(json_path) 三级兜底)
python3 ex12_sensor_tf_record_to_yaml.py \
    --input /tmp/tf_calibration_record.txt \
    --sensor-id-override lidar_single_rear --chassis-sn EXC-TEST01 \
    --out /tmp/extrinsics_EXC-TEST01.json
#   ↑ 自检 6 参数误差 <1e-4，PASS 才 exit 0 (AC-10 闭环)

# ============= ④ Phase3 可达域 / 轨迹 / 三接口 =============
python3 ex13_workspace_reachability_AC1_2_3.py   # AC-1/2/3 可达域：边界通过/远点超限/ground穿透
python3 ex14_trajectory_planner_AC4_5.py       # AC-4/5 双策略轨迹：LERP线性+Trapezoidal
python3 ex15_three_control_interfaces_AC6_7_8.py  # AC-6/7/8 三接口：单点到位/转运/卸料

# ============= ⑤ 一键 43 项 回归（推荐）=============
python3 _run_all_checks.py   # 遍历 Phase0-5 33项 + Phase3 10项 = 43 TR，exit 0 = 全过；自动适应 无 PyYAML/无 rclpy
```

---

## 4. 配置层详解 (config)

### 4.1 from_config() 标准用法

`from_config()` 是 v15 面向新用户的**一键入口**，内部自动完成 6 步：解析配置 → 应用全局常量 → 构造 Adapter → 构造 Controller（带限位裁剪）→ 构造 FK/IK → 构造 CartesianMover。

#### 用法 A：Mock 后端（无 ROS、零依赖，可直接复制运行）

```python
from v15_action_task import from_config

ctx = from_config(adapter_backend="mock")   # None=默认 60FED 配置
with ctx["controller"] as ctl:
    # 下挖点（指定铲斗角 -60°）
    r1 = ctx["mover"].move_with_bucket(1.0, 0.0, -0.20, -60.0)
    # 提斗（自动搜索最优铲斗角）
    r2 = ctx["mover"].move(0.9, 0.0, 0.10)
    # 左转 30° 卸料
    r3 = ctx["mover"].move_with_bucket(0.95, +0.5, 0.00, +10.0)
    for name, r in [("下挖", r1), ("提斗", r2), ("卸料", r3)]:
        tx, ty, tz = r.target_xyz
        fx, fy, fz = r.final_tip_xyz
        err = ((fx-tx)**2+(fy-ty)**2+(fz-tz)**2)**0.5 * 1000
        print(f"[{name:>2}] 末端误差 {err:.4f}mm ({'✓' if r else '✗'})")
```

预期输出（Mock 零延迟同步，误差 0.0000mm）：
```
[下挖] 末端误差 0.0000mm (✓)
[提斗] 末端误差 0.0000mm (✓)
[卸料] 末端误差 0.0000mm (✓)
```

#### 用法 B：ROS 后端（驱动 v14 URDF / 真实硬件）

```python
from v15_action_task import from_config

# start_adapter=True 立即 rclpy.init + 建节点（有 ROS 2 环境时）
ctx = from_config(
    "/path/to/my_excavator.yaml",           # 自定义机型配置（留空=默认）
    adapter_backend="ros",
    start_adapter=True,
    use_config_limits=True,                 # 发布前自动按 YAML 限位裁剪（默认开）
)
with ctx["controller"] as ctl:
    ctx["mover"].move(0.95, 0.0, 0.10)      # 一行到位，RViz2 模型同步动
```

> 💡 `from_config(None)` 与 `load_default_config()` 等价，都走**三级兜底链**（见 4.3 节），在完全无 PyYAML / 无网络 / 甚至包内 YAML 文件丢失的环境也能 100% 跑起来。

---

### 4.2 default_config.yaml 11 大类字段

默认配置文件位于 [`config/default_config.yaml`](config/default_config.yaml)，所有数值与原 v10 标定、v14 URDF 硬编码**1:1 完全对齐**。可直接复制改名为 `my_excavator.yaml` 修改使用。

11 大类一览（第 10/11 类新增自 Phase3 可达域+轨迹）：
| # | 大类名 | 作用 |
|---|--------|------|
| 1 | `joint_mapping` | 语义关节 ↔ URDF 关节映射 |
| 2 | `joint_limits` | 4 关节限位（含 AC-10 方向语义注释块）|
| 3 | `link_geometry` | 折弯模型连杆几何 + 传感器零点 |
| 4 | `ros_protocol` | ROS 2 话题协议（与 v14 URDF 对齐）|
| 5 | `standard_poses` | INIT/HOME/CYCLE_TRANSIT 标准姿态 |
| 6 | `motion_defaults` | 到位容差 / 超时 / 铲斗搜索范围 |
| 7 | `sensors` | 4 倾角 + 5 雷达 + ≥5 相机话题 |
| 8 | `extrinsics` | 传感器外参 6-DOF（出厂值与 v5 TF 一致）|
| 9 | `tilt_compensation` | 倾角零偏校准 + 互补滤波参数 |
| **10** | **`workspace`** | 可达域：transit_min_z / 奇异 margin / outer_radius_*（任务1 配置入口）|
| **11** | **`trajectory`** | 轨迹：step_duration_s / 四关节 max_speed_deg_s / max_accel_deg_s2（任务2 配置入口）|

#### 4.2.0 元信息（顶部 3 字段）

| 字段 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `v15_config_version` | str | `"1.0"` | 配置协议版本号，未来扩展时用 |
| `model_name` | str | `"shandong_60FED_default"` | 机型标识字符串 |
| `description` | str | *省略* | 机型描述（仅调试/日志用，不影响计算） |

#### 4.2.1 第 1 类：语义关节 ↔ URDF 关节映射 (`joint_mapping`)

> 一般 URDF 写死了关节名，**通常不需要改**。换 URDF 但关节名不同时只改这里。

| Key（语义名，代码里用） | Value（URDF 关节名，wire 上用） | 说明 |
|---|---|---|
| `swing_yaw`  | `swing_joint`  | 回转（Z 轴） |
| `boom_swing` | `boom_joint`   | 大臂相对上车 |
| `arm_boom`   | `arm_joint`    | 小臂相对大臂 |
| `bucket_arm` | `bucket_joint` | 铲斗相对小臂 |

#### 4.2.2 第 2 类：关节限位 (`joint_limits`) — **单位：度 (deg)**

| 语义关节 key | 字段 | 默认值 | 取值约束 | 说明 |
|---|---|---|---|---|
| **swing_yaw**（回转） | `min_deg` / `max_deg` | `-180.0` / `180.0` | 实数，min ≤ max | 左右回转各 ±180° 全周 |
| **boom_swing**（大臂） | `min_deg` / `max_deg` | `-5.0` / `55.0` | 实数 | 负=抬起，正=下探；0° 为水平标定点 |
| **arm_boom**（小臂） | `min_deg` / `max_deg` | `0.0` / `130.0` | ≥0 | 0°=完全伸直，值越大越收回 |
| **bucket_arm**（铲斗） | `min_deg` / `max_deg` | `-95.0` / `45.0` | 实数 | 负=开斗卸料，正=收斗挖掘 |

> 限位深度集成到 URDFController（见 4.5 节），`from_config(use_config_limits=True)` 时，任何超限指令在发布前自动裁剪到合法区间，避免打到硬件。

#### 4.2.3 第 3 类：连杆几何 (`link_geometry`) — **长度单位：米 (m)，角度单位：度 (deg)**

大臂采用折弯模型：回转中心 → (offset_x, offset_z) → 大臂销轴 → (L1) → 折弯点 → (L2, boom_bend_angle 内夹角) → 小臂连接点 → (L_arm) → 铲斗铰点 → (L_bucket) → 铲尖。

| 字段 | 默认值 | 取值约束 | 说明 |
|---|---|---|---|
| `offset_x` | `0.25` | ≥0 | 回转中心 → 大臂销轴 X 方向偏移 |
| `offset_z` | `0.40` | ≥0 | 回转中心 → 大臂销轴 Z 方向偏移 |
| `L1` | `0.35` | >0 | 大臂第一段（销轴 → 折弯点） |
| `L2` | `0.60` | >0 | 大臂第二段（折弯点 → 小臂连接点） |
| `boom_bend_angle_deg` | `46.0` | (0, 180) | 大臂两段之间的结构折弯角（内夹角） |
| `L_arm` | `0.44` | >0 | 小臂长度（boom tip → bucket pivot） |
| `L_bucket` | `0.26` | >0 | 铲斗长度（bucket pivot → tip） |
| `sensor_offsets_deg.boom`   | `40.9`  | 实数 | 大臂传感器零点偏置 (FK: abs = offset - sensor) |
| `sensor_offsets_deg.arm`    | `19.6`  | 实数 | 小臂传感器零点偏置 |
| `sensor_offsets_deg.bucket` | `-56.2` | 实数 | 铲斗传感器零点偏置（**v10 原 bug 已修正，见第 7 章**） |

> **物理生效验证**：仅把 `L_bucket` 从 0.26m → 0.50m，FK 算出的 `arm_tip → bucket_tip` 空间距离增量为 **240.00 mm**，与理论值 (0.50-0.26)×1000=240mm 误差 <0.01mm，证明配置真正进入几何计算链路。

#### 4.2.4 第 4 类：ROS 2 话题协议 (`ros_protocol`)

| 字段 | 默认值 | 说明 |
|---|---|---|
| `node_name` | `"v15_urdf_controller"` | ROS 2 节点名 |
| `joint_topic` | `"/joint_states"` | 发布/订阅的关节状态话题（与 v14 URDF 对齐） |
| `frame_id` | `"base_link"` | **消息头帧 ID，不能为空**（否则 RViz 会丢消息） |
| `qos_depth` | `10` | KeepLast QoS 队列深度（与 ros_joint_bridge.py 一致） |
| `msg_type` | `"sensor_msgs/msg/JointState"` | 仅供文档说明，实际代码里写死 |
| `first_publish_sync_from_feedback` | `true` | 首次发布前若已收到反馈，则把 feedback 的非指定关节同步到 cmd，保证未涉及的关节保持原位不动 |

#### 4.2.5 第 5 类：标准姿态 (`standard_poses`) — **单位：度 (deg)**

动作库 `standard_poses.py` 里的 INIT / HOME / CYCLE_TRANSIT 默认值来源，也可扩展加自定义姿态（如 `dig_ready`、`dump_ready`）。

| 姿态名 | swing_yaw | boom_swing | arm_boom | bucket_arm | 典型用途 |
|---|---|---|---|---|---|
| `init`（上电待机） | 0.0 | 5.0 | 60.0 | 10.0 | 大臂略抬、小臂半收、铲斗闭合 |
| `home`（运输） | 0.0 | 0.0 | 120.0 | 30.0 | 全缩回，避免运输刮蹭 |
| `cycle_transit`（举臂中转） | 0.0 | 15.0 | 70.0 | -20.0 | 举臂避让地面、回转不刮料堆 |

#### 4.2.6 第 6 类：运动层默认参数 (`motion_defaults`)

| 字段 | 默认值 | 单位 | 说明 |
|---|---|---|---|
| `at_pose_tolerance_deg` | `1.0` | 度/关节 | `is_at_pose()` 每关节独立判断的到位容差 |
| `move_timeout_s` | `3.0` | 秒 | `move_to_cartesian()` / `mover.move()` 超时时间 |
| `bucket_search_range_deg` | `[-70.0, 10.0]` | 度 | 未指定铲斗角时，`search_bucket_angle()` 的扫描范围 |
| `bucket_search_samples` | `17` | 个 | 扫描样本数（等分区间，含两端）；17 对应步长 5° |

---

### 4.3 三级兜底加载机制 (零依赖可用)

v15 作为"可独立迁移的通用控制库"，配置加载设计了**三级容错链**，保证哪怕拷到一个完全干净的 Python 3.10+ 环境（连 `pip`、`PyYAML`、甚至包内 `default_config.yaml` 文件都没有），`from_config()` / `load_default_config()` 仍 100% 工作。

加载优先级（从高到低，上一级失败自动 fallthrough 到下一级）：

| 级别 | 触发条件 | 实现方式 | 依赖 | 适用场景 |
|---|---|---|---|---|
| **① YAML 加载** | `import yaml` 成功 + 文件存在 | `PyYAML` 的 `yaml.safe_load()` 解析 `default_config.yaml` | PyYAML (pip 包) | 标准开发环境、有网络、装了依赖 |
| **② JSON 兜底** | 无 PyYAML，但同路径同文件名的 `.json` 存在 | 标准库 `json.load()` 解析 `default_config.json` | Python 标准库 json（自带，零外部依赖） | 当前 miniconda 环境（无 PyYAML + 无法 pip 联网）：手动把 YAML 另存为同目录同名 JSON 即可 |
| **③ 内置 dict 最后防线** | 前两级全部失败 | 直接用 [`loader.py`](config/loader.py) 内 `BUILTIN_DEFAULT_CONFIG_DICT` 常量（纯 Python dict，数值与 YAML 1:1 等价） | 零任何依赖（仅 Python 解释器） | 超纯净环境、离线嵌入式、代码随包分发的任何场景 |

#### 当前 miniconda 环境（无 PyYAML + 无网络）的正确用法

方法 1（推荐，最简单）：**什么都不用做**，`from_config()` 会自动走第 ③ 级内置 dict。

方法 2（需要自定义机型配置）：把自定义 YAML **另存一份同名 `.json`** 放在同目录，`load_config("/path/to/my.yaml")` 会在 PyYAML ImportError 时自动读 `my.json`。

方法 3（不改文件）：直接传 Python dict 给 `from_config(dict_obj)` 或 `load_config(dict_obj)`。

```python
# 方法 3：纯 Python dict 传配置（完全零文件零依赖）
my_cfg_dict = {
    "v15_config_version": "1.0",
    "model_name": "custom_60FED_long_bucket",
    "link_geometry": {
        "L_bucket": 0.50,            # 加长铲斗到 0.50m
        "sensor_offsets_deg": {"boom": 40.9, "arm": 19.6, "bucket": -56.2},
        # 其余字段自动用 BUILTIN dict 兜底默认值
    },
    "joint_limits": {
        "boom_swing": {"min_deg": -5.0, "max_deg": 40.0},  # 收紧大臂下探限位
    },
}
ctx = from_config(my_cfg_dict, adapter_backend="mock")
```

---

### 4.4 自定义机型配置示例

假设我们要把 60FED **加长铲斗**（0.26m → 0.50m，方便抓散料）、**收紧大臂下探限位**（55° → 40°，避免铲尖磕履带）、**把 ROS 话题改为非默认名**（多机并行时区分）。

#### 示例 1：完整 YAML 配置（需 PyYAML 或另存同名 JSON）

```yaml
# my_custom_60FED.yaml — 复制 default_config.yaml 后只改 3 处即可
v15_config_version: "1.0"
model_name: "custom_60FED_long_bucket"
description: "60FED 加长铲斗版 + 收紧大臂限位 + 自定义 ROS 话题"

# ① 关节映射（一般不改，保持默认）
joint_mapping:
  swing_yaw:  swing_joint
  boom_swing: boom_joint
  arm_boom:   arm_joint
  bucket_arm: bucket_joint

# ② 关节限位 —— 只改 boom_swing max_deg 55→40
joint_limits:
  swing_yaw:  {min_deg: -180.0, max_deg: 180.0, description: "回转"}
  boom_swing: {min_deg:   -5.0, max_deg:  40.0, description: "大臂（限 40°，防磕履带）"}
  arm_boom:   {min_deg:    0.0, max_deg: 130.0, description: "小臂"}
  bucket_arm: {min_deg:  -95.0, max_deg:  45.0, description: "铲斗"}

# ③ 连杆几何 —— 只改 L_bucket 0.26→0.50，其余保持默认
link_geometry:
  offset_x: 0.25
  offset_z: 0.40
  L1: 0.35
  L2: 0.60
  boom_bend_angle_deg: 46.0
  L_arm: 0.44
  L_bucket: 0.50          # ★ 加长铲斗 0.26→0.50 m
  sensor_offsets_deg:
    boom:    40.9
    arm:     19.6
    bucket: -56.2

# ④ ROS 协议 —— 话题改 /custom/joint_states（多机并行时区分）
ros_protocol:
  node_name: "custom_60FED_controller"
  joint_topic: "/custom/joint_states"   # ★ 改话题名
  frame_id: "base_link"
  qos_depth: 10
  msg_type: "sensor_msgs/msg/JointState"
  first_publish_sync_from_feedback: true

# ⑤ 标准姿态（保持默认即可，也可改）
standard_poses:
  init:          {swing_yaw: 0.0, boom_swing:  5.0, arm_boom:  60.0, bucket_arm:  10.0}
  home:          {swing_yaw: 0.0, boom_swing:  0.0, arm_boom: 120.0, bucket_arm:  30.0}
  cycle_transit: {swing_yaw: 0.0, boom_swing: 15.0, arm_boom:  70.0, bucket_arm: -20.0}

# ⑥ 运动默认参数（保持默认即可）
motion_defaults:
  at_pose_tolerance_deg: 1.0
  move_timeout_s: 3.0
  bucket_search_range_deg: [-70.0, 10.0]
  bucket_search_samples: 17
```

使用（3 行跑起来）：

```python
from v15_action_task import from_config
ctx = from_config("./my_custom_60FED.yaml", adapter_backend="mock")
with ctx["controller"] as ctl:
    print("机型：", ctx["config"].model_name)          # "custom_60FED_long_bucket"
    print("铲斗长度：", ctx["config"].link.L_bucket)   # 0.50
    print("大臂 max：", ctx["config"].limits.limits["boom_swing"].max_deg)  # 40.0
    print("ROS 话题：", ctx["config"].ros.joint_topic) # "/custom/joint_states"
```

#### 示例 2：等价 JSON 配置（无 PyYAML 环境用）

与 YAML **字段完全一致**，仅语法从 YAML 改为 JSON。保存为同目录同名的 `my_custom_60FED.json`，`load_config("my_custom_60FED.yaml")` 在 PyYAML 缺失时会自动 fallback 读取这个 JSON。

```json
{
  "v15_config_version": "1.0",
  "model_name": "custom_60FED_long_bucket",
  "description": "60FED 加长铲斗版 + 收紧大臂限位 + 自定义 ROS 话题",
  "joint_mapping": {
    "swing_yaw":  "swing_joint",
    "boom_swing": "boom_joint",
    "arm_boom":   "arm_joint",
    "bucket_arm": "bucket_joint"
  },
  "joint_limits": {
    "swing_yaw":  {"min_deg": -180.0, "max_deg": 180.0, "description": "回转"},
    "boom_swing": {"min_deg":   -5.0, "max_deg":  40.0, "description": "大臂（限 40°，防磕履带）"},
    "arm_boom":   {"min_deg":    0.0, "max_deg": 130.0, "description": "小臂"},
    "bucket_arm": {"min_deg":  -95.0, "max_deg":  45.0, "description": "铲斗"}
  },
  "link_geometry": {
    "offset_x": 0.25, "offset_z": 0.40,
    "L1": 0.35, "L2": 0.60, "boom_bend_angle_deg": 46.0,
    "L_arm": 0.44, "L_bucket": 0.50,
    "sensor_offsets_deg": {"boom": 40.9, "arm": 19.6, "bucket": -56.2}
  },
  "ros_protocol": {
    "node_name": "custom_60FED_controller",
    "joint_topic": "/custom/joint_states",
    "frame_id": "base_link",
    "qos_depth": 10,
    "msg_type": "sensor_msgs/msg/JointState",
    "first_publish_sync_from_feedback": true
  },
  "standard_poses": {
    "init":          {"swing_yaw": 0.0, "boom_swing":  5.0, "arm_boom":  60.0, "bucket_arm":  10.0},
    "home":          {"swing_yaw": 0.0, "boom_swing":  0.0, "arm_boom": 120.0, "bucket_arm":  30.0},
    "cycle_transit": {"swing_yaw": 0.0, "boom_swing": 15.0, "arm_boom":  70.0, "bucket_arm": -20.0}
  },
  "motion_defaults": {
    "at_pose_tolerance_deg": 1.0,
    "move_timeout_s": 3.0,
    "bucket_search_range_deg": [-70.0, 10.0],
    "bucket_search_samples": 17
  }
}
```

---

### 4.5 限位裁剪深度集成

#### 设计取舍（为什么把裁剪放进 URDFController？）

| 方案 | 优点 | 缺点 |
|---|---|---|
| **动作库/用户层手动裁剪**（原设计） | 控制协议层纯粹，不管业务逻辑 | 90% 用户会忘记调，导致超限指令发出，硬件安全无保障 |
| **URDFController 层自动裁剪**（v15 from_config 默认） | 安全默认开箱即用，超限指令永远发不出去 | 需要兼容旧 API，不能引入破坏性变更 |

**v15 最终方案**：新增**可选关键字参数** `joint_limits` 和 `clamp`，严格向后兼容：

- `URDFController(adapter)` — 不传 `joint_limits` → **完全旧行为**（不裁剪，返回 None/旧默认）
- `from_config(use_config_limits=True)` — **默认值**，自动把 YAML `joint_limits` 传给 Controller，发布前自动裁剪
- `from_config(use_config_limits=False)` — 临时关闭裁剪（例：调试时想观察真实超限误差）
- `URDFController(adapter, joint_limits=..., clamp=False)` — 传了 limits 但临时关闭裁剪（更细粒度）

#### 物理验证结果（自检 17/18 项 PASS）

默认 60FED 配置 `boom_swing` 限位为 `[-5°, 55°]`：

| 用户下发 `set_pose()` 的 boom_swing | 实际 wire 上发布（裁剪后） | 行为说明 |
|---|---|---|
| `60.0°`（超限 +5°） | `55.0°` | 自动裁剪到上限 |
| `45.0°`（未超限）   | `40.0°`（若配置 max=40°时） | 按自定义配置裁剪 |
| `30.0°`（合法值）   | `30.0°` | 合法值原样通过 |

#### 临时关闭裁剪（调试历史动作脚本时用）

```python
from v15_action_task import from_config

# 方式 1：from_config 入口直接传 False
ctx = from_config(use_config_limits=False)
# 方式 2：分块构造时不传 limits
cfg = from_config()["config"]
ctl_no_clamp = URDFController(adapter)   # 不传 limits = 旧行为不裁剪
```

> ⚠️ **兼容性结论**：所有**不使用 config 层**的旧代码（直接 `URDFController(RosV14Adapter())`）**100% 零改动可用**。新增参数均为可选关键字，默认值 = 旧行为。

### 4.6 default_config.yaml 新增 3 大类字段（sensors / extrinsics / tilt_compensation）

`default_config.yaml` 后段追加 **3 大传感器相关节**（§4.2 已覆盖的 6 大类限位/连杆/协议保持不变），所有字段均与 v0~v14 真实传感器驱动严格对齐：

#### 4.6.1 第 7 类：sensors —— 统一传感器 ROS 消息接口

```yaml
sensors:
  tilt_sensors:                          # 维特 WT901C485 ×4，单话题 4 路拆分
    tilt_bucket: {modbus_addr: "0x50", array_index: 0, semantic_joint: bucket_arm,
                  topic: /excavator/inclinometer_pitch_deg, msg_type: std_msgs/msg/Float64MultiArray, enabled: true}
    tilt_arm:    {modbus_addr: "0x51", array_index: 1, semantic_joint: arm_boom,
                  topic: /excavator/inclinometer_pitch_deg, msg_type: std_msgs/msg/Float64MultiArray, enabled: true}
    tilt_boom:   {modbus_addr: "0x52", array_index: 2, semantic_joint: boom_swing,
                  topic: /excavator/inclinometer_pitch_deg, msg_type: std_msgs/msg/Float64MultiArray, enabled: true}
    tilt_swing:  {modbus_addr: "0x53", array_index: 3, semantic_joint: swing_yaw,
                  topic: /excavator/inclinometer_pitch_deg, msg_type: std_msgs/msg/Float64MultiArray, enabled: true}
  lidars:                                # 最多 5 路：1 矩阵固态 + 2 环视360 + 2 单线
    lidar_single_rear:                   # PaceCat M300-E 单线，出厂 enabled
      {topic: /pointcloud, msg_type: sensor_msgs/msg/PointCloud2, frame_id: lidar_single_rear_link, enabled: true}
    lidar_matrix_front:                  # 矩阵式固态雷达（前向 200m）
      {topic: /lidar_matrix/points, msg_type: sensor_msgs/msg/PointCloud2, enabled: false}
    lidar_360_left / lidar_360_right:    # 2 路环视 360（车舱两侧）
      {enabled: false, ...}
    lidar_single_front: {enabled: false, ...}
  cameras:                               # 6 路 视频流 + camera_info（≥5，可扩展）
    cam_front_main:                      # 海康主相机 - 前方作业区
      {topic: /sensors/camera/front_main/image_raw, info_topic: /sensors/camera/front_main/camera_info,
       msg_type: sensor_msgs/msg/Image, frame_id: cam_front_main_link, enabled: true}
    cam_left / cam_right / cam_rear / bucket_view / arm_overlook:  {enabled: true/false, ...}
```

使用：
```python
from v15_action_task import load_default_config
cfg = load_default_config()
cfg.sensors.list_tilt_ids()    # → ['tilt_bucket','tilt_arm','tilt_boom','tilt_swing']
cfg.sensors.list_lidar_ids()   # → 5 个 lidar_*
cfg.sensors.list_camera_ids()  # → ≥6 个 cam_*
t = cfg.sensors.by_id("tilt_bucket").as_tilt()
print(t.modbus_addr, t.array_index, t.semantic_joint)   # → "0x50" 0 "bucket_arm"
```

#### 4.6.2 第 8 类：extrinsics —— 相对于挖掘机 `base_link` 的外参 6-DOF

> **出厂值 1:1 拷贝 v5 `launch/sensors_tf.launch.py` 中的 `static_transform_publisher` 参数**，格式为标准 `x y z yaw pitch roll`（米 + 弧度），与 TF GUI record.txt 输出 8 字段直接互转。

```yaml
extrinsics:
  - sensor_id: lidar_single_rear      # ★ 出厂倒装值 (AC-8 1e-4 硬验收)
    parent_frame: base_link
    child_frame:  lidar_single_rear_link
    x_m: -0.5500   y_m: -0.2000   z_m:  1.2712
    yaw_rad:   0.0532    pitch_rad: 0.0349    roll_rad: 3.0316
    enabled: true
  - sensor_id: cam_front_main         # 海康主相机 (AC-3 C-3 值)
    x_m: 0.4539 y_m: 0.1532 z_m: 1.5246 pitch_rad: 1.1519  enabled: true
  - sensor_id: cam_left               # 网络相机左 (C-3 值)
    x_m: 0.4239 y_m: -0.1768 z_m: 1.4246 pitch_rad: 0.9250 enabled: true
  # ... 剩余 7 条：0 值 + enabled=false (占位，可按需启用)
```

Python 访问（支持三种纯函数格式变换，零 tf2 硬依赖）：
```python
lidar_ext = cfg.extrinsics.get("lidar_single_rear")
x, y, z, yaw, pitch, roll = lidar_ext.to_xyzrpy()                   # 6-DOF
args = lidar_ext.to_static_transform_publisher_args()               # → ["-0.55","-0.2","1.2712","0.0532","0.0349","3.0316"]
T = lidar_ext.to_4x4_matrix()                                       # 纯数学 SE3 4×4
print(cfg.extrinsics.to_launch_static_tf_nodes_yaml())              # → 直接作为 launch static_transform_publisher CLI 参数
```

#### 4.6.3 第 9 类：tilt_compensation —— 倾角温漂/零偏/相对角补偿

> 对齐 v8 inclinometer_reader.py 零偏校准 + v11 imu_preintegration 互补滤波 + v5 gyro bias 死区 三个项目的真实方案，**作为横切关注点内置进 SensorManager**，业务层零重复。

```yaml
tilt_compensation:
  alpha: 0.98                       # 互补滤波权重 (gyro积分 vs 重力加速度绝对方向)，IMU 标准 0.98
  calib_count: 50                   # 启动静止校准帧数（≈3.0s@16.6Hz），启动后 Reading.calibration_done=True
  gyro_deadzone_rad_s: 0.002        # gyro 死区 (v5 DirectSwingEstimator 经验值)，静止 |gyro| < 死区 → 归零
  auto_calibrate_on_open: true      # SensorManager.open() 立刻开始采集校准
  use_relative_subtraction: true    # 父子链相减：tilt_bucket_rel = bucket - arm  消除机体安装零偏
```

### 4.7 三级兜底加载机制 - 传感器扩展

延续 §4.3 同样的加载链，新的 `sensors / extrinsics / tilt_compensation` 三大节**各自独立**走三级兜底，缺任意一节都不会崩：

```
load_config(path)
  ↓ ① 尝试 YAML: 成功 → 解析 (缺 sensors? → 生成默认 sensors)
  ↓ ② 无 PyYAML: 尝试 sibling *.json: 成功 → 解析
  ↓ ③ 缺 YAML/JSON: 直接用 BUILTIN_DEFAULT_CONFIG_DICT 的三大节 (与 YAML 数值 1:1)
```

缺省的 `build_default_sensors_config() / build_default_extrinsics_config() / build_default_tilt_compensation_config()` 三个构造函数可单独调用，支持 `V15Config.from_dict({})` 完全空字典也能跑出完整 9 大节。

### 4.8 配置层与标定闭环（AC-10 三步校验）

标定链路 (`§3.9 ③ Step A/B`) 和配置加载**形成闭环**，写入值和读回值误差 ≤1e-4：

```
 Step 1:  SSH 无头现场 → ex11 命令行标定 → 输出 /tmp/record.txt
          格式:  -0.549  -0.199  1.2722  3.0326  0.0542  0.0359  base_link  lidar_single_rear_link
           (8 空格分隔 tokens，与 v5 GUI 完全同格式)

 Step 2:  ex12 record → extrinsics JSON
          python3 ex12_sensor_tf_record_to_yaml.py \
              --input /tmp/record.txt --out /tmp/extrinsics_TEST01.json

 Step 3:  业务代码 load_config("/tmp/extrinsics_TEST01.json")
          cfg = load_config("/tmp/extrinsics_TEST01.json")
          lidar_ext = cfg.extrinsics.get("lidar_single_rear")
          assert abs(lidar_ext.roll_rad - 3.0359) < 1e-4  ✓ (AC-10)
```

---

## 5. 快速使用示例

> 无 ROS 环境下直接跑通整条链路（IK→控制器→末端回推）：

```python
from v15_action_task import (
    URDFController, MockAdapter, InverseKinematics, CartesianMover,
)

ik = InverseKinematics()
with URDFController(MockAdapter()) as ctl:
    mover = CartesianMover(ctl, ik)
    for name, (x, y, z, ba) in {
        "下挖":   (1.0,  0.0, -0.20, -60.0),
        "提斗":   (0.9,  0.0,  0.10, -20.0),
        "卸料":   (0.95, +0.5,  0.00, +10.0),
    }.items():
        r = mover.move_with_bucket(x, y, z, ba)
        tx, ty, tz = r.target_xyz
        fx, fy, fz = r.final_tip_xyz
        err_mm = ((fx-tx)**2+(fy-ty)**2+(fz-tz)**2)**0.5 * 1000
        print(f"[{name:>3}] 目标({tx:+.2f},{ty:+.2f},{tz:+.2f})m → 末端误差 {err_mm:.4f}mm ({'✓' if r else '✗'})")
```

输出（Mock 后端零延迟同步误差 0.0000mm）：

```
[ 下挖] 目标(+1.00,+0.00,-0.20)m → 末端误差 0.0000mm (✓)
[ 提斗] 目标(+0.90,+0.00,+0.10)m → 末端误差 0.0000mm (✓)
[ 卸料] 目标(+0.95,+0.50,+0.00)m → 末端误差 0.0000mm (✓)
```

---

## 6. 在 RViz2 中实际跑通 (SSH 无头模式)

### 被控电脑（有显示器）

```bash
# 只做一次：编译 workspace
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
colcon build --symlink-install --packages-select shandong
source install/setup.bash

# 启动 v14 URDF（headless=true，不启 GUI 节点，SSH 下无报错）
ros2 launch shandong_v14_urdf display.launch.py \
    headless:=true use_joint_state_publisher:=false

# 被控电脑本地开 RViz2：
#   Add → RobotModel
#   配置 RobotModel.Description Topic = /robot_description
#   Fixed Frame = base_link
#   Global Options.Fixed Frame 必须是 base_link（否则 model 会 "frame not exist"）
rviz2
```

### SSH 终端（运行 v15 控制脚本）

```bash
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
source install/setup.bash
cd src/shandong/v15_action_task

# 运行 6 段挖掘 Demo
python3 drive_v14_in_rviz.py
```

RViz 中应依次观察到：`INIT → 下挖 → 下挖更深 → 提斗 → 左转 30° → 张斗卸料 → 回零`。

> 💡 若 SSH 端无 ROS：脚本自动 fallback 到 MockAdapter，仅打印日志但不崩溃。

---

## 7. 与 v10 的数学差异说明（必知）

在移植 `v10_cailbration_arm/inverse_kinematics.py` 过程中，发现 v10 IK 中存在一个 **bucket 角度映射符号 bug**：

```
FK 正向公式（v10 和 v15 一致，正确）：
    abs_bucket = offset_bucket - sensor_bucket    (offset_bucket = -56.2°)
    → 反映射应为：sensor_bucket = offset_bucket - abs_bucket

v10 IK 的错误写法：
    sensor_bucket = theta3 + 56.2    (错误！符号反了)

v15 修正后的正确写法：
    sensor_bucket = offset_bucket - theta3    (与 FK 对称，正确)
```

**v10 后果**：用 FK 算出的铲斗绝对角直接喂回 v10 IK，FK→IK→FK 空间误差高达 **90~520 mm**，铲斗角偏差 20°~120°，**末端位姿根本无法精确达到**。v15 修正后：

- 12 组指定姿态 FK→IK→FK 闭环误差 **0.0000 mm**
- 100 组随机姿态最大误差 **0.00000 mm**，无解数 **0**

> ⚠ **兼容性提醒**：若你有历史数据/脚本基于 v10 bug 生成的 bucket 关节角，迁移到 v15 后需要重新求解 IK（因为我们以"数学自洽、末端可精确到达"为优先）。

---

## 8. 可达域 x/y/z 可执行区间说明（默认 60FED 机型）

本节给出 **默认 60FED 机型**（即 `config/default_config.yaml` 中 5 层可达域过滤所基于的几何参数）下，
**`/v15/cmd/target_point`（接口1）在 `base_link` 坐标系下可执行的 (x, y, z) 数值范围说明**，
供脚本/遥控器/UI 界面做「输入合法性预校验」直接使用。

**数据来源（代码锚点，若你改配置后务必同步改本节数值）：**
- 几何硬参数 → [default_config.yaml link_geometry](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/config/default_config.yaml#L58-L74)
- 5 层可达域过滤实现 → [WorkspaceChecker.check_point_reachable](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/motion/workspace.py#L187-L283)
- z 轴 dig / transit 分界线 → [default_config.yaml workspace](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/config/default_config.yaml#L431-L443)

### 8.1 几何基准（先理解，再看数值）

**坐标系约定：**
- 参考系：`base_link`，原点 = 回转中心（大臂销轴在 `x=+0.25m, z=+0.40m`），地面基准 = `z = 0.0`
- (x, y) 为水平面：机器人朝前 `+x`，朝左 `+y`（标准右手笛卡儿；`swing_yaw=0°` 时末端指向 `+x`）
- 径向距离（水平面离回转中心的距离）：`r = √(x² + y²)`，单位：m
- 回转角（从上往下看顺时针为正）：`θ = atan2(y, x)`，单位：° / rad 均可
- **可达域判定的是铲尖（bucket tip）坐标，不是腕点（bucket pivot）**，但由腕点三角不等式 `||L_boom ± L_arm||` 推导后再投影出铲尖

**大臂等效臂展（由 L1/L2/bend 余弦定理自动派生）：**

| 派生量 | 数值 | 说明 |
| --- | ---: | --- |
| `L_boom` 大臂等效直线长 | **0.8799 m** | boom pivot → boom tip |
| `beta_deg` 大臂等效偏置角 | 16.63° | 与 L2（大臂第二段）的结构夹角 |
| `L_arm` 小臂长 | 0.4400 m | boom tip → bucket pivot（腕点） |
| `L_bucket` 铲斗长 | 0.2600 m | bucket pivot → bucket tip（铲尖） |
| `offset_x / offset_z` | 0.25 / 0.40 m | 回转中心 → 大臂销轴 |

### 8.2 5 层可达域过滤器（与代码顺序一致）

WorkspaceChecker 在接受任意 (x, y, z) 前会依次判定以下 5 层，**每层不通过都直接 FAIL 并把所有原因写进 `ReachabilityReport.reasons`**：

| 层级 | 约束 | 数值 / 区间 | 生效模式 |
| --- | --- | --- | --- |
| **Layer 1 / 5** | 回转 swing_yaw 限位 | θ ∈ [−180.0°, +180.0°] | dig / transit 都生效 |
| **Layer 2 / 5** | 地面穿透防护 ground_penetration | z ≥ **−0.10 m**（下挖 10 cm；如需下挖 1 m 改成 −1.00 m，注释在 yaml 里已写好） | 仅在 **非 dig 模式**生效；**dig 模式本层关闭**（允许铲尖扎进土里） |
| **Layer 3 / 5** | 转运防刮地 transit_min_z | z ≥ **+0.30 m**（转运时铲尖 ≥ 离地 30 cm） | **仅 transit 模式**生效；dig 模式不启用 |
| **Layer 4 / 5** | 腕点三角不等式（几何硬极限） | 腕点半径 `d_wrist` ∈ [`inner_radius_m=0.4399 m`, `outer_radius_m=1.3199 m`] | dig / transit 都生效 |
| **Layer 5 / 5** | 奇异位姿安全 margin（避免 `d_wrist` 贴紧 inner/outer 边界导致 IK 无解/抖动） | margin = `max(0.02 m, 1 % × 1.3199 m)` = **0.0200 m**；<br>工作点必须 `d_wrist ∈ [inner+2m, outer−2m]` 附近（即安全半径带 ≈ 0.48 m ~ 1.28 m，低于此带会被 `shrink_to_closest` 自动回拉到最近可达点） | dig / transit 都生效 |

> 上面 Layer 4/5 的 `d_wrist`（腕点半径）是大臂销轴平面（已把 swing_yaw 投影掉）里的腕点到销轴距离，**与你发的 (x, y, z) 关系是**：
> 令 `r_h = √((x − offset_x)² + y²)`, `z_h = z − offset_z`（都平移到以销轴为原点），再从铲尖反推腕点（bucket 方向近似）后与 `||L_boom ± L_arm||` 比较；
> 在「极限伸平」「极限收拢」两种姿态，铲尖的水平面投影半径会比腕点再外扩 / 内收约 `L_bucket ≈ 0.26 m`，因此给出下面 8.3 的**笛卡儿近似区间**，用起来最直接。

### 8.3 可直接使用的笛卡儿 (x, y, z) 数值区间（默认 60FED）

下面表格是**对 `/v15/cmd/target_point` 接口最友好的「直接填数字」区间**：
你写脚本/遥控器/界面校验时，只要 (x, y, z) 落在**绿色行**的范围内，就一定能过 5 层过滤 + IK 有解，不会被 `shrink_to_closest` 自动回拉。

| 维度 | 极限几何区间（理论可达） | **✅ 推荐安全执行区间（不会被自动回拉）** | 备注 |
| --- | --- | --- | --- |
| **r = √(x² + y²)**（水平面回转半径） | [0.12 m, 1.60 m]（含铲斗 L_bucket 投影极限） | **[0.40 m, 1.50 m]** | 对应腕点安全半径 0.48~1.28 m + 铲斗投影 |
| **x**（前后，朝前为正） | [−1.60 m, +1.60 m] 同时受 `r` 上限约束 | **[−1.50 m, +1.50 m]**，且 `r ≤ 1.50` | 单 x=1.50 m, y=0 m, z=0.60 m = 正前方最远 |
| **y**（左右，朝左为正） | [−1.60 m, +1.60 m] 同时受 `r` 上限约束 | **[−1.50 m, +1.50 m]**，且 `r ≤ 1.50` | 单 y=1.50 m, x=0 m, z=0.60 m = 正左侧最远 |
| **z (dig 模式)**（下挖 / 平地 / 抬升） | [−0.10 m, +1.68 m]（理论） | **[−0.05 m, +1.65 m]**，同时要求 `r ∈ [0.40, 1.50]` | 默认 `min_ground_z = −0.10`；若改成 −1.00 m 则 z 下限同步改成 −0.95 m |
| **z (transit 模式)**（转运姿态禁止刮地） | [+0.30 m, +1.68 m]（理论） | **[+0.35 m, +1.65 m]**，同时要求 `r ∈ [0.40, 1.50]` | `min_transit_z = +0.30`；转运动作库（接口2）自动走 transit 模式 |

### 8.4 四个典型「极限点」示例（复制即可真机发指令验证）

下面 4 个点都落在「✅ 推荐安全执行区间」里，**直接复制到 `ros2 topic pub -1 /v15/cmd/target_point ...` 就可以当验收用例**：

| 用例名 | (x, y, z) m | dig / transit | 说明 | 预期 |
| --- | --- | --- | --- | --- |
| ① 正前方最远（平伸抓放） | `(+1.45, 0.00, 0.60)` | dig / transit 都可 | r=1.45 m < 1.50；z=0.60 > transit 0.30 双通道都过 | 成功，轨迹 lerp ~13 步，FK 误差 ≤ 3 cm |
| ② 正左方侧面（侧卸料常用） | `(0.00, +1.45, 0.80)` | transit 模式推荐 | r=1.45 < 1.50；y=+1.45 对应 swing_yaw ≈ +90° | 成功，回转先到 90°，然后抬臂+伸小臂 |
| ③ 下挖最深（默认配置） | `(+0.90, 0.00, −0.08)` | **dig 模式必须** | z=−0.08 > −0.10；若 transit 模式会在 Layer 3 FAIL（z < 0.30）→ 你需要把 mode 改成 dig（接口1 默认 mode=dig，直接发就行） | 成功，进入下挖姿态；若看到 `transit_min_z_violation` 说明你显式传了 mode=transit |
| ④ 最高举升（卸料常用） | `(+0.90, 0.00, +1.60)` | transit 模式推荐 | z=+1.60 < +1.65；r=0.90 落在安全带中间，无奇异 | 成功，大臂抬到接近 boom_swing=−5° 上限 |

### 8.5 两种常见 FAIL 原因与排查（直接对应 ReachabilityReport.reasons 关键字）

当 `/v15/cmd/target_point` 返回 `success=False` 时，`reachability_report.reasons` 会命中下面 1 条或多条，
与本节能级 1:1 对应：

| reasons 关键字（按出现概率排序） | 对应层级 | 立刻能做的修复 |
| --- | --- | --- |
| `outer_radius_exceeded` | Layer 4 | x/y 的 r 太大，把目标往回拉（例如 1.60 → 1.45 m）|
| `inner_radius_violation` | Layer 4 | r 太小（贴近回转中心），把目标往外推（例如 0.20 → 0.50 m）|
| `near_singularity` | Layer 5 | 点已经在 inner/outer 边界上；手动把 r 往安全带中间 ±3 cm 挪一下 |
| `transit_min_z_violation` | Layer 3 | 你发了 mode=transit 但 z < 0.30 m；要么 z 提到 0.35+，要么改成 mode=dig |
| `ground_penetration` | Layer 2 | z < −0.10 m（默认）；要么提高 z，要么改 `min_ground_z` 到 −1.00 |
| `swing_yaw_out_of_range` | Layer 1 | 正常点不会触发；如果确实需要超过 ±180°，请改成等价的 `θ ± 360°` |

---

## 9. 验证结果

> **总自检覆盖**：43 项 / 8 大维度（配置加载 / 物理几何 / 代数自洽 / 限位裁剪 / 旧 API 兼容 / 语法 / 端到端 mover / **Phase3 可达域+轨迹+三接口**）— **43/43 全部 PASS**
> 分类：Phase0-2 回归 **33/33** PASS + Phase3 AC-1~AC-10 新增 **10/10** PASS

### 9.1 基础层（运动学 + 协议 + 动作库）

| 检查项 | 结果 |
|---|---|
| FK→IK→FK 12 组指定姿态闭环 | **0.0000 mm** 空间误差，无解 0 |
| FK→IK→FK 100 组随机蒙特卡洛 | **0.00000 mm** 最大误差，无解 0 |
| 任意目标点 IK → FK 回推（4 组） | <0.0005 mm / Δbucket_angle <0.0001° |
| search_bucket_angle（自搜索）| 目标距离 < 1 mm |
| 端到端运动链路（Mock）3 段连贯动作 | 全段 0.0000 mm 同步误差 |
| 动作库 build_single_dig_dump_task | 正常生成 21 步 JSON 剧本 |
| 零外部依赖检查（grep 全源码） | ✅ 无 import / sys.path 指向 v10 / v14 Python 代码 |
| py_compile 33 个 .py 文件（含 config/） | 0 语法错误 |
| RosV14Adapter 对象创建（不启动节点） | ✅ OK |
| RosV14Adapter.from_config() 构造 | ✅ YAML→Adapter 参数 1:1 对齐 |
| 顶层 32+ 符号统一 import（含 config 4 个新符号） | ✅ 全部通过 |

### 9.2 配置层（新增 24 项验证）

#### ① 配置加载与三级兜底机制（7/7 PASS）

| 检查项 | 结果 |
|---|---|
| `load_default_config()` 单例缓存两次调用 | ✅ 返回同一 V15Config 对象 |
| `load_config()` 支持 V15Config 输入（原样返回） | ✅ PASS |
| `load_config()` 支持 dict 输入（raw dict 解析） | ✅ PASS |
| `load_config()` 支持文件路径 str 输入 | ✅ PASS（JSON 路径也 OK） |
| `BUILTIN_DEFAULT_CONFIG_DICT` 数值与 YAML 1:1 对齐 | ✅ 6 大类 30+ 字段逐项相等 |
| 三级兜底 ③：完全无文件无 PyYAML 仅内置 dict | ✅ `from_config(dict)` 正常构造 6 件套 |
| 三级兜底 ②：无 PyYAML 环境同名 JSON fallback | ✅ 自动读 .json 不报 ImportError |

#### ② 物理几何参数生效验证（2/2 PASS，硬核方法避免假阳性）

| 检查项 | 结果 |
|---|---|
| **L_bucket 物理增量验证**：改 YAML 中 L_bucket 0.26→0.50 m，FK 算 arm_tip→bucket_tip 距离 | **+240.00 mm**（与理论 0.24×1000 误差 <0.01mm，证明配置真正进计算链路） |
| **连杆参数全字段生效验证**：自定义 L_arm=0.50 / offset_x=0.30 / offset_z=0.50，FK 关键点对比硬编码改值 | ✅ 所有关键点坐标与硬编码改值结果逐项一致 |

#### ③ 自定义机型代数闭环自洽（1/1 PASS）

| 检查项 | 结果 |
|---|---|
| **自定义机型 FK→IK→FK 代数闭环**（L_bucket=0.50 m，10 组随机姿态） | **10/10 闭环成功，最大空间误差 0.00000 mm**（证明改连杆不破坏核心几何自洽） |

#### ④ 限位裁剪深度集成（4/4 PASS，物理验证）

| 检查项 | 结果 |
|---|---|
| boom_swing=60° 超限（默认 max=55°） | 自动裁剪到 **55.0°** ✅ |
| boom_swing=45° （自定义 max=40° 时） | 自动裁剪到 **40.0°** ✅ |
| 合法值 boom_swing=30° | 原样通过 **30.0°**（无改动）✅ |
| `use_config_limits=False` 临时关闭裁剪 | 超限 60° 原样发出不裁剪（兼容旧行为）✅ |

#### ⑤ 向后兼容性（5/5 PASS，不破坏既有代码）

| 检查项 | 结果 |
|---|---|
| 直接 `URDFController(adapter)` 不传 limits → 旧行为不裁剪 | ✅ OK |
| 直接 `CartesianMover(ctl, ik)` 不传 fk/motion 参数 → 旧默认值 | ✅ OK，容差 1° / 超时 3s / 搜索 17 个候选 |
| 旧 API 3 段连贯动作（完全不 import config） | 末端误差 **0.0000 mm** ✅ |
| `DEFAULT_PARAMS`（LinkParams 硬编码）数值未改动 | ✅ L_bucket 仍 =0.26 m，beta=16.626° |
| `SEMANTIC_TO_URDF` 字典（硬编码映射）未改动 | ✅ 4 对映射完全与初版一致 |

#### ⑥ from_config 一键构建 + 端到端 mover（5/5 PASS）

| 检查项 | 结果 |
|---|---|
| `from_config()` 默认调用 → 6 keys 齐全 | ✅ {config, controller, adapter, fk, ik, mover} 均非 None |
| `from_config(adapter_backend="ros")` → RosV14Adapter 构造 | ✅ 对象创建 OK（不启节点） |
| `from_config(dict_obj)` → 自定义 dict 当配置源 | ✅ L_bucket 0.50m 进入 FK 计算链路 |
| `from_config(use_config_limits=False)` → Controller 不裁剪 | ✅ 超限 60° 不裁剪通过 |
| **端到端 move_with_bucket x 3（from_config 构建的 mover）** | 下挖/提斗/卸料 3 段动作末端误差均为 **0.000 mm**，success=True ✅ |

---

### 9.3 新增验证：指令库 + 传感器 + 标定闭环 + examples（15/15 PASS）

> 对应本阶段 4 条规格的 10 条 AC（AC-1~AC-10）专项验证，通过方式：`cd v15_action_task/examples/ && python3 _run_all_checks.py` → **PASS 33/33**。

#### ① AC-8 / AC-9 / AC-10 三条硬验收（v0~v14 考古对齐基准）（3/3 PASS）

| AC 编号 | 验证内容 | 期望 | 实测 |
|---|---|---|---|
| **AC-8 (外参出厂值 rule)** | `load_default_config().extrinsics.get("lidar_single_rear")` 6 参数 vs `sensors_tf.launch.py` 真实倒装值 (x=-0.5500, y=-0.2000, z=1.2712, yaw=0.0532, pitch=0.0349, roll=3.0316) | 6 项 abs diff < 1e-4 | ✅ **全部 =0.0000**，位级相等 |
| **AC-9 (倾角地址映射 rule)** | `tilt_sensors` 4 条 id ↔ modbus_addr ↔ array_index ↔ semantic_joint 一一对应：bucket↔0x50↔0↔bucket_arm / arm↔0x51↔1↔arm_boom / boom↔0x52↔2↔boom_swing / swing↔0x53↔3↔swing_yaw | 16 字段全 match | ✅ **16/16 全匹配** |
| **AC-10 (标定闭环 rule)** | 三步闭环：①手写 record_test.txt (8 tokens含 roll=3.0316) → ②`ex12` 转 JSON → ③`load_config(json)` 断言 roll_rad 误差 | <1e-4 | ✅ **diff =0.0000** exit 0 |

#### ② 动作库 8 指令 / 方向语义 (AC-7) + 限位裁剪（4/4 PASS）

| 检查项 (AC #) | 期望 | 实测 |
|---|---|---|
| **AC-7 (方向语义自洽)** | 4 关节 × 2 布尔方向共 8 次调用：`swing=True→+; boom=True→-; arm=True→+; bucket=True→+` 各 ±5° | 8/8 方向与期望一致 | ✅ **8/8 PASS** (`ex02` 自断言脚本) |
| 限位夹紧深度：`boom_move(up=False, angle=999°)` 与 `bucket_move(True, 999)` | 分别裁剪到 55.0° 与 45.0°（Config 层 clamp + URDFController 层 clamp 双层防护） | | ✅ **55.00° / 45.00° 精确** |
| `MoveResult.__bool__()` 简写：`if boom_move(...): print("到位")` | `success=True → bool=True` | | ✅ 支持简写法 |
| 扩展函数 `swing_move_to / bucket_tilt_to` 调用 | 不 crash，MoveResult.success == True | | ✅ OK |

#### ③ 传感器订阅接口 / TiltCompensator / grep 旧版本 (AC-6)（5/5 PASS）

| 检查项 (AC #) | 期望 | 实测 |
|---|---|---|
| **AC-6 (零依赖 v0~v14)** | 全库两次 grep：(a) `from v0_..v14_ / from shandong.v[0-14]` (b) `import v0_..v14_`，均 0 条匹配 | | ✅ **双 grep 均 0 行**，干净独立 |
| TiltCompensator 纯算术校准：60 次 `update(15.0)` → calib_count=50 | ①校准完成 ②`|compensated| < 0.01°` (因 15.0−15.0=0) | calibration_done=True & \|compensated\|=0.000000° | ✅ PASS |
| use_relative_subtraction 父子链 4 路注入 `40/30/20/10` | bucket_rel=10 arm_rel=10 boom_rel=10 (swing 无父 None) | | ✅ **10/10/10** 精确 |
| Mock 后端 (force_mock) 无 rclpy 进程环境 | ① 4 tilt_ids ② 5 lidar_ids ③ ≥6 camera_ids ④不 crash | 4 / 5 / 6 | ✅ TR3.5 OK |
| SensorManager `from_config() open() close() with` 生命周期 + `get_tilt/all_tilt/list_*_ids()` 14 个方法 | 全部可调用且不抛异常 | | ✅ 14/14 可调用 |

#### ④ examples 目录结构 + 运行 (AC-4)（3/3 PASS）

| 检查项 (AC #) | 期望 | 实测 |
|---|---|---|
| **AC-4 (examples 独立目录 rule)** | Glob `v15_action_task/**/exNN_*.py` 排除 examples/，匹配数量 =0（正式代码零 `ex_*.py`） | | ✅ **0 条匹配**，干净 |
| examples 目录下 ex01-12 12 个脚本 | 数量 ≥12 且全部 py_compile 通过 | 12/12 compile OK | ✅ 12/12 存在且编译 |
| `python3 ex10_physical_240mm_delta_L.py` | FK tip ΔL ∈ [235, 245] mm 区间 | **ΔL = 240.00 mm**，完美命中 | ✅ PASS |

### 9.4 最终验证小结（两阶段合并）

- ✅ **7 FR 全部实现**：FR-1 三层限位 / FR-2 8 指令库 / FR-3 15 路传感器对齐真实硬件 / FR-4 examples 独立目录 12 脚本 / FR-5 外参 6 参数出厂值闭环 / FR-6 倾角温漂补偿 4 子能力 / FR-7 独立预标定工具 2 脚本
- ✅ **10 AC 全过**：AC-1 限位 / AC-2 数据类型 / AC-3 路径覆盖 / AC-4 目录独立 / AC-5 旧限位不动 / AC-6 零 v0~v14 / AC-7 方向语义 / AC-8 外参出厂值 / AC-9 地址映射 / AC-10 标定闭环
- ✅ **回归 33/33 PASS**：`cd examples/ && python3 _run_all_checks.py` → `=== TOTAL PASS 33/33 ===`
- ✅ **旧 API 零 break**：`from_config()` 默认 `init_sensors=False → sensor_manager=None`，所有阶段 0 已写的代码 100% 兼容
- ✅ **Ubuntu 默认 Python 零依赖可用**：无 PyYAML (YAML→JSON→BUILTIN dict 三级兜底) / 无 rclpy (动态 import 自动回退 Mock 后端) — 两种缺失均 exit 0

---

## 10. 扩展：接真实硬件

v15 的 **Adapter 模式**让硬件接入只需要加一个文件：

```
# 在 v15_action_task/control_core/ 下新建 hardware_serial_adapter.py
from .adapter_base import ControlAdapter

class HardwareSerialAdapter(ControlAdapter):
    def open(self):                  # 打开串口 / CAN
        ...
    def close(self):                 # 关闭
        ...
    def publish_pose_deg(self, pose_deg, frame_id="base_link"):
        # → 把 {swing_yaw, boom_swing, arm_boom, bucket_arm} (度)
        #   转换成协议帧下发到硬件 (485/CAN/以太网)
        ...
    def get_current_pose_deg(self):  # 非阻塞，读反馈 (度)
        ...
    def get_last_update_ts(self):
        ...
```

上层所有代码（URDFController、CartesianMover、action_library 剧本回放）**一行不用改**，只需：

```python
with URDFController(HardwareSerialAdapter(port="/dev/ttyUSB0")) as ctl:
    move_to_cartesian(ctl, ik, 1.0, 0.0, -0.2, bucket_angle_deg=-60)
```

---

## 11. 标准控制协议（与 v14 URDF 完全对齐，不可更改）

| 项目 | 值 |
|---|---|
| 话题 | `/joint_states` |
| 消息类型 | `sensor_msgs/msg/JointState` |
| QoS | **10**（KeepLast，与 ros_joint_bridge.py 一致） |
| `header.frame_id` | **`base_link`**（**不能为空**，否则 RViz 丢弃消息） |
| `name[]` 顺序 | `["swing_joint", "boom_joint", "arm_joint", "bucket_joint"]` |
| 语义名（代码里用） | `swing_yaw` → `swing_joint`；`boom_swing` → `boom_joint`；`arm_boom` → `arm_joint`；`bucket_arm` → `bucket_joint` |
| 单位（外部 API） | **度（deg）**，语义名输入 |
| 单位（话题 wire 上） | **弧度（rad）**，由 Adapter 自动转换 |
| 首次发布逻辑 | 若已收到反馈，则将当前 cmd_deg 同步到反馈里的非指定关节，保证未涉及的关节保持不动 |

---

_最后更新：2026-09-02 · v15 标准库 · 配置层 1.0（6 大类 YAML 参数化 + 三级兜底 + from_config 一键构建 + 35/35 自检全过）_
