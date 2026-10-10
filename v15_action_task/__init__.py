"""
v15_action_task —— 通用挖掘机独立控制库 (v15 Standard Library)
=============================================================

**本包设计目标：完全自洽，可独立迁移到任何 Python 项目。**
  - ✅ 不再依赖 shandong/v10_cailbration_arm 或 shandong/v14_urdf 下的任何 Python 代码
  - ✅ 连杆参数 / FK / IK 全部本地化 (kinematics/)
  - ✅ 控制接口与 v14 URDF 协议完全一致 (control_core/)
  - ✅ 高层末端位姿运动 (motion/)
  - ✅ 4 层动作库原语 (action_library/)
  - ✅ 无 ROS 环境可用 MockAdapter 调试
  - ✅ 有 ROS 环境可直接驱动 v14 标定版 URDF 在 RViz2 中运动

=====================
标准统一接口 (其他项目全部 `from v15_action_task import X`)
=====================

1) 关节级 + Adapter 模式 (control_core)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
>>> from v15_action_task import URDFController, RosV14Adapter, MockAdapter
>>> with URDFController(RosV14Adapter()) as ctl:
...     ctl.set_pose({"swing_yaw": 15.0, "boom_swing": 30.0})  # 发布 /joint_states
...     pose = ctl.get_pose_blocking(timeout_s=2.0)              # 阻塞等待反馈
...     if ctl.is_at_pose(target, tolerance_deg=1.0): pass

2) 运动学 (kinematics)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
>>> from v15_action_task import ForwardKinematics, InverseKinematics, LinkParams, DEFAULT_PARAMS
>>> fk, ik = ForwardKinematics(), InverseKinematics()
>>> fk_sol = fk.solve(20, 40, -70, swing_yaw_deg=10)
>>> (x,y,z) = fk_sol.bucket_tip_3d                         # 正向 → 铲尖 3D
>>> ik_sol = ik.solve_bucket_pose(1.0, 0.0, -0.2, -60.0)
>>> pose_cmd = ik_sol.as_pose()                            # 逆向 → V4 语义相对角
>>> ik_sol = ik.search_bucket_angle(0.9, 0.1, 0.0)         # 未知铲斗角时自动扫描

3) 末端笛卡尔空间 (motion)  —— 一行命令让 RViz 里的挖掘机自己到位
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
>>> from v15_action_task import CartesianMover, move_to_cartesian, MoveResult
>>> mover = CartesianMover(ctl, ik)                        # ctl 来自上面的 URDFController
>>> ok, pose, tip = mover.move(1.0, 0.2, -0.3)             # 自动搜索铲斗角并到位
>>> ok, pose, tip = mover.move_with_bucket(1.0, 0.0, -0.2, -60.0)

4) 动作库 (action_library) 可选子包
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
>>> from v15_action_task.action_library import StepBuilder, build_single_dig_dump_task
>>> steps = build_single_dig_dump_task(dig_x=1.0, dig_z=-0.25, dump_yaw_deg=60.0)
>>> for s in steps: ctl.set_pose(s["target_deg"]); ...  # 按步执行挖掘-回转-卸料剧本

=====================
标准控制协议（与 v14_urdf 完全对齐，不可更改！）
=====================
  话题:     /joint_states (sensor_msgs/JointState, QoS=10, frame_id="base_link")
  关节顺序: ["swing_joint",   "boom_joint",    "arm_joint",    "bucket_joint"]
  语义名:   ["swing_yaw",     "boom_swing",    "arm_boom",     "bucket_arm"]
  单位:     语义 API 用 度 (deg)；  话题 wire 上自动转 弧度 (rad)
"""

# ─── 1) control_core 导出 ─────────────────────────────────────────
from .control_core import (
    URDFController,
    ControlAdapter,
    MockAdapter,
    RosV14Adapter,
    # type 工具
    SEMANTIC_JOINT_ORDER,
    SEMANTIC_TO_URDF,
    URDF_JOINT_ORDER,
    URDF_TO_SEMANTIC,
    DEFAULT_FRAME_ID,
    DEFAULT_JOINT_TOPIC,
    deg_to_rad,
    rad_to_deg,
    default_pose_deg,
)

# ─── 2) kinematics 导出 ─────────────────────────────────────────
from .kinematics import (
    LinkParams,
    DEFAULT_PARAMS,
    get_default_params,
    FKSolution,
    ForwardKinematics,
    IKSolution,
    InverseKinematics,
)

# ─── 3) motion 导出 ──────────────────────────────────────────────
from .motion import (
    CartesianMover,
    MoveResult,
    move_to_cartesian,
    WorkspaceChecker,
    ReachabilityReport,
    TrajectoryPlanner,
    JointTrajectoryPoint,
    IKPlanningResult,
)

# ─── 4) config 导出（v15 YAML 配置层，扩展到 5 子 Config + 1 总 Config）──
from .config import (
    V15Config,
    load_config,
    load_default_config,
    SingleSensorConfig,
    SensorsConfig,
    ExtrinsicsEntry,
    ExtrinsicsConfig,
    TiltCompensationConfig,
)

# ─── 5) sensors 导出（FR-3 传感器接口 + FR-6 倾角温漂补偿）───────
from .sensors import (
    SensorManager,
    TiltReading,
    LidarReading,
    CameraReading,
    TiltCompensator,
)

# ─── 6) action_library.semantic_moves 8 指令库（FR-2 用户指定接口）──
from .action_library import (
    swing_move,
    boom_move,
    arm_move,
    bucket_move,
    swing_move_to,
    boom_lift,
    arm_extend,
    bucket_tilt_to,
)

# ─── 7) action_library.composites 接口2 + 接口3（卸料转运 + 卸料过程）──
from .action_library.composites import (
    CompositeActionResult,
    move_to_dump_transit,
    dump_material,
)

from typing import Any as _Any, Dict as _Dict, Tuple as _Tuple, Optional as _Optional, Literal as _Literal


def from_config(
    cfg: _Any = None,
    *,
    adapter_backend: str = "mock",
    start_adapter: bool = False,
    use_config_limits: bool = True,
    init_sensors: bool = False,
    sensor_ros_mode: _Literal["auto", "force_ros", "force_mock"] = "auto",
    init_workspace: bool = False,
    init_trajectory_planner: bool = False,
) -> _Dict[str, _Any]:
    """
    【标准统一入口】从 YAML 配置一步构建完整控制工具链。

    用法（推荐所有新程序这样写）：
    ```
    from v15_action_task import from_config

    # ① 默认配置（使用包内 default_config.yaml / MockAdapter）
    ctx = from_config(adapter_backend="mock")
    with ctx["controller"] as ctl:
        ok = ctx["mover"].move_with_bucket(1.0, 0.0, -0.2, -60.0)

    # ② 用户自定义 YAML + ROS 后端 + 同步打开传感器
    ctx = from_config(
        "/path/to/my_excavator.yaml",
        adapter_backend="ros", start_adapter=True,
        init_sensors=True, sensor_ros_mode="auto",
    )
    with ctx["controller"] as ctl, ctx["sensor_manager"] as smgr:
        ctx["mover"].move(0.9, 0.5, 0.0)
        for tid in smgr.list_tilt_ids():
            r = smgr.get_tilt(tid)
    ```

    Args:
        cfg:               配置来源：None=默认配置, str=YAML 文件路径, V15Config=已加载对象, dict=raw_dict
        adapter_backend:   "mock" | "ros"
        start_adapter:     True=立刻 adapter.open()（for ros backend）
        use_config_limits: True=用 YAML 里的关节限位覆盖 URDFController 默认限位
        init_sensors:      True=构建并调用 sensor_manager.open()（可 with 进入上下文）；False=不构建（旧 API 零 break）
        sensor_ros_mode:   "auto"=自动检测 rclpy / "force_ros"=强制 ros / "force_mock"=强制 mock
        init_workspace:    True=构建 WorkspaceChecker（可达域 5 层检查）；False=不构建（旧 API 零 break）
        init_trajectory_planner: True=构建 TrajectoryPlanner（LERP/trapezoidal 双策略）；False=不构建

    Returns:
        dict{
          "config":             V15Config 对象,
          "controller":         URDFController (未 enter 上下文),
          "adapter":            MockAdapter 或 RosV14Adapter,
          "fk":                 ForwardKinematics,
          "ik":                 InverseKinematics,
          "mover":              CartesianMover,
          "sensor_manager":     SensorManager 或 None（init_sensors=False 时返回 None，旧代码零 break）,
          "workspace_checker":  WorkspaceChecker 或 None（init_workspace=False 时返回 None）,
          "trajectory_planner": TrajectoryPlanner 或 None（init_trajectory_planner=False 时返回 None）,
        }
    """
    import os as _os

    # 1) 解析 cfg 参数
    if cfg is None:
        v15cfg = load_default_config()
    elif isinstance(cfg, str):
        v15cfg = load_config(cfg)
    elif isinstance(cfg, dict):
        v15cfg = V15Config.from_dict(cfg)
    elif isinstance(cfg, V15Config):
        v15cfg = cfg
    else:
        raise TypeError(f"from_config: cfg 类型无法识别: {type(cfg)}")

    # 2) 应用全局常量（mapping / frame_id / topic）—— 只在配置与当前默认不同时有副作用
    try:
        from .control_core import types as _types
        _types.apply_joint_mapping_config(v15cfg.mapping)
        _types.apply_ros_protocol_constants(v15cfg.ros)
    except Exception:
        pass

    # 3) 构建 adapter
    backend = str(adapter_backend).lower()
    if backend == "mock":
        adapter = MockAdapter()
    elif backend in ("ros", "ros2", "v14"):
        adapter = RosV14Adapter.from_config(v15cfg)
    else:
        raise ValueError(f"from_config: adapter_backend 必须是 'mock' / 'ros'，实际 {adapter_backend}")

    if start_adapter:
        adapter.open()

    # 4) 构建 controller + kinematics + mover
    controller = v15cfg.build_controller(adapter, use_config_limits=use_config_limits)
    fk, ik = v15cfg.build_kinematics()
    mover = v15cfg.build_mover(controller, fk=fk, ik=ik)

    # 5) 可选构建 SensorManager（init_sensors=False 时返回 None，保持旧 API 零 break）
    sensor_manager: _Optional[SensorManager] = None
    if init_sensors:
        sensor_manager = SensorManager.from_config(v15cfg, ros_environment=sensor_ros_mode)
        sensor_manager.open()

    # 6) 可选构建 WorkspaceChecker（init_workspace=False 时返回 None）
    workspace_checker: _Optional[WorkspaceChecker] = None
    if init_workspace:
        workspace_checker = WorkspaceChecker.from_config(v15cfg)

    # 7) 可选构建 TrajectoryPlanner（init_trajectory_planner=False 时返回 None）
    trajectory_planner: _Optional[TrajectoryPlanner] = None
    if init_trajectory_planner:
        trajectory_planner = TrajectoryPlanner.from_config(
            v15cfg, workspace_checker=workspace_checker, ik=ik,
        )

    return {
        "config": v15cfg,
        "controller": controller,
        "adapter": adapter,
        "fk": fk,
        "ik": ik,
        "mover": mover,
        "sensor_manager": sensor_manager,
        "workspace_checker": workspace_checker,
        "trajectory_planner": trajectory_planner,
    }


__all__ = [
    # control_core
    "URDFController",
    "ControlAdapter",
    "MockAdapter",
    "RosV14Adapter",
    "SEMANTIC_JOINT_ORDER",
    "SEMANTIC_TO_URDF",
    "URDF_JOINT_ORDER",
    "URDF_TO_SEMANTIC",
    "DEFAULT_FRAME_ID",
    "DEFAULT_JOINT_TOPIC",
    "deg_to_rad",
    "rad_to_deg",
    "default_pose_deg",
    # kinematics
    "LinkParams",
    "DEFAULT_PARAMS",
    "get_default_params",
    "FKSolution",
    "ForwardKinematics",
    "IKSolution",
    "InverseKinematics",
    # motion
    "CartesianMover",
    "MoveResult",
    "move_to_cartesian",
    "WorkspaceChecker",
    "ReachabilityReport",
    "TrajectoryPlanner",
    "JointTrajectoryPoint",
    "IKPlanningResult",
    # config (扩展：7 子配置类 + 2 加载器 + 1 顶层构建器)
    "V15Config",
    "load_config",
    "load_default_config",
    "SingleSensorConfig",
    "SensorsConfig",
    "ExtrinsicsEntry",
    "ExtrinsicsConfig",
    "TiltCompensationConfig",
    # sensors (FR-3 传感器订阅接口 + FR-6 温漂补偿)
    "SensorManager",
    "TiltReading",
    "LidarReading",
    "CameraReading",
    "TiltCompensator",
    # action_library semantic_moves (FR-2 8 指令库)
    "swing_move",
    "boom_move",
    "arm_move",
    "bucket_move",
    "swing_move_to",
    "boom_lift",
    "arm_extend",
    "bucket_tilt_to",
    # action_library composites (接口2 + 接口3, 卸料转运 + 卸料过程)
    "CompositeActionResult",
    "move_to_dump_transit",
    "dump_material",
    # 顶层 from_config 聚合入口
    "from_config",
]

