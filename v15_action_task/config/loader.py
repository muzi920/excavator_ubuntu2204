"""
v15_action_task YAML 配置加载器。

使用示例：
```
    from v15_action_task.config import load_config, load_default_config, V15Config

    # 方式 A：加载默认配置（包内 default_config.yaml）
    cfg = load_default_config()

    # 方式 B：加载用户自定义 YAML（其他项目只需改这个文件）
    cfg = load_config("/path/to/my_excavator.yaml")

    # → 构造运动学（连杆参数 + 传感器偏置）
    fk = ForwardKinematics(cfg.link.to_link_params())
    ik = InverseKinematics(cfg.link.to_link_params())

    # → 构造 ROS Adapter（话题、frame_id、QoS 全来自 YAML）
    adapter = RosV14Adapter.from_config(cfg)

    # → 构造控制器（带上 YAML 里的限位）
    ctl = URDFController(adapter, limits=cfg.limits.to_joint_limits_dict())
```
"""

from __future__ import annotations

import copy
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


# ==============================================================
# 1. 各子配置的 dataclass
# ==============================================================


@dataclass
class JointMappingConfig:
    """语义关节名 ↔ URDF 关节名的绑定。"""

    mapping: Dict[str, str]  # {semantic: urdf_joint}

    # ---- 便捷属性 ----
    @property
    def semantic_order(self) -> Tuple[str, ...]:
        return tuple(self.mapping.keys())

    @property
    def urdf_order(self) -> Tuple[str, ...]:
        return tuple(self.mapping[k] for k in self.mapping.keys())

    @property
    def urdf_to_semantic(self) -> Dict[str, str]:
        return {v: k for k, v in self.mapping.items()}

    def to_constants(self) -> Dict[str, Any]:
        """返回 types 所需的常量 dict：SEMANTIC_TO_URDF / URDF_TO_SEMANTIC / SEMANTIC_JOINT_ORDER / URDF_JOINT_ORDER"""
        return {
            "SEMANTIC_TO_URDF": dict(self.mapping),
            "URDF_TO_SEMANTIC": self.urdf_to_semantic,
            "SEMANTIC_JOINT_ORDER": self.semantic_order,
            "URDF_JOINT_ORDER": self.urdf_order,
        }


@dataclass
class SingleJointLimit:
    min_deg: float
    max_deg: float
    description: str = ""

    def to_tuple(self) -> Tuple[float, float]:
        return (float(self.min_deg), float(self.max_deg))


@dataclass
class JointLimitsConfig:
    """4 个关节的限位集合。"""

    limits: Dict[str, SingleJointLimit]  # {semantic: SingleJointLimit}

    # ---- 便捷方法 ----
    def to_joint_limits_dict(self) -> Dict[str, Any]:
        """返回 action_library.utils.joint_limits 可直接替换的 JOINT_LIMITS dict。"""
        # 懒加载 import，避免循环依赖
        try:
            from ..action_library.utils.joint_limits import JointLimit
        except (ImportError, ValueError):
            try:
                from action_library.utils.joint_limits import JointLimit
            except ImportError:
                raise  # 让用户知道 joint_limits 模块不可用

        return {
            name: JointLimit(float(lm.min_deg), float(lm.max_deg))
            for name, lm in self.limits.items()
        }

    def clamp_pose(self, pose_deg: Dict[str, float]) -> Dict[str, float]:
        """在 config 层直接裁剪 pose（不依赖 action_library）。"""
        out: Dict[str, float] = {}
        for k, v in pose_deg.items():
            lm = self.limits.get(k)
            if lm is None:
                out[k] = float(v)
            else:
                vf = float(v)
                if vf < lm.min_deg:
                    vf = lm.min_deg
                elif vf > lm.max_deg:
                    vf = lm.max_deg
                out[k] = vf
        return out

    def check(self, pose_deg: Dict[str, float]) -> Tuple[bool, Dict[str, Tuple[float, float, float]]]:
        violations: Dict[str, Tuple[float, float, float]] = {}
        for k, v in pose_deg.items():
            lm = self.limits.get(k)
            if lm is None:
                continue
            vf = float(v)
            if vf < lm.min_deg - 1e-9 or vf > lm.max_deg + 1e-9:
                violations[k] = (vf, lm.min_deg, lm.max_deg)
        return (len(violations) == 0, violations)


@dataclass
class LinkGeometryConfig:
    """连杆几何 + 传感器偏置。"""

    offset_x: float
    offset_z: float
    L1: float
    L2: float
    boom_bend_angle_deg: float
    L_arm: float
    L_bucket: float
    offset_sensor_boom: float
    offset_sensor_arm: float
    offset_sensor_bucket: float

    # ---- 派生量（惰性计算）----
    _cached_link_params: Any = field(default=None, repr=False)

    def to_link_params(self) -> Any:
        """返回 kinematics.link_params.LinkParams 实例。"""
        if self._cached_link_params is not None:
            return self._cached_link_params

        # 懒加载 import
        try:
            from ..kinematics.link_params import LinkParams
        except (ImportError, ValueError):
            try:
                from kinematics.link_params import LinkParams
            except ImportError:
                raise ImportError("kinematics.link_params 模块不可用，无法构造 LinkParams")

        L_boom, beta_deg = LinkParams.compute_boom_equiv(
            self.L1, self.L2, self.boom_bend_angle_deg
        )
        lp = LinkParams(
            offset_x=float(self.offset_x),
            offset_z=float(self.offset_z),
            L1=float(self.L1),
            L2=float(self.L2),
            boom_bend_angle_deg=float(self.boom_bend_angle_deg),
            L_arm=float(self.L_arm),
            L_bucket=float(self.L_bucket),
            offset_sensor_boom=float(self.offset_sensor_boom),
            offset_sensor_arm=float(self.offset_sensor_arm),
            offset_sensor_bucket=float(self.offset_sensor_bucket),
            L_boom=L_boom,
            beta_deg=beta_deg,
        )
        self._cached_link_params = lp
        return lp


@dataclass
class RosProtocolConfig:
    """ROS 2 话题协议。"""

    node_name: str
    joint_topic: str
    frame_id: str
    qos_depth: int
    msg_type: str
    first_publish_sync_from_feedback: bool

    def to_adapter_kwargs(self) -> Dict[str, Any]:
        """返回 RosV14Adapter.__init__ 可直接 **unpack 的 kwargs。"""
        return {
            "node_name": str(self.node_name),
            "topic": str(self.joint_topic),
            "qos_depth": int(self.qos_depth),
            "default_frame_id": str(self.frame_id),
            "first_publish_sync": bool(self.first_publish_sync_from_feedback),
        }


@dataclass
class MotionDefaultsConfig:
    at_pose_tolerance_deg: float
    move_timeout_s: float
    bucket_search_range_deg: Tuple[float, float]
    bucket_search_samples: int


@dataclass
class StandardPosesConfig:
    """默认标准姿态集合（init / home / cycle_transit 等）。"""

    poses: Dict[str, Dict[str, float]]


@dataclass
class SingleSensorConfig:
    """通用单传感器配置（超类，兼容 tilt / lidar / camera 三种类型）。"""

    id: str
    type: str
    topic: str
    msg_type: str
    frame_id: str
    qos_depth: int
    enabled: bool
    extra: Dict[str, Any]
    modbus_addr: Optional[str] = None
    array_index: Optional[int] = None
    semantic_joint: Optional[str] = None
    info_topic: str = ""

    # ---- typed view accessors ----
    def as_tilt(self) -> Optional["SingleSensorConfig"]:
        if (
            self.modbus_addr is not None
            and self.array_index is not None
            and self.semantic_joint is not None
        ):
            return self
        return None

    def as_lidar(self) -> Optional["SingleSensorConfig"]:
        return self

    def as_camera(self) -> Optional["SingleSensorConfig"]:
        return self


@dataclass
class SensorsConfig:
    """传感器集合：4倾角 + 5激光雷达 + 6相机。"""

    tilt_sensors: Dict[str, SingleSensorConfig]
    lidars: Dict[str, SingleSensorConfig]
    cameras: Dict[str, SingleSensorConfig]

    def list_tilt_ids(self) -> List[str]:
        return list(self.tilt_sensors.keys())

    def list_lidar_ids(self) -> List[str]:
        return list(self.lidars.keys())

    def list_camera_ids(self) -> List[str]:
        return list(self.cameras.keys())

    def by_id(self, sid: str) -> Optional[SingleSensorConfig]:
        if sid in self.tilt_sensors:
            return self.tilt_sensors[sid]
        if sid in self.lidars:
            return self.lidars[sid]
        if sid in self.cameras:
            return self.cameras[sid]
        return None


@dataclass
class ExtrinsicsEntry:
    """单个 SE(3) 6-DOF 外参标定条目。"""

    sensor_id: str
    parent_frame: str
    child_frame: str
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    pitch_rad: float
    roll_rad: float
    enabled: bool
    note: str
    quaternion_xyzw_or_none: Optional[Tuple[float, float, float, float]] = None
    matrix_4x4_or_none: Optional[List[List[float]]] = None

    def to_xyzrpy(self) -> Tuple[float, float, float, float, float, float]:
        return (
            float(self.x_m),
            float(self.y_m),
            float(self.z_m),
            float(self.yaw_rad),
            float(self.pitch_rad),
            float(self.roll_rad),
        )

    def to_4x4_matrix(self) -> List[List[float]]:
        """纯数学计算：从 x/y/z + yaw/pitch/roll 计算 4x4 SE(3) 变换矩阵。"""
        import math

        x, y, z = float(self.x_m), float(self.y_m), float(self.z_m)
        yaw, pitch, roll = float(self.yaw_rad), float(self.pitch_rad), float(self.roll_rad)

        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)

        R00 = cy * cp
        R01 = cy * sp * sr - sy * cr
        R02 = cy * sp * cr + sy * sr
        R10 = sy * cp
        R11 = sy * sp * sr + cy * cr
        R12 = sy * sp * cr - cy * sr
        R20 = -sp
        R21 = cp * sr
        R22 = cp * cr

        return [
            [R00, R01, R02, x],
            [R10, R11, R12, y],
            [R20, R21, R22, z],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def to_static_transform_publisher_args(self) -> List[str]:
        """返回 static_transform_publisher CLI 参数列表（顺序 x y z yaw pitch roll）。"""
        return [
            str(float(self.x_m)),
            str(float(self.y_m)),
            str(float(self.z_m)),
            str(float(self.yaw_rad)),
            str(float(self.pitch_rad)),
            str(float(self.roll_rad)),
        ]


@dataclass
class ExtrinsicsConfig:
    """外参标定集合。"""

    entries: Dict[str, ExtrinsicsEntry]

    def get(self, sid: str) -> Optional[ExtrinsicsEntry]:
        for entry in self.entries.values():
            if entry.sensor_id == sid:
                return entry
        return None

    def items(self):
        return self.entries.items()

    def to_launch_static_tf_nodes_yaml(self) -> str:
        """生成 ROS 2 static_transform_publisher CLI 行（用于 launch 文件）。"""
        lines: List[str] = []
        for key, entry in self.entries.items():
            if not entry.enabled:
                continue
            args = entry.to_static_transform_publisher_args()
            line = (
                f"# {key} ({entry.note})\n"
                f"ros2 run tf2_ros static_transform_publisher "
                f"{' '.join(args)} {entry.parent_frame} {entry.child_frame}"
            )
            lines.append(line)
        return "\n\n".join(lines)


@dataclass
class TiltCompensationConfig:
    """倾角传感器互补滤波 + 自动校准参数。"""

    alpha: float
    calib_count: int
    gyro_deadzone_rad_s: float
    auto_calibrate_on_open: bool
    use_relative_subtraction: bool


@dataclass
class WorkspaceConfig:
    """可达域/工作空间 5 层约束的配置（WorkspaceChecker）。"""

    min_ground_z: float
    min_transit_z: float
    singularity_margin_min_m: float
    singularity_margin_ratio: float


@dataclass
class TrajectoryConfig:
    """双策略 TrajectoryPlanner 默认参数。"""

    default_strategy: str
    lerp_step_deg: float
    max_speed_deg_s: Dict[str, float]
    accel_deg_s2: Dict[str, float]


# ==============================================================
# 2. 顶层配置 dataclass
# ==============================================================


@dataclass
class V15Config:
    """v15_action_task 全部配置的聚合对象。"""

    version: str
    model_name: str
    description: str
    mapping: JointMappingConfig
    limits: JointLimitsConfig
    link: LinkGeometryConfig
    ros: RosProtocolConfig
    motion: MotionDefaultsConfig
    standard_poses: StandardPosesConfig
    sensors: SensorsConfig
    extrinsics: ExtrinsicsConfig
    tilt_compensation: TiltCompensationConfig
    workspace: WorkspaceConfig
    trajectory: TrajectoryConfig
    raw_dict: Dict[str, Any]  # 原始 YAML dict，供扩展调试

    # ---- 快捷构造函数 ----
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "V15Config":
        """从 YAML 解析出的 dict 构造 V15Config（带类型校验 + 兜底默认值）。"""
        d = copy.deepcopy(data or {})

        version = str(d.get("v15_config_version", "1.0"))
        model_name = str(d.get("model_name", "excavator_default"))
        description = str(d.get("description", ""))

        # ① 关节映射
        raw_map = dict(d.get("joint_mapping", {}) or {})
        if not raw_map:
            raw_map = {
                "swing_yaw": "swing_joint",
                "boom_swing": "boom_joint",
                "arm_boom": "arm_joint",
                "bucket_arm": "bucket_joint",
            }
        mapping = JointMappingConfig(mapping={str(k): str(v) for k, v in raw_map.items()})

        # ② 关节限位
        raw_lim = dict(d.get("joint_limits", {}) or {})
        default_lims = {
            "swing_yaw":  (-180.0, 180.0, "回转角"),
            "boom_swing": (-5.0,   55.0,  "大臂"),
            "arm_boom":   (0.0,    130.0, "小臂"),
            "bucket_arm": (-95.0,  45.0,  "铲斗"),
        }
        limits_map: Dict[str, SingleJointLimit] = {}
        for sem in mapping.semantic_order:
            if sem in raw_lim:
                entry = raw_lim[sem]
                if isinstance(entry, dict):
                    mn = float(entry.get("min_deg", default_lims.get(sem, (-9e9, 9e9))[0]))
                    mx = float(entry.get("max_deg", default_lims.get(sem, (-9e9, 9e9))[1]))
                    desc = str(entry.get("description", ""))
                else:
                    raise TypeError(f"joint_limits[{sem}] 必须是 dict(含 min_deg/max_deg)，实际 {type(entry)}")
            elif sem in default_lims:
                mn, mx, desc = default_lims[sem]
            else:
                mn, mx, desc = -9e9, 9e9, ""
            limits_map[sem] = SingleJointLimit(mn, mx, desc)
        limits = JointLimitsConfig(limits=limits_map)

        # ③ 连杆几何
        raw_link = dict(d.get("link_geometry", {}) or {})
        link_defs = {
            "offset_x": 0.25,
            "offset_z": 0.40,
            "L1": 0.35,
            "L2": 0.60,
            "boom_bend_angle_deg": 46.0,
            "L_arm": 0.44,
            "L_bucket": 0.26,
        }
        link_kwargs = {k: float(raw_link.get(k, v)) for k, v in link_defs.items()}
        sensor_offsets = dict(raw_link.get("sensor_offsets_deg", {}) or {})
        link_kwargs["offset_sensor_boom"]   = float(sensor_offsets.get("boom",   40.9))
        link_kwargs["offset_sensor_arm"]    = float(sensor_offsets.get("arm",    19.6))
        link_kwargs["offset_sensor_bucket"] = float(sensor_offsets.get("bucket", -56.2))
        link = LinkGeometryConfig(**link_kwargs)

        # ④ ROS 协议
        raw_ros = dict(d.get("ros_protocol", {}) or {})
        ros = RosProtocolConfig(
            node_name=str(raw_ros.get("node_name", "v15_urdf_controller")),
            joint_topic=str(raw_ros.get("joint_topic", "/joint_states")),
            frame_id=str(raw_ros.get("frame_id", "base_link")),
            qos_depth=int(raw_ros.get("qos_depth", 10)),
            msg_type=str(raw_ros.get("msg_type", "sensor_msgs/msg/JointState")),
            first_publish_sync_from_feedback=bool(
                raw_ros.get("first_publish_sync_from_feedback", True)
            ),
        )

        # ⑤ 运动默认参数
        raw_mot = dict(d.get("motion_defaults", {}) or {})
        rng = raw_mot.get("bucket_search_range_deg", [-70.0, 10.0])
        if isinstance(rng, (list, tuple)) and len(rng) >= 2:
            rng_tuple = (float(rng[0]), float(rng[1]))
        else:
            rng_tuple = (-70.0, 10.0)
        motion = MotionDefaultsConfig(
            at_pose_tolerance_deg=float(raw_mot.get("at_pose_tolerance_deg", 1.0)),
            move_timeout_s=float(raw_mot.get("move_timeout_s", 3.0)),
            bucket_search_range_deg=rng_tuple,
            bucket_search_samples=int(raw_mot.get("bucket_search_samples", 17)),
        )

        # ⑥ 标准姿态
        raw_poses = dict(d.get("standard_poses", {}) or {})
        poses: Dict[str, Dict[str, float]] = {}
        for name, entry in raw_poses.items():
            if isinstance(entry, dict):
                poses[str(name)] = {str(k): float(v) for k, v in entry.items() if k in mapping.semantic_order}
        standard_poses = StandardPosesConfig(poses=poses)

        # ⑦ 传感器配置（sensors）
        if "sensors" in d and isinstance(d["sensors"], dict):
            raw_sensors = dict(d["sensors"])
            _parse_sensor_entry = lambda e: SingleSensorConfig(
                id=str(e.get("id", "")),
                type=str(e.get("type", "")),
                topic=str(e.get("topic", "")),
                msg_type=str(e.get("msg_type", "")),
                frame_id=str(e.get("frame_id", "")),
                qos_depth=int(e.get("qos_depth", 10)),
                enabled=bool(e.get("enabled", False)),
                extra=dict(e.get("extra", {}) or {}),
                modbus_addr=(str(e["modbus_addr"]) if "modbus_addr" in e and e["modbus_addr"] is not None else None),
                array_index=(int(e["array_index"]) if "array_index" in e and e["array_index"] is not None else None),
                semantic_joint=(str(e["semantic_joint"]) if "semantic_joint" in e and e["semantic_joint"] is not None else None),
                info_topic=str(e.get("info_topic", "")),
            )
            tilt_raw = dict(raw_sensors.get("tilt_sensors", {}) or {})
            tilt_map: Dict[str, SingleSensorConfig] = {}
            for k, v in tilt_raw.items():
                if isinstance(v, dict):
                    tilt_map[str(k)] = _parse_sensor_entry(v)
            lidar_raw = dict(raw_sensors.get("lidars", {}) or {})
            lidar_map: Dict[str, SingleSensorConfig] = {}
            for k, v in lidar_raw.items():
                if isinstance(v, dict):
                    lidar_map[str(k)] = _parse_sensor_entry(v)
            cam_raw = dict(raw_sensors.get("cameras", {}) or {})
            cam_map: Dict[str, SingleSensorConfig] = {}
            for k, v in cam_raw.items():
                if isinstance(v, dict):
                    cam_map[str(k)] = _parse_sensor_entry(v)
            sensors = SensorsConfig(tilt_sensors=tilt_map, lidars=lidar_map, cameras=cam_map)
        else:
            sensors = build_default_sensors_config()

        # ⑧ 外参配置（extrinsics）
        if "extrinsics" in d and isinstance(d["extrinsics"], dict):
            raw_ext = dict(d["extrinsics"])
            entries: Dict[str, ExtrinsicsEntry] = {}
            for k, v in raw_ext.items():
                if isinstance(v, dict):
                    q = v.get("quaternion_xyzw", None)
                    if isinstance(q, (list, tuple)) and len(q) >= 4:
                        q_opt: Optional[Tuple[float, float, float, float]] = (
                            float(q[0]), float(q[1]), float(q[2]), float(q[3])
                        )
                    else:
                        q_opt = None
                    m = v.get("matrix_4x4", None)
                    if isinstance(m, list) and len(m) >= 4 and all(isinstance(row, list) and len(row) >= 4 for row in m):
                        m_opt: Optional[List[List[float]]] = [
                            [float(m[r][c]) for c in range(4)] for r in range(4)
                        ]
                    else:
                        m_opt = None
                    entries[str(k)] = ExtrinsicsEntry(
                        sensor_id=str(v.get("sensor_id", "")),
                        parent_frame=str(v.get("parent_frame", "")),
                        child_frame=str(v.get("child_frame", "")),
                        x_m=float(v.get("x_m", 0.0)),
                        y_m=float(v.get("y_m", 0.0)),
                        z_m=float(v.get("z_m", 0.0)),
                        yaw_rad=float(v.get("yaw_rad", 0.0)),
                        pitch_rad=float(v.get("pitch_rad", 0.0)),
                        roll_rad=float(v.get("roll_rad", 0.0)),
                        enabled=bool(v.get("enabled", False)),
                        note=str(v.get("note", "")),
                        quaternion_xyzw_or_none=q_opt,
                        matrix_4x4_or_none=m_opt,
                    )
            extrinsics = ExtrinsicsConfig(entries=entries)
        else:
            extrinsics = build_default_extrinsics_config()

        # ⑨ 倾角补偿（tilt_compensation）
        if "tilt_compensation" in d and isinstance(d["tilt_compensation"], dict):
            raw_tc = dict(d["tilt_compensation"])
            tilt_compensation = TiltCompensationConfig(
                alpha=float(raw_tc.get("alpha", 0.98)),
                calib_count=int(raw_tc.get("calib_count", 50)),
                gyro_deadzone_rad_s=float(raw_tc.get("gyro_deadzone_rad_s", 0.002)),
                auto_calibrate_on_open=bool(raw_tc.get("auto_calibrate_on_open", True)),
                use_relative_subtraction=bool(raw_tc.get("use_relative_subtraction", True)),
            )
        else:
            tilt_compensation = build_default_tilt_compensation_config()

        # ⑩ 可达域/工作空间（workspace）
        if "workspace" in d and isinstance(d["workspace"], dict):
            raw_ws = dict(d["workspace"])
            workspace = WorkspaceConfig(
                min_ground_z=float(raw_ws.get("min_ground_z", -0.1)),
                min_transit_z=float(raw_ws.get("min_transit_z", 0.3)),
                singularity_margin_min_m=float(raw_ws.get("singularity_margin_min_m", 0.02)),
                singularity_margin_ratio=float(raw_ws.get("singularity_margin_ratio", 0.01)),
            )
        else:
            workspace = build_default_workspace_config()

        # ⑪ 轨迹规划默认参数（trajectory）
        if "trajectory" in d and isinstance(d["trajectory"], dict):
            raw_tr = dict(d["trajectory"])
            raw_speed = dict(raw_tr.get("max_speed_deg_s", {}) or {})
            raw_accel = dict(raw_tr.get("accel_deg_s2", {}) or {})
            default_speed = {
                "swing_yaw": 15.0, "boom_swing": 20.0, "arm_boom": 30.0, "bucket_arm": 30.0,
            }
            default_accel = {
                "swing_yaw": 30.0, "boom_swing": 40.0, "arm_boom": 60.0, "bucket_arm": 60.0,
            }
            max_speed = {
                k: float(raw_speed[k]) if k in raw_speed else default_speed[k]
                for k in default_speed
            }
            accel = {
                k: float(raw_accel[k]) if k in raw_accel else default_accel[k]
                for k in default_accel
            }
            trajectory = TrajectoryConfig(
                default_strategy=str(raw_tr.get("default_strategy", "lerp")),
                lerp_step_deg=float(raw_tr.get("lerp_step_deg", 5.0)),
                max_speed_deg_s=max_speed,
                accel_deg_s2=accel,
            )
        else:
            trajectory = build_default_trajectory_config()

        return V15Config(
            version=version,
            model_name=model_name,
            description=description,
            mapping=mapping,
            limits=limits,
            link=link,
            ros=ros,
            motion=motion,
            standard_poses=standard_poses,
            sensors=sensors,
            extrinsics=extrinsics,
            tilt_compensation=tilt_compensation,
            workspace=workspace,
            trajectory=trajectory,
            raw_dict=data,
        )

    @classmethod
    def from_yaml_file(cls, path: str) -> "V15Config":
        """从磁盘 YAML 文件加载（若无 PyYAML，自动 fallback 尝试用 .json 同路径同名文件）。"""
        if not os.path.isfile(path):
            raise FileNotFoundError(f"配置文件不存在: {path}")
        ext = os.path.splitext(path)[1].lower()
        if ext in (".yaml", ".yml"):
            try:
                import yaml  # type: ignore
            except ImportError:
                # 无 pyyaml：尝试同目录同文件名的 .json 作为替代
                alt_json = os.path.splitext(path)[0] + ".json"
                if os.path.isfile(alt_json):
                    import json as _json
                    with open(alt_json, "r", encoding="utf-8") as fh:
                        raw = _json.load(fh)
                    if not isinstance(raw, dict):
                        raise ValueError(f"JSON 顶层必须是 dict，实际: {type(raw)}")
                    return cls.from_dict(raw)
                raise ImportError(
                    "加载 YAML 配置需要 PyYAML：pip install pyyaml (或 apt install python3-yaml)。"
                    " 无网络环境下可改存为 .json 并放入同目录（文件名同名），load_config() 会自动 fallback 读取。"
                )
            with open(path, "r", encoding="utf-8") as fh:
                raw = yaml.safe_load(fh)
        elif ext == ".json":
            import json as _json
            with open(path, "r", encoding="utf-8") as fh:
                raw = _json.load(fh)
        else:
            raise ValueError(f"不支持的配置文件扩展名: {ext} (仅支持 .yaml/.yml/.json)")
        if not isinstance(raw, dict):
            raise ValueError(f"配置顶层必须是 dict，实际: {type(raw)}")
        return cls.from_dict(raw)

    # ---- 一次性构造完整工具链 ----
    def build_controller(
        self,
        adapter: Any,
        use_config_limits: bool = True,
    ) -> Any:
        """使用当前配置 + 给定 adapter，构建 URDFController。"""
        try:
            from ..control_core import URDFController
        except (ImportError, ValueError):
            from control_core import URDFController
        if use_config_limits:
            return URDFController(adapter, joint_limits=self.limits.to_joint_limits_dict())
        return URDFController(adapter)

    def build_kinematics(self) -> Tuple[Any, Any]:
        """使用当前配置构建 (ForwardKinematics, InverseKinematics)。"""
        lp = self.link.to_link_params()
        try:
            from ..kinematics import ForwardKinematics, InverseKinematics
        except (ImportError, ValueError):
            from kinematics import ForwardKinematics, InverseKinematics
        return ForwardKinematics(lp), InverseKinematics(lp)

    def build_mover(self, controller: Any, fk: Optional[Any] = None, ik: Optional[Any] = None) -> Any:
        """使用当前配置的 motion_defaults 构建 CartesianMover。"""
        if ik is None:
            _, ik = self.build_kinematics()
        try:
            from ..motion import CartesianMover
        except (ImportError, ValueError):
            from motion import CartesianMover
        return CartesianMover(
            controller,
            ik,
            fk=fk,
            default_tolerance_deg=self.motion.at_pose_tolerance_deg,
            default_timeout_s=self.motion.move_timeout_s,
            default_bucket_range=self.motion.bucket_search_range_deg,
            default_bucket_candidates=self.motion.bucket_search_samples,
        )



# ==============================================================
# 默认构造辅助函数（当 YAML 缺新 section 时兜底生成）
# ==============================================================


def build_default_sensors_config() -> SensorsConfig:
    """构造与 default_config.yaml Section 7 数值完全一致的 SensorsConfig。"""
    _mk_sensor = lambda **kw: SingleSensorConfig(
        id=str(kw.get("id", "")),
        type=str(kw.get("type", "")),
        topic=str(kw.get("topic", "")),
        msg_type=str(kw.get("msg_type", "")),
        frame_id=str(kw.get("frame_id", "")),
        qos_depth=int(kw.get("qos_depth", 10)),
        enabled=bool(kw.get("enabled", False)),
        extra=dict(kw.get("extra", {}) or {}),
        modbus_addr=kw.get("modbus_addr", None),
        array_index=kw.get("array_index", None),
        semantic_joint=kw.get("semantic_joint", None),
        info_topic=str(kw.get("info_topic", "")),
    )
    tilt_sensors: Dict[str, SingleSensorConfig] = {
        "tilt_bucket": _mk_sensor(
            id="tilt_bucket", type="tilt", topic="/excavator/inclinometer_pitch_deg",
            msg_type="std_msgs/msg/Float64", frame_id="bucket_link", qos_depth=10,
            enabled=True, extra={}, modbus_addr="0x50", array_index=0, semantic_joint="bucket_arm",
        ),
        "tilt_arm": _mk_sensor(
            id="tilt_arm", type="tilt", topic="/excavator/inclinometer_pitch_deg",
            msg_type="std_msgs/msg/Float64", frame_id="arm_link", qos_depth=10,
            enabled=True, extra={}, modbus_addr="0x51", array_index=1, semantic_joint="arm_boom",
        ),
        "tilt_boom": _mk_sensor(
            id="tilt_boom", type="tilt", topic="/excavator/inclinometer_pitch_deg",
            msg_type="std_msgs/msg/Float64", frame_id="boom_link", qos_depth=10,
            enabled=True, extra={}, modbus_addr="0x52", array_index=2, semantic_joint="boom_swing",
        ),
        "tilt_swing": _mk_sensor(
            id="tilt_swing", type="tilt", topic="/excavator/inclinometer_pitch_deg",
            msg_type="std_msgs/msg/Float64", frame_id="swing_link", qos_depth=10,
            enabled=True, extra={}, modbus_addr="0x53", array_index=3, semantic_joint="swing_yaw",
        ),
    }
    lidars: Dict[str, SingleSensorConfig] = {
        "lidar_single_rear": _mk_sensor(
            id="lidar_single_rear", type="lidar", topic="/pointcloud",
            msg_type="sensor_msgs/msg/PointCloud2", frame_id="lidar_rear_link",
            qos_depth=10, enabled=True, extra={},
        ),
        "lidar_front_left": _mk_sensor(
            id="lidar_front_left", type="lidar", topic="/lidar/front_left/points",
            msg_type="sensor_msgs/msg/PointCloud2", frame_id="lidar_front_left_link",
            qos_depth=10, enabled=False, extra={},
        ),
        "lidar_front_right": _mk_sensor(
            id="lidar_front_right", type="lidar", topic="/lidar/front_right/points",
            msg_type="sensor_msgs/msg/PointCloud2", frame_id="lidar_front_right_link",
            qos_depth=10, enabled=False, extra={},
        ),
        "lidar_rear_left": _mk_sensor(
            id="lidar_rear_left", type="lidar", topic="/lidar/rear_left/points",
            msg_type="sensor_msgs/msg/PointCloud2", frame_id="lidar_rear_left_link",
            qos_depth=10, enabled=False, extra={},
        ),
        "lidar_rear_right": _mk_sensor(
            id="lidar_rear_right", type="lidar", topic="/lidar/rear_right/points",
            msg_type="sensor_msgs/msg/PointCloud2", frame_id="lidar_rear_right_link",
            qos_depth=10, enabled=False, extra={},
        ),
    }
    cameras: Dict[str, SingleSensorConfig] = {
        "cam_front_main": _mk_sensor(
            id="cam_front_main", type="camera", topic="/sensors/camera/front_main/image_raw",
            msg_type="sensor_msgs/msg/Image", frame_id="cam_front_main_link",
            qos_depth=10, enabled=True, info_topic="/sensors/camera/front_main/camera_info", extra={},
        ),
        "cam_left": _mk_sensor(
            id="cam_left", type="camera", topic="/sensors/camera/left/image_raw",
            msg_type="sensor_msgs/msg/Image", frame_id="cam_left_link",
            qos_depth=10, enabled=True, info_topic="", extra={},
        ),
        "cam_right": _mk_sensor(
            id="cam_right", type="camera", topic="/sensors/camera/right/image_raw",
            msg_type="sensor_msgs/msg/Image", frame_id="cam_right_link",
            qos_depth=10, enabled=True, info_topic="", extra={},
        ),
        "cam_rear": _mk_sensor(
            id="cam_rear", type="camera", topic="/sensors/camera/rear/image_raw",
            msg_type="sensor_msgs/msg/Image", frame_id="cam_rear_link",
            qos_depth=10, enabled=False, info_topic="", extra={},
        ),
        "cab_left": _mk_sensor(
            id="cab_left", type="camera", topic="/sensors/camera/cab_left/image_raw",
            msg_type="sensor_msgs/msg/Image", frame_id="cab_left_link",
            qos_depth=10, enabled=False, info_topic="", extra={},
        ),
        "cab_right": _mk_sensor(
            id="cab_right", type="camera", topic="/sensors/camera/cab_right/image_raw",
            msg_type="sensor_msgs/msg/Image", frame_id="cab_right_link",
            qos_depth=10, enabled=False, info_topic="", extra={},
        ),
    }
    return SensorsConfig(tilt_sensors=tilt_sensors, lidars=lidars, cameras=cameras)


def build_default_extrinsics_config() -> ExtrinsicsConfig:
    """构造与 default_config.yaml Section 8 数值完全一致的 ExtrinsicsConfig（10 条目）。"""
    _mk = lambda **kw: ExtrinsicsEntry(
        sensor_id=str(kw.get("sensor_id", "")),
        parent_frame=str(kw.get("parent_frame", "base_link")),
        child_frame=str(kw.get("child_frame", "")),
        x_m=float(kw.get("x_m", 0.0)),
        y_m=float(kw.get("y_m", 0.0)),
        z_m=float(kw.get("z_m", 0.0)),
        yaw_rad=float(kw.get("yaw_rad", 0.0)),
        pitch_rad=float(kw.get("pitch_rad", 0.0)),
        roll_rad=float(kw.get("roll_rad", 0.0)),
        enabled=bool(kw.get("enabled", False)),
        note=str(kw.get("note", "")),
        quaternion_xyzw_or_none=None,
        matrix_4x4_or_none=None,
    )
    entries: Dict[str, ExtrinsicsEntry] = {
        "ext_lidar_single_rear": _mk(
            sensor_id="lidar_single_rear", parent_frame="base_link", child_frame="lidar_rear_link",
            x_m=-0.5500, y_m=-0.2000, z_m=1.2712, yaw_rad=0.0532, pitch_rad=0.0349, roll_rad=3.0316,
            enabled=True, note="后向单线束激光雷达（标定值）",
        ),
        "ext_cam_front_main": _mk(
            sensor_id="cam_front_main", parent_frame="base_link", child_frame="cam_front_main_link",
            x_m=0.4539, y_m=0.1532, z_m=1.5246, yaw_rad=0.0, pitch_rad=1.1519, roll_rad=0.0,
            enabled=True, note="前视主相机（标定值）",
        ),
        "ext_cam_left": _mk(
            sensor_id="cam_left", parent_frame="base_link", child_frame="cam_left_link",
            x_m=0.4239, y_m=-0.1768, z_m=1.4246, yaw_rad=0.0, pitch_rad=0.9250, roll_rad=0.0,
            enabled=True, note="左视相机（标定值）",
        ),
        "ext_cam_right": _mk(
            sensor_id="cam_right", parent_frame="base_link", child_frame="cam_right_link",
            enabled=False, note="右视相机（占位，待标定）",
        ),
        "ext_cam_rear": _mk(
            sensor_id="cam_rear", parent_frame="base_link", child_frame="cam_rear_link",
            enabled=False, note="后视相机（占位，待标定）",
        ),
        "ext_cab_left": _mk(
            sensor_id="cab_left", parent_frame="base_link", child_frame="cab_left_link",
            enabled=False, note="驾驶室左相机（占位，待标定）",
        ),
        "ext_cab_right": _mk(
            sensor_id="cab_right", parent_frame="base_link", child_frame="cab_right_link",
            enabled=False, note="驾驶室右相机（占位，待标定）",
        ),
        "ext_lidar_front_left": _mk(
            sensor_id="lidar_front_left", parent_frame="base_link", child_frame="lidar_front_left_link",
            enabled=False, note="前左激光雷达（占位，待标定）",
        ),
        "ext_lidar_front_right": _mk(
            sensor_id="lidar_front_right", parent_frame="base_link", child_frame="lidar_front_right_link",
            enabled=False, note="前右激光雷达（占位，待标定）",
        ),
        "ext_lidar_rear_left": _mk(
            sensor_id="lidar_rear_left", parent_frame="base_link", child_frame="lidar_rear_left_link",
            enabled=False, note="后左激光雷达（占位，待标定）",
        ),
    }
    return ExtrinsicsConfig(entries=entries)


def build_default_tilt_compensation_config() -> TiltCompensationConfig:
    """构造与 default_config.yaml Section 9 数值完全一致的 TiltCompensationConfig。"""
    return TiltCompensationConfig(
        alpha=0.98,
        calib_count=50,
        gyro_deadzone_rad_s=0.002,
        auto_calibrate_on_open=True,
        use_relative_subtraction=True,
    )


def build_default_workspace_config() -> WorkspaceConfig:
    """构造与 default_config.yaml Section 10 数值完全一致的 WorkspaceConfig。"""
    return WorkspaceConfig(
        min_ground_z=-0.1,
        min_transit_z=0.3,
        singularity_margin_min_m=0.02,
        singularity_margin_ratio=0.01,
    )


def build_default_trajectory_config() -> TrajectoryConfig:
    """构造与 default_config.yaml Section 11 数值完全一致的 TrajectoryConfig。"""
    return TrajectoryConfig(
        default_strategy="lerp",
        lerp_step_deg=5.0,
        max_speed_deg_s={
            "swing_yaw": 15.0,
            "boom_swing": 20.0,
            "arm_boom": 30.0,
            "bucket_arm": 30.0,
        },
        accel_deg_s2={
            "swing_yaw": 30.0,
            "boom_swing": 40.0,
            "arm_boom": 60.0,
            "bucket_arm": 60.0,
        },
    )


# ==============================================================
# 内置默认配置 dict（当 yaml 模块不可用 + default_config.yaml 无法解析时使用）
#   —— 数值与 config/default_config.yaml 完全等价。
# ==============================================================


BUILTIN_DEFAULT_CONFIG_DICT: Dict[str, Any] = {
    "v15_config_version": "1.0",
    "model_name": "shandong_60FED_default",
    "description": "山东 60FED 挖掘机（v10 标定参数 + v14 URDF 关节映射 + v15 标准控制协议）",
    "joint_mapping": {
        "swing_yaw":  "swing_joint",
        "boom_swing": "boom_joint",
        "arm_boom":   "arm_joint",
        "bucket_arm": "bucket_joint",
    },
    "joint_limits": {
        "swing_yaw":  {"min_deg": -180.0, "max_deg": 180.0, "description": "回转角"},
        "boom_swing": {"min_deg":   -5.0, "max_deg":  55.0, "description": "大臂"},
        "arm_boom":   {"min_deg":    0.0, "max_deg": 130.0, "description": "小臂"},
        "bucket_arm": {"min_deg":  -95.0, "max_deg":  45.0, "description": "铲斗"},
    },
    "link_geometry": {
        "offset_x": 0.25,
        "offset_z": 0.40,
        "L1": 0.35,
        "L2": 0.60,
        "boom_bend_angle_deg": 46.0,
        "L_arm": 0.44,
        "L_bucket": 0.26,
        "sensor_offsets_deg": {"boom": 40.9, "arm": 19.6, "bucket": -56.2},
    },
    "ros_protocol": {
        "node_name": "v15_urdf_controller",
        "joint_topic": "/joint_states",
        "frame_id": "base_link",
        "qos_depth": 10,
        "msg_type": "sensor_msgs/msg/JointState",
        "first_publish_sync_from_feedback": True,
    },
    "standard_poses": {
        "init": {"swing_yaw": 0.0, "boom_swing": 5.0, "arm_boom": 60.0, "bucket_arm": 10.0},
        "home": {"swing_yaw": 0.0, "boom_swing": 0.0, "arm_boom": 120.0, "bucket_arm": 30.0},
        "cycle_transit": {"swing_yaw": 0.0, "boom_swing": 15.0, "arm_boom": 70.0, "bucket_arm": -20.0},
    },
    "motion_defaults": {
        "at_pose_tolerance_deg": 1.0,
        "move_timeout_s": 3.0,
        "bucket_search_range_deg": [-70.0, 10.0],
        "bucket_search_samples": 17,
    },
    "sensors": {
        "tilt_sensors": {
            "tilt_bucket": {"id":"tilt_bucket","type":"tilt","topic":"/excavator/inclinometer_pitch_deg","msg_type":"std_msgs/msg/Float64","frame_id":"bucket_link","qos_depth":10,"enabled":True,"modbus_addr":"0x50","array_index":0,"semantic_joint":"bucket_arm","extra":{}},
            "tilt_arm":    {"id":"tilt_arm","type":"tilt","topic":"/excavator/inclinometer_pitch_deg","msg_type":"std_msgs/msg/Float64","frame_id":"arm_link","qos_depth":10,"enabled":True,"modbus_addr":"0x51","array_index":1,"semantic_joint":"arm_boom","extra":{}},
            "tilt_boom":   {"id":"tilt_boom","type":"tilt","topic":"/excavator/inclinometer_pitch_deg","msg_type":"std_msgs/msg/Float64","frame_id":"boom_link","qos_depth":10,"enabled":True,"modbus_addr":"0x52","array_index":2,"semantic_joint":"boom_swing","extra":{}},
            "tilt_swing":  {"id":"tilt_swing","type":"tilt","topic":"/excavator/inclinometer_pitch_deg","msg_type":"std_msgs/msg/Float64","frame_id":"swing_link","qos_depth":10,"enabled":True,"modbus_addr":"0x53","array_index":3,"semantic_joint":"swing_yaw","extra":{}},
        },
        "lidars": {
            "lidar_single_rear": {"id":"lidar_single_rear","type":"lidar","topic":"/pointcloud","msg_type":"sensor_msgs/msg/PointCloud2","frame_id":"lidar_rear_link","qos_depth":10,"enabled":True,"extra":{}},
            "lidar_front_left":  {"id":"lidar_front_left","type":"lidar","topic":"/lidar/front_left/points","msg_type":"sensor_msgs/msg/PointCloud2","frame_id":"lidar_front_left_link","qos_depth":10,"enabled":False,"extra":{}},
            "lidar_front_right": {"id":"lidar_front_right","type":"lidar","topic":"/lidar/front_right/points","msg_type":"sensor_msgs/msg/PointCloud2","frame_id":"lidar_front_right_link","qos_depth":10,"enabled":False,"extra":{}},
            "lidar_rear_left":   {"id":"lidar_rear_left","type":"lidar","topic":"/lidar/rear_left/points","msg_type":"sensor_msgs/msg/PointCloud2","frame_id":"lidar_rear_left_link","qos_depth":10,"enabled":False,"extra":{}},
            "lidar_rear_right":  {"id":"lidar_rear_right","type":"lidar","topic":"/lidar/rear_right/points","msg_type":"sensor_msgs/msg/PointCloud2","frame_id":"lidar_rear_right_link","qos_depth":10,"enabled":False,"extra":{}},
        },
        "cameras": {
            "cam_front_main": {"id":"cam_front_main","type":"camera","topic":"/sensors/camera/front_main/image_raw","msg_type":"sensor_msgs/msg/Image","frame_id":"cam_front_main_link","qos_depth":10,"enabled":True,"info_topic":"/sensors/camera/front_main/camera_info","extra":{}},
            "cam_left":       {"id":"cam_left","type":"camera","topic":"/sensors/camera/left/image_raw","msg_type":"sensor_msgs/msg/Image","frame_id":"cam_left_link","qos_depth":10,"enabled":True,"info_topic":"","extra":{}},
            "cam_right":      {"id":"cam_right","type":"camera","topic":"/sensors/camera/right/image_raw","msg_type":"sensor_msgs/msg/Image","frame_id":"cam_right_link","qos_depth":10,"enabled":True,"info_topic":"","extra":{}},
            "cam_rear":       {"id":"cam_rear","type":"camera","topic":"/sensors/camera/rear/image_raw","msg_type":"sensor_msgs/msg/Image","frame_id":"cam_rear_link","qos_depth":10,"enabled":False,"info_topic":"","extra":{}},
            "cab_left":       {"id":"cab_left","type":"camera","topic":"/sensors/camera/cab_left/image_raw","msg_type":"sensor_msgs/msg/Image","frame_id":"cab_left_link","qos_depth":10,"enabled":False,"info_topic":"","extra":{}},
            "cab_right":      {"id":"cab_right","type":"camera","topic":"/sensors/camera/cab_right/image_raw","msg_type":"sensor_msgs/msg/Image","frame_id":"cab_right_link","qos_depth":10,"enabled":False,"info_topic":"","extra":{}},
        },
    },
    "extrinsics": {
        "ext_lidar_single_rear": {"sensor_id":"lidar_single_rear","parent_frame":"base_link","child_frame":"lidar_rear_link","x_m":-0.5500,"y_m":-0.2000,"z_m":1.2712,"yaw_rad":0.0532,"pitch_rad":0.0349,"roll_rad":3.0316,"enabled":True,"note":"后向单线束激光雷达（标定值）"},
        "ext_cam_front_main":    {"sensor_id":"cam_front_main","parent_frame":"base_link","child_frame":"cam_front_main_link","x_m":0.4539,"y_m":0.1532,"z_m":1.5246,"yaw_rad":0.0,"pitch_rad":1.1519,"roll_rad":0.0,"enabled":True,"note":"前视主相机（标定值）"},
        "ext_cam_left":          {"sensor_id":"cam_left","parent_frame":"base_link","child_frame":"cam_left_link","x_m":0.4239,"y_m":-0.1768,"z_m":1.4246,"yaw_rad":0.0,"pitch_rad":0.9250,"roll_rad":0.0,"enabled":True,"note":"左视相机（标定值）"},
        "ext_cam_right":         {"sensor_id":"cam_right","parent_frame":"base_link","child_frame":"cam_right_link","x_m":0.0,"y_m":0.0,"z_m":0.0,"yaw_rad":0.0,"pitch_rad":0.0,"roll_rad":0.0,"enabled":False,"note":"右视相机（占位，待标定）"},
        "ext_cam_rear":          {"sensor_id":"cam_rear","parent_frame":"base_link","child_frame":"cam_rear_link","x_m":0.0,"y_m":0.0,"z_m":0.0,"yaw_rad":0.0,"pitch_rad":0.0,"roll_rad":0.0,"enabled":False,"note":"后视相机（占位，待标定）"},
        "ext_cab_left":          {"sensor_id":"cab_left","parent_frame":"base_link","child_frame":"cab_left_link","x_m":0.0,"y_m":0.0,"z_m":0.0,"yaw_rad":0.0,"pitch_rad":0.0,"roll_rad":0.0,"enabled":False,"note":"驾驶室左相机（占位，待标定）"},
        "ext_cab_right":         {"sensor_id":"cab_right","parent_frame":"base_link","child_frame":"cab_right_link","x_m":0.0,"y_m":0.0,"z_m":0.0,"yaw_rad":0.0,"pitch_rad":0.0,"roll_rad":0.0,"enabled":False,"note":"驾驶室右相机（占位，待标定）"},
        "ext_lidar_front_left":  {"sensor_id":"lidar_front_left","parent_frame":"base_link","child_frame":"lidar_front_left_link","x_m":0.0,"y_m":0.0,"z_m":0.0,"yaw_rad":0.0,"pitch_rad":0.0,"roll_rad":0.0,"enabled":False,"note":"前左激光雷达（占位，待标定）"},
        "ext_lidar_front_right": {"sensor_id":"lidar_front_right","parent_frame":"base_link","child_frame":"lidar_front_right_link","x_m":0.0,"y_m":0.0,"z_m":0.0,"yaw_rad":0.0,"pitch_rad":0.0,"roll_rad":0.0,"enabled":False,"note":"前右激光雷达（占位，待标定）"},
        "ext_lidar_rear_left":   {"sensor_id":"lidar_rear_left","parent_frame":"base_link","child_frame":"lidar_rear_left_link","x_m":0.0,"y_m":0.0,"z_m":0.0,"yaw_rad":0.0,"pitch_rad":0.0,"roll_rad":0.0,"enabled":False,"note":"后左激光雷达（占位，待标定）"},
    },
    "tilt_compensation": {
        "alpha": 0.98,
        "calib_count": 50,
        "gyro_deadzone_rad_s": 0.002,
        "auto_calibrate_on_open": True,
        "use_relative_subtraction": True,
    },
    "workspace": {
        "min_ground_z": -0.1,
        "min_transit_z": 0.3,
        "singularity_margin_min_m": 0.02,
        "singularity_margin_ratio": 0.01,
    },
    "trajectory": {
        "default_strategy": "lerp",
        "lerp_step_deg": 5.0,
        "max_speed_deg_s": {
            "swing_yaw": 15.0, "boom_swing": 20.0, "arm_boom": 30.0, "bucket_arm": 30.0,
        },
        "accel_deg_s2": {
            "swing_yaw": 30.0, "boom_swing": 40.0, "arm_boom": 60.0, "bucket_arm": 60.0,
        },
    },
}


# ==============================================================
# 3. 顶层入口函数
# ==============================================================


_DEFAULT_CONFIG_CACHE: Optional[V15Config] = None


def load_config(path: Any) -> V15Config:
    """
    统一配置加载入口，支持多种输入类型。

    支持的输入类型：
      1) str: 路径 → 加载 .yaml/.yml 或 .json 文件
         - .yaml / .yml: 需要 PyYAML，无环境时自动寻找同名 .json fallback
         - .json: 标准库自带，零依赖
      2) dict: 直接解析 raw dict（相当于 V15Config.from_dict(path)）
      3) V15Config: 直接原样返回（无操作，兼容上层 isinstance 判断）
    """
    if isinstance(path, V15Config):
        return path
    if isinstance(path, dict):
        return V15Config.from_dict(path)
    if isinstance(path, (str, bytes, os.PathLike)):
        return V15Config.from_yaml_file(path)
    raise TypeError(f"load_config() 不支持的输入类型: {type(path).__name__} (str / dict / V15Config 之一)")


def load_default_config() -> V15Config:
    """
    加载默认配置（单例缓存）。

    优先级：
      1) default_config.yaml（PyYAML 可用）
      2) default_config.json（有 json 同路径 fallback 文件）
      3) 内置 BUILTIN_DEFAULT_CONFIG_DICT（纯 Python dict，零任何依赖 → 真正通用）
    """
    global _DEFAULT_CONFIG_CACHE
    if _DEFAULT_CONFIG_CACHE is not None:
        return _DEFAULT_CONFIG_CACHE

    here = os.path.dirname(os.path.abspath(__file__))
    yaml_path = os.path.join(here, "default_config.yaml")

    # 路径 A/B：尝试 yaml（或 yaml 不可用时自动找同目录 .json）
    try:
        _DEFAULT_CONFIG_CACHE = load_config(yaml_path)
        return _DEFAULT_CONFIG_CACHE
    except (ImportError, FileNotFoundError, ValueError):
        # 路径 C：内置 Python dict 兜底（保证在任何无网络/无依赖环境都能跑起来）
        _DEFAULT_CONFIG_CACHE = V15Config.from_dict(BUILTIN_DEFAULT_CONFIG_DICT)
        return _DEFAULT_CONFIG_CACHE



__all__ = [
    "V15Config",
    "JointMappingConfig",
    "JointLimitsConfig",
    "SingleJointLimit",
    "LinkGeometryConfig",
    "RosProtocolConfig",
    "MotionDefaultsConfig",
    "StandardPosesConfig",
    "SingleSensorConfig",
    "SensorsConfig",
    "ExtrinsicsEntry",
    "ExtrinsicsConfig",
    "TiltCompensationConfig",
    "WorkspaceConfig",
    "TrajectoryConfig",
    "build_default_sensors_config",
    "build_default_extrinsics_config",
    "build_default_tilt_compensation_config",
    "build_default_workspace_config",
    "build_default_trajectory_config",
    "BUILTIN_DEFAULT_CONFIG_DICT",
    "load_config",
    "load_default_config",
]
