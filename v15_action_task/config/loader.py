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
    "BUILTIN_DEFAULT_CONFIG_DICT",
    "load_config",
    "load_default_config",
]
