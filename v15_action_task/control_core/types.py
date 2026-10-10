"""
关节语义常量与单位换算。

这里是 v14 URDF 协议与 v4 语义的绑定关系：

  语义名 (API 层)      <->    URDF 关节名 (JointState.name)
  ─────────────────────────────────────────────────────────
  swing_yaw             <->    swing_joint   (回转)
  boom_swing            <->    boom_joint    (大臂)
  arm_boom              <->    arm_joint     (小臂)
  bucket_arm            <->    bucket_joint  (铲斗)

动作库 (action_library) 中用的是语义名，因此控制核心也接受语义名输入，
再在最底层的发布函数里转换成 URDF 关节名。
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple


# 语义名 → URDF 关节名
SEMANTIC_TO_URDF: Dict[str, str] = {
    "swing_yaw": "swing_joint",
    "boom_swing": "boom_joint",
    "arm_boom": "arm_joint",
    "bucket_arm": "bucket_joint",
}

# URDF 关节名 → 语义名
URDF_TO_SEMANTIC: Dict[str, str] = {v: k for k, v in SEMANTIC_TO_URDF.items()}

# 所有语义关节名（顺序固定，便于 zip 展开）
SEMANTIC_JOINT_ORDER: Tuple[str, ...] = tuple(SEMANTIC_TO_URDF.keys())

# 对应的 URDF 关节顺序（发布时保持和 JointState.name 一致）
URDF_JOINT_ORDER: Tuple[str, ...] = tuple(SEMANTIC_TO_URDF[k] for k in SEMANTIC_JOINT_ORDER)

DEFAULT_FRAME_ID = "base_link"
DEFAULT_JOINT_TOPIC = "/joint_states"


def deg_to_rad(v: float) -> float:
    return float(v) * math.pi / 180.0


def rad_to_deg(v: float) -> float:
    return float(v) * 180.0 / math.pi


def all_semantic_names() -> List[str]:
    return list(SEMANTIC_JOINT_ORDER)


def default_pose_deg() -> Dict[str, float]:
    return {name: 0.0 for name in SEMANTIC_JOINT_ORDER}


# ==============================================================
# 配置文件集成（v15 YAML config 层）
# ==============================================================


def apply_joint_mapping_config(cfg: Any) -> Dict[str, Any]:
    """
    用 V15Config.mapping / JointMappingConfig 替换本模块全局的
    SEMANTIC_TO_URDF / URDF_TO_SEMANTIC / SEMANTIC_JOINT_ORDER / URDF_JOINT_ORDER，
    并同步修改 RosV14Adapter 默认 topic / frame_id / QoS 所依赖的 DEFAULT_* 常量。

    ⚠ 全局副作用：后续所有未显式传入 mapping 的 API 都会用新配置。
       推荐在程序 **最开始** 调用一次，不要在运行中切换。

    返回: 新的常量 dict（同 constants）
    """
    if hasattr(cfg, "to_constants"):
        constants = cfg.to_constants()
    elif isinstance(cfg, dict) and "SEMANTIC_TO_URDF" in cfg:
        constants = dict(cfg)
    elif isinstance(cfg, dict):
        # 传入的是 raw mapping dict {semantic: urdf}
        mapping = {str(k): str(v) for k, v in cfg.items()}
        sem_order = tuple(mapping.keys())
        urdf_order = tuple(mapping[s] for s in sem_order)
        constants = {
            "SEMANTIC_TO_URDF": mapping,
            "URDF_TO_SEMANTIC": {v: k for k, v in mapping.items()},
            "SEMANTIC_JOINT_ORDER": sem_order,
            "URDF_JOINT_ORDER": urdf_order,
        }
    else:
        raise TypeError(f"无法识别的 JointMappingConfig 类型: {type(cfg)}")

    global SEMANTIC_TO_URDF, URDF_TO_SEMANTIC, SEMANTIC_JOINT_ORDER, URDF_JOINT_ORDER
    SEMANTIC_TO_URDF  = constants["SEMANTIC_TO_URDF"]
    URDF_TO_SEMANTIC  = constants["URDF_TO_SEMANTIC"]
    SEMANTIC_JOINT_ORDER = constants["SEMANTIC_JOINT_ORDER"]
    URDF_JOINT_ORDER  = constants["URDF_JOINT_ORDER"]
    return constants


def apply_ros_protocol_constants(cfg: Any) -> Dict[str, Any]:
    """
    覆盖 DEFAULT_FRAME_ID / DEFAULT_JOINT_TOPIC 常量。

    接受：RosProtocolConfig / dict (含 joint_topic / frame_id / qos_depth)
    返回：新的 {DEFAULT_FRAME_ID, DEFAULT_JOINT_TOPIC, DEFAULT_QOS_DEPTH}
    """
    if hasattr(cfg, "joint_topic"):
        topic = str(cfg.joint_topic)
        frame = str(cfg.frame_id)
        qos = int(getattr(cfg, "qos_depth", 10))
    elif isinstance(cfg, dict):
        topic = str(cfg.get("joint_topic", "/joint_states"))
        frame = str(cfg.get("frame_id", "base_link"))
        qos = int(cfg.get("qos_depth", 10))
    else:
        raise TypeError(f"无法识别的 RosProtocolConfig 类型: {type(cfg)}")

    global DEFAULT_FRAME_ID, DEFAULT_JOINT_TOPIC
    DEFAULT_FRAME_ID    = frame
    DEFAULT_JOINT_TOPIC = topic
    # 额外返回 DEFAULT_QOS_DEPTH（调用方传给 RosV14Adapter 构造函数）
    return {
        "DEFAULT_FRAME_ID": frame,
        "DEFAULT_JOINT_TOPIC": topic,
        "DEFAULT_QOS_DEPTH": qos,
    }


__all__ = [
    "SEMANTIC_TO_URDF", "URDF_TO_SEMANTIC",
    "SEMANTIC_JOINT_ORDER", "URDF_JOINT_ORDER",
    "DEFAULT_FRAME_ID", "DEFAULT_JOINT_TOPIC",
    "deg_to_rad", "rad_to_deg",
    "all_semantic_names", "default_pose_deg",
    "apply_joint_mapping_config", "apply_ros_protocol_constants",
]

