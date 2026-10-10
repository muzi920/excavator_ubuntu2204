"""
control_core —— v15 独立控制核心
=================================

最小依赖：不依赖 action_library，不依赖 UI。
ROS 依赖在 RosV14Adapter 中惰性导入，无 ROS 时可以用 MockAdapter 调试。

顶层快速导入：
    from control_core import (
        URDFController,
        RosV14Adapter, MockAdapter,
        SEMANTIC_TO_URDF, URDF_JOINT_ORDER,
        deg_to_rad, rad_to_deg, default_pose_deg,
    )
"""

from .types import (
    DEFAULT_FRAME_ID,
    DEFAULT_JOINT_TOPIC,
    SEMANTIC_JOINT_ORDER,
    SEMANTIC_TO_URDF,
    URDF_JOINT_ORDER,
    URDF_TO_SEMANTIC,
    all_semantic_names,
    deg_to_rad,
    default_pose_deg,
    rad_to_deg,
)
from .adapter_base import ControlAdapter
from .mock_adapter import MockAdapter
from .ros_v14_adapter import RosV14Adapter
from .urdf_controller import URDFController

__all__ = [
    # types
    "DEFAULT_FRAME_ID",
    "DEFAULT_JOINT_TOPIC",
    "SEMANTIC_JOINT_ORDER",
    "SEMANTIC_TO_URDF",
    "URDF_JOINT_ORDER",
    "URDF_TO_SEMANTIC",
    "all_semantic_names",
    "deg_to_rad",
    "default_pose_deg",
    "rad_to_deg",
    # adapter
    "ControlAdapter",
    "MockAdapter",
    "RosV14Adapter",
    # controller
    "URDFController",
]
