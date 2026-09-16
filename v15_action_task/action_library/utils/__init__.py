"""
action_library.utils
--------------------
纯工具层：关节限位、Step 构造器、IK 封装。

不依赖 composites / tasks / primitives，可以在最底层被其它层 import。
"""

from .joint_limits import (
    JOINT_LIMITS,
    JointLimit,
    VALID_JOINT_NAMES,
    clamp_angle,
    clamp_pose,
    check_pose_limits,
    default_pose_deg,
)

from .step_builder import (
    DEFAULT_RAMP_DOWN_S,
    DEFAULT_RAMP_UP_S,
    DEFAULT_SPEED_DEG_S,
    DEFAULT_SWING_SPEED_DEG_S,
    DEFAULT_TOLERANCE_DEG,
    StepBuilder,
)

from .ik_wrapper import (
    CylindricalPoint,
    IKSolver,
    PoseSolution,
)

__all__ = [
    # joint_limits
    "JOINT_LIMITS",
    "JointLimit",
    "VALID_JOINT_NAMES",
    "clamp_angle",
    "clamp_pose",
    "check_pose_limits",
    "default_pose_deg",
    # step_builder
    "DEFAULT_RAMP_DOWN_S",
    "DEFAULT_RAMP_UP_S",
    "DEFAULT_SPEED_DEG_S",
    "DEFAULT_SWING_SPEED_DEG_S",
    "DEFAULT_TOLERANCE_DEG",
    "StepBuilder",
    # ik_wrapper
    "CylindricalPoint",
    "IKSolver",
    "PoseSolution",
]
