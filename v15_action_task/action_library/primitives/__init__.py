"""
action_library.primitives
-------------------------
原子动作：单关节移动 + 铲斗开关。

不依赖 composites / tasks，可以被 composites / tasks 直接调用。
"""

from .joint_motion import (
    move_joint_step,
    move_joint_steps_independent,
)

from .bucket_control import (
    BUCKET_CLOSED_DEG,
    BUCKET_FULL_OPEN_FOR_DUMP_DEG,
    BUCKET_HALF_OPEN_FOR_DIG_DEG,
    close_bucket,
    full_open_bucket_for_dump,
    half_open_bucket_for_dig,
)

from .semantic_moves import (
    swing_move,
    boom_move,
    arm_move,
    bucket_move,
    swing_move_to,
    boom_lift,
    arm_extend,
    bucket_tilt_to,
)

__all__ = [
    # joint_motion
    "move_joint_step",
    "move_joint_steps_independent",
    # bucket_control
    "BUCKET_CLOSED_DEG",
    "BUCKET_FULL_OPEN_FOR_DUMP_DEG",
    "BUCKET_HALF_OPEN_FOR_DIG_DEG",
    "close_bucket",
    "full_open_bucket_for_dump",
    "half_open_bucket_for_dig",
    # semantic_moves
    "swing_move",
    "boom_move",
    "arm_move",
    "bucket_move",
    "swing_move_to",
    "boom_lift",
    "arm_extend",
    "bucket_tilt_to",
]
