"""
action_library.composites
------------------------
组合动作层：把多个原语拼起来，形成“标准姿态”、“挖掘序列”、“卸料序列”、“回转动

不依赖 tasks，可以被 tasks 直接调用。
"""

from .standard_poses import (
    CYCLE_TRANSIT_POSE,
    HOME_POSE,
    INIT_POSE,
    move_to_cycle_transit_pose,
    move_to_home_pose,
    move_to_init_pose,
)

from .arm_motion import (
    dig_entry_sequence,
    dump_release_sequence,
)

from .swing_motion import (
    align_swing,
    align_swing_to_point,
)

__all__ = [
    # standard_poses
    "CYCLE_TRANSIT_POSE",
    "HOME_POSE",
    "INIT_POSE",
    "move_to_cycle_transit_pose",
    "move_to_home_pose",
    "move_to_init_pose",
    # arm_motion
    "dig_entry_sequence",
    "dump_release_sequence",
    # swing_motion
    "align_swing",
    "align_swing_to_point",
]
