"""
v15 动作库（action_library）
=============================

分层结构：

  utils        ← 纯工具：限位裁剪、StepBuilder、IK 封装
   ↑
  primitives   ← 原语：单关节移动、铲斗开/闭
   ↑
  composites   ← 组合：标准姿态、挖掘序列、卸料序列、回转对齐
   ↑
  tasks        ← 任务：单点 dig→dump、多点循环

顶层接口（从此模块直接 import）：
  from action_library import (
      JOINT_LIMITS, clamp_pose, StepBuilder,
      move_joint_step, close_bucket, BUCKET_CLOSED_DEG,
      INIT_POSE, move_to_init_pose, dig_entry_sequence,
      build_single_dig_dump_task, build_multi_dig_task,
  )
"""

from .utils import (
    # joint_limits
    JOINT_LIMITS,
    JointLimit,
    VALID_JOINT_NAMES,
    clamp_angle,
    clamp_pose,
    check_pose_limits,
    default_pose_deg,
    # step_builder
    DEFAULT_RAMP_DOWN_S,
    DEFAULT_RAMP_UP_S,
    DEFAULT_SPEED_DEG_S,
    DEFAULT_SWING_SPEED_DEG_S,
    DEFAULT_TOLERANCE_DEG,
    StepBuilder,
    # ik_wrapper
    CylindricalPoint,
    IKSolver,
    PoseSolution,
)

from .primitives import (
    BUCKET_CLOSED_DEG,
    BUCKET_FULL_OPEN_FOR_DUMP_DEG,
    BUCKET_HALF_OPEN_FOR_DIG_DEG,
    close_bucket,
    full_open_bucket_for_dump,
    half_open_bucket_for_dig,
    move_joint_step,
    move_joint_steps_independent,
)

from .composites import (
    CYCLE_TRANSIT_POSE,
    HOME_POSE,
    INIT_POSE,
    align_swing,
    align_swing_to_point,
    dig_entry_sequence,
    dump_release_sequence,
    move_to_cycle_transit_pose,
    move_to_home_pose,
    move_to_init_pose,
)

from .tasks import (
    build_multi_dig_cycles,
    build_multi_dig_task,
    build_single_dig_dump_script,
    build_single_dig_dump_task,
)

__all__ = [
    # utils / joint_limits
    "JOINT_LIMITS",
    "JointLimit",
    "VALID_JOINT_NAMES",
    "clamp_angle",
    "clamp_pose",
    "check_pose_limits",
    "default_pose_deg",
    # utils / step_builder
    "DEFAULT_RAMP_DOWN_S",
    "DEFAULT_RAMP_UP_S",
    "DEFAULT_SPEED_DEG_S",
    "DEFAULT_SWING_SPEED_DEG_S",
    "DEFAULT_TOLERANCE_DEG",
    "StepBuilder",
    # utils / ik_wrapper
    "CylindricalPoint",
    "IKSolver",
    "PoseSolution",
    # primitives
    "BUCKET_CLOSED_DEG",
    "BUCKET_FULL_OPEN_FOR_DUMP_DEG",
    "BUCKET_HALF_OPEN_FOR_DIG_DEG",
    "close_bucket",
    "full_open_bucket_for_dump",
    "half_open_bucket_for_dig",
    "move_joint_step",
    "move_joint_steps_independent",
    # composites
    "CYCLE_TRANSIT_POSE",
    "HOME_POSE",
    "INIT_POSE",
    "align_swing",
    "align_swing_to_point",
    "dig_entry_sequence",
    "dump_release_sequence",
    "move_to_cycle_transit_pose",
    "move_to_home_pose",
    "move_to_init_pose",
    # tasks
    "build_multi_dig_cycles",
    "build_multi_dig_task",
    "build_single_dig_dump_script",
    "build_single_dig_dump_task",
]
