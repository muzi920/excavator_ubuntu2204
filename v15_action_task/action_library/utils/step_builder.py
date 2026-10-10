"""
JSON Step 构造器。

统一 JSON 剧本中每一步的字段格式，保证和 terminal_stepper.py、replay_json_script.py
读取时兼容：
  - step          : int，步骤编号（从 1 开始）
  - joint         : str，v4 风格关节名（swing_yaw / boom_swing / arm_boom / bucket_arm）
  - target_val    : float，目标角度（单位：度，deg）
  - description   : str，人读得懂的描述
  - ramp_up_s     : float，加速段时间（秒），默认 0.2
  - ramp_down_s   : float，减速段时间（秒），默认 0.2
  - speed_deg_s   : float，运动速度（度/秒），可选
  - tolerance_deg : float，到位容差（度），可选
  - is_init_step  : bool，是否属于初始化段（初始化段不参与循环回退判断）
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional


DEFAULT_RAMP_UP_S = 0.2
DEFAULT_RAMP_DOWN_S = 0.2
DEFAULT_SPEED_DEG_S = 6.0           # boom/arm/bucket 默认速度
DEFAULT_SWING_SPEED_DEG_S = 15.0    # swing 速度更快
DEFAULT_TOLERANCE_DEG = 1.0


class StepBuilder:
    """递增式 Step 构造器，自动管理 step 编号。"""

    def __init__(self, start: int = 1):
        self._counter = start
        self.steps: List[Dict[str, Any]] = []

    @property
    def next_index(self) -> int:
        return self._counter

    def _default_speed_for(self, joint: str) -> float:
        if joint == "swing_yaw":
            return DEFAULT_SWING_SPEED_DEG_S
        return DEFAULT_SPEED_DEG_S

    def build(
        self,
        joint: str,
        target_val_deg: float,
        description: str = "",
        *,
        ramp_up_s: Optional[float] = None,
        ramp_down_s: Optional[float] = None,
        speed_deg_s: Optional[float] = None,
        tolerance_deg: Optional[float] = None,
        is_init_step: bool = False,
    ) -> Dict[str, Any]:
        """
        构造单步 JSON，追加到内部列表，并返回。
        """
        step: Dict[str, Any] = {
            "step": self._counter,
            "joint": joint,
            "target_val": float(target_val_deg),
            "description": description,
            "ramp_up_s": float(ramp_up_s if ramp_up_s is not None else DEFAULT_RAMP_UP_S),
            "ramp_down_s": float(ramp_down_s if ramp_down_s is not None else DEFAULT_RAMP_DOWN_S),
            "speed_deg_s": float(speed_deg_s if speed_deg_s is not None else self._default_speed_for(joint)),
            "tolerance_deg": float(tolerance_deg if tolerance_deg is not None else DEFAULT_TOLERANCE_DEG),
            "is_init_step": bool(is_init_step),
        }
        self.steps.append(step)
        self._counter += 1
        return step

    def extend(self, new_steps: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """把外部生成的 steps 直接拼进来，并按内部计数器重新编号。"""
        appended: List[Dict[str, Any]] = []
        for s in new_steps:
            s2 = dict(s)
            s2["step"] = self._counter
            self.steps.append(s2)
            appended.append(s2)
            self._counter += 1
        return appended
