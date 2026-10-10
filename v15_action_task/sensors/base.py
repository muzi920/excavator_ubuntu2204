from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple


@dataclass
class TiltReading:
    id: str
    ts_s: float
    is_valid: bool
    raw_angle_deg: Optional[float] = None
    compensated_angle_deg: Optional[float] = None
    temperature_c_or_none: Optional[float] = None
    roll_deg_or_none: Optional[float] = None
    pitch_deg_or_none: Optional[float] = None
    calibration_done: bool = False
    calibration_remaining: int = 0
    relative_angle_deg: Optional[float] = None


@dataclass
class LidarReading:
    id: str
    ts_s: float
    is_valid: bool
    points_count: int = 0
    width_or_none: Optional[int] = None
    height_or_none: Optional[int] = None
    raw_ref_or_none: Optional[Any] = None


@dataclass
class CameraReading:
    id: str
    ts_s: float
    is_valid: bool
    width_px: int = 0
    height_px: int = 0
    encoding: str = ""
    seq_or_none: Optional[int] = None
    raw_ref_or_none: Optional[Any] = None


def _resolve_cfg(cfg: Any) -> Any:
    try:
        from v15_action_task.config import TiltCompensationConfig as _TC
    except ImportError:
        try:
            from config import TiltCompensationConfig as _TC
        except ImportError:
            _TC = None
    return cfg


class TiltCompensator:
    def __init__(self, cfg: Any) -> None:
        self._cfg = _resolve_cfg(cfg)
        self._calib_accum: float = 0.0
        self._calib_seen: int = 0
        self._zero_bias_deg: Optional[float] = None
        self._last_compensated: Optional[float] = None

    def update(
        self,
        raw_angle_deg: float,
        *,
        gyro_rad_s_or_none: Optional[float] = None,
        dt_s: Optional[float] = None,
    ) -> Tuple[float, bool, int]:
        if self._zero_bias_deg is None:
            self._calib_accum += float(raw_angle_deg)
            self._calib_seen += 1
            remaining = self._cfg.calib_count - self._calib_seen
            if self._calib_seen >= self._cfg.calib_count:
                zero_bias = self._calib_accum / self._calib_seen
                self._zero_bias_deg = zero_bias
                done = True
                compensated = float(raw_angle_deg) - zero_bias
                self._last_compensated = compensated
                return (compensated, done, max(0, remaining))
            temp_comp = 0.0
            return (temp_comp, False, max(0, remaining))

        zero_bias = self._zero_bias_deg
        raw_comp = float(raw_angle_deg) - zero_bias

        has_gyro = (
            gyro_rad_s_or_none is not None
            and dt_s is not None
            and self._last_compensated is not None
        )

        if has_gyro:
            gyro_rad = float(gyro_rad_s_or_none)
            dt = float(dt_s)
            if abs(gyro_rad) < self._cfg.gyro_deadzone_rad_s:
                gyro_rad = 0.0
            gyro_deg_s = math.degrees(gyro_rad)
            predicted = self._last_compensated + gyro_deg_s * dt
            alpha = float(self._cfg.alpha)
            compensated = alpha * raw_comp + (1.0 - alpha) * predicted
        else:
            compensated = raw_comp

        self._last_compensated = compensated
        return (compensated, True, 0)
