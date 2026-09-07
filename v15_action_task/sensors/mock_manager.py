from __future__ import annotations

import threading
import time
from typing import Any, Dict, Optional

from .base import CameraReading, LidarReading, TiltReading


class MockManager:
    def __init__(self, cfg: Any) -> None:
        self._cfg = cfg
        self._sensors_cfg = cfg.sensors
        self._lock = threading.Lock()
        self._tilt_cache: Dict[str, TiltReading] = {}
        self._lidar_cache: Dict[str, LidarReading] = {}
        self._camera_cache: Dict[str, CameraReading] = {}
        self._opened = False

    @classmethod
    def from_config(
        cls,
        cfg: Any,
        ros_environment: str = "auto",
    ) -> "MockManager":
        return cls(cfg)

    def open(self) -> None:
        self._opened = True

    def close(self) -> None:
        self._opened = False

    def __enter__(self) -> "MockManager":
        self.open()
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.close()

    def get_tilt(self, sid: str) -> Optional[TiltReading]:
        with self._lock:
            return self._tilt_cache.get(sid)

    def all_tilt(self) -> Dict[str, TiltReading]:
        with self._lock:
            return dict(self._tilt_cache)

    def get_lidar(self, sid: str) -> Optional[LidarReading]:
        with self._lock:
            return self._lidar_cache.get(sid)

    def all_lidar(self) -> Dict[str, LidarReading]:
        with self._lock:
            return dict(self._lidar_cache)

    def get_camera(self, sid: str) -> Optional[CameraReading]:
        with self._lock:
            return self._camera_cache.get(sid)

    def all_camera(self) -> Dict[str, CameraReading]:
        with self._lock:
            return dict(self._camera_cache)

    def list_tilt_ids(self) -> list:
        return list(self._sensors_cfg.list_tilt_ids())

    def list_lidar_ids(self) -> list:
        return list(self._sensors_cfg.list_lidar_ids())

    def list_camera_ids(self) -> list:
        return list(self._sensors_cfg.list_camera_ids())

    def is_ros_backend(self) -> bool:
        return False

    def ros_backend_error_reason_if_any(self) -> str:
        return "force_mock backend"

    def _inject_reading(self, sid: str, reading: Any) -> None:
        with self._lock:
            if isinstance(reading, TiltReading):
                self._tilt_cache[sid] = reading
            elif isinstance(reading, LidarReading):
                self._lidar_cache[sid] = reading
            elif isinstance(reading, CameraReading):
                self._camera_cache[sid] = reading
