from __future__ import annotations

import threading
import time
from typing import Any, Dict, Optional

from .base import CameraReading, LidarReading, TiltCompensator, TiltReading


_ROS_AVAILABLE = False
_ROS_IMPORT_ERROR: Optional[str] = None
_rclpy = None
_Node = None
_QoSProfile = None
_Subscription = None
_Float64MultiArray = None
_PointCloud2 = None
_Image = None
_CameraInfo = None

try:
    import rclpy as _rclpy
    from rclpy.node import Node as _NodeClass
    from rclpy.qos import QoSProfile as _QoSProfileClass
    from rclpy.subscription import Subscription as _SubscriptionClass
    from std_msgs.msg import Float64MultiArray as _Float64MultiArrayClass
    from sensor_msgs.msg import PointCloud2 as _PointCloud2Class
    from sensor_msgs.msg import Image as _ImageClass
    from sensor_msgs.msg import CameraInfo as _CameraInfoClass

    _rclpy_OK = True
    try:
        _test = _rclpy.ok()
    except Exception:
        _rclpy_OK = False

    _ROS_AVAILABLE = _rclpy_OK
    _rclpy = _rclpy if _rclpy_OK else None
    _Node = _NodeClass if _rclpy_OK else None
    _QoSProfile = _QoSProfileClass if _rclpy_OK else None
    _Subscription = _SubscriptionClass if _rclpy_OK else None
    _Float64MultiArray = _Float64MultiArrayClass if _rclpy_OK else None
    _PointCloud2 = _PointCloud2Class if _rclpy_OK else None
    _Image = _ImageClass if _rclpy_OK else None
    _CameraInfo = _CameraInfoClass if _rclpy_OK else None
    if not _rclpy_OK:
        _ROS_IMPORT_ERROR = "rclpy.ok() raised Exception (ROS 2 not initialized)"
except ImportError as _e:
    _ROS_AVAILABLE = False
    _ROS_IMPORT_ERROR = f"ImportError: {_e}"
except Exception as _e:
    _ROS_AVAILABLE = False
    _ROS_IMPORT_ERROR = f"Exception: {type(_e).__name__}: {_e}"


TILT_PARENT_CHAIN: Dict[str, Optional[str]] = {
    "tilt_bucket": "tilt_arm",
    "tilt_arm": "tilt_boom",
    "tilt_boom": "tilt_swing",
    "tilt_swing": None,
}


class SensorManager:
    def __init__(self, cfg: Any, use_ros_backend: bool) -> None:
        self._cfg = cfg
        self._sensors_cfg = cfg.sensors
        self._tilt_comp_cfg = cfg.tilt_compensation
        self._use_ros = bool(use_ros_backend) and _ROS_AVAILABLE
        self._lock = threading.Lock()
        self._tilt_cache: Dict[str, TiltReading] = {}
        self._lidar_cache: Dict[str, LidarReading] = {}
        self._camera_cache: Dict[str, CameraReading] = {}
        self._tilt_compensators: Dict[str, TiltCompensator] = {}
        self._opened = False
        self._ros_node: Any = None
        self._ros_subs: list = []
        self._ros_init_by_us = False
        self._ros_error_reason: str = ""
        if self._use_ros and not _ROS_AVAILABLE:
            self._use_ros = False
            self._ros_error_reason = _ROS_IMPORT_ERROR or "ROS 2 unavailable"
        if not self._use_ros:
            self._ros_error_reason = _ROS_IMPORT_ERROR or "not using ROS backend"

    @classmethod
    def from_config(
        cls,
        cfg: Any,
        ros_environment: str = "auto",
    ) -> "SensorManager":
        env = str(ros_environment).lower()
        if env == "force_ros":
            use_ros = True
        elif env == "force_mock":
            use_ros = False
        else:
            use_ros = _ROS_AVAILABLE
        return cls(cfg, use_ros)

    def _init_tilt_compensators(self) -> None:
        for sid in self._sensors_cfg.list_tilt_ids():
            self._tilt_compensators[sid] = TiltCompensator(self._tilt_comp_cfg)

    def _compute_relative_angles(self) -> None:
        if not self._tilt_comp_cfg.use_relative_subtraction:
            return
        for child_id, parent_id in TILT_PARENT_CHAIN.items():
            child_rd = self._tilt_cache.get(child_id)
            if child_rd is None:
                continue
            if parent_id is None:
                child_rd.relative_angle_deg = (
                    child_rd.compensated_angle_deg
                    if child_rd.compensated_angle_deg is not None
                    else None
                )
                continue
            parent_rd = self._tilt_cache.get(parent_id)
            if (
                parent_rd is None
                or child_rd.compensated_angle_deg is None
                or parent_rd.compensated_angle_deg is None
            ):
                child_rd.relative_angle_deg = None
                continue
            child_rd.relative_angle_deg = (
                child_rd.compensated_angle_deg - parent_rd.compensated_angle_deg
            )

    def open(self) -> None:
        if self._opened:
            return
        self._init_tilt_compensators()
        if self._use_ros:
            try:
                self._open_ros_backend()
            except Exception as e:
                self._use_ros = False
                self._ros_error_reason = f"ROS open failed: {type(e).__name__}: {e}"
        self._opened = True

    def _open_ros_backend(self) -> None:
        if not _ROS_AVAILABLE or _rclpy is None or _Node is None:
            raise RuntimeError("ROS 2 not available")
        try:
            if not _rclpy.ok():
                try:
                    _rclpy.init(args=None)
                    self._ros_init_by_us = True
                except Exception:
                    pass
            node_name = "v15_sensor_manager"
            self._ros_node = _Node(node_name)
            self._create_ros_subscriptions()
        except Exception as e:
            raise e

    def _create_ros_subscriptions(self) -> None:
        if self._ros_node is None or _QoSProfile is None:
            return
        node = self._ros_node
        default_qos = _QoSProfile(depth=10)
        tilt_topics_seen: Dict[str, bool] = {}
        for sid, sensor in self._sensors_cfg.tilt_sensors.items():
            topic = sensor.topic
            if topic in tilt_topics_seen:
                continue
            tilt_topics_seen[topic] = True
            try:
                qos = sensor.qos_depth
                if qos and qos > 0:
                    sub_qos = _QoSProfile(depth=int(qos))
                else:
                    sub_qos = default_qos
                sub = node.create_subscription(
                    _Float64MultiArray,
                    topic,
                    self._make_tilt_callback(topic),
                    sub_qos,
                )
                self._ros_subs.append(sub)
            except Exception:
                pass
        for sid, sensor in self._sensors_cfg.lidars.items():
            try:
                qos = sensor.qos_depth
                if qos and qos > 0:
                    sub_qos = _QoSProfile(depth=int(qos))
                else:
                    sub_qos = default_qos
                sub = node.create_subscription(
                    _PointCloud2,
                    sensor.topic,
                    self._make_lidar_callback(sid),
                    sub_qos,
                )
                self._ros_subs.append(sub)
            except Exception:
                pass
        for sid, sensor in self._sensors_cfg.cameras.items():
            try:
                qos = sensor.qos_depth
                if qos and qos > 0:
                    sub_qos = _QoSProfile(depth=int(qos))
                else:
                    sub_qos = default_qos
                sub = node.create_subscription(
                    _Image,
                    sensor.topic,
                    self._make_camera_callback(sid),
                    sub_qos,
                )
                self._ros_subs.append(sub)
            except Exception:
                pass

    def _make_tilt_callback(self, topic: str):
        def _cb(msg: Any) -> None:
            try:
                ts_s = float(time.time())
                try:
                    header_stamp = getattr(msg, "header", None)
                    if header_stamp is not None:
                        stamp = getattr(header_stamp, "stamp", None)
                        if stamp is not None:
                            sec = float(getattr(stamp, "sec", 0))
                            nsec = float(getattr(stamp, "nanosec", 0))
                            ts_s = sec + nsec / 1e9
                except Exception:
                    pass
                data = list(getattr(msg, "data", []) or [])
                with self._lock:
                    for sid, sensor in self._sensors_cfg.tilt_sensors.items():
                        if sensor.topic != topic:
                            continue
                        idx = sensor.array_index
                        if idx is None or idx < 0 or idx >= len(data):
                            continue
                        raw_val = float(data[idx])
                        comp = self._tilt_compensators.get(sid)
                        compensated: Optional[float] = None
                        done = False
                        remaining = 0
                        if comp is not None:
                            try:
                                compensated, done, remaining = comp.update(raw_val)
                            except Exception:
                                compensated = raw_val
                                done = False
                                remaining = 0
                        reading = TiltReading(
                            id=sid,
                            ts_s=ts_s,
                            is_valid=True,
                            raw_angle_deg=raw_val,
                            compensated_angle_deg=compensated,
                            calibration_done=done,
                            calibration_remaining=max(0, int(remaining)),
                        )
                        self._tilt_cache[sid] = reading
                    self._compute_relative_angles()
            except Exception:
                pass
        return _cb

    def _make_lidar_callback(self, sid: str):
        def _cb(msg: Any) -> None:
            try:
                ts_s = float(time.time())
                width = 0
                height = 0
                try:
                    header_stamp = getattr(msg, "header", None)
                    if header_stamp is not None:
                        stamp = getattr(header_stamp, "stamp", None)
                        if stamp is not None:
                            sec = float(getattr(stamp, "sec", 0))
                            nsec = float(getattr(stamp, "nanosec", 0))
                            ts_s = sec + nsec / 1e9
                    width = int(getattr(msg, "width", 0) or 0)
                    height = int(getattr(msg, "height", 0) or 0)
                except Exception:
                    pass
                points_count = width * height
                reading = LidarReading(
                    id=sid,
                    ts_s=ts_s,
                    is_valid=True,
                    points_count=points_count,
                    width_or_none=width,
                    height_or_none=height,
                    raw_ref_or_none=msg,
                )
                with self._lock:
                    self._lidar_cache[sid] = reading
            except Exception:
                pass
        return _cb

    def _make_camera_callback(self, sid: str):
        def _cb(msg: Any) -> None:
            try:
                ts_s = float(time.time())
                width = 0
                height = 0
                encoding = ""
                seq = None
                try:
                    header_stamp = getattr(msg, "header", None)
                    if header_stamp is not None:
                        stamp = getattr(header_stamp, "stamp", None)
                        if stamp is not None:
                            sec = float(getattr(stamp, "sec", 0))
                            nsec = float(getattr(stamp, "nanosec", 0))
                            ts_s = sec + nsec / 1e9
                        seq = int(getattr(header_stamp, "seq", 0) or 0)
                    width = int(getattr(msg, "width", 0) or 0)
                    height = int(getattr(msg, "height", 0) or 0)
                    encoding = str(getattr(msg, "encoding", "") or "")
                except Exception:
                    pass
                reading = CameraReading(
                    id=sid,
                    ts_s=ts_s,
                    is_valid=True,
                    width_px=width,
                    height_px=height,
                    encoding=encoding,
                    seq_or_none=seq,
                    raw_ref_or_none=msg,
                )
                with self._lock:
                    self._camera_cache[sid] = reading
            except Exception:
                pass
        return _cb

    def close(self) -> None:
        if not self._opened:
            return
        self._opened = False
        if self._ros_node is not None:
            try:
                for sub in list(self._ros_subs):
                    try:
                        self._ros_node.destroy_subscription(sub)
                    except Exception:
                        pass
                self._ros_subs = []
            except Exception:
                pass
            try:
                self._ros_node.destroy_node()
            except Exception:
                pass
            self._ros_node = None
            if self._ros_init_by_us and _rclpy is not None:
                try:
                    if _rclpy.ok():
                        _rclpy.shutdown()
                except Exception:
                    pass
                self._ros_init_by_us = False

    def __enter__(self) -> "SensorManager":
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
        return self._use_ros

    def ros_backend_error_reason_if_any(self) -> str:
        if self._use_ros:
            return ""
        return self._ros_error_reason
