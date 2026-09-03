"""
ROS 2 (rclpy) 后端适配器 —— 与 v14 协议 100% 对齐。

行为与 shandong/v14_urdf/ros_joint_bridge.py 保持一致：
  - 发布: /joint_states (sensor_msgs/JointState)
  - 订阅: /joint_states（作为角度反馈）
  - msg.name 顺序: ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"]
  - frame_id: "base_link"
  - 首次 publish 时如已收到反馈，则以反馈角度作为初始值，再叠加用户修改
"""

from __future__ import annotations

import threading
import time
from typing import Any, Dict, Optional

from .types import (
    DEFAULT_FRAME_ID,
    DEFAULT_JOINT_TOPIC,
    SEMANTIC_TO_URDF,
    SEMANTIC_JOINT_ORDER,
    URDF_JOINT_ORDER,
    URDF_TO_SEMANTIC,
    deg_to_rad,
    rad_to_deg,
    default_pose_deg,
)
from .adapter_base import ControlAdapter


class RosV14Adapter(ControlAdapter):
    """ROS 2 适配器，兼容 v14 /joint_states 话题协议。"""

    @classmethod
    def from_config(cls, cfg: Any, **override_kwargs) -> "RosV14Adapter":
        """
        从 v15 YAML config 构造 RosV14Adapter。

        ```
        from v15_action_task.config import load_default_config
        cfg = load_default_config()
        adapter = RosV14Adapter.from_config(cfg)
        ```
        """
        if hasattr(cfg, "ros"):
            ros_cfg = cfg.ros
        else:
            ros_cfg = cfg
        kwargs: Dict[str, Any] = {}
        if hasattr(ros_cfg, "to_adapter_kwargs"):
            kwargs = dict(ros_cfg.to_adapter_kwargs())
        else:
            kwargs = {
                "node_name": str(getattr(ros_cfg, "node_name", "v15_urdf_controller")),
                "topic": str(getattr(ros_cfg, "joint_topic", "/joint_states")),
                "qos_depth": int(getattr(ros_cfg, "qos_depth", 10)),
                "default_frame_id": str(getattr(ros_cfg, "frame_id", "base_link")),
                "first_publish_sync": bool(getattr(ros_cfg, "first_publish_sync_from_feedback", True)),
            }
        # 应用 cfg.mapping 到全局常量（如果用户传了完整 V15Config）
        if hasattr(cfg, "mapping"):
            try:
                from .types import apply_joint_mapping_config
                apply_joint_mapping_config(cfg.mapping)
            except Exception:
                pass
        kwargs.update(override_kwargs)
        return cls(**kwargs)

    def __init__(
        self,
        node_name: str = "v15_urdf_controller",
        topic: str = DEFAULT_JOINT_TOPIC,
        qos_depth: int = 10,
        default_frame_id: str = DEFAULT_FRAME_ID,
        first_publish_sync: bool = True,
    ):
        self._node_name = node_name
        self._topic = topic
        self._qos_depth = qos_depth
        self._default_frame_id = default_frame_id
        self._first_publish_sync = first_publish_sync

        # 以下属性在 open() 后才有效
        self._rclpy = None
        self._node = None
        self._JointState = None
        self._pub = None
        self._sub = None
        self._spin_thread: Optional[threading.Thread] = None
        self._closing = threading.Event()

        # 线程安全的反馈缓存
        self._lock = threading.Lock()
        self._last_msg = None
        self._last_ts: Optional[float] = None

        # 发布状态机：首次发布前若有反馈，先同步到 cmd_deg
        self._cmd_initialized = False
        self._cmd_deg: Dict[str, float] = default_pose_deg()

    # ── 生命周期 ────────────────────────────────────────────────────

    def open(self) -> None:
        if self._rclpy is not None:
            return  # 已经打开过
        try:
            import rclpy  # type: ignore
            from rclpy.node import Node  # type: ignore
            from sensor_msgs.msg import JointState  # type: ignore
        except Exception as e:
            raise ImportError(
                "rclpy / sensor_msgs 不可用，无法启动 RosV14Adapter。"
                "请先 source ROS 环境 (source /opt/ros/humble/setup.bash)，"
                "或者改用 MockAdapter 在本地调试。"
            ) from e

        self._rclpy = rclpy
        self._JointState = JointState

        if not rclpy.ok():
            rclpy.init()
        self._node = Node(self._node_name)
        self._pub = self._node.create_publisher(JointState, self._topic, self._qos_depth)
        self._sub = self._node.create_subscription(
            JointState, self._topic, self._on_joint_state, self._qos_depth
        )

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def close(self) -> None:
        try:
            self._closing.set()
        except Exception:
            pass
        rclpy = self._rclpy
        if rclpy is not None:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
        try:
            if self._spin_thread and self._spin_thread.is_alive():
                self._spin_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass

    # ── 内部线程 ────────────────────────────────────────────────────

    def _spin(self) -> None:
        rclpy = self._rclpy
        node = self._node
        if rclpy is None or node is None:
            return
        try:
            while rclpy.ok() and not self._closing.is_set():
                rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:
            return

    def _on_joint_state(self, msg) -> None:
        with self._lock:
            self._last_msg = msg
            self._last_ts = time.time()

    # ── ControlAdapter 接口 ─────────────────────────────────────────

    def get_current_pose_deg(self) -> Optional[Dict[str, float]]:
        with self._lock:
            msg = self._last_msg
        if msg is None:
            return None
        name_to_rad: Dict[str, float] = {}
        for idx, name in enumerate(list(msg.name)):
            try:
                name_to_rad[name] = float(msg.position[idx])
            except Exception:
                pass
        pose = default_pose_deg()
        for urdf_name, semantic in URDF_TO_SEMANTIC.items():
            if urdf_name in name_to_rad:
                pose[semantic] = rad_to_deg(name_to_rad[urdf_name])
        return pose

    def get_last_update_ts(self) -> Optional[float]:
        with self._lock:
            return self._last_ts

    def publish_pose_deg(self, pose_deg: Dict[str, float], frame_id: Optional[str] = None) -> bool:
        if self._rclpy is None or self._JointState is None or self._pub is None:
            return False
        if not self._rclpy.ok():
            return False
        if frame_id is None:
            frame_id = self._default_frame_id

        # 1) 首次发布：如果已经有反馈，则以反馈值为初始 cmd，保证不动的关节保持原位
        if self._first_publish_sync and not self._cmd_initialized:
            feedback = self.get_current_pose_deg()
            if feedback is not None:
                with self._lock:
                    if not self._cmd_initialized:
                        for k in SEMANTIC_JOINT_ORDER:
                            self._cmd_deg[k] = float(feedback[k])
                        self._cmd_initialized = True

        # 2) 叠加用户传入的目标角
        with self._lock:
            for k, v in pose_deg.items():
                if k in self._cmd_deg and v is not None:
                    self._cmd_deg[k] = float(v)
            cmd = dict(self._cmd_deg)

        # 3) 组装 JointState（URDF 名 + 弧度）
        msg = self._JointState()
        now = self._node.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = frame_id
        msg.name = list(URDF_JOINT_ORDER)
        msg.position = [deg_to_rad(cmd[s]) for s in SEMANTIC_JOINT_ORDER]
        try:
            self._pub.publish(msg)
            return True
        except Exception:
            return False
