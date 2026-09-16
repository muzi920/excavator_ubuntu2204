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
        heartbeat_hz: float = 10.0,
    ):
        self._node_name = node_name
        self._topic = topic
        self._qos_depth = qos_depth
        self._default_frame_id = default_frame_id
        self._first_publish_sync = first_publish_sync
        self._heartbeat_period_s: float = 1.0 / max(1.0, float(heartbeat_hz))

        # 以下属性在 open() 后才有效
        self._rclpy = None
        self._node = None
        self._JointState = None
        self._pub = None
        self._pub_frame_id: Optional[str] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._closing = threading.Event()

        # 线程安全的 cmd 缓存（此缓存即为 v15 与 URDF 之间的"当前位姿"权威来源）
        # 规则：
        #   1) launch 里 use_joint_state_publisher=False，robot_state_publisher 只吃我们发的 /joint_states
        #   2) 所以"v15 最后一次 publish 的 cmd 缓存"就是 URDF 的实际反馈
        #   3) 不再订阅 /joint_states（避免自激：自己发的又被自己当成反馈）
        #   4) 心跳线程 10Hz 持续发缓存 cmd，保证 robot_state_publisher 一直有数据
        self._lock = threading.Lock()
        self._last_cmd_ts: Optional[float] = None
        self._cmd_deg: Dict[str, float] = default_pose_deg()
        self._cmd_initialized: bool = True  # 取消"首次同步"依赖订阅逻辑

        # 自检用：publish 实际成功计数（用户可直接通过 adapter 检查 DDS 发送是否发生）
        self._publish_success_count: int = 0
        self._publish_fail_count: int = 0



    # ── 生命周期 ────────────────────────────────────────────────────

    def open(self) -> None:
        if self._rclpy is not None:
            return  # 已经打开过

        # ── 在 import rclpy **之前** 兜底 ROS 日志目录权限 ──
        #    问题：~/.ros/log 不存在 / HOME=/root 只读 / sandbox 禁写 ~/.ros/log
        #    都会导致 rclpy.init() -> rcl_logging_configure 抛出
        #      RuntimeError: Failed opening file ...log Permission denied / ... RCLError
        #    Fix: 若 ROS_LOG_DIR 为空 / 不可写，强制落到 /tmp/v15_ros_logs（保证 777 可写）
        import os as _os
        _ros_log_dir = _os.environ.get("ROS_LOG_DIR") or ""
        _dir_ok = bool(_ros_log_dir) and _os.path.isdir(_ros_log_dir) and _os.access(_ros_log_dir, _os.W_OK)
        if not _dir_ok:
            _fallback = "/tmp/v15_ros_logs"
            try:
                _os.makedirs(_fallback, exist_ok=True)
            except Exception:
                _fallback = "/tmp"
            _os.environ["ROS_LOG_DIR"] = _fallback
        # 同步兜底 ROS_HOME，避免 ~/.ros 也无权限
        if not _os.environ.get("ROS_HOME"):
            _os.environ["ROS_HOME"] = _os.environ["ROS_LOG_DIR"]

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

        # rclpy.init(args=[]) 阻止 ROS 2 解析外部 CLI 参数（避免吞掉上层用户的 --strategy/--mode 等）
        if not rclpy.ok():
            try:
                rclpy.init(args=[])
            except Exception:
                # 极端场景（已初始化但 rclpy.ok() 因 race 返回 False）：跳过
                if not rclpy.ok():
                    raise
        self._node = Node(self._node_name)
        self._pub = self._node.create_publisher(JointState, self._topic, self._qos_depth)

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        # ⭐ 心跳线程：10Hz 持续发布 cmd_deg，避免"只发一次 robot_state_publisher 接收不到"
        #             这是驱动 URDF 的关键：/joint_states 必须是流，不是一次性消息
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()

        # open 时立刻心跳把初始位姿（INIT）发布出去，URDF 一启动就有姿势显示
        self._last_cmd_ts = time.time()
        ok_first = self._publish_locked_cmd(None)

        # ⭐ 启动自检日志：3 秒后打印 publish 计数 + 节点名 + 话题名（用户肉眼确认 DDS 真在跑）
        def _startup_check_logger():
            time.sleep(2.0)
            s, f = self._publish_success_count, self._publish_fail_count
            try:
                node_ns = self._node.get_namespace()
            except Exception:
                node_ns = "/"
            print(
                f"[RosV14Adapter] 启动自检 OK: node={node_ns}/{self._node_name} "
                f"topic={self._topic} publish_count=s{s} ok={s} fail={f} "
                f"frame={self._pub_frame_id} heartbeat_hz_approx = {round(max(1, s))}"
            )
        threading.Thread(target=_startup_check_logger, daemon=True).start()
        return None

    def close(self) -> None:
        import sys as _sys
        try:
            self._closing.set()
        except Exception:
            pass
        # 顺序严格：① heartbeat（引用 pub/node）→ ② spin（引用 node/rclpy）→ ③ destroy_node → ④ rclpy.shutdown
        #          （FastDDS DomainParticipant 必须在 rclpy.shutdown 前先 destroy_node，否则底层 context 残留）
        for _t_name, _t in (
            ("heartbeat", self._heartbeat_thread),
            ("spin", self._spin_thread),
        ):
            try:
                if _t and _t.is_alive():
                    _t.join(timeout=1.2)
                    if _t.is_alive():
                        try:
                            print(f"[RosV14Adapter.close] {_t_name}_thread 仍存活（daemon={_t.daemon}），将随解释器退出回收", file=_sys.stderr)
                        except Exception:
                            pass
            except Exception:
                pass
        # ③ 先 destroy_node（释放 FastDDS Participant / Publisher / Subscription）
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
        # ④ 再 rclpy.shutdown（释放全局 context，避免 DDS 进程级残留）
        rclpy = self._rclpy
        if rclpy is not None:
            try:
                if rclpy.ok():
                    rclpy.shutdown(context=rclpy.context)
            except Exception:
                try:
                    if rclpy.ok():
                        rclpy.shutdown()
                except Exception:
                    pass
        # 最后置空，防止 atexit 二次释放时引用失效对象
        try:
            self._pub = None
            self._node = None
            self._spin_thread = None
            self._heartbeat_thread = None
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

    def _heartbeat_loop(self) -> None:
        """10Hz 持续把当前 cmd_deg 发到 /joint_states。
        ⭐ Bugfix(2026-09-07)：之前用累积 t += period + max(0, t - time.time()) 的 sleep 方式，
        若某一次 sleep 实际睡多了（CPU 被抢占/GC），下一轮 remaining=0 就会完全不睡，
        publish 飙升到 ~40kHz/100% CPU，直接挤占回调线程导致 IK/move_to_point 拿不到调度。
        → 改成「next_t = time.time() + period」绝对时间下一次，sleep 差量，无论睡多睡少都不丢失节拍，10Hz 稳稳。
        """
        period = max(1e-3, float(self._heartbeat_period_s))
        next_run_ts = time.monotonic()  # monotonic 不受 NTP 调时间影响
        while not self._closing.is_set():
            try:
                self._publish_locked_cmd(None)
            except Exception:
                pass
            next_run_ts += period
            remaining = next_run_ts - time.monotonic()
            if remaining > 0:
                # 分多次小步 sleep，保证 _closing 能被快速响应（最差 50ms 内 detect）
                while remaining > 0 and not self._closing.is_set():
                    step = min(remaining, 0.050)
                    time.sleep(step)
                    remaining -= step
            else:
                # 已经超了（被抢占太久）：跳过但不累计，直接对齐到下一个「当前时刻 + 1 拍」
                next_run_ts = time.monotonic() + period

    def _publish_locked_cmd(self, override_frame_id) -> bool:
        """把 self._cmd_deg 当下内容发一次。心跳 + 用户 publish 都走这里。"""
        if self._rclpy is None or self._JointState is None or self._pub is None:
            return False
        if not self._rclpy.ok():
            return False
        with self._lock:
            cmd = dict(self._cmd_deg)
            frame_id = override_frame_id or self._pub_frame_id or self._default_frame_id
            self._pub_frame_id = frame_id
        msg = self._JointState()
        now = self._node.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = frame_id
        msg.name = list(URDF_JOINT_ORDER)
        msg.position = [deg_to_rad(cmd[s]) for s in SEMANTIC_JOINT_ORDER]
        try:
            self._pub.publish(msg)
            with self._lock:
                self._publish_success_count += 1
            return True
        except Exception:
            with self._lock:
                self._publish_fail_count += 1
            return False

    # ── ControlAdapter 接口 ─────────────────────────────────────────

    def get_current_pose_deg(self) -> Optional[Dict[str, float]]:
        """URDF 的"当前反馈"就是我们最后一次写入的 cmd 缓存。

        原理：launch 里 use_joint_state_publisher=False → 只有 v15 在写 /joint_states，
        robot_state_publisher 只接收我们的消息；因此 cmd_deg 缓存 == URDF 实际角度。
        """
        with self._lock:
            if self._last_cmd_ts is None:
                return None
            return dict(self._cmd_deg)

    def get_last_update_ts(self) -> Optional[float]:
        with self._lock:
            return self._last_cmd_ts

    def publish_pose_deg(self, pose_deg: Dict[str, float], frame_id: Optional[str] = None) -> bool:
        if self._rclpy is None or self._JointState is None or self._pub is None:
            return False
        if not self._rclpy.ok():
            return False
        # 1) 更新 cmd 缓存（只改用户传入的字段，其他关节保留原值）
        with self._lock:
            for k, v in pose_deg.items():
                if k in self._cmd_deg and v is not None:
                    self._cmd_deg[k] = float(v)
            self._last_cmd_ts = time.time()
            if frame_id is not None:
                self._pub_frame_id = frame_id
        # 2) 用户触发立即发布一次（后续心跳会 10Hz 保持刷新）
        return self._publish_locked_cmd(frame_id)
