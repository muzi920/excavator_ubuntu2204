#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
V11 传感器 → shandong_0.5 桥接器（★ 只订阅 / 不发布，只读 v11 sensor data）

目标：
  0. 不修改 v11 目录下任何文件。所有新代码写在 shandong_0.5/v11_control 下。
  1. 关节角输入：订阅 v11 ROS2 GUI 发布的 /excavator/joint_states，把 v11 自带的
     4 路 WT901C485 倾角 + 雷达 IMU 空间积分（大臂/小臂/铲斗/回转）作为 shandong_0.5 单关节串行
     控制的【物理反馈关节角】，对齐到统一的四语义角度字典
     {'swing_yaw','boom_swing','arm_boom','bucket_arm'}（单位 = 度）。
  2. 点云图像输入：订阅 v11 发布的 /lidar/points（默认 base_link 系下，随车体转），
     解包成 Nx3 的 float32 numpy 数组（单位 = 米），作为下游视觉/采样/点云输入。

输入话题（严格按 v11 ros2_multimodal_gui.py 考古的字段写死，不做任何主观假设）：
  T1 /excavator/joint_states  sensor_msgs/JointState
      - name（v11 固定写死）: ['boom_joint', 'arm_joint', 'bucket_joint', 'swing_joint']
        position[0..3] = 弧度；分别对应 boom_swing / arm_boom / bucket_arm / swing_yaw
      - 发布频率 ~20Hz（v11 GUI 主循环 after(50, ...)）
      - 正方向语义（与 shandong_0.5 完全一致，天然对齐，无需取反）：
        +boom_swing 向下压 / +arm_boom 小臂内收 / +bucket_arm 铲斗内卷 / +swing_yaw 从上往下顺时针 CW
  T2 /lidar/points            sensor_msgs/PointCloud2
      - frame_id = base_link
      - 字段 PointField = (x,y,z,rgb) float32×3 + uint32×1，步长 16 字节
      - 发布频率 ~10Hz（点云节流批处理）

输出 API（纯 Python 调用，不发布话题，零依赖任何 v11 import）：
  get_joint_angles_deg(blocking:bool, timeout_s:float) -> SemanticJointAnglesDeg
  get_point_cloud_xyz(blocking:bool, timeout_s:float)  -> PointCloudXYZ
  has_new_joint(since_ts:float) -> bool
  has_new_pointcloud(since_ts:float) -> bool

反向控制（从 shandong_0.5 → v11_ros_topic_controller 执行单关节/单 Point 动作）：
  由于用户显式要求「v11 已加控制接口，不要修改 v11」，桥接器还提供 2 个纯工具函数（无状态）：
    make_state_joint_msg(semantic_pose_deg: dict) -> sensor_msgs/JointState
    make_target_point_msg(x,y,z, frame_id='base_link') -> geometry_msgs/PointStamped
  用于通过你的主节点 create_publisher 发布到 v11 控制话题 /state_joint 或 /target_point。

★ 门禁 / 防破：
  - 零修改 v11。
  - 不 import 任何 v11/v4/v10/v5/v3 目录下的模块（只 import 系统 rclpy/numpy + 标准库）。
  - 不 import v15 主目录任何模块（只和 shandong_0.5 同级 control_lib 打配合，需要时外部再连）。
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

import numpy as np


# ============================================================
# 0. 语义关节角 / 点云 类型定义（shandong_0.5 通用四语义）
# ============================================================
@dataclass
class SemanticJointAnglesDeg:
    """shandong_0.5 统一的四语义关节角度字典的 dataclass 版本。"""
    swing_yaw: float = 0.0
    boom_swing: float = 0.0
    arm_boom: float = 0.0
    bucket_arm: float = 0.0
    ts: float = 0.0

    def as_pose_deg(self) -> Dict[str, float]:
        return {
            "swing_yaw": float(self.swing_yaw),
            "boom_swing": float(self.boom_swing),
            "arm_boom": float(self.arm_boom),
            "bucket_arm": float(self.bucket_arm),
        }


@dataclass
class PointCloudXYZ:
    """解包后的点云。xyz 为 N×3 float32 numpy；rgb 若存在则 N×3 uint8，否则 None。"""
    xyz: np.ndarray = field(default_factory=lambda: np.zeros((0, 3), dtype=np.float32))
    rgb: Optional[np.ndarray] = None
    ts: float = 0.0

    @property
    def n_points(self) -> int:
        return int(self.xyz.shape[0])


# ============================================================
# 1. 线程安全缓存：最新一帧 joint + 最新一帧 pointcloud + Event
# ============================================================
class _FrameCache:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._joint: Optional[SemanticJointAnglesDeg] = None
        self._pc: Optional[PointCloudXYZ] = None
        self._joint_event = threading.Event()
        self._pc_event = threading.Event()

    # ---- joint ----
    def set_joint(self, j: SemanticJointAnglesDeg) -> None:
        with self._lock:
            self._joint = j
        self._joint_event.set()

    def get_joint(self) -> Optional[SemanticJointAnglesDeg]:
        with self._lock:
            return None if self._joint is None else SemanticJointAnglesDeg(**self._joint.__dict__)

    def wait_joint(self, timeout_s: float) -> bool:
        return self._joint_event.wait(timeout=timeout_s)

    # ---- pc ----
    def set_pc(self, pc: PointCloudXYZ) -> None:
        with self._lock:
            self._pc = pc
        self._pc_event.set()

    def get_pc(self) -> Optional[PointCloudXYZ]:
        with self._lock:
            cur = self._pc
            if cur is None:
                return None
            return PointCloudXYZ(
                xyz=cur.xyz.copy(),
                rgb=None if cur.rgb is None else cur.rgb.copy(),
                ts=cur.ts,
            )

    def wait_pc(self, timeout_s: float) -> bool:
        return self._pc_event.wait(timeout=timeout_s)


# ============================================================
# 2. V11SensorBridge
# ============================================================
class V11SensorBridge:
    """
    最简用法（真机 ROS2 环境已 source）：
        import rclpy
        rclpy.init()
        br = V11SensorBridge()
        br.start_spin_thread()
        js = br.get_joint_angles_deg(blocking=True, timeout_s=2.0)
        pc = br.get_point_cloud_xyz(blocking=True, timeout_s=5.0)
        br.shutdown()

    Mock 用法（无 ROS 环境）：
        br = V11SensorBridge(use_mock=True)
        br.start_spin_thread()
        br.inject_mock_joint_deg(swing_yaw=30, boom_swing=-5, arm_boom=90, bucket_arm=0)
        js = br.get_joint_angles_deg(blocking=True, timeout_s=1.0)
    """

    # -------- 话题名 / 关节名 100% 写死来自 v11 考古 --------
    TOPIC_EXCAVATOR_JOINT_STATES: str = "/excavator/joint_states"
    TOPIC_LIDAR_POINTS: str = "/lidar/points"
    TOPIC_LIDAR_POINTS_ODOM: str = "/lidar/points_odom"

    # v11 publish_joint_state() L273 name 写死：[boom,arm,bucket,swing]
    V11_JOINT_NAME_ORDER: Tuple[str, ...] = (
        "boom_joint",
        "arm_joint",
        "bucket_joint",
        "swing_joint",
    )
    # 四语义对应：v11 的每个 name -> shandong_0.5 语义键
    V11_NAME_TO_SEMANTIC: Dict[str, str] = {
        "boom_joint": "boom_swing",
        "arm_joint": "arm_boom",
        "bucket_joint": "bucket_arm",
        "swing_joint": "swing_yaw",
    }

    # -------- 反向控制：v11_ros_topic_controller 订阅的话题（用于纯工具函数，本类不发布）--------
    TOPIC_V11_STATE_JOINT: str = "/state_joint"
    TOPIC_V11_TARGET_POINT: str = "/target_point"
    V11_CONTROL_JOINT_ORDER: Tuple[str, ...] = (
        "swing_joint",   # position[0] 对应 swing_yaw
        "boom_joint",    # position[1] 对应 boom_swing
        "arm_joint",     # position[2] 对应 arm_boom
        "bucket_joint",  # position[3] 对应 bucket_arm
    )
    V11_CONTROL_SEMANTIC_TO_IDX: Dict[str, int] = {
        "swing_yaw": 0,
        "boom_swing": 1,
        "arm_boom": 2,
        "bucket_arm": 3,
    }

    def __init__(
        self,
        *,
        node_name: str = "shandong_0_5_v11_sensor_bridge",
        use_mock: bool = False,
        use_odom_pointcloud: bool = False,
    ) -> None:
        self.node_name = node_name
        self.use_mock = bool(use_mock)
        self.pc_topic = self.TOPIC_LIDAR_POINTS_ODOM if use_odom_pointcloud else self.TOPIC_LIDAR_POINTS

        self._cache = _FrameCache()
        self._closing = threading.Event()

        self._rclpy: Optional[Any] = None
        self._node: Optional[Any] = None
        self._spin_thread: Optional[threading.Thread] = None

        if self.use_mock:
            # mock：不 touch rclpy，全靠 inject_mock_* 手动打
            return

        self._init_ros()

    # ============================================================
    # ROS 初始化 & spin
    # ============================================================
    def _init_ros(self) -> None:
        import rclpy  # type: ignore
        from rclpy.node import Node  # type: ignore
        from rclpy.qos import (  # type: ignore
            QoSProfile,
            HistoryPolicy,
            ReliabilityPolicy,
        )

        if not rclpy.ok():
            rclpy.init(args=[])
        self._rclpy = rclpy
        self._node = Node(self.node_name)
        self._log = self._node.get_logger()

        qos_js = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        qos_pc = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        from sensor_msgs.msg import JointState, PointCloud2  # type: ignore
        self._msg_JointState = JointState
        self._msg_PointCloud2 = PointCloud2

        self._node.create_subscription(
            JointState, self.TOPIC_EXCAVATOR_JOINT_STATES,
            self._on_joint_states, qos_js,
        )
        self._node.create_subscription(
            PointCloud2, self.pc_topic,
            self._on_lidar_points, qos_pc,
        )

        self._log.info(
            f"[v11_bridge] Node={self.node_name} 订阅 2 条 v11 sensor topics:\n"
            f"   1) {self.TOPIC_EXCAVATOR_JOINT_STATES} (JointState, 顺序按 name 对齐)\n"
            f"   2) {self.pc_topic} (PointCloud2, step=16B  xyz+rgb)"
        )

    def start_spin_thread(self) -> None:
        if self.use_mock:
            return
        if self._spin_thread and self._spin_thread.is_alive():
            return

        def _spin() -> None:
            try:
                while not self._closing.is_set() and self._rclpy and self._node:
                    self._rclpy.spin_once(self._node, timeout_sec=0.01)
            except Exception as e:  # pragma: no cover
                if self._node is not None:
                    try:
                        self._log.error(f"[v11_bridge] spin 异常: {e!r}")
                    except Exception:
                        pass

        self._spin_thread = threading.Thread(target=_spin, daemon=True)
        self._spin_thread.start()

    # ============================================================
    # API（对外）
    # ============================================================
    def get_joint_angles_deg(
        self,
        *,
        blocking: bool = False,
        timeout_s: float = 1.0,
    ) -> Optional[SemanticJointAnglesDeg]:
        if blocking:
            self._cache.wait_joint(timeout_s)
        return self._cache.get_joint()

    def get_point_cloud_xyz(
        self,
        *,
        blocking: bool = False,
        timeout_s: float = 5.0,
    ) -> Optional[PointCloudXYZ]:
        if blocking:
            self._cache.wait_pc(timeout_s)
        return self._cache.get_pc()

    def has_new_joint(self, since_ts: float) -> bool:
        cur = self._cache.get_joint()
        return cur is not None and cur.ts > float(since_ts)

    def has_new_pointcloud(self, since_ts: float) -> bool:
        cur = self._cache.get_pc()
        return cur is not None and cur.ts > float(since_ts)

    # ============================================================
    # 反向控制：纯工具函数（不发话题；请在你自己的节点里 create_publisher 用）
    # ============================================================
    def make_state_joint_msg(self, semantic_pose_deg: Dict[str, float]):
        """
        semantic_pose_deg = {"swing_yaw":..., "boom_swing":..., "arm_boom":..., "bucket_arm":...}
        返回 sensor_msgs/JointState（弧度；name/position 顺序严格对齐 v11_ros_topic_controller 的订阅）。
        """
        if self.use_mock:
            # mock：返回 dict，单元测试能断言
            deg = {k: float(semantic_pose_deg.get(k, 0.0)) for k in self.V11_CONTROL_SEMANTIC_TO_IDX}
            rad = {k: v * math.pi / 180.0 for k, v in deg.items()}
            return {
                "name": list(self.V11_CONTROL_JOINT_ORDER),
                "position_rad": [
                    rad["swing_yaw"], rad["boom_swing"], rad["arm_boom"], rad["bucket_arm"],
                ],
            }
        JointState = self._msg_JointState
        msg = JointState()
        try:
            msg.header.stamp = self._node.get_clock().now().to_msg()  # type: ignore
        except Exception:
            pass
        msg.header.frame_id = "base_link"
        msg.name = list(self.V11_CONTROL_JOINT_ORDER)
        deg = {k: float(semantic_pose_deg.get(k, 0.0)) for k in self.V11_CONTROL_SEMANTIC_TO_IDX}
        msg.position = [
            deg["swing_yaw"] * math.pi / 180.0,
            deg["boom_swing"] * math.pi / 180.0,
            deg["arm_boom"] * math.pi / 180.0,
            deg["bucket_arm"] * math.pi / 180.0,
        ]
        return msg

    def make_target_point_msg(self, x_m: float, y_m: float, z_m: float, frame_id: str = "base_link"):
        if self.use_mock:
            return {"frame_id": str(frame_id), "xyz_m": (float(x_m), float(y_m), float(z_m))}
        from geometry_msgs.msg import PointStamped  # type: ignore
        msg = PointStamped()
        try:
            msg.header.stamp = self._node.get_clock().now().to_msg()  # type: ignore
        except Exception:
            pass
        msg.header.frame_id = str(frame_id)
        msg.point.x = float(x_m)
        msg.point.y = float(y_m)
        msg.point.z = float(z_m)
        return msg

    # ============================================================
    # Mock 注入（离线单测用）
    # ============================================================
    def inject_mock_joint_deg(
        self,
        *,
        swing_yaw: float,
        boom_swing: float,
        arm_boom: float,
        bucket_arm: float,
    ) -> None:
        if not self.use_mock:
            raise RuntimeError("inject_mock_* 仅在 use_mock=True 时可用")
        self._cache.set_joint(SemanticJointAnglesDeg(
            swing_yaw=float(swing_yaw),
            boom_swing=float(boom_swing),
            arm_boom=float(arm_boom),
            bucket_arm=float(bucket_arm),
            ts=time.time(),
        ))

    def inject_mock_pointcloud_xyz(self, xyz_np: np.ndarray, *, rgb_uint8: Optional[np.ndarray] = None) -> None:
        if not self.use_mock:
            raise RuntimeError("inject_mock_* 仅在 use_mock=True 时可用")
        arr = np.asarray(xyz_np, dtype=np.float32)
        if arr.ndim != 2 or arr.shape[1] != 3:
            raise ValueError(f"点云 xyz 要求 (N,3)，实际 shape={arr.shape}")
        self._cache.set_pc(PointCloudXYZ(
            xyz=arr.copy(),
            rgb=None if rgb_uint8 is None else np.asarray(rgb_uint8, dtype=np.uint8).copy(),
            ts=time.time(),
        ))

    # ============================================================
    # 回调：/excavator/joint_states
    # ============================================================
    def _on_joint_states(self, msg) -> None:
        try:
            names = list(msg.name or [])
            positions = list(msg.position or [])
            if len(positions) < 4:
                return

            sem_rad: Dict[str, float] = {
                "swing_yaw": 0.0, "boom_swing": 0.0,
                "arm_boom": 0.0, "bucket_arm": 0.0,
            }
            if len(names) == len(positions) and len(names) >= 4:
                # 按 name 映射（v11 默认顺序写死，这里走 name 更稳，兼容未来 v11 改顺序）
                for n, rad in zip(names, positions):
                    if n in self.V11_NAME_TO_SEMANTIC:
                        sem_rad[self.V11_NAME_TO_SEMANTIC[n]] = float(rad)
            else:
                # fallback：按 v11 L273 固定顺序 [boom, arm, bucket, swing]
                sem_rad["boom_swing"] = float(positions[0])
                sem_rad["arm_boom"] = float(positions[1])
                sem_rad["bucket_arm"] = float(positions[2])
                sem_rad["swing_yaw"] = float(positions[3])

            deg = SemanticJointAnglesDeg(
                swing_yaw=sem_rad["swing_yaw"] * 180.0 / math.pi,
                boom_swing=sem_rad["boom_swing"] * 180.0 / math.pi,
                arm_boom=sem_rad["arm_boom"] * 180.0 / math.pi,
                bucket_arm=sem_rad["bucket_arm"] * 180.0 / math.pi,
                ts=time.time(),
            )
            self._cache.set_joint(deg)
        except Exception as e:  # pragma: no cover
            if self._node is not None:
                try:
                    self._log.error(f"[v11_bridge] _on_joint_states: {e!r}")
                except Exception:
                    pass

    # ============================================================
    # 回调：/lidar/points（或 odom 版）
    # 按 v11 ros2_multimodal_gui L180-L197 写死：dt=(x,y,z,rgb) 步长 16 byte
    # ============================================================
    def _on_lidar_points(self, msg) -> None:
        try:
            data = bytes(msg.data or b"")
            n_pts = int(msg.width or 0)
            step = int(msg.point_step or 0)
            if n_pts <= 0 or step <= 0:
                return
            # 步长 >= 16 时取前 16B；步长 <16 时放弃（不猜测字段）
            if step < 16:
                return
            # 只收前 n_pts*step，防止越界
            need = n_pts * step
            if len(data) < need:
                n_pts = len(data) // step
                if n_pts <= 0:
                    return
            flat = np.frombuffer(data[: n_pts * step], dtype=np.uint8).reshape(n_pts, step)
            xyz_bytes = flat[:, :12]  # x,y,z 前 12B
            xyz = xyz_bytes.view(np.float32).reshape(n_pts, 3).astype(np.float32, copy=True)
            rgb_raw = None
            if step >= 16:
                # L178-L179: rgb_vals = gray<<16 | gray<<8 | gray  (UINT32 小端打包)
                rgb_packed = flat[:, 12:16].copy().view(np.uint32).reshape(n_pts)
                r = ((rgb_packed >> 16) & 0xFF).astype(np.uint8)
                g = ((rgb_packed >> 8) & 0xFF).astype(np.uint8)
                b = (rgb_packed & 0xFF).astype(np.uint8)
                rgb_raw = np.stack([r, g, b], axis=1)
            self._cache.set_pc(PointCloudXYZ(xyz=xyz, rgb=rgb_raw, ts=time.time()))
        except Exception as e:  # pragma: no cover
            if self._node is not None:
                try:
                    self._log.error(f"[v11_bridge] _on_lidar_points: {e!r}")
                except Exception:
                    pass

    # ============================================================
    # Shutdown
    # ============================================================
    def shutdown(self) -> None:
        self._closing.set()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
        if self._rclpy is not None:
            try:
                self._rclpy.shutdown()
            except Exception:
                pass
            self._rclpy = None
