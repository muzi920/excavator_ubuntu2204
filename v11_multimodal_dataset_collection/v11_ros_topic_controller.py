#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
V11 独立 ROS Topic 控制入口（★ 新增文件，不修改任何 v11 原程序）
=====================================================================

目标：给『v11_multimodal_dataset_collection』增加 2 条 ROS 控制话题，
     与『v15_action_task』的控制方式、指令格式完全对接。

订阅（输入）：
  1) /state_joint         sensor_msgs/JointState
        - name 顺序固定 = ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"]
        - position[] = 弧度（ROS 标准），四元素对应 swing_yaw / boom_swing / arm_boom / bucket_arm
        - 对应 v15 的 /v15/cmd/joint_states （复制粘贴即可复用）
  2) /target_point        geometry_msgs/PointStamped
        - frame_id = "base_link"
        - point (x, y, z) 单位 = 米；x=前、y=左、z=上
        - 对应 v15 的 /v15/cmd/target_point （复制粘贴即可复用）

发布（可选反馈，仅用于调试）：
  * /v11/current_state    sensor_msgs/JointState (度，方便实时查看目标下发值)

依赖（v11 原本就 import 的，完全复用，不新增）：
  - shandong.v4_control_closed.angle_controller.AngleController
  - shandong.v1_control_base.zs_excavator_controller.build_controller
  - shandong.v10_cailbration_arm.inverse_kinematics.ExcavatorIK

使用方式 1（独立运行，不启相机）：
  cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v11_multimodal_dataset_collection
  python3 v11_ros_topic_controller.py --offline

使用方式 2（配合 v11 camera_all.launch.py，同进程独立节点运行）：
  ros2 launch v11_multimodal_dataset_collection v11_cameras_with_ros_control.launch.py

★ 方向语义（与 v15 完全一致，保证指令可互换）：
  swing_yaw   +deg  = 从上往下看顺时针 (+CW)
  boom_swing  +deg  = 大臂向下压    (-deg = 向上抬)
  arm_boom    +deg  = 小臂向内收    (-deg = 小臂向外推)
  bucket_arm  +deg  = 铲斗向内卷铲  (-deg = 铲斗向外打开)
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# ============================================================================
# 1. 兼容：追加 v1/v4/v10 路径（v11 multimodal_gui.py 原本就是这么做的，保持一致）
# ============================================================================
_THIS_DIR = os.path.abspath(os.path.dirname(__file__))
_SRC_PARENT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
for _p in (
    os.path.join(_SRC_PARENT, "v1_control_base"),
    os.path.join(_SRC_PARENT, "v4_control_closed"),
    os.path.join(_SRC_PARENT, "v10_cailbration_arm"),
):
    if _p not in sys.path:
        sys.path.insert(0, _p)
if _SRC_PARENT not in sys.path:
    sys.path.insert(0, _SRC_PARENT)


def _rad_to_deg(x: float) -> float:
    return float(x) * 180.0 / math.pi


def _deg_to_rad(x: float) -> float:
    return float(x) * math.pi / 180.0


# ============================================================================
# 2. URDF 关节名 → v4 AngleController joint_name 映射（保证 v15 指令原样复用）
# ============================================================================
URDF_JOINT_ORDER: Tuple[str, ...] = (
    "swing_joint",   # position[0]
    "boom_joint",    # position[1]
    "arm_joint",     # position[2]
    "bucket_joint",  # position[3]
)
SEMANTIC_JOINT_ORDER: Tuple[str, ...] = (
    "swing_yaw",
    "boom_swing",
    "arm_boom",
    "bucket_arm",
)
URDF_TO_SEMANTIC: Dict[str, str] = dict(zip(URDF_JOINT_ORDER, SEMANTIC_JOINT_ORDER))


@dataclass
class BridgeState:
    """线程安全的缓存：最近一次下发到 v4 AngleController 的目标角度（度）。"""
    _lock: threading.Lock = field(default_factory=threading.Lock)
    target_deg: Dict[str, float] = field(default_factory=lambda: {
        "swing_yaw": 0.0,
        "boom_swing": 5.0,
        "arm_boom": 90.0,
        "bucket_arm": 90.0,
    })
    last_ts: float = 0.0
    # 目标点配置：默认桶斗姿态 = 水平向下 45°（挖掘常用姿态）
    default_bucket_guess_deg: float = -45.0
    swing_guess_deg: float = 0.0

    def snapshot(self) -> Tuple[Dict[str, float], float]:
        with self._lock:
            return dict(self.target_deg), float(self.last_ts)

    def update(self, new_deg: Dict[str, float]) -> None:
        with self._lock:
            for k, v in new_deg.items():
                if k in self.target_deg:
                    self.target_deg[k] = float(v)
            self.last_ts = time.time()


class V11RosControlBridge:
    """
    独立控制桥：
      - 订阅 ROS 两个话题（state_joint / target_point）
      - 内部实例化 v4 build_controller + AngleController（与 multimodal_gui.py 完全一致）
      - 对 state_joint（弧度）做 4 关节逐路 move_joint_to_angle；
      - 对 target_point（米，XYZ）做 v10 ExcavatorIK 逆解 → swing 单独按 Y/X 偏置求出 → 逐路到位
    """

    def __init__(
        self,
        *,
        can_port: str = "/dev/ttyUSB_Controller",
        can_baud: int = 115200,
        offline: bool = False,
        node_name: str = "v11_ros_topic_controller",
        default_bucket_guess_deg: float = -45.0,
    ) -> None:
        self.node_name = node_name
        self.offline = bool(offline)
        self.can_port = can_port
        self.can_baud = int(can_baud)
        self.state = BridgeState(default_bucket_guess_deg=default_bucket_guess_deg)

        # -------- 硬件（v1/v4 闭环控制器） --------
        self.base_controller = None
        self.angle_ctrl = None
        if not self.offline:
            self._init_hw()
        else:
            print("[v11_bridge] offline 模式：不连接 CAN 串口，只打印日志")

        # -------- 逆解器（v10 原生） --------
        self.ik = None
        try:
            from inverse_kinematics import ExcavatorIK  # type: ignore
            self.ik = ExcavatorIK()
        except Exception as e:
            print(f"[v11_bridge][warn] 加载 v10 ExcavatorIK 失败: {e!r}；/target_point 将不可用")

        # -------- ROS 节点 --------
        self._rclpy = None
        self._node = None
        self._spin_thread: Optional[threading.Thread] = None
        self._closing = threading.Event()
        self._init_ros()

        # -------- 心跳：周期发布当前目标角度（调试用） --------
        self._pub_thread: Optional[threading.Thread] = None
        self._pub_thread = threading.Thread(target=self._publisher_loop, daemon=True)
        self._pub_thread.start()

    # ------------------------------------------------------------------
    # 初始化：v4 硬件
    # ------------------------------------------------------------------
    def _init_hw(self) -> None:
        try:
            from zs_excavator_controller import build_controller  # type: ignore
            from angle_controller import AngleController  # type: ignore
            self.base_controller = build_controller(port=self.can_port, baudrate=self.can_baud)
            if not getattr(self.base_controller, "connect", lambda: False)():
                print(f"[v11_bridge][warn] CAN 串口 {self.can_port} 连接失败；降级为 offline 模式")
                self.offline = True
                self.base_controller = None
                return
            self.angle_ctrl = AngleController(self.base_controller)
            print(f"[v11_bridge] v4 AngleController OK  CAN={self.can_port}:{self.can_baud}")
        except Exception as e:
            print(f"[v11_bridge][warn] v4 硬件初始化失败，降级为 offline 模式: {type(e).__name__}: {e}")
            self.offline = True
            self.base_controller = None
            self.angle_ctrl = None

    # ------------------------------------------------------------------
    # 初始化：ROS2 Node + 2 订阅 + 1 发布
    # ------------------------------------------------------------------
    def _init_ros(self) -> None:
        import rclpy  # type: ignore
        from rclpy.node import Node  # type: ignore
        from rclpy.qos import QoSProfile, HistoryPolicy  # type: ignore
        if not rclpy.ok():
            rclpy.init(args=[])
        self._rclpy = rclpy
        self._node = Node(self.node_name)
        self._log = self._node.get_logger()
        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST)
        qos1 = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST)

        from sensor_msgs.msg import JointState  # type: ignore
        from geometry_msgs.msg import PointStamped  # type: ignore
        self._JointState = JointState
        self._node.create_subscription(
            JointState, "/state_joint", self._on_state_joint, qos,
        )
        self._node.create_subscription(
            PointStamped, "/target_point", self._on_target_point, qos1,
        )
        self._pub_cur = self._node.create_publisher(JointState, "/v11/current_state", qos)

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        self._log.info(
            f"[v11_bridge] 启动完成: offline={self.offline}  hw={'OK' if self.angle_ctrl else 'N/A'}  "
            f"ik={'OK' if self.ik else 'N/A'}\n"
            f"[v11_bridge] 订阅 2 条（与 v15 指令完全兼容）：\n"
            f"   ① /state_joint    (sensor_msgs/JointState 弧度；name=swing/boom/arm/bucket_joint)\n"
            f"   ② /target_point   (geometry_msgs/PointStamped 米；frame_id=base_link)\n"
            f"[v11_bridge] 反馈调试话题：/v11/current_state (sensor_msgs/JointState 度)\n"
        )

    # ------------------------------------------------------------------
    # 辅助：下发 1 个关节目标角度（°）到 v4 AngleController
    # ------------------------------------------------------------------
    def _dispatch_one_joint(self, sem: str, target_deg: float, *, tolerance_deg: float = 2.0) -> bool:
        self.state.target_deg[sem] = float(target_deg)
        self.state.last_ts = time.time()
        if self.offline or self.angle_ctrl is None:
            # offline：仍然保留缓存（方便调试 /v11/current_state）
            print(f"[v11_bridge][offline] -> {sem:>10s} = {target_deg:8.2f} deg")
            return True
        try:
            # 这里故意不阻塞等待完成（用户的 v15 控制是 4 关节独立闭环异步到位）
            self.angle_ctrl.move_joint_to_angle(
                sem, float(target_deg), tolerance=tolerance_deg,
                ch1_mv=0, ch2_mv=0, ch3_mv=2000,
                ramp_up_s=0.2, ramp_down_s=0.2,
            )
            return True
        except Exception as e:
            self._log.error(f"[v11_bridge] 下发失败 {sem}={target_deg:.2f}deg: {e!r}")
            return False

    # ------------------------------------------------------------------
    # 回调 1：/state_joint (JointState 弧度)
    # ------------------------------------------------------------------
    def _on_state_joint(self, msg) -> None:
        # 支持两种格式：
        #  A) name=4 个 urdf 关节顺序，position=4 个弧度（推荐，100% 复用 v15 指令）
        #  B) name=任意但包含 swing_joint/boom_joint/arm_joint/bucket_joint（部分写/乱序也行）
        pairs: Dict[str, float] = {}
        if len(msg.name) == len(msg.position) and len(msg.name) > 0:
            for n, v in zip(msg.name, msg.position):
                if n in URDF_TO_SEMANTIC:
                    pairs[URDF_TO_SEMANTIC[n]] = _rad_to_deg(float(v))
        # 兜底：当没有 name 或 name 不匹配，按默认 4 关节顺序来（方便用户直接发 4 元素数组）
        if not pairs and len(msg.position) == 4:
            for sem, v in zip(SEMANTIC_JOINT_ORDER, msg.position):
                pairs[sem] = _rad_to_deg(float(v))
        if not pairs:
            self._log.warn(f"[v11_bridge][/state_joint] 无法解析消息 name={list(msg.name)!r} pos_len={len(msg.position)}")
            return
        # 下发 4 关节（异步并行）
        for sem in SEMANTIC_JOINT_ORDER:
            if sem in pairs:
                self._dispatch_one_joint(sem, pairs[sem])
        self._log.info(
            f"[v11_bridge][/state_joint] 下发  sw={pairs.get('swing_yaw', float('nan')):7.2f}°  "
            f"bm={pairs.get('boom_swing', float('nan')):7.2f}°  ar={pairs.get('arm_boom', float('nan')):7.2f}°  "
            f"bk={pairs.get('bucket_arm', float('nan')):7.2f}°"
        )

    # ------------------------------------------------------------------
    # 回调 2：/target_point (PointStamped 米 XYZ)
    # ------------------------------------------------------------------
    def _on_target_point(self, msg) -> None:
        if self.ik is None:
            self._log.error("[v11_bridge][/target_point] v10 ExcavatorIK 不可用，跳过")
            return
        x, y, z = float(msg.point.x), float(msg.point.y), float(msg.point.z)
        bucket_guess = float(self.state.default_bucket_guess_deg)
        # 2D 回转角：y/x（平面投影偏航）；x≈0 时用 sign(y) * 90°
        if abs(x) < 1e-4:
            swing_deg = 90.0 if y >= 0 else -90.0
        else:
            swing_deg = math.degrees(math.atan2(y, x))
        # 平面距离 d：IK 的 input_x（v10 calculate_ik 的 target_x 是投影距离）
        d = math.hypot(x, y)
        try:
            solution = self.ik.calculate_ik(d, z, bucket_guess)
        except Exception as e:
            self._log.error(f"[v11_bridge][/target_point] IK 异常: {e!r}")
            return
        if solution is None:
            self._log.warn(
                f"[v11_bridge][/target_point] IK 无解: (x={x:.3f}, y={y:.3f}, z={z:.3f})m  "
                f"d={d:.3f}m bucket_guess={bucket_guess:.1f}deg"
            )
            return
        boom_deg, arm_deg, bucket_deg = (float(v) for v in solution)
        self._dispatch_one_joint("swing_yaw", swing_deg)
        self._dispatch_one_joint("boom_swing", boom_deg)
        self._dispatch_one_joint("arm_boom", arm_deg)
        self._dispatch_one_joint("bucket_arm", bucket_deg)
        self._log.info(
            f"[v11_bridge][/target_point] 收到 (x={x:.3f}, y={y:.3f}, z={z:.3f})m  IK OK  "
            f"下发 → sw={swing_deg:7.2f}°  bm={boom_deg:7.2f}°  ar={arm_deg:7.2f}°  bk={bucket_deg:7.2f}°"
        )

    # ------------------------------------------------------------------
    # 背景线程：spin ROS + 周期 publish /v11/current_state（度反馈）
    # ------------------------------------------------------------------
    def _spin(self) -> None:
        try:
            self._rclpy.spin(self._node)
        except Exception:
            pass

    def _publisher_loop(self) -> None:
        JointState = self._JointState
        period_s = 0.1
        while not self._closing.is_set():
            try:
                tgt, _ts = self.state.snapshot()
                js = JointState()
                js.header.frame_id = "base_link"
                js.header.stamp = self._node.get_clock().now().to_msg()
                js.name = list(URDF_JOINT_ORDER)
                # 反馈给调试：用"度"作为 position（注意：这是非标准，仅用于观察/debug）
                js.position = [float(tgt[s]) for s in SEMANTIC_JOINT_ORDER]
                self._pub_cur.publish(js)
            except Exception:
                pass
            time.sleep(period_s)

    # ------------------------------------------------------------------
    # Lifecycle：阻塞 run（直到 Ctrl+C）
    # ------------------------------------------------------------------
    def run(self) -> None:
        print("[v11_bridge] 运行中。Ctrl+C 退出...")
        try:
            while not self._closing.is_set():
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        self._closing.set()
        try:
            if self.angle_ctrl is not None:
                self.angle_ctrl.stop_all()
        except Exception:
            pass
        try:
            if self.base_controller is not None and hasattr(self.base_controller, "close"):
                self.base_controller.close()
        except Exception:
            pass
        try:
            if self._node is not None:
                self._node.destroy_node()
            if self._rclpy is not None and self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass
        print("[v11_bridge] 已退出。")


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="v11 ROS Topic 控制桥（独立脚本，零修改原 v11 程序）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # --offline 支持两种传法：
    #   1) --offline            (单独开关，等价于 offline=True)
    #   2) --offline true/false (显式值，兼容 ros2 launch offline:=true / false)
    p.add_argument("--offline", nargs="?", const="true", default="false",
                   choices=["0", "1", "true", "false", "TRUE", "FALSE", "True", "False", "on", "off", "yes", "no"],
                   help="离线模式：不连接 CAN 串口，只打印下发目标。不传值等价于 true。")
    p.add_argument("--can-port", default="/dev/ttyUSB_Controller",
                   help="v4 CAN 控制串口设备路径（与 multimodal_gui.py 一致）")
    p.add_argument("--can-baud", type=int, default=115200,
                   help="CAN 串口波特率")
    p.add_argument("--default-bucket-guess-deg", type=float, default=-45.0,
                   help="/target_point IK 时的默认铲斗绝对倾角(°)，挖掘常用 -45°~-60°")
    return p


def _str_to_bool(v: str) -> bool:
    if v is None:
        return False
    s = str(v).strip().lower()
    return s in ("1", "true", "on", "yes")


def main() -> None:
    args = _build_arg_parser().parse_args()
    offline_flag = _str_to_bool(args.offline) or False
    bridge = V11RosControlBridge(
        can_port=args.can_port,
        can_baud=args.can_baud,
        offline=offline_flag,
        default_bucket_guess_deg=args.default_bucket_guess_deg,
    )
    print(f"[v11_bridge] 启动参数解析：--offline={args.offline!r} → offline_flag={offline_flag}")
    bridge.run()


if __name__ == "__main__":
    main()
