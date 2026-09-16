#!/usr/bin/env python3
"""
V15 ROS 2 验证节点（URDF + rospub 测试专用）
=============================================
一句话启动：
    source /opt/ros/humble/setup.bash
    ros2 launch shandong_v14_urdf display.launch.py headless:=true use_joint_state_publisher:=false &
    python3 ex16_ros_urdf_control_node.py [--config /path/to.yaml] [--strategy lerp|trapezoidal]

功能：
  1) 手动 rospub 控制单一关节/整体位姿（走 v15 三层限位夹紧 + URDF 同步）
  2) 订阅空间目标点 (geometry_msgs/PointStamped)，触发：
        可达域预检 → IK 求解 → 双策略轨迹规划 → 逐路点下发执行
     同时把结果发布到 ROS 话题，RViz 可视化：
        - /v15/planned_path              (nav_msgs/Path)              完整规划的铲尖轨迹（Line）
        - /v15/target_point              (visualization_msgs/Marker)  目标点：绿=可达、红=超限
        - /v15/closest_point             (visualization_msgs/Marker)  最近可达收缩点（若超限才显示）
        - /v15/execution_waypoints       (sensor_msgs/JointState)     执行过程回放（可选 history size）
        - /v15/reachability_reasons      (std_msgs/String)            不可达时的失败原因列表
"""

from __future__ import annotations

import os
import sys
import time
import threading
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

# ── V15 导入兜底 ─────────────────────────────────────────────────
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(HERE, ".."))
try:
    from examples._util_import_helper import ensure_v15_importable
except (ImportError, ModuleNotFoundError):
    from _util_import_helper import ensure_v15_importable  # type: ignore
ensure_v15_importable()

from v15_action_task import (
    V15Config,
    CartesianMover,
    ForwardKinematics,
    MoveResult,
    WorkspaceChecker,
    from_config,
    load_default_config,
    rad_to_deg,
)


# ── 话题常量（与 README 清单严格一致，不可随意改）────────────────────
@dataclass(frozen=True)
class RosTopics:
    # === 手动 rospub 测试：输入到 v15 ===
    MANUAL_JOINT_CMD = "/v15/cmd/joint_states"      # sensor_msgs/JointState  手动目标位姿(弧度)
    MANUAL_TARGET_POINT = "/v15/cmd/target_point"   # geometry_msgs/PointStamped  用户 rospub 的空间点(m, frame_id=base_link)

    # === v15 → RViz：可视化输出 ===
    JOINT_FEEDBACK = "/joint_states"                # sensor_msgs/JointState  发给 URDF 的当前位姿（弧度）
    PLANNED_PATH = "/v15/planned_path"              # nav_msgs/Path              完整规划的铲尖轨迹
    TARGET_POINT_MARKER = "/v15/target_point"       # visualization_msgs/Marker 目标点（绿=OK / 红=FAIL）
    CLOSEST_POINT_MARKER = "/v15/closest_point"     # visualization_msgs/Marker 最近可达点（若超限）
    EXECUTION_WAYPOINTS = "/v15/execution_waypoints"  # sensor_msgs/JointState 执行过程（关节级回放）
    REACHABILITY_REASONS = "/v15/reachability_reasons"  # std_msgs/String       失败原因（JSON）


# ==================================================================
# 核心 ROS 节点
# ==================================================================
class V15RosControlNode:
    def __init__(
        self,
        cfg_path: Optional[str] = None,
        strategy: str = "lerp",
        mode: str = "transit",
        bucket_guess_deg: float = -45.0,
    ) -> None:
        self.strategy = strategy
        self.mode = mode
        self.bucket_guess_deg = bucket_guess_deg
        self._lock = threading.Lock()

        # 1) V15 标准工厂（ros 后端 → 直接接 URDF）
        #    NOTE: from_config 第一个位置参数 cfg = None / str(path) / V15Config / dict；
        #          ROS 后端必须传 start_adapter=True 才会立刻调 adapter.open() 发话题。
        self.ctx: Dict[str, Any] = from_config(
            cfg_path,
            adapter_backend="ros",
            start_adapter=True,
            init_workspace=True,
            init_trajectory_planner=True,
        )
        self.cfg: V15Config = self.ctx["config"]
        self.ctl = self.ctx["controller"]
        self.fk: ForwardKinematics = self.ctx["fk"]
        self.ws: WorkspaceChecker = self.ctx["workspace_checker"]
        self.mover: CartesianMover = self.ctx["mover"]
        self.mover.set_workspace_checker(self.ws)
        self.mover.set_trajectory_planner(self.ctx["trajectory_planner"])

        self.base_frame: str = getattr(self.cfg.ros, "frame_id", "base_link")

        # 2) ROS 基础
        #    Fix1: 强制 ROS 日志写到 /tmp/v15_ros_logs，避免 ~/.ros/log 不存在/无权限时报:
        #        RuntimeError: Failed opening file ~/.ros/log/...log for writing: Permission denied
        #    （必须在 import rclpy 之前设置，rclpy 首次 import 就会创建日志句柄）
        import os as _os
        _ros_log_dir = _os.environ.get("ROS_LOG_DIR") or ""
        if not _ros_log_dir or not _os.path.isdir(_ros_log_dir) or not _os.access(_ros_log_dir, _os.W_OK):
            _fallback = "/tmp/v15_ros_logs"
            _os.makedirs(_fallback, exist_ok=True)
            _os.environ["ROS_LOG_DIR"] = _fallback
            _ros_log_dir = _fallback
        #    Fix2: 禁用 ROS 2 信号处理 + ROS_HOME 兜底（sandbox 下 HOME=/root 只读时 ~/.ros 失败）
        _ros_home = _os.environ.get("ROS_HOME") or ""
        if not _ros_home:
            _os.environ["ROS_HOME"] = _ros_log_dir

        import rclpy  # type: ignore
        from rclpy.node import Node  # type: ignore
        #    Fix3: init(args=[]) 阻止 rclpy 解析我们的自定义 CLI 参数（--strategy/--mode 等）
        if not rclpy.ok():
            rclpy.init(args=[])
        self._rclpy = rclpy
        self._node = Node("v15_ros_control_node")
        self._log = self._node.get_logger()

        # 3) Publishers（可视化输出）
        self._pub_publishers: Dict[str, Any] = {}
        self._create_pubs_and_subs()

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        self._log.info(
            f"[ex16] 启动完成. strategy={self.strategy}, mode={self.mode}, "
            f"base_frame={self.base_frame}"
        )
        self._log.info(
            "[ex16] 话题清单:\n"
            "  手动 rospub 输入:\n"
            f"    - {RosTopics.MANUAL_JOINT_CMD}   (sensor_msgs/JointState, 弧度)\n"
            f"    - {RosTopics.MANUAL_TARGET_POINT}(geometry_msgs/PointStamped, 米, frame=base_link)\n"
            "  RViz 可视化订阅:\n"
            f"    - {RosTopics.PLANNED_PATH}\n"
            f"    - {RosTopics.TARGET_POINT_MARKER}\n"
            f"    - {RosTopics.CLOSEST_POINT_MARKER}\n"
            f"    - {RosTopics.EXECUTION_WAYPOINTS}\n"
            f"    - {RosTopics.REACHABILITY_REASONS}\n"
        )

    # ── 话题创建 ──────────────────────────────────────────────────
    def _create_pubs_and_subs(self) -> None:
        from sensor_msgs.msg import JointState  # type: ignore
        from geometry_msgs.msg import PointStamped  # type: ignore
        from nav_msgs.msg import Path  # type: ignore
        from visualization_msgs.msg import Marker  # type: ignore
        from std_msgs.msg import String  # type: ignore
        from rclpy.qos import QoSProfile, HistoryPolicy  # type: ignore

        # ROS 2 Humble API：qos history=HistoryPolicy.KEEP_LAST；Iron/Jazzy 也都兼容此写法
        qos10 = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST)
        qos1 = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST)

        P = self._node.create_publisher
        self._pub_path = P(Path, RosTopics.PLANNED_PATH, qos10)
        self._pub_tgt = P(Marker, RosTopics.TARGET_POINT_MARKER, qos1)
        self._pub_closest = P(Marker, RosTopics.CLOSEST_POINT_MARKER, qos1)
        self._pub_exec = P(JointState, RosTopics.EXECUTION_WAYPOINTS, qos10)
        self._pub_reasons = P(String, RosTopics.REACHABILITY_REASONS, qos1)

        S = self._node.create_subscription
        S(JointState, RosTopics.MANUAL_JOINT_CMD, self._on_manual_joint_cmd, qos10)
        S(PointStamped, RosTopics.MANUAL_TARGET_POINT, self._on_target_point, qos10)
        self._JointState = JointState
        self._Marker = Marker
        self._Path = Path
        self._String = String

    def _spin(self) -> None:
        try:
            self._rclpy.spin(self._node)
        except Exception:
            pass

    # ── 工具：FK 算铲尖 3D ───────────────────────────────────────
    def _tip_from_pose_deg(self, pose_deg: Dict[str, float]) -> Tuple[float, float, float]:
        sol = self.fk.solve(
            swing_yaw_deg=pose_deg.get("swing_yaw", 0.0),
            boom_swing_deg=pose_deg.get("boom_swing", 0.0),
            arm_boom_deg=pose_deg.get("arm_boom", 0.0),
            bucket_arm_deg=pose_deg.get("bucket_arm", 0.0),
        )
        return sol.bucket_tip_3d  # (x, y, z) 单位米，base_link

    # ── 功能 1：手动 rospub 位姿 ───────────────────────────────────
    def _on_manual_joint_cmd(self, msg) -> None:
        try:
            import math as _math
            pose_deg: Dict[str, float] = {}
            from v15_action_task.control_core.types import URDF_TO_SEMANTIC, URDF_JOINT_ORDER
            name_to_rad = {}
            for idx, n in enumerate(list(msg.name)):
                try:
                    name_to_rad[n] = float(msg.position[idx])
                except Exception:
                    pass
            for urdf_name in URDF_JOINT_ORDER:
                if urdf_name in name_to_rad and urdf_name in URDF_TO_SEMANTIC:
                    pose_deg[URDF_TO_SEMANTIC[urdf_name]] = rad_to_deg(name_to_rad[urdf_name])
            if not pose_deg:
                return
            with self._lock:
                clamped = self.ctl._clamp_pose({k: float(v) for k, v in pose_deg.items()})
                ok_pub = self.ctl.set_pose(clamped)
                if ok_pub:
                    deadline = _time_now = time.time() + 3.0
                    while time.time() < deadline:
                        if self.ctl.is_at_pose(clamped, tolerance_deg=1.0):
                            break
                        time.sleep(0.05)
            self._log.info(
                f"[manual] 应用目标位姿（度）: "
                f"swing={pose_deg.get('swing_yaw', float('nan')):.1f} "
                f"boom={pose_deg.get('boom_swing', float('nan')):.1f} "
                f"arm={pose_deg.get('arm_boom', float('nan')):.1f} "
                f"bucket={pose_deg.get('bucket_arm', float('nan')):.1f}"
            )
        except Exception as e:
            self._log.error(f"[manual] 应用失败: {type(e).__name__}: {e}")

    # ── 功能 2：订阅空间点 → 规划 + 执行 + 发布可视化 ────────────────
    def _on_target_point(self, msg) -> None:
        try:
            xyz = (float(msg.point.x), float(msg.point.y), float(msg.point.z))
            frame = msg.header.frame_id or self.base_frame
            self._log.info(
                f"[plan] 收到目标点 frame={frame!r}, xyz=({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f})m"
            )
        except Exception as e:
            self._log.error(f"[plan] 解析 PointStamped 失败: {e}")
            return

        # 可达域预检
        rep = self.ws.check_point_reachable(xyz, mode=self.mode)
        if not rep.success:
            self._log.warn(f"[plan] 不可达，原因: {rep.reasons}")
            self._publish_target_marker(xyz, reachable=False)
            if rep.closest_point is not None:
                self._publish_closest_marker(rep.closest_point)
            self._publish_reasons(rep.reasons)
            return

        self._publish_target_marker(xyz, reachable=True)
        self._publish_closest_clear()
        self._publish_reasons(["OK: reachable"])

        # 触发 move_to_point（阻塞式；轨迹内部已按 step_duration_s 下发）
        try:
            with self._lock:
                res: MoveResult = self.mover.move_to_point(
                    xyz,
                    mode=self.mode,
                    bucket_guess_deg=self.bucket_guess_deg,
                    strategy=self.strategy,
                    execute=True,
                    blocking=True,
                )
        except Exception as e:
            self._log.error(f"[plan] move_to_point 异常: {e}")
            return

        if not res.success:
            self._log.warn(f"[plan] 执行失败: {res.reason}")
            self._publish_reasons([f"EXEC_FAIL: {res.reason}"])
            return

        # 发布完整规划轨迹（从 waypoints FK 算每一步铲尖 → Path）
        self._publish_planned_path(res.trajectory or [])
        # 顺便再发一次执行过程（如果有 joint_trajectories）
        if res.joint_trajectories:
            self._publish_execution_waypoints(res.trajectory or [])
        self._log.info(
            f"[plan] 完成: success={res.success}, "
            f"waypoints={len(res.trajectory) if res.trajectory else 0}, "
            f"final_tip={tuple(round(v, 3) for v in (res.final_tip_xyz or (0,0,0)))}"
        )

    # ── 发布：Path 完整铲尖轨迹 ───────────────────────────────────
    def _publish_planned_path(self, waypoints: List[Dict[str, float]]) -> None:
        from geometry_msgs.msg import PoseStamped  # type: ignore
        if not waypoints:
            return
        path = self._Path()
        path.header.frame_id = self.base_frame
        path.header.stamp = self._node.get_clock().now().to_msg()
        for pose_deg in waypoints:
            (x, y, z) = self._tip_from_pose_deg(pose_deg)
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(z)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._pub_path.publish(path)

    # ── 发布：目标点 Marker（绿 OK / 红 FAIL）───────────────────────
    def _publish_target_marker(self, xyz: Tuple[float, float, float], reachable: bool) -> None:
        m = self._Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self._node.get_clock().now().to_msg()
        m.ns = "v15_target"
        m.id = 0
        m.type = self._Marker.SPHERE
        m.action = self._Marker.ADD
        m.pose.position.x = float(xyz[0])
        m.pose.position.y = float(xyz[1])
        m.pose.position.z = float(xyz[2])
        m.pose.orientation.w = 1.0
        s = 0.06
        m.scale.x = m.scale.y = m.scale.z = s
        m.color.r = 0.0 if reachable else 1.0
        m.color.g = 1.0 if reachable else 0.0
        m.color.b = 0.0
        m.color.a = 0.9
        self._pub_tgt.publish(m)

    def _publish_closest_marker(self, xyz: Tuple[float, float, float]) -> None:
        m = self._Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self._node.get_clock().now().to_msg()
        m.ns = "v15_closest"
        m.id = 1
        m.type = self._Marker.SPHERE
        m.action = self._Marker.ADD
        m.pose.position.x = float(xyz[0])
        m.pose.position.y = float(xyz[1])
        m.pose.position.z = float(xyz[2])
        m.pose.orientation.w = 1.0
        s = 0.07
        m.scale.x = m.scale.y = m.scale.z = s
        m.color.r = 1.0
        m.color.g = 0.6
        m.color.b = 0.0
        m.color.a = 0.9
        self._pub_closest.publish(m)

    def _publish_closest_clear(self) -> None:
        m = self._Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = self._node.get_clock().now().to_msg()
        m.ns = "v15_closest"
        m.id = 1
        m.action = self._Marker.DELETE
        self._pub_closest.publish(m)

    # ── 发布：失败原因（JSON）────────────────────────────────────
    def _publish_reasons(self, reasons: List[str]) -> None:
        s = self._String()
        try:
            import json
            s.data = json.dumps({"reasons": reasons}, ensure_ascii=False)
        except Exception:
            s.data = ", ".join(reasons)
        self._pub_reasons.publish(s)

    # ── 发布：执行过程 waypoints（JointState 按顺序连续发布）──────
    def _publish_execution_waypoints(self, waypoints: List[Dict[str, float]]) -> None:
        from v15_action_task.control_core.types import (
            SEMANTIC_JOINT_ORDER, URDF_JOINT_ORDER, deg_to_rad,
        )
        for pose_deg in waypoints:
            js = self._JointState()
            js.header.frame_id = self.base_frame
            js.header.stamp = self._node.get_clock().now().to_msg()
            js.name = list(URDF_JOINT_ORDER)
            js.position = [deg_to_rad(pose_deg.get(s, 0.0)) for s in SEMANTIC_JOINT_ORDER]
            self._pub_exec.publish(js)
            time.sleep(0.002)  # 避免瞬间发送淹没 rviz

    # ── 生命周期 ─────────────────────────────────────────────────
    def spin_forever(self) -> None:
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            self._log.info("[ex16] Ctrl+C 收到，退出")
        finally:
            # RosV14Adapter 已通过 from_config(start_adapter=True) 打开；这里按 URDFController 的 close 流程收尾
            try:
                self.ctl.close()
            except Exception:
                pass
            try:
                if self._rclpy.ok():
                    self._rclpy.shutdown()
            except Exception:
                pass


def _parse_args(argv: List[str]) -> Dict[str, Any]:
    out = {"config": None, "strategy": "lerp", "mode": "transit", "bucket_guess": -45.0}
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in ("-c", "--config") and i + 1 < len(argv):
            out["config"] = argv[i + 1]; i += 2
        elif a in ("-s", "--strategy") and i + 1 < len(argv):
            out["strategy"] = argv[i + 1]; i += 2
        elif a in ("-m", "--mode") and i + 1 < len(argv):
            out["mode"] = argv[i + 1]; i += 2
        elif a in ("-b", "--bucket-guess") and i + 1 < len(argv):
            out["bucket_guess"] = float(argv[i + 1]); i += 2
        elif a in ("-h", "--help"):
            print("Usage: python3 ex16_ros_urdf_control_node.py [--config PATH] [--strategy lerp|trapezoidal] [--mode transit|dig]")
            sys.exit(0)
        else:
            i += 1
    return out


def main() -> int:
    # ── 0. 启动环境自检（3 秒自查，避免 Python 版本 / ROS 环境不匹配导致的玄学报错）────────
    import sys as _sys
    PY_OK = True
    print("[ex16] ================ 启动环境自检 ================", flush=True)
    print(f"[ex16] sys.executable      = {_sys.executable}", flush=True)
    print(f"[ex16] Python 版本          = {_sys.version.split()[0]}", flush=True)
    print(f"[ex16] 期望解释器            = /usr/bin/python3 (3.10.*, 对应 ROS Humble)", flush=True)
    if _sys.executable != "/usr/bin/python3":
        print(
            "[ex16] ⚠️  警告：当前 python3 不是 /usr/bin/python3！\n"
            "[ex16]     若你后续出现 ModuleNotFound rclpy / _rclpy_pybind11 ABI 不兼容，\n"
            "[ex16]     请执行：  export PATH=\"/usr/bin:$PATH\"  &&  hash -r  &&  which python3\n"
            "[ex16]     确认输出 /usr/bin/python3 后再重新跑 ex16",
            file=_sys.stderr, flush=True,
        )
        PY_OK = False
    if _sys.version_info[:2] != (3, 10):
        print(
            f"[ex16] ⚠️  警告：Python {_sys.version_info.major}.{_sys.version_info.minor} != 3.10\n"
            "[ex16]     ROS 2 Humble 的 rclpy 是按 3.10 编译的 C 扩展，非 3.10 必崩 _rclpy_pybind11",
            file=_sys.stderr, flush=True,
        )
        PY_OK = False
    try:
        import rclpy as _rclpy  # noqa: F401
        print(f"[ex16] rclpy 可导入        = OK（来自 {_rclpy.__file__}）", flush=True)
    except Exception as _e:
        print(
            f"[ex16] ❌ rclpy 导入失败: {type(_e).__name__}: {_e}\n"
            "[ex16]     解决: source /opt/ros/humble/setup.bash  再确认 which python3=/usr/bin/python3",
            file=_sys.stderr, flush=True,
        )
        PY_OK = False
    print(f"[ex16] 自检结论              = {'✅ 通过（可继续 ROS 节点初始化）' if PY_OK else '❌ 未通过（后续会报错，先修上面红字）'}", flush=True)
    print("[ex16] =================================================", flush=True)

    args = _parse_args(sys.argv[1:])
    node = V15RosControlNode(
        cfg_path=args["config"],
        strategy=args["strategy"],
        mode=args["mode"],
        bucket_guess_deg=args["bucket_guess"],
    )
    node.spin_forever()
    return 0


if __name__ == "__main__":
    sys.exit(main())
