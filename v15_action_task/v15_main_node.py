#!/usr/bin/env python3
"""
V15 Action Task —— 最简主入口（ROS 2 节点）
==========================================
一句话启动：
    source /opt/ros/humble/setup.bash
    source /path/to/ws/install/setup.bash
    cd v15_action_task
    python3 v15_main_node.py                       # 默认 lerp + transit
    python3 v15_main_node.py -s trapezoidal -m dig  # 梯形速度 + 挖掘模式

说明（满足用户要求的极简设计）：
  - 本文件是 V15 "主函数入口"，不做任何可视化（不发 Path / Marker 等）
  - 只连 ROS2，订阅 2 条用户手动 pub 的指令，执行结果通过 RosV14Adapter 发到
      /joint_states （URDF display launch 会自动订阅它来驱动机器人）

订阅话题（用户手动 ros2 topic pub）：
  1) /v15/cmd/joint_states     sensor_msgs/JointState
        直接写目标关节（弧度），会走 v15 三层限位夹紧后驱动 URDF
        name 顺序 = [swing_joint, boom_joint, arm_joint, bucket_joint]
  2) /v15/cmd/target_point     geometry_msgs/PointStamped  (frame_id=base_link, 单位米)
        笛卡尔空间点 → 可达域预检 → IK → LERP/Trap 轨迹 → 逐路点下发
"""

from __future__ import annotations

import atexit
import os
import queue
import signal
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

# 保证以 `python3 v15_main_node.py` 方式运行时可以 import v15_action_task 子模块
_HERE = os.path.dirname(os.path.abspath(__file__))
_V15_PARENT = os.path.dirname(_HERE)
if _V15_PARENT not in sys.path:
    sys.path.insert(0, _V15_PARENT)

from v15_action_task import (  # noqa: E402
    CartesianMover,
    MoveResult,
    WorkspaceChecker,
    from_config,
    rad_to_deg,
)


TOPIC_MANUAL_JOINT_CMD = "/v15/cmd/joint_states"   # I1 手动位姿输入
TOPIC_MANUAL_TARGET_PT = "/v15/cmd/target_point"   # I2 空间点输入
TOPIC_URDF_JOINT_STATES = "/joint_states"          # O1 发给 URDF 的执行结果


class V15MainNode:
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

        # ⭐ P0 根因修复：Executor 单线程被点控回调阻塞 → 角度控制也饿死。
        #    架构：_on_target_point 回调 ≈ 把消息「压入一个 queue」就立刻返回（通常 <1ms）
        #           后台永久 worker 线程串行地 queue.get() → 跑 preview → execute → 清理
        #           这样就算 move_to_point 内部死循环 1 小时，角度控制回调照样能被 Executor 调度到，
        #           因为 Executor 线程根本不会在点控里停留 >1ms。
        #    同时天然串行：单 worker + queue，不会有并发，不需要 _move_running Event 防死锁。
        #    防连点：queue 深度上限 1（当前 worker 在处理时，新消息直接丢弃，提示 [cmd/busy] 等当前执行完）。
        self._tp_queue: "queue.Queue[Dict[str, Any]]" = queue.Queue(maxsize=1)
        self._tp_worker_closing = threading.Event()
        self._tp_worker_thread: Optional[threading.Thread] = None

        # Path 持续发布线程状态（T1 可视化：执行中 10Hz 刷 stamp，RViz 不会丢 Path；执行完立刻停）
        self._path_rwlock = threading.Lock()
        self._path_active = threading.Event()
        self._path_pose_deg_list: Optional[List[Dict[str, float]]] = None
        self._path_frame_id: Optional[str] = None
        self._path_progress_idx: int = 0  # 当前执行到的 waypoint 下标（0-based，用于黄球进度可视化）
        self._pub_progress: Optional[Any] = None

        # 1) 标准工厂：ROS 后端立刻 open，workspace + trajectory 都开
        self.ctx: Dict[str, Any] = from_config(
            cfg_path,
            adapter_backend="ros",
            start_adapter=True,
            init_workspace=True,
            init_trajectory_planner=True,
        )
        self.ctl = self.ctx["controller"]
        self.ws: Optional[WorkspaceChecker] = self.ctx.get("workspace_checker")
        self.mover: CartesianMover = self.ctx["mover"]
        if self.ws is not None:
            self.mover.set_workspace_checker(self.ws)
        tp = self.ctx.get("trajectory_planner")
        if tp is not None:
            self.mover.set_trajectory_planner(tp)
        self.base_frame = getattr(self.ctx["config"].ros, "frame_id", "base_link")
        self.cfg = self.ctx["config"]
        self.fk = self.ctx["fk"]
        self.ik = self.ctx["ik"]

        # 2) ROS2（★ 关键修复：绝对不要自己再 rclpy.init + 开第二个 Node！）
        #    ROS 2 Humble 单进程里两个 Node 各自 rclpy.init 会让 DDS publisher 静默空发。
        #    正确做法：复用 RosV14Adapter 里已经创建好的单一 Node。所有 pub/sub 全挂在它上面。
        try:
            self.adapter = self.ctl.adapter   # URDFController 自带 @property adapter
        except Exception:
            self.adapter = None
        if self.adapter is None:
            # 兜底：旧版 ctl 没暴露 adapter，去顶层 ctx 找 adapter（from_config 返回里就有）
            self.adapter = self.ctx.get("adapter")
        if self.adapter is None or not hasattr(self.adapter, "_node") or self.adapter._node is None:
            raise RuntimeError(
                "无法拿到 RosV14Adapter 的 Node。请先确认 adapter_backend='ros' 且 "
                "start_adapter=True（from_config 已默认 start_adapter=True 当 backend=ros）"
            )
        self._rclpy = self.adapter._rclpy
        self._node = self.adapter._node   # ⭐ 复用 Adapter 的唯一 Node
        self._log = self._node.get_logger()

        from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy  # type: ignore
        # ★ 关键修复：2 条订阅 + 所有可视化发布 必须使用与 ros2 topic pub 默认完全兼容的 QoS
        #   ros2 topic pub 默认 Reliability=RELIABLE + Durability=VOLATILE + depth=10 KEEP_LAST
        #   之前 target_point 误写 depth=1 + 没有显式 Reliability/Durability，会在多进程或存在残留时 DDS 匹配失败
        #   → 所有输入订阅 + 输出发布统一用 self._qos（RELIABLE+VOLATILE+KEEP_LAST+depth=10），100% 对齐 ros2 CLI 默认
        self._qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 3) 消息类型 + 订阅 2 条指令（挂在 Adapter 的同一个 Node 上，DDS 保证能发能收）
        from sensor_msgs.msg import JointState  # type: ignore
        from geometry_msgs.msg import PointStamped, PoseStamped  # type: ignore
        from nav_msgs.msg import Path  # type: ignore
        from visualization_msgs.msg import Marker  # type: ignore
        from std_msgs.msg import ColorRGBA, Header, String as StdMsgsString  # type: ignore
        from geometry_msgs.msg import Point  # type: ignore
        self._JointState = JointState
        self._PointStamped = PointStamped
        self._PoseStamped = PoseStamped
        self._Path = Path
        self._Marker = Marker
        self._ColorRGBA = ColorRGBA
        self._Header = Header
        self._StdMsgsString = StdMsgsString
        self._Point = Point
        _sub_joint = self._node.create_subscription(
            JointState, TOPIC_MANUAL_JOINT_CMD,
            self._on_joint_cmd, self._qos,
        )
        _sub_target = self._node.create_subscription(
            PointStamped, TOPIC_MANUAL_TARGET_PT,
            self._on_target_point, self._qos,
        )
        self._log.info(
            f"[sub OK] 2 条输入已注册（统一 QoS: RELIABLE/VOLATILE/KEEP_LAST depth=10）：\n"
            f"  {TOPIC_MANUAL_JOINT_CMD}  (state_joint, sensor_msgs/JointState) -> handler={_sub_joint is not None}\n"
            f"  {TOPIC_MANUAL_TARGET_PT}  (target_point, geometry_msgs/PointStamped) -> handler={_sub_target is not None}"
        )

        # 3.1) 新增：4 个可视化话题发布器（RViz 看轨迹/目标球/最近可达球/执行回放）
        # ★ 所有发布器统一 QoS (RELIABLE/VOLATILE/KEEP_LAST depth=10)，100% 对齐 ros2 CLI 默认，避免 DDS 静默不匹配
        self._pub_path = self._node.create_publisher(Path, "/v15/planned_path", self._qos)
        self._pub_tgt_marker = self._node.create_publisher(Marker, "/v15/target_point", self._qos)
        self._pub_closest_marker = self._node.create_publisher(Marker, "/v15/closest_point", self._qos)
        self._pub_tip_marker = self._node.create_publisher(Marker, "/v15/current_tip", self._qos)
        self._pub_progress = self._node.create_publisher(Marker, "/v15/progress_tip", self._qos)
        self._pub_wp = self._node.create_publisher(JointState, "/v15/execution_waypoints", self._qos)
        self._pub_reason = self._node.create_publisher(
            StdMsgsString,
            "/v15/reachability_reasons", self._qos,
        )

        # 4) spin 已经在 Adapter 后台启动了；这里额外开个 5s 自检日志，方便查"URDF 为啥不动"
        self._prev_pub_ok = 0
        self._prev_pub_fail = 0
        threading.Thread(target=self._health_check_loop, daemon=True).start()
        # 4.1) 每 1s 刷一次当前末端 FK 蓝色球（用户要求的"末端运动终端小球对齐"）
        threading.Thread(target=self._current_tip_visualizer_loop, daemon=True).start()
        # 4.2) ★ Path 持续发布后台线程（10Hz stamp 刷新 + 进度黄球）—— 用户要求运动过程中持续发布，结束停
        threading.Thread(target=self._path_stream_daemon, daemon=True).start()

        # 5) workspace 边界日志（对应 T3：下挖最低点等边界提示用户可改）
        try:
            ws_cfg = self.cfg.workspace
            ws_outer = getattr(self.ws, "outer_radius_m", None) or "?"
            ws_inner = getattr(self.ws, "inner_radius_m", None) or "?"
            self._log.info(
                f"[v15_workspace] 可达域边界：下挖最低点 min_ground_z={getattr(ws_cfg,'min_ground_z','?')} m  "
                f"(想下挖 1m 改成 -1.00，见 config/default_config.yaml workspace 节)\n"
                f"[v15_workspace]   转运最小离地 min_transit_z={getattr(ws_cfg,'min_transit_z','?')} m  "
                f"腕点半径 inner≈{ws_inner}m  outer≈{ws_outer}m  奇异 margin={getattr(ws_cfg,'singularity_margin_min_m','?')}m"
            )
        except Exception as e:
            self._log.warn(f"[v15_workspace] 边界日志失败: {e}")

        self._log.info(
            f"[v15_main] 启动完成：strategy={self.strategy}, mode={self.mode}, "
            f"frame={self.base_frame}, ROS Node=/{self._node.get_name()}"
        )
        self._log.info(
            f"[v15_main] 订阅 2 条输入话题：\n"
            f"  手动位姿  → {TOPIC_MANUAL_JOINT_CMD}  (sensor_msgs/JointState, 弧度)\n"
            f"  空间点    → {TOPIC_MANUAL_TARGET_PT}  (PointStamped, m, frame=base_link)\n"
            f"[v15_main] 输出到 URDF + RViz 可视化 8 条话题：\n"
            f"  关节状态 → {TOPIC_URDF_JOINT_STATES}  (robot_state_publisher 自动订阅，URDF 本体)\n"
            f"  ★完整轨迹→ /v15/planned_path              (Path，10Hz 持续刷新直到运动结束，★运动过程中一直有)\n"
            f"  ★目标点   → /v15/target_point              (Marker SPHERE，绿=可达/红=不可达)\n"
            f"  ★最近可达 → /v15/closest_point             (Marker SPHERE，橙，仅不可达时出现)\n"
            f"  ★当前末端 → /v15/current_tip               (Marker SPHERE，蓝=实际 FK 铲尖位置，对齐末端)\n"
            f"  ★执行进度 → /v15/progress_tip              (Marker SPHERE，🟡黄球在 Path 上滚动，显示运动到第几步)\n"
            f"  关节回放 → /v15/execution_waypoints    (JointState，waypoint 级单关节过程)\n"
            f"  失败原因 → /v15/reachability_reasons   (String JSON，5 层失败原因)\n"
        )
        self._log.info(
            "[v15_main] 若 URDF 不动，请先看下面每 5s 一次的心跳日志："
            "  publish_delta≈50 → DDS 已发；ros2 topic echo /joint_states 应该能看到数据\n"
            "[v15_main] RViz 看不到轨迹/小球：请先确认 Fixed Frame=base_link；再按下面 6 条 Ctrl+A Add → By topic 逐一加上：\n"
            "   Path(/v15/planned_path) + Marker(/v15/target_point) + Marker(/v15/closest_point) + Marker(/v15/current_tip) + Marker(/v15/progress_tip) + RobotModel。"
        )

        # ★ P0 修复：注册 SIGINT + atexit 双保险资源清理，解决「每次 Ctrl+C 后必须 pkill 残留进程」
        #   - signal.SIGINT：Ctrl+C 主路径（同步立即清理，100ms 内 destroy_node + shutdown）
        #   - atexit.register：兜底（例如 kill -TERM、解释器退出、未捕获异常等场景也会调）
        #   - __cleanup_done 防重复 cleanup 触发 destroy_node on already-destroyed 死锁
        self.__cleanup_done = False
        self.__cleanup_lock = threading.Lock()
        try:
            signal.signal(signal.SIGINT, self._sigint_handler)
        except (ValueError, OSError):
            pass  # 非主线程里注册会 ValueError，忽略（此时 spin_forever 会 KeyboardInterrupt 兜底）
        try:
            signal.signal(signal.SIGTERM, self._sigint_handler)
        except (ValueError, OSError):
            pass
        atexit.register(self._cleanup_exit)
        self._log.info(
            "[cleanup OK] SIGINT/SIGTERM + atexit 双清理已注册。Ctrl+C 后会立刻 destroy_node + rclpy.shutdown，"
            "不再需要手动 pkill。"
        )

        # ⭐ 启动后台点控 worker 单线程（Executor 线程永不被点控阻塞，角度控制永远不会被牵连饿死）
        self._tp_worker_thread = threading.Thread(
            target=self._tp_worker_loop,
            name="v15_tp_worker",
            daemon=True,
        )
        self._tp_worker_thread.start()
        self._log.info(
            "[tp_worker OK] 点控已迁移到后台单线程（queue maxsize=1，串行化执行）。\n"
            "  - 点控发命令：回调<1ms 压完 queue 就返回，Executor 永不阻塞；\n"
            "  - 角度控制：即使点控内部 move_to_point 卡死 N 小时，手动关节指令照样立刻生效；\n"
            "  - 防连点：worker 在忙时新点控直接丢弃，打印 [cmd/busy] 请等当前执行结束。"
        )

    # ── 后台点控 worker：单线程串行化处理所有 target_point（Executor 永不被阻塞） ──
    def _tp_worker_loop(self) -> None:
        while not self._tp_worker_closing.is_set():
            try:
                job = self._tp_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._process_target_point_job(job)
            except Exception as e:
                tb = self._format_traceback()
                try:
                    self._log.error(f"[tp_worker] job 异常（已隔离，不影响后续角度控制）：{type(e).__name__}: {e}\n" + ("-"*32) + "\n" + tb + ("-"*32))
                except Exception:
                    pass
                # 异常也停 Path，别让 RViz 一直亮着
                try:
                    self._path_stream_stop()
                except Exception:
                    pass
            finally:
                try:
                    self._tp_queue.task_done()
                except Exception:
                    pass
        try:
            self._log.info("[tp_worker] worker 线程正常退出（closing event 触发）")
        except Exception:
            pass
        return None

    # ── P0 修复：Ctrl+C / kill 退出时 100% 回收 DDS 资源，杜绝进程残留 ──
    def _sigint_handler(self, signum, frame) -> None:
        """同步信号处理（Ctrl+C / kill -2 / kill -15）。必须：立刻打印 -> cleanup -> 不抛异常。"""
        import sys as _s
        try:
            signame = signal.Signals(signum).name
        except Exception:
            signame = f"SIG_{signum}"
        try:
            print(f"\n[v15_main] 收到 {signame}，开始同步清理资源…", file=_s.stderr, flush=True)
        except Exception:
            pass
        try:
            self._log.info(f"[cleanup] signal={signame} 触发同步清理")
        except Exception:
            pass
        self._do_cleanup(force=True)
        try:
            _s.exit(128 + int(signum))
        except Exception:
            os._exit(128 + int(signum))

    def _cleanup_exit(self) -> None:
        """atexit 兜底：进程退出路径（解释器正常结束 / 未捕获异常）。"""
        self._do_cleanup(force=False)

    def _do_cleanup(self, force: bool = False) -> None:
        """统一 cleanup 入口（线程安全、幂等）。
        顺序严格：① 停后台点控 worker（免得它还调 mover）→ ② 停 Path daemon → ③ ctl.close → ④ rclpy shutdown
        """
        with self.__cleanup_lock:
            if self.__cleanup_done:
                return
            self.__cleanup_done = True
        # ① 先停 tp_worker（给它 1s 退循环，否则 daemon 随解释器走）
        try:
            self._tp_worker_closing.set()
        except Exception:
            pass
        try:
            if self._tp_worker_thread is not None and self._tp_worker_thread.is_alive():
                self._tp_worker_thread.join(timeout=1.2)
        except Exception:
            pass
        # ② 停 Path 后台线程（避免它在 destroy_node 之后还引用 self._node）
        try:
            self._path_stream_stop()
        except Exception:
            pass
        try:
            self._path_active.clear()
        except Exception:
            pass
        # ctl.close() 会调 adapter.close()（里面按顺序 join 线程 + destroy_node + rclpy.shutdown）
        try:
            if hasattr(self, "ctl") and self.ctl is not None:
                self.ctl.close()
        except Exception:
            pass
        # 双重保险：即使 ctl.close() 里 rclpy.shutdown 因异常跳过，这里再补一次
        rclpy = getattr(self, "_rclpy", None)
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
        import sys as _s
        try:
            print("[v15_main] 清理完成：DDS Node/Context 已释放，无进程残留。", file=_s.stderr, flush=True)
        except Exception:
            pass
        return None

    def _health_check_loop(self) -> None:
        """每 5s 打一行 publish 计数增量，用户一眼知道 DDS 在不在发。"""
        import time as _t
        while True:
            _t.sleep(5.0)
            try:
                ok = getattr(self.adapter, "_publish_success_count", 0)
                fail = getattr(self.adapter, "_publish_fail_count", 0)
                d_ok = ok - self._prev_pub_ok
                d_fail = fail - self._prev_pub_fail
                self._prev_pub_ok = ok
                self._prev_pub_fail = fail
                cmd = getattr(self.adapter, "_cmd_deg", None) or {}
                s = round(float(cmd.get("swing_yaw", 0.0)), 1)
                b = round(float(cmd.get("boom_swing", 0.0)), 1)
                a = round(float(cmd.get("arm_boom", 0.0)), 1)
                k = round(float(cmd.get("bucket_arm", 0.0)), 1)
                self._log.info(
                    f"[v15_health] 5s heartbeat: publish_delta=+{d_ok} ok / +{d_fail} fail  "
                    f"total(ok={ok}, fail={fail})  cmd_deg(swing={s}, boom={b}, arm={a}, bucket={k})"
                )
            except Exception as e:
                self._log.error(f"[v15_health] check fail: {type(e).__name__}: {e}")

    def _spin(self) -> None:
        # Adapter 已经在它自己的 spin_thread 里 spin，这里保留空实现以免用户自己写 _spin 调用
        pass

    # ================= 可视化辅助（T1 轨迹/目标球/最近可达/当前末端） =================
    def _make_color(self, r: float, g: float, b: float, a: float = 0.9) -> Any:
        c = self._ColorRGBA()
        c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
        return c

    def _make_sphere_marker(self, xyz, *, color_rgba, ns: str, mid: int = 0, radius: float = 0.04, lifetime_s: float = 0.0) -> Any:
        m = self._Marker()
        m.header.frame_id = self.base_frame
        m.ns = ns
        m.id = int(mid)
        m.type = self._Marker.SPHERE
        m.action = self._Marker.ADD if all(v is not None for v in xyz) else self._Marker.DELETE
        m.pose.position.x = 0.0 if xyz[0] is None else float(xyz[0])
        m.pose.position.y = 0.0 if xyz[1] is None else float(xyz[1])
        m.pose.position.z = 0.0 if xyz[2] is None else float(xyz[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = float(radius)
        m.color = self._make_color(*color_rgba)
        if lifetime_s > 0:
            # ros2 Duration(seconds=x) -> rclpy Duration
            try:
                from builtin_interfaces.msg import Duration  # type: ignore
                import math as _math
                m.lifetime = Duration(sec=int(_math.floor(lifetime_s)),
                                      nanosec=int(round((lifetime_s - _math.floor(lifetime_s)) * 1e9)))
            except Exception:
                pass
        return m

    def _publish_path_from_pose_list(self, pose_deg_list, base_header, force_stamp=None) -> None:
        """把 move_to_point 返回的 trajectory_plan 每个 pose FK 成铲尖，串成 nav_msgs/Path 发出去。
        force_stamp: 可选覆盖 stamp，后台持续刷 timestamp 用（RViz Path 要求持续最新 stamp，否则会不显示）。
        """
        if self._pub_path is None or self.fk is None or not pose_deg_list:
            return
        path = self._Path()
        if force_stamp is not None:
            path.header.stamp = force_stamp
        else:
            path.header.stamp = base_header.stamp if hasattr(base_header, "stamp") else self._node.get_clock().now().to_msg()
        path.header.frame_id = self.base_frame if self._path_frame_id is None else str(self._path_frame_id)
        for pose_deg in pose_deg_list:
            ps = self._PoseStamped()
            ps.header.frame_id = path.header.frame_id
            ps.header.stamp = path.header.stamp
            try:
                tip = self.fk.solve(
                    boom_swing_deg=float(pose_deg.get("boom_swing", 0.0)),
                    arm_boom_deg=float(pose_deg.get("arm_boom", 0.0)),
                    bucket_arm_deg=float(pose_deg.get("bucket_arm", 0.0)),
                    swing_yaw_deg=float(pose_deg.get("swing_yaw", 0.0)),
                ).bucket_tip_3d
            except Exception:
                continue
            ps.pose.position.x = float(tip[0])
            ps.pose.position.y = float(tip[1])
            ps.pose.position.z = float(tip[2])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        try:
            self._pub_path.publish(path)
        except Exception:
            pass

    def _fk_pose_deg_to_xyz(self, pose_deg: Dict[str, float]) -> Optional[Tuple[float, float, float]]:
        if self.fk is None:
            return None
        try:
            return tuple(v for v in self.fk.solve(
                boom_swing_deg=float(pose_deg.get("boom_swing", 0.0)),
                arm_boom_deg=float(pose_deg.get("arm_boom", 0.0)),
                bucket_arm_deg=float(pose_deg.get("bucket_arm", 0.0)),
                swing_yaw_deg=float(pose_deg.get("swing_yaw", 0.0)),
            ).bucket_tip_3d)
        except Exception:
            return None

    # ===========================================================================
    # ★ 用户要求：Path 在「运动过程中持续发布；运动结束停止发布」
    #   - 机制：_path_stream_daemon 后台线程 10Hz
    #     while _path_active.is_set() → 每 100ms 重新用最新 timestamp 组装同一个 Path 发布
    #     （RViz Path display 要求 timestamp 持续最新；只发一帧会不显示/淡出消失）
    #   - 激活时机：收到 target_point 且 plan 拿到 trajectory_plan 后立刻 set() 同时首发一次
    #   - 停止时机：move_to_point(blocking=True) 返回（即运动执行结束）立刻 clear() → 后台线程下一轮检查停止发布
    # ===========================================================================
    def _path_stream_start(self, pose_deg_list: List[Dict[str, float]], frame_id: str, progress_idx: int = 0) -> None:
        with self._path_rwlock:
            self._path_pose_deg_list = list(pose_deg_list or [])
            self._path_frame_id = str(frame_id or self.base_frame)
            self._path_progress_idx = int(max(0, progress_idx))
            self._path_active.set()

    def _path_stream_stop(self) -> None:
        # 先 set=False 让后台下一轮停（后台锁保护）
        self._path_active.clear()
        # 清理当前 progress 黄球（不让 RViz 残留）
        try:
            if self._pub_progress is not None:
                self._pub_progress.publish(
                    self._make_sphere_marker((None, None, None), color_rgba=(0, 0, 0, 0), ns="progress", mid=0)
                )
        except Exception:
            pass

    def _path_stream_daemon(self) -> None:
        """后台 10Hz 刷 Path + 10Hz 刷『当前执行进度』黄球（Path 上滚动的小球）。"""
        import time as _t
        step_sleep = 0.10  # 10Hz
        while True:
            active_waiting = self._path_active.wait(timeout=0.5)
            if not active_waiting:
                _t.sleep(0.05)
                continue
            while self._path_active.is_set():
                # 1) 取最新 list（读写锁）
                with self._path_rwlock:
                    plist = list(self._path_pose_deg_list) if self._path_pose_deg_list is not None else []
                    frame = str(self._path_frame_id or self.base_frame)
                    idx = int(self._path_progress_idx)
                    if idx < 0:
                        idx = 0
                    if plist and idx >= len(plist):
                        idx = len(plist) - 1
                # 2) 发布 Path（用最新 stamp，RViz 才不会掉）
                stamp_now = self._node.get_clock().now().to_msg() if self._node is not None else None
                self._publish_path_from_pose_list(plist, base_header=object(), force_stamp=stamp_now)
                # 3) 发布当前执行进度黄球（🟡，在 Path 上滚动）
                if plist and self._pub_progress is not None:
                    tip = self._fk_pose_deg_to_xyz(plist[idx])
                    if tip is not None:
                        m = self._make_sphere_marker(
                            tip, color_rgba=(1.0, 0.85, 0.1, 0.95), ns="progress", mid=0, radius=0.05, lifetime_s=1.0
                        )
                        try:
                            self._pub_progress.publish(m)
                        except Exception:
                            pass
                    # 自动前进 progress_idx（纯计时预估前进，不依赖 controller 反馈；视觉效果稳定）
                    with self._path_rwlock:
                        if self._path_progress_idx < max(0, len(plist) - 1):
                            self._path_progress_idx += 1
                _t.sleep(step_sleep)
            # active 刚被 clear，把最后残留黄球清掉
            try:
                if self._pub_progress is not None:
                    self._pub_progress.publish(
                        self._make_sphere_marker((None, None, None), color_rgba=(0, 0, 0, 0), ns="progress", mid=0)
                    )
            except Exception:
                pass

    def _publish_wp_joint_states(self, pose_deg_list, base_stamp) -> None:
        if self._pub_wp is None or not pose_deg_list:
            return
        from v15_action_task.control_core.types import deg_to_rad as _d2r
        for pose_deg in pose_deg_list:
            js = self._JointState()
            js.header.frame_id = self.base_frame
            js.header.stamp = base_stamp
            from v15_action_task.control_core.types import SEMANTIC_JOINT_ORDER, URDF_JOINT_ORDER
            js.name = list(URDF_JOINT_ORDER)
            js.position = [float(_d2r(pose_deg.get(sem, 0.0))) for sem in SEMANTIC_JOINT_ORDER]
            try:
                self._pub_wp.publish(js)
            except Exception:
                pass

    def _current_tip_visualizer_loop(self) -> None:
        """★T2：每秒刷一次『当前实际 FK 末端』蓝色球（对齐末端运动终端，用户要的就是这个）。"""
        import time as _t
        while True:
            _t.sleep(1.0)
            try:
                pose_deg = self.ctl.get_pose() or {}
                tip = self.fk.solve(
                    boom_swing_deg=float(pose_deg.get("boom_swing", 0.0)),
                    arm_boom_deg=float(pose_deg.get("arm_boom", 0.0)),
                    bucket_arm_deg=float(pose_deg.get("bucket_arm", 0.0)),
                    swing_yaw_deg=float(pose_deg.get("swing_yaw", 0.0)),
                ).bucket_tip_3d
            except Exception:
                continue
            m = self._make_sphere_marker(tip, color_rgba=(0.1, 0.35, 1.0, 0.9), ns="current_tip", mid=0, radius=0.05)
            try:
                self._pub_tip_marker.publish(m)
            except Exception:
                pass

    # ── 小工具：发布目标位姿后阻塞等到位（URDFController.set_pose 没有 blocking 参数）──
    def _set_pose_and_wait(
        self,
        pose_deg: Dict[str, float],
        *,
        tolerance_deg: float = 1.0,
        timeout_s: float = 3.0,
        sleep_s: float = 0.05,
    ) -> bool:
        clamped = self.ctl._clamp_pose({k: float(v) for k, v in pose_deg.items()})
        ok = self.ctl.set_pose(clamped)
        if not ok:
            return False
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.ctl.is_at_pose(clamped, tolerance_deg=tolerance_deg):
                return True
            time.sleep(sleep_s)
        return self.ctl.is_at_pose(clamped, tolerance_deg=tolerance_deg)

    # ── 回调 1：手动关节位姿 ────────────────────────────────────
    def _on_joint_cmd(self, msg) -> None:
        # ✅ 一接到消息先立刻打一行"收到了"的回显，避免用户以为 v15 在接收但其实没有回调（💥 之前 URDF 不动的头号误判点）
        self._log.info(
            f"[cmd/recv] 收到 state_joint:  name_len={len(list(msg.name))}  pos_len={len(list(msg.position))}"
            f"  name={list(msg.name)[:6]!r}  pos≈{[round(float(x), 4) for x in list(msg.position)[:6]]}"
        )
        try:
            from v15_action_task.control_core.types import (
                URDF_JOINT_ORDER, URDF_TO_SEMANTIC,
            )
            name_to_rad: Dict[str, float] = {}
            for idx, n in enumerate(list(msg.name)):
                try:
                    name_to_rad[n] = float(msg.position[idx])
                except Exception:
                    continue
            # 💥 Fallback：用户有时 rospub 会漏写 name 数组或 name 全空；但 position 长度=4 → 按 URDF_JOINT_ORDER 顺序映射
            if not name_to_rad and len(list(msg.position)) == 4:
                self._log.info(
                    f"[cmd/recv] name 为空但 position[4] 存在 → 按 URDF_JOINT_ORDER 顺序解析"
                )
                for urdf, v in zip(URDF_JOINT_ORDER, msg.position):
                    name_to_rad[urdf] = float(v)
            pose_deg: Dict[str, float] = {}
            for urdf in URDF_JOINT_ORDER:
                if urdf in name_to_rad and urdf in URDF_TO_SEMANTIC:
                    pose_deg[URDF_TO_SEMANTIC[urdf]] = rad_to_deg(name_to_rad[urdf])
            if not pose_deg:
                self._log.warn(
                    f"[cmd/recv] 解析失败！支持两种格式：\n"
                    f"   ① name=[swing_joint,boom_joint,arm_joint,bucket_joint] + position[4]\n"
                    f"   ② name 空 但 position[4] 按 swing/boom/arm/bucket 顺序写弧度\n"
                    f"  实际收到：name={list(msg.name)!r} pos={[round(float(x),4) for x in list(msg.position)]!r}"
                )
                return
            self._log.info(
                f"[cmd/joint_cmd] 下发目标角度(度): "
                f"sw={pose_deg.get('swing_yaw', float('nan')):.2f}  "
                f"bm={pose_deg.get('boom_swing', float('nan')):.2f}  "
                f"ar={pose_deg.get('arm_boom', float('nan')):.2f}  "
                f"bk={pose_deg.get('bucket_arm', float('nan')):.2f}"
            )
            with self._lock:
                ok = self._set_pose_and_wait(pose_deg)
            self._log.info(
                f"[manual] {'OK' if ok else 'TIMEOUT'} "
                f"swing={pose_deg.get('swing_yaw',0):.1f}° "
                f"boom={pose_deg.get('boom_swing',0):.1f}° "
                f"arm={pose_deg.get('arm_boom',0):.1f}° "
                f"bucket={pose_deg.get('bucket_arm',0):.1f}°"
            )
        except Exception as e:
            self._log.error(f"[manual] 失败: {type(e).__name__}: {e}")

    # ── 回调 2：笛卡尔空间点 → 压 queue 立刻返回（Executor 线程永不阻塞） ──
    def _on_target_point(self, msg) -> None:
        # 0) 最顶级 raw recv 日志（与之前一致，用于证明 ROS 层收到了）
        try:
            _hdr_fid = str(getattr(getattr(msg, "header", None), "frame_id", "<no_header>"))
            try:
                _px = float(getattr(getattr(msg, "point", None), "x", float("nan")))
                _py = float(getattr(getattr(msg, "point", None), "y", float("nan")))
                _pz = float(getattr(getattr(msg, "point", None), "z", float("nan")))
            except Exception:
                _px = _py = _pz = float("nan")
            try:
                _stamp_s = int(getattr(getattr(getattr(msg, "header", None), "stamp", None), "sec", -1))
                _stamp_ns = int(getattr(getattr(getattr(msg, "header", None), "stamp", None), "nanosec", -1))
            except Exception:
                _stamp_s = _stamp_ns = -1
            self._log.info(
                f"[cmd/recv target_point] frame_id={_hdr_fid!r} "
                f"point=({_px:.4f}, {_py:.4f}, {_pz:.4f})m  "
                f"header_stamp=sec.{_stamp_s}/ns.{_stamp_ns}  "
                f"mode={self.mode} strategy={self.strategy} qos_unified_depth=10"
            )
        except Exception as _e0:
            try:
                self._log.warn(f"[cmd/recv target_point] 打 raw 日志本身失败: {type(_e0).__name__}: {_e0}")
            except Exception:
                pass

        # 1) 基础解析（也在 Executor 线程做，避免 worker 里解析失败没法在 queue 层不打日志；解析成 job dict，把 ros 句柄，不要 msg 对象引用
        try:
            xyz = (float(msg.point.x), float(msg.point.y), float(msg.point.z))
        except Exception as e:
            self._log.error(f"[plan] 解析失败: {e!r}")
            return
        try:
            if msg.header and (msg.header.stamp.sec or msg.header.stamp.nanosec):
                stamp_sec = int(msg.header.stamp.sec)
                stamp_ns = int(msg.header.stamp.nanosec)
            else:
                stamp_sec = stamp_ns = 0
            frame_id = str(getattr(getattr(msg, "header", None), "frame_id", str(self.base_frame)))
        except Exception:
            stamp_sec = stamp_ns = 0
            frame_id = str(self.base_frame)
        job: Dict[str, Any] = {
            "xyz": xyz,
            "stamp_sec": stamp_sec,
            "stamp_ns": stamp_ns,
            "frame_id": frame_id,
            "recv_ts": time.time(),
        }
        # 2) 压 queue，maxsize=1 + block=False，忙就走 except queue.Full 直接丢，提示 busy。
        try:
            self._tp_queue.put_nowait(job)
        except queue.Full:
            self._log.warn(
                f"[cmd/busy] worker 仍在执行上一条 target_point，队列已满（maxsize=1）。本条目标点已丢弃："
                f" ({xyz[0]:.2f},{xyz[1]:.2f},{xyz[2]:.2f})。请等上一条 [plan_trace] T9 打印完成后再发。"
            )
            return
        # 压成功就立刻返回，Executor 绝不阻塞
        self._log.info(
            f"[cmd/queued] target_point 已入后台 worker 队列（压入<1ms 返回，角度控制仍立即生效，永不被点控卡死）；"
            f" 目标点=({xyz[0]:.2f},{xyz[1]:.2f},{xyz[2]:.2f})"
        )
        return

    # ── Worker 里执行真正的点控主流程（preview + execute（原 _on_target_point 主体完整搬过来，零改动到这里 ──

    # Helper: 从 MoveResult 中抽取出「四关节字典」形式的 plan_list（与 Path/黄球进度一致）
    # ⭐ 背景：MoveResult 真实字段名是 .trajectory (List[JointTrajectoryPoint])，不是 trajectory_plan；
    #   但 trajectory 里每个点已经有 .pose_deg: Dict[str, float]（键=JOINT_4 四语义名），我们把它直接原样返回就行。
    def _extract_plan_list(self, res: Any) -> List[Dict[str, float]]:
        if res is None:
            return []
        traj = getattr(res, "trajectory", None)
        if isinstance(traj, list) and len(traj) > 0:
            # 优先 JointTrajectoryPoint.pose_deg：语义键 dict 形式（swing_yaw/boom_swing/arm_boom/bucket_arm）
            out: List[Dict[str, float]] = []
            for pt in traj:
                pose = getattr(pt, "pose_deg", None)
                if isinstance(pose, dict) and len(pose) >= 4:
                    out.append({str(k): float(v) for k, v in pose.items()})
                else:
                    # Fallback：如果某些旧点没有 pose_deg，尝试 positions + 关节顺序（极少见，兜底）
                    try:
                        positions = list(getattr(pt, "positions"))
                    except Exception:
                        positions = []
                    if len(positions) >= 4:
                        keys = ("swing_yaw", "boom_swing", "arm_boom", "bucket_arm")
                        out.append({keys[i]: float(positions[i]) for i in range(4)})
                    else:
                        # 完全拿不到：点跳过
                        continue
            return out
        # 再老版本可能把 plan_list 塞在其他字段？最后 getattr 全兜底
        for alt_name in ("trajectory_plan", "waypoints", "plan"):
            alt = getattr(res, alt_name, None)
            if isinstance(alt, list) and len(alt) > 0 and isinstance(alt[0], dict):
                return [{str(k): float(v) for k, v in p.items()} for p in alt]
        return []

    def _process_target_point_job(self, job: Dict[str, Any]) -> None:
        xyz: Tuple[float, float, float] = job["xyz"]
        stamp_sec = int(job.get("stamp_sec", 0) or 0)
        stamp_ns = int(job.get("stamp_ns", 0) or 0)
        # 构造 stamp_now：要么用 job 传进来的 sec/nanosec，要么用 ROS 时钟新取
        try:
            if stamp_sec or stamp_ns:
                stamp_now = self._Header()
                stamp_now.sec = stamp_sec
                stamp_now.nanosec = stamp_ns
            else:
                stamp_now = self._node.get_clock().now().to_msg()
        except Exception:
            try:
                stamp_now = self._node.get_clock().now().to_msg()
            except Exception:
                stamp_now = self._Header()

        # 清理前一轮可视化残留
        try:
            self._pub_closest_marker.publish(
                self._make_sphere_marker((None, None, None), color_rgba=(0, 0, 0, 0), ns="closest", mid=0)
            )
        except Exception:
            pass

        self._log.info(f"[plan] 收到目标点 xyz=({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f})m  mode={self.mode} strategy={self.strategy}")
        self._log.info(
            f"[plan_trace] T0 收到消息 -> Step1=可达域预检；若 T1 之前没有日志 = check_point_reachable 卡住（WS 未初始化？看 ws={self.ws is not None}）"
        )
        t0 = time.time()

        # Step 1: 可达域预检 + 目标/失败 Marker
        reachable = True
        reasons: List[str] = []
        closest = None
        if self.ws is not None:
            try:
                report = self.ws.check_point_reachable(xyz, mode=self.mode)
                reachable = bool(getattr(report, "success", False))
                reasons = list(getattr(report, "reasons", []) or [])
                closest = getattr(report, "closest_reachable_point", None)
            except Exception as e:
                self._log.error(f"[plan_trace] T1b WS 预检异常 {type(e).__name__}: {e}")
                reachable = False
                reasons = [f"workspace_exception:{type(e).__name__}"]
        else:
            self._log.warn("[plan_trace] T1a self.ws=None，跳过可达域预检（from_config init_workspace=False 或构建失败）")
        t1 = time.time()
        self._log.info(
            f"[plan_trace] T1 可达域预检 done  dt={t1-t0:.3f}s  reachable={reachable}  n_reasons={len(reasons)}  closest={closest}"
        )
        reachable_ok_color = (0.1, 0.9, 0.2, 0.9)
        unreachable_color = (0.95, 0.1, 0.15, 0.9)
        closest_color = (1.0, 0.55, 0.0, 0.9)
        try:
            self._pub_tgt_marker.publish(
                self._make_sphere_marker(xyz,
                                         color_rgba=(reachable_ok_color if reachable else unreachable_color),
                                         ns="target", mid=0, radius=0.055, lifetime_s=60.0)
            )
        except Exception:
            pass
        if not reachable and closest is not None:
            try:
                self._pub_closest_marker.publish(
                    self._make_sphere_marker(closest,
                                             color_rgba=closest_color,
                                             ns="closest", mid=0, radius=0.045, lifetime_s=60.0)
                )
            except Exception:
                pass
        try:
            import json
            reason_msg = self._StdMsgsString()
            reason_msg.data = json.dumps({
                "success": reachable,
                "reasons": reasons,
                "target": list(xyz),
                "closest_reachable_point": list(closest) if closest is not None else None,
                "mode": self.mode,
            }, ensure_ascii=False)
            self._pub_reason.publish(reason_msg)
        except Exception:
            pass

        if not reachable:
            self._log.warn(f"[plan] FAIL(reachability)  reasons=" + ",".join(reasons or ["<?>"]))
            self._path_stream_stop()
            return

        # Worker 版最小稳定版：preview/execute 同步阻塞不超时（单worker内部）
        self._log.info(f"[plan_trace] T2 同步 preview：move_to_point(execute=False) 直接执行")
        try:
            plan_preview: Optional[MoveResult] = self.mover.move_to_point(
                xyz,
                mode=self.mode,
                bucket_guess_deg=self.bucket_guess_deg,
                strategy=self.strategy,
                execute=False,
                blocking=False,
            )
            preview_exception = None
        except Exception as e:
            preview_exception = e
            plan_preview = None
            self._log.error(
                f"[plan_trace] T2b preview 异常: {type(e).__name__}: {e}\n"
                + ("-"*32) + "\n" + self._format_traceback() + ("-"*32)
            )
        plan_list: List[Dict[str, float]] = self._extract_plan_list(plan_preview)
        preview_success = bool(getattr(plan_preview, "success", None)) if plan_preview is not None else None
        preview_reason = getattr(plan_preview, "reason", None) if plan_preview is not None else None
        try:
            preview_keys = list(plan_preview.__dict__.keys()) if plan_preview is not None else "<MoveResult>"
        except Exception:
            preview_keys = "?"
        t2 = time.time()
        self._log.info(
            f"[plan_trace] T3 preview done  dt={t2-t1:.3f}s  preview_exception={preview_exception is not None}  "
            f"preview_success={preview_success}  preview_reason={preview_reason!r}  "
            f"len(plan_list)={len(plan_list)}  (source=MoveResult.trajectory.pose_deg, via _extract_plan_list)"
        )
        plan_list_was_fallback = not bool(plan_list)
        if plan_list_was_fallback:
            self._log.warn("[plan_trace] T3b preview plan_list=空，fallback：先执行，结束后从执行结果补 Path")
        execute_success = False
        execute_error: Optional[str] = None
        try:
            if plan_list:
                self._log.info(f"[plan_trace] T4 stream_start(n={len(plan_list)})")
                try:
                    self._path_stream_start(plan_list, frame_id=str(self.base_frame), progress_idx=0)
                    self._publish_wp_joint_states(plan_list, stamp_now)
                except Exception as e:
                    self._log.error(f"[plan_trace] T4b stream_start 异常: {type(e).__name__}: {e}")
            else:
                self._log.info("[plan_trace] T4 plan_list 空，暂不发 Path（fallback）")
            t4 = time.time()
            self._log.info(f"[plan_trace] T5 同步阻塞执行 move_to_point(execute=True, blocking=True)（dt={t4-t2:.3f}s）")
            with self._lock:
                res: MoveResult = self.mover.move_to_point(
                    xyz,
                    mode=self.mode,
                    bucket_guess_deg=self.bucket_guess_deg,
                    strategy=self.strategy,
                    execute=True,
                    blocking=True,
                )
            t5 = time.time()
            self._log.info(
                f"[plan_trace] T6 执行返回  dt_execute={t5-t4:.3f}s  success={res.success}  reason={res.reason!r}  "
                f"len(res.trajectory)={0 if getattr(res,'trajectory',None) is None else len(res.trajectory or [])}  "
                f"final_tip_xyz={tuple(round(v,3) for v in (res.final_tip_xyz or (0,0,0)))}"
            )
            if not res.success:
                rr = res.reachability_report
                reasons_log = "  reasons=" + ",".join(rr.reasons or []) if rr is not None else ""
                execute_error = f"{res.reason!r}{reasons_log}"
                self._log.warn(f"[plan] FAIL reason={execute_error}")
            else:
                execute_success = True
                if plan_list_was_fallback:
                    plan_list = self._extract_plan_list(res)
                    if plan_list:
                        self._log.info(f"[plan_trace] T6b fallback：补 Path(n={len(plan_list)})，停留 3s 后停")
                        try:
                            self._path_stream_start(plan_list, frame_id=str(self.base_frame), progress_idx=len(plan_list) - 1)
                            self._publish_wp_joint_states(plan_list, stamp_now)
                            def _stop_after_3s():
                                try: time.sleep(3.0)
                                finally:
                                    try: self._path_stream_stop()
                                    except Exception: pass
                            threading.Thread(target=_stop_after_3s, daemon=True).start()
                        except Exception as e:
                            self._log.error(f"[plan_trace] T6c fallback 异常: {type(e).__name__}: {e}")
                n = len(plan_list)
                tip = tuple(round(v, 3) for v in (res.final_tip_xyz or (0,0,0)))
                self._log.info(
                    f"[plan] OK  waypoints={n}  目标点=({xyz[0]:.2f},{xyz[1]:.2f},{xyz[2]:.2f})  "
                    f"最终铲尖=({tip[0]:.2f},{tip[1]:.2f},{tip[2]:.2f})  strategy={self.strategy}"
                )
                try:
                    self._publish_wp_joint_states(plan_list, stamp_now)
                except Exception:
                    pass
        except Exception as e:
            execute_error = f"{type(e).__name__}: {e}"
            tb = self._format_traceback()
            self._log.error(f"[plan] 异常: {execute_error}\n" + ("-"*32) + "\n" + tb + ("-"*32))
        finally:
            t_final = time.time()
            self._log.info(
                f"[plan_trace] T9 finally stream_stop  总耗时={t_final - t0:.3f}s  execute_success={execute_success}  "
                f"execute_error={('None' if execute_error is None else execute_error[:120])}"
            )
            self._path_stream_stop()
        return

    @staticmethod
    def _format_traceback() -> str:
        import traceback as _tb
        return _tb.format_exc()
    def spin_forever(self) -> None:
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            self._log.info("[v15_main] Ctrl+C 退出")
        finally:
            try:
                self.ctl.close()
            except Exception:
                pass
            try:
                if self._rclpy.ok():
                    self._rclpy.shutdown()
            except Exception:
                pass


def _parse(argv: List[str]) -> Dict[str, Any]:
    out = {"cfg": None, "strategy": "lerp", "mode": "transit", "bucket": -45.0}
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in ("-c", "--config") and i + 1 < len(argv):
            out["cfg"] = argv[i + 1]; i += 2
        elif a in ("-s", "--strategy") and i + 1 < len(argv):
            out["strategy"] = argv[i + 1]; i += 2
        elif a in ("-m", "--mode") and i + 1 < len(argv):
            out["mode"] = argv[i + 1]; i += 2
        elif a in ("-b", "--bucket") and i + 1 < len(argv):
            out["bucket"] = float(argv[i + 1]); i += 2
        elif a in ("-h", "--help"):
            print(
                "Usage: python3 v15_main_node.py\n"
                "  -c, --config PATH       自定义 YAML/JSON 配置(可选)\n"
                "  -s, --strategy [lerp|trapezoidal]   轨迹策略 (默认 lerp)\n"
                "  -m, --mode     [transit|dig]        可达域模式 (默认 transit)\n"
                "  -b, --bucket   FLOAT                IK 搜索铲斗起始角(度,默认 -45)"
            )
            sys.exit(0)
        else:
            i += 1
    return out


def main() -> int:
    args = _parse(sys.argv[1:])
    V15MainNode(
        cfg_path=args["cfg"],
        strategy=args["strategy"],
        mode=args["mode"],
        bucket_guess_deg=args["bucket"],
    ).spin_forever()
    return 0


if __name__ == "__main__":
    sys.exit(main())
