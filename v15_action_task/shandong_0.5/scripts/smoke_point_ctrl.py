"""shandong_0.5 脚本 2：目标点控制（Point -> IK -> 4 段串行单关节，永不复合）。

使用：
  # 1) URDF+RViz（先起）
  ros2 launch shandong_0_5 display_calibrated.launch.py use_joint_state_publisher:=false
  # 2) 跑 smoke：
  cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/scripts
  python3 smoke_point_ctrl.py            # 真机 ROS
  python3 smoke_point_ctrl.py --use-mock # 离线 MockAdapter 自检
"""
from __future__ import annotations

import argparse
import logging
import os
import sys

logging.basicConfig(level=logging.INFO, format="[%(asctime)s][%(name)s][%(levelname)s] %(message)s")
_LOG = logging.getLogger("smoke_point")

# 保证 import 到 v15_action_task（0 改 v15 源码，只 import）
_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_ROOT = os.path.abspath(os.path.join(_HERE, os.pardir, os.pardir, os.pardir))  # src/
_SHANDONG_PKG = os.path.join(_SRC_ROOT, "shandong")
for p in (_SHANDONG_PKG, _SRC_ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)
_PKG_ROOT_V15 = os.path.join(_SHANDONG_PKG, "v15_action_task")
if _PKG_ROOT_V15 not in sys.path:
    sys.path.insert(0, _PKG_ROOT_V15)

from v15_action_task import URDFController, RosV14Adapter  # type: ignore
# shandong_0_5 位于 v15_action_task/shandong_0_5 下 — 用 importlib 直接按文件加载两个 control_lib 模块（避免依赖 setuptools package_dir 显式声明）
import importlib.util as _ilu
_HERE2 = os.path.dirname(os.path.abspath(__file__))
_CTRL_DIR = os.path.abspath(os.path.join(_HERE2, os.pardir, "control_lib"))
# 1. 加载 angle_control + 导出 JointAngleController
_ac_spec = _ilu.spec_from_file_location(
    "_s05_angle_control", os.path.join(_CTRL_DIR, "angle_control.py"),
    submodule_search_locations=[_CTRL_DIR],
)
_ac_mod = _ilu.module_from_spec(_ac_spec)  # type: ignore[arg-type]
_ac_spec.loader.exec_module(_ac_mod)  # type: ignore[union-attr]
JointAngleController = _ac_mod.JointAngleController
# 2. 加载 point_control + 导出 SingleJointPointMover（point_control 内部用 from .angle_control import X，所以要先注册 sys.modules 子路径）
import sys as _sys
_sys.modules.setdefault("shandong_0_5", type(sys)("shandong_0_5"))  # stub
if not hasattr(_sys.modules["shandong_0_5"], "__path__"):
    _sys.modules["shandong_0_5"].__path__ = [os.path.abspath(os.path.join(_HERE2, os.pardir))]
_sys.modules["shandong_0_5.control_lib"] = type(sys)("shandong_0_5.control_lib")
_sys.modules["shandong_0_5.control_lib"].__path__ = [_CTRL_DIR]
_sys.modules["shandong_0_5.control_lib.angle_control"] = _ac_mod
_sys.modules["shandong_0_5.control_lib.angle_control"].JointAngleController = JointAngleController
_pc_spec = _ilu.spec_from_file_location(
    "shandong_0_5.control_lib.point_control",
    os.path.join(_CTRL_DIR, "point_control.py"),
    submodule_search_locations=[_CTRL_DIR],
)
_pc_mod = _ilu.module_from_spec(_pc_spec)  # type: ignore[arg-type]
# ⚠️ dataclass 装饰器会用 sys.modules.get(cls.__module__) 拿到 cls 所在模块命名空间，所以必须先把 _pc_mod 注册到 sys.modules，名字必须和模块文件内部的 __name__ 一致（=spec_from_file_location 里的首参）：
_sys.modules["shandong_0_5.control_lib.point_control"] = _pc_mod
_pc_mod.__package__ = "shandong_0_5.control_lib"
_pc_spec.loader.exec_module(_pc_mod)  # type: ignore[union-attr]
SingleJointPointMover = _pc_mod.SingleJointPointMover
# 把 JointAngleController 挂上去（以防 from shandong_0_5.control_lib.angle_control import 没走通时，fallback 模块级全局 angle_control 已加载）：
_sys.modules["shandong_0_5.control_lib.point_control"].JointAngleController = JointAngleController
# 清理
del _ilu, _HERE2, _CTRL_DIR, _ac_spec, _ac_mod, _pc_spec, _pc_mod, _sys



def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--use-mock", action="store_true", help="MockAdapter 离线无 ROS")
    ap.add_argument("--mode", default="dig", choices=["dig", "transit"], help="可达域过滤 mode")
    ap.add_argument("--x", type=float, default=1.2)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--z", type=float, default=0.8)
    ap.add_argument("--auto-closest", action="store_true", help="目标不可达时自动用 closest_reachable_point")
    ap.add_argument("--per-joint-timeout-s", type=float, default=10.0)
    ap.add_argument("--tolerance-deg", type=float, default=1.2)
    args = ap.parse_args()

    if args.use_mock:
        from v15_action_task import MockAdapter  # type: ignore
        ctor = lambda: MockAdapter()  # noqa: E731
        _LOG.info("[smoke_point] MockAdapter（离线，不启 ROS 节点）")
    else:
        ctor = lambda: RosV14Adapter()  # noqa: E731

    xyz = (float(args.x), float(args.y), float(args.z))
    _LOG.info("[smoke_point] 目标点 (x,y,z)=(%.3f,%.3f,%.3f)m  mode=%s  约束：4 段串行单关节，不复合", *xyz, args.mode)
    mover = SingleJointPointMover.from_config_default(mode_default=args.mode)

    with URDFController(ctor()) as ctl:
        jac = JointAngleController(
            ctl,
            default_tolerance_deg=args.tolerance_deg,
            default_timeout_s=args.per_joint_timeout_s,
        )
        mover.bind_controller(jac)

        r = mover.move_to_point_serial(
            xyz,
            mode=args.mode,
            auto_use_closest_if_unreachable=args.auto_closest,
        )
        print("\n== smoke_point 结果 ==")
        print(f"  success          : {r.success}")
        print(f"  reason           : {r.reason}")
        print(f"  target_xyz       : {r.target_xyz}")
        print(f"  reachable        : {r.reachable}  reasons={r.reachability_reasons}")
        print(f"  closest          : {r.closest_reachable_point}")
        print(f"  ik_success       : {r.ik_success}  ik_reason={r.ik_reason}")
        if r.ik_target_pose_deg:
            print(f"  ik_target_pose   : { {k: round(v,1) for k,v in r.ik_target_pose_deg.items()} }")
        print(f"  order (4 段串行)  : {r.order}")
        for j in r.per_joint:
            print(
                f"   - {j.get('joint'):12s} success={str(j.get('success')):5s} "
                f"target={j.get('target_after_clamp_deg'):+.1f}° final={j.get('final_joint_deg'):+.1f}° "
                f"waited={j.get('waited_s',0):.1f}s  {j.get('reason')}"
            )
        if r.final_tip_xyz:
            print(
                f"  final_tip_xyz    : {tuple(round(v,3) for v in r.final_tip_xyz)} "
                f"3D err={ (r.tip_error_3d_m or 0)*1000:.1f} mm"
            )
        print(f"  waited_total_s   : {r.waited_total_s:.2f}s")

        # ──────────────────────────────────────────
        # ★ 话题（Topics）信息：给用户看 ros2 topic list / echo 用
        # ──────────────────────────────────────────
        print("\n== 话题（Topics）==")
        adapter = getattr(ctl, "adapter", None)
        adapter_cls = type(adapter).__name__ if adapter is not None else "N/A"
        print(f"  后端适配器（Adapter）: {adapter_cls}")
        if adapter_cls == "RosV14Adapter":
            topic   = str(getattr(adapter, "_topic", "/joint_states"))
            node    = str(getattr(adapter, "_node_name", "v15_urdf_controller"))
            frame   = str(getattr(adapter, "_default_frame_id", "base_link"))
            qos     = int(getattr(adapter, "_qos_depth", 10))
            qos_str = f"KEEP_LAST(depth={qos}) RELIABLE VOLATILE"
            print(f"  【控制输出 ★ 本脚本真正发布的话题】:")
            print(f"    {topic}  ← sensor_msgs/msg/JointState  (10Hz 心跳持续发，驱动 URDF/RViz2)")
            print(f"         frame_id={frame}   node={node}   QoS={qos_str}")
            print(f"         urdf_joint_order: [swing_joint, boom_joint, arm_joint, bucket_joint]  ← 弧度")
            print(f"  【shandong_0.5 输出特点（与 v15_main 不同）】:")
            print(f"    • 4 段串行单关节：每次 publish 只有 1 个关节角度变化，其他 3 个不变")
            print(f"    • 不发 /v15/planned_path、/v15/target_point 等 Marker/Path 可视化")
            print(f"    • 想可视化 Point→4 段串行，请用 RViz2 里的 RobotModel，盯着 swing/boom/arm/bucket 依次一个一个动")
            print(f"  【参考：v15_main 监听的输入话题（本 smoke 不使用，仅供对照）】:")
            print(f"    /v15/cmd/joint_states   ← sensor_msgs/JointState  (同上 4 关节顺序，弧度)")
            print(f"    /v15/cmd/target_point   ← geometry_msgs/PointStamped (frame_id=base_link, 单位米)")
            print(f"  【参考：v15_main 可视化话题（本 smoke 不发布；如需要请起 v15_main_node）】:")
            print(f"    /v15/planned_path       ← nav_msgs/Path              (整个关节轨迹)")
            print(f"    /v15/target_point       ← visualization_msgs/Marker  (目标球 绿=可达 / 红=不可达)")
            print(f"    /v15/closest_point      ← visualization_msgs/Marker  (最近可达点橙球)")
            print(f"    /v15/current_tip        ← visualization_msgs/Marker  (当前末端蓝球)")
            print(f"    /v15/progress_tip       ← visualization_msgs/Marker  (运动中进度黄球)")
            print(f"    /v15/execution_waypoints← sensor_msgs/JointState     (逐路点关节)")
            print(f"    /v15/reachability_reasons← std_msgs/String           (Fail 原因文字)")
            print(f"  💡 Debug 常用命令（真机 ROS）：")
            print(f"     ros2 topic list ; ros2 topic hz {topic} ; ros2 topic echo {topic} --once")
            print(f"     ros2 topic info {topic} ; ros2 interface show sensor_msgs/msg/JointState")
        elif adapter_cls == "MockAdapter":
            print(f"  （MockAdapter 离线模式，不发布 / 订阅任何 ROS 话题）")
            print(f"  （真机 ROS 模式运行时不加 --use-mock，将发布：/joint_states）")

    return 0 if r.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
