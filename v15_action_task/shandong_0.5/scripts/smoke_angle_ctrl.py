"""shandong_0.5 脚本 1：角度控制（只允许单关节运动，绝不复合）。

使用：
  source /opt/ros/humble/setup.bash
  source /media/libo/libo_sn7100/ubuntu2204/shandong_ws/install/setup.bash
  # 1) 先另一个终端起 URDF+RViz（必须参数 use_joint_state_publisher:=false 顺序别写反）：
  #    ros2 launch shandong_0_5 display_calibrated.launch.py use_joint_state_publisher:=false
  # 2) 跑这个 smoke：
  #    cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/scripts
  #    python3 smoke_angle_ctrl.py
"""
from __future__ import annotations

import argparse
import logging
import os
import sys

logging.basicConfig(level=logging.INFO, format="[%(asctime)s][%(name)s][%(levelname)s] %(message)s")
_LOG = logging.getLogger("smoke_angle")

# 保证 import 到 v15_action_task（0 改 v15 源码，只 import）
_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_ROOT = os.path.abspath(os.path.join(_HERE, os.pardir, os.pardir, os.pardir))  # src/
_SHANDONG_PKG = os.path.join(_SRC_ROOT, "shandong")  # src/shandong
for p in (_SHANDONG_PKG, _SRC_ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)
# 保证导入 shandong_0_5.control_lib（shandong_0_5 就在 v15_action_task 下，所以 _SRC_ROOT/shandong 也能拿到 shandong.v15_action_task.shandong_0_5）
_PKG_ROOT_V15 = os.path.join(_SHANDONG_PKG, "v15_action_task")
if _PKG_ROOT_V15 not in sys.path:
    sys.path.insert(0, _PKG_ROOT_V15)

from v15_action_task import URDFController, RosV14Adapter  # type: ignore
# shandong_0_5 位于 v15_action_task/shandong_0_5/ — 用 importlib 作为子包加载，或直接用 control_lib 路径 import（避免依赖 setuptools package_dir 配置）
import importlib.util as _ilu
_HERE2 = os.path.dirname(os.path.abspath(__file__))
_ctrl_pkg = os.path.abspath(os.path.join(_HERE2, os.pardir, "control_lib"))
_jac_path = os.path.join(_ctrl_pkg, "angle_control.py")
_spec = _ilu.spec_from_file_location(
    "_s05_angle_control",
    _jac_path,
    submodule_search_locations=[_ctrl_pkg],
)
_mod = _ilu.module_from_spec(_spec)  # type: ignore[arg-type]
_spec.loader.exec_module(_mod)  # type: ignore[union-attr]
JointAngleController = _mod.JointAngleController
del _ilu, _HERE2, _ctrl_pkg, _jac_path, _spec, _mod



def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tolerance-deg", type=float, default=1.0, help="单关节到位容差（度）")
    ap.add_argument("--timeout-s", type=float, default=8.0, help="单关节超时秒")
    ap.add_argument("--use-mock", action="store_true", help="本地调试：MockAdapter 离线不通 ROS")
    args = ap.parse_args()

    if args.use_mock:
        from v15_action_task import MockAdapter  # type: ignore
        ctor = lambda: MockAdapter()  # noqa: E731
        _LOG.info("[smoke_angle] 使用 MockAdapter（离线，不启 ROS 节点）")
    else:
        ctor = lambda: RosV14Adapter()  # noqa: E731

    _LOG.info("[smoke_angle] 约束：每一步只动 1 个关节，顺序默认 swing->boom->arm->bucket")
    with URDFController(ctor()) as ctl:
        jac = JointAngleController(
            ctl,
            default_tolerance_deg=args.tolerance_deg,
            default_timeout_s=args.timeout_s,
        )
        # 测试 1：4 个关节各单关节一次（典型角度，非复合）
        cases = [
            ("swing_yaw",  +30.0, "顺时针回转 30°（从上往下看）"),
            ("boom_swing", +20.0, "大臂下俯 20°（boom_swing +=向下）"),
            ("arm_boom",   +60.0, "小臂收回 60°（arm_boom +=内收）"),
            ("bucket_arm", -60.0, "铲斗打开 -60°（bucket_arm -=开斗卸料）"),
        ]
        print("\n== smoke 1：单关节各自独立运动（4 次，每次只驱动 1 个关节）==")
        for name, deg, desc in cases:
            print(f"\n> move_joint({name}, {deg:+.1f}°)  — {desc}")
            r = jac.move_joint(name, deg)
            print(f"   <- {r}")

        print("\n== smoke 2：完整 4 关节目标，用 move_pose_serial 串行单关节执行 ==")
        target = {"swing_yaw": -45.0, "boom_swing": -5.0, "arm_boom": 100.0, "bucket_arm": 0.0}
        print(f"   target_pose_deg = {target}")
        r = jac.move_pose_serial(target)
        print(f"   <- overall success={r['success']} reason={r['reason']} total_wait={r['waited_total_s']:.2f}s")
        for j in r.get("per_joint", []):
            print(f"      per_joint: {j.get('joint')} -> success={j.get('success')}  {j.get('reason')} (waited={j.get('waited_s',0):.2f}s)")

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
        print(f"  💡 Debug 常用命令：")
        print(f"     ros2 topic list ; ros2 topic hz {topic} ; ros2 topic echo {topic} --once")
    elif adapter_cls == "MockAdapter":
        print(f"  （MockAdapter 离线模式，不发布 / 订阅任何 ROS 话题）")
        print(f"  （真机 ROS 模式运行时不加 --use-mock，将发布：/joint_states）")

    print("\n[smoke_angle] DONE — 全程无复合动作（每一步只改 1 个关节值下发）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
