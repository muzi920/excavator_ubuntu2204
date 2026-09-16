"""
control_core 自检 + Demo：
  1) MockAdapter 无 ROS 环境测试（每次都能跑）
  2) 如果 source 了 ROS 环境，再跑 RosV14Adapter 的 dry-run 测试（不发消息，只创建对象）

运行：
  cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
  /usr/bin/python3 src/shandong/v15_action_task/control_core/demo_usage.py
"""

from __future__ import annotations

import os
import sys
import time

_V15_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _V15_ROOT not in sys.path:
    sys.path.insert(0, _V15_ROOT)

from control_core import (
    URDFController,
    MockAdapter,
    SEMANTIC_TO_URDF,
    URDF_JOINT_ORDER,
    default_pose_deg,
    deg_to_rad,
    rad_to_deg,
)


def section(title: str):
    print(f"\n{'=' * 60}\n  {title}\n{'=' * 60}")


def run_mock_demo() -> int:
    section("Demo 1: MockAdapter 本地调试（无 ROS）")
    passed = 0

    # 1. 初始化（with 自动 open/close）
    init = default_pose_deg()
    init.update({"swing_yaw": 5.0, "boom_swing": 10.0})
    with URDFController(MockAdapter(initial_pose_deg=init)) as ctl:
        p = ctl.get_pose()
        ok = p is not None and abs(p["swing_yaw"] - 5.0) < 1e-9
        print(f"  ✅ 初始 pose: swing_yaw={p['swing_yaw'] if p else None}")
        passed += 1 if ok else 0

        # 2. set_joint 单关节
        ok1 = ctl.set_joint("boom_swing", 25.5)
        p2 = ctl.get_pose()
        ok = ok1 and p2 is not None and abs(p2["boom_swing"] - 25.5) < 1e-9
        print(f"  ✅ set_joint boom_swing=25.5 → 当前: {p2['boom_swing'] if p2 else None}")
        passed += 1 if ok else 0

        # 3. set_pose 多关节
        ok2 = ctl.set_pose({"arm_boom": 40.0, "bucket_arm": -45.0})
        p3 = ctl.get_pose()
        ok = (
            ok2
            and p3 is not None
            and abs(p3["arm_boom"] - 40.0) < 1e-9
            and abs(p3["bucket_arm"] - (-45.0)) < 1e-9
        )
        print(f"  ✅ set_pose {{arm_boom=40, bucket_arm=-45}} → 当前: arm_boom={p3['arm_boom'] if p3 else None}, bucket={p3['bucket_arm'] if p3 else None}")
        passed += 1 if ok else 0

        # 4. 到位检测
        target = {"swing_yaw": 5.0, "boom_swing": 25.5, "arm_boom": 40.0, "bucket_arm": -45.0}
        ok = ctl.is_at_pose(target, tolerance_deg=1.0)
        print(f"  ✅ is_at_pose 检测到位: {ok}")
        passed += 1 if ok else 0

        # 5. 未包含在 dict 中的关节保持不变
        ctl.set_pose({"swing_yaw": 15.0})
        p4 = ctl.get_pose()
        ok = p4 is not None and abs(p4["boom_swing"] - 25.5) < 1e-9
        print(f"  ✅ 只改 swing_yaw 时 boom_swing 保持: {p4['boom_swing'] if p4 else None}")
        passed += 1 if ok else 0

    # 6. 语义名映射
    print(f"\n  语义 → URDF 映射: {SEMANTIC_TO_URDF}")
    print(f"  发布时 name 顺序: {URDF_JOINT_ORDER}")
    print(f"  deg↔rad: 180° = {deg_to_rad(180.0):.6f} rad, π rad = {rad_to_deg(3.1415926535):.3f}°")
    passed += 1

    return passed


def run_ros_smoke_test() -> int:
    """尝试 import RosV14Adapter 并创建对象（不启动节点，不连 ROS）。"""
    section("Demo 2: RosV14Adapter 冒烟测试（需要 source ROS）")
    try:
        from control_core import RosV14Adapter
    except ImportError as e:
        print(f"  ⚠️  RosV14Adapter import 失败（未 source ROS，正常跳过）: {e}")
        return 0

    # 只创建对象，不 open（open 才会 init rclpy），避免污染全局
    try:
        adapter = RosV14Adapter(node_name="v15_demo_smoke")
        print(f"  ✅ RosV14Adapter 对象创建成功: topic={adapter._topic}, node={adapter._node_name}")
        return 1
    except Exception as e:
        print(f"  ❌ RosV14Adapter 创建失败: {e}")
        return 0


def main() -> int:
    print("v15_action_task / control_core 演示与自检")
    print(f"  Python: {sys.executable}")

    mock_passed = run_mock_demo()
    ros_ok = run_ros_smoke_test()

    section("总结")
    print(f"  Mock 端: {mock_passed}/6 通过")
    print(f"  Ros 冒烟: {'通过' if ros_ok else '跳过（未 source ROS）'}")
    print("\n后续在 UI 中：")
    print("  with URDFController(MockAdapter()) as ctl:")
    print("      ctl.set_pose({从 UI 界面来的角度字典})")
    print("  切到真机时只要把 MockAdapter 换成 RosV14Adapter 即可。")
    return 0 if mock_passed == 6 else 1


if __name__ == "__main__":
    sys.exit(main())
