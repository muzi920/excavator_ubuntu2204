"""
v15 通用控制库 端到端 Demo  —— drive_v14_in_rviz.py
====================================================

功能：
  1. 连接 ROS 2 的 /joint_states 话题 (v14 URDF 控制协议)
  2. 用 IK 解算末端目标点 → 驱动 v14 标定版 URDF 在 RViz2 里真实运动
  3. 演示 init_pose → move_to_cartesian → move_with_bucket → 回零 的完整流程

使用步骤（SSH 远程 + 被控电脑本地 RViz）：
────────────────────────────────────────────────────────────────────
┌─ 被控电脑 (有显示器) ─────────────────────────────────────────┐
│ 1) 编译工作空间 (只做一次):
│    cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
│    colcon build --symlink-install --packages-select shandong
│    source install/setup.bash
│
│ 2) 启动 v14 URDF headless display (不启 GUI 节点，SSH 下也不报错):
│    ros2 launch shandong_v14_urdf display.launch.py \
│        headless:=true use_joint_state_publisher:=false
│
│ 3) 被控电脑本地打开 RViz2，订阅 /joint_states /robot_description，
│    添加 RobotModel Display 即可看到模型。
└────────────────────────────────────────────────────────────────┘

┌─ SSH 终端 (运行本 demo) ────────────────────────────────────────┐
│    cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
│    source install/setup.bash
│    cd src/shandong/v15_action_task
│    python3 drive_v14_in_rviz.py
│
│  观察：RViz 里的挖掘机会依次执行 6 段动作，每段动作 2 秒到位。
└────────────────────────────────────────────────────────────────┘

无 ROS 环境也能跑本脚本：它会自动 fallback 到 MockAdapter 输出日志演示。

CLI 用法（新增）:
────────────────────────────────────────────────────────────────────
  # 默认 60FED 配置 + 自动检测 ROS / Mock
  python3 drive_v14_in_rviz.py

  # ★ 自定义机型 YAML 配置（改连杆/限位/ROS 话题不用改代码）
  python3 drive_v14_in_rviz.py --config ./my_custom_60FED.yaml

  # 强制指定后端（绕过自动检测）
  python3 drive_v14_in_rviz.py --backend mock           # 无 ROS 调试
  python3 drive_v14_in_rviz.py --backend ros            # 强制 ROS（需先 source）

  # 临时关闭限位自动裁剪（例：调试历史动作脚本超限误差）
  python3 drive_v14_in_rviz.py --no-limits

  # 组合：自定义配置 + ROS + 不关限位
  python3 drive_v14_in_rviz.py -c ./my_long_bucket.yaml -b ros
"""

from __future__ import annotations

import argparse
import math
import sys
import time

# 兼容：在 v15_action_task 根目录直接运行
ROOT = "/".join(__file__.split("/")[:-1]) or "."
sys.path.insert(0, ROOT)
sys.path.insert(0, ROOT + "/..")


def have_ros2() -> bool:
    try:
        import rclpy  # noqa: F401
        return True
    except Exception:
        return False


def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        prog="drive_v14_in_rviz.py",
        description="v15 通用控制库 端到端 Demo —— 驱动 v14 URDF 在 RViz2 中执行 6 段挖掘动作",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "-c", "--config", type=str, default=None, metavar="<yaml_or_json>",
        help="自定义机型配置文件路径（.yaml/.yml/.json 均可；Ubuntu 默认 Python 无 PyYAML 时 .yaml 会自动 fallback 同名 .json）。"
             " 留空则用包内默认 60FED + 内置 dict 兜底。",
    )
    ap.add_argument(
        "-b", "--backend", type=str, default="auto", choices=["auto", "mock", "ros"],
        help="指定后端：auto=自动检测 rclpy；mock=内存后端（无 ROS 调试）；ros=强制 ROS 2 v14 Adapter。默认 auto。",
    )
    ap.add_argument(
        "--no-limits", action="store_true",
        help="临时关闭 YAML 关节限位自动裁剪（等价 from_config(use_config_limits=False)）。"
             " 默认开启裁剪，避免超限指令发布。",
    )
    return ap.parse_args()


def main() -> int:
    from v15_action_task import (
        URDFController,
        RosV14Adapter,
        MockAdapter,
        InverseKinematics,
        ForwardKinematics,
        CartesianMover,
        move_to_cartesian,
        default_pose_deg,
        from_config,
    )

    args = _parse_args()

    # ── ① 选择 backend（CLI 优先，否则 auto）──
    if args.backend == "auto":
        use_ros = have_ros2()
        backend = "ros" if use_ros else "mock"
    else:
        backend = args.backend
        use_ros = (backend == "ros")

    # ── ② 用 from_config 一步构建完整工具链（统一走配置层，支持 --config / --no-limits）──
    cfg_source = args.config if args.config else None   # None = 包内默认 + 三级兜底
    start_adapter = use_ros   # ROS 后端立即 open()，Mock 后端不需要先 open（with 会处理）
    use_limits = (not args.no_limits)

    print("[INFO] " + "=" * 68)
    print(f"[INFO] 配置来源  : {'默认 60FED + 三级兜底' if cfg_source is None else cfg_source}")
    print(f"[INFO] 控制后端  : {backend.upper()} ({'RosV14Adapter' if use_ros else 'MockAdapter'})")
    print(f"[INFO] 限位裁剪  : {'自动裁剪 (use_config_limits=True)' if use_limits else '关闭 (--no-limits)'}")

    ctx = from_config(
        cfg_source,
        adapter_backend=backend,
        start_adapter=start_adapter,
        use_config_limits=use_limits,
    )
    cfg = ctx["config"]
    controller = ctx["controller"]
    adapter = ctx["adapter"]
    fk = ctx["fk"]
    ik = ctx["ik"]
    mover = ctx["mover"]

    print(f"[INFO] 机型名    : {cfg.model_name}")
    print(f"[INFO] 连杆参数  : boom_eff={cfg.link.to_link_params().L_boom:.4f}m  L_arm={cfg.link.L_arm}m  L_bucket={cfg.link.L_bucket}m")
    print(f"[INFO] ROS 话题  : {cfg.ros.joint_topic}  (frame_id={cfg.ros.frame_id})")
    print(f"[INFO] 大臂限位  : [{cfg.limits.limits['boom_swing'].min_deg}°, {cfg.limits.limits['boom_swing'].max_deg}°]")
    print("[INFO] " + "=" * 68)

    # 覆盖 mover 的轮询参数为 demo 风格（配置层的是短超时，demo 观察需要稍长）
    # （不改动配置，只在 demo 层临时调整 mover 的默认容差/超时）
    mover.tol = 2.0
    mover.timeout = 3.0
    mover.poll_s = 0.05
    if not use_ros:
        print("[WARN] 未检测到 ROS 2，使用 MockAdapter 仅做演示 (不会真的发布话题)")
    else:
        print("[INFO] RosV14Adapter 已启动（话题已由 from_config(..., start_adapter=True) 内部 open）")

    # 目标点列表: (x,y,z, bucket_angle=None 表示自动搜索, 文字描述)
    plan = [
        (None, None, None, None, "先回到 INIT_POSE (手动复位姿态)"),
        (1.00,  0.00, -0.10, -60.0, "① 前方 1.0m 下 0.1m，挖掘姿态 (指定铲斗角 -60°)"),
        (1.05,  0.00, -0.35, -60.0, "② 下挖 —— 同一 Yaw，再向前+向下 25cm"),
        (0.90,  0.00,  0.10, -20.0, "③ 提斗 —— 向后上方收起，铲斗 -20° 保持"),
        (0.95, +0.55,  0.05, -30.0, "④ 左转约 30° 卸土方向"),
        (0.95, +0.55, -0.10, +10.0, "⑤ 张斗卸料 (bucket 角 +10° 最大张开)"),
        (0.00,  0.00,  0.00,  None, "⑥ 回到 DEFAULT 姿态 (swing=0)"),
    ]

    try:
        # controller / mover 已由 from_config 预构建；进入上下文 manager 后 ctl = controller.__enter__()
        # 注意：mover.ctl 和 controller 实际上是同一个对象引用，进入上下文即可，无需重绑定
        with controller as ctl:
            print()
            print("=" * 72)
            print("v15 Standard Control Library Demo 开始执行 (6 段动作)")
            print("=" * 72)

            for i, (x, y, z, ba, desc) in enumerate(plan):
                print(f"\n▶ Step {i+1}: {desc}")
                t0 = time.monotonic()

                if x is None:
                    # INIT 姿态：手动 0/0/0/0 或 safe pose
                    pose_cmd = default_pose_deg()
                    # 更合理的 INIT 是一个中间安全姿态而不是全 0
                    pose_cmd = {
                        "swing_yaw": 0.0,
                        "boom_swing": 30.0,
                        "arm_boom":  50.0,
                        "bucket_arm": -60.0,
                    }
                    ok = ctl.set_pose(pose_cmd)
                    waited = 0.0
                    deadline = t0 + 3.0
                    while time.monotonic() < deadline:
                        if ctl.is_at_pose(pose_cmd, tolerance_deg=2.0):
                            waited = time.monotonic() - t0
                            break
                        time.sleep(0.05)
                    reached = ctl.get_pose_or_default()
                    # FK 参数名是 <joint>_deg 形式，不能直接用语义 joint 名
                    fk_sol = fk.solve(
                        boom_swing_deg=reached.get("boom_swing", 0.0),
                        arm_boom_deg=reached.get("arm_boom", 0.0),
                        bucket_arm_deg=reached.get("bucket_arm", 0.0),
                        swing_yaw_deg=reached.get("swing_yaw", 0.0),
                    )
                    tip = fk_sol.bucket_tip_3d
                    success = ok
                else:
                    if ba is None:
                        r = mover.move(x, y, z)
                    else:
                        r = mover.move_with_bucket(x, y, z, ba)
                    success = bool(r)
                    waited  = r.waited_s
                    tip     = r.final_tip_xyz
                    reached = r.reached_pose_deg

                dt = time.monotonic() - t0
                status = "✓" if success else "✗"
                tip_str = f"({tip[0]:.3f},{tip[1]:.3f},{tip[2]:.3f})m" if tip else "N/A"
                pose_str = {k: round(v, 1) for k, v in reached.items()} if reached else "N/A"
                print(f"   {status} 耗时总 {dt:.2f}s, 轮询 {waited:.2f}s")
                print(f"      到达关节: {pose_str}")
                print(f"      铲尖 3D:  {tip_str}")
                if not success and use_ros:
                    print(f"      ⚠ 未到位 (可能 RViz 没开或 topic 无反馈)。这是 demo，不会中断。")
                time.sleep(0.4)  # 动作间隔，便于观察 RViz

            print()
            print("=" * 72)
            print("✅ v15 Demo 全部 6 段动作执行完毕。")
            print("=" * 72)
            return 0
    except KeyboardInterrupt:
        print("\n[INFO] 用户中断 (Ctrl+C)")
        return 130
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"\n[FATAL] Demo 崩溃: {type(e).__name__}: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
