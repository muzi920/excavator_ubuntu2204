import argparse
import os
import sys
import time

CURRENT_DIR = os.path.dirname(__file__)
if CURRENT_DIR not in sys.path:
    sys.path.append(CURRENT_DIR)

from ros_joint_bridge import RosJointBridge
from script_replay import JsonScriptReplayer


def main():
    parser = argparse.ArgumentParser(description="在 v14_urdf 中回放 v4 JSON 剧本到 /joint_states。")
    parser.add_argument("script_path", help="要回放的 JSON 剧本路径")
    parser.add_argument("--background", action="store_true", help="后台运行并持续保持进程，适合手动观察或后续中断")
    parser.add_argument(
        "--feedback",
        action="store_true",
        help="基于 /joint_states 角度反馈逐步执行（到达目标角度再进入下一步），而不是按时间直接走完。",
    )
    parser.add_argument("--tolerance-deg", type=float, default=1.5, help="feedback 模式的到位容差（deg）。")
    parser.add_argument(
        "--feedback-publish-mode",
        choices=["interpolate", "target"],
        default="interpolate",
        help="feedback 模式下的发布方式：interpolate 会用速度插值推进；target 会持续发布目标角度并等待反馈到位。",
    )
    parser.add_argument("--joint-speed-deg-s", type=float, default=12.0, help="feedback 模式下非回转关节的速度（deg/s）。")
    parser.add_argument("--swing-speed-deg-s", type=float, default=30.0, help="feedback 模式下回转关节的速度（deg/s）。")
    parser.add_argument("--fps", type=float, default=30.0, help="feedback 模式下的发布频率（Hz）。")
    parser.add_argument("--max-step-s", type=float, default=30.0, help="feedback 模式下单步最大等待时间（s）。")
    parser.add_argument("--min-step-s", type=float, default=0.0, help="feedback 模式下单步最短时间（s），用于放慢观察节奏。")
    parser.add_argument("--dwell-s", type=float, default=0.05, help="每步结束后的停顿（s）。")
    args = parser.parse_args()

    bridge = RosJointBridge(node_name="v14_json_script_replay")

    def on_status(info):
        state = info.get("state")
        if state == "started":
            total_s = float(info.get("planned_total_duration_s", 0.0))
            print(
                f"[回放] 开始执行，共 {info.get('total_steps', 0)} 步，"
                f"预计总时长约 {total_s:.1f}s: {args.script_path}"
            )
        elif state == "step":
            print(
                f"[回放] 步骤 {info.get('step_index')}/{info.get('total_steps')} | "
                f"{info.get('joint')} -> {info.get('target_val')}° | "
                f"{info.get('description')} | "
                f"剩余约 {float(info.get('remaining_s', 0.0)):.1f}s"
            )
        elif state == "finished":
            if info.get("finished_normally", False):
                print(
                    f"[回放] 执行完成，共 {info.get('total_steps', 0)} 步，"
                    f"总耗时 {float(info.get('elapsed_total_s', 0.0)):.1f}s。"
                )
            else:
                print(
                    f"[回放] 已中止，执行到途中退出，总耗时 "
                    f"{float(info.get('elapsed_total_s', 0.0)):.1f}s。"
                )

    replayer = JsonScriptReplayer(
        bridge,
        status_callback=on_status,
        feedback_mode=args.feedback,
        feedback_publish_mode=args.feedback_publish_mode,
        tolerance_deg=args.tolerance_deg,
        joint_speed_deg_s=args.joint_speed_deg_s,
        swing_speed_deg_s=args.swing_speed_deg_s,
        fps=args.fps,
        max_step_s=args.max_step_s,
        min_step_s=args.min_step_s,
        dwell_s=args.dwell_s,
    )
    replayer.start(script_path=args.script_path, daemon=args.background)

    if args.background:
        print("[回放] 已在后台线程启动。保持当前终端进程存活即可持续发布。")
        try:
            while replayer.is_running():
                time.sleep(0.5)
        finally:
            bridge.close()
    else:
        try:
            while replayer.is_running():
                time.sleep(0.2)
        finally:
            bridge.close()


if __name__ == "__main__":
    main()
