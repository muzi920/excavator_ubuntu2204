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

    replayer = JsonScriptReplayer(bridge, status_callback=on_status)
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
