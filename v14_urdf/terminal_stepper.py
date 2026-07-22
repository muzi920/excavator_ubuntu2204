import argparse
import json
import os
import sys
import time


CURRENT_DIR = os.path.dirname(__file__)
if CURRENT_DIR not in sys.path:
    sys.path.append(CURRENT_DIR)

from ros_joint_bridge import RosJointBridge
from sim_angle_controller import SimAngleController


def load_script_with_metadata(script_path):
    with open(script_path, "r", encoding="utf-8") as f:
        raw = json.load(f)

    if isinstance(raw, list):
        return {}, raw
    if isinstance(raw, dict) and isinstance(raw.get("script"), list):
        return raw.get("metadata", {}), raw["script"]
    raise ValueError("JSON 剧本格式错误，根节点必须是数组，或包含 script 数组字段。")


class TerminalStepper:
    def __init__(self, script_path):
        self.script_path = os.path.abspath(script_path)
        self.metadata, self.script = load_script_with_metadata(self.script_path)
        self.index = 0 if self.script else -1
        self.bridge = RosJointBridge(node_name="v14_terminal_stepper")
        self.ctrl = SimAngleController(self.bridge)

    def close(self):
        self.bridge.close()

    def get_angles(self):
        angles = self.bridge.get_v4_angles_from_joint_states_deg()
        if angles is None:
            return {
                "swing_yaw": 0.0,
                "boom_swing": 0.0,
                "arm_boom": 0.0,
                "bucket_arm": 0.0,
            }
        return angles

    def print_header(self):
        print("\n=== v14_urdf 终端步进控制 ===")
        print("脚本:", self.script_path)
        print("步数:", len(self.script))
        if self.metadata:
            print("metadata 已加载")
            dig = self.metadata.get("dig_point", {})
            dump = self.metadata.get("dump_point", {})
            if dig:
                print(
                    f"挖掘点: r={dig.get('radius', 0):.3f}m, "
                    f"yaw={dig.get('swing_yaw', 0):.2f}°, "
                    f"z={dig.get('z', 0):.3f}"
                )
            if dump:
                print(
                    f"卸料点: r={dump.get('radius', 0):.3f}m, "
                    f"yaw={dump.get('swing_yaw', 0):.2f}°, "
                    f"safe_z={dump.get('safe_dump_z', 0):.3f}"
                )
        print(
            "命令: help, list, show, angles, next, prev, goto N, run, runall, reload, reset, quit"
        )

    def print_help(self):
        print("\n可用命令:")
        print("  help           显示帮助")
        print("  list           列出全部步骤")
        print("  show           显示当前选中步骤")
        print("  angles         显示当前 /joint_states 角度")
        print("  next           选择下一步并执行")
        print("  prev           选择上一步并执行")
        print("  goto N         选中第 N 步（不执行）")
        print("  run            执行当前选中步骤")
        print("  runall         从当前选中步骤开始逐步执行到末尾")
        print("  reload         重新读取 JSON 文件")
        print("  reset          把选中步骤重置到第 1 步")
        print("  quit           退出")

    def print_angles(self):
        angles = self.get_angles()
        print(
            f"当前角度 | swing_yaw={angles.get('swing_yaw', 0.0):.2f}°, "
            f"boom_swing={angles.get('boom_swing', 0.0):.2f}°, "
            f"arm_boom={angles.get('arm_boom', 0.0):.2f}°, "
            f"bucket_arm={angles.get('bucket_arm', 0.0):.2f}°"
        )

    def print_current_step(self):
        if self.index < 0 or self.index >= len(self.script):
            print("当前没有可选步骤。")
            return
        step = self.script[self.index]
        print(
            f"当前步骤 {self.index + 1}/{len(self.script)} | "
            f"{step.get('joint')} -> {float(step.get('target_val', 0.0)):.2f}° | "
            f"{step.get('description', '')}"
        )

    def list_steps(self):
        if not self.script:
            print("脚本为空。")
            return
        print("")
        for idx, step in enumerate(self.script, start=1):
            prefix = "->" if idx - 1 == self.index else "  "
            print(
                f"{prefix} {idx:02d}. {step.get('joint')} -> "
                f"{float(step.get('target_val', 0.0)):.2f}° | "
                f"{step.get('description', '')}"
            )

    def execute_step(self, index):
        if index < 0 or index >= len(self.script):
            print("步骤索引越界。")
            return False
        step = self.script[index]
        joint_name = step.get("joint")
        target_val = float(step.get("target_val", 0.0))
        self.ctrl.move_joint_to_angle(
            joint_name,
            target_val,
            tolerance=1.5,
            ch1_mv=0,
            ch2_mv=0,
            ch3_mv=int(step.get("ch3_mv", 3500)),
            ramp_up_s=float(step.get("ramp_up_s", 0.2)),
            ramp_down_s=float(step.get("ramp_down_s", 0.2)),
        )
        self.index = index
        print(
            f"已执行第 {index + 1} 步 | {joint_name} -> {target_val:.2f}° | "
            f"{step.get('description', '')}"
        )
        return True

    def run_next(self):
        if not self.script:
            print("脚本为空。")
            return
        next_index = min(self.index + 1, len(self.script) - 1)
        self.execute_step(next_index)

    def run_prev(self):
        if not self.script:
            print("脚本为空。")
            return
        prev_index = max(self.index - 1, 0)
        self.execute_step(prev_index)

    def goto(self, index_1_based):
        index = index_1_based - 1
        if index < 0 or index >= len(self.script):
            print("步骤编号越界。")
            return
        self.index = index
        self.print_current_step()

    def reload(self):
        self.metadata, self.script = load_script_with_metadata(self.script_path)
        self.index = 0 if self.script else -1
        print("已重新加载脚本。")
        self.print_current_step()

    def run_all_from_current(self):
        if not self.script:
            print("脚本为空。")
            return
        if self.index < 0:
            self.index = 0
        print(f"开始从第 {self.index + 1} 步执行到末尾。按 Ctrl+C 可中断。")
        try:
            for i in range(self.index, len(self.script)):
                self.execute_step(i)
                step = self.script[i]
                ramp_up = float(step.get("ramp_up_s", 0.2))
                ramp_down = float(step.get("ramp_down_s", 0.2))
                joint_name = step.get("joint")
                current = self.get_angles().get(joint_name, 0.0)
                target = float(step.get("target_val", 0.0))
                speed_deg_s = 90.0 if joint_name == "swing_yaw" else 35.0
                duration = max(abs(target - current) / speed_deg_s + ramp_up + ramp_down, 0.35)
                time.sleep(duration + 0.1)
        except KeyboardInterrupt:
            print("\n已手动中断 runall。")

    def repl(self):
        self.print_header()
        self.print_current_step()
        self.print_angles()
        while True:
            try:
                raw = input("\nstepper> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\n退出终端步进控制。")
                break

            if not raw:
                continue

            if raw == "help":
                self.print_help()
            elif raw == "list":
                self.list_steps()
            elif raw == "show":
                self.print_current_step()
            elif raw == "angles":
                self.print_angles()
            elif raw == "next":
                self.run_next()
            elif raw == "prev":
                self.run_prev()
            elif raw == "run":
                self.execute_step(self.index)
            elif raw == "runall":
                self.run_all_from_current()
            elif raw == "reload":
                self.reload()
            elif raw == "reset":
                self.index = 0 if self.script else -1
                self.print_current_step()
            elif raw.startswith("goto "):
                try:
                    idx = int(raw.split()[1])
                except Exception:
                    print("用法: goto 3")
                    continue
                self.goto(idx)
            elif raw in {"quit", "exit"}:
                print("退出终端步进控制。")
                break
            else:
                print("未知命令。输入 help 查看可用命令。")


def main():
    parser = argparse.ArgumentParser(description="v14_urdf 终端步进控制器。")
    parser.add_argument(
        "script_path",
        nargs="?",
        default=os.path.join(CURRENT_DIR, "json", "generated_dig_dump_trajectory.json"),
        help="要加载的 JSON 剧本路径",
    )
    args = parser.parse_args()

    stepper = TerminalStepper(args.script_path)
    try:
        stepper.repl()
    finally:
        stepper.close()


if __name__ == "__main__":
    main()
