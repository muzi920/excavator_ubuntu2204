import json
import os
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

CURRENT_DIR = os.path.dirname(__file__)
if CURRENT_DIR not in sys.path:
    sys.path.append(CURRENT_DIR)

from ros_joint_bridge import RosJointBridge
from sim_angle_controller import SimAngleController
from script_replay import JsonScriptReplayer


class V4UrdfSimGUI:
    """位于 v14_urdf 内、沿用 v4 关节语义的仿真版 GUI。"""

    def __init__(self, root):
        self.root = root
        self.root.title("V4 挖掘机 URDF 仿真控制测试系统")
        self.root.geometry("800x600")

        self.ros_bridge = RosJointBridge()
        self.angle_ctrl = SimAngleController(self.ros_bridge)
        self.script_replayer = JsonScriptReplayer(
            self.ros_bridge, status_callback=self._on_replay_status
        )

        self.sensor_data = {
            "大臂": {"pitch": 0.0, "yaw": 0.0},
            "小臂": {"pitch": 0.0, "yaw": 0.0},
            "铲斗": {"pitch": 0.0, "yaw": 0.0},
            "回转": {"pitch": 0.0, "yaw": 0.0},
        }

        self.target_bucket_arm = tk.DoubleVar(value=0.0)
        self.target_arm_boom = tk.DoubleVar(value=0.0)
        self.target_boom_swing = tk.DoubleVar(value=0.0)
        self.target_swing_yaw = tk.DoubleVar(value=0.0)

        self.ch1_var = tk.IntVar(value=0)
        self.ch2_var = tk.IntVar(value=0)
        self.ch3_var = tk.IntVar(value=2000)
        self.ramp_up_var = tk.DoubleVar(value=0.2)
        self.ramp_down_var = tk.DoubleVar(value=0.2)

        self.is_recording = False
        self.recorded_script = []
        self.current_angles = {
            "bucket_arm": 0.0,
            "arm_boom": 0.0,
            "boom_swing": 0.0,
            "swing_yaw": 0.0,
        }
        self.loaded_script = []
        self.loaded_script_path = ""
        self.loaded_metadata = {}
        self.selected_step_index = -1
        self.replay_status_text = "未加载剧本"
        self.replay_info = {
            "step_index": 0,
            "total_steps": 0,
            "description": "",
            "target_val": 0.0,
            "remaining_s": 0.0,
            "elapsed_s": 0.0,
        }

        self.json_dir = os.path.join(CURRENT_DIR, "json")
        os.makedirs(self.json_dir, exist_ok=True)

        self._build_ui()
        self._update_loop()

    def _build_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        status_frame = ttk.LabelFrame(main_frame, text="URDF 仿真关节状态", padding=10)
        status_frame.pack(fill=tk.X, pady=5)
        self.lbl_bucket_arm = ttk.Label(status_frame, text="铲斗-小臂 夹角: --°")
        self.lbl_bucket_arm.grid(row=0, column=0, padx=20, pady=5, sticky="w")
        self.lbl_arm_boom = ttk.Label(status_frame, text="小臂-大臂 夹角: --°")
        self.lbl_arm_boom.grid(row=1, column=0, padx=20, pady=5, sticky="w")
        self.lbl_boom_swing = ttk.Label(status_frame, text="大臂-回转 夹角: --°")
        self.lbl_boom_swing.grid(row=0, column=1, padx=20, pady=5, sticky="w")
        self.lbl_swing_yaw = ttk.Label(status_frame, text="回转 偏航角: --°")
        self.lbl_swing_yaw.grid(row=1, column=1, padx=20, pady=5, sticky="w")

        analog_frame = ttk.LabelFrame(main_frame, text="保留 v4 参数显示", padding=10)
        analog_frame.pack(fill=tk.X, pady=5)
        ttk.Label(analog_frame, text="CH1(左):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(analog_frame, textvariable=self.ch1_var, width=6, state="disabled").pack(side=tk.LEFT, padx=5)
        ttk.Label(analog_frame, text="CH2(右):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(analog_frame, textvariable=self.ch2_var, width=6, state="disabled").pack(side=tk.LEFT, padx=5)
        ttk.Label(analog_frame, text="CH3(液压):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(analog_frame, textvariable=self.ch3_var, width=6).pack(side=tk.LEFT, padx=5)
        ttk.Label(analog_frame, text="|  加速时间(s):").pack(side=tk.LEFT, padx=(15, 5))
        ttk.Entry(analog_frame, textvariable=self.ramp_up_var, width=5).pack(side=tk.LEFT, padx=5)
        ttk.Label(analog_frame, text="减速时间(s):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(analog_frame, textvariable=self.ramp_down_var, width=5).pack(side=tk.LEFT, padx=5)

        ctrl_frame = ttk.LabelFrame(main_frame, text="闭环角度目标控制（发布到 /joint_states）", padding=10)
        ctrl_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self._create_ctrl_row(ctrl_frame, 0, "铲斗-小臂", "bucket_arm", self.target_bucket_arm, "目标角度(°):")
        self._create_ctrl_row(ctrl_frame, 1, "小臂-大臂", "arm_boom", self.target_arm_boom, "目标角度(°):")
        self._create_ctrl_row(ctrl_frame, 2, "大臂-回转", "boom_swing", self.target_boom_swing, "目标角度(°):")
        self._create_ctrl_row(ctrl_frame, 3, "回转动作", "swing_yaw", self.target_swing_yaw, "目标角度(°): (正右负左)")

        record_frame = ttk.Frame(main_frame)
        record_frame.pack(fill=tk.X, pady=10)
        self.btn_record = tk.Button(
            record_frame,
            text="🔴 开始录制剧本",
            command=self._toggle_recording,
            bg="#ffcccc",
            width=15,
        )
        self.btn_record.pack(side=tk.LEFT, padx=10)
        ttk.Button(record_frame, text="💾 保存为 JSON 剧本", command=self._save_script, width=20).pack(side=tk.LEFT, padx=10)

        replay_frame = ttk.LabelFrame(main_frame, text="JSON 剧本回放", padding=10)
        replay_frame.pack(fill=tk.X, pady=5)
        self.lbl_script_path = ttk.Label(replay_frame, text="当前剧本: 未加载")
        self.lbl_script_path.grid(row=0, column=0, columnspan=5, padx=5, pady=5, sticky="w")
        self.lbl_replay_status = ttk.Label(replay_frame, text="状态: 未加载剧本")
        self.lbl_replay_status.grid(row=1, column=0, columnspan=5, padx=5, pady=5, sticky="w")
        ttk.Button(replay_frame, text="📂 加载 JSON 剧本", command=self._load_script).grid(row=2, column=0, padx=5, pady=5, sticky="w")
        ttk.Button(replay_frame, text="▶ 执行当前剧本", command=self._start_loaded_script).grid(row=2, column=1, padx=5, pady=5, sticky="w")
        ttk.Button(replay_frame, text="■ 停止剧本", command=self._stop_loaded_script).grid(row=2, column=2, padx=5, pady=5, sticky="w")
        ttk.Button(replay_frame, text="⏮ 上一步", command=self._select_prev_step).grid(row=2, column=3, padx=5, pady=5, sticky="w")
        ttk.Button(replay_frame, text="⏭ 下一步", command=self._select_next_step).grid(row=2, column=4, padx=5, pady=5, sticky="w")
        ttk.Button(replay_frame, text="▶ 执行选中步", command=self._execute_selected_step).grid(row=3, column=0, padx=5, pady=5, sticky="w")
        ttk.Button(replay_frame, text="↺ 重置到步骤0", command=self._reset_step_selection).grid(row=3, column=1, padx=5, pady=5, sticky="w")
        self.lbl_selected_step = ttk.Label(replay_frame, text="当前选中: 无")
        self.lbl_selected_step.grid(row=3, column=2, columnspan=3, padx=5, pady=5, sticky="w")

        steps_frame = ttk.LabelFrame(main_frame, text="手动步进验证", padding=10)
        steps_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        self.steps_listbox = tk.Listbox(steps_frame, height=10)
        self.steps_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.steps_listbox.bind("<<ListboxSelect>>", self._on_step_select)
        self.steps_listbox.bind("<Double-Button-1>", self._on_step_double_click)
        steps_scrollbar = ttk.Scrollbar(steps_frame, orient=tk.VERTICAL, command=self.steps_listbox.yview)
        steps_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.steps_listbox.config(yscrollcommand=steps_scrollbar.set)

        ttk.Button(main_frame, text="【停止仿真控制】", command=self.angle_ctrl.stop_all).pack(pady=10, ipadx=20, ipady=10)

    def _toggle_recording(self):
        if not self.is_recording:
            self.is_recording = True
            self.recorded_script = []
            self.btn_record.config(text="⏹ 停止录制剧本", bg="#ccffcc")
            messagebox.showinfo("开始录制", "已开始录制剧本。现在下发的每次移动都会被记录。")
        else:
            self.is_recording = False
            self.btn_record.config(text="🔴 开始录制剧本", bg="#ffcccc")
            messagebox.showinfo("停止录制", f"录制已停止，当前共记录了 {len(self.recorded_script)} 个动作。")

    def _save_script(self):
        if self.is_recording:
            messagebox.showwarning("警告", "请先停止录制，再进行保存。")
            return

        if not self.recorded_script:
            messagebox.showwarning("提示", "当前没有录制任何动作。")
            return

        file_path = filedialog.asksaveasfilename(
            defaultextension=".json",
            initialdir=self.json_dir,
            title="保存 URDF 仿真剧本",
            filetypes=[("JSON files", "*.json")],
        )
        if file_path:
            with open(file_path, "w", encoding="utf-8") as f:
                json.dump(self.recorded_script, f, ensure_ascii=False, indent=2)
            messagebox.showinfo("保存成功", f"成功保存 {len(self.recorded_script)} 步动作到:\n{file_path}")

    def _load_script(self):
        default_dir = os.path.abspath(os.path.join(CURRENT_DIR, "..", "json"))
        file_path = filedialog.askopenfilename(
            initialdir=default_dir,
            title="选择要回放的 JSON 剧本",
            filetypes=[("JSON files", "*.json")],
        )
        if not file_path:
            return
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                raw = json.load(f)
            if isinstance(raw, dict):
                self.loaded_metadata = raw.get("metadata", {})
            else:
                self.loaded_metadata = {}
            self.loaded_script = self.script_replayer.load_script(file_path)
            self.loaded_script_path = file_path
            self.selected_step_index = 0 if self.loaded_script else -1
            self.replay_status_text = f"已加载，共 {len(self.loaded_script)} 步"
            self.lbl_script_path.config(text=f"当前剧本: {file_path}")
            self.lbl_replay_status.config(text=f"状态: {self.replay_status_text}")
            self._refresh_steps_list()
            self._update_selected_step_label()
        except Exception as e:
            messagebox.showerror("加载失败", str(e))

    def _start_loaded_script(self):
        if not self.loaded_script:
            messagebox.showwarning("提示", "请先加载 JSON 剧本。")
            return
        if self.script_replayer.is_running():
            messagebox.showwarning("提示", "当前已有剧本在执行。")
            return

        try:
            self.script_replayer.start(script=self.loaded_script)
        except Exception as e:
            messagebox.showerror("执行失败", str(e))

    def _stop_loaded_script(self):
        self.script_replayer.stop()
        self.replay_status_text = "已请求停止剧本"
        self.lbl_replay_status.config(text=f"状态: {self.replay_status_text}")

    def _refresh_steps_list(self):
        self.steps_listbox.delete(0, tk.END)
        for idx, step in enumerate(self.loaded_script):
            joint = step.get("joint", "?")
            desc = step.get("description", "")
            target = step.get("target_val", 0.0)
            self.steps_listbox.insert(
                tk.END,
                f"{idx + 1:02d}. {joint} -> {target:.2f} | {desc}",
            )
        if self.selected_step_index >= 0 and self.selected_step_index < len(self.loaded_script):
            self.steps_listbox.selection_clear(0, tk.END)
            self.steps_listbox.selection_set(self.selected_step_index)
            self.steps_listbox.see(self.selected_step_index)

    def _update_selected_step_label(self):
        if self.selected_step_index < 0 or self.selected_step_index >= len(self.loaded_script):
            self.lbl_selected_step.config(text="当前选中: 无")
            return
        step = self.loaded_script[self.selected_step_index]
        self.lbl_selected_step.config(
            text=(
                f"当前选中: 第 {self.selected_step_index + 1} 步 | "
                f"{step.get('joint')} -> {float(step.get('target_val', 0.0)):.2f}° | "
                f"{step.get('description', '')}"
            )
        )

    def _on_step_select(self, _event=None):
        selection = self.steps_listbox.curselection()
        if not selection:
            return
        self.selected_step_index = int(selection[0])
        self._update_selected_step_label()

    def _on_step_double_click(self, _event=None):
        self._on_step_select()
        self._execute_selected_step()

    def _select_prev_step(self):
        if not self.loaded_script:
            return
        if self.selected_step_index <= 0:
            self.selected_step_index = 0
        else:
            self.selected_step_index -= 1
        self._refresh_steps_list()
        self._update_selected_step_label()

    def _select_next_step(self):
        if not self.loaded_script:
            return
        if self.selected_step_index < 0:
            self.selected_step_index = 0
        elif self.selected_step_index >= len(self.loaded_script) - 1:
            self.selected_step_index = len(self.loaded_script) - 1
        else:
            self.selected_step_index += 1
        self._refresh_steps_list()
        self._update_selected_step_label()

    def _reset_step_selection(self):
        if not self.loaded_script:
            self.selected_step_index = -1
        else:
            self.selected_step_index = 0
        self._refresh_steps_list()
        self._update_selected_step_label()
        self.replay_status_text = "已重置到步骤 0，请手动选择并执行"
        self.lbl_replay_status.config(text=f"状态: {self.replay_status_text}")

    def _execute_selected_step(self):
        if not self.loaded_script:
            messagebox.showwarning("提示", "请先加载 JSON 剧本。")
            return
        if self.script_replayer.is_running():
            messagebox.showwarning("提示", "当前有整段剧本在执行，请先停止。")
            return
        if self.selected_step_index < 0 or self.selected_step_index >= len(self.loaded_script):
            messagebox.showwarning("提示", "请先选中要执行的步骤。")
            return

        step = self.loaded_script[self.selected_step_index]
        joint_name = step.get("joint")
        target_val = float(step.get("target_val", 0.0))
        self.angle_ctrl.move_joint_to_angle(
            joint_name,
            target_val,
            tolerance=1.5,
            ch1_mv=0,
            ch2_mv=0,
            ch3_mv=int(step.get("ch3_mv", self.ch3_var.get())),
            ramp_up_s=float(step.get("ramp_up_s", self.ramp_up_var.get())),
            ramp_down_s=float(step.get("ramp_down_s", self.ramp_down_var.get())),
        )
        self.replay_status_text = (
            f"已执行第 {self.selected_step_index + 1} 步 | "
            f"{joint_name} -> {target_val:.2f}°"
        )
        self.lbl_replay_status.config(text=f"状态: {self.replay_status_text}")

    def _on_replay_status(self, info):
        state = info.get("state")
        if state == "started":
            self.replay_info["total_steps"] = int(info.get("total_steps", 0))
            self.replay_status_text = f"执行中: 0/{self.replay_info['total_steps']}"
        elif state == "step":
            self.replay_info["step_index"] = int(info.get("step_index", 0))
            self.replay_info["total_steps"] = int(info.get("total_steps", 0))
            self.replay_info["description"] = info.get("description", "")
            self.replay_info["target_val"] = float(info.get("target_val", 0.0))
            self.replay_info["remaining_s"] = float(info.get("remaining_s", 0.0))
            self.replay_info["elapsed_s"] = float(info.get("elapsed_s", 0.0))
            self.replay_status_text = (
                f"执行中: {self.replay_info['step_index']}/"
                f"{self.replay_info['total_steps']} | "
                f"{self.replay_info['description']} -> {self.replay_info['target_val']:.1f}° | "
                f"剩余约 {self.replay_info['remaining_s']:.1f}s"
            )
        elif state == "finished":
            if info.get("finished_normally", False):
                self.replay_status_text = (
                    f"剧本执行完成 | 总耗时 {float(info.get('elapsed_total_s', 0.0)):.1f}s"
                )
            else:
                self.replay_status_text = (
                    f"剧本已中止 | 已耗时 {float(info.get('elapsed_total_s', 0.0)):.1f}s"
                )

    def _record_current_angle(self, joint_name, label_text, target_var, is_init=False):
        if not self.is_recording:
            messagebox.showwarning("提示", "请先点击下方的『🔴 开始录制剧本』按钮。")
            return

        current_val = round(self.current_angles.get(joint_name, 0.0), 1)
        target_var.set(current_val)
        record_item = {
            "step": len(self.recorded_script) + 1,
            "joint": joint_name,
            "description": f"{label_text}(手动示教{' - 初始位置' if is_init else ''})",
            "ch1_mv": 0,
            "ch2_mv": 0,
            "ch3_mv": self.ch3_var.get(),
            "ramp_up_s": self.ramp_up_var.get(),
            "ramp_down_s": self.ramp_down_var.get(),
            "target_val": current_val,
        }
        if is_init:
            record_item["is_init_step"] = True
        self.recorded_script.append(record_item)

    def _handle_move(self, joint_name, label_text, target_val):
        if self.is_recording:
            self.recorded_script.append(
                {
                    "step": len(self.recorded_script) + 1,
                    "joint": joint_name,
                    "description": label_text,
                    "ch1_mv": 0,
                    "ch2_mv": 0,
                    "ch3_mv": self.ch3_var.get(),
                    "ramp_up_s": self.ramp_up_var.get(),
                    "ramp_down_s": self.ramp_down_var.get(),
                    "target_val": target_val,
                }
            )

        self.angle_ctrl.move_joint_to_angle(
            joint_name,
            target_val,
            tolerance=2.0,
            ch1_mv=0,
            ch2_mv=0,
            ch3_mv=self.ch3_var.get(),
            ramp_up_s=self.ramp_up_var.get(),
            ramp_down_s=self.ramp_down_var.get(),
        )

    def _create_ctrl_row(self, parent, row, label_text, joint_name, target_var, entry_label):
        ttk.Label(parent, text=f"{label_text} {entry_label}").grid(row=row, column=0, padx=10, pady=10, sticky="e")
        ttk.Entry(parent, textvariable=target_var, width=15).grid(row=row, column=1, padx=5, pady=10)
        ttk.Button(
            parent,
            text=f"开始移动 {label_text}",
            command=lambda: self._handle_move(joint_name, label_text, target_var.get()),
        ).grid(row=row, column=2, padx=10, pady=10)
        ttk.Button(
            parent,
            text="📍 记录当前角度",
            command=lambda j=joint_name, l=label_text, v=target_var: self._record_current_angle(j, l, v, is_init=False),
        ).grid(row=row, column=3, padx=5, pady=10)
        ttk.Button(
            parent,
            text="🏠 记录为初始位置",
            command=lambda j=joint_name, l=label_text, v=target_var: self._record_current_angle(j, l, v, is_init=True),
        ).grid(row=row, column=4, padx=5, pady=10)

    def _update_loop(self):
        angles = self.ros_bridge.get_v4_angles_from_joint_states_deg()
        if angles is not None:
            ts = angles.get("ts", time.time())
            boom = float(angles.get("boom_swing", 0.0))
            arm = float(angles.get("arm_boom", 0.0))
            bucket = float(angles.get("bucket_arm", 0.0))
            swing = float(angles.get("swing_yaw", 0.0))

            self.sensor_data["回转"]["pitch"] = 0.0
            self.sensor_data["回转"]["yaw"] = swing
            self.sensor_data["回转"]["ts"] = ts

            self.sensor_data["大臂"]["pitch"] = boom
            self.sensor_data["大臂"]["yaw"] = 0.0
            self.sensor_data["大臂"]["ts"] = ts

            self.sensor_data["小臂"]["pitch"] = boom + arm
            self.sensor_data["小臂"]["yaw"] = 0.0
            self.sensor_data["小臂"]["ts"] = ts

            self.sensor_data["铲斗"]["pitch"] = boom + arm + bucket
            self.sensor_data["铲斗"]["yaw"] = 0.0
            self.sensor_data["铲斗"]["ts"] = ts

        self.angle_ctrl.update_sensor_data(self.sensor_data)

        d = self.sensor_data
        diff_ba = d["铲斗"]["pitch"] - d["小臂"]["pitch"]
        diff_ab = d["小臂"]["pitch"] - d["大臂"]["pitch"]
        diff_bs = d["大臂"]["pitch"] - d["回转"]["pitch"]
        yaw_s = d["回转"]["yaw"]

        self.lbl_bucket_arm.config(text=f"铲斗-小臂 夹角: {diff_ba:6.1f}°")
        self.lbl_arm_boom.config(text=f"小臂-大臂 夹角: {diff_ab:6.1f}°")
        self.lbl_boom_swing.config(text=f"大臂-回转 夹角: {diff_bs:6.1f}°")
        self.lbl_swing_yaw.config(text=f"回转 偏航角: {yaw_s:6.1f}°")

        self.current_angles["bucket_arm"] = diff_ba
        self.current_angles["arm_boom"] = diff_ab
        self.current_angles["boom_swing"] = diff_bs
        self.current_angles["swing_yaw"] = yaw_s

        self.lbl_replay_status.config(text=f"状态: {self.replay_status_text}")

        self.root.after(50, self._update_loop)

    def on_closing(self):
        try:
            self.angle_ctrl.stop_all()
        except Exception:
            pass
        try:
            self.script_replayer.stop()
        except Exception:
            pass
        try:
            self.ros_bridge.close()
        except Exception:
            pass
        self.root.destroy()
        os._exit(0)


if __name__ == "__main__":
    root = tk.Tk()
    app = V4UrdfSimGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
