import os
import sys
import time
import tkinter as tk
from tkinter import ttk, messagebox

from serial.tools import list_ports


sys.path.append(
    os.path.join(
        os.path.dirname(__file__),
        "WitStandardModbus_WT901C485-main",
        "Python",
        "Python-SDK-WT901C485_new",
    )
)
import device_model


class SensorPortMapperGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("倾角传感器 COM 口识别工具")
        self.root.geometry("1250x760")

        self.addr_list = [0x50]
        self.baudrate = 230400
        self.port_vars = {}
        self.cards = {}
        self.devices = {}
        self.closing = False

        self._build_ui()
        self._refresh_ports()
        self.root.after(120, self._refresh_ui)

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        top = ttk.LabelFrame(main, text="使用说明", padding=10)
        top.pack(fill=tk.X, pady=(0, 10))
        tip = (
            "1. 点击“刷新串口”，勾选需要识别的 COM 口；"
            "2. 点击“连接选中串口”；"
            "3. 轻微晃动某一个传感器，观察哪个 COM 卡片的角度/变化量在跳动；"
            "4. 在该卡片里手动标记为大臂/小臂/铲斗/回转。"
        )
        ttk.Label(top, text=tip, foreground="blue").pack(anchor="w")

        toolbar = ttk.LabelFrame(main, text="串口控制", padding=10)
        toolbar.pack(fill=tk.X, pady=(0, 10))

        ttk.Button(toolbar, text="刷新串口", command=self._refresh_ports).pack(side=tk.LEFT, padx=5)
        ttk.Button(toolbar, text="连接选中串口", command=self._connect_selected).pack(side=tk.LEFT, padx=5)
        ttk.Button(toolbar, text="断开全部", command=self._disconnect_all).pack(side=tk.LEFT, padx=5)
        ttk.Button(toolbar, text="全部设为当前基准", command=self._baseline_all).pack(side=tk.LEFT, padx=5)
        ttk.Button(toolbar, text="复制映射结果", command=self._copy_mapping).pack(side=tk.LEFT, padx=5)

        self.port_select_frame = ttk.Frame(toolbar)
        self.port_select_frame.pack(side=tk.LEFT, padx=(15, 0))

        cards_frame = ttk.LabelFrame(main, text="传感器实时识别区", padding=10)
        cards_frame.pack(fill=tk.BOTH, expand=True)

        self.cards_container = ttk.Frame(cards_frame)
        self.cards_container.pack(fill=tk.BOTH, expand=True)

        bottom = ttk.LabelFrame(main, text="当前映射结果", padding=10)
        bottom.pack(fill=tk.X, pady=(10, 0))

        self.result_text = tk.Text(bottom, height=6, state=tk.DISABLED, bg="#f8f8f8")
        self.result_text.pack(fill=tk.X, expand=False)

    def _refresh_ports(self):
        for child in self.port_select_frame.winfo_children():
            child.destroy()

        detected = sorted([p.device for p in list_ports.comports()])
        if not detected:
            ttk.Label(self.port_select_frame, text="未检测到串口").pack(side=tk.LEFT, padx=5)
            return

        new_vars = {}
        for port in detected:
            checked = self.port_vars.get(port).get() if port in self.port_vars else True
            var = tk.BooleanVar(value=checked)
            new_vars[port] = var
            ttk.Checkbutton(self.port_select_frame, text=port, variable=var).pack(side=tk.LEFT, padx=4)
        self.port_vars = new_vars

    def _connect_selected(self):
        selected = [port for port, var in self.port_vars.items() if var.get()]
        if not selected:
            messagebox.showwarning("未选择串口", "请先勾选至少一个 COM 口。")
            return

        for port in selected:
            if port in self.devices:
                continue
            self._connect_port(port)

        self._rebuild_cards()
        self._update_mapping_text()

    def _connect_port(self, port):
        card = self.cards.setdefault(port, self._new_card_state(port))

        def callback(dm, sensor_port=port):
            addr = dm.addrLis[0]
            data = dm.deviceData.get(addr, {})
            if not data:
                return
            card_state = self.cards.get(sensor_port)
            if not card_state:
                return
            card_state["roll"] = data.get("AngX", 0.0)
            card_state["pitch"] = data.get("AngY", 0.0)
            card_state["yaw"] = data.get("AngZ", 0.0)
            card_state["last_update"] = time.time()

        try:
            dev = device_model.DeviceModel(f"传感器_{port}", port, self.baudrate, self.addr_list, callback)
            dev.openDevice()
            dev.startLoopRead()
            self.devices[port] = dev
            card["status"] = "已连接"
        except Exception as exc:
            card["status"] = f"连接失败: {exc}"

    def _disconnect_all(self):
        for port, dev in list(self.devices.items()):
            try:
                dev.stopLoopRead()
            except Exception:
                pass

        time.sleep(0.4)

        for port, dev in list(self.devices.items()):
            try:
                dev.isOpen = False
                dev.closeDevice()
            except Exception:
                pass

        self.devices.clear()
        for card in self.cards.values():
            card["status"] = "未连接"
        self._update_mapping_text()

    def _new_card_state(self, port):
        return {
            "port": port,
            "status": "未连接",
            "label_var": tk.StringVar(value="未标记"),
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "base_roll": 0.0,
            "base_pitch": 0.0,
            "base_yaw": 0.0,
            "last_update": 0.0,
            "widgets": {},
        }

    def _rebuild_cards(self):
        for child in self.cards_container.winfo_children():
            child.destroy()

        ports = sorted(self.cards.keys())
        for idx, port in enumerate(ports):
            card = self.cards[port]
            frame = ttk.LabelFrame(self.cards_container, text=port, padding=10)
            frame.grid(row=idx // 2, column=idx % 2, sticky="nsew", padx=8, pady=8)
            self.cards_container.grid_columnconfigure(idx % 2, weight=1)

            ttk.Label(frame, text="传感器名称:").grid(row=0, column=0, sticky="e", padx=5, pady=3)
            combo = ttk.Combobox(
                frame,
                textvariable=card["label_var"],
                values=["未标记", "大臂", "小臂", "铲斗", "回转"],
                state="readonly",
                width=12,
            )
            combo.grid(row=0, column=1, sticky="w", padx=5, pady=3)
            combo.bind("<<ComboboxSelected>>", lambda _e: self._update_mapping_text())

            ttk.Label(frame, text="连接状态:").grid(row=1, column=0, sticky="e", padx=5, pady=3)
            status_label = ttk.Label(frame, text=card["status"])
            status_label.grid(row=1, column=1, sticky="w", padx=5, pady=3)

            ttk.Button(frame, text="设为当前基准", command=lambda p=port: self._set_baseline(p)).grid(
                row=2, column=0, columnspan=2, sticky="ew", padx=5, pady=6
            )

            roll_label = ttk.Label(frame, text="Roll(X): --")
            roll_label.grid(row=3, column=0, columnspan=2, sticky="w", padx=5, pady=2)
            pitch_label = ttk.Label(frame, text="Pitch(Y): --")
            pitch_label.grid(row=4, column=0, columnspan=2, sticky="w", padx=5, pady=2)
            yaw_label = ttk.Label(frame, text="Yaw(Z): --")
            yaw_label.grid(row=5, column=0, columnspan=2, sticky="w", padx=5, pady=2)

            delta_roll_label = ttk.Label(frame, text="ΔRoll: --")
            delta_roll_label.grid(row=6, column=0, columnspan=2, sticky="w", padx=5, pady=2)
            delta_pitch_label = ttk.Label(frame, text="ΔPitch: --")
            delta_pitch_label.grid(row=7, column=0, columnspan=2, sticky="w", padx=5, pady=2)
            delta_yaw_label = ttk.Label(frame, text="ΔYaw: --")
            delta_yaw_label.grid(row=8, column=0, columnspan=2, sticky="w", padx=5, pady=2)

            hint_label = ttk.Label(frame, text="提示: 主要看谁在跳动", foreground="blue")
            hint_label.grid(row=9, column=0, columnspan=2, sticky="w", padx=5, pady=(8, 2))

            card["widgets"] = {
                "status": status_label,
                "roll": roll_label,
                "pitch": pitch_label,
                "yaw": yaw_label,
                "droll": delta_roll_label,
                "dpitch": delta_pitch_label,
                "dyaw": delta_yaw_label,
            }

    def _set_baseline(self, port):
        card = self.cards.get(port)
        if not card:
            return
        card["base_roll"] = card["roll"]
        card["base_pitch"] = card["pitch"]
        card["base_yaw"] = card["yaw"]

    def _baseline_all(self):
        for port in list(self.cards.keys()):
            self._set_baseline(port)

    def _copy_mapping(self):
        lines = []
        for port in sorted(self.cards.keys()):
            label = self.cards[port]["label_var"].get()
            lines.append(f"{port} -> {label}")
        text = "\n".join(lines) if lines else "暂无映射结果"
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        messagebox.showinfo("复制成功", "当前 COM 对应关系已复制到剪贴板。")

    def _update_mapping_text(self):
        lines = [
            "建议识别方法:",
            "- 晃动某个传感器，观察哪个 COM 的 ΔPitch/ΔRoll 变化最大。",
            "- 大多数机械臂俯仰动作优先看 ΔPitch(Y)。",
            "",
            "当前映射:",
        ]
        for port in sorted(self.cards.keys()):
            label = self.cards[port]["label_var"].get()
            lines.append(f"- {port} -> {label}")

        self.result_text.config(state=tk.NORMAL)
        self.result_text.delete("1.0", tk.END)
        self.result_text.insert(tk.END, "\n".join(lines))
        self.result_text.config(state=tk.DISABLED)

    def _refresh_ui(self):
        now = time.time()
        for port, card in self.cards.items():
            widgets = card.get("widgets")
            if not widgets:
                continue

            d_roll = card["roll"] - card["base_roll"]
            d_pitch = card["pitch"] - card["base_pitch"]
            d_yaw = card["yaw"] - card["base_yaw"]
            stale = now - card["last_update"] > 1.5
            status_text = card["status"]
            if port in self.devices and stale:
                status_text = "已连接, 等待数据..."

            widgets["status"].config(text=status_text)
            widgets["roll"].config(text=f"Roll(X): {card['roll']:7.1f}°")
            widgets["pitch"].config(text=f"Pitch(Y): {card['pitch']:7.1f}°")
            widgets["yaw"].config(text=f"Yaw(Z): {card['yaw']:7.1f}°")
            widgets["droll"].config(text=f"ΔRoll: {d_roll:+6.1f}°")
            widgets["dpitch"].config(text=f"ΔPitch: {d_pitch:+6.1f}°")
            widgets["dyaw"].config(text=f"ΔYaw: {d_yaw:+6.1f}°")

        if not self.closing:
            self.root.after(120, self._refresh_ui)

    def on_closing(self):
        self.closing = True
        self._disconnect_all()
        self.root.destroy()
        os._exit(0)


if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = SensorPortMapperGUI(root)
        root.protocol("WM_DELETE_WINDOW", app.on_closing)
        root.mainloop()
    except KeyboardInterrupt:
        os._exit(0)
