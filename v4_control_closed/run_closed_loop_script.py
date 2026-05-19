import json
import time
import os
import sys
import argparse
import threading
import tkinter as tk
from tkinter import ttk

# 引入底层库
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v1_control_base")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v3_sensor_read_wit", "WitStandardModbus_WT901C485-main", "Python", "Python-SDK-WT901C485_new")))

from zs_excavator_controller import build_controller
import device_model
from angle_controller import AngleController

class ExecutionMonitorGUI:
    def __init__(self, runner):
        self.runner = runner
        self.root = tk.Tk()
        self.root.title("挖掘机 V4 闭环剧本执行监控")
        self.root.geometry("600x450")
        
        # 拦截关闭按钮，防止直接关 GUI 导致后台失控
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        self._build_ui()
        self._update_loop()
        
    def _build_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # --- 任务信息 ---
        task_frame = ttk.LabelFrame(main_frame, text="当前任务状态", padding=10)
        task_frame.pack(fill=tk.X, pady=5)
        
        self.lbl_loop = ttk.Label(task_frame, text="循环进度: -- / --", font=("Arial", 12, "bold"))
        self.lbl_loop.pack(anchor="w", pady=2)
        
        self.lbl_step = ttk.Label(task_frame, text="当前步骤: 等待执行...", font=("Arial", 11))
        self.lbl_step.pack(anchor="w", pady=2)
        
        self.lbl_target = ttk.Label(task_frame, text="目标角度: --°", font=("Arial", 11))
        self.lbl_target.pack(anchor="w", pady=2)
        
        # --- 实时传感器数据 ---
        sensor_frame = ttk.LabelFrame(main_frame, text="实时关节角度", padding=10)
        sensor_frame.pack(fill=tk.X, pady=5)
        
        self.lbl_bucket = ttk.Label(sensor_frame, text="铲斗-小臂 夹角: --°", font=("Arial", 11))
        self.lbl_bucket.grid(row=0, column=0, padx=20, pady=5, sticky="w")
        
        self.lbl_arm = ttk.Label(sensor_frame, text="小臂-大臂 夹角: --°", font=("Arial", 11))
        self.lbl_arm.grid(row=1, column=0, padx=20, pady=5, sticky="w")
        
        self.lbl_boom = ttk.Label(sensor_frame, text="大臂-回转 夹角: --°", font=("Arial", 11))
        self.lbl_boom.grid(row=0, column=1, padx=20, pady=5, sticky="w")
        
        self.lbl_swing = ttk.Label(sensor_frame, text="回转 偏航角: --°", font=("Arial", 11))
        self.lbl_swing.grid(row=1, column=1, padx=20, pady=5, sticky="w")
        
        # --- 实时控制状态 ---
        ctrl_frame = ttk.LabelFrame(main_frame, text="控制器实时状态", padding=10)
        ctrl_frame.pack(fill=tk.X, pady=5)
        
        self.lbl_analog = ttk.Label(ctrl_frame, text="当前输出模拟量: CH1: --, CH2: --, CH3: --", font=("Arial", 11))
        self.lbl_analog.pack(anchor="w", pady=2)
        
        # --- 急停按钮 ---
        btn_frame = ttk.Frame(main_frame)
        btn_frame.pack(fill=tk.X, pady=15)
        
        ttk.Button(btn_frame, text="【紧急停止执行】", command=self.on_closing).pack(ipadx=20, ipady=10)
        
    def _update_loop(self):
        if not self.runner._running:
            self.root.destroy()
            return
            
        # 1. 更新任务进度
        info = self.runner.current_execution_info
        self.lbl_loop.config(text=f"循环进度: {info['loop_current']} / {info['loop_total']}")
        
        if info['step_desc']:
            self.lbl_step.config(text=f"当前步骤: [第 {info['step_num']} 步] {info['step_desc']}")
            if "回转" in info['step_desc']:
                self.lbl_target.config(text=f"目标执行时间: {info['target_val']} 秒")
            else:
                self.lbl_target.config(text=f"目标角度: {info['target_val']}°")
        else:
            self.lbl_step.config(text="当前步骤: 等待执行 / 执行完毕")
            self.lbl_target.config(text="目标角度: --°")
            
        # 2. 更新传感器角度
        d = self.runner.sensor_data
        diff_ba = d['铲斗']['pitch'] - d['小臂']['pitch']
        diff_ab = d['小臂']['pitch'] - d['大臂']['pitch']
        diff_bs = d['大臂']['pitch'] - d['回转']['pitch']
        yaw_s = d['回转']['yaw']
        
        self.lbl_bucket.config(text=f"铲斗-小臂 夹角: {diff_ba:6.1f}°")
        self.lbl_arm.config(text=f"小臂-大臂 夹角: {diff_ab:6.1f}°")
        self.lbl_boom.config(text=f"大臂-回转 夹角: {diff_bs:6.1f}°")
        self.lbl_swing.config(text=f"回转 偏航角: {yaw_s:6.1f}°")
        
        # 3. 更新实时模拟量
        # 从底层控制器读取最后一次下发的模拟量记录
        try:
            # 兼容 v1_control_base 最新的缓存设计
            ch1, ch2, ch3 = self.runner.base_controller.last_analog_values
            self.lbl_analog.config(text=f"当前输出模拟量: CH1: {ch1}, CH2: {ch2}, CH3: {ch3}")
        except AttributeError:
            self.lbl_analog.config(text=f"当前输出模拟量: (不支持实时读取)")
        
        self.root.after(100, self._update_loop)
        
    def on_closing(self):
        print("\n[GUI] 收到关闭/急停请求，正在终止任务...")
        self.runner.close()
        self.root.destroy()
        os._exit(0)

class ClosedLoopScriptRunner:
    def __init__(self, port="/dev/ttyUSB_Controller", baudrate=115200):
        self.base_controller = build_controller(port=port, baudrate=baudrate)
        self.angle_ctrl = AngleController(self.base_controller)
        
        self.sensor_data = {
            "大臂": {"pitch": 0.0, "yaw": 0.0},
            "小臂": {"pitch": 0.0, "yaw": 0.0},
            "铲斗": {"pitch": 0.0, "yaw": 0.0},
            "回转": {"pitch": 0.0, "yaw": 0.0},
        }
        self.devices = []
        self._running = True
        
        # 用于与 GUI 共享的执行状态信息
        self.current_execution_info = {
            "loop_current": 0,
            "loop_total": 0,
            "step_num": 0,
            "step_desc": "",
            "target_val": 0.0
        }

    def init_sensors(self):
        addrLis = [0x50, 0x51, 0x52, 0x53]
        baud = 230400
        ports = [
            "/dev/ttyUSB_Sensor1",
            "/dev/ttyUSB_Sensor2",
            "/dev/ttyUSB_Sensor3",
            "/dev/ttyUSB_Sensor4",
        ]
        
        for port in ports:
            try:
                dev = device_model.DeviceModel(port, port, baud, addrLis, self._sensor_callback(port))
                dev.openDevice()
                dev.startLoopRead()
                self.devices.append(dev)
                print(f"[{port}] 传感器初始化成功")
            except Exception as e:
                print(f"[{port}] 初始化失败: {e}")
                
        # 启动后台线程持续更新传感器数据给控制器
        threading.Thread(target=self._update_loop, daemon=True).start()
        print("[Runner] 等待 2 秒让传感器数据稳定...")
        time.sleep(2.0)

    def _sensor_callback(self, port_name):
        id_to_name = {
            0x50: "铲斗",
            0x51: "小臂",
            0x52: "大臂",
            0x53: "回转"
        }
        def update(dm):
            for addr, name in id_to_name.items():
                data = dm.deviceData.get(addr, {})
                if data and "AngX" in data:
                    self.sensor_data[name]["pitch"] = data.get("AngX", 0.0)
                    self.sensor_data[name]["yaw"] = data.get("AngZ", 0.0)
                    dm.deviceData[addr].clear()
        return update

    def _update_loop(self):
        while self._running:
            self.angle_ctrl.update_sensor_data(self.sensor_data)
            time.sleep(0.05)

    def load_script(self, json_path: str) -> list:
        if not os.path.exists(json_path):
            raise FileNotFoundError(f"找不到剧本文件: {json_path}")
        with open(json_path, 'r', encoding='utf-8') as f:
            script_data = json.load(f)
        print(f"[ScriptRunner] 成功加载闭环剧本: {json_path}，共 {len(script_data)} 个步骤。")
        return script_data

    def execute_script(self, script_data: list, loop_count: int = 1):
        if not self.base_controller.connect():
            print("[警告] 串口连接失败，当前处于离线测试模式 (指令仅打印不会下发)！\n")
            
        print(f"============= 开始执行挖掘机 V4 闭环剧本 (计划循环 {loop_count} 次) =============")
        self.current_execution_info["loop_total"] = loop_count
        
        try:
            for loop in range(1, loop_count + 1):
                self.current_execution_info["loop_current"] = loop
                if loop_count > 1:
                    print(f"\n>>>>>>>>>>> 开始第 {loop}/{loop_count} 次循环 <<<<<<<<<<<")
                    
                for step in script_data:
                    step_num = step.get('step', '?')
                    joint = step.get('joint', '')
                    desc = step.get('description', '')
                    
                    if joint == "swing_yaw":
                        target_val = step.get('duration_s', step.get('target_val', 0.0))
                    else:
                        target_val = step.get('target_val', 0.0)
                        
                    # 更新状态供 GUI 读取
                    self.current_execution_info.update({
                        "step_num": step_num,
                        "step_desc": desc,
                        "target_val": target_val
                    })
                        
                    ch1 = step.get('ch1_mv', 2000)
                    ch2 = step.get('ch2_mv', 2000)
                    ch3 = step.get('ch3_mv', 2000)
                    
                    ramp_up = step.get('ramp_up_s', 0.0)
                    ramp_down = step.get('ramp_down_s', 0.0)
                    
                    print(f"\n--- [第 {step_num} 步] {desc} (目标: {target_val}) ---")
                    if ramp_up > 0 or ramp_down > 0:
                        print(f"            [柔性控制开启] 加速: {ramp_up}s, 减速标志: {ramp_down}s")
                    
                    # 判定是否为初始步骤（不再死板依赖前3步，而是读取剧本中录制的 is_init_step 标识）
                    # 为了兼容老剧本，如果没有这个字段，则降级为判断是否为前 3 步且包含“初始”字样
                    is_init = step.get('is_init_step', False)
                    if not is_init and step_num <= 3 and ("初始" in desc or "归位" in desc):
                        is_init = True
                    
                    # 触发闭环运动
                    self.angle_ctrl.move_joint_to_angle(
                        joint, target_val, tolerance=2.0, 
                        ch1_mv=ch1, ch2_mv=ch2, ch3_mv=ch3,
                        ramp_up_s=ramp_up, ramp_down_s=ramp_down,
                        is_init_step=is_init
                    )
                    
                    # 阻塞等待当前关节运动完成
                    # angle_ctrl 会在后台启动线程并把任务标记在 _running_tasks 中
                    time.sleep(0.1) # 等待线程启动
                    while self.angle_ctrl._running_tasks.get(joint, False):
                        time.sleep(0.1)
                        
                    # 动作之间强制加一个安全间隔
                    time.sleep(0.5)
                
                if loop < loop_count:
                    print(f"\n第 {loop} 次循环完成，等待 1 秒后开始下一次...")
                    time.sleep(1.0)
                    
        except KeyboardInterrupt:
            print("\n[紧急] 检测到 Ctrl+C，正在紧急停止...")
        except Exception as e:
            print(f"\n[异常] 剧本执行中断: {e}")
        finally:
            print("\n============= 剧本执行结束，安全复位 =============")
            self.angle_ctrl.stop_all()
            self.close()

    def close(self):
        self._running = False
        self.angle_ctrl.stop_all()
        for dev in self.devices:
            dev.stopLoopRead()
        time.sleep(0.5)
        for dev in self.devices:
            dev.isOpen = False
            dev.closeDevice()
        self.base_controller.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="运行挖掘机 V4 闭环 JSON 动作剧本")
    parser.add_argument("--json", type=str, required=True,
                        help="要执行的 JSON 剧本文件名或路径")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB_Controller",
                        help="控制器串口路径 (默认: /dev/ttyUSB_Controller)")
    parser.add_argument("--times", type=int, default=1,
                        help="剧本循环执行的次数 (默认: 1)")

    args = parser.parse_args()
    loop_count = max(1, args.times)

    script_path = args.json if os.path.isabs(args.json) else os.path.join(os.path.dirname(__file__), args.json)
    
    runner = ClosedLoopScriptRunner(port=args.port)
    try:
        runner.init_sensors()
        script = runner.load_script(script_path)
        
        # 启动后台线程来执行剧本，主线程运行 GUI
        threading.Thread(
            target=runner.execute_script, 
            args=(script, loop_count),
            daemon=True
        ).start()
        
        # 启动监控界面
        gui = ExecutionMonitorGUI(runner)
        gui.root.mainloop()
        
    except Exception as e:
        print(f"致命错误: {e}")
        runner.close()
