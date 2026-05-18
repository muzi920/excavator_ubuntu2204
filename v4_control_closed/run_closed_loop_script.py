import json
import time
import os
import sys
import argparse
import threading

# 引入底层库
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v1_control_base")))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v3_sensor_read_wit", "WitStandardModbus_WT901C485-main", "Python", "Python-SDK-WT901C485_new")))

from zs_excavator_controller import build_controller
import device_model
from angle_controller import AngleController

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
        
        try:
            for loop in range(1, loop_count + 1):
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
                        
                    ch1 = step.get('ch1_mv', 2000)
                    ch2 = step.get('ch2_mv', 2000)
                    ch3 = step.get('ch3_mv', 2000)
                    
                    ramp_up = step.get('ramp_up_s', 0.0)
                    ramp_down = step.get('ramp_down_s', 0.0)
                    
                    print(f"\n--- [第 {step_num} 步] {desc} (目标: {target_val}) ---")
                    if ramp_up > 0 or ramp_down > 0:
                        print(f"            [柔性控制开启] 加速: {ramp_up}s, 减速标志: {ramp_down}s")
                    
                    # 触发闭环运动
                    self.angle_ctrl.move_joint_to_angle(
                        joint, target_val, tolerance=2.0, 
                        ch1_mv=ch1, ch2_mv=ch2, ch3_mv=ch3,
                        ramp_up_s=ramp_up, ramp_down_s=ramp_down
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
        runner.execute_script(script, loop_count=loop_count)
    except Exception as e:
        print(f"致命错误: {e}")
        runner.close()
