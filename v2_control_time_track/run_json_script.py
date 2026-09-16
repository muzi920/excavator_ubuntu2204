import json
import time
import os
import sys
import argparse

# 引用调度器
sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from action_scheduler import ActionScheduler

class ScriptRunner:
    def __init__(self, port="/dev/ttyUSB_Controller", baudrate=115200):
        self.scheduler = ActionScheduler(port=port, baudrate=baudrate)
        
    def load_script(self, json_path: str) -> list:
        """加载 JSON 剧本文件"""
        if not os.path.exists(json_path):
            raise FileNotFoundError(f"找不到剧本文件: {json_path}")
            
        with open(json_path, 'r', encoding='utf-8') as f:
            script_data = json.load(f)
            
        print(f"[ScriptRunner] 成功加载剧本: {json_path}，共 {len(script_data)} 个步骤。")
        return script_data
        
    def execute_script(self, script_data: list, loop_count: int = 1):
        """解析并依次执行剧本中的动作"""
        if not self.scheduler.connect():
            print("[警告] 串口连接失败，当前处于离线测试模式 (指令仅打印不会下发)！\n")
            
        print(f"============= 开始执行挖掘机 JSON 剧本 (计划循环 {loop_count} 次) =============")
        
        try:
            for loop in range(1, loop_count + 1):
                if loop_count > 1:
                    print(f"\n>>>>>>>>>>> 开始第 {loop}/{loop_count} 次循环 <<<<<<<<<<<")
                    
                for step in script_data:
                    step_num = step.get('step', '?')
                    action_id = step.get('action', '')
                    desc = step.get('description', '')
                    duration = step.get('duration_s', 1.0)
                    ch1 = step.get('ch1_mv', 2000)
                    ch2 = step.get('ch2_mv', 2000)
                    ch3 = step.get('ch3_mv', 2000)
                    ramp_up = step.get('ramp_up_s', 0.0)
                    ramp_down = step.get('ramp_down_s', 0.0)
                    
                    print(f"\n--- [第 {step_num} 步] {desc} ---")
                    
                    if action_id == "wait":
                        self.scheduler.wait(duration)
                    else:
                        # 动态从 controller 中获取对应的控制函数
                        action_func = getattr(self.scheduler.controller, action_id, None)
                        if not action_func:
                            print(f"[错误] 找不到控制函数 '{action_id}'，跳过此步！")
                            continue
                            
                        self.scheduler.run_action(
                            action_name=desc,
                            action_func=action_func,
                            duration_s=duration,
                            ch1_mv=ch1,
                            ch2_mv=ch2,
                            ch3_mv=ch3,
                            ramp_up_s=ramp_up,
                            ramp_down_s=ramp_down
                        )
                        
                    # 动作之间强制加一个安全间隔 (修改为 0.1s 停顿)
                    time.sleep(0.1)
                
                # 每次循环结束后可以稍微停顿一下
                if loop < loop_count:
                    print(f"\n第 {loop} 次循环完成，等待 1 秒后开始下一次...")
                    time.sleep(1.0)
                    
        except KeyboardInterrupt:
            print("\n[紧急] 检测到 Ctrl+C，正在紧急停止...")
        except Exception as e:
            print(f"\n[异常] 剧本执行中断: {e}")
        finally:
            print("\n============= 剧本执行结束，安全复位 =============")
            self.scheduler.controller.stop_all()
            self.scheduler.close()

if __name__ == "__main__":
    # python3 run_json_script.py --json preset_working_script.json --times 5
    parser = argparse.ArgumentParser(description="运行挖掘机 JSON 动作剧本")
    parser.add_argument("--json", type=str, default="preset_working_script.json",
                        help="要执行的 JSON 剧本文件名或路径 (默认: preset_working_script.json)")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB_Controller",
                        help="控制器串口路径 (默认: /dev/ttyUSB_Controller)")
    parser.add_argument("--times", type=int, default=1,
                        help="剧本循环执行的次数 (默认: 1)")

    args = parser.parse_args()

    # 确保循环次数至少为 1
    loop_count = max(1, args.times)

    # 获取剧本的绝对路径
    # 如果用户传入的是绝对路径，os.path.join 会直接使用用户的路径；否则会基于当前目录拼接
    script_path = args.json if os.path.isabs(args.json) else os.path.join(os.path.dirname(__file__), args.json)
    
    runner = ScriptRunner(port=args.port)
    try:
        script = runner.load_script(script_path)
        runner.execute_script(script, loop_count=loop_count)
    except Exception as e:
        print(f"致命错误: {e}")
