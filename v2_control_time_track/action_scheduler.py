import time
import sys
import os
from typing import Callable, Optional

# 为了能引用到 v1 下的控制器，我们把上一级目录加到 sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from v1.zs_excavator_controller import build_controller, ExcavatorController

class ActionScheduler:
    """
    挖掘机动作调度器 (v2)。
    允许你像写剧本一样，编排一系列带时间参数的动作。
    """
    def __init__(self, port: str = "COM5", baudrate: int = 115200):
        self.controller = build_controller(port, baudrate)
        
    def connect(self) -> bool:
        return self.controller.connect()
        
    def close(self) -> None:
        self.controller.close()

    def run_action(self, action_name: str, action_func: Callable, duration_s: float, 
                   ch1_mv: int = 2000, ch2_mv: int = 2000, ch3_mv: int = 2000):
        """
        执行单个指定时间的动作。
        
        参数:
        - action_name: 动作名称，仅用于打印日志
        - action_func: 要执行的 controller 的方法引用 (如 controller.boom_up)
        - duration_s: 动作持续时间 (秒)
        - ch1_mv, ch2_mv, ch3_mv: 动作期间给予的模拟量推力 (默认 2000mV)
        """
        print(f"\n[Scheduler] 开始执行 -> {action_name} | 持续: {duration_s}s | 模拟量: [{ch1_mv}, {ch2_mv}, {ch3_mv}]")
        
        # 1. 设置驱动推力 (模拟量)
        self.controller.set_analog(ch1_mv, ch2_mv, ch3_mv)
        
        # 2. 触发继电器动作
        action_func()
        
        # 3. 阻塞等待动作完成
        time.sleep(duration_s)
        
        # 4. 安全保护，停止所有动作（模拟量不会清零，仅停止继电器）
        self.controller.stop_all()
        print(f"[Scheduler] 停止执行 -> {action_name}")
        
    def wait(self, duration_s: float):
        """仅仅是发呆等待"""
        print(f"[Scheduler] 发呆等待 {duration_s}s...")
        time.sleep(duration_s)

    def interactive_mode(self):
        """交互式手动控制端口"""
        print("\n============= 进入手动控制模式 =============")
        print("提示: 随时可以按 Ctrl+C 退出\n")
        
        # 预设好所有动作的映射字典
        actions = {
            "1": ("双侧前进", lambda: self.controller.drive_forward(self._temp_mv, self._temp_mv)),
            "2": ("双侧后退", lambda: self.controller.drive_backward(self._temp_mv, self._temp_mv)),
            "3": ("机身左转", lambda: self.controller.turn_left(self._temp_mv, self._temp_mv)),
            "4": ("机身右转", lambda: self.controller.turn_right(self._temp_mv, self._temp_mv)),
            "5": ("左履带前进", lambda: self.controller.left_track_forward(self._temp_mv)),
            "6": ("左履带后退", lambda: self.controller.left_track_backward(self._temp_mv)),
            "7": ("右履带前进", lambda: self.controller.right_track_forward(self._temp_mv)),
            "8": ("右履带后退", lambda: self.controller.right_track_backward(self._temp_mv)),
            "9": ("大臂抬起", self.controller.boom_up),
            "10": ("大臂落下", self.controller.boom_down),
            "11": ("铲斗回拉", self.controller.bucket_in),
            "12": ("铲斗外推", self.controller.bucket_out),
            "13": ("小臂回拉", self.controller.arm_pull),
            "14": ("小臂前推", self.controller.arm_push),
            "15": ("回转左转", self.controller.swing_left),
            "16": ("回转右转", self.controller.swing_right),
            "0": ("仅发呆(Wait)", lambda: None)
        }
        
        while True:
            try:
                print("\n--- 动作列表 ---")
                for k, v in actions.items():
                    print(f"[{k}] {v[0]}")
                print("----------------")
                
                choice = input("请选择动作序号 (直接回车退出): ").strip()
                if not choice:
                    break
                
                if choice not in actions:
                    print("无效的选择！")
                    continue
                    
                duration_str = input("请输入动作持续时间 (秒，例如 1.5): ").strip()
                duration = float(duration_str) if duration_str else 1.0
                
                mv_str = input("请输入动作推力 (0-5000，直接回车默认 2000): ").strip()
                mv = int(mv_str) if mv_str else 2000
                
                # 内部变量用于 lambda 传参
                self._temp_mv = mv
                
                action_name, action_func = actions[choice]
                
                if choice == "0":
                    self.wait(duration)
                else:
                    self.run_action(action_name, action_func, duration_s=duration, 
                                    ch1_mv=mv, ch2_mv=mv, ch3_mv=mv)
                                    
            except KeyboardInterrupt:
                break
            except ValueError:
                print("输入格式有误，请输入数字！")
            except Exception as e:
                print(f"执行异常: {e}")
                
        print("\n[Scheduler] 手动控制模式结束。")


def main():
    # 请确认 COM 端口是否正确
    scheduler = ActionScheduler(port="COM3")
    
    if not scheduler.connect():
        print("串口连接失败，进入离线测试模式 (指令仅打印不会生效)")

    try:
        choice = input("\n请选择模式 [1] 运行预设自动剧本 [2] 交互式手动控制: ").strip()
        
        if choice == "1":
            print("\n============= 挖掘机自动剧本开始 =============")
            # 1. 先发呆 1 秒
            scheduler.wait(1.0)
            
            # 2. 大臂抬起 2.5 秒，给出 5000mV 满量程推力
            scheduler.run_action("大臂抬起", scheduler.controller.boom_up, duration_s=2.5, 
                                 ch1_mv=5000, ch2_mv=5000, ch3_mv=5000)
                                 
            scheduler.wait(0.5)
            
            # 3. 底盘双侧前进 1.5 秒，推力设为 3000mV
            scheduler.run_action("双侧前进", 
                                 lambda: scheduler.controller.drive_forward(3000, 3000), 
                                 duration_s=1.5, 
                                 ch1_mv=3000, ch2_mv=3000, ch3_mv=3000)
                                 
            scheduler.wait(0.5)
            
            # 4. 机身向左转 1 秒，推力 2000mV
            scheduler.run_action("机身左转", 
                                 lambda: scheduler.controller.turn_left(2000, 2000), 
                                 duration_s=1.0, 
                                 ch1_mv=2000, ch2_mv=2000, ch3_mv=2000)
                                 
            scheduler.wait(0.5)
            
            # 5. 铲斗外推 1.2 秒，推力 4000mV
            scheduler.run_action("铲斗外推", scheduler.controller.bucket_out, duration_s=1.2, 
                                 ch1_mv=4000, ch2_mv=4000, ch3_mv=4000)
                                 
            print("\n============= 挖掘机自动剧本结束 =============")
        elif choice == "2":
            scheduler.interactive_mode()
        else:
            print("退出。")
        
    finally:
        # 最后无论如何都要停止所有动作并断开连接
        scheduler.controller.stop_all()
        scheduler.close()

if __name__ == "__main__":
    main()