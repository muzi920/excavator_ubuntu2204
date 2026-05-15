import time
import threading

class AngleController:
    """
    闭环角度控制器 (v4)
    该类使用 v1 的底层控制器进行物理动作，同时接受外部传入的传感器实时角度数据。
    它将基于您测量的极限状态运动量程，进行闭环控制（达到目标角度后自动停止），替代 v2 的“时间控制”或“直接运动”。
    """
    def __init__(self, controller):
        # 传入 v1 中的 ExcavatorController
        self.controller = controller
        
        # -------------------------------------------------------------
        # 【预留接口】 运动量程 / 极限范围配置 (等您测试后在这里填入实际差值)
        # 格式示例： {"min_angle": 完全收缩时的夹角, "max_angle": 完全伸展时的夹角}
        # -------------------------------------------------------------
        self.joint_limits = {
            "bucket_arm": {"min_angle": -180.0, "max_angle": 180.0},  # 铲斗与小臂
            "arm_boom":   {"min_angle": -10.0, "max_angle": 110.0},   # 小臂与大臂
            "boom_swing": {"min_angle": -180.0, "max_angle": 180.0},  # 大臂与回转
            "swing_yaw":  {"min_angle": -360.0, "max_angle": 360.0}   # 回转偏航角
        }
        
        # 内部状态：记录当前是否有闭环任务在运行
        self._running_tasks = {}
        self._lock = threading.Lock()
        
        # 实时传感器数据引用（外部需不断更新这个字典）
        self.current_sensor_data = {
            "大臂": {"pitch": 0.0, "yaw": 0.0},
            "小臂": {"pitch": 0.0, "yaw": 0.0},
            "铲斗": {"pitch": 0.0, "yaw": 0.0},
            "回转": {"pitch": 0.0, "yaw": 0.0}
        }

    def update_sensor_data(self, sensor_data):
        """外部不断调用此方法，更新最新的传感器数据"""
        with self._lock:
            self.current_sensor_data = sensor_data

    def _get_current_angle(self, joint_name):
        """根据当前的传感器数据计算指定的关节夹角"""
        with self._lock:
            d = self.current_sensor_data
            if joint_name == "bucket_arm":
                return d['铲斗']['pitch'] - d['小臂']['pitch']
            elif joint_name == "arm_boom":
                return d['小臂']['pitch'] - d['大臂']['pitch']
            elif joint_name == "boom_swing":
                return d['大臂']['pitch'] - d['回转']['pitch']
            elif joint_name == "swing_yaw":
                return d['回转']['yaw']
        return 0.0

    def stop_all(self):
        """停止所有闭环任务和物理动作"""
        with self._lock:
            for task_name in list(self._running_tasks.keys()):
                self._running_tasks[task_name] = False
        self.controller.stop_all()

    def move_joint_to_angle(self, joint_name, target_angle, tolerance=2.0, ch1_mv=2000, ch2_mv=2000, ch3_mv=2000):
        """
        核心闭环控制方法。
        参数:
            joint_name: 关节名称 (例如 "bucket_arm")
            target_angle: 目标角度 (若是回转 swing_yaw，则此值代表旋转秒数，正数右转，负数左转)
            tolerance: 容差 (当与目标角度误差小于这个值时，认为到达并停止)
        """
        # 如果是回转，走时间开环控制逻辑
        if joint_name == "swing_yaw":
            # target_angle 在回转中代表秒数
            self.move_swing_by_time(target_angle, ch1_mv, ch2_mv, ch3_mv)
            return

        # 1. 量程保护检查
        limits = self.joint_limits.get(joint_name)
        if limits:
            if target_angle < limits["min_angle"]:
                print(f"[{joint_name}] 目标角度 {target_angle} 小于最小极限 {limits['min_angle']}，自动截断。")
                target_angle = limits["min_angle"]
            elif target_angle > limits["max_angle"]:
                print(f"[{joint_name}] 目标角度 {target_angle} 大于最大极限 {limits['max_angle']}，自动截断。")
                target_angle = limits["max_angle"]

        # 2. 如果当前有相同关节的任务在运行，先停止它
        with self._lock:
            if self._running_tasks.get(joint_name):
                self._running_tasks[joint_name] = False
                time.sleep(0.1) # 稍等之前的线程退出
            self._running_tasks[joint_name] = True

        # 3. 启动后台线程执行闭环控制
        # 启动前清理旧的初始差值记录
        if hasattr(self, "_initial_diff") and joint_name in self._initial_diff:
            self._initial_diff.pop(joint_name)
            
        threading.Thread(
            target=self._angle_control_loop,
            args=(joint_name, target_angle, tolerance, ch1_mv, ch2_mv, ch3_mv),
            daemon=True
        ).start()

    def move_swing_by_time(self, duration_s, ch1_mv, ch2_mv, ch3_mv):
        """
        基于时间的开环回转控制
        duration_s: 旋转时间，正数代表右转，负数代表左转
        """
        with self._lock:
            if self._running_tasks.get("swing_yaw"):
                self._running_tasks["swing_yaw"] = False
                time.sleep(0.1)
            self._running_tasks["swing_yaw"] = True
            
        threading.Thread(
            target=self._swing_time_loop,
            args=(duration_s, ch1_mv, ch2_mv, ch3_mv),
            daemon=True
        ).start()

    def _swing_time_loop(self, duration_s, ch1_mv, ch2_mv, ch3_mv):
        direction_str = "右转" if duration_s > 0 else "左转"
        actual_duration = abs(duration_s)
        print(f"[开环控制开始] 回转 {direction_str} {actual_duration:.1f} 秒")
        
        self.controller.set_analog(ch1_mv, ch2_mv, ch3_mv)
        
        try:
            if duration_s > 0:
                self._start_joint_movement("swing_yaw", "increase") # 映射为右转
            else:
                self._start_joint_movement("swing_yaw", "decrease") # 映射为左转
                
            # 倒计时等待
            start_time = time.time()
            while self._running_tasks.get("swing_yaw"):
                if time.time() - start_time >= actual_duration:
                    print(f"[开环控制完成] 回转动作结束 ({direction_str} {actual_duration:.1f} 秒)")
                    break
                time.sleep(0.05)
                
        finally:
            with self._lock:
                self._running_tasks["swing_yaw"] = False
            self._stop_joint_movement("swing_yaw")

    def _angle_control_loop(self, joint_name, target_angle, tolerance, ch1_mv, ch2_mv, ch3_mv):
        """实际执行闭环逻辑的后台循环"""
        print(f"[闭环控制开始] {joint_name} 目标: {target_angle}°")
        
        # 每次控制前设置推力
        self.controller.set_analog(ch1_mv, ch2_mv, ch3_mv)
        
        # 提前量补偿 (根据液压惯性设置提前停止的度数)
        # 例如设置为 2.0，意味着在距离目标还差 2 度的时候就发停止指令，靠惯性滑到目标
        advance_compensation = 2.0
        
        try:
            while self._running_tasks.get(joint_name):
                current_angle = self._get_current_angle(joint_name)
                diff = target_angle - current_angle
                
                # -------------------------
                # 记录初始的差值方向，用于判断是否越过目标
                # -------------------------
                if not hasattr(self, "_initial_diff"):
                    self._initial_diff = {}
                if joint_name not in self._initial_diff:
                    self._initial_diff[joint_name] = diff
                
                # 到达目标 (加入提前量补偿)
                # 只要距离目标的绝对值小于 容差 + 提前量，就立刻触发停止
                # 【防冲顶逻辑】如果当前差值的符号与初始差值的符号相反，说明已经越过了目标点！必须立刻停止
                is_crossed = (self._initial_diff[joint_name] * diff) < 0
                
                if abs(diff) <= (tolerance + advance_compensation) or is_crossed:
                    print(f"[闭环控制完成] {joint_name} 发送停止指令时当前角度: {current_angle:.1f}° (目标: {target_angle}°)")
                    self._stop_joint_movement(joint_name)
                    # 任务完成，清除初始差值记录
                    self._initial_diff.pop(joint_name, None)
                    break
                    
                # 根据差值的正负决定运动方向
                if diff > 0:
                    self._start_joint_movement(joint_name, direction="increase")
                else:
                    self._start_joint_movement(joint_name, direction="decrease")
                    
                time.sleep(0.02) # 将检查频率从 50ms 提高到 20ms，反应更迅速
                
        finally:
            with self._lock:
                self._running_tasks[joint_name] = False
            self._stop_joint_movement(joint_name)

    def _start_joint_movement(self, joint_name, direction):
        """调用 v1 控制器让关节开始运动"""
        if joint_name == "bucket_arm":
            # 铲斗的运动方向与传感器读数相反，这里将 out 和 in 对调
            if direction == "increase": self.controller.bucket_in()
            else:                       self.controller.bucket_out()
        elif joint_name == "arm_boom":
            # 小臂的运动方向与传感器读数相反，这里将 push 和 pull 对调
            if direction == "increase": self.controller.arm_pull()
            else:                       self.controller.arm_push()
        elif joint_name == "boom_swing":
            # 大臂的运动方向与传感器读数相反，这里将 up 和 down 对调
            if direction == "increase": self.controller.boom_down()
            else:                       self.controller.boom_up()
        elif joint_name == "swing_yaw":
            if direction == "increase": self.controller.swing_right()
            else:                       self.controller.swing_left()

    def _stop_joint_movement(self, joint_name):
        """调用 v1 控制器让对应关节停止运动"""
        if joint_name == "bucket_arm":
            self.controller.stop_boom_bucket()
        elif joint_name == "arm_boom":
            self.controller.stop_arm_swing()
        elif joint_name == "boom_swing":
            self.controller.stop_boom_bucket()
        elif joint_name == "swing_yaw":
            self.controller.stop_arm_swing()