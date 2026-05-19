import time
import threading
import datetime
import os

class AngleController:
    """
    闭环角度控制器 (v4)
    该类使用 v1 的底层控制器进行物理动作，同时接受外部传入的传感器实时角度数据。
    它将基于您测量的极限状态运动量程，进行闭环控制（达到目标角度后自动停止），替代 v2 的“时间控制”或“直接运动”。
    """
    def __init__(self, controller):
        # 传入 v1 中的 ExcavatorController
        self.controller = controller
        
        # --- 初始化日志系统 ---
        log_dir = os.path.join(os.path.dirname(__file__), "logs")
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        self.log_file = os.path.join(log_dir, f"excavator_v4_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
        self._log_lock = threading.Lock()
        self.log_msg("=== 挖掘机 V4 闭环控制器初始化 ===")
        
        # -------------------------------------------------------------
        # 【关节极限状态运动量程】 
        # 当目标角度或当前实时角度超过这个范围时，会进行截断或紧急停止保护
        # -------------------------------------------------------------
        self.joint_limits = {
            "boom_swing": {"min_angle": -5.0, "max_angle": 55.0},   # 大臂与回转 (大臂)
            "arm_boom":   {"min_angle": -5.0, "max_angle": 95.0},   # 小臂与大臂 (小臂)
            "bucket_arm": {"min_angle": -95.0, "max_angle": 20.0},  # 铲斗与小臂 (铲斗)
            "swing_yaw":  {"min_angle": -360.0, "max_angle": 360.0} # 回转偏航角 (不受限)
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

    def log_msg(self, msg: str, also_print: bool = True):
        """线程安全的日志记录方法"""
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        formatted_msg = f"[{timestamp}] {msg}"
        if also_print:
            print(formatted_msg)
        with self._log_lock:
            with open(self.log_file, "a", encoding="utf-8") as f:
                f.write(formatted_msg + "\n")

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

    def move_joint_to_angle(self, joint_name, target_angle, tolerance=2.0, ch1_mv=2000, ch2_mv=2000, ch3_mv=2000, ramp_up_s=0.0, ramp_down_s=0.0, is_init_step=False):
        """
        核心闭环控制方法。
        参数:
            joint_name: 关节名称 (例如 "bucket_arm")
            target_angle: 目标角度 (若是回转 swing_yaw，则此值代表旋转秒数，正数右转，负数左转)
            tolerance: 容差 (当与目标角度误差小于这个值时，认为到达并停止)
            ramp_up_s: 柔性启动加速时间(秒)
            ramp_down_s: 柔性停止减速使能(秒，闭环中作为标识，开环中作为时间)
            is_init_step: 是否为初始归位步，若是则允许超时（6秒）后强行视为到达目标
        """
        # 如果是回转，走时间开环控制逻辑
        if joint_name == "swing_yaw":
            # target_angle 在回转中代表秒数
            self.move_swing_by_time(target_angle, ch1_mv, ch2_mv, ch3_mv, ramp_up_s, ramp_down_s)
            return

        # 1. 量程保护检查
        limits = self.joint_limits.get(joint_name)
        if limits:
            if target_angle < limits["min_angle"]:
                self.log_msg(f"[{joint_name}] 目标角度 {target_angle} 小于最小极限 {limits['min_angle']}，自动截断。")
                target_angle = limits["min_angle"]
            elif target_angle > limits["max_angle"]:
                self.log_msg(f"[{joint_name}] 目标角度 {target_angle} 大于最大极限 {limits['max_angle']}，自动截断。")
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
            args=(joint_name, target_angle, tolerance, ch1_mv, ch2_mv, ch3_mv, ramp_up_s, ramp_down_s, is_init_step),
            daemon=True
        ).start()

    def move_swing_by_time(self, duration_s, ch1_mv, ch2_mv, ch3_mv, ramp_up_s=0.0, ramp_down_s=0.0):
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
            args=(duration_s, ch1_mv, ch2_mv, ch3_mv, ramp_up_s, ramp_down_s),
            daemon=True
        ).start()

    def _swing_time_loop(self, duration_s, ch1_mv, ch2_mv, ch3_mv, ramp_up_s, ramp_down_s):
        direction_str = "右转" if duration_s > 0 else "左转"
        actual_duration = abs(duration_s)
        self.log_msg(f"[开环控制开始] 回转 {direction_str} {actual_duration:.1f} 秒 | 基础推力: {ch1_mv}, {ch2_mv}, {ch3_mv}")
        
        # 容错：如果加减速时间超过了总时间，按比例缩放
        if ramp_up_s + ramp_down_s > actual_duration:
            scale = actual_duration / (ramp_up_s + ramp_down_s)
            ramp_up_s *= scale
            ramp_down_s *= scale
            
        try:
            start_time = time.time()
            while self._running_tasks.get("swing_yaw"):
                elapsed = time.time() - start_time
                if elapsed >= actual_duration:
                    self.log_msg(f"[开环控制完成] 回转动作结束 ({direction_str} {actual_duration:.1f} 秒)")
                    break
                    
                # -------------------------
                # 柔性控制 (基于时间)
                # -------------------------
                scale = 1.0
                if ramp_up_s > 0 and elapsed < ramp_up_s:
                    tau = elapsed / ramp_up_s
                    scale = 3 * (tau ** 2) - 2 * (tau ** 3)
                elif ramp_down_s > 0 and elapsed > (actual_duration - ramp_down_s):
                    tau = (actual_duration - elapsed) / ramp_down_s
                    s = 3 * (tau ** 2) - 2 * (tau ** 3)
                    scale = 0.2 + 0.8 * s # 保底 20% 推力
                    
                current_ch1 = int(ch1_mv * scale)
                current_ch2 = int(ch2_mv * scale)
                current_ch3 = int(ch3_mv * scale)
                self.controller.set_analog(current_ch1, current_ch2, current_ch3)
                
                # 偶尔记录一下状态 (每 ~0.5秒记录一次)
                if int(elapsed * 50) % 25 == 0:
                    self.log_msg(f"[状态] 回转 {direction_str} | 已运行: {elapsed:.2f}s / {actual_duration}s | 模拟量输出: {current_ch1}, {current_ch2}, {current_ch3} | 柔性系数: {scale:.2f}", also_print=False)
                
                # 持续下发继电器动作，防止动作断触（与角度闭环逻辑保持一致）
                if duration_s > 0:
                    self._start_joint_movement("swing_yaw", "increase") # 映射为右转
                else:
                    self._start_joint_movement("swing_yaw", "decrease") # 映射为左转
                    
                time.sleep(0.02)
                
        finally:
            with self._lock:
                self._running_tasks["swing_yaw"] = False
            self._stop_joint_movement("swing_yaw")

    def _angle_control_loop(self, joint_name, target_angle, tolerance, ch1_mv, ch2_mv, ch3_mv, ramp_up_s, ramp_down_s, is_init_step=False):
        """实际执行闭环逻辑的后台循环"""
        self.log_msg(f"\n==========================================")
        self.log_msg(f"[闭环控制开始] {joint_name} 目标: {target_angle}° | 基础推力: {ch1_mv}, {ch2_mv}, {ch3_mv} | 初始步骤: {is_init_step}")
        
        base_ch1, base_ch2, base_ch3 = ch1_mv, ch2_mv, ch3_mv
        
        start_time = time.time()
        exit_confirm_count = 0  # 连续确认到达的次数，防止传感器抖动导致的误判
        
        # 卡顿遇阻检测变量
        last_progress_time = time.time()
        last_progress_angle = self._get_current_angle(joint_name)
        
        # 自身遇阻液压递增补偿变量
        current_boost_mv = 0
        
        current_scale = 1.0 # 记录当前的柔性比例，用于退出时缓降
        
        try:
            while self._running_tasks.get(joint_name):
                current_angle = self._get_current_angle(joint_name)
                diff = target_angle - current_angle
                elapsed = time.time() - start_time
                
                # 偶尔记录一下闭环的实时状态 (每 ~0.5秒记录一次)
                if int(elapsed * 50) % 25 == 0:
                    self.log_msg(f"[状态] {joint_name} | 当前角度: {current_angle:.1f}° | 距离目标: {diff:.1f}° | 已耗时: {elapsed:.2f}s", also_print=False)
                
                # -------------------------
                # 针对初始状态归位的特殊超时保护 (6秒强行跳出)
                # -------------------------
                if is_init_step and elapsed > 6.0:
                    self.log_msg(f"[初始步骤超时] {joint_name} 已执行超过 6 秒，强行判定为到达初始位置！(当前角度: {current_angle:.1f}°)")
                    # 执行软停机逻辑
                    if ramp_down_s > 0:
                        self.log_msg(f"[柔性刹车] 保持动作，液压从 {current_scale*100:.1f}% 缓降至 0%，耗时: {ramp_down_s}s")
                        steps = max(1, int(ramp_down_s / 0.05))
                        dt = ramp_down_s / steps
                        for i in range(1, steps + 1):
                            tau = i / steps
                            s = current_scale * (1.0 - (3 * (tau ** 2) - 2 * (tau ** 3)))
                            self.controller.set_analog(int(base_ch1 * s), int(base_ch2 * s), int(base_ch3 * s))
                            time.sleep(dt)
                    self._stop_joint_movement(joint_name)
                    if hasattr(self, "_initial_diff"):
                        self._initial_diff.pop(joint_name, None)
                    break
                
                # -------------------------
                # 智能遇阻补偿 (卡死自动纾解与液压递增)
                # -------------------------
                if abs(current_angle - last_progress_angle) > 0.5:
                    last_progress_angle = current_angle
                    last_progress_time = time.time()
                    # 一旦恢复正常运动，将递增补偿清零
                    if current_boost_mv > 0:
                        self.log_msg(f"[智能补偿恢复] {joint_name} 恢复正常运动，取消自身液压递增补偿。")
                        current_boost_mv = 0
                elif time.time() - last_progress_time > 1.5:
                    # 超过 1.5 秒角度变化不到 0.5 度，判定为卡死或推力不足
                    self.log_msg(f"[警告] 检测到 {joint_name} 卡死！(当前角度: {current_angle:.1f}°, 距目标: {diff:.1f}°) 停滞超过 1.5 秒。")
                    
                    # 1. 首先尝试自身液压递增（每次增加 200mV，最高不超过 4500mV）
                    if base_ch3 + current_boost_mv < 4500:
                        current_boost_mv += 200
                        # 确保加上补偿后不会超过 4500
                        if base_ch3 + current_boost_mv > 4500:
                            current_boost_mv = 4500 - base_ch3
                        self.log_msg(f"[智能补偿] {joint_name} 推力不足，自身液压增加 200mV (当前总液压: {base_ch3 + current_boost_mv}mV)")
                        # 重置进度检测时间，给递增后的液压 1.5 秒的时间看是否能推动
                        last_progress_time = time.time()
                        continue
                        
                    # 2. 如果自身液压已经递增到了最大（4500mV）还是推不动，则触发其他关节协助纾解
                    if joint_name == "bucket_arm" and diff > 0: # 铲斗回拉遇阻
                        self.log_msg(f"[智能补偿] {joint_name} 挖土遇阻卡死，自动前推小臂纾解...")
                        self._stop_joint_movement(joint_name)
                        
                        # 同样使用基础推力+最低保障，避免起步过猛
                        relief_mv = max(3500, base_ch3)
                        self.controller.set_analog(relief_mv, relief_mv, relief_mv)
                        
                        self.controller.arm_push()
                        time.sleep(0.5) # 抬起小臂 0.5 秒
                        self.controller.stop_arm_swing()
                        last_progress_time = time.time()
                        self.log_msg(f"[智能补偿完成] 恢复 {joint_name} 原有动作...")
                        continue
                        
                    elif joint_name == "arm_boom" and diff > 0: # 小臂回拉遇阻
                        self.log_msg(f"[智能补偿] {joint_name} 挖土遇阻卡死，自动抬起大臂 5 度纾解...")
                        self._stop_joint_movement(joint_name)
                        
                        # 使用当前设定的基础推力进行纾解，而不是直接给满 4500
                        # 但为了保证能抬起来，给予一定的最低保障推力 (3500)
                        relief_mv = max(3500, base_ch3)
                        self.controller.set_analog(relief_mv, relief_mv, relief_mv)
                        
                        boom_start = self._get_current_angle("boom_swing")
                        self.controller.boom_up()
                        r_start = time.time()
                        while time.time() - r_start < 2.5: # 最多给 2.5 秒时间
                            if abs(self._get_current_angle("boom_swing") - boom_start) >= 5.0:
                                break
                            time.sleep(0.05)
                        self.controller.stop_boom_bucket()
                        last_progress_time = time.time()
                        self.log_msg(f"[智能补偿完成] 恢复 {joint_name} 原有动作...")
                        continue
                
                # -------------------------
                # 记录初始的差值方向，用于判断是否越过目标
                # -------------------------
                if not hasattr(self, "_initial_diff"):
                    self._initial_diff = {}
                if joint_name not in self._initial_diff:
                    self._initial_diff[joint_name] = diff
                
                # -------------------------
                # 安全保护：如果实时角度超出设定的硬极限范围，直接急停并报警
                # -------------------------
                limits = self.joint_limits.get(joint_name)
                if limits:
                    # 留出 1 度的缓冲以防抖动误触发
                    if current_angle < limits["min_angle"] - 1.0:
                        err_msg = f"[严重警告] {joint_name} 实时角度 {current_angle:.1f}° 已低于最小安全极限 {limits['min_angle']}°！强制急停并终止程序！"
                        self.log_msg(err_msg)
                        self.stop_all()
                        os._exit(1) # 直接杀掉整个进程，确保绝对安全
                    elif current_angle > limits["max_angle"] + 1.0:
                        err_msg = f"[严重警告] {joint_name} 实时角度 {current_angle:.1f}° 已超出最大安全极限 {limits['max_angle']}°！强制急停并终止程序！"
                        self.log_msg(err_msg)
                        self.stop_all()
                        os._exit(1) # 直接杀掉整个进程，确保绝对安全
                
                # -------------------------
                # 动态设置大臂的提前量和补偿
                # -------------------------
                current_adv_comp = 2.0
                if joint_name == "boom_swing":
                    if self._initial_diff[joint_name] > 0: # 初始是下降
                        current_adv_comp = 4.0 # 下降惯性大，提前更多刹车
                    else:
                        current_adv_comp = 1.0 # 抬起阻力大，提前量小
                
                # 到达目标 (加入提前量补偿)
                # 【防冲顶逻辑】如果当前差值的符号与初始差值的符号相反，说明已经越过了目标点
                is_crossed = (self._initial_diff[joint_name] * diff) < 0
                
                if abs(diff) <= (tolerance + current_adv_comp) or is_crossed:
                    exit_confirm_count += 1
                    # 连续3次(60ms)确认，或者在距离目标10度以内发生了越过，才真正停止（过滤传感器噪声突变）
                    if exit_confirm_count >= 3 or (is_crossed and abs(diff) < 10.0):
                        self.log_msg(f"[闭环控制完成] {joint_name} 抵达目标区域 (当前角度: {current_angle:.1f}° 目标: {target_angle}°)")
                        
                        # ========================================
                        # V2 风格的柔性软停机 (彻底消除继电器急断导致的顿挫)
                        # ========================================
                        if ramp_down_s > 0:
                            self.log_msg(f"[柔性刹车] 保持动作，液压从 {current_scale*100:.1f}% 缓降至 0%，耗时: {ramp_down_s}s")
                            steps = max(1, int(ramp_down_s / 0.05))
                            dt = ramp_down_s / steps
                            for i in range(1, steps + 1):
                                tau = i / steps
                                # 三次样条平滑下降
                                s = current_scale * (1.0 - (3 * (tau ** 2) - 2 * (tau ** 3)))
                                self.controller.set_analog(int(base_ch1 * s), int(base_ch2 * s), int(base_ch3 * s))
                                time.sleep(dt)
                                
                        self._stop_joint_movement(joint_name)
                        # 任务完成，清除初始差值记录
                        self._initial_diff.pop(joint_name, None)
                        break
                else:
                    exit_confirm_count = 0
                    
                # -------------------------
                # 柔性控制逻辑 (修改液压流量)
                # -------------------------
                elapsed = time.time() - start_time
                scale = 1.0
                
                # 判断当前是否处于初始步骤的 6 秒强制退出末期 (留出 ramp_down_s 的时间来执行泄压)
                force_timeout_stopping = False
                if is_init_step and elapsed >= (6.0 - ramp_down_s):
                    force_timeout_stopping = True
                
                # 1. 柔性加速 (基于时间)
                if ramp_up_s > 0 and elapsed < ramp_up_s:
                    tau = elapsed / ramp_up_s
                    scale = 3 * (tau ** 2) - 2 * (tau ** 3)
                
                elif force_timeout_stopping:
                    # 超时强行停机阶段：按时间减速
                    # 此时已经超过 (6.0 - ramp_down_s)，需要在剩下的 ramp_down_s 内把推力降到 0
                    time_left = 6.0 - elapsed
                    if time_left > 0:
                        tau = time_left / ramp_down_s
                        scale = 3 * (tau ** 2) - 2 * (tau ** 3)
                    else:
                        scale = 0.0
                        
                # 2. 柔性减速 (基于剩余角度)
                # 只有当没有被卡死（没有推力补偿）且距离目标很近时，才执行基于角度的柔性减速
                elif ramp_down_s > 0 and current_boost_mv == 0:
                    ramp_down_threshold = 30.0 if (joint_name == "boom_swing" and diff > 0) else 15.0
                    min_scale = 0.05 if (joint_name == "boom_swing" and diff > 0) else 0.2
                    
                    if abs(diff) < ramp_down_threshold:
                        tau = abs(diff) / ramp_down_threshold
                        s = 3 * (tau ** 2) - 2 * (tau ** 3)
                        scale = min_scale + (1.0 - min_scale) * s
                        
                # -------------------------
                # 3. 动态流量补偿 (针对大臂抬起)
                # -------------------------
                dynamic_ch1, dynamic_ch2, dynamic_ch3 = base_ch1, base_ch2, base_ch3
                
                # 将自身液压递增补偿加上
                if current_boost_mv > 0:
                    dynamic_ch1 = min(5000, dynamic_ch1 + current_boost_mv)
                    dynamic_ch2 = min(5000, dynamic_ch2 + current_boost_mv)
                    dynamic_ch3 = min(5000, dynamic_ch3 + current_boost_mv)

                if joint_name == "boom_swing" and diff < 0: # 抬起动作
                    # 如果记录的基础推力（如示教时）已经足够大（>3500），或者刚刚启动还在加速期（防止起步过猛），
                    # 则削弱或取消额外的动态重力补偿。
                    # 注意：如果触发了 current_boost_mv，说明已经被卡死了，此时即使在加速期内也可以给力。
                    if base_ch3 < 3500 and (elapsed > ramp_up_s or current_boost_mv > 0):
                        # 角度越大（大臂越低），需要克服重力的流量越大。
                        # 以 0~45 度作为参考，最大补偿 +1500mV
                        comp_factor = max(0.0, min(1.0, current_angle / 45.0))
                        boost = int(comp_factor * 1500)
                        dynamic_ch1 = min(5000, base_ch1 + boost)
                        dynamic_ch2 = min(5000, base_ch2 + boost)
                        dynamic_ch3 = min(5000, base_ch3 + boost)
                        
                self.controller.set_analog(int(dynamic_ch1 * scale), int(dynamic_ch2 * scale), int(dynamic_ch3 * scale))
                
                current_scale = scale # 保存给退出时的缓降使用
                
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