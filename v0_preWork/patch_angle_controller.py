import re

with open("src/shandong/v4_control_closed/angle_controller.py", "r") as f:
    content = f.read()

# 1. 修改 bucket_arm 纾解逻辑
old_bucket = """                    if joint_name == "bucket_arm" and diff > 0: # 铲斗回拉遇阻
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
                        continue"""

new_bucket = """                    if joint_name == "bucket_arm" and diff > 0: # 铲斗回拉遇阻
                        self.log_msg(f"[智能补偿] {joint_name} 挖土遇阻卡死，自动前推小臂纾解...")
                        self._stop_joint_movement(joint_name)
                        
                        # 使用录制的 JSON 参数进行纾解，并加入柔性起步，避免突然暴走
                        r_start = time.time()
                        self.controller.arm_push()
                        while time.time() - r_start < 0.5:
                            t_el = time.time() - r_start
                            s = 1.0
                            if ramp_up_s > 0 and t_el < ramp_up_s:
                                tau = t_el / ramp_up_s
                                s = 3 * (tau ** 2) - 2 * (tau ** 3)
                            self.controller.set_analog(int(base_ch1 * s), int(base_ch2 * s), int(base_ch3 * s))
                            time.sleep(0.02)
                            
                        self.controller.stop_arm_swing()
                        
                        # 纾解完成后，重置起始时间和补偿，让原动作重新从 JSON 参数开始柔性起步
                        start_time = time.time()
                        current_boost_mv = 0
                        last_progress_time = time.time()
                        self.log_msg(f"[智能补偿完成] 恢复 {joint_name} 原有动作，重新柔性起步...")
                        continue"""

content = content.replace(old_bucket, new_bucket)

# 2. 修改 arm_boom 纾解逻辑
old_arm = """                    elif joint_name == "arm_boom" and diff > 0: # 小臂回拉遇阻
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
                        continue"""

new_arm = """                    elif joint_name == "arm_boom" and diff > 0: # 小臂回拉遇阻
                        self.log_msg(f"[智能补偿] {joint_name} 挖土遇阻卡死，自动抬起大臂 5 度纾解...")
                        self._stop_joint_movement(joint_name)
                        
                        boom_start = self._get_current_angle("boom_swing")
                        self.controller.boom_up()
                        r_start = time.time()
                        while time.time() - r_start < 2.5: # 最多给 2.5 秒时间
                            t_el = time.time() - r_start
                            s = 1.0
                            if ramp_up_s > 0 and t_el < ramp_up_s:
                                tau = t_el / ramp_up_s
                                s = 3 * (tau ** 2) - 2 * (tau ** 3)
                            self.controller.set_analog(int(base_ch1 * s), int(base_ch2 * s), int(base_ch3 * s))
                            
                            if abs(self._get_current_angle("boom_swing") - boom_start) >= 5.0:
                                break
                            time.sleep(0.02)
                            
                        self.controller.stop_boom_bucket()
                        
                        # 纾解完成后，重置起始时间和补偿，让原动作重新从 JSON 参数开始柔性起步
                        start_time = time.time()
                        current_boost_mv = 0
                        last_progress_time = time.time()
                        self.log_msg(f"[智能补偿完成] 恢复 {joint_name} 原有动作，重新柔性起步...")
                        continue"""

content = content.replace(old_arm, new_arm)

# 3. 修改 ramp_down 逻辑
old_ramp_down = """                        if ramp_down_s > 0:
                            self.log_msg(f"[柔性刹车] 保持动作，液压从 {current_scale*100:.1f}% 缓降至 0%，耗时: {ramp_down_s}s")
                            steps = max(1, int(ramp_down_s / 0.05))
                            dt = ramp_down_s / steps
                            for i in range(1, steps + 1):
                                tau = i / steps
                                # 三次样条平滑下降
                                s = current_scale * (1.0 - (3 * (tau ** 2) - 2 * (tau ** 3)))
                                self.controller.set_analog(int(base_ch1 * s), int(base_ch2 * s), int(base_ch3 * s))
                                time.sleep(dt)"""

new_ramp_down = """                        if ramp_down_s > 0:
                            self.log_msg(f"[柔性刹车] 保持动作，液压从 {current_scale*100:.1f}% 缓降至 0%，耗时: {ramp_down_s}s")
                            
                            # 重新计算当前应有的 dynamic_ch，防止瞬间掉压
                            d_ch1, d_ch2, d_ch3 = base_ch1, base_ch2, base_ch3
                            if current_boost_mv > 0:
                                d_ch1 = min(5000, d_ch1 + current_boost_mv)
                                d_ch2 = min(5000, d_ch2 + current_boost_mv)
                                d_ch3 = min(5000, d_ch3 + current_boost_mv)
                            if joint_name == "boom_swing" and diff < 0:
                                if base_ch3 < 3500 and (elapsed > ramp_up_s or current_boost_mv > 0):
                                    comp_factor = max(0.0, min(1.0, current_angle / 45.0))
                                    boost = int(comp_factor * 1500)
                                    d_ch1 = min(5000, d_ch1 + boost)
                                    d_ch2 = min(5000, d_ch2 + boost)
                                    d_ch3 = min(5000, d_ch3 + boost)

                            steps = max(1, int(ramp_down_s / 0.05))
                            dt = ramp_down_s / steps
                            for i in range(1, steps + 1):
                                tau = i / steps
                                # 三次样条平滑下降
                                s = current_scale * (1.0 - (3 * (tau ** 2) - 2 * (tau ** 3)))
                                self.controller.set_analog(int(d_ch1 * s), int(d_ch2 * s), int(d_ch3 * s))
                                time.sleep(dt)"""

content = content.replace(old_ramp_down, new_ramp_down)

with open("src/shandong/v4_control_closed/angle_controller.py", "w") as f:
    f.write(content)

print("Patch applied successfully.")
