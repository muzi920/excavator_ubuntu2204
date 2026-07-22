import datetime
import os
import threading


class SimAngleController:
    """基于 v4 关节语义的 URDF 仿真控制器。"""

    def __init__(self, ros_bridge):
        self.ros_bridge = ros_bridge
        self._lock = threading.Lock()
        self.current_sensor_data = {
            "大臂": {"pitch": 0.0, "yaw": 0.0},
            "小臂": {"pitch": 0.0, "yaw": 0.0},
            "铲斗": {"pitch": 0.0, "yaw": 0.0},
            "回转": {"pitch": 0.0, "yaw": 0.0},
        }
        self.joint_limits = {
            "boom_swing": {"min_angle": -5.0, "max_angle": 55.0},
            "arm_boom": {"min_angle": -5.0, "max_angle": 95.0},
            "bucket_arm": {"min_angle": -95.0, "max_angle": 20.0},
            "swing_yaw": {"min_angle": -180.0, "max_angle": 180.0},
        }

        log_dir = os.path.join(os.path.dirname(__file__), "logs")
        os.makedirs(log_dir, exist_ok=True)
        self.log_file = os.path.join(
            log_dir,
            f"v14_urdf_sim_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.log",
        )
        self._log_lock = threading.Lock()
        self.log_msg("=== v14_urdf 仿真控制器初始化 ===")

    def log_msg(self, msg, also_print=True):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        formatted = f"[{timestamp}] {msg}"
        if also_print:
            print(formatted)
        with self._log_lock:
            with open(self.log_file, "a", encoding="utf-8") as f:
                f.write(formatted + "\n")

    def update_sensor_data(self, sensor_data):
        with self._lock:
            self.current_sensor_data = sensor_data

    def stop_all(self):
        self.log_msg("[SIM] stop_all() 调用，仿真模式不下发硬件急停。")

    def move_joint_to_angle(
        self,
        joint_name,
        target_angle,
        tolerance=2.0,
        ch1_mv=2000,
        ch2_mv=2000,
        ch3_mv=2000,
        ramp_up_s=0.0,
        ramp_down_s=0.0,
        is_init_step=False,
    ):
        del tolerance, ch1_mv, ch2_mv, ch3_mv, ramp_up_s, ramp_down_s, is_init_step

        limits = self.joint_limits.get(joint_name)
        if limits:
            if target_angle < limits["min_angle"]:
                target_angle = limits["min_angle"]
            elif target_angle > limits["max_angle"]:
                target_angle = limits["max_angle"]

        payload = {}
        if joint_name == "bucket_arm":
            payload["bucket_arm"] = target_angle
        elif joint_name == "arm_boom":
            payload["arm_boom"] = target_angle
        elif joint_name == "boom_swing":
            payload["boom_swing"] = target_angle
        elif joint_name == "swing_yaw":
            payload["swing_yaw"] = target_angle
        else:
            self.log_msg(f"[SIM] 未知关节 {joint_name}，忽略。")
            return

        self.ros_bridge.publish_v4_targets_deg(**payload)
        self.log_msg(f"[SIM] 已发布 {joint_name} 目标角度到 /joint_states: {target_angle}")
