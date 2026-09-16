#!/usr/bin/env python3
import copy
import json
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, String


def find_repo_path(relative_path):
    here = os.path.abspath(os.path.dirname(__file__))
    probe = here
    while True:
        candidate = os.path.join(probe, relative_path)
        if os.path.exists(candidate):
            return candidate
        parent = os.path.dirname(probe)
        if parent == probe:
            break
        probe = parent
    raise FileNotFoundError(f"Cannot locate repository path: {relative_path}")


V1_CONTROL_BASE = find_repo_path(os.path.join("src", "shandong", "v1_control_base"))
V3_SENSOR_WIT = find_repo_path(
    os.path.join(
        "src",
        "shandong",
        "v3_sensor_read_wit",
        "WitStandardModbus_WT901C485-main",
        "Python",
        "Python-SDK-WT901C485_new",
    )
)
V4_CONTROL_CLOSED = find_repo_path(os.path.join("src", "shandong", "v4_control_closed"))
V5_SENSOR_LIDAR = find_repo_path(os.path.join("src", "shandong", "v5_sensor_read_lidar"))
sys.path.append(V1_CONTROL_BASE)
sys.path.append(V3_SENSOR_WIT)
sys.path.append(V4_CONTROL_CLOSED)
sys.path.append(V5_SENSOR_LIDAR)

from zs_excavator_controller import build_controller  # noqa: E402
import device_model  # noqa: E402
from angle_controller import AngleController  # noqa: E402
from imu_direct_swing_estimator import DirectSwingAngleEstimator  # noqa: E402


JOINT_NAME_MAP = {
    "boom_joint": "boom_swing",
    "arm_joint": "arm_boom",
    "bucket_joint": "bucket_arm",
    "swing_joint": "swing_yaw",
    "boom_swing": "boom_swing",
    "arm_boom": "arm_boom",
    "bucket_arm": "bucket_arm",
    "swing_yaw": "swing_yaw",
    "swing_time": "swing_time",
}


class ControlExecutorNode(Node):
    def __init__(self):
        super().__init__("control_executor_node")

        self.declare_parameter("controller_port", "/dev/ttyUSB_Controller")
        self.declare_parameter("controller_baud", 115200)
        self.declare_parameter(
            "sensor_ports",
            ["/dev/ttyUSB_Sensor1", "/dev/ttyUSB_Sensor2", "/dev/ttyUSB_Sensor3", "/dev/ttyUSB_Sensor4"],
        )
        self.declare_parameter("sensor_baud", 230400)
        self.declare_parameter("sensor_publish_hz", 20.0)

        self.sensor_lock = threading.Lock()
        self.sensor_data = {
            "大臂": {"pitch": 0.0, "yaw": 0.0},
            "小臂": {"pitch": 0.0, "yaw": 0.0},
            "铲斗": {"pitch": 0.0, "yaw": 0.0},
            "回转": {"pitch": 0.0, "yaw": 0.0},
        }
        self.sensor_ts = {
            "大臂": 0.0,
            "小臂": 0.0,
            "铲斗": 0.0,
            "回转": 0.0,
        }
        self.devices = []
        self.seen_addr = set()
        self.addr_to_name = {0x50: "铲斗", 0x51: "小臂", 0x52: "大臂", 0x53: "回转"}
        self.swing_estimator = DirectSwingAngleEstimator()

        self.pub_pitch_ = self.create_publisher(Float64MultiArray, "/excavator/inclinometer_pitch_deg", 10)
        self.pub_relative_ = self.create_publisher(Float64MultiArray, "/excavator/inclinometer_relative_deg", 10)

        port = self.get_parameter("controller_port").get_parameter_value().string_value
        baud = self.get_parameter("controller_baud").get_parameter_value().integer_value
        self.base_controller = None
        self.angle_ctrl = None

        try:
            self.base_controller = build_controller(port=port, baudrate=baud)
            if self.base_controller.connect():
                self.get_logger().info(f"Closed-loop controller connected on {port} @ {baud}")
            else:
                self.get_logger().warning(f"Cannot open controller port {port}, node runs in offline mode")
                self.base_controller = None
        except Exception as exc:
            self.get_logger().error(f"Controller init failed: {exc}")
            self.base_controller = None

        if self.base_controller is not None:
            self.angle_ctrl = AngleController(self.base_controller)
            self.push_sensor_state()

        self.init_direct_sensors()
        publish_hz = max(float(self.get_parameter("sensor_publish_hz").value), 1.0)
        self.create_timer(1.0 / publish_hz, self.publish_sensor_topics)

        self.create_subscription(
            Imu,
            "/imu",
            self.imu_callback,
            50,
        )
        self.create_subscription(
            String,
            "/excavator/joint_command_json",
            self.command_callback,
            10,
        )
        self.get_logger().info("Control executor node started (direct sensor feedback mode)")

    def init_direct_sensors(self):
        ports = list(self.get_parameter("sensor_ports").value)
        baud = int(self.get_parameter("sensor_baud").value)
        addr_list = [0x50, 0x51, 0x52, 0x53]
        self.get_logger().info(f"Control executor uses direct sensor ports: {ports}")
        self.get_logger().info(f"Control executor sensor baud: {baud}")

        for port in ports:
            try:
                dev = device_model.DeviceModel(port, port, baud, addr_list, self.make_sensor_callback(port))
                dev.openDevice()
                dev.startLoopRead()
                self.devices.append(dev)
                self.get_logger().info(f"{port} opened for direct closed-loop feedback")
            except Exception as exc:
                self.get_logger().warning(f"Failed to open {port}: {exc}")

    def make_sensor_callback(self, port_name):
        def update(dm):
            now_ts = time.time()
            touched = False
            with self.sensor_lock:
                for addr, name in self.addr_to_name.items():
                    data = dm.deviceData.get(addr, {})
                    if data and "AngX" in data:
                        self.sensor_data[name]["pitch"] = float(data.get("AngX", 0.0))
                        self.sensor_ts[name] = now_ts
                        touched = True
                        if addr not in self.seen_addr:
                            self.seen_addr.add(addr)
                            self.get_logger().info(
                                f"{port_name} received first direct AngX from {name}(0x{addr:02X})"
                            )
                        dm.deviceData[addr].clear()
            if touched:
                self.push_sensor_state()

        return update

    def publish_sensor_topics(self):
        with self.sensor_lock:
            bucket_pitch = self.sensor_data["铲斗"]["pitch"]
            arm_pitch = self.sensor_data["小臂"]["pitch"]
            boom_pitch = self.sensor_data["大臂"]["pitch"]
            swing_pitch = self.sensor_data["回转"]["pitch"]

        pitch_msg = Float64MultiArray()
        pitch_msg.data = [bucket_pitch, arm_pitch, boom_pitch, swing_pitch]
        self.pub_pitch_.publish(pitch_msg)

        relative_msg = Float64MultiArray()
        relative_msg.data = [
            bucket_pitch - arm_pitch,
            arm_pitch - boom_pitch,
            boom_pitch - swing_pitch,
        ]
        self.pub_relative_.publish(relative_msg)

    def push_sensor_state(self):
        if self.angle_ctrl is None:
            return
        with self.sensor_lock:
            data_copy = copy.deepcopy(self.sensor_data)
            ts_copy = dict(self.sensor_ts)
        self.angle_ctrl.update_sensor_state(data_copy, ts_copy)

    def imu_callback(self, msg):
        accel = (
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
            float(msg.linear_acceleration.z),
        )
        gyro = (
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        )
        timestamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        result = self.swing_estimator.process_imu(accel, gyro, timestamp_ns)
        if result is None:
            return

        swing_deg, _ = result
        now_ts = time.time()
        with self.sensor_lock:
            self.sensor_data["回转"]["yaw"] = float(swing_deg)
            self.sensor_ts["回转"] = now_ts
        self.push_sensor_state()

    def command_callback(self, msg):
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid joint command JSON: {exc}")
            return

        if command.get("command") == "stop_all":
            self.handle_stop_all()
            return

        joint_raw = command.get("joint", "")
        joint_name = JOINT_NAME_MAP.get(joint_raw)
        if joint_name is None:
            self.get_logger().error(f"Unsupported joint name: {joint_raw}")
            return

        target_val = float(command.get("target_val", 0.0))
        ch1_mv = int(command.get("ch1_mv", 0))
        ch2_mv = int(command.get("ch2_mv", 0))
        ch3_mv = int(command.get("ch3_mv", 2000))
        ramp_up_s = float(command.get("ramp_up_s", 0.2))
        ramp_down_s = float(command.get("ramp_down_s", 0.2))
        tolerance = float(command.get("tolerance_deg", 2.0))
        is_init_step = bool(command.get("is_init_step", False))
        description = command.get("description", "")

        if self.angle_ctrl is None:
            self.get_logger().warning(
                f"Offline mode, ignored command: joint={joint_name}, target={target_val}, CH3={ch3_mv}, desc={description}"
            )
            return

        self.push_sensor_state()
        self.get_logger().info(
            f"Execute command: joint={joint_name}, target={target_val:.2f}, "
            f"CH=({ch1_mv},{ch2_mv},{ch3_mv}), ramp=({ramp_up_s:.2f},{ramp_down_s:.2f}), "
            f"init={is_init_step}, desc={description}"
        )
        self.angle_ctrl.move_joint_to_angle(
            joint_name,
            target_val,
            tolerance=tolerance,
            ch1_mv=ch1_mv,
            ch2_mv=ch2_mv,
            ch3_mv=ch3_mv,
            ramp_up_s=ramp_up_s,
            ramp_down_s=ramp_down_s,
            is_init_step=is_init_step,
        )

    def handle_stop_all(self):
        if self.angle_ctrl is None:
            self.get_logger().warning("Offline mode, stop_all ignored")
            return
        self.get_logger().warning("Received stop_all command")
        self.angle_ctrl.stop_all()

    def destroy_node(self):
        if self.angle_ctrl is not None:
            try:
                self.angle_ctrl.stop_all()
            except Exception:
                pass
        for dev in self.devices:
            try:
                dev.stopLoopRead()
                dev.closeDevice()
            except Exception:
                pass
        if self.base_controller is not None:
            try:
                self.base_controller.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControlExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
