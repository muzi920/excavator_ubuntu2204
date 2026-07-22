#!/usr/bin/env python3
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))


def resolve_device_model_dir():
    candidates = [
        os.path.abspath(
            os.path.join(
                CURRENT_DIR,
                "..",
                "..",
                "..",
                "..",
                "src",
                "shandong",
                "v3_sensor_read_wit",
                "WitStandardModbus_WT901C485-main",
                "Python",
                "Python-SDK-WT901C485_new",
            )
        ),
        os.path.abspath(
            os.path.join(
                CURRENT_DIR,
                "..",
                "..",
                "v3_sensor_read_wit",
                "WitStandardModbus_WT901C485-main",
                "Python",
                "Python-SDK-WT901C485_new",
            )
        ),
    ]
    for candidate in candidates:
        if os.path.exists(os.path.join(candidate, "device_model.py")):
            return candidate
    return candidates[0]


DEVICE_MODEL_DIR = resolve_device_model_dir()
if DEVICE_MODEL_DIR not in sys.path:
    sys.path.append(DEVICE_MODEL_DIR)

import device_model


class InclinometerSensorBridge(Node):
    def __init__(self):
        super().__init__("inclinometer_sensor_bridge")

        self.declare_parameter(
            "sensor_ports",
            ["/dev/ttyUSB_Sensor1", "/dev/ttyUSB_Sensor2", "/dev/ttyUSB_Sensor3", "/dev/ttyUSB_Sensor4"],
        )
        self.declare_parameter("sensor_baud", 230400)
        self.declare_parameter("publish_hz", 20.0)

        self.addr_to_index = {0x50: 0, 0x51: 1, 0x52: 2, 0x53: 3}
        self.addr_to_name = {0x50: "铲斗", 0x51: "小臂", 0x52: "大臂", 0x53: "回转"}
        self.pitch_deg = [0.0, 0.0, 0.0, 0.0]
        self.data_lock = threading.Lock()
        self.devices = []
        self.seen_addr = set()
        self.running = True
        self.publish_count = 0
        self.publish_period = 1.0 / max(float(self.get_parameter("publish_hz").value), 1.0)
        self.start_time = time.time()

        self.pub_pitch_ = self.create_publisher(Float64MultiArray, "/excavator/inclinometer_pitch_deg", 10)
        self.pub_relative_ = self.create_publisher(Float64MultiArray, "/excavator/inclinometer_relative_deg", 10)

        ports = list(self.get_parameter("sensor_ports").value)
        baud = int(self.get_parameter("sensor_baud").value)
        addr_list = list(self.addr_to_index.keys())
        self.get_logger().info(f"Configured sensor ports: {ports}")
        self.get_logger().info(f"Configured sensor baud: {baud}")
        self.get_logger().info(f"Using device_model from: {DEVICE_MODEL_DIR}")

        for port in ports:
            try:
                dev = device_model.DeviceModel(port, port, baud, addr_list, self.make_sensor_callback(port))
                dev.openDevice()
                dev.startLoopRead()
                self.devices.append(dev)
                self.get_logger().info(f"{port} opened via V11 device_model bridge")
            except Exception as exc:
                self.get_logger().warning(f"Failed to open {port}: {exc}")

        self.get_logger().info(f"Opened device count: {len(self.devices)}")
        self.publisher_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.publisher_thread.start()
        self.get_logger().info("Inclinometer sensor bridge started")

    def make_sensor_callback(self, port_name):
        def update(dm):
            for addr, index in self.addr_to_index.items():
                data = dm.deviceData.get(addr, {})
                if data and "AngX" in data:
                    with self.data_lock:
                        self.pitch_deg[index] = data.get("AngX", 0.0)
                    if addr not in self.seen_addr:
                        self.seen_addr.add(addr)
                        self.get_logger().info(
                            f"{port_name} received first valid AngX from {self.addr_to_name[addr]}(0x{addr:02X})"
                        )
                    dm.deviceData[addr].clear()

        return update

    def publish_loop(self):
        while self.running:
            with self.data_lock:
                pitch_values = list(self.pitch_deg)

            pitch_msg = Float64MultiArray()
            pitch_msg.data = pitch_values

            bucket_pitch = pitch_values[0]
            arm_pitch = pitch_values[1]
            boom_pitch = pitch_values[2]
            swing_pitch = pitch_values[3]

            # Match V11 display/cache order:
            # [bucket-arm, arm-boom, boom-swing]
            relative_msg = Float64MultiArray()
            relative_msg.data = [
                bucket_pitch - arm_pitch,
                arm_pitch - boom_pitch,
                boom_pitch - swing_pitch,
            ]

            self.pub_pitch_.publish(pitch_msg)
            self.pub_relative_.publish(relative_msg)

            self.publish_count += 1
            if self.publish_count == 1:
                self.get_logger().info(f"First publish sent: pitch_deg={pitch_values}")
            elif self.publish_count % 50 == 0 and not self.seen_addr:
                elapsed = time.time() - self.start_time
                self.get_logger().warning(
                    f"No valid AngX received after {elapsed:.1f}s, still publishing defaults: {pitch_values}"
                )
            time.sleep(self.publish_period)

    def destroy_node(self):
        self.running = False
        if hasattr(self, "publisher_thread") and self.publisher_thread.is_alive():
            self.publisher_thread.join(timeout=0.5)
        for dev in self.devices:
            try:
                dev.stopLoopRead()
                dev.closeDevice()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InclinometerSensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
