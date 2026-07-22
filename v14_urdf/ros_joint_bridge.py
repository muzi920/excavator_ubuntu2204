import math
import threading
import time


class RosJointBridge:
    """在 v14_urdf 中维护 /joint_states 的收发桥接。"""

    def __init__(self, node_name="v14_v4_joint_state_bridge"):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        self._rclpy = rclpy
        self._JointState = JointState

        if not self._rclpy.ok():
            self._rclpy.init()

        self._node = Node(node_name)
        self._pub = self._node.create_publisher(JointState, "/joint_states", 10)
        self._sub = self._node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

        self._lock = threading.Lock()
        self._last_joint_state = None
        self._last_joint_state_ts = 0.0
        self._cmd_initialized = False
        self._cmd_deg = {
            "swing_yaw": 0.0,
            "boom_swing": 0.0,
            "arm_boom": 0.0,
            "bucket_arm": 0.0,
        }

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    @staticmethod
    def _deg_to_rad(value):
        return value * math.pi / 180.0

    @staticmethod
    def _rad_to_deg(value):
        return value * 180.0 / math.pi

    def _spin(self):
        try:
            while self._rclpy.ok():
                self._rclpy.spin_once(self._node, timeout_sec=0.05)
        except Exception:
            # 关闭或外部中断时，rclpy 可能抛出 shutdown 相关异常；这里静默退出线程。
            return

    def _on_joint_state(self, msg):
        with self._lock:
            self._last_joint_state = msg
            self._last_joint_state_ts = time.time()

    def close(self):
        try:
            self._node.destroy_node()
        except Exception:
            pass
        try:
            if self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass

    def get_v4_angles_from_joint_states_deg(self):
        with self._lock:
            msg = self._last_joint_state
            ts = self._last_joint_state_ts

        if msg is None:
            return None

        name_to_pos = {}
        for idx, name in enumerate(list(msg.name)):
            try:
                name_to_pos[name] = msg.position[idx]
            except Exception:
                pass

        return {
            "ts": ts,
            "swing_yaw": self._rad_to_deg(name_to_pos.get("swing_joint", 0.0)),
            "boom_swing": self._rad_to_deg(name_to_pos.get("boom_joint", 0.0)),
            "arm_boom": self._rad_to_deg(name_to_pos.get("arm_joint", 0.0)),
            "bucket_arm": self._rad_to_deg(name_to_pos.get("bucket_joint", 0.0)),
        }

    def publish_v4_targets_deg(self, **kwargs):
        if not self._rclpy.ok():
            return False

        init_msg = None
        with self._lock:
            if not self._cmd_initialized:
                init_msg = self._last_joint_state

        if init_msg is not None:
            name_to_pos = {}
            for idx, name in enumerate(list(init_msg.name)):
                try:
                    name_to_pos[name] = init_msg.position[idx]
                except Exception:
                    pass
            initial_cmd = {
                "swing_yaw": self._rad_to_deg(name_to_pos.get("swing_joint", 0.0)),
                "boom_swing": self._rad_to_deg(name_to_pos.get("boom_joint", 0.0)),
                "arm_boom": self._rad_to_deg(name_to_pos.get("arm_joint", 0.0)),
                "bucket_arm": self._rad_to_deg(name_to_pos.get("bucket_joint", 0.0)),
            }
            with self._lock:
                if not self._cmd_initialized:
                    for key, value in initial_cmd.items():
                        self._cmd_deg[key] = float(value)
                    self._cmd_initialized = True

        with self._lock:
            for key, value in kwargs.items():
                if key in self._cmd_deg and value is not None:
                    self._cmd_deg[key] = float(value)
            cmd = dict(self._cmd_deg)

        msg = self._JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"]
        msg.position = [
            self._deg_to_rad(cmd["swing_yaw"]),
            self._deg_to_rad(cmd["boom_swing"]),
            self._deg_to_rad(cmd["arm_boom"]),
            self._deg_to_rad(cmd["bucket_arm"]),
        ]
        try:
            self._pub.publish(msg)
            return True
        except Exception:
            return False
