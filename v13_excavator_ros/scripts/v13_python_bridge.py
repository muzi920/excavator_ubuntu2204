#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class V13PythonBridge(Node):
    def __init__(self) -> None:
        super().__init__("v13_python_bridge")
        self.get_logger().info("v13 Python bridge is ready.")


def main() -> None:
    rclpy.init()
    node = V13PythonBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
