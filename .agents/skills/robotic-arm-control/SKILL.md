---
name: "robotic-arm-control"
description: "Provides ROS2 mechanical arm control guidelines and templates. Invoke when the user asks to control an excavator arm, robotic joint, or send commands to motors/servos."
---

# Robotic Arm Control (ROS2)

This skill provides the best practices and code templates for controlling multi-joint robotic arms or excavators in a ROS2 environment.

## When to Invoke
- When creating a publisher to send joint commands to the robotic arm.
- When mapping real-world physical angles (e.g., from IMUs or encoders) to robotic arm joint states.
- When creating inverse kinematics (IK) wrappers.

## Best Practices

1. **Use Standard Message Types**:
   Always prefer standard ROS2 message types for joint control:
   - `sensor_msgs/msg/JointState` for broadcasting current positions.
   - `trajectory_msgs/msg/JointTrajectory` for sending target commands to controllers.

2. **Coordinate Systems (TF)**:
   - Always map the base of the arm to `base_link` and subsequent joints in a strict hierarchical order.
   - For excavators: `base_link` -> `boom` (大臂) -> `arm` (小臂) -> `bucket` (铲斗).

3. **Rate Limiting & Safety**:
   - Limit the command publication rate to match the hardware receiver's capability (usually 10Hz to 50Hz).
   - Implement software limits on maximum angles and maximum velocities to prevent hardware collisions.

## Template: Basic Joint Publisher
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.publisher_ = self.create_publisher(JointState, 'joint_commands', 10)
        
    def send_command(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['boom_joint', 'arm_joint', 'bucket_joint']
        msg.position = positions # Expects list of radians
        self.publisher_.publish(msg)
```
