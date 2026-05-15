#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class SimpleTfPublisher(Node):
    def __init__(self):
        super().__init__('simple_tf_publisher')
        
        # 使用 StaticTransformBroadcaster，避免因点云时间戳与当前系统时间不同步导致的报错
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 发布一次静态 TF 即可，静态 TF 会对所有时间生效
        self.publish_static_tf()
        self.get_logger().info('已发布静态 TF: map -> base_link')

    def publish_static_tf(self):
        t = self._create_transform_msg()
        self.static_tf_broadcaster.sendTransform(t)

    def _create_transform_msg(self):
        t = TransformStamped()
        
        # 时间戳：这里需要加上一点未来时间的偏移量，或者直接取当前时间
        # ROS2 中有时候发布频率过高或者系统时间同步问题会导致 timestamp dropping 报错
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'        # 父坐标系
        t.child_frame_id = 'base_link'   # 子坐标系
        
        # 平移变换
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # 旋转变换 (雷达倒装，roll = 180度 = pi)
        roll = math.pi
        pitch = 0.0
        yaw = 0.0
        
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        return t

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
