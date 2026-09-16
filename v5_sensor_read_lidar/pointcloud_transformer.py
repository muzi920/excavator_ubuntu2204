import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import tf2_ros
import tf2_sensor_msgs

class PointCloudTransformer(Node):
    def __init__(self):
        super().__init__('pointcloud_transformer')
        
        # 目标坐标系
        self.target_frame = 'base_link'
        
        # 订阅原始点云数据
        self.subscription = self.create_subscription(
            PointCloud2,
            '/pointcloud',
            self.listener_callback,
            10)
            
        # 发布转换后的点云数据
        self.publisher = self.create_publisher(
            PointCloud2,
            '/pointcloud_base_link',
            10)
            
        # TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 缓存一下是否成功获取过 TF，减少打印警告
        self.tf_found = False
        
        self.get_logger().info('点云转换节点已启动。')
        self.get_logger().info(f'正在订阅 /pointcloud，并将转换后的点云发布到 /pointcloud_base_link (目标坐标系: {self.target_frame})')

    def listener_callback(self, msg):
        try:
            # 查找从点云原始坐标系 (如 map) 到目标坐标系 (base_link) 的最新变换关系
            t = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )
            
            if not self.tf_found:
                self.get_logger().info(f'成功获取到 {msg.header.frame_id} -> {self.target_frame} 的 TF 变换关系，开始发布点云...')
                self.tf_found = True
                
            # 执行点云坐标系转换
            transformed_cloud = tf2_sensor_msgs.do_transform_cloud(msg, t)
            
            # 发布转换后的点云
            self.publisher.publish(transformed_cloud)
            
        except tf2_ros.LookupException as e:
            # 只在最开始的时候警告，避免刷屏
            if not self.tf_found:
                self.get_logger().warn(f'等待 TF 变换关系: {e}')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f'TF 时间外推失败: {e}')
        except Exception as e:
            self.get_logger().error(f'点云转换时发生未知错误: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
