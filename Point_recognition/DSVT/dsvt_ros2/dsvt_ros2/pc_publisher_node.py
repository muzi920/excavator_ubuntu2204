#!/usr/bin/env python3
"""
测试用点云发布节点。

读取本地 .bin / .npy / .pcd 文件，定期发布为 sensor_msgs/PointCloud2。

用法:
    ros2 run dsvt_ros2 pc_publisher \
        --ros-args -p data_path:=/path/to/points.npy -p rate:=10.0
"""

import glob
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from dsvt_ros2.utils import numpy_to_pointcloud2


class PointCloudPublisher(Node):
    """从本地文件发布 PointCloud2 消息。"""

    def __init__(self):
        super().__init__('pc_publisher')

        self.declare_parameter('data_path', '')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('frame_id', 'lidar')

        data_path = self.get_parameter('data_path').get_parameter_value().string_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        if not data_path:
            self.get_logger().fatal('data_path 参数未设置!')
            raise RuntimeError('Missing required parameter: data_path')

        path = Path(data_path)
        if path.is_dir():
            self.file_list = sorted(glob.glob(str(path / '*.bin'))
                                    + glob.glob(str(path / '*.npy'))
                                    + glob.glob(str(path / '*.pcd')))
        else:
            self.file_list = [str(path)]

        if not self.file_list:
            raise FileNotFoundError(f'No point cloud files in: {data_path}')

        self.get_logger().info(f'Found {len(self.file_list)} files, rate={self.rate}Hz')

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(PointCloud2, '/lidar/points', qos)

        period = 1.0 / max(self.rate, 0.1)
        self.timer = self.create_timer(period, self.timer_callback)
        self.idx = 0

    def timer_callback(self):
        file_path = self.file_list[self.idx]
        try:
            points = self._load_points(file_path)
        except Exception as e:
            self.get_logger().error(f'Failed to load {file_path}: {e}')
            self._advance()
            return

        msg = numpy_to_pointcloud2(points, frame_id=self.frame_id)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.get_logger().info(
            f'Published {Path(file_path).name} ({points.shape[0]} pts)',
            throttle_duration_sec=2.0,
        )
        self._advance()

    def _advance(self):
        self.idx += 1
        if self.idx >= len(self.file_list):
            if self.loop:
                self.idx = 0
                self.get_logger().info('Looping back to first file.')
            else:
                self.get_logger().info('All files published. Shutting down.')
                self.timer.cancel()

    @staticmethod
    def _load_points(file_path):
        ext = Path(file_path).suffix.lower()
        if ext == '.bin':
            return np.fromfile(file_path, dtype=np.float32).reshape(-1, 4)
        elif ext == '.npy':
            return np.load(file_path)
        elif ext == '.pcd':
            import open3d as o3d
            pcd = o3d.io.read_point_cloud(file_path)
            xyz = np.asarray(pcd.points)
            intensity = (np.asarray(pcd.colors)[:, 0] if pcd.has_colors()
                         else np.zeros((xyz.shape[0], 1)))
            return np.column_stack([xyz, intensity])
        raise ValueError(f'Unsupported format: {ext}')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
