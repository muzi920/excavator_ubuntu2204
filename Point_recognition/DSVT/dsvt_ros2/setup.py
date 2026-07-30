"""DSVT ROS2 Package Setup."""

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dsvt_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 文件
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # 配置文件
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='libo',
    maintainer_email='libo@example.com',
    description='DSVT 3D LiDAR Object Detection ROS2 Inference Pipeline',
    license='MIT',
    entry_points={
        'console_scripts': [
            'inference_node = dsvt_ros2.inference_node:main',
            'pc_publisher = dsvt_ros2.pc_publisher_node:main',
        ],
    },
)
