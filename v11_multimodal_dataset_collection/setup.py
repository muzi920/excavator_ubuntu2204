from setuptools import setup
import os
from glob import glob

package_name = 'v11_multimodal_dataset_collection'

setup(
    name=package_name,
    version='0.1.0',
    py_modules=['hikvision_cam_node', 'network_cam_node'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装 launch 文件
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='libo',
    maintainer_email='user@example.com',
    description='ROS2 camera RTSP streaming nodes (Hikvision + network cameras)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hikvision_cam_node = hikvision_cam_node:main',
            'network_cam_node = network_cam_node:main',
        ],
    },
)
