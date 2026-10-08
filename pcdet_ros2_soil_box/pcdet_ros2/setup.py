import glob
import os

from setuptools import setup

package_name = 'pcdet_ros2'


def package_files(pattern):
    """Return only regular files matched by a data-files glob."""
    return [path for path in glob.glob(pattern) if os.path.isfile(path)]


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            package_files(os.path.join('launch', '*'))),
        ('share/' + package_name + '/config',
            package_files(os.path.join('config', '*'))),
        ('share/' + package_name + '/cfgs',
            package_files(os.path.join('cfgs', '*.*'))),
        ('share/' + package_name + '/cfgs/custom_models',
            package_files(os.path.join('cfgs/custom_models', '*.*'))),
        ('share/' + package_name + '/cfgs/dataset_configs',
            package_files(os.path.join('cfgs/dataset_configs', '*.*'))),
        ('share/' + package_name + '/cfgs/kitti_models',
            package_files(os.path.join('cfgs/kitti_models', '*.*'))),
        ('share/' + package_name + '/cfgs/lyft_models',
            package_files(os.path.join('cfgs/lyft_models', '*.*'))),
        ('share/' + package_name + '/cfgs/nuscenes_models',
            package_files(os.path.join('cfgs/nuscenes_models', '*.*'))),
        ('share/' + package_name + '/cfgs/once_models',
            package_files(os.path.join('cfgs/once_models', '*.*'))),
        ('share/' + package_name + '/cfgs/waymo_models',
            package_files(os.path.join('cfgs/waymo_models', '*.*'))),
        ('share/' + package_name + '/checkpoints',
            package_files(os.path.join('checkpoints', '*'))),
    ],
    install_requires=['setuptools', 'pcdet'],
    zip_safe=True,
    maintainer='Shrijal Pradhan',
    maintainer_email='pradhan.shrijal@gmail.com',
    description='ROS 2 Wrapper for OpenPCDet',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pcdet = pcdet_ros2.pcdet_node:main',
        ],
    },
)
