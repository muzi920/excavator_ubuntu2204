import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/pcdet_ros2_soil_box/ros2_ws/install/pcdet_ros2'
