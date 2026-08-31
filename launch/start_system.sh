#!/bin/bash
# 一键启动脚本：包含点云实时检测、V11多模态控制、D435i相机节点

# 启动第一个终端：DSVT API 服务 (Conda环境)
gnome-terminal --title="DSVT API Server" -- bash -c "
source ~/miniconda3/etc/profile.d/conda.sh;
conda activate dsvt;
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/Point_recognition/DSVT;
export PYTHONPATH=/media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/Point_recognition/online:\$PYTHONPATH
echo 'Starting DSVT API Server...';
python ../online/api_server.py \
    --cfg_file tools/cfgs/custom_models/second.yaml \
    --ckpt output/cfgs/custom_models/second/default/ckpt/checkpoint_epoch_79.pth;
exec bash" &

# 启动第二个终端：ROS2 Filter Node (非Conda环境)
gnome-terminal --title="ROS2 Filter Node" -- bash -c "
source /opt/ros/humble/setup.bash;
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/Point_recognition/online;
echo 'Starting ROS2 Filter Node...';
/usr/bin/python3 ros2_filter_node.py --ros-args -p topic_in:=/lidar/points_odom;
exec bash" &

# 启动第三个终端：V11 多模态控制 GUI
gnome-terminal --title="V11 Multimodal GUI" -- bash -c "
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws;
source /opt/ros/humble/setup.bash;
source install/setup.bash;
echo 'Starting V11 Multimodal GUI...';
/usr/bin/python3 src/shandong/v11_multimodal_dataset_collection/ros2_multimodal_gui.py;
exec bash" &

# 启动第四个终端：D435i Realsense Camera
gnome-terminal --title="D435i Camera" -- bash -c "
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/realsense-ros;
source /opt/ros/humble/setup.bash;
source ../../install/setup.bash;
echo 'Starting D435i Camera...';
ros2 launch realsense2_camera rs_launch.py;
exec bash" &

# 启动第五个终端：RViz2 可视化
gnome-terminal --title="RViz2" -- bash -c "
source /opt/ros/humble/setup.bash;
echo 'Starting RViz2...';
rviz2;
exec bash" &

echo "All system components have been launched in separate terminals."
