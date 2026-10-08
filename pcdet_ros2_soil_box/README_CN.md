# Soil + Box 无 Conda 最小迁移

本文件夹用于在另一台 Ubuntu 24.04 + ROS 2 Jazzy + NVIDIA CUDA 电脑上复现当前
Soil + Box 实时点云检测。它不使用 Conda，也不包含训练数据、rosbag 或旧电脑
编译的 CUDA 二进制文件。

## 文件内容

```text
no_conda_migration_soil_box_20260920/
├── ros2_ws/src/pcdet_ros2/       ROS 2 节点、配置和权重
├── third_party/DSVT/             OpenPCDet/DSVT 源码
├── requirements.txt              venv Python 依赖
└── scripts/
    ├── install_and_build.sh      创建 venv、安装并编译
    ├── verify_bundle.sh          检查迁移文件
    ├── verify_install.sh         检查目标机安装
    ├── run_detection.sh          启动检测
    └── run_rviz.sh               启动 RViz2
```

## 目标机要求

- Ubuntu 24.04；
- ROS 2 Jazzy；
- 系统 Python 3.12；
- NVIDIA 驱动和 CUDA Toolkit 12.8；
- 支持 CUDA 的 NVIDIA GPU；
- 当前默认编译架构为 RTX 5060 Ti 的 `12.0`。

安装 ROS 2 Jazzy 后执行：

```bash
sudo apt update
sudo apt install \
  python3-venv \
  python3-dev \
  python3-colcon-common-extensions \
  build-essential \
  ninja-build \
  ros-jazzy-desktop \
  ros-jazzy-sensor-msgs-py \
  ros-jazzy-vision-msgs \
  ros-jazzy-vision-msgs-rviz-plugins
```

确认：

```bash
nvidia-smi
nvcc --version
```

## 安装和构建

进入本文件夹后先检查文件：

```bash
bash scripts/verify_bundle.sh
```

RTX 5060 Ti 使用：

```bash
CUDA_HOME=/usr/local/cuda-12.8 \
TORCH_CUDA_ARCH_LIST=12.0 \
bash scripts/install_and_build.sh
```

安装脚本会在本文件夹创建隔离的 `.venv`，安装 PyTorch、spconv 和 OpenPCDet，
再重新编译 OpenPCDet CUDA 扩展。整个过程不调用 Conda，也不使用
`--system-site-packages`。

其他 GPU 架构需要修改 `TORCH_CUDA_ARCH_LIST`，例如：

```bash
TORCH_CUDA_ARCH_LIST=8.9 bash scripts/install_and_build.sh
```

然后验证：

```bash
bash scripts/verify_install.sh
```

预期包含：

```text
CUDA available: True
OpenPCDet CUDA extensions: OK
pcdet_ros2 package: OK
Installation verification: OK
```

## 启动检测

默认输入话题是 `/pointcloud`，默认输出是 `/detection3_d_array`：

```bash
bash scripts/run_detection.sh
```

也可以指定话题：

```bash
bash scripts/run_detection.sh /你的点云话题 /你的检测输出话题
```

启动日志应包含：

```text
Done (loaded 391/391)
Loaded CenterPoint ... classes=['Soil', 'Box']
```

当前配置每帧最多保留一个 Soil 和一个 Box：

- Soil 发布框内最高点（挖掘点）；
- Box 发布框顶面中心点 `(cx, cy, cz + dz/2)`。

```text
/pointcloud
  sensor_msgs/msg/PointCloud2

/detection3_d_array
  vision_msgs/msg/Detection3DArray

/perception/soil_highest_points
  geometry_msgs/msg/PoseArray

/perception/soil_highest_point_markers
  visualization_msgs/msg/MarkerArray

/perception/box_highest_points
  geometry_msgs/msg/PoseArray

/perception/box_highest_point_markers
  visualization_msgs/msg/MarkerArray
```

### 检测框与最高点平滑

检测框和 Soil 最高点默认开启时间平滑（EMA），避免逐帧抖动。在
`ros2_ws/src/pcdet_ros2/config/pcdet_second_soil_box.param.yaml` 中调节：

```yaml
# Soil 最高点（挖掘点）
highest_point_smoothing_alpha: 0.35   # EMA 强度，越小越稳、越滞后；1.0 = 不平滑
highest_point_hold_frames: 5          # 检测丢失后继续发布最后一点的帧数
highest_point_max_jump: 2.0           # 新点距平滑点超过此距离(米)则直接跳过去

# 3D 检测框（Soil / Box 各一个）
box_smoothing_alpha: 0.5
box_hold_frames: 5
box_max_jump: 2.0
```

- `alpha` 调大更跟手但有抖动，调小更平滑但有滞后；
- `max_jump` 用于在目标切换时立即重锚而不是拖尾；
- `hold_frames` 用于目标短暂丢失时保持输出。

Box 顶面中心点由平滑后的 Box 框直接算出，继承框的平滑，无需单独调节。
改完参数重新构建后生效。

## RViz2

启动：

```bash
bash scripts/run_rviz.sh
```

RViz 设置：

```text
Global Options -> Fixed Frame: base_link
PointCloud2 -> Topic: /pointcloud
Detection3DArray -> Topic: /detection3_d_array
MarkerArray -> Topic: /perception/soil_highest_point_markers   (Soil 挖掘点,红色球)
MarkerArray -> Topic: /perception/box_highest_point_markers    (Box 顶面中心,蓝色球)
```

当前点云和检测消息的 `frame_id` 是 `base_link`。如果 Fixed Frame 使用 `odom`，
必须存在 `base_link -> odom` TF。

## 诊断

不要只看 `ros2 topic list`，还要确认话题有发布者：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 topic info /pointcloud --verbose
ros2 topic hz /pointcloud
ros2 topic echo /detection3_d_array --once
```

如果 `/pointcloud` 的 `Publisher count` 为 0，说明雷达或 rosbag 没有发布数据。

模型输入特征为 `x/y/z/intensity`。当前 `intensity_scale` 是 `0.0`，适用于训练
强度通道为零的权重；如果训练使用真实强度，需要同步修改参数并确保输入消息
包含 intensity 字段。
