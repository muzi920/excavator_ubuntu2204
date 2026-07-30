# DSVT ROS2 实时 3D 点云目标检测

基于 ROS2 的 DSVT/PointPillars 实时 LiDAR 点云 3D 目标检测方案。

## 目录结构

```
dsvt_ros2/
├── README.md
├── package.xml                       # ROS2 包清单
├── setup.py                          # Python 包安装
├── CMakeLists.txt                    # ROS2 CMake (C++ 未来支持)
├── resource/                         # ROS2 ament 资源
├── launch/
│   └── dsvt_inference.launch.py     # ROS2 launch 文件
├── config/
│   └── params.yaml                   # 默认参数
├── dsvt_ros2/                        # Python 包
│   ├── __init__.py
│   ├── inference_engine.py           # 核心: 模型加载 + 推理
│   ├── inference_node.py             # ROS2 推理节点 (rclpy)
│   ├── pc_publisher_node.py          # ROS2 测试节点: 文件→PointCloud2
│   ├── visualizer_node.py            # ROS2 可视化节点
│   └── utils.py                      # ROS2 消息转换工具
├── scripts/
│   ├── run_standalone_demo.py        # 独立 Demo (无需 ROS2) ✅ 可直接运行
│   └── setup_ros2_env.sh            # ROS2 环境配置脚本
└── test/
```

## 快速开始

### 1. 独立 Demo (无需 ROS2, 可直接运行)

```bash
# PointPillars 模型 (已有 Soil checkpoint)
python dsvt_ros2/scripts/run_standalone_demo.py \
    --ckpt /home/libo/PointPillars/soil_logs/checkpoints/best.pth \
    --data_path pcd_npy/ \
    --score_thresh 0.1 \
    --no_vis

# 单帧 + 可视化
python dsvt_ros2/scripts/run_standalone_demo.py \
    --ckpt /home/libo/PointPillars/soil_logs/checkpoints/best.pth \
    --data_path pcd_npy/fused3_pointcloud_0000_1779956573451778231.npy
```

### 2. ROS2 推理

**前置条件**: ROS2 Humble + Python 3.10 环境 (当前 conda 环境为 3.11, 需要 Python 3.10)

```bash
# 方案 A: 使用 Python 3.10 conda 环境
conda create -n dsvt_ros2 python=3.10
conda activate dsvt_ros2
pip install torch numpy open3d
source /opt/ros/humble/setup.bash

# 方案 B: 通过 robostack 在当前环境安装 ROS2
# conda install -c robostack -c conda-forge ros-humble-rclpy ros-humble-sensor-msgs
```

```bash
# 安装 dsvt_ros2 包
cd /path/to/DSVT/dsvt_ros2
pip install -e .

# 启动推理 pipeline
ros2 launch dsvt_ros2 dsvt_inference.launch.py \
    ckpt_path:=/home/libo/PointPillars/soil_logs/checkpoints/best.pth \
    data_path:=pcd_npy/

# 或者分步启动
ros2 run dsvt_ros2 pc_publisher \
    --ros-args -p data_path:=pcd_npy/ -p rate:=10.0 &
ros2 run dsvt_ros2 inference_node \
    --ros-args -p ckpt_path:=/path/to/checkpoint.pth -p class_names:=Soil
```

### 3. RViz2 可视化

```bash
rviz2
# 添加 Marker 显示, topic 设为 /perception/markers
```

## ROS2 节点架构

```
/lidar/points (PointCloud2 @10Hz)
    │
    ▼
┌─────────────────────────────┐
│  dsvt_inference_node        │  GPU 推理
│  - VFE / PillarEncoder      │
│  - DSVT / PointPillars      │
│  - Detection Head           │
│  - Post-processing (NMS)    │
└──────┬──────────────────────┘
       │
       ▼
/perception/detections (Detection3DArray)
/perception/markers     (MarkerArray, RViz2)
```

## 支持模型

| 引擎类型 | 类名 | 配置 | Checkpoint 格式 |
|---------|------|------|----------------|
| `InferenceEngine` | OpenPCDet (DSVT, SECOND, etc.) | `.yaml` + `.pth` | `{'model_state': ..., ...}` |
| `PointPillarsEngine` | 独立 PointPillars | 无配置文件 | raw `state_dict` |
| `create_engine` | 自动检测 | 自动 | 自动 |

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `cfg_file` | - | OpenPCDet 配置文件 (PointPillars 不需要) |
| `ckpt_path` | **必填** | 模型权重文件路径 |
| `engine_type` | auto | auto / openpcdet / pointpillars |
| `class_names` | Soil | 类别名 (逗号分隔) |
| `point_cloud_range` | -75.2,-75.2,-2,75.2,75.2,4 | 点云 ROI 范围 |
| `score_thresh` | 0.1 | 置信度阈值 |
| `device` | cuda | cuda / cpu |

## 性能

| 场景 | 模型 | 延迟 | GPU |
|------|------|------|-----|
| 独立 Demo | PointPillars (Soil) | ~50ms | RTX 3090 |
| ROS2 节点 | PointPillars (Soil) | ~50ms | RTX 3090 |
| TRT 优化 (Phase 1) | DSVT | ~25ms | Jetson Orin AGX |

注: 独立 Demo 首次推理含 CUDA warmup (~300ms), 后续帧约 50ms。

## 下一步

1. **训练 DSVT 模型**: 在自定义数据集上训练 DSVT 获得 OpenPCDet checkpoint
2. **TensorRT 全链路**: 将 VFE + DSVT + Head 全部转 TRT (参考 plan)
3. **Jetson Orin 部署**: 迁移到 Jetson, 实现实时推理 (目标 20-30Hz)
4. **多目标跟踪**: 添加卡尔曼滤波 / Hungarian 匹配跟踪
