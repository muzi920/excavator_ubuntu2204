# Point_recognition —— 点云目标识别模块

本目录承载挖掘机作业场景中的点云在线目标识别能力。核心思路是用 DSVT（Dynamic Sparse Voxel Transformer）模型对雷达点云做 3D 目标检测，识别出土堆等目标物体，并将检测结果接入 ROS 2 系统供后续挖掘规划使用。

---

## 目录结构

```text
Point_recognition/
├── DSVT/                    # DSVT 3D 目标检测框架（基于 OpenPCDet）
│   ├── pcdet/               # 核心数据集、模型、算子、评估逻辑
│   ├── tools/               # 训练、测试、demo、数据处理脚本
│   │   ├── cfgs/            # 模型与数据集配置文件
│   │   │   ├── custom_models/     # 自定义 Soil 单类模型配置
│   │   │   ├── dataset_configs/   # 数据集配置
│   │   │   ├── dsvt_models/       # DSVT 官方模型配置
│   │   │   └── waymo_models/      # Waymo 基线模型配置
│   │   ├── train.py         # 训练入口
│   │   ├── test.py          # 评估入口
│   │   └── demo.py          # 单帧推理可视化
│   ├── kitti_data/          # 原始点云与标注数据（PCD + KITTI 格式标签）
│   ├── data/custom/         # 转换后的自定义训练数据集
│   ├── docs/                # 安装文档与中文数据转换说明
│   ├── README.md            # DSVT 官方说明
│   └── README_CUSTOM.md     # 自定义数据集训练/评估/推理完整指南
│
├── online/                  # 在线推理与 ROS 2 接入模块
│   ├── online_detector.py   # 核心检测器类（Conda 环境）
│   ├── api_server.py        # FastAPI 推理服务（Conda 环境）
│   ├── folder_watcher.py    # 离线文件夹监控推理（Conda 环境）
│   ├── ros2_online_node.py  # ROS 2 直连推理节点（已废弃，见下方说明）
│   ├── ros2_bridge_node.py  # ROS 2 轻量桥接节点（系统环境）
│   ├── ros2_filter_node.py  # ROS 2 过滤+染色节点（系统环境）
│   └── README.md            # 在线模块详细使用说明
│
└── trae/                    # Trae 项目上下文文档
    └── TRAE_PROJECT_CONTEXT.md
```

---

## 架构设计

### 为什么用 IPC 架构

ROS 2 Humble 要求系统级 Python 3.10，而 DSVT 模型依赖 Conda Python 3.11 + PyTorch + CUDA。
两个环境的底层 C++ 库版本冲突，无法在同一个进程中同时导入 `rclpy` 和 `pcdet`。

因此采用 **HTTP API 桥接**：

```text
┌──────────────────────────────────┐     HTTP POST      ┌──────────────────────────┐
│  ROS 2 节点（系统 Python 3.10）  │ ──────────────────> │  API 服务（Conda Python） │
│  ros2_bridge_node.py             │ <────────────────── │  api_server.py           │
│  ros2_filter_node.py             │   JSON 检测结果     │  OnlineDetector          │
│  订阅 /pointcloud_base_link      │                     │  GPU 模型驻留            │
│  发布 /dsvt_detections           │                     │  FastAPI + uvicorn       │
│  发布 /dsvt_filtered_points      │                     │  localhost:8000/predict   │
└──────────────────────────────────┘                     └──────────────────────────┘
```

### 数据流

```text
雷达点云 (/pointcloud_base_link)
    │
    ├── ros2_bridge_node.py ──> API ──> 3D 检测框 ──> /dsvt_detections (MarkerArray)
    │
    └── ros2_filter_node.py ──> API ──> 3D 检测框
                                      ├─> 框内点云过滤 + z>=0 筛选
                                      ├─> 红色染色
                                      └─> /dsvt_filtered_points (PointCloud2)
```

---

## 检查结果

在代码审查中发现以下问题：

### 1. ros2_online_node.py 存在环境冲突（已废弃）

[ros2_online_node.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/Point_recognition/online/ros2_online_node.py) 直接在 ROS 2 节点中 `from online_detector import OnlineDetector`，而 `OnlineDetector` 需要 `torch` 和 `pcdet`，这些只在 Conda 环境中可用。

在系统 Python 3.10 下运行会直接 `ImportError`。这个文件应该是早期尝试直连的版本，已被 IPC 架构（`api_server.py` + `ros2_bridge_node.py`）替代。建议后续删除或标记为废弃。

### 2. online/README.md 中路径错误

`online/README.md` 中的示例路径写的是：

```text
/media/libo/libo_sn7100/ubuntu2204/PointCloud_ws/src/Point_recognition/online
```

实际路径应该是：

```text
/media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/Point_recognition/online
```

### 3. ros2_filter_node.py 中 struct 导入位置

[ros2_filter_node.py 第 148 行](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/Point_recognition/online/ros2_filter_node.py#L148) 在函数体内 `import struct`，虽然不影响运行，但建议移到文件顶部。

### 4. ros2_filter_node.py 的 z<0 过滤位置

`filter_points_in_boxes()` 中先过滤 `z < 0`，再做框内判断。这个顺序不影响结果，但如果未来要改成可配置的高度阈值，建议提取为参数。

---

## 与主项目的关联

本模块的检测结果可以接入 `v14_urdf` 的挖掘规划链路：

```text
Point_recognition                    v14_urdf
─────────────────                    ────────
/dsvt_detections (3D 框)     ──>     候选挖掘区域筛选
/dsvt_filtered_points        ──>     土堆表面点云提取
(目标点云，红色)                      point_to_dig_dump_trajectory.py
                                     mode1/mode2 规划
```

当前 `v14_urdf/mode1/real_pcd/` 中的点云处理链路使用的是离线 PCD 文件。
如果要切换到在线识别，可以直接用 `/dsvt_filtered_points` 替代离线 PCD 作为土堆点云来源。

---

## 快速启动

详见 [online/README.md](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/Point_recognition/online/README.md) 的完整启动指南。核心步骤：

1. **Conda 终端**：启动 API 服务
```bash
conda activate dsvt
cd src/shandong/Point_recognition/online
python api_server.py \
    --cfg_file ../DSVT/tools/cfgs/custom_models/second.yaml \
    --ckpt ../DSVT/output/cfgs/custom_models/second/default/ckpt/checkpoint_epoch_79.pth
```

2. **系统终端**：启动 ROS 2 节点（二选一）
```bash
source /opt/ros/humble/setup.bash
cd src/shandong/Point_recognition/online

# 方案 A：只看 3D 框
python3 ros2_bridge_node.py --ros-args -p topic:=/pointcloud_base_link

# 方案 B：过滤+染色目标点云（推荐）
python3 ros2_filter_node.py --ros-args -p topic_in:=/pointcloud_base_link
```

3. **RViz**：添加 `/dsvt_detections`（MarkerArray）和 `/dsvt_filtered_points`（PointCloud2，RGB8 颜色模式）

---

## 模型信息

- 检测框架：DSVT（基于 OpenPCDet）
- 当前模型配置：`DSVT/tools/cfgs/custom_models/second.yaml`
- 目标类别：`Soil`（土堆）
- 输入格式：`(N, 4)` numpy 数组，列为 `[x, y, z, intensity]`
- 输出格式：`boxes (M, 7)` 为 `[cx, cy, cz, dx, dy, dz, yaw]`
