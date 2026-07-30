# DSVT 在线点云识别与 ROS 2 桥接模块

本目录包含将 DSVT (Dynamic Sparse Voxel Transformer) 模型从离线推理改造为 **在线实时识别**，并无缝接入 ROS 2 系统的核心代码。

为了解决 ROS 2 依赖系统级 Python 3.10 环境，而深度学习模型依赖 Conda Python 3.11 环境（并存在底层的 C++ 库版本冲突）的问题，我们采用了 **进程间通信 (IPC) 架构**。即在 Conda 中将模型封装为 HTTP API 服务，在系统环境中使用 ROS 2 节点订阅点云并通过 API 实时获取推理结果。

---

## 目录结构与脚本说明

| 文件名 | 环境要求 | 作用说明 |
| :--- | :--- | :--- |
| `online_detector.py` | **Conda 环境** | **核心检测器类**。负责在初始化时一次性将配置文件和权重加载到 GPU 中，并提供 `inference(points)` 方法供外部直接传入 Numpy Array 进行实时推理。 |
| `api_server.py` | **Conda 环境** | **API 后端服务**。基于 FastAPI 编写，封装了 `OnlineDetector`。启动后在 `localhost:8000/predict` 提供 HTTP POST 接口，接收内存二进制点云数据，毫秒级返回 3D 框的 JSON 数据。 |
| `folder_watcher.py` | **Conda 环境** | *(备用)* **离线流式模拟器**。持续监控指定文件夹，一旦有新的 `.npy` 或 `.bin` 文件落盘，立刻进行推理并打印结果，适合脱离 ROS 的纯算法测试。 |
| `ros2_bridge_node.py` | **系统 ROS 2 环境** | **轻量级 ROS 2 桥接节点**。订阅传感器点云话题，将点云转为字节流发送给 API 服务，收到结果后在 RViz 中发布绿色的 3D 检测框 (MarkerArray)。 |
| `ros2_filter_node.py` | **系统 ROS 2 环境** | **高级 ROS 2 过滤节点**。除了具备桥接节点的功能外，还会**剔除高度 $z < 0$ 的背景点**，精确过滤出只位于 3D 检测框内部的目标点云，**将其染成红色**，并发布为一个全新的点云话题。 |

---

## 快速启动指南

要完整运行这套在线识别系统，你需要开启 **三个终端**，并严格遵守每个终端的环境要求。

### 终端 1：启动底层深度学习 API 服务
该终端**必须**使用 Conda 虚拟环境，以确保 PyTorch/CUDA 能正常工作。

```bash
# 1. 激活 Conda 环境
conda activate dsvt

# 2. 启动 API 服务
cd /media/libo/libo_sn7100/ubuntu2204/PointCloud_ws/src/Point_recognition/online
python api_server.py \
    --cfg_file ../DSVT/tools/cfgs/custom_models/second.yaml \
    --ckpt ../DSVT/output/cfgs/custom_models/second/default/ckpt/checkpoint_epoch_79.pth
```
*成功启动后，终端会提示 `Uvicorn running on http://0.0.0.0:8000`。*

---

### 终端 2：启动 ROS 2 业务节点 (选择其一)
该终端**千万不要**激活 Conda，必须使用系统原生的 Python 3.10 环境。

```bash
# 1. 确保退出 Conda 环境
conda deactivate  # 如果之前激活了的话

# 2. Source ROS 2 Humble 环境变量
source /opt/ros/humble/setup.bash

# 3. 运行节点
cd /media/libo/libo_sn7100/ubuntu2204/PointCloud_ws/src/Point_recognition/online
```

**选择 A：只看 3D 框 (资源消耗极低)**
```bash
python3 ros2_bridge_node.py --ros-args -p topic:=/pointcloud_base_link
```

**选择 B：过滤并高亮目标点云 (推荐，带点云截取与红色渲染功能)**
```bash
python3 ros2_filter_node.py --ros-args -p topic_in:=/pointcloud_base_link
```
*(注意：请根据你实际的数据集，修改上述命令中 `topic:=` 后面的话题名称以完成对齐。)*

---

### 终端 3：播放数据集与 RViz 可视化
同样在系统环境中操作。

```bash
source /opt/ros/humble/setup.bash

# 播放你的点云数据包
ros2 bag play /media/libo/libo_sn7100/cy
```

**在 RViz2 中的配置方法：**
1. 启动 `rviz2`。
2. 将 **Fixed Frame** 设置为你的点云 Frame ID (例如 `base_link` 或 `lidar`)。
3. **查看原始点云**：添加 `PointCloud2`，订阅你播放的原始话题（如 `/pointcloud_base_link`），建议将其颜色设置为灰色 (FlatColor) 或调低透明度。
4. **查看 3D 检测框**：添加 `MarkerArray`，订阅 `/dsvt_detections` 话题。
5. **查看目标红色点云 (仅限运行了 filter_node)**：添加一个新的 `PointCloud2`，订阅 **`/dsvt_filtered_points`** 话题，并在设置中将 **Color Transformer** 修改为 **`RGB8`**，即可看到被提取出来的红色目标点。

---

## Topic 对齐与参数配置

如果你在其他机器人或项目上使用，可以通过 ROS 2 传参机制轻松对齐 Topic，无需修改代码：

对于 `ros2_bridge_node.py`：
- `topic`：输入的原始点云话题 (默认: `/pointcloud_base_link`)
- `api_url`：API 服务地址 (默认: `http://127.0.0.1:8000/predict`)

对于 `ros2_filter_node.py`：
- `topic_in`：输入的原始点云话题 (默认: `/pointcloud_base_link`)
- `topic_out`：输出的截取后目标点云话题 (默认: `/dsvt_filtered_points`)
- `api_url`：API 服务地址 (默认: `http://127.0.0.1:8000/predict`)

**示例：适配新的雷达话题**
```bash
python3 ros2_filter_node.py --ros-args -p topic_in:=/velodyne_points -p topic_out:=/my_filtered_points
```