# V12 多模态混合架构：交接与总结文档

## 1. 核心架构背景
在本项目 (V12) 中，我们将原先基于 Python (`rclpy`) 的高并发传感器读取方案彻底迁移至了 **C++ (rclcpp) + Python 混合架构**。这解决了此前 Python 全局解释器锁 (GIL) 和高频数据通信导致的网络拥塞、点云重影和卡顿问题。

- **C++ 负责底层 I/O 与密集计算**：包括 RTSP 视频流硬件解码、高频 IMU 互补滤波、点云位姿矩阵正向变换与 Z 轴伪彩色渲染。
- **Python 负责上层算法与 UI**：在 `v12_hybrid_gui.py` 中，采用纯订阅 (Subscriber-only) 模式接收处理好的数据，用于后续的高程图生成和深度学习推理。

## 2. 目录结构
架构位于：`/media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v12_multimodal_hybrid_architecture`
- `src/rtsp_camera_node.cpp`：基于 OpenCV FFMPEG/GStreamer 的 RTSP 硬件解码推流节点。
- `src/imu_sensor_node.cpp`：订阅雷达 IMU，管理 `TiltCompensator` 和 `SwingEstimator`。
- `src/tilt_compensator.cpp`：提取自 V11 的互补滤波和预积分核心算法库。
- `src/lidar_processor_node.cpp`：点云过滤、Z 轴着色以及 odom 全局位姿变换节点。
- `launch/v12_launch.py`：一键启动所有 C++ 节点及官方雷达驱动 (`pacecat_m300_driver`)。

## 3. 已解决的关键难题与技术细节

### 3.1 点云重影与卡顿问题
- **原因**：V11 中的 0.1s 缓存机制结合复杂的矩阵计算，导致点云跨帧叠加，压垮了 ROS 2 中间件。
- **解决**：在 C++ 中引入 `Eigen3`，以纳秒级速度对点云进行逐点硬件加速变换，彻底取消缓存堆叠，实现了 10Hz 丝滑的无重影单帧发布。

### 3.2 坐标系法则与“环境反向旋转”问题
- **原因**：底盘协议（向左转为负）与 ROS 2 TF 右手法则（向左转为正）的极性冲突。
- **解决**：在 `SwingEstimator` 中，严格遵循 ROS 右手法则向上传递航向角 (Yaw)。

### 3.3 “环境倾斜倒置”与坐标变换乘法逻辑
- **痛点**：在计算 `odom` 点云时，发现地面无法回正，反而向错误方向倾倒。
- **理论模型重构**：
  1. 放弃缓慢的 TF 树监听，改为 `lidar_processor_node` 直接订阅 IMU 并与 `TiltCompensator` 内存共享。
  2. 确立变换公式为：`P_odom = R_odom * P_base` (正向乘法)。
  3. 由于使用正向乘法拉平环境，`R_odom` 必须是一个**反向补偿矩阵**。
  4. 最终在 `tilt_compensator.cpp` 中加入了 `rel_roll = -rel_roll` 和 `rel_pitch = -rel_pitch`，使得生成的四元数能够完美抵消车体的物理倾斜。

### 3.4 雷达的 Z 轴伪彩色映射
- 在 C++ 点云迭代器中实现了标准的 **Jet Colormap**（蓝->绿->红），将 Z 轴高度 `[-0.4, 0.7]` 米映射到 `0-255` 的 RGB 字段，在 RViz 中提供了极佳的立体地形视觉反馈。

## 4. 下一步待办任务 (TODO for Next Session)
1. **完善 IMU 传感器节点 (`imu_sensor_node.cpp`)**：
   - 目前节点中 `external_yaw` 是通过陀螺仪 Z 轴积分模拟的。
   - **需要**：接入真实的 CAN/串口通信，读取底盘真实的 `swing_deg` 等挖掘机四维关节角数据，并替换掉目前的积分逻辑。
2. **迁移高程图生成算法**：
   - 将 V11 中的 Python 高程图 (Elevation Map) 2D 投影算法完整搬运到 `v12_hybrid_gui.py` 的回调函数中，并测试其实时性能。
3. **验证多相机的延迟**：
   - 检查目前通过 Launch 启动的 3 个 RTSP 摄像头节点的解码延迟，如有必要可在 C++ 节点中增加更底层的 GStreamer 管道字符串以进一步降低缓冲。
