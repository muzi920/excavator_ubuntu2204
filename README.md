# 挖掘机控制与多传感器采集系统

本仓库用于模型挖掘机的底层控制、时间脚本控制、倾角传感器采集、闭环角度控制、M300 雷达接入、多路摄像头读取与 ROS 2 坐标系标定。

当前仓库已经从早期的 `v1`、`v2`、`v3` 顺序试验目录，整理为按功能命名的目录结构，便于长期维护和现场调试。

## 项目概览
- `v1_control_base`：中盛 CAN 控制板底层控制与点动 GUI
- `v2_control_time_track`：基于时间的动作调度与剧本控制
- `v3_sensor_read_wit`：WIT 倾角传感器读取、串口识别与 ROS 2 发布
- `v4_control_closed`：基于倾角反馈的闭环角度控制
- `v5_sensor_read_lidar`：M300 雷达读取、可视化与 TF 标定
- `v6_sensor_read_camera`：USB/RTSP 摄像头读取与 ROS 2 图像发布
- `launch`：多传感器启动脚本与静态 TF 统一管理

## 推荐调试流程
1. 先在 `v1_control_base` 确认底盘、大臂、小臂、铲斗、回转的底层 CAN 控制正常。
2. 在 `v3_sensor_read_wit` 中完成倾角传感器串口识别、角度读取与极限工况测量。
3. 根据测得的关节活动范围，在 `v4_control_closed` 中进行闭环角度控制调试。
4. 在 `v5_sensor_read_lidar` 中完成 M300 雷达接入、倒装修正与 `map -> base_link` 标定。
5. 在 `v6_sensor_read_camera` 中接入海康或普通网络摄像头，并在 `launch/sensors_tf.launch.py` 中维护相机相对于 `base_link` 的静态外参。
6. 最后通过 `launch` 中的脚本统一启动感知链路。

## 当前目录结构

### `v1_control_base`：底层控制与实时交互
该目录直接面向中盛 ZS-USB-CAN 转接板和继电器/模拟量控制模块。

- `zs_excavator_controller.py`
  - 封装底层 13 字节协议、握手、CAN ID 编码和动作语义接口。
  - 提供底盘行走、大臂、小臂、铲斗、回转以及模拟量通道控制。
- `zs_excavator_gui.py`
  - 基于 `Tkinter` 的实时点动控制界面。
  - 支持滑条调整模拟量、键盘快捷键控制和急停。
- `zs_excavator_controller.cpp`
  - 与 Python 控制逻辑对应的 C++ 版本，便于 Windows 侧联调。

启动方式：

```bash
python3 v1_control_base/zs_excavator_gui.py
```

### `v2_control_time_track`：时间调度与动作剧本
该目录在底层控制之上增加时间维度，用于实现连续动作编排。

- `action_scheduler.py`
  - 负责动作定时执行、自动停止和多动作串联。
  - 适合编写“抬臂 1.2s -> 收斗 0.8s -> 回转 1.5s”这类时间脚本。
- `action_gui.py`
  - 图形界面版本的时间控制工具。
  - 可做单步测试，也可按预设顺序逐步执行一整套动作流程。

启动方式：

```bash
python3 v2_control_time_track/action_gui.py
```

### `v3_sensor_read_wit`：倾角传感器采集与 ROS 2 发布
该目录主要处理 WIT-Motion 系列倾角传感器的数据读取、串口识别和相对角度计算。

- `readRad_ubuntu.py`
  - Ubuntu 下的传感器读取主脚本。
  - 支持多串口轮询、根据传感器 ID 区分关节，并适配现场的固定软链接串口方案。
- `ros2_readRad_pub.py`
  - 将倾角数据发布为 ROS 2 Topic。
  - 保留原始角度/加速度数据，同时额外发布关节相对角度。
- `sensor_port_mapper_gui.py`
  - 串口识别工具，用于排查 USB 重新插拔后端口号变化的问题。
- `sensor_action_gui.py`
  - 动作控制与传感器实时监控结合的 GUI。
  - 可记录极限状态并计算各关节活动范围。

当前约定的关节传感器 ID：
- `0x50`：铲斗
- `0x51`：小臂
- `0x52`：大臂
- `0x53`：回转参考

启动方式示例：

```bash
python3 v3_sensor_read_wit/readRad_ubuntu.py
python3 v3_sensor_read_wit/ros2_readRad_pub.py
```

### `v4_control_closed`：闭环角度控制
该目录将底层控制与倾角反馈结合，实现机械臂关节的自动寻的。

- `angle_controller.py`
  - 闭环控制核心。
  - 根据目标角度和当前反馈角度决定运动方向，并在进入容差或越过目标时主动刹停。
  - 已加入提前量补偿、符号翻转防越界和不同关节独立停止指令适配。
- `closed_loop_gui.py`
  - 闭环控制调试界面。
  - 支持输入目标角度并观察各关节实时状态。

当前控制逻辑中的关键经验：
- 铲斗、小臂、大臂的相对角度控制已适配传感器安装方向反向问题。
- 回转已从倾角闭环改为按时间控制，因为当前传感器无法直接提供可靠的 Z 轴回转角反馈。

启动方式：

```bash
python3 v4_control_closed/closed_loop_gui.py
```

### `v5_sensor_read_lidar`：M300 雷达接入与 TF 标定
该目录用于 M300 雷达驱动测试、点云读取与坐标系标定。

- `m300-main`
  - 官方驱动和 SDK 源码。
- `lidar_direct_reader.py`、`lidar_viewer.py`
  - 点云读取与独立测试脚本。
- `tf_calibration_gui.py`
  - 雷达 TF 可视化标定工具。
  - 用于动态调整 `map -> base_link` 的平移和姿态参数。
- `tf_calibration_record.txt`
  - 标定记录文件，可将最终参数复制到 `launch/sensors_tf.launch.py`。
- `sensor_calibration_guide.md`
  - 雷达与多传感器坐标系整理说明。

这里有两个重要约定：
- 雷达点云的 `frame_id` 保持为 `map`，不直接改驱动内部输出。
- 为避免 Rviz2 中出现 `timestamp earlier than all the data in the transform cache`，标定和最终使用阶段以静态 TF 为主。

### `v6_sensor_read_camera`：多摄像头读取与图像发布
该目录负责 USB 摄像头、海康摄像头和普通 RTSP 网络摄像头的接入。

- `read_hikvision_cam.py`
  - 海康 RTSP 拉流测试。
- `read_network_cam.py`
  - 普通网络摄像头拉流测试。
- `read_usb_cam.py`
  - USB 摄像头测试。
- `ros2_hikvision_cam_pub.py`、`ros2_network_cam_pub.py`、`ros2_usb_cam_pub.py`
  - 单路图像 ROS 2 发布节点。
- `ros2_net_cams_pub.py`
  - 多路网络摄像头联合发布节点。
- `ros2_all_cams_pub.py`
  - 综合相机发布节点。

已处理的现场问题：
- RTSP 拉流时通过 OpenCV/FFmpeg 参数优化，降低卡死、花屏和高延迟问题。
- 在 `launch/sensors_tf.launch.py` 中维护普通网络摄像头 `.102`、`.103` 以及海康相机相对于 `base_link` 的静态外参。

注意：
- 如果希望在 Rviz2 中把图像按相机视角投影到 3D 场景，而不是只看 2D 图像窗口，除了发布 `Image` 之外，还需要同步发布 `CameraInfo`。

### `launch`：统一启动与坐标系管理
该目录是 ROS 2 启动入口，负责把雷达、相机、IMU/倾角传感器与静态 TF 整理到统一坐标树中。

- `sensors_tf.launch.py`
  - 维护 `map -> base_link -> camera_frame/...` 的静态 TF 关系。
  - 当前已包含 M300 雷达、海康摄像头、普通网络摄像头的标定参数。
- `all_sensors.launch.py`
  - 用于统一启动多传感器节点。

TF 结构约定：

```text
map
└── base_link
    ├── network_cam_frame
    ├── network_cam2_frame
    └── hikvision_cam_frame
```

## 与现场部署相关的补充说明

### 1. 串口稳定绑定
由于多个 USB 转串口设备会在拔插后造成 `/dev/ttyUSB*` 乱序，本项目已经实践过两种思路：

- 传感器脚本内部按 Modbus ID 轮询识别
- 使用 `udev` 根据物理 USB 端口拓扑绑定固定设备名

现场长期使用时，推荐优先采用 `udev` 固定软链接。

### 2. 相对角度计算
当前相对角度主要关注绕 X 轴的关节关系，典型定义为：
- 铲斗角 = 铲斗相对小臂
- 小臂角 = 小臂相对大臂
- 大臂角 = 大臂相对回转参考

这些相对值在 ROS 2 话题中已与原始角度分开发布，便于后续控制和记录。

### 3. Rviz2 显示原则
- 点云能否从 `base_link` 视角正常看到，关键在于 `map` 与 `base_link` 的 TF 是否正确连接。
- 普通图像若要参与 3D 投影，需要 `Image + CameraInfo + 正确的 camera frame TF` 三者同时存在。

## 快速启动参考

### 仅测试底层控制

```bash
python3 v1_control_base/zs_excavator_gui.py
```

### 仅测试倾角传感器

```bash
python3 v3_sensor_read_wit/readRad_ubuntu.py
```

### 启动闭环控制界面

```bash
python3 v4_control_closed/closed_loop_gui.py
```

### 启动雷达与多传感器 TF

```bash
ros2 launch shandong launch/sensors_tf.launch.py
```

## 环境依赖

常用 Python 依赖：

```bash
pip install pyserial opencv-python numpy
```

如需 ROS 2 图像桥接、TF、Rviz2 联调，还需要确保本机已安装对应的 ROS 2 发行版及常用包。

可能会用到的系统或 Python 组件包括：
- `pyserial`
- `opencv-python`
- `numpy`
- `tkinter`
- `rclpy`
- `tf2_ros`
- `sensor_msgs`
- `geometry_msgs`

## 相关补充文档
- `camera_direct_connect_ubuntu.md`：摄像头直连测试说明
- `switch_network_setup.md`：网络交换机与 IP 环境配置
- `device_network_inventory.md`：设备网络信息记录
- `readme_ubuntu.md`：Ubuntu 环境下的补充说明

## 说明
- 当前 Git 仓库根目录位于本目录，即 `shandong_ws/src/shandong`。
- 部分脚本保留了历史试验路径和演进痕迹，使用时建议优先参考各功能目录下最新的 `README.md`。
