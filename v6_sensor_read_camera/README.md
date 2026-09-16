# V6 摄像头读取与发布 (Sensor Read Camera)

本目录主要包含挖掘机多路摄像头（USB 摄像头、海康网络摄像头、普通 RTSP 网络摄像头）的读取、测试以及 ROS 2 图像消息发布节点。

## 主要工作与功能

1. **单路相机测试脚本**
   - `read_usb_cam.py`: 本地 USB 摄像头读取测试。
   - `read_hikvision_cam.py`: 海康威视网络摄像头 RTSP 拉流测试。
   - `read_network_cam.py`: 普通网络摄像头 RTSP 拉流测试。
   - **流媒体优化**：解决了拉流卡死和花屏问题，通过配置 OpenCV 的 FFmpeg 环境变量（如 `rtsp_transport;udp|stimeout;3000000`）显著降低了网络延迟并增强了稳定性。

2. **ROS 2 图像发布节点**
   - `ros2_usb_cam_pub.py` / `ros2_hikvision_cam_pub.py` / `ros2_network_cam_pub.py`: 将各路摄像头画面转换为 ROS 2 标准的 `sensor_msgs/msg/Image` 消息并发布到指定 Topic。
   - `ros2_net_cams_pub.py`: 整合发布多路网络摄像头数据的脚本。
   - `ros2_all_cams_pub.py`: 综合发布所有可用摄像头（含 USB 和网络相机）的 ROS 2 节点。

3. **快照与说明**
   - 目录中包含测试时抓取的画面快照（如 `hikvision_cam_snapshot.jpg`，`network_cam_snapshot.jpg` 等），用于验证图像读取的正确性。
   - `readme_shuoming.md`: 记录了一些相关的说明信息。
