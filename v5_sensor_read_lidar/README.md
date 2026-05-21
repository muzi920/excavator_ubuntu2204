# V5 雷达读取与 TF 标定 (Sensor Read LiDAR)

本目录主要包含 M300 雷达的数据读取、可视化以及与 ROS 2 TF 坐标系标定相关的工具与说明。

## 主要工作与功能

1. **雷达驱动与数据读取**
   - 包含了 `m300-main` 驱动源码，用于与 M300 雷达进行底层通信。
   - 提供 `lidar_direct_reader.py` 和 `lidar_viewer.py` 等脚本进行点云数据的读取与独立可视化测试。

2. **动态 TF 标定工具 (`tf_calibration_gui.py`)**
   - 为了解决雷达倒装及实际安装位置的偏差，开发了基于 Tkinter 的可视化标定 GUI。
   - 可以动态微调 X, Y, Z 平移以及 Yaw, Pitch, Roll 旋转（界面显示度数，底层转换弧度）。
   - **核心修复**：采用纯静态 TF (`StaticTransformBroadcaster`) 每次更新时重新发布覆盖的方式，彻底解决了 Rviz2 中由于雷达硬件时间戳与系统当前时间不同步导致的 `timestamp dropping`（时间戳过老）报错问题。
   - 提供一键导出功能，将调整好的参数保存至 `tf_calibration_record.txt`。

3. **标定记录与指南**
   - `tf_calibration_record.txt`：记录了历次标定的历史参数，最新的记录可以直接复制到 Launch 文件中作为雷达的静态 TF 参数。
   - `sensor_calibration_guide.md`：详细的传感器标定与 TF 树配置指南文档。
