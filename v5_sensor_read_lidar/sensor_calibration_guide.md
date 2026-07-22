# 挖掘机多传感器统一坐标系及标定说明

为了实现挖掘机的无人作业，我们需要将所有传感器（雷达、摄像头、倾角传感器）的坐标系统一到一个基准坐标系（通常为 `base_link`，即挖掘机车体中心）。

本篇文档记录了对 M300 激光雷达及多个摄像头坐标系统一的修改过程和后续微调标定方法。

## 1. 代码及参数修改记录

### 1.1 雷达参数文件修改
**修改文件**：`v5/m300-main/M300-ROS2/src/pacecat_m300_driver/params/LDS-M300-E.yaml`
**修改内容**：
- 将雷达点云输出的 `frame_id` 从默认的 `map` 修改为了 `m300_link`。
- **目的**：将雷达的坐标系与其自身硬件绑定，而不是直接绑定到全局地图，方便后续通过 TF 树与挖掘机车体（`base_link`）进行相对位置计算。

### 1.2 统一 Launch 文件修改 (静态 TF 树)
**修改文件**：`launch/all_sensors.launch.py`
**修改内容**：
在原本启动雷达、IMU、相机的基础上，加入了 ROS2 标准的 `static_transform_publisher`（静态 TF 发布器）。我们将所有传感器的坐标系统一挂载到 `base_link` 之下。

新增的 TF 关系如下：
1. **雷达 TF (`m300_link`)**：
   - 因为雷达是**倒着安装**的，所以我已经在启动参数中加入了 `roll = 180°`（即代码中的 `math.pi`，3.14159弧度）。这会让点云在 Rviz2 中自动翻转为正向。
   - 包含的倾角和其他平移量可以在代码中调整。
2. **网络摄像头 TF (`network_cam_frame`)**：发布了到 `base_link` 的 TF。
3. **海康摄像头 TF (`hikvision_cam_frame`)**：发布了到 `base_link` 的 TF。

## 2. 后续如何微调传感器的物理安装参数？

在后续的实际调试中，雷达和摄像头不可能完美安装在车体正中心（0, 0, 0），并且雷达除了倒装，可能还会存在俯仰角（pitch）和偏航角（yaw）的倾斜。

你需要通过修改 `launch/all_sensors.launch.py` 文件中的参数来进行标定。

### 调整方法
打开 `all_sensors.launch.py`，找到对应的 `arguments` 列表：
```python
arguments=['x', 'y', 'z', 'yaw', 'pitch', 'roll', '父坐标系', '子坐标系']
```

#### 雷达标定示例：
假设你的雷达安装在挖掘机中心向前 1.5 米，向上 2 米的地方，并且雷达头部向下倾斜了 10 度（约 0.174 弧度），同时是倒装的（roll 为 180 度）：
```python
lidar_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='lidar_static_tf',
    # [ x=1.5, y=0, z=2.0, yaw=0, pitch=0.174, roll=3.14159 ]
    arguments=['1.5', '0', '2.0', '0', '0.174', '3.14159', 'base_link', 'm300_link']
)
```

#### 摄像头标定示例：
假设海康摄像头安装在车体右侧 0.5 米，向上 1 米的地方：
```python
hik_cam_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='hik_cam_static_tf',
    arguments=['0', '-0.5', '1.0', '0', '0', '0', 'base_link', 'hikvision_cam_frame']
)
```

## 3. 在 Rviz2 中的可视化使用方法

完成上述配置并使用 `ros2 launch all_sensors.launch.py` 启动所有传感器后：

1. 打开 `rviz2`。
2. 在左侧的 **Global Options** -> **Fixed Frame** 中，将其手动修改为 `base_link`。
3. 点击 **Add**，添加 `PointCloud2`，并将 Topic 设置为 `/pointcloud`。你会发现倒装的雷达点云已经被正确翻转过来了。
4. 点击 **Add**，添加 `Image`，将 Topic 设置为对应的相机话题（如 `/hikvision_cam/image_raw`）。
5. 点击 **Add**，添加 `TF` 插件，你可以直观地看到 `base_link` 与各个传感器（`m300_link`, `hikvision_cam_frame` 等）之间的相对空间位置关系。
