# 挖掘机 V10 标定与运动学模块 (v10_calibration)

本目录用于构建挖掘机机械臂的物理运动学模型，包括正向运动学（FK）和逆向运动学（IK）。
通过这些脚本，你可以分析挖掘机的工作空间边界、将传感器读数转换为真实的 2D 平面坐标，或者将预期的挖掘目标点（X,Z）反推为 V4 控制器可以执行的 JSON 剧本参数。

## 文件结构与作用

### 1. `readme_arm.md` (核心理论文档)
- **作用**：记录了挖掘机的物理尺寸（大臂折弯等效计算）、坐标系定义、传感器极性映射规则，以及正向/逆向运动学的数学推导过程。
- **使用**：阅读该文档以了解当前模型的参数设定。
  - [x] **TODO: 测量真实物理参数并替换默认标定**
  - **大臂**：读数 `5.9` 时，$L_2$ 真实仰角 `35°` => `Offset_boom = 40.9`
  - **小臂**：读数(大臂+小臂夹角) `99.3` 时，真实仰角 `-80°` => `Offset_arm = 19.6`
  - **铲斗**：读数(三个相加) `105.6` 时，真实仰角 `-163°` (向后下) => `Offset_bucket = -56.2` (极性为负)

### 2. `kinematics.py` (正向运动学核心类)
- **作用**：提供 `ExcavatorKinematics` 类。它能够将大臂、小臂、铲斗的**绝对倾角传感器读数**，转换为铲尖在标准坐标系（回转中心地面投影为原点，X向前，Z向上）下的 `(X, Z)` 绝对坐标。
- **使用方法**：
  在你的其他控制代码中引入：
  ```python
  from kinematics import ExcavatorKinematics
  fk = ExcavatorKinematics()
  # 传入你的三个绝对倾角传感器读数
  coords = fk.forward_kinematics(sensor_boom, sensor_arm, sensor_bucket)
  print(coords['bucket_tip']) # 输出 (X, Z)
  ```
  直接运行 `python3 kinematics.py` 会执行一个简单的内部测试。

### 3. `inverse_kinematics.py` (逆向运动学核心类)
- **作用**：提供 `ExcavatorIK` 类。给定目标铲尖位置 `(X, Z)` 和期望的铲斗挖掘姿态角，**反向计算**出到达该点所需的 V4 剧本相对角度（`boom_swing`, `arm_boom`, `bucket_arm`）。
- **使用方法**：
  如果你想写一个脚本自动生成一条水平挖掘轨迹：
  ```python
  from inverse_kinematics import ExcavatorIK
  ik = ExcavatorIK()
  # 计算挖掘机前方 1.0m，地下 0.2m 处的所需角度
  result = ik.calculate_ik(target_x=1.0, target_z=-0.2, bucket_angle=-60.0)
  if result:
      print("大臂剧本参数:", result['boom_swing'])
  ```
  直接运行 `python3 inverse_kinematics.py` 会测试一个特定点是否可达，并输出 V4 参数。

### 4. `workspace_analyzer.py` (工作空间分析与绘制)
- **作用**：读取 `test3` 等实际剧本中的关节活动限位，通过正向运动学穷举所有可能的组合，计算出铲尖能够到达的所有二维坐标点，并计算极值（最大挖掘深度、最大前伸距离等）。
- **使用方法**：
  直接运行：
  ```bash
  python3 workspace_analyzer.py
  ```
  运行后，会输出最大挖掘深度等信息，并在当前目录下生成一张 `workspace_plot.png` 图片，直观展示挖掘机的有效工作区域剖面图。

### 5. `animate_trajectory.py` (JSON 剧本运动轨迹动画)
- **作用**：读取 V4 控制器录制的 JSON 剧本（如 `test3_generated_30.json`），利用正向运动学，将剧本中干瘪的角度参数转化为直观的 2D 机械臂运动动画（GIF），并画出铲尖的切削轨迹红线。
- **使用方法**：
  1. 在代码底部修改你想预览的 JSON 文件路径。
  2. 运行（支持输入相对于 `src/shandong` 根目录的路径）：
     ```bash
     python3 src/shandong/v10_cailbration/animate_trajectory.py --json v4_control_closed/test3.json
     ```
  3. 脚本会在当前目录生成与 JSON 文件同名的 GIF 动画（例如 `test3.gif`），你可以在图片查看器中打开以预览动作是否连贯、是否符合预期。

### 6. `animate_trajectory_3d.py` (3D 运动轨迹动画)
- **作用**：基于回转角（Yaw）的估算，将机械臂的二维轨迹扩展为带方向的三维空间运动。它会生成一个双视图（俯视图 + 侧视切面图）的动图，清晰地展示挖掘机是如何左右回转并下挖的。
- **回转参数基准**：当前代码内标定：在 `CH3_mv=3000` 时，`2.5s` 回转耗时约等于 `90度`（即角速度约 `36度/秒`）。
- **运行**：
  ```bash
  python3 src/shandong/v10_cailbration/animate_trajectory_3d.py --json v4_control_closed/test3.json
  ```
  运行后，会生成一个带有 `_3d.gif` 后缀的动画文件（例如 `test3_3d.gif`）。
```