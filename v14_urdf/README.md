# v14_urdf：URDF 模型与控制接口

这个目录提供挖掘机 URDF 模型包，用于在 ROS 2 里发布 TF、加载
`robot_description`，并通过 `/joint_states` 驱动模型做可视化验证。

除了模型包，这里还放了一套“基于 `v4_control_closed` 控制语义、但代码完全位于
`v14_urdf` 目录”的 URDF 仿真桥接脚本，用于：

- 把 v4 风格的关节目标值映射为 `/joint_states`
- 用 RViz/URDF 模型关节角替代原来的倾角传感器与雷达角度输入
- 在不修改 `v4_control_closed` 源码的前提下，做仿真功能展示

当前主要包是 `describe_60FED`，包含两类模型：

- CAD 导出模型：`describe_60FED.urdf`（搭配 meshes）
- 标定对齐模型：`describe_60FED_calibrated.urdf`（优先对齐运动学与控制语义）

<!-- prettier-ignore -->
> [!IMPORTANT]
> 如果你通过 SSH 连接远端机器且没有可用的图形显示（`DISPLAY` 为空），请使用
> `headless:=true` 启动，否则 `rviz2` 和 `joint_state_publisher_gui` 会因为
> 无法连接显示而退出。

## 构建与启动

你需要先在工作区根目录构建并 source 环境，确保 ROS 2 能找到这个包。

1. 在工作区根目录构建包：

   ```bash
   cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
   colcon build --packages-select describe_60FED
   ```

2. source 工作区环境：

   ```bash
   source install/setup.bash
   ```

3. 启动（有图形界面）：

   ```bash
   ros2 launch describe_60FED display.launch.py
   ```

4. 启动（无图形界面，适合纯 SSH）：

   ```bash
   ros2 launch describe_60FED display.launch.py headless:=true
   ```

## 启动参数

`describe_60FED/launch/display.launch.py` 支持以下参数。

- `model`：URDF 文件的绝对路径。
  - 默认：`describe_60FED_calibrated.urdf`
- `headless`：是否禁用 RViz 和 GUI 工具。
  - `true`：只启动 `robot_state_publisher` + `joint_state_publisher`
  - `false`：额外启动 `rviz2` + `joint_state_publisher_gui`
- `use_joint_state_publisher`：是否启动 `joint_state_publisher` 或 GUI 版本。
  - `true`：使用包内自带的关节滑块/默认关节发布器
  - `false`：由外部程序独占发布 `/joint_states`

示例：切换到 CAD 原始 URDF。

```bash
ros2 launch describe_60FED display.launch.py \
  model:=/media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v14_urdf/describe_60FED/urdf/describe_60FED.urdf
```

示例：给 `v14_urdf` 下的仿真 GUI 独占 `/joint_states`。

```bash
ros2 launch describe_60FED display.launch.py use_joint_state_publisher:=false
```

## 控制接口（驱动 URDF）

URDF 的“控制接口”本质是 ROS 2 标准的 `/joint_states`。只要你发布
`sensor_msgs/msg/JointState`，`robot_state_publisher` 就会计算 TF，你就能在
RViz 里看到模型动起来。

### 关节名称

以 `describe_60FED_calibrated.urdf` 为准，模型关节名如下：

- `swing_joint`：回转（yaw）
- `boom_joint`：大臂俯仰
- `arm_joint`：小臂俯仰
- `bucket_joint`：铲斗俯仰

末端参考点：

- `bucket_tip_link`：铲尖点（便于直接取 TF 做轨迹验证）

### 数据格式与单位

- 话题：`/joint_states`
- 消息类型：`sensor_msgs/msg/JointState`
- `position` 单位：弧度（rad）

如果你的控制程序使用角度（deg），需要先转换为弧度：

```text
rad = deg * pi / 180
```

### 最小发布示例

下面示例会把 4 个关节设到某个固定角度（弧度制）。你可以在 RViz 里添加
**RobotModel** + **TF** 观察变化。

```bash
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "{
  header: {frame_id: ''},
  name: ['swing_joint','boom_joint','arm_joint','bucket_joint'],
  position: [0.0, 0.2, 0.5, -0.3]
}"
```

如果你希望持续发布（例如 30 Hz），建议写一个小节点定时发布，而不是用命令行
循环。

## v4 仿真桥接脚本

如果你想保留 `v4_control_closed` 的关节语义和 GUI 交互方式，但把真实传感器输入
替换成 URDF 模型关节角，那么直接使用本目录下的三个脚本：

- `ros_joint_bridge.py`：收发 `/joint_states`
- `sim_angle_controller.py`：仿真版角度控制器
- `v4_urdf_sim_gui.py`：仿真版 GUI

推荐按下面两步启动。

1. 启动 URDF 与 RViz，但关闭 `joint_state_publisher`：

   ```bash
   cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
   source install/setup.bash
   ros2 launch describe_60FED display.launch.py use_joint_state_publisher:=false
   ```

2. 在另一个终端启动仿真 GUI：

   ```bash
   cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
   source /opt/ros/humble/setup.bash
   python3 src/shandong/v14_urdf/v4_urdf_sim_gui.py
   ```

这时：

- GUI 上输入的 `boom_swing / arm_boom / bucket_arm / swing_yaw` 目标值会发布到
  `/joint_states`
- GUI 显示的“当前角度”会从 RViz/URDF 仿真模型当前关节角反算得到
- 原来的倾角传感器和雷达 IMU 角度来源不再参与这条仿真链路

<!-- prettier-ignore -->
> [!WARNING]
> 如果你当前终端使用的是 Conda 或其他非系统 Python，直接运行
> `python3 src/shandong/v14_urdf/v4_urdf_sim_gui.py` 很可能因为 ROS 2 Humble 的
> `rclpy` 与 Python 版本不匹配而失败。运行本目录下的 ROS 2 Python 脚本时，统一
> 使用：
>
> `source /opt/ros/humble/setup.bash && /usr/bin/python3 <script>`

## 推荐的终端步进调试

当前这套仿真链路已经支持“终端步进执行”，适合你一边看 RViz，一边逐步验证标准
挖掘流程。相比 Tk GUI，这个方式更适合远端 SSH、无桌面、或图形环境不稳定的场景。

入口脚本是 `terminal_stepper.py`。它会加载当前 JSON 剧本，并提供一个 REPL 风格的
交互界面。

```bash
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
source /opt/ros/humble/setup.bash
/usr/bin/python3 src/shandong/v14_urdf/terminal_stepper.py
```

默认会加载：

- `src/shandong/v14_urdf/json/generated_dig_dump_trajectory.json`

如果你想加载别的脚本，直接把路径作为参数传入：

```bash
/usr/bin/python3 src/shandong/v14_urdf/terminal_stepper.py \
  src/shandong/v14_urdf/json/generated_dig_dump_trajectory.json
```

这个终端步进器支持以下命令：

- `help`：显示帮助
- `list`：列出全部步骤
- `show`：显示当前选中步骤
- `angles`：显示当前 `/joint_states` 角度
- `next`：选择下一步并执行
- `prev`：选择上一步并执行
- `goto N`：选中第 `N` 步，不执行
- `run`：执行当前选中步骤
- `runall`：从当前选中步骤开始执行到末尾
- `reload`：重新读取 JSON 文件
- `reset`：把选中步骤重置到第 1 步
- `quit`：退出

推荐你按下面的顺序手动验证：

1. 执行 `list`，确认当前步骤顺序。
2. 使用 `goto 1`、`run`、`angles` 检查初始动作。
3. 继续逐步执行挖掘阶段：
   - `goto 2`
   - `run`
   - `goto 3`
   - `run`
   - `goto 4`
   - `run`
   - `goto 5`
   - `run`
4. 在 RViz 里观察姿态是否符合预期，再决定是否修改 JSON 或轨迹生成器。

## 当前阶段总结

截至目前，`v14_urdf` 已经从“仅加载 URDF 的展示目录”扩展为一套可用于功能演示
的仿真验证目录。你现在可以直接在这一目录下完成模型加载、关节驱动、JSON 剧本
回放，以及 RViz 中的状态观察，而不需要回改 `v4_control_closed` 的源码。

- 已完成 `describe_60FED_calibrated.urdf` 标定版模型，关节连接方式与
  `v10_cailbration_arm` 的机械参数保持一致
- 已完成 `display.launch.py` 的 `headless` 模式，适配纯 SSH 环境
- 已完成 `use_joint_state_publisher:=false` 开关，支持由外部程序独占发布
  `/joint_states`
- 已完成 `ros_joint_bridge.py`，实现 URDF 关节名与 `v4` 风格关节语义之间的转换
- 已完成 `sim_angle_controller.py` 与 `v4_urdf_sim_gui.py`，实现仿真版 GUI 和
  关节目标控制
- 已完成 `terminal_stepper.py`，支持纯终端逐步执行 JSON 步骤，便于人工校正动作
- 已完成 JSON 剧本回放，支持把历史录制的 `v4` 剧本直接映射为 URDF 关节运动
- 已完成回放进度、剩余时间、完成状态显示，避免误判“程序已经结束但模型还在动”
- 已完成 `point_to_dig_dump_trajectory.py`，支持从单个挖掘点和单个卸料点生成一份
  可回放的挖掘到卸料轨迹
- 已确认 ROS 2 Python 节点必须使用系统 Python，也就是 `/usr/bin/python3`

## 单点逆解与轨迹生成

当前目录已经不只是“能回放已有 JSON”，也已经具备“从单个挖掘点和卸料点生成轨迹”
的能力。入口脚本是 `point_to_dig_dump_trajectory.py`。

它接收一个挖掘点和一个卸料点，自动完成：

- 三维点到回转角的转换
- 点到二维工作平面 `(r, z)` 的投影
- 挖掘姿态与卸料姿态的可达解搜索
- 单次“挖掘 -> 卸料”流程的阶段化 JSON 生成

示例：

```bash
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
python3 src/shandong/v14_urdf/point_to_dig_dump_trajectory.py \
  --dig-x 1.3027558326721191 \
  --dig-y -0.09856557846069336 \
  --dig-z 0.0009403228759765625 \
  --dump-x -0.055274009704589844 \
  --dump-y -1.1006269454956055 \
  --dump-z -0.001651763916015625
```

生成文件默认输出到：

- `src/shandong/v14_urdf/json/generated_dig_dump_trajectory.json`

### 当前单点流程语义

当前生成器已经按本项目确认过的关节语义做了几次修正，主要规则如下：

- `bucket_arm = 0`：铲斗接近闭合
- `bucket_arm ≈ -90`：铲斗接近完全打开
- 挖掘阶段：
  - 先半开斗下探
  - 再执行收斗取料
- 卸料阶段：
  - 回转到卸料点
  - 大臂、小臂、铲斗到卸料预备位
  - 最后开斗到接近 `-90`

当前这份生成器输出的是“标准化演示轨迹”，不是严格的动力学最优控制。它更偏向
工程调试和流程确认，适合先把动作语义调顺，再继续做多点规划。

### 当前生成脚本的几何定义

当前单点规划里，两个点的语义不是完全一样：

- 挖掘点：按铲尖切入点理解
- 卸料点：按“小臂与铲斗连接点附近的目标点”理解

这样做是为了让卸料姿态更符合你在 RViz 里观察到的真实要求，也就是：

- 小臂需要外推
- 铲斗需要外推到预备位
- 然后再开斗卸料

## 从点云目标到逆解控制

当前阶段的重点不再只是“回放已有动作”，而是从环境感知结果中直接生成动作。目标
是：给定两个点云坐标，分别作为挖掘点和卸料点，自动求解机械臂应如何运动，并把
这个过程转换为可执行的关节角序列。

这一步的输入和输出建议统一为下面的形式。

- 输入 1：挖掘点坐标 `(x, y, z)`
- 输入 2：卸料点坐标 `(x, y, z)`
- 输入 3：可选的姿态约束
  - 挖掘姿态角，例如铲斗切土角
  - 卸料姿态角，例如抬臂后的倒料角
- 输出：一组关节目标序列
  - `swing_yaw`
  - `boom_swing`
  - `arm_boom`
  - `bucket_arm`

从几何上看，这个过程可以拆成“回转求解”和“平面逆解”两部分。当前仓库已经有可
复用的平面逆解基础，见 [inverse_kinematics.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v10_cailbration_arm/inverse_kinematics.py)。

建议采用下面的求解流程。

1. 统一坐标系。先把点云中的 `(x, y, z)` 转换到“回转中心地面投影点”为原点的
   挖掘机工作坐标系。
2. 求回转角。根据目标点的平面投影计算：

   ```text
   swing_yaw = atan2(y, x)
   ```

3. 投影到机械臂工作平面。设：

   ```text
   r = sqrt(x^2 + y^2)
   ```

   然后把三维问题转成当前已支持的二维逆解问题 `(r, z)`。
4. 给定末端姿态约束。因为仅靠 `(r, z)` 不能唯一确定铲斗姿态，所以必须给出
   挖掘点和卸料点的末端姿态角。
5. 调用平面逆解。使用现有的 `ExcavatorIK.calculate_ik(target_x, target_z,
   bucket_angle_deg)` 求出：
   - `boom_swing`
   - `arm_boom`
   - `bucket_arm`
6. 生成过程轨迹。不要只算两个终点，还要插入中间动作阶段，例如：
   - 初始姿态
   - 对准挖掘点
   - 切入和收斗
   - 抬臂离坑
   - 回转到卸料点
   - 卸料
   - 回到下一次挖掘准备姿态

<!-- prettier-ignore -->
> [!IMPORTANT]
> 两个点的 `(x, y, z)` 只能确定“去哪里”，还不能唯一确定“铲斗以什么姿态到达”。
> 所以下一阶段实现时，除了挖掘点和卸料点坐标，你还需要同时定义挖掘姿态角和卸料
> 姿态角，否则逆解会出现多解或解不稳定的问题。

## 多挖掘点循环方案

当前单点流程已经能跑通“挖掘 -> 卸料”的完整闭环。下一步要解决的是：真实作业时，
一次挖掘通常不够，系统需要连续处理多个挖掘点，并在所有点完成后再执行一次最终归位。

这里最重要的设计原则是把“初始化”、“循环内部过渡”和“最终归位”拆开。

- 初始化：只执行一次
- 挖掘循环：执行多次，每次对应一个挖掘点
- 归位：只执行一次

初始化和归位都不参与挖掘循环本身。这样可以避免每挖一次都回到初始位，导致动作冗余、
节奏不自然，也更符合现场作业逻辑。

推荐把完整任务拆成三段：

1. `init_segment`
   - 开机后的标准准备位
   - 回转归零或到任务起始朝向
   - 大臂、小臂、铲斗到待机姿态
2. `cycle_segment`
   - 对每个挖掘点执行一次单点挖掘流程
   - 每次卸料后回到一个“循环内部过渡位”
   - 然后转入下一个挖掘点
3. `home_segment`
   - 所有挖掘点完成后，回到标准归位姿态

推荐的数据结构如下：

```json
{
  "task_name": "multi_dig_demo",
  "init_pose": {
    "swing_yaw": 0.0,
    "boom_swing": 10.0,
    "arm_boom": 20.0,
    "bucket_arm": -30.0
  },
  "cycle_transit_pose": {
    "boom_swing": 25.0,
    "arm_boom": 55.0,
    "bucket_arm": -10.0
  },
  "home_pose": {
    "swing_yaw": 0.0,
    "boom_swing": 5.0,
    "arm_boom": 10.0,
    "bucket_arm": -80.0
  },
  "dump_point": {
    "x": -0.05,
    "y": -1.10,
    "z": 0.0
  },
  "dig_points": [
    {"x": 1.30, "y": -0.10, "z": 0.00},
    {"x": 1.24, "y": -0.05, "z": -0.02},
    {"x": 1.18, "y": 0.02, "z": -0.03}
  ]
}
```

在实现层面，建议保留 `point_to_dig_dump_trajectory.py` 作为“单点规划器”，再新增一
个上层任务规划器，把多个单点结果拼接起来：

- `init_segment`
- `point_1_cycle`
- `point_2_cycle`
- `point_3_cycle`
- `home_segment`

这样做的好处是：

- 单点逻辑和多点调度逻辑分离
- 后续你可以继续用终端步进器逐轮检查
- 多挖掘点扩展不会破坏当前已调通的单点流程

## 常见问题

### 启动时提示无法连接显示

- `qt.qpa.xcb: could not connect to display`
- `Can't open display`

说明当前终端没有图形显示环境。使用：

```bash
ros2 launch describe_60FED display.launch.py headless:=true
```

或者改用带 X11 转发的终端，或使用远程桌面/VNC。

### 使用 `python3` 运行脚本时提示 `rclpy` 找不到

如果你看到类似错误：

- `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`

通常说明你当前使用的是 Conda Python，而 ROS 2 Humble 的 `rclpy` 是按系统
Python 3.10 编译的。请改用：

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 src/shandong/v14_urdf/terminal_stepper.py
```

### KDL 警告：root link 带 inertia

你可能会看到警告：

`The root link base_link has an inertia specified in the URDF...`

这是 KDL 的限制提示，不影响 TF 发布和 RViz 可视化。如果你需要消除该警告，
可以在 URDF 里增加一个 dummy root link（后续可以再做一次模型整理）。

## Next steps

- 把当前单点流程继续通过终端步进器逐步校正，先固化一份标准挖掘模板。
- 将 `point_to_dig_dump_trajectory.py` 上提为“单点轨迹生成器”，保持其接口稳定。
- 新增多挖掘点任务规划器，拆分 `init_segment`、`cycle_segment` 和 `home_segment`。
- 将初始化与归位彻底从循环中剥离，只在任务开始和结束时执行一次。
- 为多点循环增加“循环内部过渡位”，避免每轮重复回到初始姿态。
