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

## launch 文件实际启动了什么

如果你后续要把 `v14_urdf` 作为统一控制接口基准，首先要清楚
`describe_60FED/launch/display.launch.py` 到底拉起了哪些 ROS 节点，以及这些节点之间是怎么
联动的。

这份 launch 文件当前会按参数组合启动以下节点：

1. `robot_state_publisher`
2. `joint_state_publisher_gui`
3. `joint_state_publisher`
4. `rviz2`

其中真正一定会参与模型联动的是：

- `robot_state_publisher`

其余几个节点是否参与，取决于 `headless` 和 `use_joint_state_publisher` 的取值。

### `robot_state_publisher`

这是最核心的节点。它做的事情是：

1. 读取 `robot_description`
2. 解析 URDF 里的连杆和关节结构
3. 订阅 `/joint_states`
4. 根据当前关节角计算整棵 TF 树

可以把它理解为：

```text
URDF 几何模型 + /joint_states
-> TF
```

也就是说，`v14` 里真正“驱动 URDF 动起来”的不是 RViz，也不是 GUI，而是：

```text
/joint_states -> robot_state_publisher -> TF
```

### `joint_state_publisher_gui`

当：

- `headless:=false`
- `use_joint_state_publisher:=true`

时，launch 会启动 `joint_state_publisher_gui`。

它的作用是：

- 提供一个带滑块的 GUI
- 让你手动拖动四个关节
- 直接发布 `/joint_states`

这个模式适合：

- 快速检查 URDF 关节方向
- 验证模型是否能正确运动
- 不接入任何外部控制脚本时做最小联调

### `joint_state_publisher`

当：

- `headless:=true`
- `use_joint_state_publisher:=true`

时，launch 会启动无 GUI 版本的 `joint_state_publisher`。

它的作用是：

- 在没有图形界面的情况下仍然发布默认 `/joint_states`
- 让 `robot_state_publisher` 有输入可用

这个模式适合：

- 纯 SSH 环境
- 只想先确认 URDF 能正常启动

### `rviz2`

当：

- `headless:=false`

时，launch 会启动 `rviz2` 并加载包内默认配置。

它的作用不是计算模型状态，而是：

- 订阅 TF
- 订阅 `robot_description`
- 在界面上显示模型

也就是说：

```text
RViz 只负责显示，不负责控制
```

## URDF 与 ROS 的联动原理

这一节把“模型为什么会动”写清楚。后续无论你是接 `v4`、接 JSON 剧本，还是接新规划
器，本质上都绕不过这一条链路。

### 最核心的数据流

`v14_urdf` 当前的模型联动主链路是：

```text
控制目标（deg）
-> 转换为 JointState.position（rad）
-> 发布到 /joint_states
-> robot_state_publisher 计算 TF
-> RViz 显示模型姿态
```

只要这条链路是通的，URDF 模型就能动。

### 为什么是 `/joint_states`

ROS 里对关节型机器人最标准的状态输入就是：

- 话题：`/joint_states`
- 消息：`sensor_msgs/msg/JointState`

`robot_state_publisher` 并不关心你的上层控制逻辑来自哪里。它只关心两件事：

1. URDF 里有哪些关节
2. `/joint_states` 当前给这些关节发布了什么角度

所以对 `v14` 来说，所谓“控制 URDF”，本质上就是：

```text
正确发布 /joint_states
```

### 为什么上层还保留 v4 风格的角度语义

虽然 URDF 标准关节名是：

- `swing_joint`
- `boom_joint`
- `arm_joint`
- `bucket_joint`

但工程里早就已经形成了 `v4` 风格控制语义：

- `swing_yaw`
- `boom_swing`
- `arm_boom`
- `bucket_arm`

因此 `v14` 并没有强行让上层规划器或脚本直接使用 URDF 原始关节名，而是保留了一层
工程语义映射。这样做的好处是：

1. 和 `v4` 闭环控制逻辑一致
2. 和 `v10` 逆解/轨迹生成输出一致
3. 更符合你当前实机控制习惯

## v14 的 ROS 控制接口是怎么定义的

这一节把 `v14` 当前真正用于控制的接口明确列出来，后续统一接口时也应该以这里为准。

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

### 常用检查命令

如果你怀疑模型没动、话题没通、或者桥接没生效，可以直接用下面这些 ROS 命令检查。

先看 `/joint_states` 是否存在：

```bash
ros2 topic list | grep joint_states
```

再看当前是否有数据：

```bash
ros2 topic echo /joint_states
```

看当前是否是你期望的发布者在发：

```bash
ros2 topic info /joint_states
```

如果你要确认 TF 是否正常生成：

```bash
ros2 topic list | grep tf
ros2 topic echo /tf
```

如果你要看当前所有节点：

```bash
ros2 node list
```

## `ros_joint_bridge.py` 是怎么联动的

`ros_joint_bridge.py` 是 `v14` 当前最关键的桥接层。它的作用不是做复杂控制，而是做两件
事：

1. 订阅 `/joint_states`
2. 以 `v4` 风格关节语义发布新的 `/joint_states`

换句话说，它是：

```text
v4 风格角度语义 <-> URDF 标准关节消息
```

### 读取方向

当桥接器收到 `/joint_states` 后，会把：

- `swing_joint`
- `boom_joint`
- `arm_joint`
- `bucket_joint`

转换回角度制，并映射为：

- `swing_yaw`
- `boom_swing`
- `arm_boom`
- `bucket_arm`

所以 GUI、终端步进器、回放器都可以继续使用 `v4` 关节语义读取当前状态。

### 发布方向

当上层调用：

```python
publish_v4_targets_deg(
    swing_yaw=...,
    boom_swing=...,
    arm_boom=...,
    bucket_arm=...,
)
```

桥接器会做四件事：

1. 保存最近一次命令角度
2. 把角度从 `deg` 转成 `rad`
3. 组装 `JointState`
4. 发布到 `/joint_states`

也就是说，`ros_joint_bridge.py` 是当前 `v14` 中最接近“统一控制接口适配器”的那个文件。

## `sim_angle_controller.py` 是怎么控制的

`sim_angle_controller.py` 的定位不是硬件控制器，而是：

```text
仿真版角度控制器
```

它沿用 `v4` 的关节语义和调用方式，但最终不下发真实硬件，而是调用
`RosJointBridge.publish_v4_targets_deg()`。

### 当前控制方式

当前 `move_joint_to_angle()` 的行为可以概括为：

1. 接收 `joint_name + target_angle`
2. 先按内部限位做裁剪
3. 把目标角度整理成 `v4` 风格 payload
4. 调桥接器发布到 `/joint_states`

也就是说，当前仿真控制不是“连续轨迹控制”，而是：

```text
把目标角度直接写到 JointState
```

对 RViz 来说，这已经足够看到姿态变化。

### 为什么还保留 `tolerance/ch1/ch2/ch3/ramp`

虽然在仿真模式里：

- `tolerance`
- `ch1_mv`
- `ch2_mv`
- `ch3_mv`
- `ramp_up_s`
- `ramp_down_s`

并不会真正驱动液压和比例阀，但这些字段仍然保留，是为了：

1. 和 `v4` 旧接口保持一致
2. 让 JSON 剧本格式不需要改
3. 方便以后切回真实控制后端

## `v4_urdf_sim_gui.py` 是怎么联动的

`v4_urdf_sim_gui.py` 是一个仿真版 GUI，它把三层东西串起来了：

1. `RosJointBridge`
2. `SimAngleController`
3. `JsonScriptReplayer`

因此它既能：

- 手动下发单个关节角度

也能：

- 加载和执行 JSON 剧本

### 手动控制链路

GUI 里当你点击某个关节动作时，链路是：

```text
Tk 输入目标角度
-> SimAngleController.move_joint_to_angle()
-> RosJointBridge.publish_v4_targets_deg()
-> /joint_states
-> robot_state_publisher
-> TF
-> RViz 模型变化
```

### 状态回读链路

GUI 里显示“当前角度”时，链路是：

```text
/joint_states
-> RosJointBridge.get_v4_angles_from_joint_states_deg()
-> GUI 标签显示
```

所以这个 GUI 的“当前角度”并不是本地假想值，而是从 ROS 话题真实回读的。

## JSON 剧本是怎么控制 URDF 的

这一节是很多后续规划脚本都会依赖的关键逻辑。

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

## `replay_json_script.py` 的控制流程

如果你不是想手动点 GUI，而是想自动执行一个任务 JSON，那么入口就是：

- `replay_json_script.py`

它内部主要做了三件事：

1. 创建 `RosJointBridge`
2. 创建 `JsonScriptReplayer`
3. 把 JSON 剧本中的每一步顺序发布到 `/joint_states`

### JSON 剧本格式

当前支持两种根格式：

1. 顶层直接是数组
2. 顶层是对象，并包含：
   - `metadata`
   - `script`

其中真正执行的是：

- `script[]`

每个步骤至少关心：

- `joint`
- `target_val`
- `description`
- `ramp_up_s`
- `ramp_down_s`

### 非 feedback 模式

如果你直接运行：

```bash
/usr/bin/python3 src/shandong/v14_urdf/replay_json_script.py path/to/script.json
```

那脚本会按预估持续时间进行插值发布：

```text
当前角度 -> 目标角度
```

这更适合：

- 快速演示
- 不强调严格到位判断的仿真播放

### feedback 模式

如果你加上：

```bash
--feedback
```

那执行逻辑会变成：

1. 读取当前 `/joint_states`
2. 计算当前关节与目标角的误差
3. 在误差进入容差之前，不切到下一步
4. 到位后，再执行下一步

这更适合：

- 手动观察动作是否到位
- 后续和实机闭环逻辑保持一致

### `feedback-publish-mode` 的区别

当前有两种模式：

1. `interpolate`
2. `target`

`interpolate` 的逻辑是：

- 以固定速度逐渐逼近目标角
- 每一帧都重新发一个更接近目标的角度

`target` 的逻辑是：

- 直接反复发布目标角
- 等 `/joint_states` 回读进入容差

前者更适合仿真展示，后者更接近真实硬件闭环等待。

### 推荐回放命令

当前最常用的回放方式是：

```bash
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
source /opt/ros/humble/setup.bash
/usr/bin/python3 src/shandong/v14_urdf/replay_json_script.py \
  src/shandong/v14_urdf/json/generated_dig_dump_trajectory.json \
  --feedback \
  --feedback-publish-mode interpolate \
  --joint-speed-deg-s 6 \
  --swing-speed-deg-s 15 \
  --min-step-s 1.0 \
  --dwell-s 0.4
```

## 终端步进器是怎么控制的

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

### 内部控制方式

`terminal_stepper.py` 内部并没有额外定义新的控制协议，而是仍然复用：

- `RosJointBridge`
- `SimAngleController`

所以它的链路其实和 GUI 一样，只是输入方式变成了终端命令：

```text
终端命令
-> TerminalStepper.execute_step()
-> SimAngleController.move_joint_to_angle()
-> RosJointBridge.publish_v4_targets_deg()
-> /joint_states
-> TF
-> RViz
```

## v14 当前的完整联动关系

如果把 `v14` 当前已经具备的全部链路压缩成一张图，可以写成：

```text
单点/多点规划脚本
        |
        v
  生成 JSON 剧本
        |
        +-------------------+
        |                   |
        v                   v
  replay_json_script   terminal_stepper / GUI
        |                   |
        +---------> SimAngleController
                            |
                            v
                     RosJointBridge
                            |
                            v
                      /joint_states
                            |
                            v
                 robot_state_publisher
                            |
                            v
                           TF
                            |
                            v
                          RViz
```

也就是说，当前 `v14` 已经形成了一个完整闭环：

```text
规划 -> 脚本 -> 控制接口 -> URDF -> 可视化验证
```

## 推荐的实际操作流程

如果你现在要用 `v14` 做一次完整联调，推荐严格按下面顺序执行。

### 方式一：只验证 URDF 和 ROS 联动

1. 启动模型：

   ```bash
   cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
   source install/setup.bash
   ros2 launch describe_60FED display.launch.py use_joint_state_publisher:=false
   ```

2. 手工发布一个 `JointState`：

   ```bash
   ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "{
     header: {frame_id: ''},
     name: ['swing_joint','boom_joint','arm_joint','bucket_joint'],
     position: [0.0, 0.2, 0.5, -0.3]
   }"
   ```

3. 在 RViz 看模型是否变化。

### 方式二：验证 v4 风格语义到 URDF 的桥接

1. 启动模型并关闭默认关节发布器：

   ```bash
   ros2 launch describe_60FED display.launch.py use_joint_state_publisher:=false
   ```

2. 启动仿真 GUI：

   ```bash
   source /opt/ros/humble/setup.bash
   /usr/bin/python3 src/shandong/v14_urdf/v4_urdf_sim_gui.py
   ```

3. 在 GUI 中输入：
   - `boom_swing`
   - `arm_boom`
   - `bucket_arm`
   - `swing_yaw`

4. 在 RViz 观察模型运动。

### 方式三：验证 JSON 剧本自动控制

1. 启动模型：

   ```bash
   ros2 launch describe_60FED display.launch.py use_joint_state_publisher:=false
   ```

2. 执行剧本回放：

   ```bash
   source /opt/ros/humble/setup.bash
   /usr/bin/python3 src/shandong/v14_urdf/replay_json_script.py \
     src/shandong/v14_urdf/json/generated_dig_dump_trajectory.json \
     --feedback \
     --feedback-publish-mode interpolate
   ```

3. 在 RViz 观察每一步动作。

## 对后续统一控制接口的意义

如果你后续真的要把整个项目的控制接口统一到 `v14`，那么这个 README 当前已经明确了三件
最重要的事：

1. 统一上层关节语义是：
   - `swing_yaw`
   - `boom_swing`
   - `arm_boom`
   - `bucket_arm`
2. 统一 ROS 执行接口是：
   - `/joint_states`
   - `sensor_msgs/msg/JointState`
3. 统一仿真验证后端是：
   - `robot_state_publisher + TF + RViz`

也就是说，后续无论你从 `v4`、`v13` 还是未来新的 C++ 传感器与控制节点接入，只要最后
能正确落到 `/joint_states`，`v14` 这一层就仍然成立。

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

### Mode1 收口结果

`v14_urdf/mode1/` 在这一阶段已经完成了：

- 工作区域约束采样与 JSON 导出
- 真实点云独立处理链路
- 工作区域三维点云生成
- 点云辅助的候选挖掘点筛选
- 小规模多点任务生成与 RViz 回放验证

当前建议把 `mode1` 视为“阶段性收口完成”，后续继续启动时直接从固定保留资产开始，
不要再依赖早期试验输出目录。

长期保留路径如下：

- 工作区域点云：
  - `src/shandong/v14_urdf/final_assets/mode1_workspace/mode1_workspace_volume_zmax0p5.pcd`
- 工作区域约束：
  - `src/shandong/v14_urdf/final_assets/mode1_workspace/mode1_workspace_constraints_360_z0.json`

这两份文件分别对应：

- `bucket_tip_link` 在 `z <= 0.5m` 下的三维工作区域点云
- 360° 回转采样得到的地面切片工作区域约束 JSON

另外，和后续 `mode2` 设计直接相关的工作区域统计结果，包括：

- 工作区域整体 `z` 取值范围
- 每个 `z` 分层对应的 `x/y` 取值范围
- 每个 `z` 平面对应的圆环半径范围
- 平面圆环判定公式

已经补充到：

- `src/shandong/v14_urdf/mode1/README.md`

后续如果你要基于高度层做人工点选、多点连续挖掘、或 `mode2` 区域几何设计，建议直接
查看 `mode1/README.md` 中的“工作区域 z 范围与平面圆环”章节。

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
- 将多点规划按模式拆分到独立目录：
  - `multi_dig/README.md`：[多点规划 TODO](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v14_urdf/multi_dig/README.md)
  - `mode1/README.md`：[模式 1 设计](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v14_urdf/mode1/README.md)
