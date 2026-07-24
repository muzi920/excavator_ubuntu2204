# 模式 1：区域采样多点挖掘

模式 1 用于你描述的第一种多点实现方式：在点云平面上绘制一个矩形框作为挖掘区域，
并设置“左旋或右旋 `X°`”作为卸料区域。系统需要在明确工作空间边界的前提下，自动
生成多挖掘点循环任务，并在 RViz 中完成仿真演示。

本目录只做模式 1，不依赖修改 `v4_control_closed`，所有新增内容都落在
`v14_urdf/mode1/` 下。

## 目标

你最终希望一条命令就能做到：

1. 读取一个“区域采样配置”（矩形框 + 工作空间 + 卸料策略）。
2. 自动生成 `dig_points[]`（有执行顺序）。
3. 根据卸料策略生成 `dump_point`（或卸料扇区）。
4. 复用单点规划器为每个 `dig_point` 生成一个循环段。
5. 拼接 `init_segment` + `cycle_segment[]` + `home_segment`。
6. 输出一份任务级 JSON，供 `terminal_stepper.py` 或回放器直接执行。

## 输入定义（建议）

模式 1 建议统一使用一个 JSON 配置文件，例如：

- `v14_urdf/mode1/config/mode1-task.json`

示例结构如下：

```json
{
  "task_name": "mode1_area_sampling_demo",
  "workspace": {
    "r_min": 0.85,
    "r_max": 1.45,
    "yaw_min_deg": -140.0,
    "yaw_max_deg": 140.0,
    "z_min": -0.10,
    "z_max": 1.20
  },
  "dig_area_rect": {
    "center": {"x": 1.10, "y": 0.10, "z": 0.00},
    "length": 0.60,
    "width": 0.40,
    "yaw_deg": 0.0
  },
  "sampling": {
    "step_along_length": 0.12,
    "step_along_width": 0.15,
    "pattern": "boustrophedon"
  },
  "dump_strategy": {
    "direction": "left",
    "yaw_deg": 90.0,
    "yaw_tolerance_deg": 10.0,
    "dump_radius": 1.10,
    "dump_height": 0.50
  },
  "poses": {
    "init_pose": {"swing_yaw": 0.0, "boom_swing": 10.0, "arm_boom": 20.0, "bucket_arm": -30.0},
    "cycle_transit_pose": {"boom_swing": 25.0, "arm_boom": 55.0, "bucket_arm": -10.0},
    "home_pose": {"swing_yaw": 0.0, "boom_swing": 5.0, "arm_boom": 10.0, "bucket_arm": -80.0}
  }
}
```

## 工作空间标识（必须）

你提出“工作空间需要给出明确的标识”。在模式 1 中，这通常意味着两件事：

1. **规划侧约束必须明确**
   - 任何采样点必须经过 `workspace` 过滤，不可达点不能进入 `dig_points[]`。
2. **可视化侧边界最好能被检查**
   - 第一版可以先只在日志里打印：
     - “采样点总数”
     - “可达点数量”
     - “被过滤原因统计（超半径、超回转、超高度）”
   - 第二版再考虑把边界画到 RViz（Marker 的圆环扇区 + 高度提示）。

建议用“圆环扇区”表达平面工作空间：

- 内半径 `r_min`
- 外半径 `r_max`
- 回转范围 `yaw_min_deg` 到 `yaw_max_deg`

如果某个点位于扇区之外，直接丢弃或裁剪到边界，并给出原因。

## 区域采样（点生成）

模式 1 的核心是把一个矩形框转换为一组有顺序的挖掘点。

推荐采用条带式采样：

- 沿矩形长边方向生成若干条“挖掘带”
- 每条带上按固定步长采样点

推荐的默认参数：

- 点间距：`0.08 m` 到 `0.15 m`
- 行间距：`0.10 m` 到 `0.20 m`

执行顺序建议使用 “boustrophedon（牛耕式、蛇形）”：

- 第 1 条带从左到右
- 第 2 条带从右到左

这样可以减少回转角与伸缩的频繁大幅跳变。

## 卸料区域（左旋/右旋 X°）

你提出的卸料区域输入是“左旋或右旋 `X°`”。模式 1 里建议把它扩展为“卸料扇区”
而不是单一角度：

- `direction`: `left` 或 `right`
- `yaw_deg`: 扇区中心角
- `yaw_tolerance_deg`: 扇区角容差
- `dump_radius`: 卸料半径（固定值或区间）
- `dump_height`: 卸料高度（固定安全高度）

第一版可以把卸料点固定为一个点：

```text
dump_yaw = +yaw_deg  (left)  or  -yaw_deg  (right)
dump_x = dump_radius * cos(dump_yaw)
dump_y = dump_radius * sin(dump_yaw)
dump_z = dump_height
```

后续如果你希望“卸料区域”具有面积，则在扇区中采样一个点即可。

## 任务拼接规则（初始化与归位不入循环）

模式 1 必须强制拆分三段：

1. `init_segment`：只执行一次
2. `cycle_segment[]`：每个挖掘点一段，循环执行
3. `home_segment`：只执行一次

循环内部动作建议为：

- 对准挖掘点
- 挖掘（下探 + 收斗）
- 抬臂离坑
- 回转到卸料点
- 卸料（开斗到接近 `-90`）
- 回到 `cycle_transit_pose`

`cycle_transit_pose` 的作用是把每轮结束姿态统一到一个稳定状态，避免连续多点执行
时姿态漂移或出现不自然的大动作。

## 计划的文件划分（实现 TODO）

模式 1 的实现建议拆成若干小模块，便于单测与复用：

- `mode1/workspace.py`
  - 负责工作空间配置、可达性判断与裁剪
- `mode1/area_sampling.py`
  - 负责矩形区域点生成与排序
- `mode1/dump_strategy.py`
  - 负责从 “left/right + yaw” 生成卸料点或卸料扇区
- `mode1/mode1_task_planner.py`
  - 负责生成任务级 JSON（init + cycles + home）
  - 内部复用 `v14_urdf/point_to_dig_dump_trajectory.py` 做单点循环段生成

## 验证方式（RViz）

模式 1 的最小验证闭环是：

1. 启动 URDF，并关闭包内关节发布器：

   ```bash
   cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
   source install/setup.bash
   ros2 launch describe_60FED display.launch.py use_joint_state_publisher:=false
   ```

2. 生成模式 1 的多点任务 JSON（后续实现后补命令）。
3. 用 `terminal_stepper.py` 或回放器执行任务，并在 RViz 观察：
   - 是否按采样点顺序循环
   - 是否按指定方向回转到卸料扇区
   - 初始化与归位是否只执行一次
   - 循环内部是否总能回到 `cycle_transit_pose`

## 可视化运动约束（工作空间）

你可以先在 RViz 里看到一份“运动轨迹约束”的可视化结果：基于
`describe_60FED_calibrated.urdf` 的关节角限制，采样计算 `bucket_tip_link` 的可达
点云，并额外生成一份在 `z=0` 切片上的可作业 XY 区域（用于后续接入环境点云时做
区域过滤）。

1. 在 RViz 中添加 **MarkerArray**，并把 topic 设置为：

- `/v14_urdf/workspace_constraints`

2. 启动约束可视化节点：

```bash
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
source /opt/ros/humble/setup.bash
/usr/bin/python3 src/shandong/v14_urdf/mode1/workspace_constraints_viz.py
```

你会看到：

- 一团彩色点云：`bucket_tip_link` 可达点（颜色随高度变化）
- 一段文字信息：输出当前采样条件下的 `r/x/z` 范围，以及三关节的角度限制（deg）
- 一团绿色点云：`z=0` 切片上的可达 XY bin（用于表示地面可作业范围）

同时脚本会输出一份 JSON 到：

- 默认输出：`v14_urdf/mode1/constraints/workspace_constraints.json`
- 可选输出：你也可以通过 `--out` 另存一份（例如
  `v14_urdf/mode1/constraints/workspace_constraints_360_z0.json`），用于记录某次特定采样
  参数下的结果，方便对比。

你后续可以把这个文件作为规划器的约束输入，避免对不可达区域做采样。

### 360° 回转与 z=0 可作业区域

标定版 URDF 的 `swing_joint` 是 `[-180°, 180°]`，意味着回转轴本身支持 360°。本脚本
默认会对 `swing_joint` 也进行采样（参数 `--swing-samples`），并在指定 `z` 切片上
生成可达 XY 区域。

默认切片参数：

- `--z-slice 0.0`
- `--z-tol 0.02`
- `--xy-bin 0.05`

如果你想生成更稠密的地面可作业区域，减少 `xy-bin` 或增加采样数即可：

```bash
/usr/bin/python3 src/shandong/v14_urdf/mode1/workspace_constraints_viz.py \
  --swing-samples 90 \
  --boom-samples 45 \
  --arm-samples 45 \
  --bucket-samples 30 \
  --xy-bin 0.03 \
  --z-slice 0.0 \
  --z-tol 0.02
```

后续接入环境点云时，你可以先用 `bucket_tip_zslice_bounds` 的 XY 范围或 bin 集合过滤
点云，只保留可作业区域内的点。然后在每个 XY bin 内取 `z` 最大的点作为当前“地表或
料堆高度”，再进入挖掘点生成与规划流程。

### constraints 目录与 JSON 是什么

`mode1/constraints/` 下的 JSON 不是“手写配置”，而是你从 URDF 关节限制出发，通过采样
计算得到的**派生约束数据**。它的目标是把“这个挖掘机在当前 URDF 与角度限制下，大概
能伸到哪里”变成一份可以被程序读取的文件，后续用于：

- 在生成挖掘点之前做快速过滤，避免对明显不可达的区域采样。
- 在接入环境点云时做 ROI（Region of Interest）裁剪，只保留可作业区域的点。
- 在调参时对比不同采样密度、不同 `z` 切片参数下，可作业范围的变化。

<!-- prettier-ignore -->
> [!NOTE]
> `constraints/*.json` 会被 `workspace_constraints_viz.py` 自动覆盖写入。把它当作“输出
> 结果/缓存”即可，不要把它当作需要长期手工维护的配置文件。

目前目录里主要有两类文件：

- `workspace_constraints.json`
  - 默认输出文件。
  - 用途：提供一份“固定回转角（`swing_rad`）下”的 XZ 包络与关节限制，适合做半径与
    高度方向的粗过滤。
- `workspace_constraints_360_z0.json`
  - 常见做法是用 `--out` 另存一份，专门用于记录“包含 360° 回转采样 + `z=0` 切片”的
    地面可作业范围。
  - 用途：为“点云地面区域裁剪 + 网格取 `z_max`”提供最小可用的边界信息。

#### 字段解释（两份 JSON 通用）

`workspace_constraints*.json` 的结构一致，主要字段含义如下：

- `urdf_path`：参与计算的 URDF 路径，用于溯源。
- `frame_id`：计算与 Marker 发布使用的坐标系（通常是 `base_link`）。
- `sample_config`：本次采样的离散化参数（越大越密，越慢）。
  - `boom_samples`、`arm_samples`、`bucket_samples`：三个关节在限制区间的采样点数。
  - `swing_rad` / `swing_deg`：用于生成 `bucket_tip_bounds` 时采用的固定回转角。
  - `swing_samples`：用于生成 `z` 切片时对 `swing_joint` 的采样数量。
  - `z_slice`、`z_tol`：切片高度与容差，仅影响 `bucket_tip_zslice_bounds`。
  - `xy_bin`：地面可作业区域的 XY 网格大小（单位 m）。
- `joint_limits`：从 URDF 解析的关节角上下限（同时给出 rad 与 deg）。
- `bucket_tip_bounds`：`bucket_tip_link` 的可达包络（在 `swing_rad` 固定条件下采样）。
  - `min_r/max_r`：可达半径范围（`r = sqrt(x^2 + y^2)`），后续最常用。
  - `min_z/max_z`：可达高度范围。
  - `min_x/max_x`：在当前 `swing_rad` 下的 X 向包络（本质上是 XZ 平面投影的一部分）。

#### `bucket_tip_zslice_bounds`（只有在计算切片时才有意义）

当你开启了 `swing_samples` 并配置了 `z_slice/z_tol` 时，脚本会额外生成
`bucket_tip_zslice_bounds`，表示“在 `z=z_slice` 附近，`bucket_tip_link` 能落到哪些 XY
网格上”：

- `min_x/max_x/min_y/max_y`：在 `xy_bin` 网格对齐后的 XY 边界。
- `min_r/max_r`：对应的半径边界（同样是网格化后的）。
- `bins`：命中的网格数量（去重后的 `(bx, by)` 数量）。
- `slice_bin_points`：每个可作业 bin 的浮点坐标列表，格式为
  `[{ "x": ..., "y": ... }, ...]`。这里的 `x/y` 是经过 `xy_bin` 网格对齐后的 bin 坐标，
  后续可以直接当作可作业 mask 使用。

相比只看 `min/max` 边界，`slice_bin_points` 的价值在于它提供了一份更精确的可作业 mask。
边界框内部并不是每个点都一定可达，但落在 `slice_bin_points` 对应网格内的点，至少通过了
当前 URDF 关节限制与采样配置下的可达性筛选。

#### 你后续会怎么用（推荐最小流程）

1. **规划前粗过滤（先快后准）**
   - 在区域采样生成候选点后，先用 `bucket_tip_bounds.min_r/max_r` 与
     `bucket_tip_bounds.min_z/max_z` 做快速过滤。
   - 过滤通过后，再做 IK 求解与更严格的碰撞/姿态约束（后续实现）。

2. **环境点云裁剪 + “每格取最高点”**
   - 先用 `bucket_tip_zslice_bounds` 的 XY 边界做第一层粗裁剪，减少数据量。
   - 再把点投影到 `slice_bin_points` 对应的 bin 集合里，只保留可作业 mask 内的点。
   - 最后按 `xy_bin` 做网格化，对每个网格只保留 `z` 最大的点，得到地表高度或料堆高度的
     稀疏表示。

示例伪代码（Python）：

```python
import json

with open("v14_urdf/mode1/constraints/workspace_constraints_360_z0.json", "r") as f:
    c = json.load(f)

b = c["bucket_tip_zslice_bounds"]
xy_bin = float(b["xy_bin"])
min_x, max_x = float(b["min_x"]), float(b["max_x"])
min_y, max_y = float(b["min_y"]), float(b["max_y"])
valid_bins = {
    (int(round(p["x"] / xy_bin)), int(round(p["y"] / xy_bin)))
    for p in b["slice_bin_points"]
}

grid_max = {}
for x, y, z in points_xyz:  # points_xyz: iterable[(x,y,z)]
    if x < min_x or x > max_x or y < min_y or y > max_y:
        continue
    bx = int(round(x / xy_bin))
    by = int(round(y / xy_bin))
    key = (bx, by)
    if key not in valid_bins:
        continue
    if key not in grid_max or z > grid_max[key][2]:
        grid_max[key] = (x, y, z)

surface_points = list(grid_max.values())
```

## 点云转候选挖掘点

现在已经补了一条最小可用链路：把环境点云先投影到 `constraints` 生成的可作业 mask 上，再
输出一组可直接进入后续规划器的 `candidate_dig_points`。

脚本路径：

- `v14_urdf/mode1/pointcloud_to_dig_points.py`

它的职责只做三件事：

1. 读取点云 JSON。
2. 用 `workspace_constraints_360_z0.json` 里的 `slice_bin_points` 做可作业区域过滤。
3. 对每个 XY bin 只保留 `z` 最大的点，并按模式 1 的顺序输出候选挖掘点。

### 输入要求

当前版本不依赖 PCD 或 ROS topic，先使用 JSON 文件作为输入，便于离线验证和和后续实物
控制解耦。支持两种点格式：

- 顶层数组：`[[x, y, z], ...]`
- 对象形式：`{"points": [{"x": ..., "y": ..., "z": ...}, ...]}`

如果你还传入模式 1 任务配置 JSON，脚本会继续读取：

- `dig_area_rect`
- `workspace`
- `sampling.pattern`

也就是说，它会先做“可作业 mask 过滤”，再做“矩形区域过滤”和“工作空间过滤”。

### 输出内容

脚本会输出一份 JSON，例如：

- `v14_urdf/mode1/output/candidate_dig_points.json`

其中最重要的字段有：

- `surface_points`
  - 经过 `slice_bin_points` 过滤后，每个 bin 的最高点。
- `candidate_dig_points`
  - 经过矩形区域和工作空间再次筛选，并按执行顺序排序后的候选挖掘点。
- `filter_stats`
  - 记录每一步被过滤的数量，便于你判断是矩形太小、工作空间太紧，还是 mask 太稀疏。

`candidate_dig_points` 中每个点目前包含：

- `x/y/z`
- `radius`
- `yaw_deg`
- `bin_x/bin_y`
- `bin_key`
- `candidate_index`

这组数据已经足够作为后续 `dig_points[]` 的雏形，再往后只需要继续接：

- 卸料点生成
- 单点 IK / 轨迹生成
- `init/cycle/home` 拼接

### 运行示例

仓库里放了两个最小示例文件：

- `v14_urdf/mode1/examples/sample_pointcloud.json`
- `v14_urdf/mode1/examples/sample_mode1_task.json`

可以直接这样运行：

```bash
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
/usr/bin/python3 src/shandong/v14_urdf/mode1/pointcloud_to_dig_points.py \
  --points src/shandong/v14_urdf/mode1/examples/sample_pointcloud.json \
  --constraints src/shandong/v14_urdf/mode1/constraints/workspace_constraints_360_z0.json \
  --task-config src/shandong/v14_urdf/mode1/examples/sample_mode1_task.json \
  --out src/shandong/v14_urdf/mode1/output/sample_candidate_dig_points.json
```

如果你暂时只想验证 `slice_bin_points` 的 mask 过滤，而不想叠加矩形区域或 workspace 约束，
可以不传 `--task-config`。

## 候选点转多点任务 JSON

在拿到 `candidate_dig_points` 之后，下一步就是把它们拼成一份可以直接交给
`terminal_stepper.py` 的任务剧本。现在已经补了：

- `v14_urdf/mode1/mode1_task_planner.py`

它会完成这几件事：

1. 读取 `candidate_dig_points`。
2. 从 `task-config` 里生成统一的 `dump_point`。
3. 复用 `point_to_dig_dump_trajectory.py` 为每个候选点生成单轮挖掘/卸料脚本。
4. 自动拼接 `init_segment + cycle_segment[] + home_segment`。
5. 输出 `metadata + script` 结构，供终端步进器直接加载。

### 输入要求

这个脚本依赖两份输入：

- `candidate-json`
  - 由 `pointcloud_to_dig_points.py` 输出。
- `task-config`
  - 至少需要提供 `dump_strategy` 或 `dump_point`。
  - 如果提供 `poses`，会用于 `init_pose`、`cycle_transit_pose`、`home_pose`。

如果你当前**没有点云**，也可以直接走“预设区域模式”：

- 不传 `--candidate-json`
- 在 `task-config` 中提供：
  - `dig_area_rect`
  - `sampling`
  - `workspace`
  - `dump_strategy` 或 `dump_point`

这时 `mode1_task_planner.py` 会直接从矩形区域生成候选挖掘点。
如果你再提供 `--constraints-json`，脚本会优先使用 `constraints` 中的 `slice_bin_points`
作为底层采样网格，再与矩形区域求交。这样生成的预设区域挖掘点会更贴近真实可达域，而不是
只按理想矩形均匀铺点。

仓库里的示例配置已经补齐了：

- `v14_urdf/mode1/examples/sample_mode1_task.json`

### 输出结构

输出文件默认是：

- `v14_urdf/mode1/output/mode1_task_plan.json`

根节点仍然采用终端步进器兼容的结构：

- `metadata`
- `script`

其中：

- `metadata.segments`
  - 记录 `init / cycle / transit / home` 各段的 step 范围。
- `metadata.cycles`
  - 记录每轮循环对应的候选点、step 范围、bin 信息。
- `metadata.skipped_candidates`
  - 记录哪些候选点虽然通过了 workspace/mask 过滤，但没有通过当前单点 IK 与姿态搜索。
- `metadata.requested_cycle_count` / `metadata.cycle_count`
  - 前者表示“想执行多少个候选点”，后者表示“最终真正拼进任务里的循环数”。
- `metadata.dig_point_source`
  - 标记这些挖掘点来自 `candidate_json` 还是 `area_sampling`。
- `metadata.grid_source`
  - 如果是预设区域模式，标记底层网格来自 `rect_sampling` 还是 `constraints_bins`。
- `script`
  - 可直接交给 `terminal_stepper.py` 执行。

这一步很重要，因为 `candidate_dig_points` 只是“几何上值得尝试”的候选点，不等于它一定能在
当前单点挖掘姿态搜索范围里找到可行解。任务规划器会把失败点记到
`metadata.skipped_candidates`，并继续保留可执行的循环段。

### 运行示例

先生成候选点：

```bash
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
/usr/bin/python3 src/shandong/v14_urdf/mode1/pointcloud_to_dig_points.py \
  --points src/shandong/v14_urdf/mode1/examples/sample_pointcloud.json \
  --constraints src/shandong/v14_urdf/mode1/constraints/workspace_constraints_360_z0.json \
  --task-config src/shandong/v14_urdf/mode1/examples/sample_mode1_task.json \
  --out src/shandong/v14_urdf/mode1/output/sample_candidate_dig_points.json
```

再生成多点任务 JSON：

```bash
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
/usr/bin/python3 src/shandong/v14_urdf/mode1/mode1_task_planner.py \
  --candidate-json src/shandong/v14_urdf/mode1/output/sample_candidate_dig_points.json \
  --task-config src/shandong/v14_urdf/mode1/examples/sample_mode1_task.json \
  --out src/shandong/v14_urdf/mode1/output/sample_mode1_task_plan.json
```

如果你想先做小规模验证，可以只取前几个候选点：

```bash
/usr/bin/python3 src/shandong/v14_urdf/mode1/mode1_task_planner.py \
  --candidate-json src/shandong/v14_urdf/mode1/output/sample_candidate_dig_points.json \
  --task-config src/shandong/v14_urdf/mode1/examples/sample_mode1_task.json \
  --max-candidates 2 \
  --out src/shandong/v14_urdf/mode1/output/sample_mode1_task_plan_2cycles.json
```

生成后，你可以直接交给终端步进器：

```bash
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
/usr/bin/python3 src/shandong/v14_urdf/terminal_stepper.py \
  src/shandong/v14_urdf/mode1/output/sample_mode1_task_plan.json
```

这样你就能逐步检查：

- 初始化段是否只执行一次。
- 每轮挖掘是否都回到 `cycle_transit_pose`。
- 归位段是否只在全部循环结束后执行。

### 直接用预设区域生成任务

如果你当前还没有环境点云，但想先把“预设区域挖掘 -> 卸料 -> 过渡 -> 归位”的整条动作链跑
通，可以直接这样生成任务：

```bash
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws
/usr/bin/python3 src/shandong/v14_urdf/mode1/mode1_task_planner.py \
  --task-config src/shandong/v14_urdf/mode1/examples/sample_mode1_task.json \
  --constraints-json src/shandong/v14_urdf/mode1/constraints/workspace_constraints_360_z0.json \
  --out src/shandong/v14_urdf/mode1/output/sample_mode1_area_task_plan.json
```

这条命令的含义是：

- 直接使用 `sample_mode1_task.json` 中的矩形区域与卸料策略。
- 用 `workspace_constraints_360_z0.json` 的 `slice_bin_points` 作为可作业网格。
- 自动筛掉不在工作空间内、或未通过单点 IK 的点。
- 输出一份已经拼好 `init + cycles + transit + home` 的任务 JSON。

对于“无点云预设区域模式”，`dig_area_rect.center.z` 很关键。因为这时没有外部点云提供地表
高度，所以这个 `z` 实际上就是当前预设挖掘面的目标高度。如果这个高度设得太低或太高，可能
会导致所有候选点都无法通过单点 IK。

## 当前阶段总结

到当前阶段，`mode1` 已经从“预设矩形区域采样”扩展成“真实点云辅助的可挖区域
筛选与动作验证”链路，并完成了下面这些关键工作：

- 已完成 `bucket_tip_link` 工作区域采样、`z=0` 切片约束导出，以及
  `slice_bin_points` 形式的 JSON 约束保存。
- 已完成真实点云链路的独立工具目录 `mode1/real_pcd/`，避免修改已有稳定的
  `mode1` API。
- 已完成本体点云过滤、原始点云减去 removed 点云、工作区域点云生成，以及与原始
  场景点云的融合检查。
- 已确认真正应长期保留的“工作区域”不是单纯的 `z=0` 投影，而是
  `bucket_tip_link` 的三维可达域点云。
- 已完成基于 `pointcloud_base_link_*.pcd` 的真实场景验证，并确认最终动作验证
  阶段采用“前方土堆点 + 工作区域匹配 + 候选点规划”的流程更符合当前需求。
- 已完成一轮小规模多点验证：从最终 `x > 0.5`、`0.02 <= z <= 0.5` 的前方可挖
  区域里提取候选点，生成 mode1 多点任务，并在 RViz 中执行动作回放。

## Mode1 最终保留资产

本阶段结束后，`mode1` 最需要长期保留的内容只有两类：

1. 工作区域点云 `pcd`
2. 工作区域约束 `json`

为了避免后续清理 `mode1/output` 或 `real_pcd/output` 时误删，这两份文件已经额外
复制到 `v14_urdf/final_assets/mode1_workspace/` 下，作为固定保留件：

- 工作区域点云：
  - `src/shandong/v14_urdf/final_assets/mode1_workspace/mode1_workspace_volume_zmax0p5.pcd`
- 工作区域约束：
  - `src/shandong/v14_urdf/final_assets/mode1_workspace/mode1_workspace_constraints_360_z0.json`

它们分别对应：

- `mode1_workspace_volume_zmax0p5.pcd`
  - 语义：`bucket_tip_link` 在 `z <= 0.5m` 条件下的三维可达工作区域点云。
  - 用途：作为后续真实点云、ROI 融合、点云匹配、以及工作区域可视化的统一几何
    基准。
- `mode1_workspace_constraints_360_z0.json`
  - 语义：基于 360° 回转采样导出的地面切片工作区域约束。
  - 用途：作为后续 XY 可作业 mask、区域粗过滤、以及与场景点云做平面约束时的
    统一约束基准。

## 工作区域 z 范围与平面圆环

对于后续 `mode2` 方案设计，当前最关键的基础不是单个候选点，而是这份最终保留的
工作区域三维点云：

- `src/shandong/v14_urdf/final_assets/mode1_workspace/mode1_workspace_volume_zmax0p5.pcd`

这份点云表达的是：

- 末端 `bucket_tip_link` 在 `z <= 0.5m` 条件下的三维可达工作区域
- 坐标系为 `base_link`
- 本质上可以看成一组“随 z 变化的平面圆环截面”叠加形成的三维体

根据当前保留点云统计，整体范围如下：

- 点云总数：`294048`
- `z` 范围：`-0.2608 m` 到 `0.5000 m`
- `x` 范围：`-1.7816 m` 到 `1.7798 m`
- `y` 范围：`-1.7811 m` 到 `1.7811 m`
- 半径 `r = sqrt(x^2 + y^2)` 范围：`0.5839 m` 到 `1.7816 m`

### 用于生成稠密环形半球区域的参数化公式

如果你后续 `mode2` 想做的不是“读取已有点”，而是“直接生成一片更密集的环形半球状
挖掘区域”，建议不要再从离散点反推，而是直接使用柱坐标参数化。

先定义：

- 圆心平移：`(x_c, y_c, z_c)`
- 水平半径：`r`
- 水平角：`theta`
- 高度：`z`

那么最基本的空间生成公式是：

```text
x = x_c + r * cos(theta)
y = y_c + r * sin(theta)
z = z
```

其中关键不在于 `x/y` 本身，而在于你如何定义 `r` 随 `z` 的变化。

#### 公式 1：按 z 分层生成圆环截面

如果你希望每一个高度平面 `z = z0` 都是一个圆环，那么最直接的生成方式是：

```text
r_min(z0) <= r <= r_max(z0)
theta_min <= theta <= theta_max
x = x_c + r * cos(theta)
y = y_c + r * sin(theta)
z = z0
```

这相当于说：

```text
r_min(z0)^2 <= (x - x_c)^2 + (y - y_c)^2 <= r_max(z0)^2
```

这是你后续 `mode2` 最稳妥、也最容易控制密度的基础公式。

#### 公式 2：生成环形半球壳

如果你想要的形状更接近“半球壳”而不是简单直筒圆环，那么可以把内外半径定义为
高度 `z` 的函数：

```text
r_out(z) = sqrt(max(0, R_out^2 - (z - z_c)^2))
r_in(z)  = sqrt(max(0, R_in^2  - (z - z_c)^2))
```

于是空间中的任意一点由下面的参数生成：

```text
z in [z_min, z_max]
theta in [theta_min, theta_max]
rho in [r_in(z), r_out(z)]

x = x_c + rho * cos(theta)
y = y_c + rho * sin(theta)
z = z
```

这里：

- `R_out` 控制外半球壳半径
- `R_in` 控制内空腔半径
- `R_in < R_out`
- 当 `R_in > 0` 时，形状就是“环形半球壳”
- 当 `R_in = 0` 时，形状退化成普通半球体

#### 公式 3：均匀填充环带厚度

如果你希望在每一层圆环里生成得更均匀，不要直接线性采样 `r`，而要按面积均匀采样：

```text
u in [0, 1]
rho(z, u) = sqrt((1 - u) * r_in(z)^2 + u * r_out(z)^2)
```

然后再代入：

```text
x = x_c + rho(z, u) * cos(theta)
y = y_c + rho(z, u) * sin(theta)
z = z
```

这样生成出来的点不会在内圈过密、外圈过疏，更适合你想做的“更加密集的环形半球状
挖掘区域”。

#### 前方挖掘区域的角度约束

如果你只想保留挖掘机前方区域，不需要整个 360° 圆环，那么只要对 `theta` 加限制。

例如，以 `base_link` 前方为 `+x` 方向时：

```text
x > 0        <=> theta in (-pi/2, pi/2)
x > 0.5      <=> x_c + rho * cos(theta) > 0.5
```

因此前方半环的生成公式可以写成：

```text
theta_min = -pi/2
theta_max =  pi/2
```

如果还要避开车体长度，你可以直接增加一个前向裁剪条件：

```text
x = x_c + rho * cos(theta) > x_front_min
```

例如当前你已经验证过的一版保守条件就是：

```text
x_front_min = 0.5
```

#### 离散生成方式

如果你要把上述公式落成一片稠密点云，可以使用三重采样：

```text
z_k     = z_min + k * dz
theta_j = theta_min + j * dtheta
u_i     = i / (N_u - 1)
rho_ijk = rho(z_k, u_i)

x_ijk = x_c + rho_ijk * cos(theta_j)
y_ijk = y_c + rho_ijk * sin(theta_j)
z_ijk = z_k
```

也就是：

1. 先按高度 `dz` 分层。
2. 每一层按角度 `dtheta` 展开成圆弧。
3. 在每一层圆弧厚度内再用 `u` 生成内外环带。

这样你就能直接生成一个比当前工作区域点云更密集的环形半球状区域。

#### mode2 推荐的实用写法

如果你下一步的目标是先把 `mode2` 做出来，而不是严格复现真实工作区域边界，那么推荐
先用下面这套统一形式：

```text
z in [z_min, z_max]
theta in [theta_min, theta_max]

r_out(z) = sqrt(max(0, R_out^2 - (z - z_c)^2))
r_in(z)  = sqrt(max(0, R_in^2  - (z - z_c)^2))
rho(z, u) = sqrt((1 - u) * r_in(z)^2 + u * r_out(z)^2)

x = x_c + rho(z, u) * cos(theta)
y = y_c + rho(z, u) * sin(theta)
z = z
```

如果需要前方区域，再加：

```text
x > x_front_min
```

这一套就是你后续最核心的 `x/y/z` 生成公式。

### 公式中的 x、y、z 取值范围

为了让这套公式可以直接用于 `mode2`，这里把生成后 `x/y/z` 的可用范围明确写出来。
这些范围来自当前最终保留工作区域点云的统计结果，以及前面定义的几何约束。

#### 1. z 的取值范围

如果你直接以当前 `mode1` 最终保留工作区域为基准，那么：

```text
z_min = -0.2608
z_max = 0.5000
```

也就是：

```text
z in [-0.2608, 0.5000]
```

如果你只想保留地面以上、用于土堆挖掘的正高度部分，也可以进一步约束为：

```text
z in [0.02, 0.50]
```

#### 2. x、y 的全局包围范围

在当前最终保留工作区域点云中，整体包围范围为：

```text
x in [-1.7816, 1.7798]
y in [-1.7811, 1.7811]
```

因此如果你完全不加前方裁剪，公式生成出的点理论上就会落在这个整体包围盒内。

#### 3. x、y 的公式约束范围

对任意一个由参数生成的点：

```text
x = x_c + rho(z, u) * cos(theta)
y = y_c + rho(z, u) * sin(theta)
```

其中：

```text
rho(z, u) in [r_in(z), r_out(z)]
theta in [theta_min, theta_max]
```

因此在任意固定高度 `z = z0` 上：

```text
x in [x_c + r_in(z0) * cos(theta), x_c + r_out(z0) * cos(theta)]
y in [y_c + r_in(z0) * sin(theta), y_c + r_out(z0) * sin(theta)]
```

更实用的写法是直接用上下界表示：

```text
x_min(z) = min_theta,rho (x_c + rho * cos(theta))
x_max(z) = max_theta,rho (x_c + rho * cos(theta))
y_min(z) = min_theta,rho (y_c + rho * sin(theta))
y_max(z) = max_theta,rho (y_c + rho * sin(theta))
```

如果 `theta` 是完整圆周 `[-pi, pi]`，那么有：

```text
x in [x_c - r_out(z), x_c + r_out(z)]
y in [y_c - r_out(z), y_c + r_out(z)]
```

如果只取前方半环：

```text
theta in [-pi/2, pi/2]
```

那么：

```text
x in [x_c, x_c + r_out(z)]
y in [y_c - r_out(z), y_c + r_out(z)]
```

如果再叠加车体前方约束：

```text
x > x_front_min
```

例如当前已经验证过的保守条件：

```text
x_front_min = 0.5
```

那么最终生成点需要同时满足：

```text
x in [0.5, x_c + r_out(z)]
y in [y_c - r_out(z), y_c + r_out(z)]
z in [z_min, z_max]
```

#### 4. 在当前 mode1 统计下可直接使用的推荐范围

如果你想基于当前 `mode1` 统计结果，先做一版和现有工作区域近似一致的密集环形半球，
可以先直接用下面这组经验范围：

```text
z in [0.02, 0.50]
x in [0.50, 1.78]
y in [-1.78, 1.78]
```

然后在每一个 `z` 层上，再叠加对应的圆环半径范围：

```text
r in [r_min(z), r_max(z)]
```

例如对地面以上部分，可以近似记成：

```text
0.02 <= z <= 0.50
0.58 <= r <= 1.78
x > 0.5
```

这组约束非常适合作为 `mode2` 第一版密集点云生成的初始条件。

#### 5. 用于程序实现的统一边界条件

如果你后续要直接写代码生成点云，可以把最终判定统一写成：

```text
z_min <= z <= z_max
r_in(z) <= sqrt((x - x_c)^2 + (y - y_c)^2) <= r_out(z)
theta_min <= atan2(y - y_c, x - x_c) <= theta_max
x >= x_front_min
```

其中一组适合当前 `mode2` 起步验证的参数是：

```text
z_min = 0.02
z_max = 0.50
theta_min = -pi/2
theta_max =  pi/2
x_front_min = 0.5
```

这样你就同时得到：

- 公式本身
- `x/y/z` 的可生成范围
- 前方半环的几何约束
- 可直接落成代码的统一判定条件

### 各 z 分层对应的 x、y、r 范围

下面这张表按 `0.05m` 做 z 分层，给出每层对应的 `x/y` 范围和圆环半径范围。这里的
`r_min/r_max` 比直接用 `x/y` 包围框更适合后续做 `mode2` 几何设计。

| z 范围 (m) | 点数 | x 范围 (m) | y 范围 (m) | r 范围 (m) |
| --- | ---: | --- | --- | --- |
| `[-0.30, -0.25]` | 1224 | `[-1.2313, 1.2301]` | `[-1.2310, 1.2310]` | `[1.0344, 1.2313]` |
| `[-0.25, -0.20]` | 5400 | `[-1.3875, 1.3861]` | `[-1.3872, 1.3872]` | `[0.8537, 1.3875]` |
| `[-0.20, -0.15]` | 9432 | `[-1.5013, 1.4999]` | `[-1.5010, 1.5010]` | `[0.7657, 1.5013]` |
| `[-0.15, -0.10]` | 13032 | `[-1.5538, 1.5523]` | `[-1.5534, 1.5534]` | `[0.6891, 1.5538]` |
| `[-0.10, -0.05]` | 16488 | `[-1.6259, 1.6243]` | `[-1.6255, 1.6255]` | `[0.6394, 1.6259]` |
| `[-0.05, 0.00]` | 19584 | `[-1.6690, 1.6674]` | `[-1.6686, 1.6686]` | `[0.6198, 1.6690]` |
| `[0.00, 0.05]` | 20376 | `[-1.7075, 1.7059]` | `[-1.7071, 1.7071]` | `[0.5929, 1.7075]` |
| `[0.05, 0.10]` | 21816 | `[-1.7412, 1.7395]` | `[-1.7408, 1.7408]` | `[0.5861, 1.7412]` |
| `[0.10, 0.15]` | 22176 | `[-1.7546, 1.7529]` | `[-1.7542, 1.7542]` | `[0.5839, 1.7546]` |
| `[0.15, 0.20]` | 23832 | `[-1.7636, 1.7619]` | `[-1.7632, 1.7632]` | `[0.5934, 1.7636]` |
| `[0.20, 0.25]` | 23760 | `[-1.7742, 1.7725]` | `[-1.7738, 1.7738]` | `[0.6049, 1.7742]` |
| `[0.25, 0.30]` | 23184 | `[-1.7749, 1.7732]` | `[-1.7745, 1.7745]` | `[0.6291, 1.7749]` |
| `[0.30, 0.35]` | 23112 | `[-1.7802, 1.7785]` | `[-1.7798, 1.7798]` | `[0.6425, 1.7802]` |
| `[0.35, 0.40]` | 23400 | `[-1.7782, 1.7764]` | `[-1.7777, 1.7777]` | `[0.6518, 1.7782]` |
| `[0.40, 0.45]` | 23832 | `[-1.7816, 1.7798]` | `[-1.7811, 1.7811]` | `[0.6521, 1.7816]` |
| `[0.45, 0.50]` | 23400 | `[-1.7768, 1.7750]` | `[-1.7763, 1.7763]` | `[0.6426, 1.7768]` |

### 给 mode2 的直接结论

如果你后续的 `mode2` 需要“用户手工给一批连续挖掘点”，最稳妥的做法不是直接在完整
三维点云里手选，而是按下面方式做门限：

1. 先根据目标点的 `z`，找到所属的 `z` 分层。
2. 使用该层的 `r_min/r_max` 做圆环判定：

   ```text
   r = sqrt(x^2 + y^2)
   ```

3. 只有满足 `r_min(z) <= r <= r_max(z)` 的点，才作为这一高度层的合法作业点。
4. 如果还需要更保守，可以再叠加：
   - `x > 0` 或 `x > 0.5`
   - 左右侧 `y` 范围限制
   - 点云表层高度带限制

这样设计的优点是：

- `mode2` 可以直接继承 `mode1` 已验证过的工作区域几何
- 不需要每次重新从 URDF 全量采样
- 可以很自然地把“某个高度层的可挖圆环”转换为人工选点约束

## Mode1 收口说明

当前建议把 `mode1` 收口在“保留工作区域资产 + 保留一份最终成功验证结果”这一状态，
不再继续堆积试验版输出目录。

本轮清理的原则是：

- 保留源码、示例、约束文件和最终保留资产。
- 保留最后一版成功验证目录：
  - `mode1/real_pcd/output/base_link_0000_workspace_zmax0p5/`
  - `mode1/real_pcd/output/base_link_0000_ceres_match_cpu_xyfront_x0p5/`
- 删除早期试验目录、临时输出和失败过程文件，避免后续再误用旧结果。

## 后续接手建议

如果后续重新启动 `mode1`，建议直接从下面两份资产开始，而不要从旧的试验输出目录
继续回溯：

1. `v14_urdf/final_assets/mode1_workspace/mode1_workspace_volume_zmax0p5.pcd`
2. `v14_urdf/final_assets/mode1_workspace/mode1_workspace_constraints_360_z0.json`

这样可以保证后续无论走：

- 点云 ROI 融合
- 点云匹配
- 候选点选取
- 多点轨迹规划

都以同一份工作区域几何和同一份约束 JSON 为基准。
