# v15_action_task examples —— 独立测试与演示脚本

<!-- prettier-ignore -->
> [!NOTE]
> 本目录下所有脚本**仅用于演示与回归验收**，不会出现在
> `action_library / config / control_core / sensors / motion / kinematics`
> 等正式模块的发布清单中。正式业务代码必须通过
> `from v15_action_task import from_config, ...` 的标准接口使用。

## 1. 快速上手

所有脚本都通过 `_util_import_helper.ensure_v15_importable()` 自动把
`v15_action_task` 的父目录插入 `sys.path`，因此**不需要提前
`pip install`，直接 `python3 xxx.py` 就能跑**。

运行方式（在本目录下执行即可）：

```bash
cd v15_action_task/examples/

# ① 一键 43 项全量回归（推荐先跑这条）
python3 _run_all_checks.py

# ② 单跑某个具体演示
python3 ex02_semantic_direction_AC7_self_assert.py   # 动作库方向语义验收
python3 ex10_physical_240mm_delta_L.py                # 连杆参数生效验证
python3 ex07_fk_ik_closed_loop.py                     # FK→IK→FK 数学闭环
```

<!-- prettier-ignore -->
> [!TIP]
> 如果你的机器上**没有安装 ROS 2 / rclpy**，或者**没有 PyYAML**
> （例如 Ubuntu 22.04 裸系统的默认 Python 3.10），所有脚本都不会崩溃：
> - 配置层自动走三级兜底：`YAML 失败 → 同名 JSON → BUILTIN dict`
> - 传感器层自动走 `MockManager` 回退：`try/except rclpy` +
>   `sensor_ros_mode="auto"` 动态检测

---

## 2. 辅助文件（2 个，不以 `exNN_` 开头）

| 文件名 | 作用 | 是否需要手动运行 |
|--------|------|------------------|
| `__init__.py` | 让 `examples/` 成为可 import 的包，便于脚本之间引用 | 否 |
| [`_util_import_helper.py`](./_util_import_helper.py) | 提供 `ensure_v15_importable()`，每个 `exNN_*.py` 的第 1 行都会调用它，保证脚本直接运行时也能找到 `v15_action_task` 顶层包 | 否（脚本自动调用） |

---

## 3. 回归脚本（1 个）

### [`_run_all_checks.py`](./_run_all_checks.py) —— 43 项一键回归

**这是验收阶段的唯一入口**：运行后会按 `TR0 → TR1 → TR2 → TR3 → TR4 → TR5 → TR6`
七大分组顺序自动跑 43 个断言，最后打印：

```
====================================================================
  === TOTAL PASS 43/43 ===
====================================================================
```

- **`exit code = 0`** → 100% 通过，可放心合并 / 部署
- **非 0 exit code** → 失败项会带具体细节（实际值、期望值、匹配到的
  grep 行等），直接搜索输出中的 `FAIL` 即可定位

内部 TR 分类与用户规格的对应关系：

| TR 大类 | 覆盖的用户规格 | 覆盖的 AC 硬验收 |
|---------|---------------|-----------------|
| TR0 | 独立 TF 标定脚本（ex11 / ex12） | AC-10 标定闭环 |
| TR1 | 配置层 sensors / extrinsics / tilt_compensation 三大节 | AC-5 限位不动 / AC-8 外参出厂值 / AC-9 倾角地址映射 |
| TR2 | 语义作业 8 指令库 | AC-7 方向语义自洽 / AC-1 三层夹紧 |
| TR3 | sensors 子包 + TiltCompensator + 旧版本 grep | AC-6 零 v0~v14 依赖 |
| TR4 | 顶层 from_config / __init__ 聚合导出 | AC-2 数据类型 / 旧 API 零 break |
| TR5 | examples 目录独立结构 + 所有脚本可运行 | AC-4 目录独立 |
| **TR6** | **Phase3 可达域 + 双策略轨迹 + 三控制接口（本轮 10 项）** | **AC-1 可达域边界 / AC-2 远点超限 / AC-3 ground穿透 / AC-4 LERP线性 / AC-5 Trapezoidal速度 / AC-6 接口1 FK误差≤5cm / AC-7 接口2 抬臂+回转 / AC-8 接口3 卸料步数+角度 / AC-9 grep 0行 / AC-10 config方向语义注释块** |

---

## 4. 测试与演示脚本（15 个 `exNN_*.py`）

按功能分五组：**核心配置/动作**、**传感器接口**、**运动学/笛卡尔**、
**标定工具链**、**Phase3 验收（可达域+轨迹+三接口）**。推荐第一次浏览时按编号顺序跑。

### 4.1 核心配置 / 动作库（ex01~ex03）

| # | 脚本名 | 功能 | 关键输出 |
|---|--------|------|---------|
| 01 | [`ex01_joint_limits_three_layer_clamp.py`](./ex01_joint_limits_three_layer_clamp.py) | 演示 **FR-1 三层限位夹紧契约**：业务层下发超限 → ① Config 层 `limits.clamp_pose` 裁剪 → ② URDFController 内部 `_clamp_pose` 裁剪 → ③ `semantic_moves` 显式调用；对比三层裁剪前后的 `boom_swing` / `bucket_arm` 实际值 | 超限 999° → 夹紧到 55.0°，PASS / FAIL |
| 02 | [`ex02_semantic_direction_AC7_self_assert.py`](./ex02_semantic_direction_AC7_self_assert.py) | **AC-7 硬验收**：对 4 关节 × 2 布尔方向共 8 种组合逐一自断言方向正确性；`_run_all_checks.py` 会用子进程调用它并检查 `exit 0` | `8/8 PASS` + 打印 `AC-7 OK`，否则 `exit 1` |
| 03 | [`ex03_eight_semantic_commands.py`](./ex03_eight_semantic_commands.py) | 依次调用 **FR-2 全部 8 个函数**：4 个基础增量（swing_move / boom_move / arm_move / bucket_move）+ 4 个扩展（swing_move_to / boom_lift / arm_extend / bucket_tilt_to），打印每个 `MoveResult.success` 表格 | 8 行 `success=True` 表格 |

### 4.2 传感器接口（ex04~ex06）

| # | 脚本名 | 功能 | 关键输出 |
|---|--------|------|---------|
| 04 | [`ex04_sensors_mock_14_channels.py`](./ex04_sensors_mock_14_channels.py) | 强制 `sensor_ros_mode="force_mock"` 后端，枚举 **FR-3 15 路传感器**：4 倾角 + 5 雷达 + 6 相机；通过 Mock 的 `_inject_reading` 钩子注入合成读数并回读验证 | `tilt=4 lidar=5 camera=6` + 每路回读 OK |
| 05 | [`ex05_sensors_ros_optional.py`](./ex05_sensors_ros_optional.py) | **AC-6 / TR3.5 合规**：以默认 `auto` 模式打开传感器。当前机器无 `rclpy` 时打印 `SKIP no rclpy` 并 **exit 0**（绝不崩溃或抛异常）；有 ROS 时简单等待 0.2 s 取一次缓存 | 终端输出 `SKIP` / `ROS OK`，exit 均为 0 |
| 06 | [`ex06_full_pipeline_sensors_motion.py`](./ex06_full_pipeline_sensors_motion.py) | 控制端 + 传感器端 **全链路冒烟测试**：开 SensorManager → 开 URDFController → `boom_move(up=10°)` → 读 `tilt_bucket`（即使 Mock 模式也走一遍完整 getter 路径）→ 输出 `PIPELINE OK` | `PIPELINE OK` |

### 4.3 运动学 / 笛卡尔 / 任务脚本（ex07~ex10）

| # | 脚本名 | 功能 | 关键输出 |
|---|--------|------|---------|
| 07 | [`ex07_fk_ik_closed_loop.py`](./ex07_fk_ik_closed_loop.py) | **数学闭环验证**：取一个测试姿态 `[swing=10°, boom=30°, arm=50°, bucket=-60°]` → FK 解铲尖 `(x,y,z)` → IK 反解关节角 → 对比两组关节角累计误差；要求 4 关节 `|Δ| < 1°` | `FK→IK→FK closed loop: Δθ < 1° PASS` |
| 08 | [`ex08_cartesian_mover_demo.py`](./ex08_cartesian_mover_demo.py) | 笛卡尔层 **MoveResult + `__bool__()` 简写** 演示：`from_config()` 拿到 `ctx["mover"]`，执行 `move_with_bucket(0.9 m, 0.0 m, -0.2 m, -60°)` 一段下挖动作，`if r:` 判到位 | MoveResult 三行指标，`success=True` |
| 09 | [`ex09_task_builders_action_library.py`](./ex09_task_builders_action_library.py) | **高层动作库剧本** 演示：调用 `build_single_dig_dump_script(dig_x=1.0, dig_z=-0.25, dump_yaw_deg=60.0)` 生成 20+ 步的挖掘—回转—卸料剧本，只打印步数与每步描述，不实际下发硬件 | 剧本 `len(steps)` + 首末步描述摘要 |
| 10 | [`ex10_physical_240mm_delta_L.py`](./ex10_physical_240mm_delta_L.py) | **防止"配置假加载"的硬核验证**：构造一份修改版配置，把 `link_geometry.L2`（大臂下段）从 0.60 m 改到 0.84 m（**+0.240 m = 240 mm**），分别用默认配置与修改配置做 FK；最终测铲尖 `tip_3d` 的 ΔL。要求落在 `[235, 245] mm` 区间才算 PASS | `delta_L = 240.00 mm` + `TR5.5 accept range [235,245] mm` |

### 4.4 独立预标定工具链（ex11 + ex12，用户反馈：不需每次都运行）

> 对标 v5 `tf_calibration_gui.py` 的无头 SSH 版本，两者输出格式 8 字段
> 完全一致，可直接互相替换。

| # | 脚本名 | 功能 | 关键输出 |
|---|--------|------|---------|
| 11 | [`ex11_sensor_tf_calibration_cmdline.py`](./ex11_sensor_tf_calibration_cmdline.py) | **命令行滑条标定器**：仅标准库（`argparse + sys.stdin`），无 Tk / GUI，**SSH 无头直接用**。进入 `cmd> ` 提示符后支持：<br>`x+ / x- / y+ / y- / z+ / z-` — 平移（步长由 `--step-x` 指定，默认 0.001 m）<br>`rx± / ry± / rz±` — 滚转 / 俯仰 / 偏航（`rz=yaw`，步长由 `--step-r` 指定，默认 0.001 rad）<br>`show` — 打印当前 6 DOF 与父子 frame<br>`save` — 追加一行 8 字段到 `--output`：`x y z yaw_rad pitch_rad roll_rad parent_frame child_frame`（空格分隔）<br>`quit` / Ctrl-C — 安全退出。默认初值已填 PaceCat M300-E 后向单线雷达的**出厂倒装值**（roll=3.0316 rad） | 交互提示符；`save` 后 `/tmp/tf_calibration_record.txt` 追加一行 8 tokens |
| 12 | [`ex12_sensor_tf_record_to_yaml.py`](./ex12_sensor_tf_record_to_yaml.py) | **record.txt → v15 配置闭环器**（AC-10）。读 ex11 生成的 record.txt（多行 + 空行忽略），每行 8 字段解析为 `ExtrinsicsEntry`，输出一份**与 V15Config 原始 dict 兼容**的 JSON（不是 YAML，避免 PyYAML 依赖）。输出完成后**立即自检**：调 `load_config(out_json_path)` 实际回读配置，对每条记录的 6 个 DOF 做 `|Δ| < 1e-4` 断言，打印 `PASS` 才 `exit 0` | 输出 JSON 文件路径；最后一行 `PASS` 或 `FAIL`，exit code 对应 |

典型工作流（在被标定挖掘机 SSH 终端执行）：

```bash
# Step A：现场标定（反复 x+ rz- 观察 RViz 里的点云位置，对齐车身）
python3 ex11_sensor_tf_calibration_cmdline.py \
    --sensor-id lidar_single_rear \
    --parent base_link --child lidar_single_rear_link \
    --default-x -0.5500 --default-y -0.2000 --default-z 1.2712 \
    --default-ry 0.0532 --default-pp 0.0349 --default-rr 3.0316 \
    --output /tmp/tf_calibration_record.txt

# Step B：导入 v15 配置 + 1e-4 自检闭环
python3 ex12_sensor_tf_record_to_yaml.py \
    --input /tmp/tf_calibration_record.txt \
    --sensor-id-override lidar_single_rear \
    --chassis-sn EXC-车间-007 \
    --out /opt/v15_configs/extrinsics_EXC-车间-007.json

# Step C：业务代码直接吃这份配置
# ctx = from_config("/opt/v15_configs/extrinsics_EXC-车间-007.json", init_sensors=True)
```

### 4.5 Phase3 验收脚本（ex13 / ex14 / ex15 —— 本轮新增 10 AC）

| # | 脚本名 | 对应 TR / AC | 功能 | 关键输出（PASS 条件）|
|---|--------|--------------|------|---------------------|
| 13 | [`ex13_workspace_reachability_AC1_2_3.py`](./ex13_workspace_reachability_AC1_2_3.py) | **TR6.1 (AC-1)**<br>**TR6.2 (AC-2)**<br>**TR6.3 (AC-3)** | `WorkspaceChecker` 三类场景演示：<br>① 边界内 dig 点（1.0,0,-0.05）→ 通过<br>② 3.0m 远点 transit → 超限拦截<br>③ z=-0.5 地下点 transit → ground_penetration 拦截 | ① `success=True`（边界内）<br>② `outer_radius_exceeded ∈ reasons`<br>③ `ground_penetration ∈ reasons`；附带 `closest_point` 展示 |
| 14 | [`ex14_trajectory_planner_AC4_5.py`](./ex14_trajectory_planner_AC4_5.py) | **TR6.4 (AC-4)**<br>**TR6.5 (AC-5)** | `TrajectoryPlanner` 双策略：<br>① `strategy="lerp"` boom 45° 线性等分 10 段<br>② `strategy="trapezoidal"` 梯形速度曲线 max_speed ≤ 1.05×vmax 末速度≈0 | ① 段数=10，每相邻 waypoint Δθ 线性误差 < 0.01°<br>② 时间单调递增；`max_speed_deg/s = 20.00`（与 vmax 比 ≤ 1.05×） |
| 15 | [`ex15_three_control_interfaces_AC6_7_8.py`](./ex15_three_control_interfaces_AC6_7_8.py) | **TR6.6 (AC-6)**<br>**TR6.7 (AC-7)**<br>**TR6.8 (AC-8)** | 三条控制接口端到端演示：<br>① 接口1：`mover.move_to_point(1.2,0,0.8)` → FK 3D 误差≤5cm<br>② 接口2：`move_to_dump_transit(up=0.3m, swing=30°)` → z 抬升 + yaw 到位<br>③ 接口3：`dump_material(shake_count=1)` → 半开→半推→全开+抖斗 5 步 | ① 实际 FK tip 误差 = 3.46 cm ≤ 5 cm<br>② z ∈ [0.65, 0.75] m；`swing_yaw ∈ [28, 32] °`<br>③ steps ∈ [4,6]；`bucket_arm ∈ [-99,-91] °`；`arm_boom ∈ [26,34] °` |
| 16 | [`ex16_ros_urdf_control_node.py`](./ex16_ros_urdf_control_node.py) | **用户 rospub 端到端验证节点**（URDF / RViz + rclpy） | ① **手动 rospub 位姿控制** `/v15/cmd/joint_states` → 三层限位夹紧 → 驱动 URDF；<br>② **订阅空间点自主规划** `/v15/cmd/target_point` → 可达域预检 → IK → 双策略轨迹 → 执行；<br>③ **RViz 可视化发布**：`/v15/planned_path` (Path) `/v15/target_point` (Marker 绿/红) `/v15/closest_point` (Marker 橙) `/v15/execution_waypoints` (JointState) `/v15/reachability_reasons` (String JSON) | ① 启动后 0 ImportError；<br>② `ros2 topic pub /v15/cmd/target_point geometry_msgs/PointStamped ...` 后 RViz 出现 Path + Marker，URDF 随动；<br>③ 超限点 → target=红球 + closest=橙球 + reasons 包含 failure code |

---

<!-- prettier-ignore -->
> ⚠️ **⚠️ 强 Warning：`mode="dig"` 仅用于明确的挖掘动作，其余一律 `mode="transit"`！**
> - `check_point_reachable(x,y,z, mode="transit")` 会同时开启 **`ground_penetration` 检查** + **`transit_min_z` 最小高度检查**；任何可能穿地或在地面附近的点会被立即拦截，避免碰地。
> - `mode="dig"` 会**关闭** ground_penetration + transit_min_z 两层（因为挖掘就是要铲进地下），只保留 swing 限位 / 腕点几何 / 奇异 margin 三层。
> - 业务层**不明确是挖掘点时，一律传 `mode="transit"`**，否则会把 "转运路径点" 错误地判为可达，导致实际碰地。
> - 典型正确用法：`dig_entry → dig_scrape` 用 `mode="dig"`；`lift_up → swing → dump` 全程都用 `mode="transit"`。

---

## 5. 方向语义速查表（FR-2.1，ex02 里实际验证过）

`{joint}_move(bool direction, float angle_deg)` 的方向映射，避免业务层写
错正负号。每个函数第一个布尔参数名就是关节的"正向"含义：

| 函数 | `direction=True` 语义 | 关节变量变化 | 物理含义 |
|------|----------------------|-------------|---------|
| `swing_move(right_or_left, …)` | `True = 顺时针（车舱右转，+）` | `swing_yaw += angle_deg` | 俯视下，铲尖向右摆 |
| `boom_move(up_or_down, …)` | `True = 抬大臂（-）` | `boom_swing -= angle_deg` | 大臂向空中抬起，整体 X 缩短、Z 变高 |
| `arm_move(up_or_down, …)` | `True = 收小臂（+）` | `arm_boom += angle_deg` | 小臂折回来，铲尖靠近机身上方 |
| `bucket_move(up_or_down, …)` | `True = 收斗/卷铲（+）` | `bucket_arm += angle_deg` | 铲斗向上卷，适合装料；外扬卸料请传 `False` |

---

## 6. 常见问题

**Q1：为什么直接 `python3 ex01_joint_limits_three_layer_clamp.py` 提示
`ModuleNotFoundError: No module named 'v15_action_task'`？**

你可能从别的目录 `cd` 进来后，脚本被以模块方式（非文件方式）执行了，
或删改了脚本顶部对 `ensure_v15_importable()` 的调用。解决方式：
确认脚本头部有下面 3 行（默认模板都有），并**在当前 `examples/`
目录下用 `python3 文件名.py` 运行**：

```python
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from examples._util_import_helper import ensure_v15_importable
ensure_v15_importable()
```

**Q2：`_run_all_checks.py` 的 TR3.2 报 `Found 1 line in README.md` 怎么办？**

AC-6 的真实范围是**不允许 Python / 配置文件 `import / from shandong.v0~v14`**，
README 中作为解释说明出现的字符串（例如 `from v0_..v14_` 的 grep 解释
文字）不算违规。最新脚本已显式跳过 `.md` 文件，如果使用的是旧版本请
更新到 `_run_all_checks.py` 最新版再跑。

**Q3：ex10 的 ΔL 为什么不是"恰好 240 mm"，而是允许 ±5 mm？**

`L2`（大臂下段）变化对铲尖 `|tip|` 的影响是**投影系数**而不是 1:1
关系（`boom_bend_angle_deg = 46°` 默认值会把 L2 分一部分到 Y 轴）。
测试姿态取全伸展（`boom=arm=bucket=swing=0`）时投影比接近 1，因此
`[235, 245] mm` 是一个既能捕获"配置假加载"错误，又不会因为浮点
舍入误差产生假阳性的合理区间。

## 7. Next steps

- 需要接入真实硬件：参考 v15 根 README 的 `§9. 扩展：接真实硬件`，
  把 `HardwareSerialAdapter` 放在 `control_core/` 目录下再注册即可，
  这里所有 examples 的 `adapter_backend="ros"` 路径 100% 复用。
- 需要扩展更多传感器：在 `config/default_config.yaml` 的 `sensors`
  节直接加条目（`lidar_*` / `cam_*`），`SensorsConfig.by_id()` 会
  自动识别，不需要改 `SensorManager` 代码。
- 需要把 ex11/ex12 的导入配置和其他 6 大节（限位/连杆/ROS 协议等）
  合并成一份完整机型 YAML：直接把 ex12 输出 JSON 的 `extrinsics`
  数组拷贝到你的完整 YAML / JSON 里，`load_config()` 会把缺失节用
  三级兜底补齐。
