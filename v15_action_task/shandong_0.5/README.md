# `shandong_0.5` 单关节挖掘机两个独立功能库（不修改 v15_action_task 任何原始文件）

## 约束背景

本 `shandong_0.5` 版本的挖掘机硬件特性：**只能单关节运动，禁止任何两个以上关节同时运动的复合动作**。

因此：

1. 不使用 v15_main 里的 CartesianMover（它内部会同时下发多个关节复合动作的 trajectory 插值）。
2. 所有 Point→关节 目标 都在执行阶段**强制按 swing_yaw → boom_swing → arm_boom → bucket_arm 顺序单关节串行**执行，永远不会出现 2 个关节同时动。
3. 但**完全复用 v15 的所有数学/协议实现**（零改动 v15 源码，只读 import）：
   * `ForwardKinematics` 正向运动学
   * `InverseKinematics` 逆向运动学（含自动搜索铲斗角）
   * `WorkspaceChecker` 5 层可达域（支持 `dig` / `transit`）
   * `config/default_config.yaml` 60FED calibrated 机型连杆/限位/WS 全部参数
   * `URDFController` + `RosV14Adapter`（与 v14 URDF / `describe_60FED_calibrated.urdf` 完全对齐）

## 目录结构

```
shandong_0.5/
├── control_lib/                  # 两个独立功能库（核心就是这部分）
│   ├── __init__.py               # 导出 JointAngleController / SingleJointPointMover
│   ├── angle_control.py          # ★ 库 1：角度控制（单关节安全运动 + 4 段串行）
│   └── point_control.py          # ★ 库 2：目标点控制（WS → IK → 库1 串行单关节）
├── launch/
│   └── display_calibrated.launch.py  # 纯复用 calibrated URDF 的 URDF+RViz2 显示
└── scripts/                      # 2 个 smoke test（真机联调 + Mock 离线）
    ├── smoke_angle_ctrl.py       # 先跑这个验证库1 能单关节动
    └── smoke_point_ctrl.py       # 验证库2 Point→串行单关节→最终误差
```

## 库 1：角度控制（单关节） — `JointAngleController`

**对外 2 个接口：**

| 接口 | 作用 | 约束保证 |
| --- | --- | --- |
| `.move_joint(joint_name, deg, *, tolerance_deg, timeout_s, wait_blocking)` | 运动单个语义关节（度）| 只改 1 个关节值下发，其他 3 个关节用当前 feedback 填充，不会出现复合 |
| `.move_pose_serial(target_pose_deg_dict, *, order, ...)` | 4 关节目标字典 → 按 `order` 顺序逐关节调用 `.move_joint` | 每一步只动 1 个关节，且上一步到位（或超时失败）才进入下一步 |

语义关节顺序（与 v15 一致，不可更改）：
```
swing_yaw, boom_swing, arm_boom, bucket_arm
```

导入方式（因为 `shandong_0.5` 位于 `v15_action_task/shandong_0_5/` 下，所以推荐用子包路径）：
```python
from v15_action_task.shandong_0_5.control_lib import (
    JointAngleController,     # 库 1
    SingleJointPointMover,    # 库 2
)
```
或在 src/shandong/v15_action_task/shandong_0.5 下直接运行脚本（已配置 sys.path 自动插入 `src/shandong`）。
默认 order = `swing_yaw → boom_swing → arm_boom → bucket_arm`（先回转 → 大臂 → 小臂 → 铲斗）。

限位夹紧：目标关节角度在下发前**必过 v15 `clamp_pose()` 限位**（与 [default_config.yaml joint_limits](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/config/default_config.yaml#L40-L55) 对齐），目标超限会自动夹紧并打印 `[angle_ctrl] xxx clamped ...` 警告，不会往硬件发超限位。

## 库 2：目标点控制（Point→4 段串行） — `SingleJointPointMover`

**对外主接口：**
```python
.move_to_point_serial(
    (x, y, z),              # base_link 铲尖 3D
    mode="dig"|"transit",   # 决定 WS.z 过滤：dig ≥ -0.10m，transit ≥ +0.30m
    auto_use_closest_if_unreachable=False,  # 目标不可达时 True=自动用最近可达点
)
```

内部 4 步固定流程（本库核心，永不出现复合动作）：
1. **WS 可达域预检（5 层）**：失败时返回 `reachable=False` + `closest_reachable_point`；若 `auto_use_closest=True` 则替换继续
2. **IK 求解**：默认自动搜索 `bucket ∈ [-70°, +10°]` 17 个候选点找最平稳解；或传 `bucket_guess_deg=` 指定铲斗绝对角
3. **4 段串行单关节执行**（调用库 1 `JointAngleController.move_pose_serial`，顺序 `swing→boom→arm→bucket`）
4. **FK 闭环校验**：运动后再 ForwardKinematics 算铲尖，与目标做 3D 距离误差，≤ 5cm 判定成功（v15 TR6.6 同款容差）

返回 `SingleJointMoveResult` dataclass，全部字段名与 v15 MoveResult 对齐，方便日志 grep：
```
success, target_xyz, mode,
reachable, reachability_reasons, closest_reachable_point,
ik_success, ik_reason, ik_target_pose_deg, bucket_angle_deg_used,
final_tip_xyz, final_pose_deg, tip_error_3d_m,
order, per_joint (每关节 1 条 dict), waited_total_s, reason
```

## 启动方式（真机 + Mock 自检）

### Step 0：确认 1 个参数顺序不能写反

launch **必须** `use_joint_state_publisher:=false`（顺序写反会变成 `use_state_joint_publisher:=false` → 拉起 joint_state_publisher_gui 抢发 /joint_states，库 1/库 2 的关节下发会被 GUI 抢覆盖导致不动）。

### Step 1：起 URDF + RViz2（任何一个终端）

```bash
source /opt/ros/humble/setup.bash
source /media/libo/libo_sn7100/ubuntu2204/shandong_ws/install/setup.bash
ros2 launch shandong_0_5 display_calibrated.launch.py use_joint_state_publisher:=false
```

URDF 使用的就是你指定的 **`describe_60FED/urdf/describe_60FED_calibrated.urdf`**（launch 里纯 import，不复制/不修改原文件）。

### Step 2A：验证库 1（角度控制 — 单关节 smoke）

```bash
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/scripts
python3 smoke_angle_ctrl.py
# 或者离线自检（不启 ROS 节点）：
python3 smoke_angle_ctrl.py --use-mock
```

会依次执行：
- 单独 swing_yaw +30°
- 单独 boom_swing +20°
- 单独 arm_boom +60°
- 单独 bucket_arm -60°
- 完整 4 关节字典 → 串行 swing(-45°)→boom(-5°)→arm(+100°)→bucket(0°)

RViz 里你能观察到：**每次只有 1 个关节动，其他 3 个完全不动**（就是这个版本硬件的要求）。

### Step 2B：验证库 2（目标点控制 — Point→串行 smoke）

```bash
cd /media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/scripts
# 默认目标点 (1.2, 0, 0.8)m（正前方抬臂）
python3 smoke_point_ctrl.py
# 离线 Mock 自检：
python3 smoke_point_ctrl.py --use-mock

# 指定其他目标点：
python3 smoke_point_ctrl.py --x 1.45 --y 0 --z 0.6
python3 smoke_point_ctrl.py --x 0.9  --y 0 --z -0.08 --mode dig       # 下挖必须 mode=dig
python3 smoke_point_ctrl.py --x 2.0  --y 0 --z 0.8 --auto-closest      # 太远不可达 → 自动用 closest_reachable_point
```

RViz 里顺序应观察为：先转回转 → 再动大臂 → 再动小臂 → 最后铲斗微调；**任何中间时刻只有 1 个关节轴在动**。

## 可达域范围（与 README 第 8 章 1:1）

和 v15 完全一致（WS/IK 都是同一个 v15 只读实现）：

| 维度 | 推荐安全执行区间（不会被自动回拉）| 备注 |
| --- | --- | --- |
| 水平半径 r=√(x²+y²) | [0.40, 1.50] m | |
| x / y | [−1.50, +1.50] m，且 r ≤ 1.50 | 极限点 (1.45,0,0.6) 直接抄用 |
| z（dig 模式） | [−0.05, +1.65] m，同时 r ∈ [0.40,1.50] | 默认 min_ground_z = -0.10，改成 -1.00 可下挖 1m |
| z（transit 模式） | [+0.35, +1.65] m，同时 r ∈ [0.40,1.50] | 转运禁止刮地 |

## 保证：0 行修改 v15_action_task 的任何原始文件

- 所有新增文件仅放在 `/shandong_0.5/` 目录下（`control_lib/*.py` + `launch/*.launch.py` + `scripts/*.py`）
- 对 v15 的使用全是 `from v15_action_task import X`（只读引用，不 import 不修改 `.py` / `.yaml`）
- v15 的 ast + 43TR 门禁 100% 保持 PASS（请自行每次改 v15 时再跑 `python3 examples/_run_all_checks.py`；shandong_0.5 代码不跑 43TR，因为是独立库）

