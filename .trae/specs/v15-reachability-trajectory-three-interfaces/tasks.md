# v15 可达域 · 轨迹 · 三接口 实现任务清单 (Tasks)

> 对应 spec.md: `./spec.md`  
> 父规格版本: v1.0  
> 任务分解规则: **每 1 任务 = 1 信息边界 / 1 主要编辑目录 / ≤ 500 行代码 + ≤ 300 行测试**  
> 依赖图: T1 → T2 → T3 → T4（严格 DAG，不可跨层并行）

---

## Task 1: 可达域初始化与 5 层检查（WorkspaceChecker）

### T1.1 新建 motion/workspace.py
- **Priority**: HIGH
- **Dependencies**: 无（最先做，独立 dataclass 纯算术）
- **Status**: pending
- **Scope**: `motion/workspace.py` 单文件，~300 行
- **实现要点**：
  1. dataclass `ReachabilityReport(success: bool, reasons: list[str], wrist_radius_m: float, inner_radius_m: float, outer_radius_m: float, margin_m: float, mode_used: str, checked_xyz: tuple[float, float, float])`
  2. dataclass `WorkspaceChecker`，字段：`link_geometry: LinkGeometryConfig`（直接复用 loader 中的），`joint_limits: JointLimitsConfig`，`workspace_cfg: WorkspaceConfig`（新增 config 节 min_ground_z/min_transit_z 两个默认）
  3. classmethod `from_config(cfg: V15Config) -> WorkspaceChecker`
  4. 实例方法 `_compute_wrist_d(target_xyz, bucket_abs_angle_deg: float = -45.0) -> tuple[float, float, float]`（返回 wrist_xz / wrist_d）
  5. 实例方法 `check_point_reachable(xyz, *, mode='auto', bucket_guess_deg: float = -45.0) -> ReachabilityReport`（5 层过滤逐一执行，fail 就 append reason，不短路以便上层一次看到全部问题）
  6. 实例方法 `sample_reachable_height(x, y, *, z_min=-0.1, z_max=2.0, step=0.02) -> tuple[float, float] | None`（二分搜索第一个可达的 z）
  7. 实例方法 `closest_reachable_point(target_xyz, *, shrink_factor=0.95, max_iter=10) -> tuple[float, float, float] | None`（等比例缩回球内）
- **Test Requirements**:
  - **TR1.1 (rule, parent AC-1)**: 默认 cfg，`check_point_reachable(wrist_d=0.24 + 0.02m 边界内点, mode='wrist') success=True`
  - **TR1.2 (rule, parent AC-2)**: `(3.0,0,0.5) -> success=False` 且 reasons 含 `"outer_radius_exceeded"`
  - **TR1.3 (rule, parent AC-3)**: `(1.0,0,-0.5) mode='auto' -> False + "ground_penetration"`；`mode='dig'` 后 ground 检查被跳过

---

### T1.2 config/default_config.yaml + loader.py 扩展两节
- **Priority**: HIGH
- **Dependencies**: 无（和 T1.1 并行，改配置层不动运行时）
- **Status**: pending
- **Scope**: `config/default_config.yaml`（加注释 + 2 节） + `config/loader.py`（2 新 dataclass + V15Config 2 字段 + BUILTIN 同步） + `config/__init__.py` 导出
- **实现要点**：
  1. **AC-10 YAML 注释块（joint_limits 正上方）**：按 spec §4.1 Constraint 2 写 4 行方向说明，每行格式 `# {joint}_move(True, +angle) = {semantic}`，与 README_Agreement §2.2 逐字对齐
  2. 新增 `workspace:` 节：`min_ground_z: -0.1`，`min_transit_z: 0.3`，`singularity_margin_ratio: 0.01`，`singularity_margin_min_m: 0.02`
  3. 新增 `trajectory:` 节：`default_strategy: lerp`，`lerp_step_deg: 5.0`，`max_speed_deg_s: {swing_yaw: 15, boom_swing: 20, arm_boom: 30, bucket_arm: 30}`，`accel_deg_s2: {swing_yaw: 30, boom_swing: 40, arm_boom: 60, bucket_arm: 60}`
  4. `loader.py` 新增 `WorkspaceConfig`（4 字段） + `TrajectoryConfig`（4 字段）dataclass；`V15Config` 新增 `workspace` + `trajectory` 两字段；`BUILTIN` 三级兜底 dict 同步两节；`build_default_config()` 对应字段构造
- **Test Requirements**:
  - **TR1.4 (rule, parent AC-10)**: `default_config.yaml` 读文本，`re.search(r"^# swing_move\(True, \+angle\) =", text, re.MULTILINE)` 4 条 joint 注释全部命中
  - **TR1.5 (rule, parent FR-1.1)**: `load_default_config().workspace.min_ground_z == pytest.approx(-0.1, abs=1e-4)` 且 `.trajectory.default_strategy == "lerp"`

---

### T1.3 各层聚合导出
- **Priority**: MEDIUM
- **Dependencies**: T1.1 + T1.2
- **Status**: pending
- **Scope**: `motion/__init__.py`（导出 WorkspaceChecker/ReachabilityReport） + `v15_action_task/__init__.py`（`__all__` 追加 2 符号 + from 内导入）
- **Test Requirements**:
  - **TR1.6 (rule)**: `python3 -c "from v15_action_task import WorkspaceChecker, ReachabilityReport"` exit 0

---

## Task 2: 逆解包装 + 双策略轨迹规划器

### T2.1 新建 motion/trajectory.py
- **Priority**: HIGH
- **Dependencies**: T1.1 （需要 ReachabilityReport 类型注解）
- **Status**: pending
- **Scope**: `motion/trajectory.py` 单文件，~400 行
- **实现要点**：
  1. dataclass `JointTrajectoryPoint`（4 稳定字段：pose_deg, duration_s, t_from_start_s, waypoint_index）
  2. dataclass `IKPlanningResult`（success, pose_deg, ik_solution, reason）
  3. 函数 `planning_ik_solve(xyz, bucket_angle_deg|None, cfg, ik, workspace: WorkspaceChecker|None, *, workspace_mode='auto', skip_workspace=False) -> IKPlanningResult`（先 workspace check → 再 ik）
  4. class `TrajectoryPlanner`：
     - `from_config(cfg: V15Config)`
     - `plan_segment(start_pose_deg, end_pose_deg, *, strategy, lerp_step_deg, max_speed_dct, accel_dct, min_seg_dur=0.2) -> list[JointTrajectoryPoint]`
     - 内部 helper `_lerp_plan(...)`：取 Δθ_max → N=ceil(Δθ_max/step) → 逐 waypoint 线性插值 4 关节；每段 duration= step / max_speed_dct["boom_swing"]；t_from_start_s 前缀和
     - 内部 helper `_trapezoidal_plan(...)`：对每个关节独立解三角速度分布（t_accel, t_cruise, t_decel），取 max_total_t 作为统一总时长；按 50~100 waypoint 均匀等分；每段 duration = total/N；pose_deg 按关节梯形位置函数独立计算（四关节非同步时每关节按自己的速度曲线，保证 max_speed 和 accel 不超限）
- **Test Requirements**:
  - **TR2.1 (rule, parent AC-4)**: LERP 段数 10 + 线性插值误差 < 0.01°
  - **TR2.2 (rule, parent AC-5)**: 梯形计划 t 严格递增 + 总时长误差 < 0.5s；且 `max(|Δpose_joint| / Δt_segment_joint) <= max_speed_joint * 1.05`（速度不超 5% 误差裕度）
  - **TR2.3 (rule, parent NFR-3)**: 100 次 `plan_segment` 平均耗时 lerp < 1ms；trapezoidal < 10ms（perf_counter 测，允许 CI 慢机 × 2 系数）

---

### T2.2 聚合导出
- **Priority**: MEDIUM
- **Dependencies**: T2.1
- **Status**: pending
- **Scope**: `motion/__init__.py` + `v15_action_task/__init__.py`，追加 `JointTrajectoryPoint / TrajectoryPlanner / planning_ik_solve / IKPlanningResult`
- **Test Requirements**:
  - **TR2.4 (rule)**: `python3 -c "from v15_action_task import TrajectoryPlanner, JointTrajectoryPoint"` exit 0

---

## Task 3: 三控制接口对外暴露

### T3.1 CartesianMover 扩展 move_to_point（接口1）
- **Priority**: HIGH
- **Dependencies**: T1.3 + T2.2（WorkspaceChecker + TrajectoryPlanner）
- **Status**: pending
- **Scope**: `motion/cartesian_mover.py` 扩充 ~300 行（MoveResult 扩展 3 可选字段 + CartesianMover.move_to_point 新方法）
- **实现要点**：
  1. `MoveResult` dataclass **在尾部追加 3 可选字段**（STABLE 不破，前面 8 字段顺序不动）：
     - `waypoints: list[JointTrajectoryPoint] | None = None`
     - `workspace_checked: bool = False`
     - `workspace_report: object | None = None`（或用真实类型，若 import 无环）
  2. 新方法 `CartesianMover.move_to_point(x, y, z, *, bucket_angle_deg=None, strategy='lerp', workspace_mode='auto', skip_workspace_check=False, wait_per_waypoint_s=0.0, tolerance_deg=1.0, timeout_total_s=None, workspace: WorkspaceChecker | None = None)`，严格实现 FR-4.2 8 步流程；第 ⑥ 步的时间同步方式：`ideal_wake = t0 + next_t_from_start_s; time.sleep(max(0, ideal_wake - monotonic()))`（精度好于单段 sleep，避免累积误差）
- **Test Requirements**:
  - **TR3.1 (rule, parent AC-6)**: 接口1 (1.2,0.2,0.4) 闭环 FK 误差 ≤ 5cm（Mock 后端）
  - **TR3.2 (rule, parent NFR-2)**: 原 `CartesianMover.move()` + `move_with_bucket()` 所有形参/默认值不变，`ex08_cartesian_mover_demo.py` exit 0

---

### T3.2 新建 action_library/composites/dig_dump_actions.py（接口2+3）
- **Priority**: HIGH
- **Dependencies**: T3.1（接口2内部调用 move_to_point 做抬臂/回转的精确插值）
- **Status**: pending
- **Scope**: `action_library/composites/dig_dump_actions.py` ~300 行 + `action_library/composites/__init__.py` 导出 2 函数 + `action_library/__init__.py` + `v15_action_task/__init__.py`
- **实现要点**：
  1. 统一 ctx/参数解包 helper：`_unpack_ctx(ctx)` → `ctl, ik, mover, cfg, workspace, trajectory_planner`（缺省自动 from_config 内拿）
  2. `move_to_dump_transit(ctx, *, dig_point_xyz=None, up_m, swing_deg, dump_height_safety_margin_m=0.05, strategy='lerp') -> MoveResult`：
     - 二分迭代 boom_swing：目标 Δz=up_m，0 次近似用 `fk(start).z`，每次迭代 `boom_swing ± Δθ` 调整 Δz，≤ 10 次 或 误差 < 5mm
     - swing_yaw 直接加 swing_deg（正值顺时针，经语义函数包装：`semantic_moves.swing_move(right_or_left=True, angle_deg=swing_deg)` 等价，**但实际改 pose 以便合成一段轨迹**）
     - 合成 2 段 plan_segment 合并 waypoints；最后 1 次到位检查 tolerance_deg=1.0
     - 返回 MoveResult.waypoints 含两段
  3. `dump_material(ctx, *, bucket_half_open_deg=45, arm_half_push_deg=30, bucket_full_open_deg=50, bucket_shake=True, shake_amplitude_deg=5, shake_times=3, settle_s_per_step=0.3, strategy='lerp') -> list[MoveResult]`：
     - **严格走 semantic_moves.py 封装（FR-6.2）**，即：
       - step1 = `semantic_moves.bucket_move(ctl, cfg, False, bucket_half_open_deg, strategy)` 内部实际调用（不直接改 pose，以防符号搞反）
       - step2 = `semantic_moves.arm_move(ctl, cfg, False, arm_half_push_deg)`
       - step3 = `semantic_moves.bucket_move(ctl, cfg, False, bucket_full_open_deg)`
       - step4-shake = 循环 shake_times: `bucket_move(True, shake_amp); sleep(settle/2); bucket_move(False, shake_amp); sleep(settle/2)`
     - settle_s_per_step 每步 sleep 累加
     - 返回 list（4~6 个 MoveResult，取决于 shake 是否开启）
- **Test Requirements**:
  - **TR3.3 (rule, parent AC-7)**: 接口2 up=0.3m, swing=30° 后 z ∈ [0.65, 0.75], swing_yaw ∈ [28, 32]
  - **TR3.4 (rule, parent AC-8)**: 接口3 最终 bucket_arm ≈-95°, arm_boom≈60-30=30°（误差±4°），返回 list len ∈ [4,6]，且每个元素的 success=True
  - **TR3.5 (rule, parent §4.1 STABLE 不破)**: `semantic_moves.py` 所有函数签名/默认值/方向语义保持不动，`ex02_semantic_direction_AC7_self_assert.py` exit 0

---

### T3.3 v15 顶层 from_config 聚合
- **Priority**: MEDIUM
- **Dependencies**: T3.2
- **Status**: pending
- **Scope**: `v15_action_task/__init__.py` 的 `from_config()` 函数，**尾部追加 2 返回键（PROVISIONAL，不破坏 7 个 STABLE 键）**
- **实现要点**：
  1. `from_config()` 新增 2 形参（默认 False，**不破旧 API**）：`init_workspace: bool = False`、`init_trajectory_planner: bool = False`
  2. 返回 dict 新增键 `"workspace_checker"` 和 `"trajectory_planner"`（分别在 init_* = True 时存在，False 时缺省不出现？ 或默认 None → 选择后者更安全 STABLE：旧代码 `if "sensor_manager" not in ctx` 风格不被破坏；推荐 ctx.get("workspace_checker") 方式访问）
  3. `__all__` 顶层追加：`"move_to_dump_transit"`, `"dump_material"`, `"WorkspaceChecker"`, `"ReachabilityReport"`, `"TrajectoryPlanner"`, `"JointTrajectoryPoint"`, `"planning_ik_solve"`, `"IKPlanningResult"`（共 8 个）
- **Test Requirements**:
  - **TR3.6 (rule, parent NFR-2 TR4.2 旧 API 不破)**: 调用 `from_config()` **不**传任何新形参 → 返回 dict 7 个 key 名称位级相等（与 ex04/ex06 完全一致），无额外键；`ctx.get("workspace_checker") is None`
  - **TR3.7 (rule)**: 传 `init_workspace=True, init_trajectory_planner=True` → `isinstance(ctx["workspace_checker"], WorkspaceChecker)`，`isinstance(ctx["trajectory_planner"], TrajectoryPlanner)`

---

## Task 4: 验收脚本 + 文档同步 + _run_all_checks 43/43

### T4.1 examples/ex13_workspace_reachability_AC1_2_3.py
- **Priority**: HIGH
- **Dependencies**: T1.3
- **Status**: pending
- **Scope**: 单脚本 ~120 行（自断言 sys.exit(0/1)，含顶部 ensure_v15_importable）
- **实现要点**：
  1. 用默认 from_config(init_workspace=True) 拿 workspace
  2. 构造 3 个点（AC-1 边界内 OK / AC-2 外太远 FAIL+outer / AC-3 地面 FAIL+ground，mode=dig 跳过 ground）
  3. 所有断言不满足直接 print(msg) + sys.exit(1)；通过 sys.exit(0)
- **Test Requirements**:
  - **TR4.1 (rule)**: `python3 ex13_workspace_reachability_AC1_2_3.py` exit 0

---

### T4.2 examples/ex14_trajectory_planner_AC4_5.py
- **Priority**: HIGH
- **Dependencies**: T2.2
- **Status**: pending
- **Scope**: 单脚本 ~150 行
- **实现要点**：AC-4 段数 10 + LERP 插值误差 < 0.01°；AC-5 梯形 t 严格递增 + 总时长误差 < 0.5s + 速度不超 max_speed*1.05；NFR-3 性能计时
- **Test Requirements**:
  - **TR4.2 (rule)**: `python3 ex14_trajectory_planner_AC4_5.py` exit 0

---

### T4.3 examples/ex15_three_control_interfaces_AC6_7_8.py
- **Priority**: HIGH
- **Dependencies**: T3.3
- **Status**: pending
- **Scope**: 单脚本 ~200 行
- **实现要点**：Mock 后端，三接口顺序执行，AC-6 FK tip 误差≤5cm；AC-7 z ∈ [0.65, 0.75] + swing_yaw ∈ [28,32]；AC-8 最终 bucket_arm ∈ [-99,-91] + arm_boom ∈ [28,32] + len(接口3 list) ∈ [4,6]
- **Test Requirements**:
  - **TR4.3 (rule)**: `python3 ex15_three_control_interfaces_AC6_7_8.py` exit 0

---

### T4.4 examples/_run_all_checks.py 扩展（33→43 TR）
- **Priority**: HIGH
- **Dependencies**: T4.1 + T4.2 + T4.3
- **Status**: pending
- **Scope**: `examples/_run_all_checks.py` 编辑约 60 行
- **实现要点**：
  1. 新增 TR section = "TASK 可达域轨迹三接口"，新增 TR 编号 TR40~TR49（10 条 = AC-1~AC-10 一一对应）：
     - TR40 = AC-1 ex13 断言
     - TR41 = AC-2
     - TR42 = AC-3
     - TR43 = AC-4
     - TR44 = AC-5
     - TR45 = AC-6 ex15
     - TR46 = AC-7
     - TR47 = AC-8
     - TR48 = AC-9（复用旧 grep_negative，不用重复写）
     - TR49 = AC-10：读 `default_config.yaml` 文本，正则匹配 4 行方向注释
  2. `_do_grep_negative` 白名单**保留**原规则：过滤 .md 行；新增显式过滤 "说明文字字符串" 模式：`if "可达域外的作业点" in line or "from shandong.v0" in line in line_comment:`（避免 ex13-15 里的注释/文档字符串触发假阳性）
  3. CLI 新增 `--section phase3` 独立跑 10 条新 AC；默认 `python3 _run_all_checks.py` 跑 43/43 全量
- **Test Requirements**:
  - **TR4.4 (rule)**: `cd examples && python3 _run_all_checks.py --section phase3` → 10/10 PASS
  - **TR4.5 (rule)**: 全量 → `43/43 PASS exit 0`（含 phase0/1/2 的 33 条）

---

### T4.5 文档同步（README.md + examples/README.md）
- **Priority**: MEDIUM
- **Dependencies**: T4.4（脚本全部命名稳定后写文档）
- **Status**: pending
- **Scope**:
  1. `v15_action_task/README.md` 追加 3 节：§3.10 三控制接口签名 + 示例；§4.9 可达域/轨迹/关节方向注释 3 个新 config 节字段表；§8.5 AC-1~AC-10 10 行硬验收表
  2. `v15_action_task/examples/README.md` 追加 §4 表格里的 ex13 / ex14 / ex15 三行；§3 回归命令提示 phase3 独立运行
  3. **非强制，可选低优先级**：`README_Agreement.md` §2 表新增 3 控制接口的 STABLE/PROVISIONAL 行（因为刚创建就改可以，不破用户）
- **Test Requirements**:
  - **TR4.6 (rule)**: `ls v15_action_task/examples/ex13* ex14* ex15*` 3 文件都存在，且 examples/README 中提到这 3 个文件名（grep -c ex13 ≥1 等）
  - **TR4.7 (rule, docs-writer 规范 BLUF)**: README 新增节开头都有 1 段 ≤ 3 行的 BLUF 总结段 + GitHub 警告块（关键安全点：可达域默认 ground_penetration 开启，关闭请显式 mode=dig）

---

## 验收覆盖率追踪表

| Acceptance Criterion | 覆盖 Task / TR | 状态 |
|---|---|---|
| AC-1 可达边界内点 | T1.1 TR1.1 / T4.1 TR4.4 TR40 | pending |
| AC-2 外半径超限拒绝 | T1.1 TR1.2 / T4.1 TR41 | pending |
| AC-3 地面穿透 mode 行为 | T1.1 TR1.3 / T4.1 TR42 | pending |
| AC-4 LERP 段数 + 线性误差 | T2.1 TR2.1 / T4.2 TR43 | pending |
| AC-5 梯形速度 + 单调 t | T2.1 TR2.2 / T4.2 TR44 | pending |
| AC-6 接口1 闭环 5cm | T3.1 TR3.1 / T4.3 TR45 | pending |
| AC-7 接口2 抬臂+回转 z/swing | T3.2 TR3.3 / T4.3 TR46 | pending |
| AC-8 接口3 卸料角度/步数 | T3.2 TR3.4 / T4.3 TR47 | pending |
| AC-9 零依赖 v0~v14 | T4.4 TR48（复用） | pending |
| AC-10 config 关节方向注释 | T1.2 TR1.4 / T4.4 TR49 | pending |
| NFR-2 旧 33 回归不破 | T3.1 TR3.2 / T3.3 TR3.6 / T3.2 TR3.5 | pending |
| NFR-3 性能 | T2.1 TR2.3（ex14 内部计时） | pending |

---

## 完成定义 (Definition of Done)
1. `_run_all_checks.py` → **43/43 PASS exit 0**，旧 phase0/1/2 33 条 100% 保持
2. ex13 / ex14 / ex15 分别独立运行 exit 0
3. `from v15_action_task import (move_to_point, move_to_dump_transit, dump_material, WorkspaceChecker, TrajectoryPlanner, JointTrajectoryPoint)` → ImportError: 0
4. README.md / examples/README.md / default_config.yaml 注释均已同步
5. Spec Mode `review.md` Review 独立检查 pass（Implement 后必走）
