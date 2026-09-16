# v15 可达域 · 双策略轨迹规划 · 三控制接口 规格说明书 (Spec)

> 版本: v1.0  Spec Mode 产物，用户批准前 **禁止实现**  
> 稳定度分类: 所有新增对外接口按 `PROVISIONAL` 处理（只增不减），`MoveResult` / `V15Config` / `from_config` 保持 `STABLE`（零破坏）

---

## 1. 问题、用户与目标

### 1.1 Problem Statement
v15 前 9 个 Task 已交付：配置层三级兜底、8 语义函数、传感器 15 路订阅、12 个 examples 回归 33/33 PASS。但现有运动能力存在 3 个硬缺口：

1. **无可达域守护**：用户给点 (3.0m, 0, 0) 时，现有 `CartesianMover.move()` 只在 IK 阶段返回 `None`，但**不暴露为什么不可达**（是几何太远 / 地面穿透 / swing 超限 / 奇异位姿？），上层任务规划器只能盲猜重试；
2. **无轨迹规划**：`CartesianMover.set_pose(pose)` 是"一步到位"大跳变，液压阀实际执行时会出现 **液压冲击、关节抖动、连杆超额定角速度** 的风险；
3. **无挖-运-卸高层组合语义**：现有 `build_single_dig_dump_script` 是纯"步骤字典"，没有对外暴露 3 个独立可调用接口（单点点到位 / 抬臂+回转到卸料区 / 卸料），业务方调用要自己拼步骤。

### 1.2 Users（调用方）
| 用户 | 调用侧 | 主要用法 |
|---|---|---|
| 任务规划脚本（mode1/mode2）| `v15_action_task` import | 网格采样候选点 → `WorkspaceChecker` 过滤 → 接口1 逐点执行 → 接口2+3 卸料循环 |
| 远程控制上位机（HTTP/ROS 2 Service） | 新程序 `from v15_action_task import move_to_point, dump_material` | 接收 UI 下发 (x,y,z) → 接口1 执行 + 返回轨迹 |
| 离线剧本生成工具 | 纯 Python 无 ROS | 构建 100+ 剧本 → 先做可达域预过滤再写 JSON |

### 1.3 Goals
| Goal | 一句话描述 |
|---|---|
| G1 | 初始化时根据 `config.link_geometry`（L1/L2/L3/offX/offZ）**一次构建可达域边界**，后续 10ms 量级判断任意 (x,y,z) 是否可达 + 返回不可达原因 |
| G2 | IK 求解 + **双策略轨迹规划**（`lerp` 液压均匀型 / `trapezoidal` 加速度限幅型），可切换 |
| G3 | 对外暴露 **3 个独立高层接口**（`move_to_point` / `move_to_dump_transit` / `dump_material`），新程序直接 import 不用拼底层步骤 |

### 1.4 Non-Goals（不做的事）
- ❌ 不做视觉/点云候选点检测（留给上层 v16/v17）
- ❌ 不做模型预测控制 MPC、力控挖掘（液压传感未接入）
- ❌ 不破坏 v15 现有 33 条已通过的回归 AC（33/33 必须保持）
- ❌ **不 import 任何 v0~v14 模块**（`shandong.v[0-14]*` 全部禁止，AC-9 双 grep 强制执行）

---

## 2. 功能需求 (Functional Requirements, FR)

### FR-1 可达域初始化与 5 层检查（任务1）
- **FR-1.1 初始化**：`WorkspaceChecker.from_config(cfg: V15Config)` 读取 `cfg.link_geometry` 6 个字段（L1_boom / L2_arm / L3_bucket / offset_x_bucket / offset_z_bucket / swing_joint_z），计算：
  - 下边界 `inner_radius_m = |L1 - L2|`
  - 上边界 `outer_radius_m = L1 + L2`
  - 腕点奇异 margin = `max(0.02m, (L1+L2)*0.01)`（默认 2cm 或总臂长 1%）
- **FR-1.2 5 层过滤（任何一层失败即 `success=False`，全部原因都要追加到 `reasons: list[str]`）**：
  1. **几何腕点约束**（`mode='wrist'` 时执行，默认执行）：目标点先扣 bucket 长度偏移 → 腕点 `(x_w, z_w)` → `d = sqrt(x_w^2 + z_w^2)` 必须 ∈ `[inner_radius_m - margin, outer_radius_m + margin]`
  2. **swing 回转约束**（默认执行）：`yaw = atan2(y,x)` 必须 ∈ `cfg.joint_limits["swing_yaw"].min` ~ `max`，否则 `reason = "swing_yaw_out_of_range"`
  3. **地面穿透约束**（`mode='dig'` 关闭；`mode='transit'` 时开启，默认 mode=`auto`：`z < 0 → dig`，`z ≥ 0 → transit`）：`z >= cfg.workspace.min_ground_z`（默认 `min_ground_z = -0.1m`，可配置）
  4. **转运最小离地约束**（仅 `mode='transit'` 时执行）：`z >= cfg.workspace.min_transit_z`（默认 `0.30m`）
  5. **奇异位姿拒绝**（默认执行）：`abs(d - inner_radius_m) < margin` 或 `abs(d - outer_radius_m) < margin` → `reason = "near_singularity"`，提示用户改点 ±5cm 重试
- **FR-1.3 对外 3 个纯函数方法**：
  - `check_point_reachable(xyz: tuple[float, float, float], *, mode: Literal['auto','dig','transit','wrist']='auto') -> ReachabilityReport`
  - `sample_reachable_height(x: float, y: float) -> tuple[float, float] | None`：返回该 (x,y) 下 **可达的 z_min, z_max 范围**，供 UI 任务规划器生成候选点
  - `closest_reachable_point(target_xyz) -> tuple[float, float, float] | None`：返回最接近目标点且可达的 (x,y,z)（等比例缩回 `outer_radius_m` 内，或抬 z 到 `min_transit_z` 之上）

### FR-2 单目标点逆解标准化（任务2前半）
- **FR-2.1 统一逆解包装函数**：`planning_ik_solve(xyz, bucket_angle_deg: Optional[float], cfg: V15Config, ik: InverseKinematics) -> IKPlanningResult`
  - `bucket_angle_deg is None`：调用 `ik.search_bucket_angle`（复用 v15 现有，默认 `bucket_range_deg=(-70,10)，num_candidates=17`）
  - 指定 `bucket_angle_deg`：调用 `ik.solve_bucket_pose`
  - 返回字段：`success: bool` / `pose_deg: dict[str, float] | None` / `ik_solution: IKSolution | None` / `reason: str`（同现有 `MoveResult.reason` 风格）
- **FR-2.2 逆解前自动做可达域检查**：FR-1.2 任一层 fail → 直接返回 `IKPlanningResult(success=False, reason=report.reasons[0])`，**不浪费 IK 搜索时间**

### FR-3 双策略轨迹规划（任务2后半）
- **FR-3.1 统一数据结构**：`JointTrajectoryPoint` 4 个稳定字段（PROVISIONAL，只增不减）：
  ```python
  pose_deg: dict[str, float]     # 该路点 4 语义关节角目标（度）
  duration_s: float               # 从上一 waypoint 到本点**推荐段时长**（秒）
  t_from_start_s: float           # 从轨迹开始累计时间（秒，单调递增）
  waypoint_index: int             # 路点序号（0=起点，1..N-1 中间，N-1 终点）
  ```
- **FR-3.2 双策略 TrajectoryPlanner**：`TrajectoryPlanner.from_config(cfg)` 读取配置，提供：
  ```python
  plan_segment(start_pose_deg, end_pose_deg, *,
               strategy: Literal['lerp','trapezoidal'] = cfg.trajectory.default_strategy,
               lerp_step_deg: float = cfg.trajectory.lerp_step_deg,   # 默认 5.0°（液压推荐）
               max_speed_deg_s: dict[str, float] = cfg.trajectory.max_speed_deg_s,
               accel_deg_s2: dict[str, float] = cfg.trajectory.accel_deg_s2,
               min_segment_duration_s: float = 0.2,
              ) -> list[JointTrajectoryPoint]
  ```
  - 策略 `lerp`：在 4 关节空间**最大角位移差值**上按 `lerp_step_deg` 等分（取 Δθ_max 关节的段数 N，其余关节按相同 N 做线性插值），每段 `duration_s = (Δθ_max/N) / max_speed_deg_s["boom_swing"]`（默认 boom 最慢，按它卡周期）；
  - 策略 `trapezoidal`：对每个关节独立算 **加速-匀速-减速** 三段，段时长取所有关节中最大的那个（同步），满足 `max_speed_deg_s[joint]` 与 `accel_deg_s2[joint]` 上界。

### FR-4 控制接口1：单空间点 + 完整轨迹执行（任务3接口1）
- **FR-4.1 对外函数签名**（`PROVISIONAL`，所有默认值位级一致）：
  ```python
  def move_to_point(
      ctx_or_tuple,                      # 支持两种传法：① from_config 返回 dict；② (controller: URDFController, ik: InverseKinematics, mover: CartesianMover, cfg: V15Config)
      x_m: float, y_m: float, z_m: float,
      *,
      bucket_angle_deg: Optional[float] = None,
      strategy: Literal['lerp', 'trapezoidal'] = "lerp",
      workspace_mode: Literal['auto','dig','transit','wrist'] = "auto",
      skip_workspace_check: bool = False,       # 调试用，默认 False
      wait_per_waypoint_s: float = 0.0,         # 每个 waypoint 发布后额外 wait，默认 0（用段 duration_s 卡时间）
      tolerance_deg: float = 1.0,
      timeout_total_s: Optional[float] = None,  # None → 自动用 sum(waypoints.duration_s) * 1.5
  ) -> MoveResult
  ```
- **FR-4.2 执行流程 8 步**：① WorkspaceChecker（除非 skip） → ② FR-2 planning_ik_solve → ③ 取当前 pose 作为起点 → ④ TrajectoryPlanner.plan_segment 生成 N waypoints → ⑤ 记录 `t_start` → ⑥ 循环 `set_pose(pose_deg)` + `time.sleep(max(0, duration_s - wait_per_waypoint_s))` → ⑦ 最终轮询到位（tolerance_deg / timeout_total_s）→ ⑧ 包装 MoveResult 返回（`ik_solution`、`final_tip_xyz` 等字段复用现有）
- **FR-4.3 MoveResult 扩展**：新增 3 个可选字段（`STABLE` 字段保留，只向后追加；默认全有）：
  - `waypoints: list[JointTrajectoryPoint] | None`：完整轨迹路点数组
  - `workspace_checked: bool`：是否做了可达域检查
  - `workspace_report: ReachabilityReport | None`：检查报告（失败时 reason 可见）

### FR-5 控制接口2：挖掘后 → 抬臂 up_m → 回转 swing_deg → 到卸料区上方（任务3接口2）
- **FR-5.1 对外函数签名**（`PROVISIONAL`）：
  ```python
  def move_to_dump_transit(
      ctx_or_tuple,
      *,
      dig_point_xyz: Optional[tuple[float, float, float]] = None,  # None = 用当前末端 FK 推算
      up_m: float,
      swing_deg: float,
      dump_height_safety_margin_m: float = 0.05,
      strategy: Literal['lerp', 'trapezoidal'] = "lerp",
  ) -> MoveResult
  ```
- **FR-5.2 语义流程（严格与用户回答一致，半开斗不涉及，接口2 只负责**抬+转**）**：
  1. 起点姿态 `start_pose`：`dig_point_xyz is None` → 用 `mover.current_pose()`；否则调用 `planning_ik_solve(dig_point_xyz, bucket=None, cfg, ik)` 拿到挖掘后姿态
  2. **大臂提升 up_m**：保持 swing_yaw / arm_boom / bucket_arm 不动，只调 boom_swing（几何上通过抬 boom 使末端 FK 的 z 增量 = up_m，数值上用二分迭代 5~10 次，精度 ±5mm）
  3. **回转 swing_deg**：`swing_yaw_target = start_swing + swing_deg`（正值顺时针，与 README_Agreement §2.2 方向完全一致，swing_move(True, angle)=+swing_yaw）
  4. **合成 2 段轨迹**：`[start → lift_only_pose] + [lift_only_pose → lift_and_swing_pose]`，每段独立 `plan_segment(strategy)`，合并后发布
  5. 返回 `MoveResult.waypoints = 两段 concat`，`success = 两段都到位`

### FR-6 控制接口3：卸料过程（半开斗 → 小臂半推 → 全开斗）（任务3接口3）
- **FR-6.1 对外函数签名**（`PROVISIONAL`，用户补充"半推小臂"已显式建模）：
  ```python
  def dump_material(
      ctx_or_tuple,
      *,
      bucket_half_open_deg: float = 45.0,   # 第一步 bucket_move(False, 45°) 语义：打开一半（相对角 -45°）
      arm_half_push_deg: float = 30.0,      # 第二步 arm_move(False, 30°)  语义：小臂向外"半推"伸出（相对角 -30°，arm_move(up_or_down=False)=arm_boom 负）
      bucket_full_open_deg: float = 50.0,   # 第三步 bucket_move(False, 50°)  语义：从半开到全开，再开 50°（累计 -95° = 全开）
      bucket_shake: bool = True,            # 第四步全开后轻微 bucket ±5° × 3 次抖动辅助卸料
      shake_amplitude_deg: float = 5.0,
      shake_times: int = 3,
      settle_s_per_step: float = 0.3,       # 每步之后稳料 0.3s
      strategy: Literal['lerp', 'trapezoidal'] = "lerp",
  ) -> list[MoveResult]                   # 3~4 步的结果数组，供上层逐行打印
  ```
- **FR-6.2 严格方向语义对齐（用户"注意角度和正负值"最高约束）**：所有内部实现统一调用 v15 现有 `bucket_move / arm_move` 语义函数（`semantic_moves.py`），**禁止直接改 pose_deg**，以保证方向与 §2.2 AC-7 完全一致：
  - 半开：`bucket_move(up_or_down=False, angle_deg=bucket_half_open_deg)` → bucket_arm 相对角 **减 45°**（即打开）
  - 小臂半推：`arm_move(up_or_down=False, angle_deg=arm_half_push_deg)` → arm_boom 相对角 **减 30°**（即小臂伸出）
  - 全开：`bucket_move(up_or_down=False, angle_deg=bucket_full_open_deg)` → bucket_arm 再减 50°，累计 **-95° = 全开**
  - 抖动：`bucket_move(up_or_down=True, shake_amplitude_deg)` → `sleep settle/2` → `bucket_move(False, shake_amplitude_deg)` → `sleep settle/2`，共 `shake_times` 轮

---

## 3. 非功能需求 (Non-Functional Requirements, NFR)

| NFR | 指标 | 证据 / 验证方式 |
|---|---|---|
| NFR-1 零依赖 | 双 grep 0 行违规（`from shandong\.v[0-14]` 与 `import shandong\.v[0-14]`，仅扫 py/yaml/yml） | `_run_all_checks.py` `_do_grep_negative`（AC-9） |
| NFR-2 零破坏 | v15 阶段 1~阶段 2 的 33 条旧回归 100% 通过（33/33） | `cd examples && python3 _run_all_checks.py --section phase0_1_2`（默认 33 条独立跑） |
| NFR-3 性能 | 单次 `check_point_reachable` ≤ 2ms；单段 `plan_segment(lerp,10 waypoints)` ≤ 5ms；`plan_segment(trapezoidal, 100 waypoints)` ≤ 50ms | examples/ex14 断言 `perf_counter` 差值 < 阈值 × 2（留安全裕度） |
| NFR-4 配置文档化 | `default_config.yaml` 的 `joint_limits` 节**上方**追加注释块，逐条声明 4 语义关节 `{joint}_move(True, +angle) = 什么动作`，与 README_Agreement §2.2 位级相等 | AC-10 人工比对 + grep 4 行方向说明 |
| NFR-5 Ubuntu 裸系统兼容 | 无 PyYAML → 三级 BUILTIN 兜底；无 rclpy → SensorManager auto Mock；所有新增模块**无第三方 pip 包** | 虚拟机（待补，先保证 sys.modules 内无 scipy/numpy/casadi） |

---

## 4. 约束、依赖、假设、开放问题

### 4.1 Constraints（不可突破的硬约束）
1. **STABLE 不破**：`from_config` 返回 dict 7 key、`MoveResult` 原有 8 字段、语义函数 8 签名、`TiltReading` 11 字段全部保持不变，新增字段一律放到**尾部或可选**；
2. **AC-7 方向不可反**：`swing_move(+)=+yaw(cw) / boom_move(+)=-boom_swing(up) / arm_move(+)=+arm_boom(in) / bucket_move(+)=+bucket_arm(curl_in)` 任何新层调用必须走 `semantic_moves` 封装，不能手调符号（FR-6.2 已要求）；
3. **无 numpy/scipy/casadi**：纯 stdlib + dataclass + math 实现所有轨迹插值，保持 NFR-5 裸系统兼容。

### 4.2 Dependencies（内部复用）
- 直接复用 v15 现有：`InverseKinematics.search_bucket_angle / solve_bucket_pose`、`ForwardKinematics.solve`、`CartesianMover`、`semantic_moves.py 8 函数`、`StepBuilder / clamp_pose`、`V15Config.link_geometry / joint_limits`
- **零外部 v0~v14 import**（NFR-1）

### 4.3 Assumptions（已通过 AskUserQuestion 关闭，不再 AskUser）
| 问题 | 已确认答案 |
|---|---|
| 可达域过滤层数 | 几何腕点 + swing + ground_penetration + transit_min_z + singularity 共 5 层，用户确认 A 组合 |
| 轨迹策略 | `lerp` + `trapezoidal` 双策略可切换，用户确认 C 选项 |
| 接口2 是否包含挖掘 | 用户确认 B：**不包含挖掘**，接口2 只做抬臂 up_m + 回转 swing_deg |
| 接口3 卸料步骤 | 用户补充 B+：半开斗 → 小臂半推 → 全开斗 →（可选抖动） |
| 关节正负值注释 | config 加说明，用户明确要求 |

### 4.4 Open Questions
**全部关闭**（本 spec 所有二义性都已 AskUser 确认或默认值可配置）。

---

## 5. Acceptance Criteria（验收标准，类型唯一：rule / rubric）

### 5.1 Rule 类 AC（可客观二元判定 = pass / fail）

| ID | 类型 | 判定条件 | 证据来源 / 运行命令 |
|---|---|---|---|
| AC-1 | rule | 默认 `link_geometry`（L1=0.6, L2=0.840, L3=0.26）时，腕点下边界 `inner = 0.24m`。调用 `check_point_reachable(xyz=(0.24 + 0.26*cos(-45°), 0, 0.0 + 0.26*sin(-45°)), mode='wrist')` 返回 `success=True`，`reasons=[]` | `examples/ex13_workspace_reachability_AC1_2_3.py` 自断言 exit 0 |
| AC-2 | rule | 远外点 `(3.0, 0, 0.5)`（总臂长 1.70m，3.0m > 1.70）→ `success=False`，且 `"outer_radius_exceeded" in reasons` | 同上 ex13 |
| AC-3 | rule | 深坑点 `(1.0, 0, -0.5)`，`mode='auto'` → 因 `z < min_ground_z(-0.1m)`，`success=False`，`"ground_penetration" in reasons`；切 `mode='dig'`（关闭 ground 检查）后返回 `success` 取决于几何 | 同上 ex13 |
| AC-4 | rule | `TrajectoryPlanner.plan_segment(start={boom:0,arm:0,bucket:0,swing:0}, end={boom:30,arm:45,bucket:-30,swing:0}, strategy='lerp', lerp_step_deg=5)` → 返回 waypoints 数 = `ceil(Δθ_max/5) + 1 = ceil(45/5)+1 = 10`，且第 k 路点对所有关节角度差绝对值 ≤ `Δθ_joint/N + 0.01`（线性插值精度 0.01°） | `examples/ex14_trajectory_planner_AC4_5.py` 自断言 exit 0 |
| AC-5 | rule | 同 AC-4 起止点，`strategy='trapezoidal'`，配置 `max_speed_deg_s={boom:20, arm:30, bucket:30, swing:15}` `accel_deg_s2={boom:40, arm:60, bucket:60, swing:30}` → `t_from_start_s` 数组严格递增（每个后继 > 前驱 + 1e-6），且 `abs(waypoints[-1].t_from_start_s - expected_total) < 0.5s`，其中 expected_total = 最长关节（arm_boom 45°）的梯形总时长 ≈ 45/30 + 30/60 = 1.5+0.5 = 2.0s 量级 | 同上 ex14 |
| AC-6 | rule | **控制接口1 闭环**：`from_config(adapter_backend='mock', use_config_limits=True)` → `move_to_point(ctx, 1.2, 0.2, 0.4, strategy='lerp')` → `success=True`，且 `fk(reached_pose).tip_xyz` 与目标 (1.2,0.2,0.4) 的欧氏距离 ∈ [0, 0.05m] | `examples/ex15_three_control_interfaces_AC6_7_8.py` 自断言 exit 0 |
| AC-7 | rule | **控制接口2 抬臂+回转**：先 `move_to_point(ctx, 1.2, 0.0, 0.4)` 设起点，再 `move_to_dump_transit(ctx, up_m=0.3, swing_deg=30.0)` → 最终 FK tip.z ∈ [0.4+0.3-0.05, 0.4+0.3+0.05] = [0.65, 0.75]m，且 swing_yaw 反馈 ∈ [30-2, 30+2]° | 同上 ex15 |
| AC-8 | rule | **控制接口3 卸料 4 段**：起点 `bucket_arm = 0°，arm_boom = 60°`，`dump_material(ctx, bucket_half_open_deg=45, arm_half_push_deg=30, bucket_full_open_deg=50, bucket_shake=True, settle_s_per_step=0.01)` → 最终：① `bucket_arm` ∈ [-99, -91]°（≈-95 全开）② `arm_boom` ∈ [28, 32]°（=60-30=30）③ 返回 list 长度 ∈ [4, 6]（3 主步 + 3 shake = 6，具体按语义函数合并）④ 每步 `MoveResult.success=True` | 同上 ex15 |
| AC-9 | rule | **v15 零依赖 v0~v14**：双 grep 0 行（和阶段 1 AC-6 完全一样，白名单过滤 README 说明文字） | `_run_all_checks.py` `TR8: grep_negative_0lines` |
| AC-10 | rule | **config 关节正负说明文档化**：`default_config.yaml` 中 `joint_limits:` 上方存在注释块，4 行语义方向（swing True=cw+ / boom True=-up / arm True=+in / bucket True=+curl）与 `README_Agreement.md §2.2 表` 中 4 行字符串**逐行对比完全相等**（允许前后 `# ` 前缀） | `examples/ex02_semantic_direction_AC7_self_assert.py` 内新增断言（读 yaml 文本 + grep 注释 4 行），或 `_run_all_checks.py` 新增 10.5 TR |

### 5.2 Rubric 类 AC（暂无，本轮所有需求均可客观 rule 化，0 rubric）

---

## 6. 对用户 3 个原始任务的覆盖矩阵

| 用户任务 | 对应 FR | 核心 AC |
|---|---|---|
| **任务1** 初始化确定可达域 + 限制行为 | FR-1.1 初始化 + FR-1.2 5 层过滤 + FR-1.3 3 方法 | AC-1/2/3 |
| **任务2** 末端位置 → 逆解 + 轨迹规划 | FR-2 逆解包装 + FR-3 双策略轨迹 | AC-4/5 |
| **任务3 接口1** 空间点 → 完整轨迹（笛卡尔+单关节） | FR-4 8步执行 + 扩展 MoveResult.waypoints | AC-6 |
| **任务3 接口2** 挖掘后 → 抬 up 米 → 转 swing 度 | FR-5 2 段合成（lift_only → lift_and_swing） | AC-7 |
| **任务3 接口3** 卸料（半开+小臂半推+全开+抖动） | FR-6 4 步走 semantic_moves 封装 | AC-8 |
