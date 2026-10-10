# v15 关节指令 · 传感器 · 外参 · 温漂 · Examples —— 实现任务清单（v2 对齐 v0~v14 真实硬件）

> 关联 Spec： [spec.md](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/.trae/specs/v15-joint-cmd-sensors-examples/spec.md)
> 仓库根： `/media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task`
> v2 Changelog：新增 Task 0（FR-7 独立标定脚本）；扩展 Task 1（sensors/extrinsics/tilt_compensation 三节 dataclass + BUILTIN 第③级兜底）；扩展 Task 3（TiltCompensator 温漂/零偏）；扩展 Task 5（ex11/ex12）；新 TR 矩阵覆盖 AC-8/9/10。

---

## 开放问题决策日志（Open Question Decision Log）

| OQ | 默认决策（v2 全解答） | 理由 / 对齐依据 |
|---|---|---|
| OQ-1 倾角安装语义命名 | `tilt_bucket / tilt_arm / tilt_boom / tilt_swing`，对应 0x50/51/52/53（非索引） | 与 v12 `addr_to_index` 语义完全对齐 |
| OQ-2 雷达算法参数 | 每台雷达预留 `extra: {}` 空 dict 扩展位，不做算法（Non-Goals） | 保留 schema 兼容性不破 |
| OQ-3 相机 CameraInfo | `SingleSensorConfig.info_topic: str = ""` 默认空（不订阅），可单独配置 | 标准需求但非必需 |
| OQ-4 函数返回值 | 复用 `motion.MoveResult`（`__bool__()` 返回 success） | 与 mover.move() 统一；简写法仍可用 |
| OQ-5 examples 位置 | **`v15_action_task/examples/`（包内子包）** | colcon build 随包分发；sys.path 插入更简单 |
| OQ-6 相对角在哪算 | **SensorManager 内置**（use_relative_subtraction=True） | 对齐 v8 inclinometer_reader 父子链相减；横切关注点统一 |
| OQ-7 点云 TF 变换 API | 不提供 transform_to()，仅 ExtrinsicsConfig 暴露 `to_4x4_matrix()` / `to_static_transform_publisher_args()` | 避免 tf2 硬依赖拖垮零依赖；用户业务层自行 tf2_ros.do_transform_cloud |

---

## 任务分解（9 大任务，DAG 依赖见 §末）

### Task 0：独立 TF 预标定 examples 脚本（FR-7 核心，用户 feedback 最高优先级）

**Priority**: highest（用户 feedback 明确要求"独立标定脚本，非每次运行"）  
**Status**: pending  
**Acceptance Criterion 覆盖**: AC-10（闭环 rule）、AC-6（零依赖）、AC-4（examples 独立）

#### 子任务
0.1 新建 `examples/ex11_sensor_tf_calibration_cmdline.py`（对标 v5 `tf_calibration_gui.py`，无 GUI、SSH 无头兼容）：
   - argparse：`--sensor-id`（如 `lidar_single_rear`）、`--parent base_link`、`--child lidar_single_rear_link`、`--default-x -0.5500`（从 FR-5 出厂默认值填 6 个初始值）、`--step-x 0.001`、`--output tf_calibration_record.txt`
   - 交互：循环 `cmd> ` 读 stdin 命令：`x+` / `x-` / `y+` / `z+` / `rx+` / `rx-` / `ry+` / `ry-` / `rz+` / `rz-`（rx=roll, ry=pitch, rz=yaw，增量按 step）、`show`（打印当前 8 字段）、`save`（写一行到 output.txt，格式 `x y z yaw_rad pitch_rad roll_rad parent_frame child_frame`，**与 GUI tf_calibration_record.txt 格式完全一致**）、`quit`（Ctrl-C 视为 quit）
   - 零依赖：**不 import rclpy / tf2 / v15.control_core**，只依赖 Python 标准库 argparse + sys + os + datetime
0.2 新建 `examples/ex12_sensor_tf_record_to_yaml.py`（GUI/ex11 输出 → v15 FR-5 `extrinsics:` YAML 转写工具）：
   - argparse：`--input tf_calibration_record.txt`、`--sensor-id-override lidar_single_rear`（选填，默认从 input 匹配或推断）、`--chassis-sn TEST01`（选填，默认 `DEFAULT`）、`--output-dir ./config_overrides/`、`--out ./extrinsics_SN001.yaml`（二选一）
   - 解析 record.txt 每行 → 构造对应 `ExtrinsicsEntry` dict → 输出完整可被 load_config() 加载的 YAML（或 JSON，保证三级兜底可被 load_config(path) 读入）
   - 只依赖 config/loader 的 V15Config/BUILTIN（或直接写 YAML/JSON 文本，避免 PyYAML 依赖 → 写 JSON 格式也 OK，因为 loader 有 YAML→JSON→BUILTIN 三级兜底）
   - **硬约束**：`extrinsics.get(sensor_id)` 6 参数 float 与 record.txt 逐字段误差 ≤1e-4（AC-10 第 3 步验证）
0.3 两脚本顶部 docstring 严格写明：`Purpose / Run method / ROS dependence（零）/ Exit codes`

#### 本地测试要求
- **TR 0.1 (rule)**：`python3 examples/ex11_sensor_tf_calibration_cmdline.py --sensor-id lidar_single_rear --output /tmp/tf_rec.txt` 输入 stdin：`x+\n show\n save\n quit\n`，output.txt 第一行存在，字段数 = 8（空格 split 后 8 个 token）
- **TR 0.2 (rule — AC-10 闭环)**：按 AC-10 三步运行 ①手写测试 record.txt ②跑 ex12 输出 YAML/JSON ③`cfg = load_config(path=out_yaml)` → `abs(cfg.extrinsics.get("lidar_single_rear").roll_rad - 3.0316) < 1e-4` 断言成功；exit 0
- **TR 0.3 (rule — AC-6)**：`grep -En "import|from" examples/ex11*.py examples/ex12*.py | grep -E "v0|v1[0-4]|shandong\\.v[^15]|rclpy|tf2_ros|pacecat|wit" | grep -v v15_action_task` → 空 0 行（零依赖硬约束）
- **TR 0.4 (rule)**：两脚本 py_compile 0 语法错误

---

### Task 1：扩展配置层（YAML + dataclass 10+ 新增，覆盖 FR-3.1 / FR-5 / FR-6 + FR-1）

**Priority**: high  
**Status**: pending  
**Acceptance Criterion 覆盖**: AC-1（限位 spec 纳入）、AC-3（sensors 节完整 rubric≥2）、AC-5（限位默认值不动）、**AC-8（外参出厂值 1:1 rule）**、**AC-9（倾角地址映射 1:1 rule）**

#### 子任务
1.1 扩展现有的 `config/default_config.yaml`，**末尾追加三大节（不动现有 6 大类任何字段值，AC-5 保证）**：
   - **第 7 节 sensors:**：严格按 spec.md FR-3.1 YAML 模板写
     - tilt_sensors 4 条：modbus_addr 字符串化 `"0x50"/"0x51"/"0x52"/"0x53"`；array_index=0/1/2/3；semantic_joint=bucket_arm/arm_boom/boom_swing/swing_yaw；topic 全是 `/excavator/inclinometer_pitch_deg`（C-1 硬约束）
     - lidars 5 条：lidar_single_rear.topic = `/pointcloud`（v5 PaceCat M300 真实原始话题，其余 4 条默认 enabled=false）
     - cameras ≥5 条：cam_front_main.info_topic 非空（订阅 CameraInfo），其余 info_topic=""（默认不订阅）；cam_front_main/left/right 默认 enabled=true
   - **第 8 节 extrinsics:**：严格按 spec.md FR-5 YAML 模板写
     - 第 1 条 lidar_single_rear 6 参数 **必须 == sensors_tf.launch.py 真实值**：`x_m=-0.5500 y_m=-0.2000 z_m=1.2712 yaw_rad=0.0532 pitch_rad=0.0349 roll_rad=3.0316`（AC-8 硬验证）
     - 第 2 条 cam_front_main 真实出厂值：`x=0.4539 y=0.1532 z=1.5246 pitch=1.1519`
     - 第 3 条 cam_left 真实出厂值：`x=0.4239 y=-0.1768 z=1.4246 pitch=0.9250`
     - 其余 7 条 sensors id（lidar_matrix/360L/360R/single_bucket + cam_right/rear/bucket_view/arm_overlook → 实际 ≥10 条 total）出厂 0 值占位，enabled=false
   - **第 9 节 tilt_compensation:**：严格 FR-6 模板：alpha=0.98, calib_count=50, gyro_deadzone_rad_s=0.002, auto_calibrate_on_open=true, use_relative_subtraction=true
1.2 在 `config/loader.py` 新增 **9 个 dataclass**（保持零第三方依赖，全 `@dataclass` + `from __future__ import annotations`）：
   - **TiltSensorConfig**：id, type, modbus_addr, semantic_joint, topic, array_index, msg_type, frame_id, qos_depth, enabled, extra: dict
   - **LidarSensorConfig**：id, type, topic, msg_type, frame_id, qos_depth, enabled, extra: dict
   - **CameraSensorConfig**：id, type, topic, info_topic, msg_type, frame_id, qos_depth, enabled, extra: dict
   - （或者统一用 **SingleSensorConfig** 超级类，tilt 字段为 Optional，camera info_topic Optional；更推荐统一超级类减少代码重复，提供 `as_tilt() / as_lidar() / as_camera()` 返回 Optional[TypedView]）
   - **SensorsConfig**：`tilt_sensors: Dict[str, SingleSensorConfig]` ／ `lidars: Dict[str, SingleSensorConfig]` ／ `cameras: Dict[str, SingleSensorConfig]`；便捷方法：`list_tilt_ids() / list_lidar_ids() / list_camera_ids() / by_id(sid) -> Optional`
   - **ExtrinsicsEntry**：sensor_id, parent_frame, child_frame, x_m, y_m, z_m, yaw_rad, pitch_rad, roll_rad, quaternion_xyzw_or_none: Optional[Tuple[float,float,float,float]], matrix_4x4_or_none: Optional[List[List[float]]], enabled, note: str；便捷纯函数：`to_xyzrpy() -> Tuple[6 float]` ／ `to_4x4_matrix() -> List[List[float]]` ／ `to_static_transform_publisher_args() -> List[str]`（6 参数，对齐 C-3）
   - **ExtrinsicsConfig**：`entries: Dict[str, ExtrinsicsEntry]`；方法 `get(sensor_id) -> Optional` ／ `items()` ／ `to_launch_static_tf_nodes_yaml()`（纯字符串输出，可直接 paste 到 launch file 替换 sensors_tf.launch.py）
   - **TiltCompensationConfig**：alpha: float, calib_count: int, gyro_deadzone_rad_s: float, auto_calibrate_on_open: bool, use_relative_subtraction: bool
1.3 把三大新 config 接入 `V15Config` 顶层 dataclass 三个字段：`sensors: SensorsConfig`、`extrinsics: ExtrinsicsConfig`、`tilt_compensation: TiltCompensationConfig`
1.4 在 `V15Config.from_dict(classmethod)` 中解析 dict 的三个键 `sensors / extrinsics / tilt_compensation`；缺省或空键 → 调用 `build_default_sensors_config()` 等 3 个 `build_*` 纯函数（与 loader.py 现有的 build_joint_limits_config / build_link_geometry_config 风格完全一致）
1.5 同步扩展 **`BUILTIN_DEFAULT_CONFIG_DICT`**（loader.py 顶部第 ③ 级兜底 dict），新增 `sensors:` / `extrinsics:` / `tilt_compensation:` 三大块，**数值 1:1 等于 YAML 对应节**（三级兜底 1:1 延续保证）
1.6 在 `config/__init__.py` 追加导出：`SingleSensorConfig / SensorsConfig / ExtrinsicsEntry / ExtrinsicsConfig / TiltCompensationConfig` 5 个公开符号

#### 本地测试要求
- **TR 1.1 (rule)**：py_compile loader.py + __init__.py 0 语法错误
- **TR 1.2 (rule — AC-9)**：`cfg = load_default_config()` → 4 条 tilt 映射严格：
  ```
  cfg.sensors.tilt_sensors["tilt_bucket"].modbus_addr == "0x50" and .array_index==0 and .semantic_joint=="bucket_arm"
  cfg.sensors.tilt_sensors["tilt_arm"].modbus_addr    == "0x51" and .array_index==1 and .semantic_joint=="arm_boom"
  cfg.sensors.tilt_sensors["tilt_boom"].modbus_addr   == "0x52" and .array_index==2 and .semantic_joint=="boom_swing"
  cfg.sensors.tilt_sensors["tilt_swing"].modbus_addr  == "0x53" and .array_index==3 and .semantic_joint=="swing_yaw"
  ```
  全 True，否则 FAIL
- **TR 1.3 (rule)**：`len(cfg.sensors.list_lidar_ids())==5 and len(cfg.sensors.list_camera_ids())>=5`（14 路全覆盖）
- **TR 1.4 (rule — AC-8)**：`e = cfg.extrinsics.get("lidar_single_rear")` → `abs(e.x_m+0.55)<1e-4 and abs(e.y_m+0.2)<1e-4 and abs(e.z_m-1.2712)<1e-4 and abs(e.yaw_rad-0.0532)<1e-4 and abs(e.pitch_rad-0.0349)<1e-4 and abs(e.roll_rad-3.0316)<1e-4`（六项全通过）
- **TR 1.5 (rule)**：三级兜底 1:1：YAML flatten → 与 BUILTIN dict flatten 对比 sensors/extrinsics/tilt_compensation 三节 0 差异（复用 config 完整性检查脚本方法）
- **TR 1.6 (rule — AC-5)**：`cfg.limits.boom_swing.max_deg == 55.0` 且 `cfg.limits.bucket_arm.min_deg == -95.0`（限位未动）
- **TR 1.7 (rule)**：TiltCompensationConfig 默认值：`alpha==0.98 and calib_count==50 and gyro_deadzone_rad_s==0.002 and auto_calibrate_on_open==True and use_relative_subtraction==True`

---

### Task 2：新增语义作业指令库（FR-2 核心 8 函数 + MoveResult 适配）

**Priority**: high  
**Status**: pending  
**Acceptance Criterion 覆盖**: AC-2（rubric≥2）、AC-1 FR-1.2 第三层、AC-7（方向语义 rule）

#### 子任务（与 v1 tasks 2.1~2.6 一致，v2 无改动，保留原样）
2.1 新建 `action_library/primitives/semantic_moves.py`
2.2 实现 4 个基础布尔+增量函数（签名与 FR-2.1 表严格 1:1，返回 MoveResult）
2.3 实现 4 个扩展函数（swing_move_to / boom_lift / arm_extend / bucket_tilt_to）
2.4 统一 5 步契约（读当前 → 按 bool 方向算目标 → 限位裁剪 → set_joint → blocking 等待到位）
2.5 复用 `motion.cartesian_mover.MoveResult`，缺字段则在其 dataclass 新增可选字段（不新建 Result 类型）
2.6 导出链：primitives/__init__.py → action_library/__init__.py → 顶层 __init__.py

#### 本地测试要求
- **TR 2.1 (rule)**：4 文件 py_compile 0 语法错误
- **TR 2.2 (rule — AC-7)**：MockAdapter 初始全 0° → 分别调用 4 函数各 1 次 (True, 10°) → 断言 4 目标角方向：boom→-10，arm→+10，bucket→+10，swing→+10
- **TR 2.3 (rule)**：限位裁剪：`boom_move(ctl, False, 999)`（下探超限）→ 最终反馈 == 55.0°
- **TR 2.4 (rule)**：`bool(result)==True and result.success==True`（MockAdapter blocking 同步模式）
- **TR 2.5 (rubric 0-2 ≥2)**：8 函数全 docstring；默认参数匹配 FR-2 表；零硬编码

---

### Task 3：新增 sensors/ 子包 + SensorManager + TiltCompensator（FR-3 + FR-6 核心）

**Priority**: high  
**Status**: pending  
**Acceptance Criterion 覆盖**: AC-3（rubric≥2）、**AC-6（v0~v14 零依赖）**、**FR-6（温漂/零偏/互补滤波内置）**

#### 子任务（v2 在 v1 基础上 + 3.3 TiltCompensator + 3.6 父子链相对角）
3.1 新建目录 `sensors/` + `__init__.py`（导出：SensorManager / TiltReading / LidarReading / CameraReading / TiltCompensator）
3.2 新建 `sensors/base.py`（零 ROS）：
   - 三个 Reading dataclass（按 spec.md FR-3.3 字段完整实现，TiltReading 必含 compensated_angle_deg + calibration_done + relative_angle_deg）
   - 新增 **`TiltCompensator` 类**（纯 Python，不碰 ROS）：
     ```python
     @dataclass
     class TiltCompensator:
         cfg: TiltCompensationConfig
         _calib_accum: float = 0.0
         _calib_seen: int = 0
         _zero_bias_deg: Optional[float] = None
         _last_compensated: Optional[float] = None
         def update(self, raw_angle_deg: float, *, gyro_rad_s_or_none: float=None) -> Tuple[compensated, done, remaining]:
             # 1. if not calibrated → accum raw → seen==calib_count → zero_bias = mean
             # 2. if calibrated → compensated = raw - zero_bias
             # 3. if gyro provided & alpha<1 → 一阶互补: new = alpha*(last + gyro*dt) + (1-alpha)*compensated （简化版，若缺 gyro/时间戳则直接用）
             # 返回 (compensated_deg, done_bool, remaining_frames)
     ```
3.3 新建 `sensors/manager.py` → `SensorManager`（Adapter 模式）：
   - `from_config(cfg, ros_environment="auto")` → 检测 import rclpy 是否成功 → 失败 → Mock 后端
   - open() 时：①创建 4 TiltCompensator 实例（每个 tilt_sensor 一个）②若 auto_calibrate_on_open=True → 重置为 calibrating 状态
   - Float64MultiArray 回调（tilt）：拆 data[array_index] → 调对应 TiltCompensator.update(raw) → 存 TiltReading（含 compensated/calib_done/remaining）
   - `use_relative_subtraction=True` 时，额外在 `get_tilt("tilt_bucket")` 返回前计算 Reading.relative_angle_deg = compensated(child) - compensated(parent)（父子链：tilt_bucket 父 tilt_arm；tilt_arm 父 tilt_boom；tilt_boom 父 tilt_swing；tilt_swing 父无）
   - PointCloud2 回调：读 header.stamp / width / height / row_step → 存 LidarReading（points_count=width*height or width if organized==False）
   - Image 回调：读 stamp / width / height / encoding → 存 CameraReading
   - 可选 info_topic：若 camera.info_topic != "" → 再订阅一条 CameraInfo（存 intrinsics 不解析，只打 cache latest header）
3.4 新建 `sensors/mock_manager.py`：Mock 后端（全 getter 返回 is_valid=False 或 None），提供 `_inject_reading(sid, reading)` 测试用
3.5 **零依赖硬约束（AC-6）**：sensors/ 所有 import 严格白名单（标准库 + v15_action_task.config + **动态 rclpy/sensor_msgs try/except ImportError 兜底**）
3.6 顶层 `__init__.py` 追加 symbols；`from_config(init_sensors=False, sensor_ros_mode="auto")` 新参数 → 返回 dict 新增 key `sensor_manager`

#### 本地测试要求
- **TR 3.1 (rule)**：py_compile sensors/*.py 0 语法错误
- **TR 3.2 (rule — AC-6)**：grep 命令输出 0 行（见 tasks v1 Task3 TR3.2 同一条）
- **TR 3.3 (rule)**：Mock 后端 `open()` 后 `len(list_tilt)==4 and len(list_lidar)==5 and len(list_cam)>=5`
- **TR 3.4 (rule — FR-6 TiltCompensator 校准正确性)**：
  ```python
  comp = TiltCompensator(cfg=TiltCompensationConfig())
  for i in range(60):
      compd, done, remain = comp.update(15.0)  # 模拟 15° 恒定输入
  assert done==True
  assert abs(compd - 0.0) < 0.01   # 15-15 = 0，零偏扣除后接近 0
  ```
- **TR 3.5 (rule — 动态 import)**：Ubuntu 默认 Python 无 rclpy 下 `from v15_action_task.sensors import SensorManager` 不抛 ImportError；`is_ros_backend()==False`
- **TR 3.6 (rubric 0-2 ≥2)**：SensorManager 有 from_config/open/close/with；14 路全；Mock/ROS 双后端；TiltCompensator 4 实例且温漂字段齐全
- **TR 3.7 (rule — 父子链相对角)**：手动 inject 4 tilt 的 raw=10/20/30/40° → calibration done 后 `get_tilt("tilt_bucket").relative_angle_deg` == (10-15 零偏) - (20-25 零偏)（即 bucket 相对 arm 的 compensated 差 ≈ -10° 或 10°，取决于符号链实现，以断言具体实现定义）

---

### Task 4：顶层 from_config / __init__ / __all__ 导出聚合（所有新增符号统一入口）

**Priority**: medium  
**Status**: pending  
**Acceptance Criterion 覆盖**: AC-2/AC-3 顶层 1 行 import 可用

#### 子任务（同 v1 Task4，略扩展：tilt_compensation 暴露 cfg.tilt_compensation 访问）
4.1 __init__.py 新增 import 两块（sensors + semantic_moves 8 函数 + TiltCompensator + 三大 Config 类导出）
4.2 `from_config()` 新增 `init_sensors=False / sensor_ros_mode="auto"`；返回 dict key `sensor_manager`
4.3 旧 API 零 break 验证

#### 本地测试要求
- **TR 4.1 / 4.2 / 4.3**：同 v1

---

### Task 5：新增 examples/ 目录 + 12 独立脚本 + 2 辅助（FR-4 + FR-7 ex11/ex12）

**Priority**: medium  
**Status**: pending  
**Acceptance Criterion 覆盖**: AC-4（结构独立）、AC-7（方向语义 ex02）、AC-10（ex12 闭环）

#### 子任务
5.1 创建 `v15_action_task/examples/__init__.py`（空包标记）
5.2 新建公共辅助 `_util_import_helper.py`：封装 `ensure_v15_in_sys_path()`（兼容三种运行场景）+ `import_all_or_exit_with_skip(need_ros=False)` 标记 ROS-only 脚本
5.3 新建 12 独立脚本（ex01~ex12 按 spec.md FR-4 脚本清单 1:1 实现；其中 ex11/ex12 已在 Task 0 完成可直接搬过来或 Task 0 单独实现，不重复；**建议 Task 0 先做，Task 5 把 ex01-10 补齐**）：
   - ex01：FR-1 三层限位（Config → URDFController → arm_move 超限对比，三列打印 + 断言 PASS）
   - ex02：**AC-7 自断言脚本**（内部 self-check 4 方向，打印 `方向语义 4/4 PASS` 或 exit 1）
   - ex03：8 函数全量（MoveResult.success / final_tip / distance_mm 打印）
   - ex04：SensorManager Mock 14 路枚举 + get_* 演示（无 ROS exit 0）
   - ex05：ROS 模式（无 rclpy 打印"SKIP: no rclpy" exit 0；有 rclpy 30s 频率打印）
   - ex06：from_config 全链路（init_sensors=True）→ arm_move + sensor_manager.get_tilt 联调（Mock 模式）
   - ex07：FK→IK→FK 20 组随机闭环（max err < 1e-3 mm，否则 exit 1）
   - ex08：CartesianMover move + move_with_bucket（2 目标点，Mock）
   - ex09：build_single_dig_dump_task + build_multi_cycles 生成 → dump JSON → 读回校验
   - ex10：自定义 config（L_bucket=0.50m vs 0.26m）→ FK 铲尖 Δz = 240.00mm（abs(err)<1mm，否则 exit 1）
   - ex11：Task 0.1 已实现（搬过来）
   - ex12：Task 0.2 已实现（搬过来）
5.4 每个脚本顶部 docstring：Purpose / Run method / ROS dependence / Exit codes（0=OK 1=FAIL 130=Ctrl-C）

#### 本地测试要求
- **TR 5.1 (rule — AC-4)**：`find v15_action_task -maxdepth 2 -name "ex_*.py" -not -path "*/examples/*"` → 0 行（examples 外不混入）
- **TR 5.2 (rule — AC-4 独立运行)**：ex01/ex02/ex03/ex04/ex06/ex07/ex08/ex09/ex10/ex12 **零 ROS 环境下 python3 各自运行 exit 0**
- **TR 5.3 (rule — AC-7)**：ex02 stdout 含 `方向语义 4/4 PASS`，exit 0
- **TR 5.4 (rule)**：12 脚本 + 2 辅助 py_compile 全 0 错误
- **TR 5.5 (rubric 0-2 ≥2)**：脚本顶部 Usage / docstring / Exit codes 齐全；_util_import_helper 统一 sys.path；无重复 insert sys.path 10 次；examples 内 import 统一走 `ensure_v15_in_sys_path()`

---

### Task 6：回归测试与完整性验证（AC-1 ~ AC-10 聚合）

**Priority**: low  
**Status**: pending  
**Acceptance Criterion 覆盖**: 所有 AC 自动打分

#### 子任务
6.1 新建 `examples/_run_all_checks.py`（examples 内辅助脚本）：聚合 TR 0.x / 1.x / 2.x / 3.x / 4.x / 5.x 全部 rule 型自动检查 + rubric 人评位打占位分（2/2 或 1/2 标注 HUMAN_REVIEW）
   - 分 7 节：Config / SemanticMoves / Sensors / TopLevelAggregation / Examples / Task0Calibration / Extras
   - 每条 rule TR 自动断言 PASS/FAIL → 末尾 `x/30 PASS` 汇总（假设 30 条自动检查）
   - 人评项（AC-2 / AC-3 / TR2.5 / TR3.6 / TR5.5 rubric 项）打 `2/2 NEEDS_HUMAN_REVIEW` 占位
6.2 支持 `--skip-ros` 跳过 ex05/ex11（默认开启，零 ROS 环境可跑完）

#### 本地测试要求
- **TR 6.1 (rule)**：Ubuntu 默认 Python 无 PyYAML/无 rclpy 下 `python3 examples/_run_all_checks.py` exit 0，且 自动检查 PASS 数 ≥ 25/30

---

### Task 7：README.md 更新（新增 4 需求章节 + 基本用法 + 验证表扩展）

**Priority**: low  
**Status**: pending  
**Acceptance Criterion 覆盖**: 文档可追溯

#### 子任务
7.1 README.md TOC 新增 anchor：§3.7 语义作业原语 / §3.8 传感器接口 / §3.9 Examples 目录 / §4.6 传感器配置节 / §4.7 外参配置节 / §4.8 倾角温漂补偿配置节
7.2 各节正文：2~8 行核心 API 示例（arm_move 2 行；SensorManager 2 行；ex01/ex02/ex11/ex12 表格）
7.3 §8 验证结果表格追加 Task 6 汇总 + AC-8/9/10 各自单独行（出厂值 PASS、地址映射 PASS、标定闭环 PASS）

#### 本地测试要求
- **TR 7.1 (rule)**：grep README.md 确认 6 个 TOC anchor 标题正文存在

---

### Task 8（仅 v2 新增）：零依赖回归 · AC-6 grep 全库扫描（一次性验证任务）

**Priority**: medium  
**Status**: pending  
**Acceptance Criterion 覆盖**: AC-6（零依赖硬约束）

#### 子任务
8.1 编写 grep 命令（放在 Task 6 聚合脚本里的独立 check 函数 + 直接在 Command line 可跑）：
   ```bash
   cd v15_action_task
   grep -rnE "^\s*(from|import)\s+" --include="*.py" ./ examples/ | grep -Ev "v15_action_task|from __future__|dataclasses|typing|abc|threading|time|sys|os|json|math|pathlib|argparse|datetime|enum|functools|collections" > /tmp/imports_non_whitelist.txt
   grep -En "shandong\.v[^15]|shandong/v[^15]|v0_|v1[0-4]_|v0/|v1[0-4]/" /tmp/imports_non_whitelist.txt
   ```
   要求第 2 步 grep 输出空 0 行
8.2 字符串层面额外扫一遍非 import 引用：`grep -rnE "(pacecat|wit901|inclinometer_sensor_bridge|tf_calibration_gui|hikvision_cam_node|network_cam_node)" v15_action_task/ examples/ --include="*.py"` → 输出空（允许在注释/docstring 里出现但不能硬编码 import）；若在 docstring 出现可保留

#### 本地测试要求
- **TR 8.1 (rule — AC-6)**：两 grep 空输出

---

## 依赖顺序（DAG v2）

```
Task 0 (FR-7 独立标定脚本 examples/ex11+ex12)   Task 1 (Config 扩展：sensors/extrinsics/tilt_comp)
       ↓独立可先做                                           ↓
       │                                               Task 2 (语义原语)    Task 3 (sensors + TiltCompensator)
       │                                                    ↓                          ↓
       └─────────────────────────────────↘ ______ Task 4 (顶层聚合) ______________↙
                                                        ↓
                                                 Task 5 (examples 01-12 + 辅助)
                                                        ↓
                                          Task 6 (全量回归聚合 _run_all_checks)
                                                        ↓
                                          Task 7 (README) + Task 8 (AC-6 grep 全库)
```

（Task 0 完全不依赖控制链，可第一天先做；Task 1→2/3 并行→4→5→6→7+8）

---

## 验证汇总（Acceptance Criteria → Task.TR 矩阵 v2，10 AC 全覆盖）

| Spec AC | 类型 | 对应 Task / TR | 备注（v2 新增 vs v1） |
|---|---|---|---|
| **AC-1 限位三层契约** | rule | Task1 / TR1.6 + Task2 / TR2.3 + Task5 / ex01 | — |
| **AC-2 4 关节函数 rubric** | rubric(≥2) | Task2 / TR2.1~TR2.5 + Task5 / ex02 ex03 | — |
| **AC-3 传感器接口 rubric** | rubric(≥2) | Task1 / TR1.2~TR1.4 + Task3 / TR3.1~TR3.7 + Task5 / ex04 ex05 | v2 新增 **TR3.4 TiltCompensator 校准 + TR3.7 父子链相对角** |
| **AC-4 Examples 独立目录** | rule | Task5 / TR5.1~TR5.5 + Task0 / TR0.4 | v2 脚本数 4 → **12 + 2 辅助** |
| **AC-5 限位默认值不变** | rule | Task1 / TR1.6 | — |
| **AC-6 v0~v14 零依赖** | rule | Task3 / TR3.2 + Task0 / TR0.3 + **Task8 / TR8.1** | v2 **新增 Task8 全库 grep 双保险** |
| **AC-7 方向语义自洽** | rule | Task2 / TR2.2 + Task5 / TR5.3 (ex02 自断言) | — |
| **AC-8 外参出厂值一致性 🆕** | rule | Task1 / TR1.4 (6 参数逐项断言 1e-4) | v2 全新 AC，对应用户 feedback 外参 config 化 |
| **AC-9 倾角地址映射一致性 🆕** | rule | Task1 / TR1.2 (4 条 id/addr/array_index/semantic_joint 一一对应) | v2 全新 AC，对应用户 feedback 倾角对齐 v12 WT901C485 |
| **AC-10 独立标定脚本闭环 🆕** | rule | Task0 / TR0.2 三步闭环（手写 txt → ex12 转 YAML → load_config 断言 roll=3.0316） | v2 全新 AC，对应用户 feedback「独立标定脚本 + 外参化」 |
