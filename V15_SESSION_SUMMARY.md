# V15 挖掘机控制系统对话总结

**日期**: 2026-09-10
**项目**: shandong_ws 挖掘机自动化控制
**版本**: v15_action_task/shandong_0.5

---

## 一、环境问题修复

### 1.1 Python 环境

**问题**: miniconda Python 3.14 与 ROS2 Humble 不兼容
- numpy 2.2.6 与 cv_bridge、matplotlib 不兼容
- 系统默认 Python 被 miniconda 覆盖

**解决方案**:
- 降级 numpy: `pip3 install --user "numpy<2"` → 1.26.4
- 使用系统 Python: `/usr/bin/python3` 而非 miniconda 的 python3
- 已生成 `requirements.txt` 保存在 `/media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/`

### 1.2 ROS2 依赖

**关键依赖版本** (系统 Python 3.10.12):
- numpy: 1.21.5
- rclpy: 3.3.19
- cv-bridge: 3.2.1
- opencv-python: 4.12.0
- scipy: 1.15.3

---

## 二、代码修改记录

### 2.1 v11_sensor_bridge.py

**文件**: `v15_action_task/shandong_0.5/v11_control/v11_sensor_bridge.py`

**修改内容**:
1. 新增 `/excavator/sensor_data` 话题订阅 (V4 14路绝对倾角)
2. 新增 `_FrameCache.set_sensor()` / `get_sensor()` / `wait_sensor()`
3. 新增 `V11SensorBridge.get_current_sensor_data()` API
4. 新增 `_on_sensor_data()` 回调函数

### 2.2 ros2_multimodal_gui.py

**文件**: `v11_multimodal_dataset_collection/ros2_multimodal_gui.py`

**修改内容**:
1. 新增 `Float64MultiArray` 导入
2. 新增 `pub_sensor` publisher (话题: `/excavator/sensor_data`)
3. 新增 `publish_sensor_data()` 方法
4. 主循环中调用 `publish_sensor_data(self.sensor_data)`

### 2.3 angle_control.py

**文件**: `v15_action_task/shandong_0.5/control_lib/angle_control.py`

**修改内容**:
1. 新增 `_get_current_sensor_data()` 方法
2. 兼容链路: `ctl.bridge.get_current_sensor_data()` 或 `ctl.get_current_sensor_data()`

### 2.4 main.py

**文件**: `v15_action_task/shandong_0.5/main.py`

**修改内容**:
1. `V11ControllerAdapter` 新增 `get_current_sensor_data()` 桥接方法
2. `cmd_dump_cb` 返回值解包修复: `ok, pose = self.dig_ctrl.plan_and_execute_dump(dump_xyz)`

### 2.5 dig_control.py

**文件**: `v15_action_task/shandong_0.5/control_lib/dig_control.py`

**修改内容**:

#### 限位修复 (已回滚)
- L66-69: 注释修正为 `[0, +130]`（用户物理标定）
- L77: fallback 改为 `(0.0, 130.0)`
- grep "-125" 零残留

#### 新增方法
- L85-97: `_clamp()` 方法（按限位钳制角度）
- L114-236: `set_arm_perpendicular()` 方法（传感器闭环垂直控制）

#### 格式修复
- L200: `{delta:+d}` → `{delta:+.0f}`（float 格式化修复）

#### 挖掘序列优化
- L319-320: 铲斗角度改为固定值
  - `scoop_75 = self._clamp("bucket_arm", -20.0)`（原为 `_bucket_pct(75.0)`）
  - `scoop_100 = self._clamp("bucket_arm", 5.0)`（原为 `_bucket_pct(100.0)`）
- L336: 预位铲斗改为 `-75.0°`（原为 `-55.0°`）
- L348: 注释掉预位大臂步骤（减少运动幅度）
- L353: 注释掉 IK 铲斗步骤（避免外推）
- L364-368: 小臂垂直改为固定 65°（原为传感器闭环）

#### 超时与容差优化
- L255: 超时改为 `30.0 if joint == "swing_yaw" else 25.0`
- L256: 容差改为 `tolerance_deg=5.0`（原为 2.0）

#### 返回值解包修复
- L600: `dump_ok, _dump_pose = self.plan_and_execute_dump(dump_xyz)`
- L694: `dump_ok_b, _dump_pose_b = self.plan_and_execute_dump(auto_dump_xyz)`

#### 限位下限修改
- 多处 `max(1.0, ...)` 改为 `max(0.5, ...)`（z 高度下限）

---

## 三、待修复问题

### 3.1 WorkspaceChecker 调用方式

**问题**: `WorkspaceChecker(self.cfg)` 缺参数，应用工厂方法

**位置**:
- L290: `WorkspaceChecker(self.cfg)`
- L446: `WorkspaceChecker(self.cfg)`

**修复方案**:
```python
# 改为
_ws = WorkspaceChecker.from_config(self.cfg)
```

### 3.2 小臂外推补偿

**问题**: 大臂落下时末端后退，小臂应先外推补偿

**位置**: L351

**建议方案**:
```python
# 当前
("arm_boom", pose_target["arm_boom"], "步骤3/7 下探(小臂调整到挖掘深度姿态)")

# 改为（外推15° 补偿）
("arm_boom", self._clamp("arm_boom", pose_target["arm_boom"] - 15.0), "步骤3/7 下探(小臂外推补偿)")
```

### 3.3 boom_swing 超时优化

**问题**: 大臂经常超时（差几度）

**位置**: L255

**建议方案**:
```python
# 改为
timeout = 40.0 if joint == "boom_swing" else (30.0 if joint == "swing_yaw" else 25.0)
```

---

## 四、关键文件结构

```
shandong_ws/src/shandong/
├── v15_action_task/
│   ├── shandong_0.5/
│   │   ├── main.py                          # ROS2 主节点
│   │   ├── control_lib/
│   │   │   ├── dig_control.py               # 挖掘控制核心
│   │   │   ├── angle_control.py             # 角度控制
│   │   │   └── point_control.py             # 点位控制
│   │   └── v11_control/
│   │       └── v11_sensor_bridge.py         # 传感器桥接
│   ├── motion/
│   │   └── workspace.py                     # WorkspaceChecker
│   ├── kinematics/
│   │   ├── forward.py                       # 正运动学
│   │   └── inverse.py                       # 逆运动学
│   └── config/
│       └── default_config.yaml              # 默认配置
├── v11_multimodal_dataset_collection/
│   └── ros2_multimodal_gui.py               # V11 GUI (底层驱动)
└── requirements.txt                         # Python 依赖清单
```

---

## 五、话题接口汇总

| 话题 | 消息类型 | 功能 | 回调函数 | 核心处理函数 |
|------|----------|------|----------|--------------|
| `/excavator/target_pose` | JointState | 角度控制 | `target_pose_cb` | `angle_ctrl.move_pose_serial()` |
| `/excavator/target_point` | PointStamped | 点位控制 | `target_point_cb` | `point_mover.move_to_point_serial()` |
| `/excavator/cmd_dig_point` | PointStamped | 阶段1挖掘 | `cmd_dig_cb` | `dig_ctrl.plan_and_execute_dig()` |
| `/excavator/cmd_transit_yaw` | PointStamped | 阶段2回转 | `cmd_transit_cb` | `dig_ctrl.plan_and_execute_transit()` |
| `/excavator/cmd_dump_point` | PointStamped | 阶段3卸料 | `cmd_dump_cb` | `dig_ctrl.plan_and_execute_dump()` |
| `/excavator/cmd_full_cycle` | Float64MultiArray | 全流程A | `cmd_full_cycle_cb` | `dig_ctrl.execute_full_cycle_dig_dump()` |
| `/excavator/cmd_dig_transit` | Float64MultiArray | 全流程B | `cmd_dig_transit_cb` | `dig_ctrl.execute_dig_and_transit()` |

---

## 六、测试命令示例

### 阶段1 挖掘
```bash
ros2 topic pub --once /excavator/cmd_dig_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'base_link'}, point: {x: 1.28, y: 0.01, z: 0.01}}"
```

### 全流程A (挖掘+卸料)
```bash
ros2 topic pub --once /excavator/cmd_full_cycle std_msgs/msg/Float64MultiArray \
  "{layout: {dim: [{label: 'dig_dump', size: 6, stride: 6}], data_offset: 0}, \
    data: [1.28, 0.01, 0.01, 0.18, 0.85, 0.47]}"
```

---

## 七、关节限位

| 关节 | 最小值 | 最大值 | 说明 |
|------|--------|--------|------|
| swing_yaw | -180° | +180° | 回转 |
| boom_swing | 0° | 48° | 大臂（0=最高，48=最低） |
| arm_boom | 0° | 130° | 小臂 |
| bucket_arm | -95° | +45° | 铲斗（负=开斗，正=收斗） |

---

## 八、注意事项

1. **单关节串行**: 所有动作严格单关节串行执行，禁止并发
2. **防并发保护**: 命令执行中会拒绝新指令
3. **z 高度下限**: 挖掘点 z ≥ -0.2m，卸料点 z ≥ 0.5m
4. **WorkspaceChecker**: 需要用 `from_config()` 工厂方法，不能直接构造