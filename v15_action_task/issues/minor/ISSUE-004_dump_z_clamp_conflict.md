# ISSUE-004：dump_z 入口 max(0.5) 与 plan_and_execute_dump 内 max(0.3) 双重静默修改

**严重级**: 🟡 Minor（不直接卡死，但易引发"用户输入与实际动作不符"的误会 + 边界情况 IK 无解）  
**状态**: 🔍 Open  
**创建日期**: 2026-09-30  
**关联模块**:
- full_cycle 入口回调: [shandong_0.5/main.py `cmd_full_cycle_cb`](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L347-L355)
- dump 阶段内部 clamp: [shandong_0.5/control_lib/dig_control.py `plan_and_execute_dump`](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L603-L610)

---

## 1. 问题背景 & 触发场景

用户通过 `/excavator/cmd_full_cycle` 第 4-6 个数指定卸料点 `(dump_x, dump_y, dump_z)` 时：

- **第 1 次 clamp（入口层）**：`dump_z_use = max(0.5, dump_z_raw)`（代码实际是 0.5，但注释写的是 `<1.0 → 1.0`，存在代码注释不一致）
- **第 2 次 clamp（dump 阶段内部）**：`z_use = max(z_user, 0.3)`
- 结果：当用户输入 `dump_z = 0.1` 时，实际卸料点 z 变成 0.5，而非用户预期的 0.1
- 若用户正好把 dump 点设在"刚好 z=0.1 可达、z=0.5 不可达"的边界 → **z 被抬高后 workspace/IK 失败，流程中断**
- 更隐蔽的风险：若未来有人把入口的 0.5 改成注释里写的 1.0，叠加 dump 内部再 max(0.3)（等于强制 1.0），会导致所有低卸料点都被强制抬到 1.0，可能超过 workspace 上边界

---

## 2. 复现步骤

### 最小复现：低 z 卸料点
```bash
ros2 topic pub -1 /excavator/cmd_full_cycle std_msgs/Float64MultiArray \
  "{data: [0.6, 0.0, 0.0, 0.6, 0.0, 0.1]}"
  # dump_z = 0.1（用户想要低卸料）
```

**预期**: 要么拒绝（明确错误码：`DUMP_Z_TOO_LOW`），要么按实际传入的 0.1 走 workspace 预检  
**实际**: 静默改成 z=0.5，卸料轨迹比用户预期高 40cm；若 (0.6, 0.0, 0.5) 刚好 IK 无解 → 流程中断且错误信息不指向 z 被修改

### 边界复现：z=0.1 可达、z=0.5 不可达
精心挑选一个点，例如 workspace 奇异位姿附近 r ≈ 0.95m，z=0.1 能过预检、z=0.5 时腕点到中心距离超外半径 → 实际被抬高 0.5 后预检失败。

---

## 3. 根因代码引用

### 3.1 第 1 次 clamp（full_cycle 入口）
[main.py L347-L355](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/main.py#L347-L355)

```python
# 修复: 强制卸料点 z 不小于 1.0 防止卸料点过低倒土倒在车帮上
if len(cmd_list) == 6 and cmd_list[5] < 1.0:  # ← 注释写的 1.0
    cmd_list[5] = 1.0                         # ← 但实际代码也是 1.0（需要核实：当前看到的是 0.5？）
    self.get_logger().warning(f"修正 dump_z={cmd_list[5]}")
```

⚠️ 注意：`dig_control.py` 中引用的是 `max(0.5, dump_z_raw)`，但本处 `main.py` 写的是 `<1.0 → 1.0`，存在**代码与注释不一致甚至双重不一致**，需要二次确认实际生效代码。

### 3.2 第 2 次 clamp（dump 阶段内部）
[dig_control.py L437-L440](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L437-L440)

```python
# 卸料安全高度兜底：避免用户误传过低（如 z<0 插到地里）
z_use = max(z_user, 0.3)
```

与上层 0.5/1.0 规则不一致 → 入口 clamp 后的值会覆盖这个兜底，但若入口被人注释掉，此处兜底又从 0.3 起，行为不一致。

### 3.3 入口与 dump 内部"静默修改"均无显式日志到 ROS Topic
两处都只 `logger.warning` 打印到节点 stdout，**未发诊断 Topic**，ROS2 上层调用方（v11 GUI）看不到 z 被修改的事实，UI 上仍显示用户输入的原始 z，造成 UI 与实际不一致。

---

## 4. 预期行为 vs 实际行为

| 项 | 预期 | 实际 |
|---|---|---|
| dump_z < 安全阈值 | 显式错误码 `DUMP_Z_TOO_LOW <X.X>`，流程不开始 | 静默改到 0.5/1.0 继续执行，UI 显示错误的 z |
| 安全阈值来源 | 单一配置文件项（default_config.yaml），全链路一致 | 硬编码 2 处（main.py 和 dig_control.py），值不同（0.5 vs 0.3 vs 注释 1.0） |
| 修改反馈 | 发 `/excavator/diagnosis {clamped: {dump_z: {from: 0.1, to: 0.5}}}` | 仅 stdout warning |
| 注释与代码 | 注释和代码一致 | 注释写 `<1.0 → 1.0`，另一处代码用 `max(0.5, ...)` |

---

## 5. 修复方案建议（未实施）

### 方案 A（推荐）：统一到配置层 + 显式拒绝而非静默修改
1. 在 `default_config.yaml` 新增：
   ```yaml
   dump:
     min_safe_z: 0.5      # 统一值，所有入口读这里
   ```
2. `cmd_full_cycle_cb` 中不再静默 clamp，改为：
   ```python
   if cmd_list[5] < cfg['dump']['min_safe_z']:
       self.get_logger().error(f"dump_z={cmd_list[5]} < min_safe_z={cfg['dump']['min_safe_z']}, REJECTED")
       return  # 或发失败 state_code = DUMP_Z_TOO_LOW
   ```
3. `plan_and_execute_dump` 中 `max(z_user, 0.3)` 删除或改为读同一个 `min_safe_z` 校验，不一致即抛异常。

### 方案 B（保留静默修改但可观测）
保留静默 clamp 行为，但：
- 两处用同一个常量（定义在 dig_control.py 顶部 `MIN_DUMP_Z = 0.5`）
- 额外发诊断 topic `/excavator/input_clamped`，内容 `{dig_z: {from:..., to:...}, dump_z: {...}}`，让 GUI 能同步更新 UI 显示

### 方案 C（快速 workaround）
把注释统一：当前若实际代码是 `max(0.5, ...)`，就把 `<1.0 → 1.0` 的注释改成 `<0.5 → 0.5`，避免误导后续维护人。

---

## 6. 关联 Issue

- **ISSUE-001 叠加**：z 被抬高后，IK 解 arm_boom/bucket_arm 角度可能落入 v11/v15 不一致区间，触发限位卡死
- **ISSUE-003 叠加**：z 被抬高后卸料点离车帮太近，用户未察觉 → 铲斗磕车帮，触发 bucket_arm stall，进入 Issue-003 纾解死循环
