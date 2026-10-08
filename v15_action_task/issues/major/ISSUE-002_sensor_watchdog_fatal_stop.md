# ISSUE-002：传感器数据新鲜度看门狗 2s 超时 → fatal_stop=True 全局急停

**严重级**: 🟠 Major（触发后所有动作被拒绝，必须重启 v4 AngleController 节点才能恢复）  
**状态**: 🔍 Open  
**创建日期**: 2026-09-30  
**关联模块**: [v4_control_closed/angle_controller.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v4_control_closed/angle_controller.py)

---

## 1. 问题背景 & 触发场景

执行 `/excavator/cmd_full_cycle` 期间，若发生以下任一情况：
- IMU UDP 广播丢包（WiFi/网络抖动）
- 某个关节传感器临时掉线（线缆松动、电磁干扰）
- 传感器发布频率骤降（< 3Hz）
- 系统调度卡顿，传感器回调被阻塞

会触发 `AngleController` 内部的传感器新鲜度看门狗：

1. `move_joint_to_angle()` 开始前调用 `_wait_sensor_fresh()`
2. 若 350ms 内数据没刷新 → 持续等待，期间模拟量清零（动作卡住）
3. 等够 2s 还不新鲜 → **`trigger_fatal_stop()` → `self.fatal_stop = True`**
4. 之后所有 `set_target_angle()` 调用都会被直接拒绝（检查 `fatal_stop`）
5. 当前剧本全流程中断，**即使传感器后来恢复正常也无法再下发任何指令**
6. 必须**手动 kill 并重启 `v4_control_closed` 节点**才能清除 fatal_stop 状态

---

## 2. 复现步骤

### 最小复现：运动到一半拔掉某个传感器网线/关闭 UDP 源
```bash
# 1. 启动 v4 + v15
# 2. 发起一次长耗时 full_cycle（如挖掘+回转 90°+卸料）
ros2 topic pub -1 /excavator/cmd_full_cycle std_msgs/Float64MultiArray \
  "{data: [0.7, 0.0, 0.0, 0.0, 0.8, 0.8]}"
# 3. 执行到一半（比如 swing_yaw 动作中），临时屏蔽 IMU/传感器 UDP 2 秒以上
#    (用 iptables -A INPUT -p udp --dport <port> -j DROP，或物理拔线)
```

**预期**: 传感器恢复后，当前步骤重试或失败退出，后续步骤可继续；至少不应锁死整个节点  
**实际**: `fatal_stop=True`，后续所有指令被拒绝，需重启 v4 节点

### 边界复现：v11 GUI 点云处理占用主线程导致传感器回调延迟
v11 GUI 点云累积 0.1s 窗口 + numpy 处理若阻塞主线程超过 2s，也会连带触发（因为 v4 和 GUI 在同一台机器时系统调度压力大）。

---

## 3. 根因代码引用

### 3.1 新鲜度判定阈值
[angle_controller.py L104-L113](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v4_control_closed/angle_controller.py#L104-L113)

```python
def _is_sensor_data_fresh(self, joint_name: str, stale_threshold_s: float = 0.35) -> bool:
    now = time.time()
    last_update = self.last_sensor_update_time.get(joint_name, 0)
    age = now - last_update
    # 阈值只有 350ms，偶尔丢 1-2 包就触发等待
    return age <= stale_threshold_s
```

### 3.2 运动前等待传感器刷新（期间模拟量清零 → 动作停顿）
[angle_controller.py L118-L138](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v4_control_closed/angle_controller.py#L118-L138)

```python
def _wait_sensor_fresh(self, joint_name: str, max_wait_s: float = 2.0) -> bool:
    # 每 50ms 检查一次，最多等 2000ms
    if not fresh:
        self._apply_dac_values({...模拟量全 2000 (中值)...})  # 清阀：停
    if timeout and not fresh:
        self.trigger_fatal_stop(f"传感器 {joint_name} 数据超时未更新 >2s")
        return False
```

### 3.3 trigger_fatal_stop 把全局标志置 True，无自动恢复机制
[angle_controller.py L495-L509](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v4_control_closed/angle_controller.py#L495-L509)

```python
def trigger_fatal_stop(self, reason: str = ""):
    self.fatal_stop = True                # 一旦 True，永不恢复（除非外部手动 reset）
    self._apply_dac_values({joint: 2000 for joint in DAC_CH_MAP.values()})
```

### 3.4 所有 set_target_angle() 都会检查 fatal_stop
[angle_controller.py L175-L180](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v4_control_closed/angle_controller.py#L175-L180)

```python
if self.fatal_stop:
    self.logger.error(f"... Fatal stop 已触发，拒绝下发")
    return False
```

### 3.5 关节-传感器映射（bucket / arm 要两个传感器都新鲜，更易触发）
[angle_controller.py L128-L133](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v4_control_closed/angle_controller.py#L128-L133)

```python
sensor_joints = JOINT_SENSOR_MAP.get(joint_name, [joint_name])
# bucket_arm → [bucket, arm]   两个都要新鲜
# arm_boom    → [arm, boom]    两个都要新鲜
# boom_swing  → [boom, swing]  两个都要新鲜
# swing_yaw   → [swing]        单个
```
→ **bucket/arm/boom 卡住时触发 fatal_stop 的概率是 swing_yaw 的 2 倍以上**（两个 sensor 任意一个掉线即触发）。

---

## 4. 预期行为 vs 实际行为

| 项 | 预期 | 实际 |
|---|---|---|
| 传感器短暂丢包 < 1s | 等待+重试，不中断 | 等待中动作停，但未触发 fatal_stop（正常） |
| 传感器丢包 2-3s 后恢复 | 当前步骤 fail 或跳过，**后续步骤仍能下发** | fatal_stop=True 锁死节点，必须重启 |
| 传感器彻底掉线 | 明确错误 + 安全停止 + 暴露恢复接口 | 同上 fatal_stop；无 reset API |
| 恢复接口 | 提供 `/excavator/reset_fatal_stop` service 或 topic | 只能 kill -9 进程重启 |

---

## 5. 修复方案建议（未实施）

### 方案 A（推荐）：fatal_stop 分级 + 自动恢复
- `_wait_sensor_fresh()` 超时 2s → 不直接 fatal，改为 `WARN + 当前步骤 return False`
- 连续 3 次（跨不同步骤）都传感器超时 → 再 fatal_stop
- 新增 ROS Service `/excavator/clear_fatal_stop`，调用后将 `fatal_stop=False`（需额外验证传感器当前新鲜）

### 方案 B：阈值放宽 + 补数逻辑
- `stale_threshold_s` 从 0.35s → 1.0s（允许丢几包），或动态：运动中时更严格，空闲时更宽松
- 等待期间若数据回来，**立刻接着走**，不用等到 max_wait_s 结束（当前代码其实是这样，但 2s 上限太硬）

### 方案 C：v15 上层检测并做软恢复
v15 `JointAngleController.move_joint_to_angle()` 返回 `fatal_stopped` 状态码时，通过 ROS2 service 调用 v4 的 reset 接口（前提是 v4 提供了该接口）。

---

## 6. 关联 Issue

- 与 **ISSUE-001** 叠加：若 arm_boom 卡到 95° 物理限位，动作看起来像"停了"，期间传感器调度刚好又延迟 → 叠加触发 fatal_stop，排查更困难
- 与 **ISSUE-003** 叠加：遇阻纾解 20s 期间一旦有传感器延迟 → fatal_stop，纾解也被中断
