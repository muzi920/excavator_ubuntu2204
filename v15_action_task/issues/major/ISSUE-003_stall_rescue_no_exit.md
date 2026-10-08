# ISSUE-003：遇阻纾解穷尽后无失败退出 → 死等 25s timeout 假卡死

**严重级**: 🟠 Major（长时间堵住流程，且液压持续顶到极限值会加剧系统负载/发热）  
**状态**: 🔍 Open  
**创建日期**: 2026-09-30  
**关联模块**:
- 遇阻纾解逻辑: [v4_control_closed/angle_controller.py `move_joint_to_angle` stall detection](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v4_control_closed/angle_controller.py#L306-L424)
- 上层超时判定: [v15_action_task/shandong_0.5/control_lib/angle_control.py `_poll_until_settled`](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/angle_control.py#L150-L168)

---

## 1. 问题背景 & 触发场景

full_cycle 挖掘阶段，铲斗碰到硬土/石块/金属异物（物理极限非标定极限），导致某个关节**无论怎么加大液压都推不动**：

1. 角度变化 < 0.5° 持续 1.5s → stall detected
2. v11 `AngleController` 液压输出每次 +200mV（从 2000 起步 → 2200→2400→…→4500mV 顶满）
3. 若顶到 4500mV 仍不动 → 尝试**邻关节纾解动作**（仅特定关节+方向）：
   - `bucket_arm` 收斗方向遇阻 → 小臂前推 0.5s
   - `arm_boom` 收小臂方向遇阻 → 抬大臂最多 5°（分 2 次）
4. 如果纾解后仍然不动
5. ❌ **没有返回 stall 失败，循环继续** → 液压保持 4500mV 死顶
6. 等到 v15 上层 25s/30s timeout 才返回 False 中断
7. 用户体感："挖不动就停在那里几十秒，像卡死了"

---

## 2. 复现步骤

### 最小复现：在铲斗运动路径上放障碍物
```bash
# 1. 在铲斗前方地面上放置一个明显高/硬物（如砖头 / 金属块）
# 2. 下发深度较深的挖掘点
ros2 topic pub -1 /excavator/cmd_full_cycle std_msgs/Float64MultiArray \
  "{data: [0.5, 0.0, -0.05, 0.5, 0.5, 0.8]}"
# 3. 观察阶段1步骤3/4（insert_bucket 或 scoop_n_retract）
```
预期：几秒内识别"真卡"并返回 fail，给出 stall 错误码  
实际：液压输出冲到 4500mV，纾解也没用，死等 25s 才中断

### 软件模拟：下发不可能目标
用手扶住某个关节（安全提示：需人在旁，仅作软件调试）阻止其运动，模拟物理卡死。

---

## 3. 根因代码引用

### 3.1 stall 判定阈值（合理但单一）
[angle_controller.py L336-L350](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v4_control_closed/angle_controller.py#L336-L350)

```python
if abs(current_angle - last_stall_check_angle) < 0.5:  # 1.5s 内 < 0.5°
    stall_duration += dt
    if stall_duration >= 1.5:
        stalled = True                                  # 进入纾解分支
```
→ 阈值本身合理，但**缺少"纾解失败计数"退出条件**。

### 3.2 液压递增到顶无退出
[angle_controller.py L360-L382](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v4_control_closed/angle_controller.py#L360-L382)

```python
voltage += 200
voltage = min(voltage, 4500)                          # 顶到 4500mV 就不再增长
# ❌ 没有 "if voltage == 4500 and stalled_again_in_next_iter → return False"
```

### 3.3 邻关节纾解也不动，仍然死循环
[angle_controller.py L386-L418](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v4_control_closed/angle_controller.py#L386-L418)

```python
# 纾解动作完成标志:
#   stall_rescue_arm_push_done / stall_rescue_boom_lift_done / stall_rescue_boom_lift2_done
# ❌ 当所有标志都 True（纾解做完），如果还是 stall → 没有 break / return，继续外层 while
```

### 3.4 v15 上层 timeout 才兜底（25s 太长）
[angle_control.py L150-L168](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/angle_control.py#L150-L168)

```python
timeout_time = time.time() + timeout_s  # 25~30s
while time.time() < timeout_time:        # 除非 v11 内部 return，否则 v15 只能等到 25s 结束
    ...
return (False,  # 这里的 False 原因不明：是目标超限？还是 stall？用户分不清
        f"Timeout: did not reach target {joint} within {timeout_s}s.")
```

---

## 4. 预期行为 vs 实际行为

| 项 | 预期 | 实际 |
|---|---|---|
| 液压顶到 4500mV + stall 持续 3s 以上 | `return False, "STALL_HYDRAULIC_MAX"` | 保持 4500mV 死顶 |
| 纾解动作执行完 2 轮仍 stall | `return False, "STALL_RESCUE_FAILED"` | 继续 while 不退出 |
| 返回信息 | 区分"LIMIT_EXCEEDED" vs "STALL" vs "TIMEOUT" | 统一返回 TIMEOUT |
| 中断时间 | 5~8s 内（足够 1.5s 判定 + 2s 液压尝试 + 2 轮纾解） | 25~30s |
| 硬件保护 | 纾解失败后立刻液压中值卸压（2000mV） | 液压保持 4500mV 持续顶 |

---

## 5. 修复方案建议（未实施）

### 方案 A（推荐）：v11 内部纾解穷尽后主动 return False
在 `move_joint_to_angle` stall 分支尾部增加：

```python
# 纾解穷尽标志
rescue_exhausted = all([
    stall_rescue_arm_push_done or (joint_name != "bucket_arm" or direction <= 0),
    stall_rescue_boom_lift_done or (joint_name != "arm_boom" or direction <= 0),
    voltage >= 4500,
])
if rescue_exhausted and stall_duration > 3.0:  # 连续 stall 3s + 纾解穷尽
    self.logger.warning(f"Stall rescue exhausted for {joint_name}, aborting.")
    self._apply_dac_values({DAC_CH_MAP[joint_name]: 2000})  # 卸压
    # 可选：发一个诊断 topic /excavator/diagnosis {stall: true, joint: "..."}
    return False  # 立刻让上层知道失败，不等 25s
```

### 方案 B：v15 上层增加 stall 快速检测
v15 `_poll_until_settled` 中观察 `current_angle` 变化率：若 N 个连续周期（如 20 个 tick = 4s）内变化 < 0.5° 且 DAC 已到顶（通过读回或自定义诊断 topic 判断）→ 提前 break，返回 `STALL_DETECTED` 状态码。

缺点：v15 不知道 v11 的 DAC 输出和纾解状态，需新增 topic 通信，工作量稍大。

### 方案 C：缩短 timeout（简单粗暴但有副作用）
timeout 从 25s 改 10s → 减少卡死时间，但也会降低真实长行程（如 swing 回转 180°）的成功率，不推荐作为主方案。

---

## 6. 关联 Issue

- **ISSUE-001**：若 arm_boom 目标 130° 被 v11 截断到 95°，v15 轮询"以为没到位"，同时 v11 侧"角度已经不动了" → 可能被误判为 stall，触发液压递增 + 纾解，加剧系统负担
- **ISSUE-002**：纾解 20s 过程中一旦传感器数据偶发延迟 → 叠加触发 fatal_stop，双重卡死
- **下游影响**：长时间液压顶 4500mV → 油温升高 → 长期可能加剧 Issue-002 的传感器电子噪声问题
