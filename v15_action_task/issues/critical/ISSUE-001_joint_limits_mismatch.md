# ISSUE-001：关节限位 v11 / v15 不一致 → 目标超限后 timeout 假卡死

**严重级**: 🔴 Critical（阻断 full_cycle 流程执行，无法通过重发指令自动恢复）  
**状态**: 🔍 Open  
**创建日期**: 2026-09-30  
**关联模块**:
- v11 物理执行端: [v4_control_closed/angle_controller.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v4_control_closed/angle_controller.py)
- v15 库层限位: [action_library/utils/joint_limits.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/action_library/utils/joint_limits.py)
- v15 配置限位: [config/default_config.yaml](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/config/default_config.yaml)
- v15 dig_control Fallback 限位: [shandong_0.5/control_lib/dig_control.py](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py)

---

## 1. 问题背景 & 触发场景

通过 v11 GUI 或直接向 `/excavator/cmd_full_cycle` 发布 **接近工作空间外边界** 的挖掘/卸料目标点时：

- 目标点 r ≈ 1.0m（最远外伸） → IK 解会需要 `arm_boom` 接近 130°
- 目标点 z < 0（下挖）且 r 较小 → IK 解会需要 `bucket_arm` 接近 45°
- 剧本 `scoop_n_retract` 的 bucket_arm 硬编码值（如 `bucket_100` 等）

此时会出现：
1. v15 剧本/IK 认为目标合法（因为 v15 限位允许到 130°/45°）
2. v15 下发目标到 v11 AngleController
3. v11 AngleController **静默截断**到物理最大 95°/20° 后开始运动
4. v15 上层 `JointAngleController._poll_until_settled()` 持续轮询
5. 用 v15 自己的 `joint_limits.clamp_pose()`（130°/45°）判定目标 vs 当前实际 95°/20°
6. 差值 > tolerance_deg → 判定"还没到位"，继续等
7. 直到 25s/30s timeout → `return success=False` → 剧本中断
8. **机械臂实际上已经走到物理限位、但 v15 以为它没动到位 → 假卡死**

---

## 2. 复现步骤

### 最小复现：手动单关节下发超限目标
```bash
# 确保 v4 angle_controller_node 和 v15 main.py 都在跑
ros2 topic pub /excavator/cmd_joint std_msgs/Float64MultiArray \
  "{layout: {dim: [{label: 'joints', size: 4, stride: 4}]}, data: [0, 110, 0, 0]}"
  # arm_boom = 110°（> v11 物理 95°，< v15 限位 130°）
```

**预期**: 立刻报错或下发前就 clamp 到 95° 并成功到位  
**实际**: v11 侧走到 95° 就停了；v15 侧一直轮询，25s 后 timeout 返回 False

### full_cycle 复现：远端挖掘点
```bash
ros2 topic pub -1 /excavator/cmd_full_cycle std_msgs/Float64MultiArray \
  "{data: [0.90, 0.0, -0.05, 0.5, 0.5, 0.5]}"
  # dig_x=0.9 接近 workspace 外边界 (腕点外半径 1.03m)
```
- 阶段1 IK 解 arm_boom 会逼近 130°
- 走到 95° 就停，步骤3/5 scoop_n_retract 或 insert_bucket 报 timeout

---

## 3. 根因代码引用

### 3.1 限位不一致（核心）

| 定义位置 | arm_boom min/max | bucket_arm min/max | boom_swing min/max |
|---|---|---|---|
| v11 AngleController.\_\_init\_\_ [angle_controller.py L28-L33](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v4_control_closed/angle_controller.py#L28-L33) | **[-5, 95]** ✅ 物理真值 | **[-95, 20]** ✅ 物理真值 | [0, 48] |
| v15 `joint_limits.py` [joint_limits.py L34-L39](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/action_library/utils/joint_limits.py#L34-L39) | [0, 130] ❌ | [-95, 45] ❌ | **[-5, 55]** ❌ |
| v15 `default_config.yaml` [default_config.yaml L39-L55](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/config/default_config.yaml#L39-L55) | [0, 130] ❌ | [-95, 45] ❌ | [0, 48] |
| v15 `dig_control._clamp` Fallback [dig_control.py L51-L57](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L51-L57) | [0, 130] ❌ | [-95, 45] ❌ | [0, 48] |

### 3.2 v11 下发前静默截断（不向上游反馈）
[angle_controller.py L183-L190](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v4_control_closed/angle_controller.py#L183-L190)

```python
joint_limits = self.joint_limits[joint_name]
if target_angle < joint_limits["min_angle"]:
    target_angle = joint_limits["min_angle"]
if target_angle > joint_limits["max_angle"]:
    target_angle = joint_limits["max_angle"]
# 没有将"被截断过"这个状态反馈给 v15 上层
```

### 3.3 v15 轮询判定时用的是 v15 自己 clamp 的目标
[angle_control.py L117-L123](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/angle_control.py#L117-L123)

```python
current = self.angle_ctrl.get_joint_angles()  # v11 传感器真实角度
target_clamped = clamp_pose({joint: target})  # 用 v15 joint_limits.clamp_pose() 得到的 target
diff = abs(target_clamped[joint] - current[joint])
if diff <= tolerance: settled = True          # 只要 v11 截断了，diff 永远 = 130°-95° = 35° > 容差
```

### 3.4 timeout 返回 False 导致全剧本中断
[angle_control.py L150-L168](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/angle_control.py#L150-L168) + 调用方 `stop_on_first_fail=True`

```python
# execute_full_cycle_dig_dump
if not success and stop_on_first_fail:
    self.logger.error(f"... 步骤失败，中止挖掘流程")
    self.stop_all_motion()
    return False
```

---

## 4. 预期行为 vs 实际行为

| 项 | 预期（正确） | 实际（当前 Bug） |
|---|---|---|
| 下发 arm_boom=110° | v15 下发前就 clamp 到 min(v15,v11) = 95°；或 v11 截断后向上游发反馈 topic | v15 认为目标是 110°，v11 实际按 95° 走，两边无同步 |
| timeout 判定 | 若目标本身就超出物理限位 → 立刻 fail + 明确错误码（LIMIT_EXCEEDED） | 死等 25s → 返回普通 False，错误信息只有"未达到目标角度" |
| full_cycle 中断后恢复 | 提示用户输入更保守的点；或自动 fallback 到最近可达点 | 用户需要手动重发指令，且无报错区分是"目标不合法"还是"真的卡死" |

---

## 5. 修复方案建议（未实施，供讨论）

### 方案 A（推荐）：统一 v15 限位对齐 v11 物理真值
一次性修改 3 处 v15 限位定义：
- `action_library/utils/joint_limits.py` → arm_boom max=95, bucket_arm max=20, boom_swing min=0 max=48
- `config/default_config.yaml` → 同上
- `dig_control._clamp` Fallback → 同上（或直接删 Fallback，统一走 joint_limits 模块）

优点：从源头消除不一致，所有模块（IK/Workspace/clamp_pose/轮询判定）用同一套真值  
缺点：之前基于 130° 上限训练的轨迹/数据集需要重新评估

### 方案 B：v11 截断后向上游反馈 "实际下发目标"
v11 `set_target_angle()` 返回 (clamped_target, was_clamped) 或额外发一个 topic `/excavator/cmd_joint_feedback`；v15 轮询时以"实际下发目标"为准而不是自己再 clamp 一次。

优点：保留 v15 宽松限位，兼容历史参数  
缺点：需要双向通信协议改动，复杂度高

### 方案 C（短期绕路）：v15 超时前检测"目标 > v11 物理上限"
在 `_poll_until_settled` 内加额外条件：若 `current_angle` 已经卡在 v11 物理限位（当前角度与 max 差 < 1°）且与目标差值仍大 → 立刻返回 `limit_exceeded=True`，不等到 25s。

优点：改动最小  
缺点：属于"事后检测"，不能避免走到物理限位本身；而且需要在 v15 代码里硬编码一份 v11 限位表（破坏模块边界）

---

## 6. 关联 Issue

- **前置依赖**：无（此 Issue 是所有卡死的最常见根因）
- **可能引发**：若用户反复重试会加速 Issue-002 传感器疲劳 / Issue-003 遇阻纾解循环
