# ISSUE-005：小臂垂直闭环 reach_touch_down 触限收敛 → 返回 True（假成功）

**严重级**: 🟡 Minor（流程不中断，但后续步骤在异常姿态下执行，可能触发其他 Issue 的边界条件）  
**状态**: 🔍 Open  
**创建日期**: 2026-09-30  
**关联模块**: [shandong_0.5/control_lib/dig_control.py `set_arm_perpendicular`](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L114-L236)

---

## 1. 问题背景 & 触发场景

full_cycle 挖掘阶段 **步骤4 触地闭环（reach_touch_down）** 会调用 `set_arm_perpendicular()`，目标是让小臂对地 pitch 在 85°~95° 区间（物理垂直 ±5°）。

当几何上**小臂不可能达到垂直**（比如大臂 boom_swing 已顶到 0° 或 48° 物理限位，或者挖掘点过于靠外/靠内导致腕点三角约束无解）时：

1. `arm_ctrl_target` 会被 `_clamp("arm_boom", ...)` 截断到 0°/130°（v15 侧；v11 侧则是 0°/95°）
2. 下一步 `arm_ctrl_next` 再次 clamp 后与 `arm_ctrl_target` 相同 → 判定 `reached_touch_down = True`（**触限收敛**）
3. `return True`（假成功），仅打一条 warning，**不中断流程**
4. 此时小臂 pitch 可能是 60° 或 110°，明显不垂直
5. 后续步骤5 `scoop_n_retract` 在非垂直姿态下插入铲斗 → 挖掘轨迹变形，可能铲尖角度不对 → 触发 **ISSUE-003（遇阻纾解）**，甚至物理损伤

---

## 2. 复现步骤

### 最小复现：boom 打到极限再触地
```bash
# 1. 让 boom_swing 先抬到接近 0°（最高点）
# 2. 下发一个 r 很小、z 也小的挖掘点（几何上小臂不可能垂直）
ros2 topic pub -1 /excavator/cmd_full_cycle std_msgs/Float64MultiArray \
  "{data: [0.15, 0.0, -0.05, 0.5, 0.5, 0.8]}"
  # r = 0.15m 接近腕点内半径 (0.09m) + margin 0.03m
```
观察：
- 阶段1步骤4日志显示 `[WARN] 小臂可能无法完全垂直`，但 `set_arm_perpendicular return True`
- 后续 scoop 动作姿态明显异常（小臂斜着插，或根本铲不到土）

---

## 3. 根因代码引用

### 3.1 触限收敛判定 → 假成功
[dig_control.py L148-L165](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L148-L165)

```python
# Step 5: 确认是否真的达到垂直目标
arm_angle_now = self.angle_ctrl.get_joint_angles()["arm_boom"]
arm_ctrl_target = self._clamp("arm_boom", arm_angle_now + arm_angle_delta)  # 被 clamp 到边界
arm_ctrl_next = self._clamp("arm_boom", arm_ctrl_target + arm_angle_delta)  # 再 clamp 一次，和 target 相同

if abs(arm_ctrl_next - arm_ctrl_target) < 1.0:  # 差值 < 1°
    reached_touch_down = True                    # ← 触限收敛！误以为已经到了
    self.logger.warning("小臂可能无法完全垂直 ...")  # 只有 warning，不 return False
```

### 3.2 最终 return 值只看 reached_touch_down，不二次校验实际小臂 pitch
[dig_control.py L228-L236](file:///media/libo/libo_sn7100/ubuntu2204/shandong_ws/src/shandong/v15_action_task/shandong_0.5/control_lib/dig_control.py#L228-L236)

```python
if reached_touch_down:
    # 这里没有再次校验 abs_pitch ∈ [85, 95]
    self.logger.info("✅ 小臂垂直闭环完成 ...")
    return True   # 假成功
else:
    self.logger.error("❌ 小臂垂直闭环超时 ...")
    return False
```

### 3.3 叠加 Issue-001（限位不一致）时，触限收敛更易误判
- v15 `_clamp("arm_boom", ...)` 最大 130°
- v11 实际执行到 95° 就停
- `arm_ctrl_next - arm_ctrl_target` 在 v15 侧是 0° → **在没有任何运动的情况下就被判定为触限收敛**

---

## 4. 预期行为 vs 实际行为

| 项 | 预期 | 实际 |
|---|---|---|
| 几何上不可能达垂直 | `return False` 或至少 `reach_touch_down=False` → 调用方决定是否中止 | `return True`，调用方认为成功 |
| 最终 return True 的条件 | `abs_pitch ∈ [85, 95]`（真正垂直） | `reached_touch_down`（可能是触限收敛） |
| 后续 scoop 动作 | 前提是小臂已真正垂直 → 正确的铲斗切入角度 | 铲斗姿态异常 → 铲土效率低、遇阻概率高 |
| 触限收敛时的状态码 | `WARN_LIMIT_REACHED_BUT_NOT_VERTICAL` 区分 | 统一 info "✅ 闭环完成"，掩盖问题 |

---

## 5. 修复方案建议（未实施）

### 方案 A（推荐）：return True 前增加"实际 pitch 校验"门槛
```python
if reached_touch_down:
    # 二次校验：真正的对地物理垂直才 return True
    if abs_pitch < 85.0 or abs_pitch > 95.0:
        self.logger.error(
            f"❌ 小臂垂直闭环触限收敛但未达目标：pitch={abs_pitch:.1f}° ∉ [85,95]°"
            f" arm_boom(目标 {arm_ctrl_target:.1f}° / 实际 {arm_angle_now:.1f}°)"
        )
        return False  # 明确失败，让上层 stop_on_first_fail 中止挖掘
    self.logger.info("✅ 小臂垂直闭环完成 ...")
    return True
```

### 方案 B：触限收敛直接作为失败处理（更激进）
删除 `if abs(arm_ctrl_next - arm_ctrl_target) < 1.0: reached_touch_down = True` 这一段，完全依赖"实际 pitch 入 [85,95]"来判定完成。若 20 步内 pitch 没到范围 → 超时 return False。

缺点：某些"刚好差一点但铲土质量没问题"的场景也会被判定失败，降低成功率。

### 方案 C：增加参数 allow_partial_success（兼容两种模式）
```python
def set_arm_perpendicular(self, ..., allow_partial_success: bool = False):
    if reached_touch_down and (abs_pitch < 85 or abs_pitch > 95):
        if allow_partial_success:
            self.logger.warning(f"⚠️ 小臂仅触限成功 (pitch={abs_pitch:.1f}°)，继续执行但风险自负")
            return True
        else:
            return False
```
调用方在 `plan_and_execute_dig` 中传 `allow_partial_success=False` 严格模式；调试脚本传 True 允许非严格。

---

## 6. 关联 Issue

- **ISSUE-001（限位不一致）**：叠加后更容易出现"v15 侧认为触限收敛、v11 侧角度根本没动过" → 假成功概率大幅上升
- **ISSUE-003（遇阻纾解）**：小臂姿态不垂直 → 铲斗切入角度不对 → 插土遇阻概率高 → 纾解死等 25s
- **ISSUE-004（z 静默修改）**：挖掘点 z 被抬高后，几何约束变化也可能引发小臂垂直闭环触限收敛
