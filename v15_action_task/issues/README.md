# v15_action_task Issue 记录库

**创建日期**: 2026-09-30  
**关联模块**:
- v15: `v15_action_task/` （控制逻辑、IK、Workspace、剧本编排）
- v11: `v4_control_closed/angle_controller.py` （底层液压执行端）
- GUI: `v11_multimodal_dataset_collection/ros2_multimodal_gui.py` （数据采集 & `/excavator/cmd_full_cycle` 发布端）

---

## 📁 目录结构

```
v15_action_task/issues/
├── README.md                          ← 本文件：总索引 & 状态看板
├── critical/                          ← P0：阻断流程 / 全局急停 / 必须修复
│   └── ISSUE-001_joint_limits_mismatch.md
├── major/                             ← P1：严重影响但有绕路 / 需要修复
│   ├── ISSUE-002_sensor_watchdog_fatal_stop.md
│   └── ISSUE-003_stall_rescue_no_exit.md
├── minor/                             ← P2：边界情况 / 体验问题 / 观察
│   ├── ISSUE-004_dump_z_clamp_conflict.md
│   └── ISSUE-005_arm_vertical_false_success.md
└── closed/                            ← 已解决或已接受的 Issue
```

---

## 🎯 Issue 状态看板

| ID | 标题 | 严重级 | 状态 | 触发链路 | 责任模块 |
|---|---|---|---|---|---|
| 001 | 关节限位 v11/v15 不一致，目标超限导致 timeout 卡死 | 🔴 Critical | 🔍 Open | `/cmd_full_cycle` → execute_full_cycle → IK/剧本 → 目标 >95°(arm) 或 >20°(bucket) → v11 截断 → v15 轮询判定不到位 → 25s 超时中断 | v15 `joint_limits.py` + `default_config.yaml` + v11 `angle_controller.py` |
| 002 | 传感器数据新鲜度看门狗 2s 超时触发 fatal_stop 全局急停 | 🟠 Major | 🔍 Open | 传感器 UDP 丢包/延迟 → `_wait_sensor_fresh()` 2s → fatal_stop=True → 所有执行被拒绝，需重启 v4 控制节点 | v11 `angle_controller.py` AngleController |
| 003 | 遇阻纾解（液压顶到 4500mV + 邻关节纾解）后无失败退出，继续等 25s 超时 | 🟠 Major | 🔍 Open | 硬土/物理碰撞 → 1.5s 不动 → 液压递增 200mV/次 → 到 4500mV 仍不动 → 纾解小臂前推/抬大臂 → 仍不动 → 死等 25s 超时 | v11 `angle_controller.py` stall_detection |
| 004 | dump_z 入口 max(0.5) 与 plan_and_execute_dump 内 max(0.3) 冲突，z 值叠加静默修改 | 🟡 Minor | 🔍 Open | 用户 dump_z < 0.5 → 入口抬到 0.5 → dig_control 内再次抬到 0.3（无变化）→ 若入口注释改为 1.0 时叠加两次 | `main.py` cmd_full_cycle_cb + `dig_control.py` plan_and_execute_dump |
| 005 | 小臂垂直闭环 reach_touch_down 触限收敛返回 True（假成功） | 🟡 Minor | 🔍 Open | 几何上小臂无法达 90°（如 boom 顶到 0°）→ arm_ctrl_next == arm_ctrl_target → 返回 True，后续在异常姿态下继续 | `dig_control.py` set_arm_perpendicular |

---

## 🧪 复现与验证路径（所有 Issue 共享）

启动链路：
```bash
# 终端 1：底层 v4 闭环控制（含 AngleController & 执行端 joint_limits）
ros2 run v4_control_closed angle_controller_node

# 终端 2：v15 控制逻辑（监听 /excavator/cmd_full_cycle）
cd v15_action_task/shandong_0.5
python3 main.py                       # 含 fallback AngleController 轮询 + 剧本

# 终端 3：v11 GUI（发布 /excavator/cmd_full_cycle）
cd v11_multimodal_dataset_collection
python3 ros2_multimodal_gui.py        # 或手动: ros2 topic pub /excavator/cmd_full_cycle std_msgs/Float64MultiArray "{data: [x,y,z,dx,dy,dz]}"
```

---

## 📐 限位对照表（所有 Issue 共用基准）

| 关节 | v11 AngleController (物理执行端) | v15 `joint_limits.py` (库层) | v15 `default_config.yaml` (配置层) | dig_control Fallback |
|---|---|---|---|---|
| **arm_boom** | **[-5°, +95°]** ⚠️ 最小上限 | [0°, 130°] | [0°, 130°] | [0°, 130°] |
| **bucket_arm** | [-95°, **+20°**] ⚠️ 最小上限 | [-95°, 45°] | [-95°, 45°] | [-95°, 45°] |
| **boom_swing** | [0°, 48°] | **[-5°, 55°]** ⚠️ 下限过深 & 上限过高 | [0°, 48°] | [0°, 48°] |
| **swing_yaw** | [-180°, 180°] | [-180°, 180°] | [-180°, 180°] | [-180°, 180°] |

> **以 v11 AngleController 的物理限位为真值**，v15 所有模块必须对齐，否则会出现"v11 截断到位、v15 判定未到位"的 timeout 假卡死。

---

## 🔄 工作流

1. 新增 Issue → 选对应级别子目录，按模板命名 `ISSUE-XXX_short_title.md`
2. 每个 Issue 必须包含：
   - 问题背景 & 触发场景
   - 复现步骤（明确输入参数）
   - 根因代码引用（行号级）
   - 预期行为 vs 实际行为
   - 修复方案建议（可选，未确认修复前不要改代码）
   - 关联 Issue ID（如有依赖）
3. Issue 解决后 → 移至 `closed/` 并在状态看板更新
