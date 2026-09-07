# Debug session: s05-points-unreachable

**Phenomenon (verbatim 用户报告)**: 
> 「通过 RViz2 观察，上述 2+5 个例子只有 `python3 smoke_point_ctrl.py --x 1.2 --y 0 --z 0.8` 可用」
> 「上述 2+5 个例子」= README §8 中提到的 smoke_angle_ctrl 单关节×4 + smoke_point_ctrl 的 4 个极限点 + 下挖/举升/自动 closest，共 7 条；仅 smoke_point_ctrl (1.2,0,0.8) 一条在真机 RViz2 中表现为不动。

**Session ID**: `s05-points-unreachable`（session_status: OPEN）
**Working scope rules (严格遵守用户指令)**:
  * 0 修改 `v15_action_task/` 下除 `shandong_0.5/` 子目录以外的任何原始文件（motion, config, control_core, kinematics, v15_main_node.py 等全部锁死不碰）
  * 只能改动 `shandong_0.5/control_lib/*.py` 和 `shandong_0.5/scripts/*.py`
  * 验收：43TR（v15 原回归）必须仍然 43/43 PASS

**7 test cases checklist (verbatim)**:
| # | Case | Expected | Status | Note |
|---|---|---|---|---|
| A1 | smoke_angle_ctrl.py 4 × move_joint (swing/boom/arm/bucket) | 每次只动 1 关节；RViz2 RobotModel 相应 1 关节转动 | ⚠️ 用户说只有 (1.2,0,0.8) 可用，A1 可能表现为「也不动」？需要插桩确认是哪个 case |
| A2 | smoke_angle_ctrl.py move_pose_serial (-45,-5,100,0) | 串行 swing→boom→arm→bucket，每步只动 1 | ⚠️ 同上，待复现日志 |
| P0 | smoke_point_ctrl.py --x 1.2 --y 0 --z 0.8 (默认 dig) | success=True，最终到达 ✅ PASS（用户说唯一可用） |
| P1 | smoke_point_ctrl.py --x 1.45 --y 0 --z 0.6 (README 极限点 ①) | success=True | ❌ FAIL |
| P2 | smoke_point_ctrl.py --x 0.9 --y 0 --z -0.08 --mode dig (README 极限 ③) | success=True | ❌ FAIL |
| P3 | smoke_point_ctrl.py --x 0.9 --y 0 --z 1.6 --mode transit (README 极限 ④) | success=True | ❌ FAIL |
| P4 | smoke_point_ctrl.py --x 2.0 --y 0 --z 0.8 --auto-closest | 使用 closest_reachable_point 后 success=True | ❌ FAIL |

---

## 5 falsifiable hypotheses (按概率从高到低排序)

### H1 — IK 默认 bucket_range_deg 与 joint limits（bucket_arm ∈ [-95°,+45°]）**不一致**，且在极限点（P1 P2 P3）候选 17 条角没覆盖到可行区，导致搜索型 IK 拿不到解 → 表现为 `ik_target_pose is None` / `ik_success=False / ik_reason=""` / 最终 `success=False  不会动`

- P0（1.2,0,0.8）正好落在中等臂展 + 中等高度的中心，bucket_range(-60,+20)（InverseKinematics 默认就是这个）能覆盖，所以能用；但 P1（1.45远）需要铲斗**更开**（bucket_angle 负得更大，≤-70°）或 P2（下挖-0.08）需要 bucket_angle ≤-80° 才出几何解；P3（1.6 高）需要 bucket_angle ≥0° 或 +20~+45° 区间解——InverseKinematics 默认 (-60,20) 没覆盖到 [-95,+45] 全关节范围。
- Falsifiable：在 point_control.Step2 插桩打印 `self.bucket_range_deg` + 17 candidate 中**每个 candidate 是否 success=True** 的 mask + ik_sol.xxx；如果 P1/P2/P3 的候选全部无 success，且把 range 扩到 (-95,45) 后立刻有解 → 该假设被证据锁定。

### H2 — **transit mode 的 z 最低 0.30m 被误触发** 或 **mode 没正确传入 check_point_reachable**，P3 虽然 `--mode transit` 但 Point(0.9,0,1.6) 其实 z=1.6≥0.30 没问题；但 P2 `--mode dig` **没真传进去**，反被当作 transit 校验 (z=-0.08 < 0.30) → Layer3 transit_min_z_violation FAIL → reachable=False 直接 return（用户 RViz 看到 closest_point 橙球但不会去执行，因为默认 `auto_use_closest_if_unreachable=False`）。
- Falsifiable：打印 WS 预检里的 `mode_use` + `report.reasons`；如果 P2 打印里 `mode_use=transit` 或 reasons 首条是 `transit_min_z_violation` 就确认。

### H3 — **smoke scripts 里的 `_spec.loader.exec_module(_pc_mod)` 导致的 shandong_0_5 子包相对路径 & dataclass `__module__` 不一致**（前几轮修过）被 P1/P2/P3 的某些分支触发 **point_control.py 内部 getattr `search_bucket_angle` 返回 None / ik_sol 是 `Optional[IKSolution]` 为 None → ik_target_pose=None → 不会执行串行**。这种是"某些参数组合必现某些组合不现"，例如 P0 的 IK 走 solve_bucket_pose 指定 guess_deg 分支；P1/P2/P3 走 search_bucket_angle None 分支但模块加载异常
- Falsifiable：插桩 ik_sol 前 `type(ik_sol).__name__` + `id(ik_sol)` + 异常栈；如果 P1/P2/P3 在 ik 层抛异常被 `ik_exception:AttributeError: Optional[IKSolution] has no attribute ...` 捕获并填 ik_reason → H3 成立。

### H4 — **执行串行 move_pose_serial 过程中每一步 `ctl.get_pose_blocking(2.0s)` 在真机 RosV14Adapter 里 timeout 了 → 第一步 swing_yaw 就 fail 了**（move_pose_serial 默认 `stop_on_first_fail=True`）→ 日志里 `per_joint[0].success=False reason=timeout@8s ...` 就停止，用户 RViz2 看就是「整个都不动」（因为 swing 都没走完后面都没启动）；P0 成功可能是因为 swing_yaw=0.0°，它不用动直接 reached，第二步 boom_swing 也只动一小段，剩下的在 timeout 前能到，而 P1 swing=0 但 boom 需要伸更长，臂长/时长过了 timeout；P2 可能 bucket_arm=-83° 超过默认 `per_joint_timeout_s=10s`
- Falsifiable：插桩打印 `exec_r per_joint[*].reason` + `waited_s`；如果 P1 的 arm 或 bucket 报 timeout 就确认 H4。

### H5 — **`WorkspaceChecker.from_config` 构造时 swing limits、WS 几何参数（min_ground_z / min_transit_z）没对齐**，P2/P3/P4 的 `report.reasons` 里包含意外的 swing_yaw_out_of_range 或 ground_penetration（dig 模式下本应关闭 ground_penetration，但我们反射 from_config 时可能 mode 没保存到 ws 对象，check_point_reachable 用的仍是默认 mode=transit）—— 等价于 H2 更细的变种。
- Falsifiable：插桩打印 ws 对象的 `min_ground_z / min_transit_z / swing limits / 当前 ws.mode` + `report.success + reasons`；如果 mismatch 则 H5 定。

---

## Step 1 instrumentation plan (s05-points-unreachable)

* 不对 `v15_action_task/shandong_0_5/` 之外做任何改动
* 插桩位置 1: control_lib/point_control.py `SingleJointPointMover.move_to_point_serial` → 每步前后 report 到 Debug Server：
  * WS 预检后 (reachable, mode_use, reasons, closest)
  * IK 候选前 (range, N, bucket_guess_deg is None? )
  * IK 结果后 (ik_success, ik_reason, pose keys len, ik_final_pose_deg clamped after angle_control clamp 记录)
  * 执行前 (order, ik_target_pose_deg)
  * 每个 per_joint 结果后 (joint, success, reason, waited_s, target_after_clamp_deg, final_joint_deg)
  * FK 闭环 (final_tip_xyz, tip_error_3d_m, overall success)
* 插桩位置 2: control_lib/angle_control.py `JointAngleController.move_joint` 每步前后（辅助 H4 timeout 定位）
* 所有插桩以 `#region debug-point s05-points-unreachable/XXX ... #endregion` 形式包起来，验证完后可一键删除
* 不改动业务流程/返回字段语义（仅加 reporting 日志和网络 report）

