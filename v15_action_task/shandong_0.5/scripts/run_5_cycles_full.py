#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
五轮挖掘-卸料组合全流程 测试脚本
=================================

场景：
  - 挖掘机主控节点 main.py 必须已启动，且 ROS_DOMAIN_ID=0 相同
  - 订阅 /excavator/state (std_msgs/String)：idle=空闲、working=忙碌
  - 当 state == idle，才会发布 /excavator/cmd_full_cycle
  - 两次 cmd_full_cycle 发布之间，强制最小间隔 MIN_INTERVAL_S (默认 10s)
  - 共 5 个挖掘点，每个点都和同一个卸料点组成一次 FullCycle A

消息格式（和 v15 main.py 保持一致）：
  /excavator/cmd_full_cycle  :  std_msgs/msg/Float64MultiArray
      data = [dig_x, dig_y, dig_z, dump_x, dump_y, dump_z]   (len=6)
  /excavator/state            :  std_msgs/msg/String
      data in {"idle", "working"}
  /perception/...  (不使用，本脚本直接走 action cmd_full_cycle 闭环)

用法：
  先起主控（另一个终端）：
    export ROS_DOMAIN_ID=0
    /usr/bin/python3 -u src/shandong/v15_action_task/shandong_0.5/main.py

  再跑本脚本：
    export ROS_DOMAIN_ID=0
    /usr/bin/python3 -u src/shandong/v15_action_task/shandong_0.5/scripts/run_5_cycles_full.py

  如果你想手动换 dump 点、换间隔、换 ROS_DOMAIN_ID：
    修改本脚本顶部的 DUMP_XYZ / MIN_INTERVAL_S / ROS_DOMAIN_ID 常量即可。
"""
from __future__ import annotations

import os
import sys
import time
from typing import List, Optional, Tuple

# ------------------------------------------------------------
# 必须保证：脚本本身启动时就把 ROS_DOMAIN_ID 设对，避免忘了 export 导致不互通
# ------------------------------------------------------------
ROS_DOMAIN_ID: int = 0
os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)

# ------------------------------------------------------------
# 用户可调参数
# ------------------------------------------------------------
DIG_POINTS_XYZ: List[Tuple[float, float, float]] = [
    (1.51,  0.43, 0),   # 第 1 斗
    (1.37, -0.03, 0),   # 第 2 斗
    (1.14,  0.51, 0),   # 第 3 斗
    (1.34,  -0.24, 0),   # 第 4 斗
    (1.48,  -0.30, 0),   # 第 5 斗
]

DUMP_XYZ: Tuple[float, float, float] = (-0.05, 1.10, 0.57)   # 统一卸料点
MIN_INTERVAL_S: float = 10.0                                  # 两次 cmd_full_cycle 发布最小间隔
STATE_TOPIC_WAIT_TIMEOUT_S: float = 30.0                      # 等待 state 由 working -> idle 的超时

# 订阅 / 发布 topic（必须和 main.py 完全一致）
TOPIC_STATE         = "/excavator/state"
TOPIC_CMD_FULL_CYC  = "/excavator/cmd_full_cycle"
FRAME_ID            = "base_link"  # 仅用于日志打印，cmd_full_cycle 数组不含 frame


def _ensure_ros_workspace_in_path() -> None:
    """把 <ws>/src/shandong 注入 sys.path，保证 import rclpy 失败时仍能给出清晰报错。"""
    _SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    _SHANDONG_0_5 = os.path.dirname(_SCRIPT_DIR)
    _V15_ACTION  = os.path.dirname(_SHANDONG_0_5)
    _SRC_SHANDONG = os.path.dirname(_V15_ACTION)
    for _p in (_SRC_SHANDONG, _V15_ACTION, _SHANDONG_0_5):
        if _p not in sys.path:
            sys.path.insert(0, _p)


_ensure_ros_workspace_in_path()


def main() -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String
    except Exception as e:
        print(
            "[run_5_cycles] 导入 ROS2 依赖失败："
            f"{type(e).__name__}: {e}\n"
            "   → 请确认：用 /usr/bin/python3 (系统 3.10) 运行本脚本，且当前 shell 已 source /opt/ros/humble/setup.bash\n"
            f"   → 当前 ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')!r}"
        )
        return 2

    # ------------------------------------------------------------------
    # 简易节点：state 订阅 + cmd_full_cycle 发布
    # ------------------------------------------------------------------
    class FiveCycleTester(Node):
        def __init__(self) -> None:
            super().__init__("v15_5cycles_tester")
            self._last_state: Optional[str] = None
            self._last_state_ts: float = 0.0

            self.sub_state = self.create_subscription(
                String, TOPIC_STATE, self._on_state, 10
            )
            self.pub_cycle = self.create_publisher(
                Float64MultiArray, TOPIC_CMD_FULL_CYC, 10
            )

            self.get_logger().info(
                f"[Init] ROS_DOMAIN_ID = {os.environ.get('ROS_DOMAIN_ID')}\n"
                f"       state topic   = {TOPIC_STATE}\n"
                f"       cmd topic     = {TOPIC_CMD_FULL_CYC}\n"
                f"       挖掘轮数      = {len(DIG_POINTS_XYZ)} 轮\n"
                f"       卸料点 (x,y,z)= {DUMP_XYZ}\n"
                f"       最小发布间隔  = {MIN_INTERVAL_S:.1f}s\n"
            )

        # ------------------------------------------------------------
        # state 回调
        # ------------------------------------------------------------
        def _on_state(self, msg: String) -> None:
            s = (msg.data or "").strip().lower()
            if s:
                self._last_state = s
                self._last_state_ts = time.monotonic()

        def wait_for_idle(self, timeout_s: float = STATE_TOPIC_WAIT_TIMEOUT_S) -> bool:
            """等待 /excavator/state = idle。超时返回 False。"""
            t0 = time.monotonic()
            last_print = 0.0
            while rclpy.ok():
                # 先 spin 一次，确保 state 回调有机会跑
                rclpy.spin_once(self, timeout_sec=0.1)

                now = time.monotonic()
                age = now - self._last_state_ts if self._last_state_ts > 0 else float("inf")

                if self._last_state == "idle" and age <= 5.0:
                    # state 比较新（5s 内），可以认定主控空闲
                    return True
                if self._last_state == "working" and age <= 5.0:
                    # 仍在工作中
                    if now - last_print >= 2.0:
                        last_print = now
                        waited = now - t0
                        self.get_logger().info(
                            f"[WaitIdle] 主控仍处于 working 状态（已等 {waited:.1f}s，最多等 {timeout_s:.1f}s）..."
                        )
                else:
                    # 还没收到任何 state，或者 state 很旧
                    if now - last_print >= 2.0:
                        last_print = now
                        waited = now - t0
                        self.get_logger().warning(
                            f"[WaitIdle] 尚未收到 {TOPIC_STATE}。是否 main.py 已启动？（已等 {waited:.1f}s）"
                        )

                if (now - t0) >= timeout_s:
                    self.get_logger().error(
                        f"[WaitIdle] 等待 {TOPIC_STATE}=idle 超时（>{timeout_s:.1f}s），终止本次后续轮次。"
                    )
                    return False

            # rclpy.shutdown
            return False

        # ------------------------------------------------------------
        # 发布一个 full cycle
        # ------------------------------------------------------------
        def publish_cycle(self, dig_xyz: Tuple[float, float, float]) -> None:
            dx, dy, dz = float(dig_xyz[0]), float(dig_xyz[1]), float(dig_xyz[2])
            ux, uy, uz = float(DUMP_XYZ[0]), float(DUMP_XYZ[1]), float(DUMP_XYZ[2])

            flat = [dx, dy, dz, ux, uy, uz]
            msg = Float64MultiArray()
            msg.layout.dim = [MultiArrayDimension(label="cycle6", size=6, stride=6)]
            msg.data = [float(v) for v in flat]

            self.pub_cycle.publish(msg)
            self.get_logger().info(
                f"[Publish] {TOPIC_CMD_FULL_CYC} 已发布:\n"
                f"          dig  = ({dx:.3f}, {dy:.3f}, {dz:.3f})  m\n"
                f"          dump = ({ux:.3f}, {uy:.3f}, {uz:.3f})  m\n"
                f"          data = {flat}"
            )

    # ------------------------------------------------------------------
    # 主流程：5 轮 full cycle
    # ------------------------------------------------------------------
    rclpy.init(args=None)
    tester = FiveCycleTester()
    try:
        last_pub_ts: float = -1e9  # 初始化为“很久之前”，保证第一次不用等
        ok_all: bool = True

        for idx, dig_xyz in enumerate(DIG_POINTS_XYZ, start=1):
            tester.get_logger().info(
                "=" * 60 + "\n"
                f"[Round {idx}/{len(DIG_POINTS_XYZ)}] 准备发布挖掘-卸料全流程\n"
                f"           挖掘点 = {dig_xyz}\n"
                f"           卸料点 = {DUMP_XYZ}\n"
                + "=" * 60
            )

            # --- 1) 保证最小 10s 发布间隔（从上一次发布完成时开始算） ---
            now = time.monotonic()
            since_last = now - last_pub_ts
            if since_last < MIN_INTERVAL_S:
                wait = MIN_INTERVAL_S - since_last
                tester.get_logger().info(
                    f"[Sleep] 距离上一次发布仅 {since_last:.1f}s，"
                    f"按最小间隔规则再等 {wait:.1f}s（要求 ≥{MIN_INTERVAL_S:.1f}s）..."
                )
                end_sleep = time.monotonic() + wait
                while rclpy.ok() and time.monotonic() < end_sleep:
                    # 即使在 sleep 里也 spin 一下，保持 state 新鲜
                    rclpy.spin_once(tester, timeout_sec=0.1)
                    remain = end_sleep - time.monotonic()
                    if remain > 0 and int(remain) != int(remain + 0.15):
                        tester.get_logger().debug(f"  ... 剩余 {remain:.1f}s")

            # --- 2) 确认 state == idle，如果 working，就不发布，先等 ---
            tester.get_logger().info(
                f"[PreCheck] 检查 {TOPIC_STATE} 是否为 idle..."
            )
            if not tester.wait_for_idle(timeout_s=STATE_TOPIC_WAIT_TIMEOUT_S):
                ok_all = False
                tester.get_logger().error(
                    f"[Round {idx}] 由于 state 长时间未进入 idle，已放弃后续所有轮次。"
                )
                break

            # --- 3) 发布 full cycle ---
            tester.publish_cycle(dig_xyz)
            last_pub_ts = time.monotonic()

            # --- 4) 发布后必须立刻等到 state 从 idle -> working 再返回 idle，
            #        才能保证下一轮开始时“这次动作已经结束”的语义。
            #        如果只等 10s，会出现“动作还在转斗、下一轮又发了”的风险。
            tester.get_logger().info(
                f"[Round {idx}] 已发布，开始等待主控状态回到 idle（动作真正结束）..."
            )
            if not tester.wait_for_idle(timeout_s=99999.0):
                ok_all = False
                tester.get_logger().error(
                    f"[Round {idx}] 等待动作结束回到 idle 失败（或 rclpy 被 shutdown），停止后续轮次。"
                )
                break

            tester.get_logger().info(
                f"[Round {idx}] ✅ 已完成，当前 state=idle。"
            )

        # 5 轮全部跑完
        tester.get_logger().info(
            "#" * 60 + "\n"
            f"[Done] 五轮挖掘-卸料指令序列已 {'全部成功跑完' if ok_all else '未完全跑完'}。\n"
            f"       总挖掘点：{len(DIG_POINTS_XYZ)}；统一卸料点：{DUMP_XYZ}\n"
            + "#" * 60
        )
        return 0 if ok_all else 1

    except KeyboardInterrupt:
        tester.get_logger().warning("[Ctrl+C] 用户中断，脚本退出。")
        return 130
    finally:
        try:
            tester.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
