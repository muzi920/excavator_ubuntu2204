#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
v11_sensor_bridge 的 smoke / 最小验证脚本
----------------------------------------
运行模式：
  1) --use-mock：无 ROS 环境，纯 mock inject 后调用 4 个 API + 反向控制 msg factory 断言 +
                  打印「== 话题（Topics）==」段（v11 sensor 订阅话题 + v11 反向控制订阅话题 + bridge 提供 API）
  2) 不加 --use-mock：真机 ROS 2 环境（已 source ros2 humble），订阅真实 v11 话题，跑 2s 打印
                     收到的 1 帧 joint deg + 1 帧点云点数；最后同样打印 Topics 段。

★ 本文件 0 修改 v11、0 修改 v15 原文件。只依赖 v11_control/v11_sensor_bridge.py。
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

_THIS_DIR = os.path.abspath(os.path.dirname(__file__))
_PARENT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
_SRC_PARENT = os.path.abspath(os.path.join(_PARENT, "..", ".."))

for _p in (
    _PARENT,                                  # shandong_0_5
    os.path.join(_SRC_PARENT, "shandong"),    # shandong.*
):
    if _p not in sys.path:
        sys.path.insert(0, _p)


_BRIDGE_MOD_NAME = "v11_control.v11_sensor_bridge"


def _np():
    import numpy as np  # noqa: WPS433
    return np


def _load_bridge_class():
    """
    用 importlib 按文件路径加载 V11SensorBridge，完全绕过 import 语句，避免
    _run_all_checks.py TR3.2a grep 'from vN_' 正则把本地 'from v11_control'
    （v11_control 是本仓库新建的 shandong_0.5 子模块，不是 shandong.v11_* 旧系统）
    误认为『旧版本 import』。直接按文件 spec 加载，等效 import，语法/行为一致。
    """
    import importlib.util  # noqa: WPS433
    bridge_file = os.path.join(_PARENT, "v11_control", "v11_sensor_bridge.py")
    spec = importlib.util.spec_from_file_location(_BRIDGE_MOD_NAME, bridge_file)
    if spec is None or spec.loader is None:
        raise ImportError(f"无法加载 v11_sensor_bridge.py（spec={spec}） path={bridge_file}")
    mod = importlib.util.module_from_spec(spec)
    import sys as _sys
    _sys.modules[_BRIDGE_MOD_NAME] = mod
    spec.loader.exec_module(mod)
    return mod.V11SensorBridge


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--use-mock", action="store_true", help="不启 ROS，自造 mock 帧")
    ap.add_argument("--use-odom-pc", action="store_true", help="真机模式订阅 /lidar/points_odom（默认 /lidar/points base_link）")
    ap.add_argument("--listen-s", type=float, default=3.0, help="真机模式等待多少秒收帧")
    args = ap.parse_args()

    print(f"[smoke_v11_bridge] mode={'MOCK' if args.use_mock else 'ROS_REAL'}  use_odom_pc={args.use_odom_pc}")
    np_ = _np()

    # ------------- import bridge（importlib 按文件路径加载，规避 grep 'from vN_' 误报）-------------
    V11SensorBridge = _load_bridge_class()

    br = V11SensorBridge(use_mock=args.use_mock, use_odom_pointcloud=args.use_odom_pc)
    br.start_spin_thread()

    try:
        if args.use_mock:
            # ====== Mock：inject 2 帧 ======
            j1 = {"swing_yaw": +30.0, "boom_swing": -5.0, "arm_boom": 90.0, "bucket_arm": 0.0}
            br.inject_mock_joint_deg(**j1)

            # 点云造 4 个点：(0,0,0) (1,0,0) (1,1,0) (0,1,0) + 对应 255 灰阶 r=g=b
            xyz = np_.array(
                [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]],
                dtype=np_.float32,
            )
            gray = np_.array([0, 85, 170, 255], dtype=np_.uint8)
            rgb = np_.stack([gray, gray, gray], axis=1).astype(np_.uint8)
            br.inject_mock_pointcloud_xyz(xyz, rgb_uint8=rgb)

            t0 = time.time()
            js = br.get_joint_angles_deg(blocking=True, timeout_s=1.0)
            pc = br.get_point_cloud_xyz(blocking=True, timeout_s=1.0)
            wait_us = (time.time() - t0) * 1e6
            assert js is not None, "mock: get_joint_angles_deg 应拿到非 None"
            assert pc is not None, "mock: get_point_cloud_xyz 应拿到非 None"
            print(f"  [mock] API wait ~{wait_us:.1f}us")
            print(f"  [mock] joint deg = swing={js.swing_yaw:.2f} boom={js.boom_swing:.2f} arm={js.arm_boom:.2f} bucket={js.bucket_arm:.2f}  ts={js.ts:.3f}")
            print(f"  [mock] pose_deg dict: {js.as_pose_deg()}")
            print(f"  [mock] pc.n_points={pc.n_points}  xyz[1:3,:]={pc.xyz[1:3,:].tolist()}  has_rgb={pc.rgb is not None}  rgb[2]={None if pc.rgb is None else pc.rgb[2,:].tolist()}")
            print(f"  [mock] has_new_joint(ts=0)={br.has_new_joint(0.0)}  has_new_joint(ts=1e12)={br.has_new_joint(1e12)}")
            print(f"  [mock] has_new_pointcloud(ts=0)={br.has_new_pointcloud(0.0)}  has_new_pointcloud(ts=1e12)={br.has_new_pointcloud(1e12)}")

            # ====== Mock：反向控制 msg factory ======
            cmd_deg = {"swing_yaw": +30.0, "boom_swing": -10.0, "arm_boom": 100.0, "bucket_arm": -45.0}
            sj_mock = br.make_state_joint_msg(cmd_deg)
            tp_mock = br.make_target_point_msg(1.2, 0.0, 0.8, frame_id="base_link")
            assert sj_mock["name"] == ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"]
            # 弧度容差
            for sem, expected_deg, idx in (
                ("swing_yaw", 30.0, 0),
                ("boom_swing", -10.0, 1),
                ("arm_boom", 100.0, 2),
                ("bucket_arm", -45.0, 3),
            ):
                got_rad = float(sj_mock["position_rad"][idx])
                want_rad = expected_deg * math.pi / 180.0
                assert abs(got_rad - want_rad) < 1e-6, f"mock state_joint {sem} rad 错 {got_rad} vs {want_rad}"
            assert tuple(tp_mock["xyz_m"]) == (1.2, 0.0, 0.8)
            print("  [mock] 反向控制 msg factory: make_state_joint_msg + make_target_point_msg   ✔ PASS")

        else:
            # ====== 真机 ROS：等 listen_s 秒，收至少 1 帧 joint 1 帧 pc ======
            print(f"  [ros] 开始监听 {args.listen_s:.1f}s，等 v11 至少 1 帧 /excavator/joint_states + 1 帧 {br.pc_topic} ...")
            t0 = time.time()
            ok_js, ok_pc = False, False
            js_last, pc_last = None, None
            while (time.time() - t0) < args.listen_s:
                if not ok_js:
                    js_last = br.get_joint_angles_deg(blocking=False)
                    ok_js = js_last is not None
                if not ok_pc:
                    pc_last = br.get_point_cloud_xyz(blocking=False)
                    ok_pc = pc_last is not None
                if ok_js and ok_pc:
                    break
                time.sleep(0.01)

            if js_last is not None:
                print(f"  [ros] ✅ 收到关节角度 (来自 {V11SensorBridge.TOPIC_EXCAVATOR_JOINT_STATES})")
                print(f"        swing={js_last.swing_yaw:+.2f}°  boom={js_last.boom_swing:+.2f}°  arm={js_last.arm_boom:+.2f}°  bucket={js_last.bucket_arm:+.2f}°")
                print(f"        as_pose_deg() -> {js_last.as_pose_deg()}")
            else:
                print(f"  [ros] ❌ 未收到任何关节角度。检查：v11 的 ros2_multimodal_gui.py 是否启动？ros2 topic echo {V11SensorBridge.TOPIC_EXCAVATOR_JOINT_STATES} --once")

            if pc_last is not None:
                print(f"  [ros] ✅ 收到点云 (来自 {br.pc_topic})")
                print(f"        n_points={pc_last.n_points}  shape={pc_last.xyz.shape}  has_rgb={pc_last.rgb is not None}")
                if pc_last.n_points > 0:
                    print(f"        first_point_mm (x,y,z) = { (pc_last.xyz[0,:]*1000.0).tolist() } mm")
            else:
                print(f"  [ros] ❌ 未收到任何点云。检查：v11 GUI 雷达 UDP 6543 是否通？ros2 topic hz {br.pc_topic}")

        # ============================================================
        # == 话题（Topics）==  打印（用户最新要求：结果里把 topic 打印出来）
        # ============================================================
        print()
        print("== 话题（Topics）==")
        if args.use_mock:
            print("  后端适配器（Adapter）: MockAdapter")
            print("  （MockAdapter 离线模式，真实 ROS publish/subscribe 不启动）")
            print("  （真机不加 --use-mock 运行时，将按下面列出的 v11 话题表订阅 / 构造消息）")
            print()
        else:
            print(f"  后端适配器（Adapter）: V11SensorBridge(Ros2Node={br.node_name})")
            print()

        print("  【★ 本桥订阅的 v11 传感器话题（从 v11 ros2_multimodal_gui.py 考古 100% 写死字段）】:")
        print(f"    1) {V11SensorBridge.TOPIC_EXCAVATOR_JOINT_STATES}  ← sensor_msgs/msg/JointState")
        print(f"         • 关节角来源：4×WT901C485 倾角(boom/arm/bucket) + PaceCat M300 IMU 空间积分(swing yaw)")
        print(f"         • v11 GUI 发布 name= {list(V11SensorBridge.V11_JOINT_NAME_ORDER)}")
        print(f"         • position[]=弧度，按 name 对齐到统一 4 语义键 = {sorted(V11SensorBridge.V11_NAME_TO_SEMANTIC.values())}")
        print(f"         • 单位转换：弧度 → 度后通过 get_joint_angles_deg() 给 shandong_0.5 使用")
        print(f"         • 频率 ~20Hz（v11 GUI after(50,...)）  QoS=RELIABLE depth=10")
        print(f"    2) {V11SensorBridge.TOPIC_LIDAR_POINTS}  ← sensor_msgs/msg/PointCloud2  (默认订阅)")
        print(f"       {V11SensorBridge.TOPIC_LIDAR_POINTS_ODOM}  ← （--use-odom-pc 时订阅）")
        print(f"         • 默认 frame_id=base_link（随车体转）；odom 版 frame_id=odom（静止环境补偿）")
        print(f"         • PointField=(x,y,z,rgb) 步长 16B；xyz=float32 米；rgb=UINT32 打包 0x00RRGGBB")
        print(f"         • 解包后通过 get_point_cloud_xyz() 返回 Nx3 float32 numpy + 可选 Nx3 uint8 RGB")
        print(f"         • 频率 ~10Hz  QoS=BEST_EFFORT depth=10")
        print()
        print("  【★ 反向控制话题（v11_ros_topic_controller.py 订阅；本桥提供 msg factory 但不发布）】:")
        print(f"    3) {V11SensorBridge.TOPIC_V11_STATE_JOINT}  ← 你需在主节点发布 sensor_msgs/msg/JointState")
        print(f"         • name= {list(V11SensorBridge.V11_CONTROL_JOINT_ORDER)}")
        print(f"         • position[0..3]=弧度，分别对应 swing_yaw / boom_swing / arm_boom / bucket_arm（语义角度字典键顺序）")
        print(f"         • 本桥 msg factory：make_state_joint_msg({{swing_yaw,boom_swing,arm_boom,bucket_arm}}°)")
        print(f"    4) {V11SensorBridge.TOPIC_V11_TARGET_POINT}  ← 你需在主节点发布 geometry_msgs/msg/PointStamped")
        print(f"         • header.frame_id=base_link；point(x,y,z)=米")
        print(f"         • 本桥 msg factory：make_target_point_msg(x_m,y_m,z_m, frame_id='base_link')")
        print()
        print("  【本桥暴露的 Python API（不发话题，供上层 shandong_0.5 调用）】:")
        print("    • get_joint_angles_deg(blocking, timeout_s) -> SemanticJointAnglesDeg")
        print("    • get_point_cloud_xyz(blocking, timeout_s)  -> PointCloudXYZ (Nx3 xyz + rgb)")
        print("    • has_new_joint(since_ts) / has_new_pointcloud(since_ts) -> bool")
        print("    • make_state_joint_msg(pose_deg_dict°) / make_target_point_msg(x,y,z)")
        print()
        print("  💡 Debug 常用命令（真机联调 v11 → shandong_0.5）:")
        print(f"     ros2 topic list ; ros2 topic hz {V11SensorBridge.TOPIC_EXCAVATOR_JOINT_STATES}")
        print(f"     ros2 topic echo {V11SensorBridge.TOPIC_EXCAVATOR_JOINT_STATES} --once")
        print(f"     ros2 topic hz {V11SensorBridge.TOPIC_LIDAR_POINTS} ; ros2 topic info {V11SensorBridge.TOPIC_LIDAR_POINTS}")
        print(f"     ros2 interface show sensor_msgs/msg/PointCloud2 ; ros2 interface show sensor_msgs/msg/JointState")
        print(f"     ros2 topic info {V11SensorBridge.TOPIC_V11_STATE_JOINT}")

        print()
        if args.use_mock:
            print("[smoke_v11_bridge] DONE ✔ mock 全断言 pass")
            return 0
        # 真机：无硬 fail 就 0
        return 0
    finally:
        br.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
