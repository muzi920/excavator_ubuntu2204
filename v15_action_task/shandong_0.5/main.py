#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import re
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
grand_parent = os.path.dirname(parent_dir)
if grand_parent not in sys.path:
    sys.path.append(grand_parent)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import threading
import time

from v11_control.v11_sensor_bridge import V11SensorBridge
from control_lib.angle_control import JointAngleController
from control_lib.point_control import SingleJointPointMover
from control_lib.dig_control import DigPhase1Controller

class V11ControllerAdapter:
    def __init__(self, bridge: V11SensorBridge, node: Node):
        self.bridge = bridge
        self.node = node
        self.pub_cmd = self.node.create_publisher(JointState, 'cmd/joint_states', 10)

    def get_pose_or_default(self):
        js = self.bridge.get_joint_angles_deg(blocking=False)
        if js is not None:
            return js.as_pose_deg()
        # Fallback if no data
        return {
            "swing_yaw": 0.0,
            "boom_swing": 0.0,
            "arm_boom": 90.0,
            "bucket_arm": 90.0,
        }

    def set_pose(self, pose_deg):
        # 内部使用 bridge.make_state_joint_msg() 转换语义字典为 ROS 消息
        msg = self.bridge.make_state_joint_msg(pose_deg)
        self.pub_cmd.publish(msg)

    def set_single_joint(self, joint_name, target_deg):
        """严格单一动作：只下发需要运动的单个关节，避免复合动作"""
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        v11_name = None
        for k, v in self.bridge.V11_NAME_TO_SEMANTIC.items():
            if v == joint_name:
                v11_name = k
                break
                
        if v11_name:
            msg.name = [v11_name]
            # 转换为 ROS 的弧度
            msg.position = [target_deg * 3.141592653589793 / 180.0]
            self.pub_cmd.publish(msg)

class V15MainNode(Node):
    def __init__(self):
        super().__init__('shandong_0_5_main_node')
        self.get_logger().info("Initializing V15 Main Node for Real Excavator...")
        
        # 1. 启动传感器桥接 (订阅 v11 GUI 的处理后数据)
        self.bridge = V11SensorBridge()
        # 移除 self.bridge.start_spin_thread()，统一交由 MultiThreadedExecutor 管理，防止 wait set 冲突异常
        
        # 2. 初始化底层控制器适配器
        self.adapter = V11ControllerAdapter(self.bridge, self)
        
        # 3. 初始化控制库
        self.angle_ctrl = JointAngleController(self.adapter, default_tolerance_deg=2.0)
        self.point_mover = SingleJointPointMover.from_config_default()
        self.point_mover.bind_controller(self.angle_ctrl)
        self.dig_ctrl = DigPhase1Controller(self.angle_ctrl)
        
        # 4. 对外开放高层控制接口话题
        self.sub_target_point = self.create_subscription(PointStamped, '/excavator/target_point', self.target_point_cb, 10)
        self.sub_cmd_dig     = self.create_subscription(PointStamped, '/excavator/cmd_dig_point',    self.cmd_dig_cb,   10)   # 阶段 1：挖掘点
        self.sub_cmd_transit = self.create_subscription(PointStamped, '/excavator/cmd_transit_yaw',  self.cmd_transit_cb, 10)  # 阶段 2：回转角
        self.sub_cmd_dump    = self.create_subscription(PointStamped, '/excavator/cmd_dump_point',   self.cmd_dump_cb,  10)   # 阶段 3：卸料点
        self.sub_target_pose = self.create_subscription(JointState,  '/excavator/target_pose',       self.target_pose_cb, 10)  # 底层位姿直通

        # 4.1 高级组合话题（一次性完成多阶段全流程）
        #   类型 A：/excavator/cmd_full_cycle    Float64MultiArray size=6 → [dig_x, dig_y, dig_z, dump_x, dump_y, dump_z]
        #   类型 B：/excavator/cmd_dig_transit   Float64MultiArray size=4 → [dig_x, dig_y, dig_z, transit_yaw_deg]
        self.sub_cmd_full_cycle  = self.create_subscription(
            Float64MultiArray, '/excavator/cmd_full_cycle',  self.cmd_full_cycle_cb,  10)
        self.sub_cmd_dig_transit = self.create_subscription(
            Float64MultiArray, '/excavator/cmd_dig_transit', self.cmd_dig_transit_cb, 10)
        
        self.cmd_thread = None
        self.get_logger().info("V15 Main Node Initialized. Waiting for high-level commands...")

    def cmd_dig_cb(self, msg: PointStamped):
        xyz = (msg.point.x, msg.point.y, msg.point.z)
        self.get_logger().info(f"Received dig command point: {xyz}")
        
        if self.cmd_thread and self.cmd_thread.is_alive():
            self.get_logger().warning("A command is already executing. Ignoring new dig command.")
            return
            
        def _exec():
            # 预检可达性
            report = self.point_mover.ws.check_point_reachable(xyz, mode="dig")
            if not report.success:
                self.get_logger().error(f"目标点 {xyz} 不在可达工作空间内，拒绝执行挖掘！原因: {report.reasons}")
                return
            else:
                xyz_use = xyz
                
            success = self.dig_ctrl.plan_and_execute_dig(xyz_use)
            self.get_logger().info(f"Dig Phase 1 Result: success={success}")
            
        self.cmd_thread = threading.Thread(target=_exec, daemon=True)
        self.cmd_thread.start()

    def cmd_transit_cb(self, msg: PointStamped):
        """
        阶段 2：运输回转（先抬大臂，再回转）。
        消息格式：PointStamped.point.x  = 目标回转角度（度）
                  point.y / point.z = 预留（可传 0）
        """
        yaw_deg = float(msg.point.x)
        y_deg  = float(msg.point.y)
        z_deg  = float(msg.point.z)
        self.get_logger().info(
            f"[TransitCb] 收到阶段2指令：PointStamped.point = (x={yaw_deg}, y={y_deg}, z={z_deg})\n"
            f"            已提取回转目标 swing_yaw = x 字段 = {yaw_deg}°（y/z 字段忽略）\n"
            f"            即将执行顺序：1) boom_swing 抬大臂到 -25°   2) swing_yaw 回转到 {yaw_deg}°"
        )
        
        if self.cmd_thread and self.cmd_thread.is_alive():
            self.get_logger().warning("已有指令正在执行中，拒绝新指令。请等待上一条完成。")
            return
            
        def _exec():
            self.get_logger().info(f"[TransitCb] ====== 启动阶段2 线程：目标 swing_yaw = {yaw_deg}° ======")
            success = self.dig_ctrl.plan_and_execute_transit(yaw_deg)
            self.get_logger().info(f"[TransitCb] ====== 阶段2 完成：success={success} ======")
            
        self.cmd_thread = threading.Thread(target=_exec, daemon=True)
        self.cmd_thread.start()

    def cmd_dump_cb(self, msg: PointStamped):
        """
        阶段 3：卸料点控制（★ 与阶段 1 cmd_dig_point 挖掘点严格区分）
        消息格式：PointStamped.point.x,y,z = 卸料三维空间点（米，base_link 系）
        内部约束：无论传入 z 多少，boom_swing 都强制 = 0°（大臂最高点，用户标定）不动
        执行顺序：保证大臂 0° → 回转对齐 (x,y) → 小臂/铲斗到 (x,y,z) → 大角度开斗卸料
        """
        x = float(msg.point.x)
        y = float(msg.point.y)
        z = float(msg.point.z)
        dump_xyz = (x, y, z)
        self.get_logger().info(
            f"[DumpCb] ====== 收到阶段 3 卸料指令 ======\n"
            f"            卸料点 = XYZ ({x:.3f}, {y:.3f}, {z:.3f}) m  (base_link 系)\n"
            f"            ★ 强制约束：boom_swing ≡ 0°（大臂最高点），无论 z={z:.3f} 是多少都不动大臂\n"
            f"            即将执行：1) boom→0°  2) swing→atan2(y,x)  3) arm+bucket 到点  4) bucket 开斗卸料"
        )

        if self.cmd_thread and self.cmd_thread.is_alive():
            self.get_logger().warning("[DumpCb] 已有指令在执行，拒绝新卸料指令。请等待上一条完成。")
            return

        def _exec():
            self.get_logger().info("[DumpCb] ====== 启动阶段 3 卸料线程 ======")
            success = self.dig_ctrl.plan_and_execute_dump(dump_xyz)
            self.get_logger().info(f"[DumpCb] ====== 阶段 3 卸料完成：success={success} ======")

        self.cmd_thread = threading.Thread(target=_exec, daemon=True)
        self.cmd_thread.start()

    # ========================================================================
    # 高级组合：全流程 A — dig(3) + dump(3)  = size=6
    # ========================================================================
    def cmd_full_cycle_cb(self, msg: Float64MultiArray):
        """
        类型 A（全流程闭环）：挖掘 → 自动回转 → 卸料 → 回正（下一目标准备位）
        消息 Float64MultiArray.data = [dig_x, dig_y, dig_z, dump_x, dump_y, dump_z]
        - dig_xyz : 挖掘点（米，base_link 系）
        - dump_xyz: 卸料点（米，base_link 系）；其中 dump_z 用户约束：<1.0 强制=1.0，>=1.0 取原值
        """
        # 1. 解析数组（正则 + 长度校验）
        data = list(msg.data) if msg.data is not None else []
        # 兼容：data 可能是 tuple/list/float 包装；先 flatten
        flat = []
        for x in data:
            try:
                flat.append(float(x))
            except (TypeError, ValueError):
                pass
        # 如果 data 长度不对，尝试用正则从字符串（如果 msg.data 是 str）中提取 6 个数字
        if len(flat) < 6:
            try:
                raw_str = str(msg.data)
                nums = re.findall(r"-?\d+(?:\.\d+)?", raw_str)
                flat = [float(n) for n in nums]
            except Exception:
                pass
        if len(flat) < 6:
            self.get_logger().error(
                f"[FullCycle A] Float64MultiArray 长度错误！需要 6 个 float = [dig_x,dig_y,dig_z,dump_x,dump_y,dump_z]，实际收到 {len(flat)} 个: {flat}"
            )
            return
        dig_x, dig_y, dig_z = flat[0], flat[1], flat[2]
        dump_x, dump_y, dump_z_in = flat[3], flat[4], flat[5]
        dig_xyz = (float(dig_x), float(dig_y), float(dig_z))
        dump_xyz = (float(dump_x), float(dump_y), float(dump_z_in))

        # 2. 卸料点 z 的用户规则判断：<1.0 → 1.0；>=1.0 → 原值
        dz_use = max(1.0, dump_xyz[2])
        self.get_logger().info(
            f"[FullCycle A] ====== 收到全流程指令 ======\n"
            f"            挖掘点 dig  = ({dig_xyz[0]:.3f}, {dig_xyz[1]:.3f}, {dig_xyz[2]:.3f}) m\n"
            f"            卸料点 dump = ({dump_xyz[0]:.3f}, {dump_xyz[1]:.3f}, {dump_xyz[2]:.3f}) m  (原始)\n"
            f"            dump_z 规则判断: 原始 z={dump_xyz[2]:.3f}m  →  使用 z={dz_use:.3f}m  "
            f"(规则: z<1→1.0, z>=1→原值)"
        )

        if self.cmd_thread and self.cmd_thread.is_alive():
            self.get_logger().warning("[FullCycle A] 已有指令在执行，拒绝新全流程指令。请等待完成。")
            return

        def _exec():
            self.get_logger().info("[FullCycle A] ====== 启动全流程 A 线程 ======")
            success = self.dig_ctrl.execute_full_cycle_dig_dump(dig_xyz, dump_xyz)
            self.get_logger().info(f"[FullCycle A] ====== 全流程 A 完成：success={success} ======")

        self.cmd_thread = threading.Thread(target=_exec, daemon=True)
        self.cmd_thread.start()

    # ========================================================================
    # 高级组合：全流程 B — dig(3) + transit(1) = size=4
    # 完整闭环：挖掘 → 按给定角度回转 → 自动推卸料点（该方向正前 1.2m 高 1.0m）→ 卸料 → 回正
    # ========================================================================
    def cmd_dig_transit_cb(self, msg: Float64MultiArray):
        """
        类型 B（完整闭环，用户只给 dig+角度，卸料点自动推）：
          阶段 1 挖掘 dig_xyz
            → 阶段 2 按 transit_yaw_deg 回转
            → 阶段 3 在该回转方向正前 1.2m / 高 ≥1.0m 自动推卸料点卸料
            → 阶段 4 回正（下一目标点准备位）
        消息 Float64MultiArray.data = [dig_x, dig_y, dig_z, transit_yaw_deg]  (size=4)
        """
        data = list(msg.data) if msg.data is not None else []
        flat = []
        for x in data:
            try:
                flat.append(float(x))
            except (TypeError, ValueError):
                pass
        if len(flat) < 4:
            try:
                raw_str = str(msg.data)
                nums = re.findall(r"-?\d+(?:\.\d+)?", raw_str)
                flat = [float(n) for n in nums]
            except Exception:
                pass
        if len(flat) < 4:
            self.get_logger().error(
                f"[FullCycle B] Float64MultiArray 长度错误！需要 4 个 float = [dig_x,dig_y,dig_z,transit_yaw_deg]，实际收到 {len(flat)} 个: {flat}"
            )
            return
        dig_xyz = (float(flat[0]), float(flat[1]), float(flat[2]))
        transit_yaw_deg = float(flat[3])

        self.get_logger().info(
            f"[FullCycle B] ====== 收到完整闭环指令 ======\n"
            f"            挖掘点 dig          = ({dig_xyz[0]:.3f}, {dig_xyz[1]:.3f}, {dig_xyz[2]:.3f}) m\n"
            f"            回转角度 swing      = {transit_yaw_deg:.1f}°\n"
            f"            （后续会: 回转到 {transit_yaw_deg:.1f}° → 该方向正前 1.2m × 高 ≥1.0m 自动推卸料点 → 卸料 → 回正，不用分步）"
        )

        if self.cmd_thread and self.cmd_thread.is_alive():
            self.get_logger().warning("[FullCycle B] 已有指令在执行，拒绝新指令。请等待完成。")
            return

        def _exec():
            self.get_logger().info("[FullCycle B] ====== 启动完整闭环 B 线程 ======")
            success = self.dig_ctrl.execute_dig_and_transit(dig_xyz, transit_yaw_deg)
            self.get_logger().info(f"[FullCycle B] ====== 完整闭环 B 完成：success={success} ======")

        self.cmd_thread = threading.Thread(target=_exec, daemon=True)
        self.cmd_thread.start()

    def target_point_cb(self, msg: PointStamped):
        xyz = (msg.point.x, msg.point.y, msg.point.z)
        self.get_logger().info(f"Received target point: {xyz}")
        
        if self.cmd_thread and self.cmd_thread.is_alive():
            self.get_logger().warning("A command is already executing. Ignoring new point command.")
            return
            
        def _exec():
            # 预检可达性，如果不可达，直接报错拒绝，不再找最近点代替
            report = self.point_mover.ws.check_point_reachable(xyz, mode="dig")
            if not report.success:
                self.get_logger().error(f"目标点 {xyz} 不在可达工作空间内，拒绝移动！原因: {report.reasons}")
                return
                
            res = self.point_mover.move_to_point_serial(xyz, auto_use_closest_if_unreachable=False)
            self.get_logger().info(f"Point Move Result: success={res.success}, reason={res.reason}")
            
        self.cmd_thread = threading.Thread(target=_exec, daemon=True)
        self.cmd_thread.start()

    def target_pose_cb(self, msg: JointState):
        self.get_logger().info("Received target pose (Degrees).")
        
        if self.cmd_thread and self.cmd_thread.is_alive():
            self.get_logger().warning("A command is already executing. Ignoring new pose command.")
            return
            
        target_pose = {}
        # 将 JointState 消息映射回字典，用户输入现为度 (Degrees)，直接使用即可
        for n, p in zip(msg.name, msg.position):
            if n in self.bridge.V11_NAME_TO_SEMANTIC:
                sem = self.bridge.V11_NAME_TO_SEMANTIC[n]
                target_pose[sem] = float(p)
                
        def _exec():
            res = self.angle_ctrl.move_pose_serial(target_pose)
            self.get_logger().info(f"Pose Move Result: success={res.get('success')}, reason={res.get('reason')}")
            
        self.cmd_thread = threading.Thread(target=_exec, daemon=True)
        self.cmd_thread.start()

from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    node = V15MainNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    if node.bridge._node is not None:
        executor.add_node(node.bridge._node)
        
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.bridge.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
