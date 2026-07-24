"""
point_to_dig_dump_trajectory.py —— 单点挖掘-卸料轨迹生成器

本脚本接收一个三维挖掘点 (x, y, z) 和一个三维卸料点 (x, y, z)，
通过机械臂逆运动学求解，自动生成一份包含 14 个步骤的 JSON 轨迹文件。
该 JSON 文件可直接被 replay_json_script.py 或 terminal_stepper.py 回放执行。

核心流程：
  1. 将三维点投影到柱坐标 (r, yaw, z)
  2. 对挖掘点搜索最优的铲斗姿态解（半开斗下探）
  3. 对卸料点搜索最优的空中卸料姿态解（抬高到安全高度）
  4. 拼装 14 步关节轨迹：初始化 → 挖掘 → 运输 → 卸料

依赖：v10_cailbration_arm/inverse_kinematics.py 中的 ExcavatorIK 逆解器
"""

import argparse
import json
import math
import os
import sys


# 当前脚本所在目录（v14_urdf/）
CURRENT_DIR = os.path.dirname(__file__)
# 项目根目录（shandong/）
ROOT_DIR = os.path.abspath(os.path.join(CURRENT_DIR, ".."))
# v10 几何建模目录，包含逆运动学求解器
V10_DIR = os.path.join(ROOT_DIR, "v10_cailbration_arm")
if V10_DIR not in sys.path:
    sys.path.append(V10_DIR)

# 导入 v10 的逆运动学求解器
from inverse_kinematics import ExcavatorIK


class DigDumpPlanner:
    """
    挖掘-卸料轨迹规划器。

    作用：
      将用户给出的三维挖掘点和卸料点坐标，转换为可被
      replay_json_script.py / terminal_stepper.py 回放的关节轨迹 JSON。

    核心思路：
      1. 三维点 (x,y,z) → 柱坐标 (r, yaw, z)
      2. 在柱坐标平面上求逆解，得到 (boom_swing, arm_boom, bucket_arm)
      3. 回转角 yaw 直接映射到 swing_yaw
      4. 按挖掘流程拼装 14 步：初始化 → 挖掘 → 运输 → 卸料
    """

    def __init__(self):
        """
        初始化规划器。

        变量说明：
          self.ik            : ExcavatorIK 实例，负责二维平面逆运动学求解
          self.bucket_length : 铲斗长度（米），用于计算卸料时的腕点偏移
          self.limits        : 各关节的限位范围（度），与 v14 URDF 限位一致
        """
        self.ik = ExcavatorIK()
        self.bucket_length = 0.26  # 铲斗长度，单位：米
        self.limits = {
            "boom_swing": (-5.0, 55.0),    # 大臂俯仰角范围
            "arm_boom": (-5.0, 95.0),      # 小臂俯仰角范围
            "bucket_arm": (-95.0, 20.0),   # 铲斗开合角范围（0=闭合，-95=完全打开）
            "swing_yaw": (-180.0, 180.0),  # 回转偏航角范围
        }

    @staticmethod
    def _radius_and_yaw(point):
        """
        将三维笛卡尔坐标转换为柱坐标。

        参数：
          point : (x, y, z) 三维点坐标

        返回：
          (radius, yaw_deg, z)
            radius   : 点到回转中心的水平距离（米）
            yaw_deg  : 回转角度（度），atan2(y, x)
            z        : 高度值，原样返回

        作用：
          挖掘机逆解是在二维平面 (r, z) 上进行的，
          所以需要先把三维点投影到柱坐标，yaw 直接作为回转目标角。
        """
        x, y, z = point
        radius = math.sqrt(x * x + y * y)
        yaw = math.degrees(math.atan2(y, x))
        return radius, yaw, z

    def _within_limits(self, result):
        """
        检查逆解结果是否在关节限位范围内。

        参数：
          result : ExcavatorIK.calculate_ik() 返回的关节角度字典
                   包含 boom_swing, arm_boom, bucket_arm 等字段

        返回：
          True  : 所有关节（除回转外）都在限位内
          False : 有任一关节超出限位

        作用：
          逆解器可能返回数学上正确但物理上不可达的解，
          本函数用于过滤掉这些不可行的解。回转角单独处理，不在此检查。
        """
        for joint, (lower, upper) in self.limits.items():
            if joint == "swing_yaw":
                continue  # 回转角由 yaw 直接决定，不参与限位检查
            value = float(result[joint])
            if value < lower or value > upper:
                return False
        return True

    def _clamp(self, joint_name, value):
        """
        将角度值钳位到指定关节的限位范围内。

        参数：
          joint_name : 关节名，如 "boom_swing", "arm_boom", "bucket_arm"
          value      : 目标角度值（度）

        返回：
          钳位后的角度值（度），保证在 [lower, upper] 区间内

        作用：
          在规划过程中，某些中间步骤的角度可能略微超出限位，
          本函数将其强制收回到安全范围内。
        """
        lower, upper = self.limits[joint_name]
        return max(lower, min(upper, float(value)))

    def _search_pose(self, radius, z, angle_candidates, prefer_angle_deg=None, prefer_arm_deg=None):
        """
        在给定目标点 (r, z) 处，搜索所有可行的铲斗姿态解，并按评分排序。

        参数：
          radius            : 目标点到回转中心的水平距离（米）
          z                 : 目标点高度（米）
          angle_candidates  : 铲斗绝对姿态角的候选列表（度），例如 range(-70, -19, 5)
          prefer_angle_deg  : 优选的铲斗姿态角（度），评分越接近该值得分越低
          prefer_arm_deg    : 优选的小臂角度（度），评分越接近该值得分越低

        返回：
          solutions : [(score, bucket_abs_angle_deg, result), ...] 按 score 升序排列
            score                : 评分，越小越好
            bucket_abs_angle_deg : 铲斗绝对姿态角（度）
            result               : ExcavatorIK 返回的关节角度字典

        作用：
          遍历所有候选铲斗姿态角，对每个角调用逆解器求解。
          用评分函数综合考虑铲斗姿态偏好、小臂偏好和大臂居中程度，
          返回所有可行解中评分最优的那个。
        """
        solutions = []
        for bucket_abs_angle_deg in angle_candidates:
            # 调用 v10 的逆解器，求解二维平面 (r, z) 在给定铲斗姿态下的关节角
            result = self.ik.calculate_ik(radius, z, bucket_abs_angle_deg)
            if result is None:
                continue  # 逆解失败，跳过
            if not self._within_limits(result):
                continue  # 关节超出限位，跳过

            # 综合评分：越小越好
            score = 0.0
            if prefer_angle_deg is not None:
                # 惩罚偏离优选铲斗姿态的程度
                score += abs(bucket_abs_angle_deg - prefer_angle_deg) / 20.0
            if prefer_arm_deg is not None:
                # 惩罚偏离优选小臂角度的程度
                score += abs(float(result["arm_boom"]) - prefer_arm_deg) / 80.0
            # 偏好大臂在中间位置（35°），避免极端姿态
            score += abs(float(result["boom_swing"]) - 35.0) / 120.0
            solutions.append((score, bucket_abs_angle_deg, result))

        solutions.sort(key=lambda item: item[0])
        return solutions

    def solve_dig_pose(self, dig_point):
        """
        求解挖掘点的最优挖掘姿态。

        参数：
          dig_point : (x, y, z) 挖掘点的三维坐标

        返回：
          dict，包含：
            radius                : 挖掘点到回转中心的水平距离（米）
            yaw                   : 回转目标角度（度）
            z                     : 挖掘点高度（米）
            bucket_abs_angle_deg  : 最优铲斗绝对姿态角（度）
            result                : 逆解结果字典 {boom_swing, arm_boom, bucket_arm, ...}

        作用：
          以半开斗姿态（-70° 到 -20°）搜索挖掘点的逆解。
          偏好铲斗姿态角 -45°、小臂 70°，即"半开斗、小臂适度伸出"的姿态。
          这样铲斗可以先以半开姿态下探到挖掘点，再执行收斗取料。
        """
        radius, yaw, z = self._radius_and_yaw(dig_point)
        candidates = self._search_pose(
            radius=radius,
            z=z,
            angle_candidates=range(-70, -19, 5),  # 铲斗姿态候选：-70° 到 -20°，步长 5°
            prefer_angle_deg=-45,   # 偏好半开斗姿态
            prefer_arm_deg=70,      # 偏好小臂适度伸出
        )
        if not candidates:
            raise ValueError("挖掘点在当前限位和姿态搜索范围内无可行解。")
        # 取评分最优的解
        _, bucket_abs, result = candidates[0]
        return {
            "radius": radius,
            "yaw": yaw,
            "z": z,
            "bucket_abs_angle_deg": bucket_abs,
            "result": result,
        }

    def solve_dump_pose(self, dump_point):
        """
        求解卸料点的最优空中卸料姿态。

        参数：
          dump_point : (x, y, z) 卸料点的三维坐标

        返回：
          dict，包含：
            radius                : 卸料点到回转中心的水平距离（米）
            yaw                   : 回转目标角度（度）
            z                     : 实际使用的安全卸料高度（米），会高于原始 z
            z_original            : 用户给定的原始卸料点 z 值
            bucket_abs_angle_deg  : 最优铲斗绝对姿态角（度）
            result                : 逆解结果字典
            wrist_target_x        : 腕点目标 x 坐标（米）
            wrist_target_z        : 腕点目标 z 坐标（米）

        作用：
          卸料时铲斗需要抬到安全高度（0.35~0.55m），避免触碰卸料车边缘。
          同时考虑铲斗长度偏移，计算腕点（铲斗与小臂连接点）的实际目标位置。
          在多个安全高度和铲斗角度的组合中，按评分选出最优解。
        """
        radius, yaw, z = self._radius_and_yaw(dump_point)
        # 安全卸料高度候选列表（米）
        z_candidates = [0.35, 0.40, 0.45, 0.50, 0.55]
        merged = []
        for safe_z in z_candidates:
            # 铲斗绝对姿态角候选：25° 到 45°，步长 5°
            for bucket_abs in range(25, 46, 5):
                # 计算腕点（铲斗连接点）的目标位置
                # 腕点 = 铲尖位置 - 铲斗长度方向偏移
                target_x = radius + self.bucket_length * math.cos(math.radians(bucket_abs))
                target_z = safe_z + self.bucket_length * math.sin(math.radians(bucket_abs))
                # 对腕点位置求逆解
                result = self.ik.calculate_ik(target_x, target_z, bucket_abs)
                if result is None:
                    continue
                if not self._within_limits(result):
                    continue

                # 综合评分：越小越好
                score = 0.0
                score += abs(bucket_abs - 35.0) / 20.0       # 偏好铲斗姿态 35°
                score += abs(float(result["bucket_arm"]) - (-8.0)) / 20.0  # 偏好铲斗微开
                score += abs(float(result["arm_boom"]) - 70.0) / 60.0      # 偏好小臂 70°
                score += abs(safe_z - 0.45) / 10.0            # 偏好安全高度 0.45m
                merged.append((score, safe_z, bucket_abs, result, target_x, target_z))

        merged.sort(key=lambda item: item[0])
        if not merged:
            raise ValueError("卸料点在当前限位和空中卸料约束下无可行解。")

        # 取评分最优的解
        _, safe_z, bucket_abs, result, target_x, target_z = merged[0]
        return {
            "radius": radius,
            "yaw": yaw,
            "z": safe_z,               # 实际使用的安全卸料高度
            "z_original": z,            # 用户给定的原始 z
            "bucket_abs_angle_deg": bucket_abs,
            "result": result,
            "wrist_target_x": target_x, # 腕点目标 x
            "wrist_target_z": target_z, # 腕点目标 z
        }

    @staticmethod
    def _step(step_id, joint, description, target_val, ch3_mv=3500, ramp_up_s=0.2, ramp_down_s=0.2, is_init_step=False):
        """
        构建单个步骤的 JSON 字典。

        参数：
          step_id      : 步骤编号（从 1 开始）
          joint        : 目标关节名，如 "swing_yaw", "boom_swing", "arm_boom", "bucket_arm"
          description  : 步骤的文字描述，用于回放时显示
          target_val   : 目标角度值（度），会被四舍五入到两位小数
          ch3_mv       : 液压通道 3 的模拟量（mV），仿真模式下不使用，默认 3500
          ramp_up_s    : 加速时间（秒），仿真模式下不使用，默认 0.2
          ramp_down_s  : 减速时间（秒），仿真模式下不使用，默认 0.2
          is_init_step : 是否为初始化步骤，初始化步骤在回放时有特殊处理

        返回：
          dict，包含 step, joint, description, ch1_mv, ch2_mv, ch3_mv,
          ramp_up_s, ramp_down_s, target_val 等字段

        作用：
          这是 JSON 剧本中每个步骤的标准格式，
          与 v4_control_closed 的剧本格式完全兼容。
        """
        item = {
            "step": step_id,
            "joint": joint,
            "description": description,
            "ch1_mv": 0,
            "ch2_mv": 0,
            "ch3_mv": ch3_mv,
            "ramp_up_s": ramp_up_s,
            "ramp_down_s": ramp_down_s,
            "target_val": round(float(target_val), 2),
        }
        if is_init_step:
            item["is_init_step"] = True
        return item

    def plan(self, dig_point, dump_point):
        """
        完整规划一次"挖掘→卸料"的 14 步关节轨迹。

        参数：
          dig_point  : (x, y, z) 挖掘点三维坐标
          dump_point : (x, y, z) 卸料点三维坐标

        返回：
          dict，包含：
            metadata : 元数据，包括挖掘点/卸料点信息、逆解结果、关键角度目标
            script   : 步骤数组，共 14 步，可直接被回放器执行

        作用：
          这是整个脚本的核心函数。它调用 solve_dig_pose 和 solve_dump_pose
          分别求解挖掘姿态和卸料姿态，然后按以下 14 步流程拼装轨迹：
            步骤  1-4  : 初始化（回转对准 + 挖掘预备姿态）
            步骤  5-9  : 挖掘（收斗取料 → 抬料 → 运输姿态）
            步骤 10    : 回转到卸料点
            步骤 11-14 : 卸料（大臂/小臂到位 → 半开斗 → 全开斗卸料）
        """
        # 求解挖掘点和卸料点的最优姿态
        dig = self.solve_dig_pose(dig_point)
        dump = self.solve_dump_pose(dump_point)

        dig_res = dig["result"]    # 挖掘逆解结果
        dump_res = dump["result"]  # 卸料逆解结果

        # 计算各关键中间步骤的关节角度目标值
        # 挖掘阶段：先半开斗(-55°)下探，再收斗(0°)取料
        dig_entry_bucket = self._clamp("bucket_arm", -55.0)       # 挖掘入口：半开斗
        scoop_bucket = self._clamp("bucket_arm", 0.0)             # 收斗取料：完全闭合
        scoop_arm = self._clamp("arm_boom", max(dig_res["arm_boom"], 72.0))  # 回拉小臂
        lift_boom = self._clamp("boom_swing", dig_res["boom_swing"] - 12.0)  # 抬大臂
        # 运输阶段：小臂收回、铲斗微开稳料
        carry_arm = self._clamp("arm_boom", max(scoop_arm, 58.0))  # 运输小臂
        carry_bucket = self._clamp("bucket_arm", -5.0)             # 运输铲斗（微开稳料）
        # 卸料阶段：半开斗(-18°)预备 → 全开斗(-90°)卸料
        dump_prepare_bucket = self._clamp("bucket_arm", -18.0)     # 卸料预备
        unload_bucket = self._clamp("bucket_arm", -90.0)           # 完全打开卸料

        # 按顺序拼装 14 步轨迹
        steps = []
        append = steps.append
        idx = 1

        # === 阶段 1：初始化（步骤 1-4）===
        # 回转到挖掘点方向
        append(self._step(idx, "swing_yaw", "对准挖掘点（回转）", dig["yaw"], is_init_step=True)); idx += 1
        # 铲斗半开，准备下探
        append(self._step(idx, "bucket_arm", "挖掘预备-半开斗", dig_entry_bucket, is_init_step=True)); idx += 1
        # 大臂下探到挖掘姿态
        append(self._step(idx, "boom_swing", "挖掘预备-大臂下探", dig_res["boom_swing"], is_init_step=True)); idx += 1
        # 小臂下探到挖掘点
        append(self._step(idx, "arm_boom", "挖掘预备-小臂下探", dig_res["arm_boom"], is_init_step=True)); idx += 1

        # === 阶段 2：挖掘（步骤 5-9）===
        # 收斗取料：铲斗从半开(-55°)收拢到闭合(0°)
        append(self._step(idx, "bucket_arm", "收斗取料", scoop_bucket)); idx += 1
        # 回拉小臂，抬起物料
        append(self._step(idx, "arm_boom", "回拉小臂抬料", scoop_arm)); idx += 1
        # 抬大臂，离开挖掘点
        append(self._step(idx, "boom_swing", "抬大臂离开挖掘点", lift_boom)); idx += 1
        # 收小臂到运输姿态
        append(self._step(idx, "arm_boom", "运输姿态-收小臂", carry_arm)); idx += 1
        # 铲斗微开，稳定运输中的物料
        append(self._step(idx, "bucket_arm", "运输姿态-稳料", carry_bucket)); idx += 1

        # === 阶段 3：回转到卸料点（步骤 10）===
        append(self._step(idx, "swing_yaw", "回转到卸料点", dump["yaw"])); idx += 1

        # === 阶段 4：卸料（步骤 11-14）===
        # 大臂抬到卸料高度
        append(self._step(idx, "boom_swing", "卸料预备-大臂", dump_res["boom_swing"])); idx += 1
        # 小臂伸出到卸料位置
        append(self._step(idx, "arm_boom", "卸料预备-小臂", dump_res["arm_boom"])); idx += 1
        # 铲斗半开，预备卸料
        append(self._step(idx, "bucket_arm", "卸料预备-铲斗半开", dump_prepare_bucket)); idx += 1
        # 铲斗完全打开（-90°），完成卸料
        append(self._step(idx, "bucket_arm", "打开铲斗卸料", unload_bucket)); idx += 1

        # 构建元数据，记录完整的规划信息
        metadata = {
            "dig_point": {
                "x": dig_point[0],
                "y": dig_point[1],
                "z": dig_point[2],
                "radius": dig["radius"],
                "swing_yaw": dig["yaw"],
                "bucket_abs_angle_deg": dig["bucket_abs_angle_deg"],
            },
            "dump_point": {
                "x": dump_point[0],
                "y": dump_point[1],
                "z": dump_point[2],
                "radius": dump["radius"],
                "swing_yaw": dump["yaw"],
                "safe_dump_z": dump["z"],
                "bucket_abs_angle_deg": dump["bucket_abs_angle_deg"],
                "wrist_target_x": dump["wrist_target_x"],
                "wrist_target_z": dump["wrist_target_z"],
            },
            "ik_result": {
                "dig": dig_res,
                "dump": dump_res,
            },
            "planned_targets": {
                "dig_entry_bucket": dig_entry_bucket,
                "scoop_bucket": scoop_bucket,
                "dump_prepare_bucket": dump_prepare_bucket,
                "unload_bucket": unload_bucket,
            },
        }

        return {"metadata": metadata, "script": steps}


def main():
    """
    命令行入口。

    参数：
      --dig-x  : 挖掘点 x 坐标（米），必填
      --dig-y  : 挖掘点 y 坐标（米），必填
      --dig-z  : 挖掘点 z 坐标（米），必填
      --dump-x : 卸料点 x 坐标（米），必填
      --dump-y : 卸料点 y 坐标（米），必填
      --dump-z : 卸料点 z 坐标（米），必填
      --output : 输出 JSON 文件路径，默认为 json/generated_dig_dump_trajectory.json

    作用：
      解析命令行参数，调用 DigDumpPlanner.plan() 生成轨迹，
      将结果写入 JSON 文件，并打印关键规划信息。
    """
    parser = argparse.ArgumentParser(description="根据挖掘点和卸料点生成关节轨迹 JSON。")
    parser.add_argument("--dig-x", type=float, required=True, help="挖掘点 x 坐标（米）")
    parser.add_argument("--dig-y", type=float, required=True, help="挖掘点 y 坐标（米）")
    parser.add_argument("--dig-z", type=float, required=True, help="挖掘点 z 坐标（米）")
    parser.add_argument("--dump-x", type=float, required=True, help="卸料点 x 坐标（米）")
    parser.add_argument("--dump-y", type=float, required=True, help="卸料点 y 坐标（米）")
    parser.add_argument("--dump-z", type=float, required=True, help="卸料点 z 坐标（米）")
    parser.add_argument(
        "--output", type=str,
        default=os.path.join(CURRENT_DIR, "json", "generated_dig_dump_trajectory.json"),
        help="输出 JSON 文件路径",
    )
    args = parser.parse_args()

    # 创建规划器并生成轨迹
    planner = DigDumpPlanner()
    result = planner.plan(
        dig_point=(args.dig_x, args.dig_y, args.dig_z),
        dump_point=(args.dump_x, args.dump_y, args.dump_z),
    )

    # 写入 JSON 文件
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)

    # 打印规划结果摘要
    print("[轨迹生成] 输出文件:", args.output)
    print("[轨迹生成] 挖掘点 yaw:", round(result["metadata"]["dig_point"]["swing_yaw"], 2))
    print("[轨迹生成] 卸料点 yaw:", round(result["metadata"]["dump_point"]["swing_yaw"], 2))
    print("[轨迹生成] 空中卸料高度 z:", round(result["metadata"]["dump_point"]["safe_dump_z"], 3))
    print("[轨迹生成] 共生成步骤:", len(result["script"]))


if __name__ == "__main__":
    main()
