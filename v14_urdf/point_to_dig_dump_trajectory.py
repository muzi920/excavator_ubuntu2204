import argparse
import json
import math
import os
import sys


CURRENT_DIR = os.path.dirname(__file__)
ROOT_DIR = os.path.abspath(os.path.join(CURRENT_DIR, ".."))
V10_DIR = os.path.join(ROOT_DIR, "v10_cailbration_arm")
if V10_DIR not in sys.path:
    sys.path.append(V10_DIR)

from inverse_kinematics import ExcavatorIK


class DigDumpPlanner:
    """把三维挖掘点/卸料点转换为可回放的关节轨迹。"""

    def __init__(self):
        self.ik = ExcavatorIK()
        self.bucket_length = 0.26
        self.limits = {
            "boom_swing": (-5.0, 55.0),
            "arm_boom": (-5.0, 95.0),
            "bucket_arm": (-95.0, 20.0),
            "swing_yaw": (-180.0, 180.0),
        }

    @staticmethod
    def _radius_and_yaw(point):
        x, y, z = point
        radius = math.sqrt(x * x + y * y)
        yaw = math.degrees(math.atan2(y, x))
        return radius, yaw, z

    def _within_limits(self, result):
        for joint, (lower, upper) in self.limits.items():
            if joint == "swing_yaw":
                continue
            value = float(result[joint])
            if value < lower or value > upper:
                return False
        return True

    def _clamp(self, joint_name, value):
        lower, upper = self.limits[joint_name]
        return max(lower, min(upper, float(value)))

    def _search_pose(self, radius, z, angle_candidates, prefer_angle_deg=None, prefer_arm_deg=None):
        solutions = []
        for bucket_abs_angle_deg in angle_candidates:
            result = self.ik.calculate_ik(radius, z, bucket_abs_angle_deg)
            if result is None:
                continue
            if not self._within_limits(result):
                continue

            score = 0.0
            if prefer_angle_deg is not None:
                score += abs(bucket_abs_angle_deg - prefer_angle_deg) / 20.0
            if prefer_arm_deg is not None:
                score += abs(float(result["arm_boom"]) - prefer_arm_deg) / 80.0
            score += abs(float(result["boom_swing"]) - 35.0) / 120.0
            solutions.append((score, bucket_abs_angle_deg, result))

        solutions.sort(key=lambda item: item[0])
        return solutions

    def solve_dig_pose(self, dig_point):
        radius, yaw, z = self._radius_and_yaw(dig_point)
        candidates = self._search_pose(
            radius=radius,
            z=z,
            angle_candidates=range(-70, -19, 5),
            prefer_angle_deg=-45,
            prefer_arm_deg=70,
        )
        if not candidates:
            raise ValueError("挖掘点在当前限位和姿态搜索范围内无可行解。")
        _, bucket_abs, result = candidates[0]
        return {
            "radius": radius,
            "yaw": yaw,
            "z": z,
            "bucket_abs_angle_deg": bucket_abs,
            "result": result,
        }

    def solve_dump_pose(self, dump_point):
        radius, yaw, z = self._radius_and_yaw(dump_point)
        z_candidates = [0.35, 0.40, 0.45, 0.50, 0.55]
        merged = []
        for safe_z in z_candidates:
            for bucket_abs in range(25, 46, 5):
                target_x = radius + self.bucket_length * math.cos(math.radians(bucket_abs))
                target_z = safe_z + self.bucket_length * math.sin(math.radians(bucket_abs))
                result = self.ik.calculate_ik(target_x, target_z, bucket_abs)
                if result is None:
                    continue
                if not self._within_limits(result):
                    continue

                score = 0.0
                score += abs(bucket_abs - 35.0) / 20.0
                score += abs(float(result["bucket_arm"]) - (-8.0)) / 20.0
                score += abs(float(result["arm_boom"]) - 70.0) / 60.0
                score += abs(safe_z - 0.45) / 10.0
                merged.append((score, safe_z, bucket_abs, result, target_x, target_z))

        merged.sort(key=lambda item: item[0])
        if not merged:
            raise ValueError("卸料点在当前限位和空中卸料约束下无可行解。")

        _, safe_z, bucket_abs, result, target_x, target_z = merged[0]
        return {
            "radius": radius,
            "yaw": yaw,
            "z": safe_z,
            "z_original": z,
            "bucket_abs_angle_deg": bucket_abs,
            "result": result,
            "wrist_target_x": target_x,
            "wrist_target_z": target_z,
        }

    @staticmethod
    def _step(step_id, joint, description, target_val, ch3_mv=3500, ramp_up_s=0.2, ramp_down_s=0.2, is_init_step=False):
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
        dig = self.solve_dig_pose(dig_point)
        dump = self.solve_dump_pose(dump_point)

        dig_res = dig["result"]
        dump_res = dump["result"]

        # 动作语义修正：
        # 1. 挖掘时先以“半开斗”姿态下探，再执行收斗。
        # 2. 卸料时最终铲斗应接近 0 度，而不是继续收斗。
        dig_entry_bucket = self._clamp("bucket_arm", -55.0)
        scoop_bucket = self._clamp("bucket_arm", 0.0)
        scoop_arm = self._clamp("arm_boom", max(dig_res["arm_boom"], 72.0))
        lift_boom = self._clamp("boom_swing", dig_res["boom_swing"] - 12.0)
        carry_arm = self._clamp("arm_boom", max(scoop_arm, 58.0))
        carry_bucket = self._clamp("bucket_arm", -5.0)
        dump_prepare_bucket = self._clamp("bucket_arm", -18.0)
        unload_bucket = self._clamp("bucket_arm", -90.0)

        steps = []
        append = steps.append
        idx = 1

        append(self._step(idx, "swing_yaw", "对准挖掘点（回转）", dig["yaw"], is_init_step=True)); idx += 1
        append(self._step(idx, "bucket_arm", "挖掘预备-半开斗", dig_entry_bucket, is_init_step=True)); idx += 1
        append(self._step(idx, "boom_swing", "挖掘预备-大臂下探", dig_res["boom_swing"], is_init_step=True)); idx += 1
        append(self._step(idx, "arm_boom", "挖掘预备-小臂下探", dig_res["arm_boom"], is_init_step=True)); idx += 1

        append(self._step(idx, "bucket_arm", "收斗取料", scoop_bucket)); idx += 1
        append(self._step(idx, "arm_boom", "回拉小臂抬料", scoop_arm)); idx += 1
        append(self._step(idx, "boom_swing", "抬大臂离开挖掘点", lift_boom)); idx += 1
        append(self._step(idx, "arm_boom", "运输姿态-收小臂", carry_arm)); idx += 1
        append(self._step(idx, "bucket_arm", "运输姿态-稳料", carry_bucket)); idx += 1

        append(self._step(idx, "swing_yaw", "回转到卸料点", dump["yaw"])); idx += 1
        append(self._step(idx, "boom_swing", "卸料预备-大臂", dump_res["boom_swing"])); idx += 1
        append(self._step(idx, "arm_boom", "卸料预备-小臂", dump_res["arm_boom"])); idx += 1
        append(self._step(idx, "bucket_arm", "卸料预备-铲斗半开", dump_prepare_bucket)); idx += 1
        append(self._step(idx, "bucket_arm", "打开铲斗卸料", unload_bucket)); idx += 1

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
    parser = argparse.ArgumentParser(description="根据挖掘点和卸料点生成关节轨迹 JSON。")
    parser.add_argument("--dig-x", type=float, required=True)
    parser.add_argument("--dig-y", type=float, required=True)
    parser.add_argument("--dig-z", type=float, required=True)
    parser.add_argument("--dump-x", type=float, required=True)
    parser.add_argument("--dump-y", type=float, required=True)
    parser.add_argument("--dump-z", type=float, required=True)
    parser.add_argument("--output", type=str, default=os.path.join(CURRENT_DIR, "json", "generated_dig_dump_trajectory.json"))
    args = parser.parse_args()

    planner = DigDumpPlanner()
    result = planner.plan(
        dig_point=(args.dig_x, args.dig_y, args.dig_z),
        dump_point=(args.dump_x, args.dump_y, args.dump_z),
    )

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)

    print("[轨迹生成] 输出文件:", args.output)
    print("[轨迹生成] 挖掘点 yaw:", round(result["metadata"]["dig_point"]["swing_yaw"], 2))
    print("[轨迹生成] 卸料点 yaw:", round(result["metadata"]["dump_point"]["swing_yaw"], 2))
    print("[轨迹生成] 空中卸料高度 z:", round(result["metadata"]["dump_point"]["safe_dump_z"], 3))
    print("[轨迹生成] 共生成步骤:", len(result["script"]))


if __name__ == "__main__":
    main()
