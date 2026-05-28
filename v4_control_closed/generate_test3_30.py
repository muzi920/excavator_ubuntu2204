import copy
import json
import os


def _load(path: str):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _dump(path: str, data):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def generate(
    template_json: str = "test3.json",
    output_json: str = "test3_generated_30.json",
    cycles: int = 30,
    dig_yaw_start_s: float = 0.0,
    dig_yaw_step_s: float = 0.1,
    dump_offset_s: float = 0.4,
    dump_return_delta_s: float = 0.1,
):
    base_dir = os.path.dirname(__file__)
    json_dir = os.path.abspath(os.path.join(base_dir, "..", "json"))
    os.makedirs(json_dir, exist_ok=True)
    template_path = template_json if os.path.isabs(template_json) else os.path.join(json_dir, template_json)
    output_path = output_json if os.path.isabs(output_json) else os.path.join(json_dir, output_json)

    template = _load(template_path)
    init_steps = [s for s in template if s.get("is_init_step") is True or "初始" in s.get("description", "") or "初始化" in s.get("description", "")]

    by_step = {}
    for s in template:
        try:
            by_step[int(s.get("step"))] = s
        except Exception:
            continue

    cycle_steps = []
    for step_num in range(4, 14):
        if step_num not in by_step:
            raise RuntimeError(f"模板缺少 step={step_num}，无法提取作业循环段")
        cycle_steps.append(by_step[step_num])

    def is_swing(s):
        return s.get("joint") == "swing_yaw"

    swing_indices = [i for i, s in enumerate(cycle_steps) if is_swing(s)]
    if len(swing_indices) != 2:
        raise RuntimeError("模板作业段(4-13)中 swing_yaw 数量不等于 2，无法生成")
    dump_idx, return_idx = swing_indices

    max_swing_abs = 3.5
    out = []
    out.extend(copy.deepcopy(init_steps))
    next_step = (max([int(s.get("step", 0)) for s in init_steps]) + 1) if init_steps else 1

    yaw = float(dig_yaw_start_s)
    for i in range(1, cycles + 1):
        seg = copy.deepcopy(cycle_steps)

        dump_t = -(yaw + dump_offset_s)
        return_t = yaw + dump_offset_s + dump_return_delta_s

        if abs(dump_t) > max_swing_abs or abs(return_t) > max_swing_abs:
            raise RuntimeError(
                f"第{i}轮回转超限：dump={dump_t:.2f}s, return={return_t:.2f}s, "
                f"max={max_swing_abs}s。可调小 dig_yaw_start_s/dig_yaw_step_s 或 dump_offset_s。"
            )

        seg[dump_idx]["duration_s"] = float(round(dump_t, 2))
        seg[return_idx]["duration_s"] = float(round(return_t, 2))

        for s in seg:
            s["step"] = next_step
            base_desc = str(s.get("description", "")).split(" (轮次:")[0]
            if s.get("joint") == "swing_yaw":
                s["description"] = f"{base_desc} (轮次:{i}, 左转:{abs(dump_t):.1f}s, 右转:{return_t:.1f}s)"
            else:
                s["description"] = f"{base_desc} (轮次:{i}, 挖土方位:{yaw:.1f}s)"
            next_step += 1

        out.extend(seg)
        yaw += float(dig_yaw_step_s)

    final_reset = copy.deepcopy(cycle_steps[return_idx])
    final_reset["step"] = next_step
    final_reset["duration_s"] = float(round(-yaw, 2))
    final_reset["description"] = f"结束回正回转 (回正:{-yaw:.1f}s)"
    out.append(final_reset)

    _dump(output_path, out)
    print(f"已生成 {cycles} 轮作业 + 结束回正：{output_path}")


if __name__ == "__main__":
    generate()

