import json
import copy
import os
import argparse

# ================= 配置区 =================
# 1. 默认参数 (可以通过命令行覆盖)
DEFAULT_OUTPUT = "auto_dig_imu_30_loops.json"
DEFAULT_LOOPS = 30

# 2. 挖掘区扫掠角度配置
DIG_SWING_BASE = 0.0      # 挖掘区基准起始角度
DIG_SWING_RANGE = 20.0    # 挖掘区旋转区间
DIG_SWING_STEP = 4.0      # 每次挖掘旋转间隔

# 3. 卸料区固定角度配置
UNLOAD_SWING_TARGET = -90.0  # 假设向左转 90 度卸料 (正右负左)

# 4. 步骤识别关键字 (请确保录制 JSON 时，回转动作的 description 包含以下关键字)
KEYWORD_DIG_SWING = "到挖掘区"
KEYWORD_UNLOAD_SWING = "到卸料区"
# ==========================================

def generate_dig_angles(total_loops):
    """生成扇形扫掠的角度序列: 0, 4, 8, 12, 16, 20, 16, 12..."""
    angles = []
    current = 0.0
    direction = 1
    
    max_offset = DIG_SWING_RANGE
    step = DIG_SWING_STEP
    
    for _ in range(total_loops):
        angles.append(DIG_SWING_BASE + current)
        current += direction * step
        
        if current > max_offset:
            current = max_offset - step
            direction = -1
        elif current < 0:
            current = step
            direction = 1
            
    return angles

def main():
    parser = argparse.ArgumentParser(description="生成基于 IMU 回转绝对角度的 30 轮挖掘剧本")
    parser.add_argument("--json", type=str, required=True, help="输入的模板剧本 JSON 文件名")
    parser.add_argument("--out", type=str, default=DEFAULT_OUTPUT, help=f"输出的剧本 JSON 文件名 (默认: {DEFAULT_OUTPUT})")
    parser.add_argument("--loops", type=int, default=DEFAULT_LOOPS, help=f"生成的循环轮数 (默认: {DEFAULT_LOOPS})")
    
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    json_dir = os.path.abspath(os.path.join(script_dir, "..", "json"))
    os.makedirs(json_dir, exist_ok=True)
    
    # 默认从 json_dir 读取和保存
    template_path = os.path.join(json_dir, args.json) if not os.path.isabs(args.json) else args.json
    output_path = os.path.join(json_dir, args.out) if not os.path.isabs(args.out) else args.out

    if not os.path.exists(template_path):
        print(f"错误: 找不到模板文件 {template_path}")
        return

    with open(template_path, 'r', encoding='utf-8') as f:
        template_steps = json.load(f)

    dig_angles = generate_dig_angles(args.loops)
    
    final_script = []
    global_step_counter = 1

    for loop_idx in range(args.loops):
        current_dig_angle = dig_angles[loop_idx]
        
        for step in template_steps:
            new_step = copy.deepcopy(step)
            desc = new_step.get("description", "")
            joint = new_step.get("joint", "")
            
            # 如果是回转动作，根据关键字修改目标角度
            if joint == "swing_yaw":
                if KEYWORD_DIG_SWING in desc:
                    new_step["target_val"] = round(current_dig_angle, 1)
                    new_step["description"] = f"{desc} ({new_step['target_val']}°)"
                elif KEYWORD_UNLOAD_SWING in desc:
                    new_step["target_val"] = round(UNLOAD_SWING_TARGET, 1)
                    new_step["description"] = f"{desc} ({new_step['target_val']}°)"
            
            # 更新全局步骤号
            new_step["step"] = global_step_counter
            final_script.append(new_step)
            global_step_counter += 1

    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(final_script, f, ensure_ascii=False, indent=2)

    print(f"成功生成 {args.loops} 轮闭环挖掘剧本！")
    print(f"保存路径: {output_path}")
    print(f"挖掘区角度扫掠序列: {[round(a,1) for a in dig_angles]}")

if __name__ == "__main__":
    main()
