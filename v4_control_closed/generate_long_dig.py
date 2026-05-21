import json
import os
import copy
import numpy as np

def generate_long_script():
    template_path = os.path.join(os.path.dirname(__file__), "auto_dig_template.json")
    output_path = os.path.join(os.path.dirname(__file__), "long_auto_dig.json")
    
    with open(template_path, 'r', encoding='utf-8') as f:
        template = json.load(f)
        
    init_steps = template[:5]
    dig_template = template[5:18]
    
    final_script = copy.deepcopy(init_steps)
    
    arm_angles = np.arange(45.0, 56.0, 1.0)
    base_swing_times = np.arange(1.0, 2.6, 0.1)
    
    current_step_idx = 6
    round_count = 1
    
    # 假设我们希望每次卸料都卸在“相对于车体正前方的左侧固定点”，
    # 但是因为我们挖土的位置（回正位置）一直在向右偏移，
    # 所以从挖土点回到左侧固定卸料点所需要的向左回转时间必须越来越长！
    
    # 我们设定一个固定的“卸料点方位”，假设它是正前方偏左 1.0s 的位置 (-1.0s)
    # 而挖土点方位是正前方偏右 (swing_time，比如 +1.5s)
    # 那么从挖土点转到卸料点，需要的相对时间就是: - (swing_time + 1.0)
    
    fixed_dump_position = 1.0 # 卸料点在绝对坐标下偏左 1.0s
    
    for angle in arm_angles:
        for swing_time in base_swing_times:
            round_actions = copy.deepcopy(dig_template)
            
            # 小臂回拉深度
            round_actions[4]["target_val"] = float(angle)
            
            # 1. 卸料回转 (向左转)
            # 因为你挖土的位置向右偏了 swing_time，要转到固定左侧卸料点
            # 就必须先把向右偏的这段转回来，再继续向左转 fixed_dump_position
            dump_duration = -(float(swing_time) + fixed_dump_position)
            round_actions[9]["duration_s"] = dump_duration
            
            # 2. 挖土回正 (向右转)
            # 卸完料之后，从左侧卸料点回到右侧的挖土点
            # 就必须先把向左偏的 fixed_dump_position 转回来，再继续向右转 swing_time
            return_duration = float(swing_time) + fixed_dump_position
            round_actions[12]["duration_s"] = return_duration
            
            for action in round_actions:
                action["step"] = current_step_idx
                original_desc = action["description"].split(" (轮次:")[0]
                action["description"] = f"{original_desc} (轮次:{round_count}, 深度:{angle}°, 挖土方位:{swing_time:.1f}s)"
                current_step_idx += 1
                
            final_script.extend(round_actions)
            round_count += 1
            
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(final_script, f, ensure_ascii=False, indent=2)
        
    print(f"成功生成！修复了相对时间累积问题。")

if __name__ == "__main__":
    generate_long_script()
