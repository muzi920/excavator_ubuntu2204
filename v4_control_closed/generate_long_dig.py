import json
import os
import copy
import numpy as np

def generate_long_script():
    json_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "json"))
    os.makedirs(json_dir, exist_ok=True)
    template_path = os.path.join(json_dir, "auto_dig_template.json")
    output_path = os.path.join(json_dir, "long_auto_dig.json")
    
    with open(template_path, 'r', encoding='utf-8') as f:
        template = json.load(f)
        
    init_steps = template[:5]
    dig_template = template[5:18]
    
    final_script = copy.deepcopy(init_steps)
    
    arm_angles = np.arange(45.0, 56.0, 1.0)

    # 回转时间：每一轮包含两次回转（卸料左转 + 回正右转）。
    # 为了避免“每次回正都回到原点”，这里采用“回正比卸料多 0.1s”的方式，
    # 让每轮净向右偏移 0.1s：
    #   左转 -1.0, 回正 +1.1
    #   左转 -1.1, 回正 +1.2
    #   ...
    #   左转 -2.4, 回正 +2.5
    dump_times = np.arange(1.0, 2.5, 0.1)  # 1.0 ~ 2.4，共 15 个
    
    current_step_idx = 6
    round_count = 1
    
    for angle in arm_angles:
        for dump_t in dump_times:
            round_actions = copy.deepcopy(dig_template)
            
            # 小臂回拉深度
            round_actions[4]["target_val"] = float(angle)

            # 大臂下降过程目标角度固定为 36°（原模板为 37.1°）
            round_actions[0]["target_val"] = 36.0
            
            # 1. 卸料回转（左转）
            round_actions[9]["duration_s"] = -float(dump_t)

            # 2. 回正回转（右转）：比卸料多 0.1s，保证每轮净向右偏移 0.1s
            round_actions[12]["duration_s"] = float(dump_t + 0.1)
            
            for action in round_actions:
                action["step"] = current_step_idx
                original_desc = action["description"].split(" (轮次:")[0]
                action["description"] = f"{original_desc} (轮次:{round_count}, 深度:{angle}°, 左转:{dump_t:.1f}s, 右转:{dump_t + 0.1:.1f}s)"
                current_step_idx += 1
                
            final_script.extend(round_actions)
            round_count += 1
            
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(final_script, f, ensure_ascii=False, indent=2)
        
    print(f"成功生成！修复了相对时间累积问题。")

if __name__ == "__main__":
    generate_long_script()
