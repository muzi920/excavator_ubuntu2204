import json
import os
import copy

def generate_dig_scripts():
    template_path = os.path.join(os.path.dirname(__file__), "auto_dig_template.json")
    output_dir = os.path.join(os.path.dirname(__file__), "auto_dig_scripts")
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    with open(template_path, 'r', encoding='utf-8') as f:
        template = json.load(f)
        
    # 定义回拉角度 (45度 到 55度，步长2度，共6个深度)
    arm_angles = [45.0, 47.0, 49.0, 51.0, 53.0, 55.0]
    
    # 定义回转时间 (0.1s 到 2.5s，这里为了生成不过多文件，步长选0.4s，共7个方位)
    swing_times = [0.1, 0.5, 0.9, 1.3, 1.7, 2.1, 2.5]
    
    count = 0
    for angle in arm_angles:
        for s_time in swing_times:
            script = copy.deepcopy(template)
            
            # 1. 修改第一遍的挖土动作参数
            # 第10步是第一遍挖土的小臂回拉 (原 55.2)
            script[9]["target_val"] = angle
            # 第15步是第一遍挖土后的回转 (原 -1.0) -> 这里假设向左转倒土
            script[14]["duration_s"] = -s_time
            # 第18步是倒土后回转归位 (原 1.0) -> 这里假设向右转回来
            script[17]["duration_s"] = s_time
            
            # 2. 修改第二遍的挖土动作参数 (原版追加了后半段)
            # 第23步是第二遍挖土的小臂回拉 (原 55.2)
            script[22]["target_val"] = angle
            # 第28步是第二遍挖土后的回转 (原 -1.0)
            script[27]["duration_s"] = -s_time
            # 第31步是倒土后回转归位 (原 1.0)
            script[30]["duration_s"] = s_time
            
            # 生成文件名
            filename = f"dig_angle_{int(angle)}_swing_{s_time:.1f}s.json"
            filepath = os.path.join(output_dir, filename)
            
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(script, f, ensure_ascii=False, indent=2)
            count += 1
            
    print(f"成功基于模板生成了 {count} 个区域挖掘剧本，存放在 auto_dig_scripts/ 目录下。")

if __name__ == "__main__":
    generate_dig_scripts()
