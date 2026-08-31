#!/usr/bin/env python3
import json
import math
import argparse
from pathlib import Path
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import rosbag2_py

def process_bag(bag_path, output_json, stable_threshold=1.0, steady_time_s=1.0, min_move_deg=2.0):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    if '/excavator/joint_states' not in type_map:
        print("Error: Bag does not contain /excavator/joint_states topic.")
        return

    joint_state_type = get_message(type_map['/excavator/joint_states'])

    # State tracking
    joints_history = {"boom_swing": [], "arm_boom": [], "bucket_arm": [], "swing_yaw": []}
    
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/excavator/joint_states':
            msg = deserialize_message(data, joint_state_type)
            ts_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            try:
                boom_idx = msg.name.index('boom_joint')
                arm_idx = msg.name.index('arm_joint')
                bucket_idx = msg.name.index('bucket_joint')
                swing_idx = msg.name.index('swing_joint')
                
                # Convert radians back to degrees for the JSON script
                joints_history["boom_swing"].append((ts_sec, math.degrees(msg.position[boom_idx])))
                joints_history["arm_boom"].append((ts_sec, math.degrees(msg.position[arm_idx])))
                joints_history["bucket_arm"].append((ts_sec, math.degrees(msg.position[bucket_idx])))
                joints_history["swing_yaw"].append((ts_sec, math.degrees(msg.position[swing_idx])))
            except ValueError:
                continue

    # Segmentation algorithm to find start and end points of movements
    script_events = []
    window_size = 10 # 约 0.5 秒 (假设 20Hz)
    
    for joint_name, history in joints_history.items():
        if len(history) < window_size:
            continue
            
        is_moving = False
        steady_start_time = None
        
        # 初始值
        last_recorded_val = sum(x[1] for x in history[:window_size]) / window_size
        
        for i in range(len(history) - window_size):
            window = history[i:i+window_size]
            vals = [x[1] for x in window]
            ts = window[-1][0]
            
            val_max = max(vals)
            val_min = min(vals)
            val_avg = sum(vals)/len(vals)
            
            if val_max - val_min > stable_threshold:
                is_moving = True
                steady_start_time = None
            else:
                if is_moving:
                    if steady_start_time is None:
                        steady_start_time = ts
                    elif ts - steady_start_time > steady_time_s:
                        # 动作结束，检查是否产生了实质性位移 (过滤抖动)
                        if abs(val_avg - last_recorded_val) > min_move_deg:
                            action = {
                                "command": "move_joint",
                                "joint": joint_name,
                                "target_val": round(val_avg, 1),
                                "ch1_mv": 0,
                                "ch2_mv": 0,
                                "ch3_mv": 2000,
                                "ramp_up_s": 0.5,
                                "ramp_down_s": 0.5,
                                "tolerance_deg": 2.0,
                                "is_init_step": False,
                                "description": f"Bag自动提取: {joint_name} 运动至 {val_avg:.1f}°"
                            }
                            script_events.append((ts, action))
                            last_recorded_val = val_avg
                        
                        is_moving = False
                        steady_start_time = None

    # Sort script by timestamp so movements occur in chronological order
    script_events.sort(key=lambda x: x[0])
    
    final_json = []
    for i, (ts, act) in enumerate(script_events):
        act["step"] = i + 1
        final_json.append(act)
    
    with open(output_json, 'w', encoding='utf-8') as f:
        json.dump(final_json, f, indent=4, ensure_ascii=False)
        
    print(f"Successfully generated {output_json} with {len(final_json)} actions.")
    print("您可以打开该 JSON 文件进行手动微调与纠正。")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="将 ROS2 Bag 里的挖掘机动作提取为 JSON 剧本")
    parser.add_argument("--bag", required=True, help="rosbag 目录路径")
    parser.add_argument("--out", required=True, help="输出的 json 文件路径")
    parser.add_argument("--threshold", type=float, default=1.0, help="判定为运动的最小波动范围(度)")
    parser.add_argument("--steady_time", type=float, default=1.0, help="保持平稳多少秒后认为动作结束")
    parser.add_argument("--min_move", type=float, default=2.0, help="起始与终点角度差大于此值才记录(过滤抖动)")
    args = parser.parse_args()
    
    process_bag(args.bag, args.out, args.threshold, args.steady_time, args.min_move)
