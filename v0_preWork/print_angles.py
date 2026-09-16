import math
from src.shandong.v10_cailbration.kinematics import ExcavatorKinematics

fk = ExcavatorKinematics()

def print_frame(boom_swing, arm_boom, bucket_arm):
    sensor_boom_deg = boom_swing
    sensor_arm_deg = sensor_boom_deg + arm_boom
    sensor_bucket_deg = sensor_arm_deg + bucket_arm
    
    abs_boom_L2_deg = fk.offset_boom - sensor_boom_deg
    abs_arm_deg = fk.offset_arm - sensor_arm_deg
    abs_bucket_deg = fk.offset_bucket - sensor_bucket_deg
    
    abs_boom_L1_deg = abs_boom_L2_deg + fk.boom_bend_angle_deg
    
    print(f"boom_swing: {boom_swing:5.1f}")
    print(f"  L1 angle (deg): {abs_boom_L1_deg:5.1f}")
    print(f"  L2 angle (deg): {abs_boom_L2_deg:5.1f}")
    print(f"  Arm angle (deg): {abs_arm_deg:5.1f}")
    print(f"  Bucket angle (deg): {abs_bucket_deg:5.1f}")
    print(f"  Visual Angle (Arm - L2): {abs_arm_deg - abs_boom_L2_deg:5.1f}")
    print(f"  Visual Angle (Bucket - Arm): {abs_bucket_deg - abs_arm_deg:5.1f}")
    
    res = fk.forward_kinematics_v4(boom_swing, arm_boom, bucket_arm)
    print(f"  L1 tip: ({res['boom_bend'][0]:.2f}, {res['boom_bend'][1]:.2f})")
    print(f"  L2 tip: ({res['boom_tip'][0]:.2f}, {res['boom_tip'][1]:.2f})")
    print(f"  Arm tip: ({res['arm_tip'][0]:.2f}, {res['arm_tip'][1]:.2f})")
    print(f"  Bucket tip: ({res['bucket_tip'][0]:.2f}, {res['bucket_tip'][1]:.2f})")
    print("-" * 40)

print_frame(0.7, -1.0, -92.7)
print_frame(47.3, -1.0, -92.7)

