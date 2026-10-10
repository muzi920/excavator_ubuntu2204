from src.shandong.v10_cailbration.kinematics import ExcavatorKinematics
import math

fk = ExcavatorKinematics()
state = {'boom_swing': 0.7, 'arm_boom': -1.0, 'bucket_arm': -92.7}
start = 0.7
end = 47.3
for i in range(6):
    val = start + (end - start) * (i / 5.0)
    
    sensor_boom_deg = val
    sensor_arm_deg = sensor_boom_deg + state['arm_boom']
    sensor_bucket_deg = sensor_arm_deg + state['bucket_arm']
    
    abs_boom_L2_deg = fk.offset_boom - sensor_boom_deg
    abs_arm_deg = fk.offset_arm - sensor_arm_deg
    abs_bucket_deg = fk.offset_bucket - sensor_bucket_deg
    
    print(f"boom: {val:5.1f} | L2_deg: {abs_boom_L2_deg:5.1f} | arm_deg: {abs_arm_deg:5.1f} | bucket_deg: {abs_bucket_deg:5.1f} | rel_arm: {abs_arm_deg - abs_boom_L2_deg:5.1f} | rel_bucket: {abs_bucket_deg - abs_arm_deg:5.1f}")
