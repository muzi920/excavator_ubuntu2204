from src.shandong.v10_cailbration.kinematics import ExcavatorKinematics

fk = ExcavatorKinematics()

# Step 4 interpolation
state = {'boom_swing': 0.7, 'arm_boom': -1.0, 'bucket_arm': -92.7}
start = 0.7
end = 47.3
print("Interpolating boom_swing from", start, "to", end)
for i in range(6):
    val = start + (end - start) * (i / 5.0)
    res = fk.forward_kinematics_v4(val, state['arm_boom'], state['bucket_arm'])
    print(f"boom: {val:5.1f} | arm_tip: ({res['arm_tip'][0]:.2f}, {res['arm_tip'][1]:.2f}) | bucket_tip: ({res['bucket_tip'][0]:.2f}, {res['bucket_tip'][1]:.2f}) | theta_L2: {fk.offset_boom - val:.1f}")
