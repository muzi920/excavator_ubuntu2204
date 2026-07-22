from src.shandong.v10_cailbration.kinematics import ExcavatorKinematics

fk = ExcavatorKinematics()

# OLD logic simulation
start = 0.7
end = 47.3
print("OLD LOGIC: Interpolating boom_swing from", start, "to", end)
for i in range(6):
    val = start + (end - start) * (i / 5.0)
    # OLD LOGIC DID NOT ADD boom_swing TO arm_boom!
    res = fk.forward_kinematics(sensor_boom_deg=val, sensor_arm_deg=-1.0, sensor_bucket_deg=-92.7)
    print(f"boom: {val:5.1f} | boom_tip: ({res['boom_tip'][0]:.2f}, {res['boom_tip'][1]:.2f}) | arm_tip: ({res['arm_tip'][0]:.2f}, {res['arm_tip'][1]:.2f}) | bucket_tip: ({res['bucket_tip'][0]:.2f}, {res['bucket_tip'][1]:.2f})")
