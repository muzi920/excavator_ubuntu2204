from animate_trajectory import TrajectoryAnimator
animator = TrajectoryAnimator("../v4_control_closed/test3.json")
frames = animator.generate_frames()
fk = animator.kin

for i, f in enumerate(frames):
    res = fk.forward_kinematics_v4(f["boom_swing"], f["arm_boom"], f["bucket_arm"])
    arm_x, arm_z = res["arm_tip"]
    bucket_x, bucket_z = res["bucket_tip"]
    print(f"Frame {i:2d}: boom={f['boom_swing']:5.1f}, arm_rel={f['arm_boom']:5.1f}, buck_rel={f['bucket_arm']:5.1f} | arm_tip=({arm_x:.2f}, {arm_z:.2f}) | buck_tip=({bucket_x:.2f}, {bucket_z:.2f})")
    if i > 15: break
