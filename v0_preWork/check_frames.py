from src.shandong.v10_cailbration.animate_trajectory import TrajectoryAnimator
import numpy as np

animator = TrajectoryAnimator("src/shandong/v4_control_closed/test3.json")
frames = animator.generate_frames()
fk = animator.kin

# We are interested in the step where boom_swing changes.
# It starts around frame 6 (after 5 interp steps of step 4)
for i, f in enumerate(frames):
    res = fk.forward_kinematics_v4(f['boom_swing'], f['arm_boom'], f['bucket_arm'])
    arm_x, arm_z = res['arm_tip']
    bucket_x, bucket_z = res['bucket_tip']
    print(f"Frame {i}: boom={f['boom_swing']:.1f}, arm_rel={f['arm_boom']:.1f}, buck_rel={f['bucket_arm']:.1f} | arm_tip=({arm_x:.2f}, {arm_z:.2f}) | buck_tip=({bucket_x:.2f}, {bucket_z:.2f})")
    if i > 20: break
