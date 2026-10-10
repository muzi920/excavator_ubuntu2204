#pragma once

#include <array>
#include <cstddef>
#include <optional>

namespace v13_excavator_ros::utils
{
struct JointPitchSnapshot
{
  double bucket_pitch_deg{0.0};
  double arm_pitch_deg{0.0};
  double boom_pitch_deg{0.0};
  double swing_pitch_deg{0.0};
};

struct InclinometerGroupFrame
{
  double timestamp_sec{0.0};
  bool initialized{false};
  double bucket_pitch_deg{0.0};
  double arm_pitch_deg{0.0};
  double boom_pitch_deg{0.0};
  double swing_pitch_deg{0.0};
  double bucket_arm_deg{0.0};
  double arm_boom_deg{0.0};
  double boom_swing_deg{0.0};
};

struct LidarImuFrame
{
  double timestamp_sec{0.0};
  bool calibrated{false};
  double swing_yaw_deg{0.0};
  double yaw_rate{0.0};
};

struct RobotJointFrame
{
  double timestamp_sec{0.0};
  double bucket_arm_deg{0.0};
  double arm_boom_deg{0.0};
  double boom_swing_deg{0.0};
  double swing_yaw_deg{0.0};
  double yaw_rate{0.0};
  double sync_delta_sec{0.0};
};

class JointPreprocessor
{
public:
  explicit JointPreprocessor(std::size_t init_sample_count = 5U);

  std::optional<InclinometerGroupFrame> update(
    const JointPitchSnapshot & snapshot,
    double timestamp_sec);

  bool initialized() const;

private:
  std::size_t init_sample_count_{5U};
  std::size_t sample_counter_{0U};
  bool initialized_{false};
  std::array<double, 4> offset_sums_{0.0, 0.0, 0.0, 0.0};
  std::array<double, 4> offsets_{0.0, 0.0, 0.0, 0.0};
};

std::optional<RobotJointFrame> try_fuse_robot_joints(
  const InclinometerGroupFrame & inclinometer,
  const LidarImuFrame & lidar_imu,
  double max_sync_delta_sec);
}  // namespace v13_excavator_ros::utils
