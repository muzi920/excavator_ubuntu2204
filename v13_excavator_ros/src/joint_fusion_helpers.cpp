#include "v13_excavator_ros/utils/joint_fusion_helpers.hpp"

#include <cmath>

namespace v13_excavator_ros::utils
{
JointPreprocessor::JointPreprocessor(std::size_t init_sample_count)
: init_sample_count_(init_sample_count == 0U ? 1U : init_sample_count)
{
}

std::optional<InclinometerGroupFrame> JointPreprocessor::update(
  const JointPitchSnapshot & snapshot,
  double timestamp_sec)
{
  const std::array<double, 4> pitches{
    snapshot.bucket_pitch_deg,
    snapshot.arm_pitch_deg,
    snapshot.boom_pitch_deg,
    snapshot.swing_pitch_deg};

  if (!initialized_) {
    for (std::size_t i = 0; i < pitches.size(); ++i) {
      offset_sums_[i] += pitches[i];
    }
    ++sample_counter_;
    if (sample_counter_ < init_sample_count_) {
      return std::nullopt;
    }
    for (std::size_t i = 0; i < pitches.size(); ++i) {
      offsets_[i] = offset_sums_[i] / static_cast<double>(sample_counter_);
    }
    initialized_ = true;
    return std::nullopt;
  }

  InclinometerGroupFrame frame;
  frame.timestamp_sec = timestamp_sec;
  frame.initialized = true;
  frame.bucket_pitch_deg = pitches[0] - offsets_[0];
  frame.arm_pitch_deg = pitches[1] - offsets_[1];
  frame.boom_pitch_deg = pitches[2] - offsets_[2];
  frame.swing_pitch_deg = pitches[3] - offsets_[3];
  frame.bucket_arm_deg = frame.bucket_pitch_deg - frame.arm_pitch_deg;
  frame.arm_boom_deg = frame.arm_pitch_deg - frame.boom_pitch_deg;
  frame.boom_swing_deg = frame.boom_pitch_deg - frame.swing_pitch_deg;
  return frame;
}

bool JointPreprocessor::initialized() const
{
  return initialized_;
}

std::optional<RobotJointFrame> try_fuse_robot_joints(
  const InclinometerGroupFrame & inclinometer,
  const LidarImuFrame & lidar_imu,
  double max_sync_delta_sec)
{
  if (!inclinometer.initialized || !lidar_imu.calibrated) {
    return std::nullopt;
  }

  const double sync_delta_sec = std::fabs(inclinometer.timestamp_sec - lidar_imu.timestamp_sec);
  if (sync_delta_sec > max_sync_delta_sec) {
    return std::nullopt;
  }

  RobotJointFrame frame;
  frame.timestamp_sec = inclinometer.timestamp_sec > lidar_imu.timestamp_sec ?
    inclinometer.timestamp_sec : lidar_imu.timestamp_sec;
  frame.bucket_arm_deg = inclinometer.bucket_arm_deg;
  frame.arm_boom_deg = inclinometer.arm_boom_deg;
  frame.boom_swing_deg = inclinometer.boom_swing_deg;
  frame.swing_yaw_deg = lidar_imu.swing_yaw_deg;
  frame.yaw_rate = lidar_imu.yaw_rate;
  frame.sync_delta_sec = sync_delta_sec;
  return frame;
}
}  // namespace v13_excavator_ros::utils
