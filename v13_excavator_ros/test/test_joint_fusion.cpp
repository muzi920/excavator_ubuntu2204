#include <gtest/gtest.h>

#include "v13_excavator_ros/utils/joint_fusion_helpers.hpp"

namespace v13_excavator_ros::utils
{
TEST(JointFusionHelpersTest, ComputesRelativeAnglesAfterInitialization)
{
  JointPreprocessor preprocessor;

  for (int i = 0; i < 5; ++i) {
    const auto maybe = preprocessor.update({10.0, 20.0, 30.0, 40.0}, 1.0 + i * 0.01);
    EXPECT_FALSE(maybe.has_value());
  }

  const auto fused = preprocessor.update({15.0, 26.0, 38.0, 42.0}, 2.0);
  ASSERT_TRUE(fused.has_value());
  EXPECT_NEAR(fused->bucket_arm_deg, -1.0, 1e-6);
  EXPECT_NEAR(fused->arm_boom_deg, -2.0, 1e-6);
  EXPECT_NEAR(fused->boom_swing_deg, 6.0, 1e-6);
  EXPECT_TRUE(fused->initialized);
}

TEST(JointFusionHelpersTest, RejectsFusionWhenTimestampsTooFarApart)
{
  InclinometerGroupFrame incl{};
  incl.timestamp_sec = 10.0;
  incl.bucket_arm_deg = 11.0;
  incl.arm_boom_deg = 22.0;
  incl.boom_swing_deg = 33.0;
  incl.initialized = true;

  LidarImuFrame imu{};
  imu.timestamp_sec = 10.2;
  imu.swing_yaw_deg = 44.0;
  imu.yaw_rate = 0.5;
  imu.calibrated = true;

  const auto fused = try_fuse_robot_joints(incl, imu, 0.05);
  EXPECT_FALSE(fused.has_value());
}

TEST(JointFusionHelpersTest, FusesRobotJointStateWhenInputsAligned)
{
  InclinometerGroupFrame incl{};
  incl.timestamp_sec = 20.0;
  incl.bucket_arm_deg = 12.0;
  incl.arm_boom_deg = 23.0;
  incl.boom_swing_deg = 34.0;
  incl.initialized = true;

  LidarImuFrame imu{};
  imu.timestamp_sec = 20.01;
  imu.swing_yaw_deg = -45.0;
  imu.yaw_rate = 0.12;
  imu.calibrated = true;

  const auto fused = try_fuse_robot_joints(incl, imu, 0.05);
  ASSERT_TRUE(fused.has_value());
  EXPECT_NEAR(fused->bucket_arm_deg, 12.0, 1e-6);
  EXPECT_NEAR(fused->arm_boom_deg, 23.0, 1e-6);
  EXPECT_NEAR(fused->boom_swing_deg, 34.0, 1e-6);
  EXPECT_NEAR(fused->swing_yaw_deg, -45.0, 1e-6);
  EXPECT_NEAR(fused->yaw_rate, 0.12, 1e-6);
  EXPECT_NEAR(fused->sync_delta_sec, 0.01, 1e-6);
}
}  // namespace v13_excavator_ros::utils
