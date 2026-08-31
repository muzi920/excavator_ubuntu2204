#include "v13_excavator_ros/utils/v13_main_node.hpp"

#include <sstream>

#include "v13_excavator_ros/utils/joint_fusion_helpers.hpp"

namespace v13_excavator_ros::utils
{
namespace
{
double to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}
}  // namespace

V13MainNode::V13MainNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("v13_main_node", options)
{
  const auto inclinometer_group_topic = this->declare_parameter<std::string>(
    "inclinometer_group_topic", "/v13/inclinometer/group");
  const auto lidar_imu_topic = this->declare_parameter<std::string>(
    "lidar_imu_topic", "/v13/lidar/imu");
  const auto robot_joint_topic = this->declare_parameter<std::string>(
    "robot_joint_topic", "/v13/robot/joint_state");
  const auto summary_topic = this->declare_parameter<std::string>(
    "summary_topic", "/v13/system/summary");
  max_sync_delta_sec_ = this->declare_parameter<double>("max_sync_delta_sec", 0.05);

  inclinometer_group_sub_ = this->create_subscription<v13_excavator_ros::msg::InclinometerGroup>(
    inclinometer_group_topic, 20,
    std::bind(&V13MainNode::on_inclinometer_group, this, std::placeholders::_1));
  lidar_imu_sub_ = this->create_subscription<v13_excavator_ros::msg::LidarImu>(
    lidar_imu_topic, 20, std::bind(&V13MainNode::on_lidar_imu, this, std::placeholders::_1));
  robot_joint_pub_ = this->create_publisher<v13_excavator_ros::msg::RobotJointState>(
    robot_joint_topic, 20);
  summary_pub_ = this->create_publisher<std_msgs::msg::String>(summary_topic, 10);
}

void V13MainNode::on_inclinometer_group(
  const v13_excavator_ros::msg::InclinometerGroup::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_inclinometer_group_ = *msg;
  try_publish_locked();
}

void V13MainNode::on_lidar_imu(
  const v13_excavator_ros::msg::LidarImu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_lidar_imu_ = *msg;
  try_publish_locked();
}

void V13MainNode::try_publish_locked()
{
  if (!latest_inclinometer_group_.has_value() || !latest_lidar_imu_.has_value()) {
    return;
  }

  const double inclinometer_sec = to_seconds(latest_inclinometer_group_->header.stamp);
  const double lidar_imu_sec = to_seconds(latest_lidar_imu_->header.stamp);
  if (inclinometer_sec == last_published_inclinometer_sec_ &&
    lidar_imu_sec == last_published_lidar_imu_sec_)
  {
    return;
  }

  InclinometerGroupFrame inclinometer_frame{};
  inclinometer_frame.timestamp_sec = inclinometer_sec;
  inclinometer_frame.initialized = latest_inclinometer_group_->initialized;
  inclinometer_frame.bucket_pitch_deg = latest_inclinometer_group_->bucket_pitch_deg;
  inclinometer_frame.arm_pitch_deg = latest_inclinometer_group_->arm_pitch_deg;
  inclinometer_frame.boom_pitch_deg = latest_inclinometer_group_->boom_pitch_deg;
  inclinometer_frame.swing_pitch_deg = latest_inclinometer_group_->swing_pitch_deg;
  inclinometer_frame.bucket_arm_deg = latest_inclinometer_group_->bucket_arm_deg;
  inclinometer_frame.arm_boom_deg = latest_inclinometer_group_->arm_boom_deg;
  inclinometer_frame.boom_swing_deg = latest_inclinometer_group_->boom_swing_deg;

  LidarImuFrame lidar_imu_frame{};
  lidar_imu_frame.timestamp_sec = lidar_imu_sec;
  lidar_imu_frame.calibrated = latest_lidar_imu_->calibrated;
  lidar_imu_frame.swing_yaw_deg = latest_lidar_imu_->swing_deg;
  lidar_imu_frame.yaw_rate = latest_lidar_imu_->yaw_rate;

  const auto maybe_joint_frame = try_fuse_robot_joints(
    inclinometer_frame, lidar_imu_frame, max_sync_delta_sec_);
  if (!maybe_joint_frame.has_value()) {
    return;
  }

  v13_excavator_ros::msg::RobotJointState robot_msg;
  robot_msg.header.stamp = this->now();
  robot_msg.synchronized = true;
  robot_msg.bucket_arm_deg = maybe_joint_frame->bucket_arm_deg;
  robot_msg.arm_boom_deg = maybe_joint_frame->arm_boom_deg;
  robot_msg.boom_swing_deg = maybe_joint_frame->boom_swing_deg;
  robot_msg.swing_yaw_deg = maybe_joint_frame->swing_yaw_deg;
  robot_msg.yaw_rate = maybe_joint_frame->yaw_rate;
  robot_msg.inclinometer_stamp = latest_inclinometer_group_->header.stamp;
  robot_msg.lidar_imu_stamp = latest_lidar_imu_->header.stamp;
  robot_msg.sync_delta_sec = maybe_joint_frame->sync_delta_sec;
  robot_joint_pub_->publish(robot_msg);

  std_msgs::msg::String summary_msg;
  std::ostringstream oss;
  oss << "bucket_arm=" << robot_msg.bucket_arm_deg
      << " arm_boom=" << robot_msg.arm_boom_deg
      << " boom_swing=" << robot_msg.boom_swing_deg
      << " swing_yaw=" << robot_msg.swing_yaw_deg
      << " sync_delta=" << robot_msg.sync_delta_sec;
  summary_msg.data = oss.str();
  summary_pub_->publish(summary_msg);

  last_published_inclinometer_sec_ = inclinometer_sec;
  last_published_lidar_imu_sec_ = lidar_imu_sec;
}
}  // namespace v13_excavator_ros::utils

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<v13_excavator_ros::utils::V13MainNode>());
  rclcpp::shutdown();
  return 0;
}
