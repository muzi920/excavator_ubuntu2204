#pragma once

#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "v13_excavator_ros/msg/inclinometer_group.hpp"
#include "v13_excavator_ros/msg/lidar_imu.hpp"
#include "v13_excavator_ros/msg/robot_joint_state.hpp"

namespace v13_excavator_ros::utils
{
class V13MainNode : public rclcpp::Node
{
public:
  explicit V13MainNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_inclinometer_group(const v13_excavator_ros::msg::InclinometerGroup::SharedPtr msg);
  void on_lidar_imu(const v13_excavator_ros::msg::LidarImu::SharedPtr msg);
  void try_publish_locked();

  std::mutex mutex_;
  double max_sync_delta_sec_{0.05};
  double last_published_inclinometer_sec_{-1.0};
  double last_published_lidar_imu_sec_{-1.0};
  std::optional<v13_excavator_ros::msg::InclinometerGroup> latest_inclinometer_group_;
  std::optional<v13_excavator_ros::msg::LidarImu> latest_lidar_imu_;
  rclcpp::Subscription<v13_excavator_ros::msg::InclinometerGroup>::SharedPtr inclinometer_group_sub_;
  rclcpp::Subscription<v13_excavator_ros::msg::LidarImu>::SharedPtr lidar_imu_sub_;
  rclcpp::Publisher<v13_excavator_ros::msg::RobotJointState>::SharedPtr robot_joint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;
};
}  // namespace v13_excavator_ros::utils
