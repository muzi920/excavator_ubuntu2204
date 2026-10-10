#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "v13_excavator_ros/msg/inclinometer.hpp"
#include "v13_excavator_ros/msg/inclinometer_group.hpp"
#include "v13_excavator_ros/utils/joint_fusion_helpers.hpp"
#include "v13_excavator_ros/utils/serial_port.hpp"

namespace v13_excavator_ros::utils
{
class InclinometerReaderNode : public rclcpp::Node
{
public:
  explicit InclinometerReaderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void poll_once();
  void maybe_publish_group_locked(double timestamp_sec);
  void publish_status(const std::string & text);

  int baudrate_{230400};
  double max_group_skew_sec_{0.1};
  std::vector<std::string> port_names_;
  std::vector<int64_t> sensor_ids_;
  std::vector<std::string> sensor_names_;
  std::vector<std::unique_ptr<SerialPort>> serial_ports_;
  std::array<double, 4> latest_pitch_deg_{0.0, 0.0, 0.0, 0.0};
  std::array<double, 4> latest_timestamp_sec_{0.0, 0.0, 0.0, 0.0};
  std::array<bool, 4> have_pitch_{false, false, false, false};
  JointPreprocessor preprocessor_;
  std::mutex sensor_mutex_;
  rclcpp::Publisher<v13_excavator_ros::msg::Inclinometer>::SharedPtr publisher_;
  rclcpp::Publisher<v13_excavator_ros::msg::InclinometerGroup>::SharedPtr group_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace v13_excavator_ros::utils
