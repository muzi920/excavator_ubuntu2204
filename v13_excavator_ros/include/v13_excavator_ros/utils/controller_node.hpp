#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "v13_excavator_ros/msg/controller_command.hpp"
#include "v13_excavator_ros/utils/serial_port.hpp"

namespace v13_excavator_ros::utils
{
class ControllerNode : public rclcpp::Node
{
public:
  explicit ControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void command_callback(const v13_excavator_ros::msg::ControllerCommand::SharedPtr msg);
  bool send_can_frame(std::uint32_t can_id, const std::vector<std::uint8_t> & payload, std::uint8_t func_code = 0x00);
  void stop_all();
  void publish_status(const std::string & text);

  int baudrate_{115200};
  std::string device_;
  SerialPort serial_;
  rclcpp::Subscription<v13_excavator_ros::msg::ControllerCommand>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
};
}  // namespace v13_excavator_ros::utils
