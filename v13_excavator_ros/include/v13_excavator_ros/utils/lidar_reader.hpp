#pragma once

#include <atomic>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

namespace v13_excavator_ros::utils
{
class LidarReaderNode : public rclcpp::Node
{
public:
  explicit LidarReaderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LidarReaderNode() override;

private:
  bool setup_socket();
  void receive_loop();
  void publish_status(const std::string & text);

  int socket_fd_{-1};
  int listen_port_{6668};
  int lidar_port_{6543};
  std::string lidar_ip_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
};
}  // namespace v13_excavator_ros::utils
