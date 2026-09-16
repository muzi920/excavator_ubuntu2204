#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "v13_excavator_ros/msg/lidar_imu.hpp"

namespace v13_excavator_ros::utils
{
class LidarImuReaderNode : public rclcpp::Node
{
public:
  explicit LidarImuReaderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LidarImuReaderNode() override;

private:
  bool setup_socket();
  void receive_loop();
  void publish_status(const std::string & text);
  void update_calibration(
    double accel_x,
    double accel_y,
    double accel_z,
    double gyro_x,
    double gyro_y,
    double gyro_z,
    std::uint64_t sensor_timestamp_ns,
    double host_now_sec,
    v13_excavator_ros::msg::LidarImu & msg);

  int socket_fd_{-1};
  int listen_port_{6668};
  int lidar_port_{6543};
  std::string lidar_ip_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  rclcpp::Publisher<v13_excavator_ros::msg::LidarImu>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;

  bool calibrating_{true};
  double calibration_start_{0.0};
  double last_host_time_{0.0};
  std::uint64_t last_sensor_timestamp_ns_{0U};
  double last_w_yaw_{0.0};
  double current_swing_rad_{0.0};
  std::array<double, 3> gyro_bias_{0.0, 0.0, 0.0};
  std::array<double, 3> up_vector_{0.0, 0.0, 1.0};
  std::vector<std::array<double, 3>> accel_samples_;
  std::vector<std::array<double, 3>> gyro_samples_;
  std::mutex state_mutex_;
};
}  // namespace v13_excavator_ros::utils
