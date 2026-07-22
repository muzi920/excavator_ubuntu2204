#include "v13_excavator_ros/utils/lidar_imu_reader.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "v13_excavator_ros/utils/protocol_helpers.hpp"

namespace v13_excavator_ros::utils
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kCalibrationDuration = 3.0;
}  // namespace

LidarImuReaderNode::LidarImuReaderNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("v13_lidar_imu_reader", options)
{
  listen_port_ = this->declare_parameter<int>("listen_port", 6668);
  lidar_ip_ = this->declare_parameter<std::string>("lidar_ip", "192.168.158.98");
  lidar_port_ = this->declare_parameter<int>("lidar_port", 6543);
  const auto topic_name = this->declare_parameter<std::string>("topic_name", "/v13/lidar/imu");
  const auto status_topic = this->declare_parameter<std::string>("status_topic", "/v13/lidar_imu/status");

  publisher_ = this->create_publisher<v13_excavator_ros::msg::LidarImu>(topic_name, 20);
  status_publisher_ = this->create_publisher<std_msgs::msg::String>(status_topic, 10);

  running_.store(setup_socket());
  calibration_start_ = this->now().seconds();
  if (running_.load()) {
    worker_ = std::thread(&LidarImuReaderNode::receive_loop, this);
  }
}

LidarImuReaderNode::~LidarImuReaderNode()
{
  running_.store(false);
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
  if (worker_.joinable()) {
    worker_.join();
  }
}

bool LidarImuReaderNode::setup_socket()
{
  socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    publish_status("failed to create lidar imu socket");
    return false;
  }

  int reuse = 1;
  setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
#ifdef SO_REUSEPORT
  setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
#endif

  timeval timeout{};
  timeout.tv_sec = 0;
  timeout.tv_usec = 200000;
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(static_cast<uint16_t>(listen_port_));
  local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&local_addr), sizeof(local_addr)) < 0) {
    publish_status("failed to bind lidar imu socket");
    return false;
  }

  sockaddr_in lidar_addr{};
  lidar_addr.sin_family = AF_INET;
  lidar_addr.sin_port = htons(static_cast<uint16_t>(lidar_port_));
  if (inet_pton(AF_INET, lidar_ip_.c_str(), &lidar_addr.sin_addr) != 1) {
    publish_status("invalid lidar imu ip");
    return false;
  }

  const auto start_cmd = build_lidar_start_command();
  for (int i = 0; i < 5; ++i) {
    sendto(socket_fd_, start_cmd.data(), start_cmd.size(), 0,
      reinterpret_cast<sockaddr *>(&lidar_addr), sizeof(lidar_addr));
  }
  return true;
}

void LidarImuReaderNode::receive_loop()
{
  std::vector<std::uint8_t> buffer(65536U);
  while (running_.load()) {
    const auto bytes = recv(socket_fd_, buffer.data(), buffer.size(), 0);
    if (bytes <= 0) {
      continue;
    }

    std::vector<std::uint8_t> packet(buffer.begin(), buffer.begin() + bytes);
    if (!is_lidar_imu_packet(packet)) {
      continue;
    }

    try {
      const auto decoded = decode_lidar_imu_packet(packet);
      v13_excavator_ros::msg::LidarImu msg;
      msg.header.stamp = this->now();
      msg.calibrated = false;
      msg.accel_x = decoded.accel_x;
      msg.accel_y = decoded.accel_y;
      msg.accel_z = decoded.accel_z;
      msg.gyro_x = decoded.gyro_x;
      msg.gyro_y = decoded.gyro_y;
      msg.gyro_z = decoded.gyro_z;
      msg.sensor_timestamp_ns = decoded.timestamp_ns;
      update_calibration(
        decoded.accel_x,
        decoded.accel_y,
        decoded.accel_z,
        decoded.gyro_x,
        decoded.gyro_y,
        decoded.gyro_z,
        decoded.timestamp_ns,
        this->now().seconds(),
        msg);
      publisher_->publish(msg);
    } catch (const std::exception & exc) {
      publish_status(exc.what());
    }
  }
}

void LidarImuReaderNode::update_calibration(
  double accel_x,
  double accel_y,
  double accel_z,
  double gyro_x,
  double gyro_y,
  double gyro_z,
  std::uint64_t sensor_timestamp_ns,
  double host_now_sec,
  v13_excavator_ros::msg::LidarImu & msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (last_host_time_ == 0.0) {
    last_host_time_ = host_now_sec;
    last_sensor_timestamp_ns_ = sensor_timestamp_ns;
    calibration_start_ = host_now_sec;
    msg.swing_deg = 0.0;
    msg.yaw_rate = 0.0;
    msg.calibrated = false;
    return;
  }

  double dt = host_now_sec - last_host_time_;
  if (sensor_timestamp_ns > last_sensor_timestamp_ns_ && last_sensor_timestamp_ns_ != 0U) {
    const double sensor_dt = static_cast<double>(sensor_timestamp_ns - last_sensor_timestamp_ns_) * 1e-9;
    if (sensor_dt > 0.0 && sensor_dt < 0.5) {
      dt = sensor_dt;
    }
  }
  last_host_time_ = host_now_sec;
  last_sensor_timestamp_ns_ = sensor_timestamp_ns;

  if (dt <= 0.0 || dt > 0.5) {
    msg.swing_deg = current_swing_rad_ * 180.0 / kPi;
    msg.yaw_rate = last_w_yaw_;
    msg.calibrated = !calibrating_;
    return;
  }

  if (calibrating_) {
    accel_samples_.push_back({accel_x, accel_y, accel_z});
    gyro_samples_.push_back({gyro_x, gyro_y, gyro_z});
    if (host_now_sec - calibration_start_ >= kCalibrationDuration) {
      double sum_ax = 0.0;
      double sum_ay = 0.0;
      double sum_az = 0.0;
      double sum_gx = 0.0;
      double sum_gy = 0.0;
      double sum_gz = 0.0;
      for (const auto & sample : accel_samples_) {
        sum_ax += sample[0];
        sum_ay += sample[1];
        sum_az += sample[2];
      }
      for (const auto & sample : gyro_samples_) {
        sum_gx += sample[0];
        sum_gy += sample[1];
        sum_gz += sample[2];
      }
      const auto count = static_cast<double>(std::max<std::size_t>(1U, accel_samples_.size()));
      gyro_bias_ = {sum_gx / count, sum_gy / count, sum_gz / count};
      std::array<double, 3> avg_accel{sum_ax / count, sum_ay / count, sum_az / count};
      const auto norm = std::sqrt(
        avg_accel[0] * avg_accel[0] + avg_accel[1] * avg_accel[1] + avg_accel[2] * avg_accel[2]);
      if (norm > 1e-6) {
        up_vector_ = {avg_accel[0] / norm, avg_accel[1] / norm, avg_accel[2] / norm};
      }
      calibrating_ = false;
    }
    msg.swing_deg = 0.0;
    msg.yaw_rate = 0.0;
    msg.calibrated = false;
    return;
  }

  const double w_x = gyro_x - gyro_bias_[0];
  const double w_y = gyro_y - gyro_bias_[1];
  const double w_z = gyro_z - gyro_bias_[2];
  double w_yaw = -(w_x * up_vector_[0] + w_y * up_vector_[1] + w_z * up_vector_[2]);
  if (std::fabs(w_yaw) < 0.002) {
    w_yaw = 0.0;
  }

  current_swing_rad_ += (w_yaw + last_w_yaw_) * 0.5 * dt;
  last_w_yaw_ = w_yaw;

  double swing_deg = current_swing_rad_ * 180.0 / kPi;
  while (swing_deg > 180.0) {
    swing_deg -= 360.0;
  }
  while (swing_deg < -180.0) {
    swing_deg += 360.0;
  }

  msg.swing_deg = swing_deg;
  msg.yaw_rate = w_yaw;
  msg.calibrated = true;
}

void LidarImuReaderNode::publish_status(const std::string & text)
{
  std_msgs::msg::String msg;
  msg.data = text;
  status_publisher_->publish(msg);
}
}  // namespace v13_excavator_ros::utils

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<v13_excavator_ros::utils::LidarImuReaderNode>());
  rclcpp::shutdown();
  return 0;
}
