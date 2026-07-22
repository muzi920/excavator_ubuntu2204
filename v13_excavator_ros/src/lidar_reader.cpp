#include "v13_excavator_ros/utils/lidar_reader.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <vector>

#include "sensor_msgs/msg/point_field.hpp"
#include "v13_excavator_ros/utils/protocol_helpers.hpp"

namespace v13_excavator_ros::utils
{
LidarReaderNode::LidarReaderNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("v13_lidar_reader", options)
{
  listen_port_ = this->declare_parameter<int>("listen_port", 6668);
  lidar_ip_ = this->declare_parameter<std::string>("lidar_ip", "192.168.158.98");
  lidar_port_ = this->declare_parameter<int>("lidar_port", 6543);
  const auto topic_name = this->declare_parameter<std::string>("topic_name", "/v13/lidar/points");
  const auto status_topic = this->declare_parameter<std::string>("status_topic", "/v13/lidar/status");

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 10);
  status_publisher_ = this->create_publisher<std_msgs::msg::String>(status_topic, 10);

  running_.store(setup_socket());
  if (running_.load()) {
    worker_ = std::thread(&LidarReaderNode::receive_loop, this);
  }
}

LidarReaderNode::~LidarReaderNode()
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

bool LidarReaderNode::setup_socket()
{
  socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    publish_status("failed to create lidar socket");
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
    publish_status("failed to bind lidar socket");
    return false;
  }

  sockaddr_in lidar_addr{};
  lidar_addr.sin_family = AF_INET;
  lidar_addr.sin_port = htons(static_cast<uint16_t>(lidar_port_));
  if (inet_pton(AF_INET, lidar_ip_.c_str(), &lidar_addr.sin_addr) != 1) {
    publish_status("invalid lidar ip");
    return false;
  }

  const auto start_cmd = build_lidar_start_command();
  for (int i = 0; i < 5; ++i) {
    sendto(socket_fd_, start_cmd.data(), start_cmd.size(), 0, reinterpret_cast<sockaddr *>(&lidar_addr), sizeof(lidar_addr));
  }
  return true;
}

void LidarReaderNode::receive_loop()
{
  std::vector<std::uint8_t> buffer(65536U);
  while (running_.load()) {
    const auto bytes = recv(socket_fd_, buffer.data(), buffer.size(), 0);
    if (bytes <= 0) {
      continue;
    }

    std::vector<std::uint8_t> packet(buffer.begin(), buffer.begin() + bytes);
    if (!is_lidar_pointcloud_packet(packet)) {
      continue;
    }

    try {
      const auto decoded = decode_lidar_pointcloud_packet(packet);
      sensor_msgs::msg::PointCloud2 msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = "lidar_link";
      msg.height = 1;
      msg.width = static_cast<std::uint32_t>(decoded.points.size());
      msg.is_bigendian = false;
      msg.is_dense = true;
      msg.point_step = 12;
      msg.row_step = msg.point_step * msg.width;
      msg.fields.resize(3);
      msg.fields[0].name = "x";
      msg.fields[0].offset = 0;
      msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
      msg.fields[0].count = 1;
      msg.fields[1].name = "y";
      msg.fields[1].offset = 4;
      msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
      msg.fields[1].count = 1;
      msg.fields[2].name = "z";
      msg.fields[2].offset = 8;
      msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
      msg.fields[2].count = 1;
      msg.data.resize(decoded.points.size() * 12U);
      for (std::size_t i = 0; i < decoded.points.size(); ++i) {
        std::memcpy(msg.data.data() + i * 12U + 0U, &decoded.points[i].x, sizeof(float));
        std::memcpy(msg.data.data() + i * 12U + 4U, &decoded.points[i].y, sizeof(float));
        std::memcpy(msg.data.data() + i * 12U + 8U, &decoded.points[i].z, sizeof(float));
      }
      publisher_->publish(msg);
    } catch (const std::exception & exc) {
      publish_status(exc.what());
    }
  }
}

void LidarReaderNode::publish_status(const std::string & text)
{
  std_msgs::msg::String msg;
  msg.data = text;
  status_publisher_->publish(msg);
}
}  // namespace v13_excavator_ros::utils

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<v13_excavator_ros::utils::LidarReaderNode>());
  rclcpp::shutdown();
  return 0;
}
