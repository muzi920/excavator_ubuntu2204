#include "v13_excavator_ros/utils/inclinometer_reader.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>

#include "v13_excavator_ros/utils/protocol_helpers.hpp"

namespace v13_excavator_ros::utils
{
namespace
{
rclcpp::Time to_ros_time(double timestamp_sec)
{
  return rclcpp::Time(static_cast<std::int64_t>(timestamp_sec * 1e9));
}
}  // namespace

InclinometerReaderNode::InclinometerReaderNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("v13_inclinometer_reader", options)
{
  port_names_ = this->declare_parameter<std::vector<std::string>>(
    "serial_ports",
    {"/dev/ttyUSB_Sensor1", "/dev/ttyUSB_Sensor2", "/dev/ttyUSB_Sensor3", "/dev/ttyUSB_Sensor4"});
  sensor_ids_ = this->declare_parameter<std::vector<int64_t>>(
    "sensor_ids", {0x50, 0x51, 0x52, 0x53});
  sensor_names_ = this->declare_parameter<std::vector<std::string>>(
    "sensor_names", {"bucket", "arm", "boom", "swing"});
  baudrate_ = this->declare_parameter<int>("baudrate", 230400);
  max_group_skew_sec_ = this->declare_parameter<double>("max_group_skew_sec", 0.1);
  preprocessor_ = JointPreprocessor(static_cast<std::size_t>(
    this->declare_parameter<int>("init_sample_count", 20)));
  const auto topic_name = this->declare_parameter<std::string>("topic_name", "/v13/inclinometer/raw");
  const auto group_topic_name = this->declare_parameter<std::string>(
    "group_topic_name", "/v13/inclinometer/group");
  const auto status_topic = this->declare_parameter<std::string>("status_topic", "/v13/inclinometer/status");

  publisher_ = this->create_publisher<v13_excavator_ros::msg::Inclinometer>(topic_name, 20);
  group_publisher_ = this->create_publisher<v13_excavator_ros::msg::InclinometerGroup>(group_topic_name, 20);
  status_publisher_ = this->create_publisher<std_msgs::msg::String>(status_topic, 10);

  serial_ports_.reserve(port_names_.size());
  for (std::size_t i = 0; i < port_names_.size(); ++i) {
    serial_ports_.push_back(std::make_unique<SerialPort>());
  }

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&InclinometerReaderNode::poll_once, this));
}

void InclinometerReaderNode::poll_once()
{
  const auto sensor_count = std::min(
    {port_names_.size(), sensor_ids_.size(), sensor_names_.size(), serial_ports_.size()});
  for (std::size_t i = 0; i < sensor_count; ++i) {
    auto & serial = serial_ports_[i];
    if (!serial->is_open() && !serial->open(port_names_[i], baudrate_)) {
      publish_status("failed to open " + port_names_[i]);
      continue;
    }

    const auto request = build_inclinometer_read_request(static_cast<std::uint8_t>(sensor_ids_[i]));
    if (!serial->write_bytes(request)) {
      publish_status("failed to write request to " + port_names_[i]);
      continue;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    const auto response = serial->read_available(256U);
    if (response.size() < 27U) {
      continue;
    }

    auto it = std::find(response.begin(), response.end(), static_cast<std::uint8_t>(sensor_ids_[i]));
    if (it == response.end()) {
      continue;
    }

    std::vector<std::uint8_t> packet(it, response.end());
    if (packet.size() < 27U || packet[1] != 0x03 || packet[2] != 24U) {
      continue;
    }
    packet.resize(27U);

    try {
      const auto decoded = decode_inclinometer_packet(packet);
      const double timestamp_sec = this->now().seconds();

      v13_excavator_ros::msg::Inclinometer msg;
      msg.header.stamp = to_ros_time(timestamp_sec);
      msg.sensor_name = sensor_names_[i];
      msg.sensor_id = static_cast<std::uint8_t>(sensor_ids_[i]);
      msg.roll_deg = decoded.roll_deg;
      msg.pitch_deg = decoded.pitch_deg;
      msg.yaw_deg = decoded.yaw_deg;
      publisher_->publish(msg);

      {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        // Align with v11: use the sensor's roll channel as the excavator pitch angle.
        latest_pitch_deg_[i] = decoded.roll_deg;
        latest_timestamp_sec_[i] = timestamp_sec;
        have_pitch_[i] = true;
        maybe_publish_group_locked(timestamp_sec);
      }
    } catch (const std::exception & exc) {
      publish_status(exc.what());
    }
  }
}

void InclinometerReaderNode::maybe_publish_group_locked(double timestamp_sec)
{
  if (!std::all_of(have_pitch_.begin(), have_pitch_.end(), [](bool value) {return value;})) {
    return;
  }

  const auto minmax_timestamp = std::minmax_element(
    latest_timestamp_sec_.begin(), latest_timestamp_sec_.end());
  if ((*minmax_timestamp.second - *minmax_timestamp.first) > max_group_skew_sec_) {
    return;
  }

  JointPitchSnapshot snapshot;
  snapshot.bucket_pitch_deg = latest_pitch_deg_[0];
  snapshot.arm_pitch_deg = latest_pitch_deg_[1];
  snapshot.boom_pitch_deg = latest_pitch_deg_[2];
  snapshot.swing_pitch_deg = latest_pitch_deg_[3];

  const auto maybe_group = preprocessor_.update(snapshot, timestamp_sec);
  if (!maybe_group.has_value()) {
    return;
  }

  v13_excavator_ros::msg::InclinometerGroup group_msg;
  group_msg.header.stamp = to_ros_time(maybe_group->timestamp_sec);
  group_msg.initialized = maybe_group->initialized;
  group_msg.bucket_pitch_deg = maybe_group->bucket_pitch_deg;
  group_msg.arm_pitch_deg = maybe_group->arm_pitch_deg;
  group_msg.boom_pitch_deg = maybe_group->boom_pitch_deg;
  group_msg.swing_pitch_deg = maybe_group->swing_pitch_deg;
  group_msg.bucket_arm_deg = maybe_group->bucket_arm_deg;
  group_msg.arm_boom_deg = maybe_group->arm_boom_deg;
  group_msg.boom_swing_deg = maybe_group->boom_swing_deg;
  group_publisher_->publish(group_msg);
}

void InclinometerReaderNode::publish_status(const std::string & text)
{
  std_msgs::msg::String msg;
  msg.data = text;
  status_publisher_->publish(msg);
}
}  // namespace v13_excavator_ros::utils

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<v13_excavator_ros::utils::InclinometerReaderNode>());
  rclcpp::shutdown();
  return 0;
}
