#include "v13_excavator_ros/utils/controller_node.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <vector>

#include "v13_excavator_ros/utils/protocol_helpers.hpp"

namespace v13_excavator_ros::utils
{
namespace
{
constexpr std::uint32_t kArmSwingId = 0x0101;
constexpr std::uint32_t kBoomBucketId = 0x0102;
constexpr std::uint32_t kChassisId = 0x0103;
constexpr std::uint32_t kAnalogId = 0x0104;
}  // namespace

ControllerNode::ControllerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("v13_controller_node", options)
{
  device_ = this->declare_parameter<std::string>("device", "/dev/ttyUSB_Controller");
  baudrate_ = this->declare_parameter<int>("baudrate", 115200);
  const auto cmd_topic = this->declare_parameter<std::string>("command_topic", "/v13/controller/cmd");
  const auto status_topic = this->declare_parameter<std::string>("status_topic", "/v13/controller/status");

  status_publisher_ = this->create_publisher<std_msgs::msg::String>(status_topic, 10);
  subscription_ = this->create_subscription<v13_excavator_ros::msg::ControllerCommand>(
    cmd_topic, 10, std::bind(&ControllerNode::command_callback, this, std::placeholders::_1));

  if (!serial_.open(device_, baudrate_)) {
    publish_status("controller serial open failed");
  } else {
    send_can_frame(0x0303, std::vector<std::uint8_t>(8U, 0x00));
    publish_status("controller connected");
  }
}

void ControllerNode::command_callback(const v13_excavator_ros::msg::ControllerCommand::SharedPtr msg)
{
  static const std::map<std::string, std::pair<std::uint32_t, std::uint8_t>> command_map{
    {"left_backward", {kChassisId, 0x01}},
    {"left_forward", {kChassisId, 0x02}},
    {"right_forward", {kChassisId, 0x04}},
    {"right_backward", {kChassisId, 0x08}},
    {"forward", {kChassisId, 0x06}},
    {"backward", {kChassisId, 0x09}},
    {"turn_left", {kChassisId, 0x0A}},
    {"turn_right", {kChassisId, 0x05}},
    {"boom_down", {kBoomBucketId, 0x01}},
    {"boom_up", {kBoomBucketId, 0x02}},
    {"bucket_in", {kBoomBucketId, 0x04}},
    {"bucket_out", {kBoomBucketId, 0x08}},
    {"arm_pull", {kArmSwingId, 0x01}},
    {"arm_push", {kArmSwingId, 0x02}},
    {"swing_right", {kArmSwingId, 0x04}},
    {"swing_left", {kArmSwingId, 0x08}},
    {"stop", {kChassisId, 0x00}}};

  if (msg->emergency_stop || msg->motion_name == "stop") {
    stop_all();
    publish_status("controller stop_all");
    return;
  }

  try {
    send_can_frame(kAnalogId, build_analog_payload(msg->ch1_mv, msg->ch2_mv, msg->ch3_mv));
  } catch (const std::exception & exc) {
    publish_status(exc.what());
    return;
  }

  const auto iter = command_map.find(msg->motion_name);
  if (iter == command_map.end()) {
    publish_status("unknown motion: " + msg->motion_name);
    return;
  }

  std::vector<std::uint8_t> payload(8U, 0x00);
  payload[0] = iter->second.second;
  if (!send_can_frame(iter->second.first, payload)) {
    publish_status("controller send failed");
  } else {
    publish_status("controller executed: " + msg->motion_name);
  }
}

bool ControllerNode::send_can_frame(std::uint32_t can_id, const std::vector<std::uint8_t> & payload, std::uint8_t func_code)
{
  std::vector<std::uint8_t> frame(13U, 0x00);
  const auto encoded_id = encode_can_id(can_id, false);
  frame[0] = encoded_id[0];
  frame[1] = encoded_id[1];
  frame[2] = encoded_id[2];
  frame[3] = encoded_id[3];
  for (std::size_t i = 0; i < payload.size() && i < 8U; ++i) {
    frame[4U + i] = payload[i];
  }
  frame[12] = func_code;
  return serial_.write_bytes(frame);
}

void ControllerNode::stop_all()
{
  send_can_frame(kArmSwingId, std::vector<std::uint8_t>(8U, 0x00));
  send_can_frame(kBoomBucketId, std::vector<std::uint8_t>(8U, 0x00));
  send_can_frame(kChassisId, std::vector<std::uint8_t>(8U, 0x00));
  send_can_frame(kAnalogId, build_analog_payload(0, 0, 0));
}

void ControllerNode::publish_status(const std::string & text)
{
  std_msgs::msg::String msg;
  msg.data = text;
  status_publisher_->publish(msg);
}
}  // namespace v13_excavator_ros::utils

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<v13_excavator_ros::utils::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
