#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace v13_excavator_ros::utils
{
struct InclinometerDecoded
{
  double roll_deg{0.0};
  double pitch_deg{0.0};
  double yaw_deg{0.0};
};

struct LidarImuDecoded
{
  double accel_x{0.0};
  double accel_y{0.0};
  double accel_z{0.0};
  double gyro_x{0.0};
  double gyro_y{0.0};
  double gyro_z{0.0};
  std::uint64_t timestamp_ns{0};
};

struct PointXYZ
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

struct LidarPointcloudDecoded
{
  std::uint16_t dot_num{0};
  std::uint8_t data_type{0};
  std::vector<PointXYZ> points;
};

std::array<std::uint8_t, 4> encode_can_id(std::uint32_t can_id, bool is_extended);
std::vector<std::uint8_t> build_analog_payload(int ch1_mv, int ch2_mv, int ch3_mv);
std::uint16_t modbus_crc16(const std::vector<std::uint8_t> & data);
std::vector<std::uint8_t> build_inclinometer_read_request(std::uint8_t address);
InclinometerDecoded decode_inclinometer_packet(const std::vector<std::uint8_t> & packet);
bool is_lidar_imu_packet(const std::vector<std::uint8_t> & packet);
bool is_lidar_pointcloud_packet(const std::vector<std::uint8_t> & packet);
LidarImuDecoded decode_lidar_imu_packet(const std::vector<std::uint8_t> & packet);
LidarPointcloudDecoded decode_lidar_pointcloud_packet(const std::vector<std::uint8_t> & packet);
std::vector<std::uint8_t> build_lidar_start_command();
}  // namespace v13_excavator_ros::utils
