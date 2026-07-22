#include "v13_excavator_ros/utils/protocol_helpers.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace v13_excavator_ros::utils
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

std::uint16_t read_be_u16(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  return static_cast<std::uint16_t>((static_cast<std::uint16_t>(data.at(offset)) << 8U) |
    static_cast<std::uint16_t>(data.at(offset + 1U)));
}

std::int16_t read_be_i16(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  return static_cast<std::int16_t>(read_be_u16(data, offset));
}

std::int16_t read_le_i16(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  return static_cast<std::int16_t>(
    static_cast<std::uint16_t>(data.at(offset)) |
    (static_cast<std::uint16_t>(data.at(offset + 1U)) << 8U));
}

std::uint16_t read_le_u16(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  return static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(data.at(offset)) |
    (static_cast<std::uint16_t>(data.at(offset + 1U)) << 8U));
}

std::uint32_t read_le_u32(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  return static_cast<std::uint32_t>(data.at(offset)) |
         (static_cast<std::uint32_t>(data.at(offset + 1U)) << 8U) |
         (static_cast<std::uint32_t>(data.at(offset + 2U)) << 16U) |
         (static_cast<std::uint32_t>(data.at(offset + 3U)) << 24U);
}

std::uint64_t read_le_u64(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  std::uint64_t value = 0;
  for (std::size_t i = 0; i < 8; ++i) {
    value |= (static_cast<std::uint64_t>(data.at(offset + i)) << (8U * i));
  }
  return value;
}

std::array<std::uint8_t, 2> to_u16_bytes(int value)
{
  if (value < 0 || value > 5000) {
    throw std::out_of_range("analog channel must be within 0..5000 mV");
  }
  return {
    static_cast<std::uint8_t>((value >> 8) & 0xFF),
    static_cast<std::uint8_t>(value & 0xFF)
  };
}

std::uint32_t crc32_stm32(const std::vector<std::uint8_t> & data)
{
  std::uint32_t crc = 0xFFFFFFFFU;
  for (std::size_t i = 0; i < data.size(); i += 4U) {
    std::uint32_t word = 0;
    for (std::size_t j = 0; j < 4U && (i + j) < data.size(); ++j) {
      word |= static_cast<std::uint32_t>(data[i + j]) << (8U * (3U - j));
    }
    crc ^= word;
    for (int bit = 0; bit < 32; ++bit) {
      if ((crc & 0x80000000U) != 0U) {
        crc = (crc << 1U) ^ 0x04C11DB7U;
      } else {
        crc <<= 1U;
      }
    }
  }
  return crc;
}
}  // namespace

std::array<std::uint8_t, 4> encode_can_id(std::uint32_t can_id, bool is_extended)
{
  std::uint32_t shifted = is_extended ? ((can_id << 3U) & 0xFFFFFFFFU) : ((can_id << 21U) & 0xFFFFFFFFU);
  std::array<std::uint8_t, 4> encoded{
    static_cast<std::uint8_t>((shifted >> 24U) & 0xFF),
    static_cast<std::uint8_t>((shifted >> 16U) & 0xFF),
    static_cast<std::uint8_t>((shifted >> 8U) & 0xFF),
    static_cast<std::uint8_t>(shifted & 0xFF)};
  if (is_extended) {
    encoded[3] = static_cast<std::uint8_t>(encoded[3] | 0x02U);
  }
  return encoded;
}

std::vector<std::uint8_t> build_analog_payload(int ch1_mv, int ch2_mv, int ch3_mv)
{
  const auto ch1 = to_u16_bytes(ch1_mv);
  const auto ch2 = to_u16_bytes(ch2_mv);
  const auto ch3 = to_u16_bytes(ch3_mv);
  return {ch1[0], ch1[1], ch2[0], ch2[1], ch3[0], ch3[1], 0x00, 0x00};
}

std::uint16_t modbus_crc16(const std::vector<std::uint8_t> & data)
{
  std::uint16_t crc = 0xFFFF;
  for (const auto byte : data) {
    crc ^= static_cast<std::uint16_t>(byte);
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x0001U) != 0U) {
        crc = static_cast<std::uint16_t>((crc >> 1U) ^ 0xA001U);
      } else {
        crc = static_cast<std::uint16_t>(crc >> 1U);
      }
    }
  }
  return static_cast<std::uint16_t>(((crc & 0x00FFU) << 8U) | ((crc & 0xFF00U) >> 8U));
}

std::vector<std::uint8_t> build_inclinometer_read_request(std::uint8_t address)
{
  std::vector<std::uint8_t> request{address, 0x03, 0x00, 0x3D, 0x00, 0x0C};
  const auto crc = modbus_crc16(request);
  request.push_back(static_cast<std::uint8_t>((crc >> 8U) & 0xFF));
  request.push_back(static_cast<std::uint8_t>(crc & 0xFF));
  return request;
}

InclinometerDecoded decode_inclinometer_packet(const std::vector<std::uint8_t> & packet)
{
  if (packet.size() < 27U || packet.at(2) != 24U) {
    throw std::runtime_error("invalid inclinometer packet");
  }

  InclinometerDecoded decoded;
  decoded.roll_deg = static_cast<double>(read_be_i16(packet, 21U)) / 32768.0 * 180.0;
  decoded.pitch_deg = static_cast<double>(read_be_i16(packet, 23U)) / 32768.0 * 180.0;
  decoded.yaw_deg = static_cast<double>(read_be_i16(packet, 25U)) / 32768.0 * 180.0;
  return decoded;
}

bool is_lidar_imu_packet(const std::vector<std::uint8_t> & packet)
{
  return packet.size() >= 33U && packet[0] == 0xFA && packet[1] == 0x88;
}

bool is_lidar_pointcloud_packet(const std::vector<std::uint8_t> & packet)
{
  return packet.size() >= 36U && (packet[0] == 0x00 || packet[0] == 0x01);
}

LidarImuDecoded decode_lidar_imu_packet(const std::vector<std::uint8_t> & packet)
{
  if (!is_lidar_imu_packet(packet)) {
    throw std::runtime_error("invalid lidar imu packet");
  }

  constexpr std::size_t offset = 9U;
  LidarImuDecoded imu;
  imu.accel_x = static_cast<double>(read_le_i16(packet, offset + 1U)) * 4.0 / 65536.0;
  imu.accel_y = static_cast<double>(read_le_i16(packet, offset + 3U)) * 4.0 / 65536.0;
  imu.accel_z = static_cast<double>(read_le_i16(packet, offset + 5U)) * 4.0 / 65536.0;
  imu.gyro_x = static_cast<double>(read_le_i16(packet, offset + 7U)) * 4000.0 / 65536.0 * kPi / 180.0;
  imu.gyro_y = static_cast<double>(read_le_i16(packet, offset + 9U)) * 4000.0 / 65536.0 * kPi / 180.0;
  imu.gyro_z = static_cast<double>(read_le_i16(packet, offset + 11U)) * 4000.0 / 65536.0 * kPi / 180.0;
  imu.timestamp_ns = read_le_u64(packet, offset + 16U);
  return imu;
}

LidarPointcloudDecoded decode_lidar_pointcloud_packet(const std::vector<std::uint8_t> & packet)
{
  if (!is_lidar_pointcloud_packet(packet)) {
    throw std::runtime_error("invalid lidar pointcloud packet");
  }

  LidarPointcloudDecoded decoded;
  decoded.dot_num = read_le_u16(packet, 5U);
  decoded.data_type = packet.at(10U);

  const std::size_t point_data_offset = 36U;
  if (packet.size() < point_data_offset + static_cast<std::size_t>(decoded.dot_num) * 10U) {
    throw std::runtime_error("incomplete lidar pointcloud packet");
  }

  decoded.points.reserve(decoded.dot_num);
  for (std::size_t i = 0; i < decoded.dot_num; ++i) {
    const auto base = point_data_offset + i * 10U;
    const std::uint32_t word1 = read_le_u32(packet, base);
    const std::uint32_t word2 = read_le_u32(packet, base + 4U);

    const std::uint32_t depth = word1 & 0x00FFFFFFU;
    const std::uint32_t theta_hi = (word1 >> 24U) & 0xFFU;
    const std::uint32_t theta_lo = word2 & 0xFFFU;
    const std::uint32_t phi = (word2 >> 12U) & 0xFFFFFU;

    const std::uint32_t theta = (theta_hi << 12U) | theta_lo;
    const double ang = (90000.0 - static_cast<double>(theta)) * (kPi / 180000.0);
    const double depth_m = static_cast<double>(depth) / 1000.0;
    const double r = depth_m * std::cos(ang);
    const double z = depth_m * std::sin(ang);
    const double phi_ang = static_cast<double>(phi) * (kPi / 180000.0);

    decoded.points.push_back(PointXYZ{
      static_cast<float>(std::cos(phi_ang) * r),
      static_cast<float>(std::sin(phi_ang) * r),
      static_cast<float>(z)});
  }

  return decoded;
}

std::vector<std::uint8_t> build_lidar_start_command()
{
  static std::uint16_t sequence = 1U;
  const std::vector<std::uint8_t> payload{'L', 'S', 'T', 'A', 'R', 'H', 0x00, 0x00};
  std::vector<std::uint8_t> packet{
    0x4C, 0x48,
    0x43, 0x00,
    static_cast<std::uint8_t>(sequence & 0xFF),
    static_cast<std::uint8_t>((sequence >> 8U) & 0xFF),
    0x06, 0x00};
  packet.insert(packet.end(), payload.begin(), payload.end());
  const auto crc = crc32_stm32(packet);
  packet.push_back(static_cast<std::uint8_t>(crc & 0xFF));
  packet.push_back(static_cast<std::uint8_t>((crc >> 8U) & 0xFF));
  packet.push_back(static_cast<std::uint8_t>((crc >> 16U) & 0xFF));
  packet.push_back(static_cast<std::uint8_t>((crc >> 24U) & 0xFF));
  ++sequence;
  return packet;
}
}  // namespace v13_excavator_ros::utils
