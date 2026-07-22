#include <cmath>

#include <gtest/gtest.h>

#include "v13_excavator_ros/utils/protocol_helpers.hpp"

namespace v13_excavator_ros::utils
{
constexpr double kPi = 3.14159265358979323846;

TEST(ProtocolHelpersTest, EncodesStandardCanId)
{
  auto encoded = encode_can_id(0x0103, false);
  EXPECT_EQ(encoded[0], 0x20);
  EXPECT_EQ(encoded[1], 0x60);
  EXPECT_EQ(encoded[2], 0x00);
  EXPECT_EQ(encoded[3], 0x00);
}

TEST(ProtocolHelpersTest, BuildsAnalogPayload)
{
  auto frame = build_analog_payload(1000, 2000, 3000);
  ASSERT_EQ(frame.size(), 8U);
  EXPECT_EQ(frame[0], 0x03);
  EXPECT_EQ(frame[1], 0xE8);
  EXPECT_EQ(frame[2], 0x07);
  EXPECT_EQ(frame[3], 0xD0);
  EXPECT_EQ(frame[4], 0x0B);
  EXPECT_EQ(frame[5], 0xB8);
}

TEST(ProtocolHelpersTest, ComputesModbusCrc)
{
  std::vector<uint8_t> request{0x50, 0x03, 0x00, 0x3D, 0x00, 0x0C};
  EXPECT_EQ(modbus_crc16(request), 0xD982);
}

TEST(ProtocolHelpersTest, DecodesInclinometerAngles)
{
  std::vector<uint8_t> packet{
    0x50, 0x03, 0x18,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x20, 0x00, 0x10, 0x00, 0xF0, 0x00
  };

  auto decoded = decode_inclinometer_packet(packet);
  EXPECT_NEAR(decoded.roll_deg, 45.0, 0.05);
  EXPECT_NEAR(decoded.pitch_deg, 22.5, 0.05);
  EXPECT_NEAR(decoded.yaw_deg, -22.5, 0.05);
}

TEST(ProtocolHelpersTest, DecodesLidarImuPacket)
{
  std::vector<uint8_t> packet(33, 0);
  packet[0] = 0xFA;
  packet[1] = 0x88;

  const int offset = 9;
  packet[offset + 0] = 0x01;

  auto put_i16 = [&](int start, int16_t value) {
    packet[start] = static_cast<uint8_t>(value & 0xFF);
    packet[start + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  };

  put_i16(offset + 1, 0x1000);
  put_i16(offset + 3, 0x2000);
  put_i16(offset + 5, 0x3000);
  put_i16(offset + 7, 0x1000);
  put_i16(offset + 9, 0x0000);
  put_i16(offset + 11, static_cast<int16_t>(0xF000));
  packet[offset + 13] = 0x00;
  packet[offset + 14] = 0x00;
  packet[offset + 15] = 0x00;
  packet[offset + 16] = 0x01;
  packet[offset + 17] = 0x00;
  packet[offset + 18] = 0x00;
  packet[offset + 19] = 0x00;
  packet[offset + 20] = 0x00;
  packet[offset + 21] = 0x00;
  packet[offset + 22] = 0x00;
  packet[offset + 23] = 0x00;

  auto imu = decode_lidar_imu_packet(packet);
  EXPECT_NEAR(imu.accel_x, 0.25, 1e-6);
  EXPECT_NEAR(imu.accel_y, 0.5, 1e-6);
  EXPECT_NEAR(imu.accel_z, 0.75, 1e-6);
  EXPECT_NEAR(imu.gyro_x, 4000.0 / 16.0 * kPi / 180.0, 1e-6);
  EXPECT_NEAR(imu.gyro_z, -4000.0 / 16.0 * kPi / 180.0, 1e-6);
  EXPECT_EQ(imu.timestamp_ns, 1U);
}
}  // namespace v13_excavator_ros::utils
