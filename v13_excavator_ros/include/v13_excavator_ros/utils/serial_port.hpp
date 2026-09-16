#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace v13_excavator_ros::utils
{
class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  bool open(const std::string & device, int baudrate);
  void close();
  bool is_open() const;
  std::vector<std::uint8_t> read_available(std::size_t max_bytes = 512U) const;
  bool write_bytes(const std::vector<std::uint8_t> & data) const;

private:
  int baudrate_to_constant(int baudrate) const;

  int fd_{-1};
  std::string device_;
};
}  // namespace v13_excavator_ros::utils
