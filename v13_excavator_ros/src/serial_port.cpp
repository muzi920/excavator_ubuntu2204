#include "v13_excavator_ros/utils/serial_port.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace v13_excavator_ros::utils
{
SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & device, int baudrate)
{
  close();
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    close();
    return false;
  }

  cfmakeraw(&tty);
  const auto baud = baudrate_to_constant(baudrate);
  cfsetispeed(&tty, static_cast<speed_t>(baud));
  cfsetospeed(&tty, static_cast<speed_t>(baud));
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close();
    return false;
  }

  device_ = device;
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::is_open() const
{
  return fd_ >= 0;
}

std::vector<std::uint8_t> SerialPort::read_available(std::size_t max_bytes) const
{
  std::vector<std::uint8_t> buffer;
  if (fd_ < 0) {
    return buffer;
  }

  int available = 0;
  if (ioctl(fd_, FIONREAD, &available) != 0 || available <= 0) {
    return buffer;
  }

  const auto bytes_to_read = static_cast<std::size_t>(available) < max_bytes ?
    static_cast<std::size_t>(available) : max_bytes;
  buffer.resize(bytes_to_read);
  const auto bytes_read = ::read(fd_, buffer.data(), buffer.size());
  if (bytes_read <= 0) {
    buffer.clear();
    return buffer;
  }
  buffer.resize(static_cast<std::size_t>(bytes_read));
  return buffer;
}

bool SerialPort::write_bytes(const std::vector<std::uint8_t> & data) const
{
  if (fd_ < 0) {
    return false;
  }
  const auto bytes_written = ::write(fd_, data.data(), data.size());
  return bytes_written == static_cast<ssize_t>(data.size());
}

int SerialPort::baudrate_to_constant(int baudrate) const
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:
      return B115200;
  }
}
}  // namespace v13_excavator_ros::utils
