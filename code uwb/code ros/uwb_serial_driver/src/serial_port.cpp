#include "uwb_serial_driver/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace uwb_serial_driver
{
namespace
{

speed_t baud_constant(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
    default:
      throw std::invalid_argument("unsupported baudrate: " + std::to_string(baudrate));
  }
}

std::runtime_error system_error(const std::string & operation)
{
  return std::runtime_error(operation + ": " + std::strerror(errno));
}

}  // namespace

SerialPort::SerialPort(std::string port, int baudrate)
: port_(std::move(port)), baudrate_(baudrate)
{
}

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::is_open() const
{
  return fd_ >= 0;
}

void SerialPort::open()
{
  if (is_open()) {
    return;
  }

  const speed_t speed = baud_constant(baudrate_);
  fd_ = ::open(port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw system_error("cannot open serial port " + port_);
  }

  termios settings{};
  if (tcgetattr(fd_, &settings) != 0) {
    const auto error = system_error("cannot read serial settings");
    close();
    throw error;
  }

  cfmakeraw(&settings);
  cfsetispeed(&settings, speed);
  cfsetospeed(&settings, speed);
  settings.c_cflag |= CLOCAL | CREAD;
  settings.c_cflag &= ~CSTOPB;
  settings.c_cflag &= ~PARENB;
  settings.c_cflag &= ~CSIZE;
  settings.c_cflag |= CS8;

  if (tcsetattr(fd_, TCSANOW, &settings) != 0) {
    const auto error = system_error("cannot configure serial port");
    close();
    throw error;
  }

  tcflush(fd_, TCIFLUSH);
  buffer_.clear();
}

bool SerialPort::read_line(std::string & line)
{
  if (!is_open()) {
    throw std::runtime_error("serial port is not open");
  }

  while (true) {
    const auto newline = buffer_.find('\n');
    if (newline != std::string::npos) {
      if (newline > kMaxLineBytes) {
        buffer_.erase(0, newline + 1);
        throw std::runtime_error("serial line exceeded 4096 bytes");
      }
      line.assign(buffer_, 0, newline + 1);
      buffer_.erase(0, newline + 1);
      return true;
    }

    char chunk[512];
    const ssize_t count = ::read(fd_, chunk, sizeof(chunk));
    if (count > 0) {
      buffer_.append(chunk, static_cast<std::size_t>(count));
      if (buffer_.size() > kMaxLineBytes) {
        buffer_.clear();
        throw std::runtime_error("serial line exceeded 4096 bytes");
      }
      continue;
    }
    if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
      return false;
    }
    if (errno == EINTR) {
      continue;
    }
    throw system_error("serial read failed");
  }
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  buffer_.clear();
}

}  // namespace uwb_serial_driver
