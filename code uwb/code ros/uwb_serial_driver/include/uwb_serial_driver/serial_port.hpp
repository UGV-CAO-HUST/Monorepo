#ifndef UWB_SERIAL_DRIVER__SERIAL_PORT_HPP_
#define UWB_SERIAL_DRIVER__SERIAL_PORT_HPP_

#include <cstddef>
#include <string>

namespace uwb_serial_driver
{

class SerialPort
{
public:
  static constexpr std::size_t kMaxLineBytes = 4096;

  SerialPort(std::string port, int baudrate);
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  bool is_open() const;
  void open();
  bool read_line(std::string & line);
  void close();

private:
  std::string port_;
  int baudrate_;
  int fd_{-1};
  std::string buffer_;
};

}  // namespace uwb_serial_driver

#endif  // UWB_SERIAL_DRIVER__SERIAL_PORT_HPP_
