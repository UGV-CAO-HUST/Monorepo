#ifndef UWB_SERIAL_DRIVER__PARSER_HPP_
#define UWB_SERIAL_DRIVER__PARSER_HPP_

#include <stdexcept>
#include <string>

#include "uwb_serial_driver/uwb_types.hpp"

namespace uwb_serial_driver
{

class ParseError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

UwbFrame parse_line(const std::string & line);

}  // namespace uwb_serial_driver

#endif  // UWB_SERIAL_DRIVER__PARSER_HPP_
