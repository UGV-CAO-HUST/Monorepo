#ifndef UWB_SERIAL_DRIVER__UWB_TYPES_HPP_
#define UWB_SERIAL_DRIVER__UWB_TYPES_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace uwb_serial_driver
{

struct RangeMeasurement
{
  int anchor_id{0};
  double range_m{0.0};
  bool valid{true};
  nlohmann::json quality = nlohmann::json::object();
  nlohmann::json timestamps = nlohmann::json::object();
  std::optional<double> clock_offset;
  nlohmann::json payload;
};

struct UwbFrame
{
  std::optional<int> tag_id;
  std::string device_role{"tag"};
  std::optional<int> device_index;
  std::optional<int> device_radio_id;
  std::optional<int> sequence;
  std::vector<RangeMeasurement> measurements;
  std::string protocol;
  std::optional<int> beacon_sequence;
  std::optional<int> fail_count;
  std::optional<std::uint32_t> mask;
  std::optional<std::uint32_t> device_timestamp_ms;
  nlohmann::json payload;
  nlohmann::json metadata = nlohmann::json::object();
};

}  // namespace uwb_serial_driver

#endif  // UWB_SERIAL_DRIVER__UWB_TYPES_HPP_
