#include "uwb_serial_driver/parser.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace uwb_serial_driver
{
namespace
{

using nlohmann::json;

const std::regex kLegacyPattern(
  R"(^T([0-9]+),mask:([0-9a-fA-F]+),seq:([0-9]+)(?:,beacon:([0-9]+))?(?:,fail:([0-9]+))?,(?:range_m|range):\(([^)]*)\),ancid:\(([^)]*)\)(?:,device_role:(tag|anchor),device_index:([0-9]+),device_radio_id:([0-9]+))?$)");

std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

template<typename T>
std::vector<T> parse_values(const std::string & value, char delimiter)
{
  std::vector<T> result;
  const std::string cleaned = trim(value);
  if (cleaned.empty()) {
    return result;
  }

  std::stringstream stream(cleaned);
  std::string item;
  while (std::getline(stream, item, delimiter)) {
    item = trim(item);
    if (item.empty()) {
      throw ParseError("empty item in separated values");
    }

    std::stringstream item_stream(item);
    T parsed{};
    item_stream >> parsed;
    if (!item_stream || item_stream.peek() != std::char_traits<char>::eof()) {
      throw ParseError("invalid number in comma-separated field");
    }
    result.push_back(parsed);
  }
  return result;
}

std::optional<int> optional_int(const json & value)
{
  if (value.is_null()) {
    return std::nullopt;
  }
  if (value.is_number_unsigned()) {
    const auto number = value.get<std::uint64_t>();
    if (number > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
      throw ParseError("integer field is out of range");
    }
    return static_cast<int>(number);
  }
  if (value.is_number_integer()) {
    const auto number = value.get<std::int64_t>();
    if (number < std::numeric_limits<int>::min() ||
      number > std::numeric_limits<int>::max())
    {
      throw ParseError("integer field is out of range");
    }
    return static_cast<int>(number);
  }
  throw ParseError("expected integer field");
}

void validate_device_identity(const UwbFrame & frame)
{
  if (frame.device_index && *frame.device_index < 0) {
    throw ParseError("device_index must be non-negative");
  }
  if (frame.device_radio_id &&
    (*frame.device_radio_id < 0 || *frame.device_radio_id > 0xFFFF))
  {
    throw ParseError("device_radio_id must fit in 16 bits");
  }
  if (frame.device_role == "anchor" && frame.device_index) {
    if (*frame.device_index > 7) {
      throw ParseError("anchor device_index must be in range 0..7");
    }
    if (frame.device_radio_id && *frame.device_radio_id != 0xA0 + *frame.device_index) {
      throw ParseError("anchor radio ID must equal 0xA0 + device_index");
    }
  }
}

json optional_payload(const json & data)
{
  if (data.contains("payload")) {
    return data["payload"];
  }
  if (data.contains("payload_hex")) {
    return json{{"encoding", "hex"}, {"data", data["payload_hex"]}};
  }
  if (data.contains("data")) {
    return data["data"];
  }
  return nullptr;
}

RangeMeasurement parse_measurement(const json & item, const json & frame)
{
  if (!item.is_object()) {
    throw ParseError("each measurement must be an object");
  }

  const json * anchor_value = nullptr;
  if (item.contains("anchor_id")) {
    anchor_value = &item["anchor_id"];
  } else if (item.contains("source_id")) {
    anchor_value = &item["source_id"];
  } else if (frame.contains("source_id")) {
    anchor_value = &frame["source_id"];
  }
  if (anchor_value == nullptr) {
    throw ParseError("measurement is missing anchor_id/source_id");
  }

  const auto anchor_id = optional_int(*anchor_value);
  if (!anchor_id) {
    throw ParseError("anchor_id must not be null");
  }
  if (!item.contains("range_m") || !item["range_m"].is_number()) {
    throw ParseError("measurement is missing numeric range_m");
  }

  RangeMeasurement result;
  result.anchor_id = *anchor_id;
  result.range_m = item["range_m"].get<double>();
  if (!std::isfinite(result.range_m) || result.range_m < 0.0) {
    throw ParseError("range_m must be a finite non-negative number");
  }
  if (item.contains("valid")) {
    if (!item["valid"].is_boolean()) {
      throw ParseError("valid must be a boolean");
    }
    result.valid = item["valid"].get<bool>();
  }

  result.quality = item.value("quality", frame.value("quality", json::object()));
  result.timestamps = item.value(
    "timestamps", frame.value("timestamps", json::object()));
  if (!result.quality.is_object() || !result.timestamps.is_object()) {
    throw ParseError("quality and timestamps must be objects");
  }

  const json * clock_offset = nullptr;
  if (item.contains("clock_offset")) {
    clock_offset = &item["clock_offset"];
  } else if (frame.contains("clock_offset")) {
    clock_offset = &frame["clock_offset"];
  }
  if (clock_offset != nullptr && !clock_offset->is_null()) {
    if (!clock_offset->is_number()) {
      throw ParseError("clock_offset must be numeric");
    }
    const double value = clock_offset->get<double>();
    if (!std::isfinite(value)) {
      throw ParseError("clock_offset must be finite");
    }
    result.clock_offset = value;
  }

  result.payload = item.value("payload", json(nullptr));
  return result;
}

UwbFrame parse_json_line(const std::string & line)
{
  json data;
  try {
    data = json::parse(line);
  } catch (const json::exception & exception) {
    throw ParseError(std::string("invalid JSON record: ") + exception.what());
  }
  if (!data.is_object()) {
    throw ParseError("JSON record must be an object");
  }

  UwbFrame frame;
  frame.tag_id = data.contains("tag_id") ? optional_int(data["tag_id"]) : std::nullopt;
  frame.device_role = data.value("device_role", std::string("tag"));
  if (frame.device_role != "tag" && frame.device_role != "anchor") {
    throw ParseError("device_role must be 'tag' or 'anchor'");
  }
  frame.device_index =
    data.contains("device_index") ? optional_int(data["device_index"]) : std::nullopt;
  frame.device_radio_id =
    data.contains("device_radio_id") ?
    optional_int(data["device_radio_id"]) : std::nullopt;
  if ((data.contains("device_index") && !frame.device_index) ||
    (data.contains("device_radio_id") && !frame.device_radio_id))
  {
    throw ParseError("device identity fields must not be null");
  }
  validate_device_identity(frame);
  frame.sequence =
    data.contains("sequence") ? optional_int(data["sequence"]) : std::nullopt;
  frame.beacon_sequence =
    data.contains("beacon_sequence") ?
    optional_int(data["beacon_sequence"]) : std::nullopt;
  frame.fail_count =
    data.contains("fail_count") ? optional_int(data["fail_count"]) : std::nullopt;
  if (data.contains("timestamp_ms")) {
    const auto timestamp = optional_int(data["timestamp_ms"]);
    if (timestamp && *timestamp < 0) {
      throw ParseError("timestamp_ms must be non-negative");
    }
    if (timestamp) {
      if (static_cast<std::uint64_t>(*timestamp) >
        std::numeric_limits<std::uint32_t>::max())
      {
        throw ParseError("timestamp_ms is out of range");
      }
      frame.device_timestamp_ms = static_cast<std::uint32_t>(*timestamp);
    }
  }
  if (data.contains("mask")) {
    const auto mask = optional_int(data["mask"]);
    if (mask && *mask < 0) {
      throw ParseError("mask must be non-negative");
    }
    if (mask) {
      frame.mask = static_cast<std::uint32_t>(*mask);
    }
  }

  frame.protocol = data.value("protocol", std::string("uwb_json_v1"));
  if (data.contains("measurements")) {
    if (!data["measurements"].is_array()) {
      throw ParseError("measurements must be an array");
    }
    for (const auto & measurement : data["measurements"]) {
      frame.measurements.push_back(parse_measurement(measurement, data));
    }
  } else if (data.contains("range_m")) {
    frame.measurements.push_back(parse_measurement(data, data));
  }

  frame.payload = optional_payload(data);
  frame.metadata = std::move(data);
  return frame;
}

UwbFrame parse_legacy_line(const std::string & line)
{
  std::smatch match;
  if (!std::regex_match(line, match, kLegacyPattern)) {
    throw ParseError("line does not match the Makerfabs multi-anchor format");
  }

  try {
    UwbFrame frame;
    frame.tag_id = std::stoi(match[1].str());
    frame.device_role = match[8].matched ? match[8].str() : "tag";
    if (match[9].matched) {
      frame.device_index = std::stoi(match[9].str());
      frame.device_radio_id = std::stoi(match[10].str());
    } else {
      frame.device_index = *frame.tag_id;
      if (frame.device_role == "tag" && *frame.tag_id <= 8) {
        frame.device_radio_id = 0xB0 + *frame.tag_id;
      }
    }
    validate_device_identity(frame);
    frame.mask = static_cast<std::uint32_t>(std::stoul(match[2].str(), nullptr, 16));
    frame.sequence = std::stoi(match[3].str());
    if (match[4].matched) {
      frame.beacon_sequence = std::stoi(match[4].str());
    }
    frame.protocol = "makerfabs_multi_anchor_v1";

    if (match[5].matched) {
      frame.fail_count = std::stoi(match[5].str());
    }

    const std::string range_values = match[6].str();
    const bool range_is_meters =
      line.find(",range_m:") != std::string::npos ||
      range_values.find(';') != std::string::npos;
    const char range_delimiter = range_values.find(';') != std::string::npos ? ';' : ',';
    const auto ranges = parse_values<double>(range_values, range_delimiter);
    const auto anchor_ids = parse_values<int>(match[7].str(), ',');
    if (ranges.size() != anchor_ids.size()) {
      throw ParseError("range and anchor ID counts do not match");
    }
    if (ranges.size() > 32) {
      throw ParseError("too many anchors in multi-anchor line");
    }

    for (std::size_t index = 0; index < ranges.size(); ++index) {
      const double distance = ranges[index];
      if (!std::isfinite(distance) || distance < 0.0) {
        throw ParseError("range must be a finite non-negative number");
      }

      const bool id_valid = anchor_ids[index] >= 0;
      const bool mask_valid = ((*frame.mask >> index) & 1U) != 0U;
      if (id_valid && mask_valid) {
        RangeMeasurement measurement;
        measurement.anchor_id = anchor_ids[index];
        measurement.range_m = range_is_meters ? distance : distance / 100.0;
        frame.measurements.push_back(std::move(measurement));
      }
    }
    return frame;
  } catch (const ParseError &) {
    throw;
  } catch (const std::exception & exception) {
    throw ParseError(std::string("invalid number in multi-anchor line: ") + exception.what());
  }
}

}  // namespace

UwbFrame parse_line(const std::string & input)
{
  const std::string line = trim(input);
  if (line.empty()) {
    throw ParseError("empty line");
  }
  try {
    if (line.front() == '{') {
      return parse_json_line(line);
    }
    return parse_legacy_line(line);
  } catch (const ParseError &) {
    throw;
  } catch (const std::exception & exception) {
    throw ParseError(std::string("invalid UWB record: ") + exception.what());
  }
}

}  // namespace uwb_serial_driver
