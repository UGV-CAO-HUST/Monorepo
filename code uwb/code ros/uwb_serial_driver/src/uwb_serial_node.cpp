#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include "uwb_serial_driver/parser.hpp"
#include "uwb_serial_driver/serial_port.hpp"
#include "uwb_serial_driver/uwb_types.hpp"

namespace uwb_serial_driver
{

class UwbSerialNode : public rclcpp::Node
{
public:
  UwbSerialNode()
  : Node("uwb_serial_node"),
    port_name_(declare_parameter<std::string>("port", "/dev/ttyUSB0")),
    baudrate_(declare_parameter<int>("baudrate", 115200)),
    update_rate_hz_(declare_parameter<double>("update_rate_hz", 50.0)),
    reconnect_interval_sec_(declare_parameter<double>("reconnect_interval_sec", 2.0)),
    data_timeout_sec_(declare_parameter<double>("data_timeout_sec", 3.0)),
    serial_port_(port_name_, baudrate_)
  {
    validate_parameters();
    measurements_pub_ = create_publisher<std_msgs::msg::String>("uwb/measurements", 10);
    payload_pub_ = create_publisher<std_msgs::msg::String>("uwb/payload", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("uwb/status", 10);

    const auto interval = std::chrono::duration<double>(1.0 / update_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(interval),
      std::bind(&UwbSerialNode::poll_serial, this));
    publish_status("connecting", "Waiting for serial connection");
  }

  ~UwbSerialNode() override
  {
    serial_port_.close();
  }

private:
  void validate_parameters() const
  {
    if (port_name_.empty()) {
      throw std::invalid_argument("parameter 'port' must not be empty");
    }
    if (baudrate_ <= 0) {
      throw std::invalid_argument("parameter 'baudrate' must be positive");
    }
    if (!std::isfinite(update_rate_hz_) || update_rate_hz_ <= 0.0) {
      throw std::invalid_argument("parameter 'update_rate_hz' must be finite and positive");
    }
    if (!std::isfinite(reconnect_interval_sec_) || reconnect_interval_sec_ <= 0.0 ||
      !std::isfinite(data_timeout_sec_) || data_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("reconnect and data timeout parameters must be finite and positive");
    }
  }

  void poll_serial()
  {
    if (!serial_port_.is_open()) {
      try_connect();
      return;
    }

    try {
      std::string line;
      for (int count = 0; count < 100 && serial_port_.read_line(line); ++count) {
        handle_line(line);
        line.clear();
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Serial connection lost: %s", exception.what());
      serial_port_.close();
      has_valid_data_ = false;
      next_connect_time_ = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(reconnect_interval_sec_));
      publish_status("disconnected", exception.what());
      return;
    }

    if (has_valid_data_ &&
      std::chrono::steady_clock::now() - last_valid_data_time_ >
      std::chrono::duration<double>(data_timeout_sec_))
    {
      publish_status("stale", "No valid UWB data before timeout");
    }
  }

  void try_connect()
  {
    const auto now = std::chrono::steady_clock::now();
    if (now < next_connect_time_) {
      return;
    }

    try {
      serial_port_.open();
    } catch (const std::exception & exception) {
      RCLCPP_WARN(get_logger(), "Cannot open %s: %s", port_name_.c_str(), exception.what());
      next_connect_time_ = now +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(reconnect_interval_sec_));
      publish_status("disconnected", exception.what());
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Connected to %s at %d baud", port_name_.c_str(), baudrate_);
    last_valid_data_time_ = now;
    has_valid_data_ = true;
    publish_status("connected", "Serial open; waiting for valid UWB data");
  }

  void handle_line(const std::string & line)
  {
    try {
      const auto frame = parse_line(line);
      last_valid_data_time_ = std::chrono::steady_clock::now();
      has_valid_data_ = true;
      device_role_ = frame.device_role;
      device_index_ = frame.device_index;
      device_radio_id_ = frame.device_radio_id;
      publish_status("connected", "Receiving valid UWB data");
      publish_frame(frame);
    } catch (const ParseError & exception) {
      ++parse_errors_;
      if (parse_errors_ == 1 || parse_errors_ % 50 == 0) {
        RCLCPP_WARN(
          get_logger(), "Rejected malformed UWB line (%llu total): %s",
          static_cast<unsigned long long>(parse_errors_), exception.what());
        publish_status("parse_error", exception.what());
      }
    }
  }

  void publish_frame(const UwbFrame & frame)
  {
    const auto received_stamp = now();
    nlohmann::json measurements = nlohmann::json::array();
    for (const auto & measurement : frame.measurements) {
      measurements.push_back({
        {"anchor_id", measurement.anchor_id},
        {"anchor_radio_id", measurement.anchor_id >= 0 && measurement.anchor_id < 8 ?
          nlohmann::json(0xA0 + measurement.anchor_id) : nlohmann::json(nullptr)},
        {"valid", measurement.valid},
        {"range_m", measurement.range_m},
        {"quality", measurement.quality},
        {"timestamps", measurement.timestamps},
        {"clock_offset", measurement.clock_offset ?
          nlohmann::json(*measurement.clock_offset) : nlohmann::json(nullptr)},
        {"payload", measurement.payload}
      });
    }

    nlohmann::json normalized = {
      {"protocol", frame.protocol},
      {"device_role", frame.device_role},
      {"device_index", frame.device_index ?
        nlohmann::json(*frame.device_index) : nlohmann::json(nullptr)},
      {"device_radio_id", frame.device_radio_id ?
        nlohmann::json(*frame.device_radio_id) : nlohmann::json(nullptr)},
      {"tag_id", frame.tag_id ? nlohmann::json(*frame.tag_id) : nlohmann::json(nullptr)},
      {"sequence", frame.sequence ? nlohmann::json(*frame.sequence) : nlohmann::json(nullptr)},
      {"beacon_sequence", frame.beacon_sequence ?
        nlohmann::json(*frame.beacon_sequence) : nlohmann::json(nullptr)},
      {"fail_count", frame.fail_count ?
        nlohmann::json(*frame.fail_count) : nlohmann::json(nullptr)},
      {"mask", frame.mask ? nlohmann::json(*frame.mask) : nlohmann::json(nullptr)},
      {"device_timestamp_ms", frame.device_timestamp_ms ?
        nlohmann::json(*frame.device_timestamp_ms) : nlohmann::json(nullptr)},
      {"ros_received_stamp", {
        {"sec", received_stamp.nanoseconds() / 1000000000LL},
        {"nanosec", received_stamp.nanoseconds() % 1000000000LL}
      }},
      {"measurements", measurements},
      {"metadata", frame.metadata}
    };
    std_msgs::msg::String measurement_message;
    measurement_message.data = normalized.dump();
    measurements_pub_->publish(measurement_message);

    for (const auto & measurement : frame.measurements) {
      if (measurement.valid) {
        publish_range(frame, measurement);
        if (frame.device_role == "anchor" && frame.device_index) {
          publish_anchor_report(frame, measurement);
        }
      }
    }

    if (!frame.payload.is_null()) {
      nlohmann::json payload = {
        {"device_role", frame.device_role},
        {"device_index", frame.device_index ?
          nlohmann::json(*frame.device_index) : nlohmann::json(nullptr)},
        {"device_radio_id", frame.device_radio_id ?
          nlohmann::json(*frame.device_radio_id) : nlohmann::json(nullptr)},
        {"tag_id", frame.tag_id ? nlohmann::json(*frame.tag_id) : nlohmann::json(nullptr)},
        {"sequence", frame.sequence ? nlohmann::json(*frame.sequence) : nlohmann::json(nullptr)},
        {"device_timestamp_ms", frame.device_timestamp_ms ?
          nlohmann::json(*frame.device_timestamp_ms) : nlohmann::json(nullptr)},
        {"payload", frame.payload}
      };
      std_msgs::msg::String payload_message;
      payload_message.data = payload.dump();
      payload_pub_->publish(payload_message);
    }
  }

  void publish_range(const UwbFrame & frame, const RangeMeasurement & measurement)
  {
    const std::string tag_id = frame.tag_id ? std::to_string(*frame.tag_id) : "unknown";
    const std::string topic =
      "uwb/tag_" + tag_id + "/anchor_" + std::to_string(measurement.anchor_id) + "/range";
    auto publisher = range_publishers_.find(topic);
    if (publisher == range_publishers_.end()) {
      publisher = range_publishers_.emplace(
        topic, create_publisher<std_msgs::msg::Float32>(topic, 10)).first;
    }

    std_msgs::msg::Float32 message;
    message.data = static_cast<float>(measurement.range_m);
    publisher->second->publish(message);
  }

  void publish_anchor_report(
    const UwbFrame & frame, const RangeMeasurement & measurement)
  {
    const std::string tag_id = frame.tag_id ? std::to_string(*frame.tag_id) : "unknown";
    const std::string topic =
      "uwb/anchor_" + std::to_string(*frame.device_index) +
      "/tag_" + tag_id + "/reported_range";
    auto publisher = range_publishers_.find(topic);
    if (publisher == range_publishers_.end()) {
      publisher = range_publishers_.emplace(
        topic, create_publisher<std_msgs::msg::Float32>(topic, 10)).first;
    }

    std_msgs::msg::Float32 message;
    message.data = static_cast<float>(measurement.range_m);
    publisher->second->publish(message);
  }

  void publish_status(const std::string & state, const std::string & detail)
  {
    nlohmann::json status = {
      {"state", state},
      {"detail", detail},
      {"port", port_name_},
      {"baudrate", baudrate_},
      {"parse_errors", parse_errors_}
    };
    if (!device_role_.empty() && device_index_) {
      status["device_role"] = device_role_;
      status["device_index"] = *device_index_;
      status["device_radio_id"] = device_radio_id_ ?
        nlohmann::json(*device_radio_id_) : nlohmann::json(nullptr);
    }
    const std::string encoded = status.dump();
    if (encoded != last_status_) {
      std_msgs::msg::String message;
      message.data = encoded;
      status_pub_->publish(message);
      last_status_ = encoded;

      if (!device_role_.empty() && device_index_) {
        const std::string device_topic =
          "uwb/" + device_role_ + "_" + std::to_string(*device_index_) + "/status";
        auto publisher = device_status_publishers_.find(device_topic);
        if (publisher == device_status_publishers_.end()) {
          publisher = device_status_publishers_.emplace(
            device_topic, create_publisher<std_msgs::msg::String>(device_topic, 10)).first;
        }
        publisher->second->publish(message);
      }
    }
  }

  std::string port_name_;
  int baudrate_;
  double update_rate_hz_;
  double reconnect_interval_sec_;
  double data_timeout_sec_;
  SerialPort serial_port_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr measurements_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr payload_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr>
    range_publishers_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr>
    device_status_publishers_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point next_connect_time_{};
  std::chrono::steady_clock::time_point last_valid_data_time_{};
  bool has_valid_data_{false};
  std::string device_role_;
  std::optional<int> device_index_;
  std::optional<int> device_radio_id_;
  std::uint64_t parse_errors_{0};
  std::string last_status_;
};

}  // namespace uwb_serial_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<uwb_serial_driver::UwbSerialNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("uwb_serial_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
