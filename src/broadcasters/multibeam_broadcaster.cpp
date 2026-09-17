#include "sura_sensors/broadcasters/multibeam_broadcaster.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace sura_sensors
{
namespace
{

double value_at(
  const std::vector<hardware_interface::LoanedStateInterface> & interfaces,
  const std::size_t index)
{
  return interfaces[index].get_value();
}

uint32_t u32_value(const double value)
{
  return static_cast<uint32_t>(std::llround(value));
}

uint16_t u16_value(const double value)
{
  return static_cast<uint16_t>(std::clamp(std::llround(value), 0LL, 65535LL));
}

uint8_t u8_value(const double value)
{
  return static_cast<uint8_t>(std::clamp(std::llround(value), 0LL, 255LL));
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string transport_state_name(const int state)
{
  switch (state) {
    case 0:
      return "Disconnected";
    case 1:
      return "Connecting";
    case 2:
      return "Initializing";
    case 3:
      return "Streaming";
    case 4:
      return "Backoff";
    default:
      return "Unknown";
  }
}

}  // namespace

controller_interface::CallbackReturn MultibeamBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("sensor_name", "multibeam_sensor");
    auto_declare<std::string>("frame_id", "blueboat/multibeam_link");
    auto_declare<std::string>("points_topic", "sensors/multibeam/points");
    auto_declare<std::string>("ping_topic", "sensors/multibeam/ping");
    auto_declare<std::string>("up_vector_topic", "sensors/multibeam/attitude/up_vector");
    auto_declare<std::string>("temperature_topic", "sensors/multibeam/temperature");
    auto_declare<std::string>("pressure_topic", "sensors/multibeam/pressure");
    auto_declare<double>("publish_period_s", publish_period_s_);
    auto_declare<double>("stale_timeout_s", stale_timeout_s_);
  } catch (...) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MultibeamBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
MultibeamBroadcaster::state_interface_configuration() const
{
  const auto sensor_name = get_node()->get_parameter("sensor_name").as_string();
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    state_names(sensor_name)};
}

controller_interface::CallbackReturn MultibeamBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  points_topic_ = resolve_topic(get_node()->get_parameter("points_topic").as_string());
  ping_topic_ = resolve_topic(get_node()->get_parameter("ping_topic").as_string());
  up_vector_topic_ = resolve_topic(get_node()->get_parameter("up_vector_topic").as_string());
  temperature_topic_ = resolve_topic(get_node()->get_parameter("temperature_topic").as_string());
  pressure_topic_ = resolve_topic(get_node()->get_parameter("pressure_topic").as_string());
  publish_period_s_ = get_node()->get_parameter("publish_period_s").as_double();
  stale_timeout_s_ = get_node()->get_parameter("stale_timeout_s").as_double();

  ping_pub_ = get_node()->create_publisher<sura_msgs::msg::MultibeamPing>(
    ping_topic_,
    rclcpp::SensorDataQoS());
  points_pub_ = get_node()->create_publisher<sensor_msgs::msg::PointCloud2>(
    points_topic_,
    rclcpp::SensorDataQoS());
  up_vector_pub_ = get_node()->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    up_vector_topic_,
    rclcpp::SensorDataQoS());
  temperature_pub_ = get_node()->create_publisher<sensor_msgs::msg::Temperature>(
    temperature_topic_,
    rclcpp::SensorDataQoS());
  pressure_pub_ = get_node()->create_publisher<sensor_msgs::msg::FluidPressure>(
    pressure_topic_,
    rclcpp::SensorDataQoS());
  diagnostics_pub_ = get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics",
    10);

  const auto period = std::chrono::duration<double>(std::max(0.01, publish_period_s_));
  publish_timer_ = get_node()->create_wall_timer(period, [this]() { publish_latest(); });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MultibeamBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (state_interfaces_.size() != state_names(sensor_name_).size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "MultibeamBroadcaster expected %zu state interfaces, got %zu",
      state_names(sensor_name_).size(),
      state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  ping_pub_->on_activate();
  points_pub_->on_activate();
  up_vector_pub_->on_activate();
  temperature_pub_->on_activate();
  pressure_pub_->on_activate();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MultibeamBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (ping_pub_) {
    ping_pub_->on_deactivate();
  }
  if (points_pub_) {
    points_pub_->on_deactivate();
  }
  if (up_vector_pub_) {
    up_vector_pub_->on_deactivate();
  }
  if (temperature_pub_) {
    temperature_pub_->on_deactivate();
  }
  if (pressure_pub_) {
    pressure_pub_->on_deactivate();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MultibeamBroadcaster::update(
  const rclcpp::Time & time,
  const rclcpp::Duration &)
{
  if (state_interfaces_.size() != state_names(sensor_name_).size()) {
    return controller_interface::return_type::OK;
  }

  Snapshot snapshot;
  std::size_t i = 0;
  snapshot.ping_sequence = read_counter(value_at(state_interfaces_, i++));
  const auto sec = static_cast<int32_t>(std::llround(value_at(state_interfaces_, i++)));
  const auto nsec = u32_value(value_at(state_interfaces_, i++));
  snapshot.reception_time = sec == 0 && nsec == 0 ? time : rclcpp::Time(sec, nsec, RCL_ROS_TIME);
  snapshot.ping_number = u32_value(value_at(state_interfaces_, i++));
  snapshot.power_up_time_ms = u32_value(value_at(state_interfaces_, i++));
  const auto device_utc_time_ms_hi = value_at(state_interfaces_, i++);
  const auto device_utc_time_ms_lo = value_at(state_interfaces_, i++);
  snapshot.device_utc_time_ms = read_u64(device_utc_time_ms_hi, device_utc_time_ms_lo);
  snapshot.listening_time_s = static_cast<float>(value_at(state_interfaces_, i++));
  snapshot.sound_speed_m_s = static_cast<float>(value_at(state_interfaces_, i++));
  snapshot.acoustic_frequency_hz = u32_value(value_at(state_interfaces_, i++));
  snapshot.pulse_duration_s = static_cast<float>(value_at(state_interfaces_, i++));
  snapshot.flags = u32_value(value_at(state_interfaces_, i++));
  snapshot.reported_detection_count = u16_value(value_at(state_interfaces_, i++));
  snapshot.stored_detection_count = u16_value(value_at(state_interfaces_, i++));
  snapshot.truncated = value_at(state_interfaces_, i++) > 0.5;

  for (auto & detection : snapshot.detections) {
    detection.angle_rad = static_cast<float>(value_at(state_interfaces_, i++));
    detection.time_of_flight_s = static_cast<float>(value_at(state_interfaces_, i++));
    detection.power = static_cast<float>(value_at(state_interfaces_, i++));
    detection.point_type = u8_value(value_at(state_interfaces_, i++));
    detection.reserved = u32_value(value_at(state_interfaces_, i++));
  }

  snapshot.attitude_sequence = read_counter(value_at(state_interfaces_, i++));
  snapshot.up_vector_x = static_cast<float>(value_at(state_interfaces_, i++));
  snapshot.up_vector_y = static_cast<float>(value_at(state_interfaces_, i++));
  snapshot.up_vector_z = static_cast<float>(value_at(state_interfaces_, i++));
  i += 3;

  snapshot.water_sequence = read_counter(value_at(state_interfaces_, i++));
  snapshot.water_valid = value_at(state_interfaces_, i++) > 0.5;
  snapshot.temperature_c = static_cast<float>(value_at(state_interfaces_, i++));
  snapshot.pressure_bar = static_cast<float>(value_at(state_interfaces_, i++));

  snapshot.transport_state = static_cast<int>(std::llround(value_at(state_interfaces_, i++)));
  snapshot.connected = value_at(state_interfaces_, i++) > 0.5;
  snapshot.valid_pings = read_counter(value_at(state_interfaces_, i++));
  snapshot.reconnect_count = read_counter(value_at(state_interfaces_, i++));
  snapshot.checksum_errors = read_counter(value_at(state_interfaces_, i++));
  snapshot.malformed_packets = read_counter(value_at(state_interfaces_, i++));
  snapshot.unknown_ids = read_counter(value_at(state_interfaces_, i++));
  snapshot.truncated_pings = read_counter(value_at(state_interfaces_, i++));
  snapshot.rx_bytes = read_counter(value_at(state_interfaces_, i++));
  snapshot.tx_bytes = read_counter(value_at(state_interfaces_, i++));
  snapshot.last_errno = static_cast<int>(std::llround(value_at(state_interfaces_, i++)));
  snapshot.seconds_since_last_rx = value_at(state_interfaces_, i++);
  snapshot.seconds_since_last_ping = value_at(state_interfaces_, i++);

  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  latest_snapshot_ = snapshot;
  return controller_interface::return_type::OK;
}

std::vector<std::string> MultibeamBroadcaster::state_names(const std::string & sensor_name)
{
  std::vector<std::string> names = {
    sensor_name + "/ping_sequence",
    sensor_name + "/reception_time.sec",
    sensor_name + "/reception_time.nanosec",
    sensor_name + "/ping_number",
    sensor_name + "/power_up_time_ms",
    sensor_name + "/device_utc_time_ms_hi",
    sensor_name + "/device_utc_time_ms_lo",
    sensor_name + "/listening_time_s",
    sensor_name + "/sound_speed_m_s",
    sensor_name + "/acoustic_frequency_hz",
    sensor_name + "/pulse_duration_s",
    sensor_name + "/flags",
    sensor_name + "/reported_detection_count",
    sensor_name + "/stored_detection_count",
    sensor_name + "/truncated",
  };

  for (std::size_t i = 0; i < kMaxDetections; ++i) {
    const auto prefix = sensor_name + "/detection_" + std::to_string(i) + ".";
    names.push_back(prefix + "angle_rad");
    names.push_back(prefix + "time_of_flight_s");
    names.push_back(prefix + "power");
    names.push_back(prefix + "point_type");
    names.push_back(prefix + "reserved");
  }

  const std::vector<std::string> tail = {
    sensor_name + "/attitude_sequence",
    sensor_name + "/up_vector.x",
    sensor_name + "/up_vector.y",
    sensor_name + "/up_vector.z",
    sensor_name + "/attitude_utc_time_ms_hi",
    sensor_name + "/attitude_utc_time_ms_lo",
    sensor_name + "/attitude_power_up_time_ms",
    sensor_name + "/water_sequence",
    sensor_name + "/water_valid",
    sensor_name + "/temperature_c",
    sensor_name + "/pressure_bar",
    sensor_name + "/transport_state",
    sensor_name + "/connected",
    sensor_name + "/valid_pings",
    sensor_name + "/reconnect_count",
    sensor_name + "/checksum_errors",
    sensor_name + "/malformed_packets",
    sensor_name + "/unknown_ids",
    sensor_name + "/truncated_pings",
    sensor_name + "/rx_bytes",
    sensor_name + "/tx_bytes",
    sensor_name + "/last_errno",
    sensor_name + "/seconds_since_last_rx",
    sensor_name + "/seconds_since_last_ping",
  };
  names.insert(names.end(), tail.begin(), tail.end());
  return names;
}

uint64_t MultibeamBroadcaster::read_u64(const double hi, const double lo)
{
  return (static_cast<uint64_t>(u32_value(hi)) << 32U) | static_cast<uint64_t>(u32_value(lo));
}

uint64_t MultibeamBroadcaster::read_counter(const double value)
{
  return static_cast<uint64_t>(std::max(0.0, value));
}

std::string MultibeamBroadcaster::resolve_topic(const std::string & value) const
{
  if (value.empty() || value.front() == '/') {
    return value;
  }

  std::string ns = get_node()->get_namespace();
  if (ns == "/") {
    return "/" + value;
  }
  const std::string suffix = "/controller";
  if (ns.size() > suffix.size() && ns.compare(ns.size() - suffix.size(), suffix.size(), suffix) == 0) {
    ns.erase(ns.size() - suffix.size());
  }
  return ns + "/" + value;
}

void MultibeamBroadcaster::publish_latest()
{
  Snapshot snapshot;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot = latest_snapshot_;
  }

  publish_diagnostics(snapshot);

  if (snapshot.ping_sequence != 0U && snapshot.ping_sequence != last_published_ping_sequence_) {
    publish_ping_and_points(snapshot);
    last_published_ping_sequence_ = snapshot.ping_sequence;
  }

  if (snapshot.attitude_sequence != 0U && snapshot.attitude_sequence != last_published_attitude_sequence_) {
    publish_attitude(snapshot);
    last_published_attitude_sequence_ = snapshot.attitude_sequence;
  }

  if (
    snapshot.water_valid &&
    snapshot.water_sequence != 0U &&
    snapshot.water_sequence != last_published_water_sequence_)
  {
    publish_water(snapshot);
    last_published_water_sequence_ = snapshot.water_sequence;
  }
}

void MultibeamBroadcaster::publish_ping_and_points(const Snapshot & snapshot)
{
  if (!ping_pub_ || !points_pub_ || !ping_pub_->is_activated() || !points_pub_->is_activated()) {
    return;
  }

  sura_msgs::msg::MultibeamPing ping;
  ping.header.stamp = snapshot.reception_time;
  ping.header.frame_id = frame_id_;
  ping.ping_number = snapshot.ping_number;
  ping.power_up_time_ms = snapshot.power_up_time_ms;
  ping.device_utc_time_ms = snapshot.device_utc_time_ms;
  ping.listening_time_s = snapshot.listening_time_s;
  ping.sound_speed_m_s = snapshot.sound_speed_m_s;
  ping.acoustic_frequency_hz = snapshot.acoustic_frequency_hz;
  ping.pulse_duration_s = snapshot.pulse_duration_s;
  ping.flags = snapshot.flags;
  ping.reported_detection_count = snapshot.reported_detection_count;
  ping.truncated = snapshot.truncated;

  const auto count = std::min<std::size_t>(snapshot.stored_detection_count, kMaxDetections);
  ping.detections.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    sura_msgs::msg::MultibeamDetection detection;
    detection.angle_rad = snapshot.detections[i].angle_rad;
    detection.time_of_flight_s = snapshot.detections[i].time_of_flight_s;
    detection.power = snapshot.detections[i].power;
    detection.point_type = snapshot.detections[i].point_type;
    detection.reserved[0] = static_cast<uint8_t>(snapshot.detections[i].reserved & 0xffU);
    detection.reserved[1] = static_cast<uint8_t>((snapshot.detections[i].reserved >> 8U) & 0xffU);
    detection.reserved[2] = static_cast<uint8_t>((snapshot.detections[i].reserved >> 16U) & 0xffU);
    ping.detections.push_back(detection);
  }
  ping_pub_->publish(ping);

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = ping.header;
  cloud.height = 1;
  cloud.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    6,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "angle_rad", 1, sensor_msgs::msg::PointField::FLOAT32,
    "time_of_flight_s", 1, sensor_msgs::msg::PointField::FLOAT32,
    "range_m", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(count);

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> angle(cloud, "angle_rad");
  sensor_msgs::PointCloud2Iterator<float> tof(cloud, "time_of_flight_s");
  sensor_msgs::PointCloud2Iterator<float> range_field(cloud, "range_m");

  for (std::size_t i = 0; i < count; ++i, ++x, ++y, ++z, ++angle, ++tof, ++range_field) {
    const float range = 0.5F * snapshot.sound_speed_m_s * snapshot.detections[i].time_of_flight_s;
    *x = 0.0F;
    *y = range * std::sin(snapshot.detections[i].angle_rad);
    *z = -range * std::cos(snapshot.detections[i].angle_rad);
    *angle = snapshot.detections[i].angle_rad;
    *tof = snapshot.detections[i].time_of_flight_s;
    *range_field = range;
  }
  points_pub_->publish(cloud);
}

void MultibeamBroadcaster::publish_attitude(const Snapshot & snapshot)
{
  if (!up_vector_pub_ || !up_vector_pub_->is_activated()) {
    return;
  }
  geometry_msgs::msg::Vector3Stamped msg;
  msg.header.stamp = get_node()->get_clock()->now();
  msg.header.frame_id = frame_id_;
  msg.vector.x = snapshot.up_vector_x;
  msg.vector.y = snapshot.up_vector_y;
  msg.vector.z = snapshot.up_vector_z;
  up_vector_pub_->publish(msg);
}

void MultibeamBroadcaster::publish_water(const Snapshot & snapshot)
{
  if (!temperature_pub_ || !pressure_pub_ || !temperature_pub_->is_activated() || !pressure_pub_->is_activated()) {
    return;
  }

  sensor_msgs::msg::Temperature temperature;
  temperature.header.stamp = get_node()->get_clock()->now();
  temperature.header.frame_id = frame_id_;
  temperature.temperature = snapshot.temperature_c;
  temperature.variance = 0.0;
  temperature_pub_->publish(temperature);

  sensor_msgs::msg::FluidPressure pressure;
  pressure.header = temperature.header;
  pressure.fluid_pressure = static_cast<double>(snapshot.pressure_bar) * 100000.0;
  pressure.variance = 0.0;
  pressure_pub_->publish(pressure);
}

void MultibeamBroadcaster::publish_diagnostics(const Snapshot & snapshot)
{
  if (!diagnostics_pub_) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "/Sensors/Multibeam";
  status.hardware_id = sensor_name_;

  if (!snapshot.connected) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "Multibeam disconnected";
  } else if (snapshot.seconds_since_last_ping > stale_timeout_s_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    status.message = "Multibeam ping stream is stale";
  } else if (snapshot.truncated || snapshot.checksum_errors > 0U || snapshot.malformed_packets > 0U) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Multibeam stream has recoverable errors";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Multibeam streaming";
  }

  status.values = {
    key_value("connected", snapshot.connected ? "true" : "false"),
    key_value("transport_state", transport_state_name(snapshot.transport_state)),
    key_value("valid_pings", std::to_string(snapshot.valid_pings)),
    key_value("last_ping_age_s", std::to_string(snapshot.seconds_since_last_ping)),
    key_value("last_rx_age_s", std::to_string(snapshot.seconds_since_last_rx)),
    key_value("reported_detection_count", std::to_string(snapshot.reported_detection_count)),
    key_value("stored_detection_count", std::to_string(snapshot.stored_detection_count)),
    key_value("truncated", snapshot.truncated ? "true" : "false"),
    key_value("truncated_pings", std::to_string(snapshot.truncated_pings)),
    key_value("reconnect_count", std::to_string(snapshot.reconnect_count)),
    key_value("checksum_errors", std::to_string(snapshot.checksum_errors)),
    key_value("malformed_packets", std::to_string(snapshot.malformed_packets)),
    key_value("unknown_ids", std::to_string(snapshot.unknown_ids)),
    key_value("rx_bytes", std::to_string(snapshot.rx_bytes)),
    key_value("tx_bytes", std::to_string(snapshot.tx_bytes)),
    key_value("last_errno", std::to_string(snapshot.last_errno)),
  };

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_node()->get_clock()->now();
  array.status.push_back(status);
  diagnostics_pub_->publish(array);
}

}  // namespace sura_sensors

PLUGINLIB_EXPORT_CLASS(
  sura_sensors::MultibeamBroadcaster,
  controller_interface::ControllerInterface)
