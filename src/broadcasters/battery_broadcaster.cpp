#include "sura_sensors/broadcasters/battery_broadcaster.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>

namespace sura_sensors
{

controller_interface::CallbackReturn BatteryBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("sensor_name", "battery_sensor");
    auto_declare<std::string>("frame_id", "battery_link");
    auto_declare<std::string>("topic_name", "sensors/battery");
    auto_declare<int>("cell_count", 4);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception in BatteryBroadcaster::on_init(): %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Unknown exception in BatteryBroadcaster::on_init()");
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
BatteryBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
BatteryBroadcaster::state_interface_configuration() const
{
  const auto sensor_name = get_node()->get_parameter("sensor_name").as_string();

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      sensor_name + "/voltage",
      sensor_name + "/current",
      sensor_name + "/present",
    }};
}

controller_interface::CallbackReturn BatteryBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  topic_name_ = get_node()->get_parameter("topic_name").as_string();
  cell_count_ = get_node()->get_parameter("cell_count").as_int();

  if (cell_count_ != 4 && cell_count_ != 6) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Invalid battery cell_count=%d. Expected 4 or 6. Using 4 cells.",
      cell_count_);
    cell_count_ = 4;
  }

  publisher_ = get_node()->create_publisher<sensor_msgs::msg::BatteryState>(
    topic_name_, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured BatteryBroadcaster: sensor_name='%s', frame_id='%s', topic='%s', cell_count=%d",
    sensor_name_.c_str(),
    frame_id_.c_str(),
    topic_name_.c_str(),
    cell_count_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BatteryBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!publisher_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Publisher is null");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != 3) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected 3 state interfaces, got %zu",
      state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  publisher_->on_activate();

  RCLCPP_INFO(get_node()->get_logger(), "BatteryBroadcaster activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BatteryBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (publisher_) {
    publisher_->on_deactivate();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type BatteryBroadcaster::update(
  const rclcpp::Time & time,
  const rclcpp::Duration &)
{
  if (!publisher_ || !publisher_->is_activated()) {
    return controller_interface::return_type::OK;
  }

  if (state_interfaces_.size() != 3) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "BatteryBroadcaster expected 3 state interfaces, got %zu",
      state_interfaces_.size());
    return controller_interface::return_type::OK;
  }

  const double voltage = state_interfaces_[0].get_value();
  const double current = state_interfaces_[1].get_value();
  const double present = state_interfaces_[2].get_value();

  sensor_msgs::msg::BatteryState msg;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id_;
  msg.voltage = static_cast<float>(voltage);
  msg.current = static_cast<float>(current);
  msg.percentage = static_cast<float>(battery_percentage(voltage));
  msg.present = present > 0.5;
  msg.power_supply_status =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  msg.power_supply_health =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  msg.power_supply_technology =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

  if (!std::isfinite(voltage) || !std::isfinite(current)) {
    msg.percentage = std::numeric_limits<float>::quiet_NaN();
    msg.power_supply_status =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    msg.power_supply_health =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  }

  publisher_->publish(msg);

  return controller_interface::return_type::OK;
}

double BatteryBroadcaster::battery_percentage(double voltage) const
{
  if (!std::isfinite(voltage)) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  static constexpr std::array<std::pair<double, double>, 12> li_ion_curve{{
    {3.00, 0.00},
    {3.30, 0.05},
    {3.50, 0.10},
    {3.62, 0.20},
    {3.70, 0.30},
    {3.77, 0.40},
    {3.83, 0.50},
    {3.87, 0.60},
    {3.92, 0.70},
    {3.98, 0.80},
    {4.06, 0.90},
    {4.20, 1.00},
  }};

  const double cell_voltage = voltage / static_cast<double>(cell_count_);
  if (cell_voltage <= li_ion_curve.front().first) {
    return li_ion_curve.front().second;
  }
  if (cell_voltage >= li_ion_curve.back().first) {
    return li_ion_curve.back().second;
  }

  for (std::size_t index = 1; index < li_ion_curve.size(); ++index) {
    const auto [lower_voltage, lower_percentage] = li_ion_curve[index - 1];
    const auto [upper_voltage, upper_percentage] = li_ion_curve[index];
    if (cell_voltage <= upper_voltage) {
      const double ratio = (cell_voltage - lower_voltage) / (upper_voltage - lower_voltage);
      return lower_percentage + ratio * (upper_percentage - lower_percentage);
    }
  }

  return li_ion_curve.back().second;
}

}  // namespace sura_sensors

PLUGINLIB_EXPORT_CLASS(
  sura_sensors::BatteryBroadcaster,
  controller_interface::ControllerInterface)
