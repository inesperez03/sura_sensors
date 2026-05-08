#include "sura_sensors/broadcasters/battery_broadcaster.hpp"

#include <cmath>
#include <exception>
#include <limits>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace sura_sensors
{

controller_interface::CallbackReturn BatteryBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("sensor_name", "battery_sensor");
    auto_declare<std::string>("frame_id", "battery_link");
    auto_declare<std::string>("topic_name", "/sura/sensors/battery");
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

  publisher_ = get_node()->create_publisher<sensor_msgs::msg::BatteryState>(
    topic_name_, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured BatteryBroadcaster: sensor_name='%s', frame_id='%s', topic='%s'",
    sensor_name_.c_str(),
    frame_id_.c_str(),
    topic_name_.c_str());

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
  msg.percentage = std::numeric_limits<float>::quiet_NaN();
  msg.present = present > 0.5;
  msg.power_supply_status =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  msg.power_supply_health =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  msg.power_supply_technology =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

  if (!std::isfinite(voltage) || !std::isfinite(current)) {
    msg.power_supply_status =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    msg.power_supply_health =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  }

  publisher_->publish(msg);

  return controller_interface::return_type::OK;
}

}  // namespace sura_sensors

PLUGINLIB_EXPORT_CLASS(
  sura_sensors::BatteryBroadcaster,
  controller_interface::ControllerInterface)
