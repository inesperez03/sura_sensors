#include "sura_sensors/broadcasters/leak_broadcaster.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <exception>
#include <string>

namespace sura_sensors
{

controller_interface::CallbackReturn LeakBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("sensor_name", "leak_sensor");
    auto_declare<std::string>("topic_name", "sensors/leak");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception in LeakBroadcaster::on_init(): %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Unknown exception in LeakBroadcaster::on_init()");
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
LeakBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
LeakBroadcaster::state_interface_configuration() const
{
  const auto sensor_name = get_node()->get_parameter("sensor_name").as_string();

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      sensor_name + "/leak",
    }};
}

controller_interface::CallbackReturn LeakBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  topic_name_ = get_node()->get_parameter("topic_name").as_string();

  publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
    topic_name_, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured LeakBroadcaster: sensor_name='%s', topic='%s'",
    sensor_name_.c_str(),
    topic_name_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeakBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!publisher_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Publisher is null");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != 1) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected 1 state interface, got %zu",
      state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  publisher_->on_activate();

  RCLCPP_INFO(get_node()->get_logger(), "LeakBroadcaster activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeakBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (publisher_) {
    publisher_->on_deactivate();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LeakBroadcaster::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!publisher_ || !publisher_->is_activated()) {
    return controller_interface::return_type::OK;
  }

  if (state_interfaces_.size() != 1) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "LeakBroadcaster expected 1 state interface, got %zu",
      state_interfaces_.size());
    return controller_interface::return_type::OK;
  }

  const double leak = state_interfaces_[0].get_value();

  if (!std::isfinite(leak)) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Invalid leak value: %.3f",
      leak);
    return controller_interface::return_type::OK;
  }

  std_msgs::msg::Bool msg;
  msg.data = leak > 0.5;
  publisher_->publish(msg);

  return controller_interface::return_type::OK;
}

}  // namespace sura_sensors

PLUGINLIB_EXPORT_CLASS(
  sura_sensors::LeakBroadcaster,
  controller_interface::ControllerInterface)
