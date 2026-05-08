#include "sura_sensors/broadcasters/pressure_broadcaster.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <exception>
#include <string>

namespace sura_sensors
{

controller_interface::CallbackReturn PressureBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("sensor_name", "pressure_sensor");
    auto_declare<std::string>("frame_id", "bluerov2/base_link");
    auto_declare<std::string>("topic_name", "/pressure_broadcaster/fluid_pressure");

    auto_declare<double>("pressure_variance", 0.0);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception in PressureBroadcaster::on_init(): %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Unknown exception in PressureBroadcaster::on_init()");
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PressureBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
PressureBroadcaster::state_interface_configuration() const
{
  const auto sensor_name = get_node()->get_parameter("sensor_name").as_string();

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      sensor_name + "/fluid_pressure",
    }};
}

controller_interface::CallbackReturn PressureBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  topic_name_ = get_node()->get_parameter("topic_name").as_string();

  pressure_variance_ =
    get_node()->get_parameter("pressure_variance").as_double();

  if (pressure_variance_ < 0.0) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Invalid pressure_variance %.6f. Setting to 0.0",
      pressure_variance_);
    pressure_variance_ = 0.0;
  }

  publisher_ =
    get_node()->create_publisher<sensor_msgs::msg::FluidPressure>(
      topic_name_,
      rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured PressureBroadcaster: sensor_name='%s', frame_id='%s', topic='%s', "
    "pressure_variance=%.6f",
    sensor_name_.c_str(),
    frame_id_.c_str(),
    topic_name_.c_str(),
    pressure_variance_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PressureBroadcaster::on_activate(
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

  RCLCPP_INFO(get_node()->get_logger(), "PressureBroadcaster activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PressureBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (publisher_) {
    publisher_->on_deactivate();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PressureBroadcaster::update(
  const rclcpp::Time & time,
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
      "PressureBroadcaster expected 1 state interface, got %zu",
      state_interfaces_.size());
    return controller_interface::return_type::OK;
  }

  const double pressure_pa = state_interfaces_[0].get_value();

  if (!std::isfinite(pressure_pa) || pressure_pa <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Invalid pressure value: %.3f Pa",
      pressure_pa);
    return controller_interface::return_type::OK;
  }

  sensor_msgs::msg::FluidPressure msg;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id_;

  // Sin conversión: el dato ya viene en pascales.
  msg.fluid_pressure = pressure_pa;
  msg.variance = pressure_variance_;

  publisher_->publish(msg);

  return controller_interface::return_type::OK;
}

}  // namespace sura_sensors

PLUGINLIB_EXPORT_CLASS(
  sura_sensors::PressureBroadcaster,
  controller_interface::ControllerInterface)