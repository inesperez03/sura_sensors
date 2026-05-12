#include "sura_sensors/broadcasters/gps_broadcaster.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <exception>

namespace sura_sensors
{

controller_interface::CallbackReturn GpsBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("sensor_name", "gps_sensor");
    auto_declare<std::string>("frame_id", "gps_frame");
    auto_declare<std::string>("topic_name", "/sura/sensors/gps/fix");
    auto_declare<double>("position_covariance", 0.0);
  } catch (const std::exception &) {
    return controller_interface::CallbackReturn::ERROR;
  } catch (...) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GpsBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
GpsBroadcaster::state_interface_configuration() const
{
  const auto sensor_name = get_node()->get_parameter("sensor_name").as_string();

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      sensor_name + "/gps.latitude",
      sensor_name + "/gps.longitude",
      sensor_name + "/gps.altitude",
      sensor_name + "/gps.valid",
    }};
}

controller_interface::CallbackReturn GpsBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  topic_name_ = get_node()->get_parameter("topic_name").as_string();
  position_covariance_ = get_node()->get_parameter("position_covariance").as_double();

  if (!std::isfinite(position_covariance_) || position_covariance_ < 0.0) {
    position_covariance_ = 0.0;
  }

  publisher_ = get_node()->create_publisher<sensor_msgs::msg::NavSatFix>(
    topic_name_, rclcpp::SystemDefaultsQoS());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GpsBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!publisher_) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != 4) {
    return controller_interface::CallbackReturn::ERROR;
  }

  publisher_->on_activate();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GpsBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (publisher_) {
    publisher_->on_deactivate();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GpsBroadcaster::update(
  const rclcpp::Time & time,
  const rclcpp::Duration &)
{
  if (!publisher_ || !publisher_->is_activated()) {
    return controller_interface::return_type::OK;
  }

  if (state_interfaces_.size() != 4) {
    return controller_interface::return_type::OK;
  }

  const double latitude = state_interfaces_[0].get_value();
  const double longitude = state_interfaces_[1].get_value();
  const double altitude = state_interfaces_[2].get_value();
  const double valid = state_interfaces_[3].get_value();

  sensor_msgs::msg::NavSatFix msg;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id_;

  msg.latitude = latitude;
  msg.longitude = longitude;
  msg.altitude = altitude;

  msg.status.status =
    valid > 0.5 ?
    sensor_msgs::msg::NavSatStatus::STATUS_FIX :
    sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;

  msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  msg.position_covariance[0] = position_covariance_;
  msg.position_covariance[1] = 0.0;
  msg.position_covariance[2] = 0.0;
  msg.position_covariance[3] = 0.0;
  msg.position_covariance[4] = position_covariance_;
  msg.position_covariance[5] = 0.0;
  msg.position_covariance[6] = 0.0;
  msg.position_covariance[7] = 0.0;
  msg.position_covariance[8] = position_covariance_;
  msg.position_covariance_type =
    position_covariance_ > 0.0 ?
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED :
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

  publisher_->publish(msg);

  return controller_interface::return_type::OK;
}

}  // namespace sura_sensors

PLUGINLIB_EXPORT_CLASS(
  sura_sensors::GpsBroadcaster,
  controller_interface::ControllerInterface)
