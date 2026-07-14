#include "sura_sensors/broadcasters/dvla50_twist_stamped_broadcaster.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>

namespace sura_sensors
{

controller_interface::CallbackReturn DvlA50TwistStampedBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("sensor_name", "dvl_sensor");
    auto_declare<std::string>("frame_id", "dvl_link");
    auto_declare<std::string>("topic_name", "~/twist");

    auto_declare<double>("min_linear_velocity_covariance", 0.01);
    auto_declare<double>("max_linear_velocity_covariance", 999.0);
    auto_declare<double>("angular_velocity_covariance", 99999.0);
  } catch (const std::exception &) {
    return controller_interface::CallbackReturn::ERROR;
  } catch (...) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
DvlA50TwistStampedBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
DvlA50TwistStampedBroadcaster::state_interface_configuration() const
{
  const auto sensor_name = get_node()->get_parameter("sensor_name").as_string();

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      sensor_name + "/linear_velocity.x",
      sensor_name + "/linear_velocity.y",
      sensor_name + "/linear_velocity.z",
      sensor_name + "/fom",
    }};
}

controller_interface::CallbackReturn DvlA50TwistStampedBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  topic_name_ = get_node()->get_parameter("topic_name").as_string();

  min_linear_velocity_covariance_ =
    get_node()->get_parameter("min_linear_velocity_covariance").as_double();
  max_linear_velocity_covariance_ =
    get_node()->get_parameter("max_linear_velocity_covariance").as_double();
  angular_velocity_covariance_ =
    get_node()->get_parameter("angular_velocity_covariance").as_double();

  if (!std::isfinite(min_linear_velocity_covariance_) ||
      min_linear_velocity_covariance_ < 0.0)
  {
    min_linear_velocity_covariance_ = 0.01;
  }

  if (!std::isfinite(max_linear_velocity_covariance_) ||
      max_linear_velocity_covariance_ < min_linear_velocity_covariance_)
  {
    max_linear_velocity_covariance_ = min_linear_velocity_covariance_;
  }

  if (!std::isfinite(angular_velocity_covariance_) ||
      angular_velocity_covariance_ < 0.0)
  {
    angular_velocity_covariance_ = 99999.0;
  }

  publisher_ =
    get_node()->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      topic_name_,
      rclcpp::SensorDataQoS());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DvlA50TwistStampedBroadcaster::on_activate(
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

controller_interface::CallbackReturn DvlA50TwistStampedBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (publisher_) {
    publisher_->on_deactivate();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DvlA50TwistStampedBroadcaster::update(
  const rclcpp::Time & time,
  const rclcpp::Duration &)
{
  if (!publisher_ || !publisher_->is_activated()) {
    return controller_interface::return_type::OK;
  }

  if (state_interfaces_.size() != 4) {
    return controller_interface::return_type::OK;
  }

  const double vx = state_interfaces_[0].get_value();
  const double vy = state_interfaces_[1].get_value();
  const double vz = state_interfaces_[2].get_value();
  const double fom = state_interfaces_[3].get_value();

  const double fom_covariance = std::isfinite(fom) && fom >= 0.0 ? fom * fom :
    max_linear_velocity_covariance_;
  const double linear_velocity_covariance = std::clamp(
    fom_covariance,
    min_linear_velocity_covariance_,
    max_linear_velocity_covariance_);

  geometry_msgs::msg::TwistWithCovarianceStamped msg;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id_;

  msg.twist.twist.linear.x = vx;
  msg.twist.twist.linear.y = vy;
  msg.twist.twist.linear.z = vz;

  msg.twist.twist.angular.x = 0.0;
  msg.twist.twist.angular.y = 0.0;
  msg.twist.twist.angular.z = 0.0;

  for (auto & value : msg.twist.covariance) {
    value = 0.0;
  }

  msg.twist.covariance[0] = linear_velocity_covariance;
  msg.twist.covariance[7] = linear_velocity_covariance;
  msg.twist.covariance[14] = linear_velocity_covariance;

  msg.twist.covariance[21] = angular_velocity_covariance_;
  msg.twist.covariance[28] = angular_velocity_covariance_;
  msg.twist.covariance[35] = angular_velocity_covariance_;

  publisher_->publish(msg);
  return controller_interface::return_type::OK;
}

}  // namespace sura_sensors

PLUGINLIB_EXPORT_CLASS(
  sura_sensors::DvlA50TwistStampedBroadcaster,
  controller_interface::ControllerInterface)
