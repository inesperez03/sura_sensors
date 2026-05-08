#include "sura_sensors/broadcasters/dvl75_vel_broadcaster.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <exception>
#include <string>

namespace sura_sensors
{

controller_interface::CallbackReturn DvlVelBroadcaster::on_init()
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
DvlVelBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
DvlVelBroadcaster::state_interface_configuration() const
{
  const auto sensor_name = get_node()->get_parameter("sensor_name").as_string();

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      sensor_name + "/linear_velocity.x",
      sensor_name + "/linear_velocity.y",
      sensor_name + "/linear_velocity.z",
      sensor_name + "/angular_velocity.x",
      sensor_name + "/angular_velocity.y",
      sensor_name + "/angular_velocity.z",
      sensor_name + "/confidence",
    }};
}

controller_interface::CallbackReturn DvlVelBroadcaster::on_configure(
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
      rclcpp::SystemDefaultsQoS());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DvlVelBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!publisher_) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != 7) {
    return controller_interface::CallbackReturn::ERROR;
  }

  publisher_->on_activate();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DvlVelBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (publisher_) {
    publisher_->on_deactivate();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DvlVelBroadcaster::update(
  const rclcpp::Time & time,
  const rclcpp::Duration &)
{
  if (!publisher_ || !publisher_->is_activated()) {
    return controller_interface::return_type::OK;
  }

  if (state_interfaces_.size() != 7) {
    return controller_interface::return_type::OK;
  }

  const double vx = state_interfaces_[0].get_value();
  const double vy = state_interfaces_[1].get_value();
  const double vz = state_interfaces_[2].get_value();

  const double wx = state_interfaces_[3].get_value();
  const double wy = state_interfaces_[4].get_value();
  const double wz = state_interfaces_[5].get_value();

  // Según la documentación del DVL-75, confidence viene en rango 0..100.
  const double confidence = state_interfaces_[6].get_value();
  const double confidence_normalized = confidence / 100.0;

  const double linear_velocity_covariance =
    min_linear_velocity_covariance_ +
    (1.0 - confidence_normalized) *
    (max_linear_velocity_covariance_ - min_linear_velocity_covariance_);

  geometry_msgs::msg::TwistWithCovarianceStamped msg;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id_;

  msg.twist.twist.linear.x = vx;
  msg.twist.twist.linear.y = vy;
  msg.twist.twist.linear.z = vz;

  msg.twist.twist.angular.x = wx;
  msg.twist.twist.angular.y = wy;
  msg.twist.twist.angular.z = wz;

  for (auto & value : msg.twist.covariance) {
    value = 0.0;
  }

  msg.twist.covariance[0] = linear_velocity_covariance;   // vx
  msg.twist.covariance[7] = linear_velocity_covariance;   // vy
  msg.twist.covariance[14] = linear_velocity_covariance;  // vz

  msg.twist.covariance[21] = angular_velocity_covariance_;  // wx
  msg.twist.covariance[28] = angular_velocity_covariance_;  // wy
  msg.twist.covariance[35] = angular_velocity_covariance_;  // wz

  publisher_->publish(msg);

  return controller_interface::return_type::OK;
}

}  // namespace sura_sensors

PLUGINLIB_EXPORT_CLASS(
  sura_sensors::DvlVelBroadcaster,
  controller_interface::ControllerInterface)