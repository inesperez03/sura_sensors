#include "sura_sensors/broadcasters/magnetometer_broadcaster.hpp"

#include <array>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sura_sensors
{

namespace
{

double normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

controller_interface::CallbackReturn MagnetometerBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("sensor_name", "magnetometer_sensor");
    auto_declare<std::string>("frame_id", "imu_link");
    auto_declare<std::string>("topic_name", "~/magnetic_field");
    auto_declare<std::string>("yaw_topic_name", "");
    auto_declare<double>("yaw_offset_rad", 0.0);
  } catch (...) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MagnetometerBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
MagnetometerBroadcaster::state_interface_configuration() const
{
  const auto sensor_name = get_node()->get_parameter("sensor_name").as_string();

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      sensor_name + "/magnetic_field.x",
      sensor_name + "/magnetic_field.y",
      sensor_name + "/magnetic_field.z",
    }};
}

controller_interface::CallbackReturn MagnetometerBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  topic_name_ = get_node()->get_parameter("topic_name").as_string();
  yaw_topic_name_ = get_node()->get_parameter("yaw_topic_name").as_string();
  yaw_offset_rad_ = get_node()->get_parameter("yaw_offset_rad").as_double();

  publisher_ = get_node()->create_publisher<sensor_msgs::msg::MagneticField>(
    topic_name_, rclcpp::SystemDefaultsQoS());

  if (!yaw_topic_name_.empty()) {
    yaw_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>(
      yaw_topic_name_, rclcpp::SystemDefaultsQoS());
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured MagnetometerBroadcaster: sensor_name='%s', frame_id='%s', topic='%s', yaw_topic='%s', yaw_offset=%.3f rad",
    sensor_name_.c_str(),
    frame_id_.c_str(),
    topic_name_.c_str(),
    yaw_topic_name_.empty() ? "<disabled>" : yaw_topic_name_.c_str(),
    yaw_offset_rad_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MagnetometerBroadcaster::on_activate(
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
  if (yaw_publisher_) {
    yaw_publisher_->on_activate();
  }

  RCLCPP_INFO(get_node()->get_logger(), "MagnetometerBroadcaster activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MagnetometerBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (publisher_) {
    publisher_->on_deactivate();
  }
  if (yaw_publisher_) {
    yaw_publisher_->on_deactivate();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MagnetometerBroadcaster::update(
  const rclcpp::Time & time,
  const rclcpp::Duration &)
{
  if (!publisher_ || !publisher_->is_activated()) {
    return controller_interface::return_type::OK;
  }

  sensor_msgs::msg::MagneticField msg;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id_;

  msg.magnetic_field.x = state_interfaces_[0].get_value();
  msg.magnetic_field.y = state_interfaces_[1].get_value();
  msg.magnetic_field.z = state_interfaces_[2].get_value();

  publisher_->publish(msg);

  if (yaw_publisher_ && yaw_publisher_->is_activated()) {
    std_msgs::msg::Float64 yaw_msg;
    yaw_msg.data = normalize_angle(
      std::atan2(msg.magnetic_field.y, msg.magnetic_field.x) + yaw_offset_rad_);
    yaw_publisher_->publish(yaw_msg);
  }

  return controller_interface::return_type::OK;
}

}  // namespace sura_sensors

PLUGINLIB_EXPORT_CLASS(
  sura_sensors::MagnetometerBroadcaster,
  controller_interface::ControllerInterface)
