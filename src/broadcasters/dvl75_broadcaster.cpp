#include "sura_sensors/broadcasters/dvl75_broadcaster.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace sura_sensors
{

controller_interface::CallbackReturn DvlBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("sensor_name", "dvl_sensor");
    auto_declare<std::string>("frame_id", "dvl_link");
    auto_declare<std::string>("topic_name", "~/twist");
  } catch (...) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
DvlBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
DvlBroadcaster::state_interface_configuration() const
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
    }};
}

controller_interface::CallbackReturn DvlBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  topic_name_ = get_node()->get_parameter("topic_name").as_string();

  publisher_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
    topic_name_, rclcpp::SystemDefaultsQoS());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DvlBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!publisher_) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != 6) {
    return controller_interface::CallbackReturn::ERROR;
  }

  publisher_->on_activate();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DvlBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (publisher_) {
    publisher_->on_deactivate();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DvlBroadcaster::update(
  const rclcpp::Time & time,
  const rclcpp::Duration &)
{
  if (!publisher_ || !publisher_->is_activated()) {
    return controller_interface::return_type::OK;
  }

  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id_;

  msg.twist.linear.x = state_interfaces_[0].get_value();
  msg.twist.linear.y = state_interfaces_[1].get_value();
  msg.twist.linear.z = state_interfaces_[2].get_value();

  msg.twist.angular.x = state_interfaces_[3].get_value();
  msg.twist.angular.y = state_interfaces_[4].get_value();
  msg.twist.angular.z = state_interfaces_[5].get_value();

  publisher_->publish(msg);

  return controller_interface::return_type::OK;
}

}  // namespace sura_sensors

PLUGINLIB_EXPORT_CLASS(
  sura_sensors::DvlBroadcaster,
  controller_interface::ControllerInterface)