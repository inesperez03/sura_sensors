#include "sura_sensors/broadcasters/imu_broadcaster.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace sura_sensors
{

controller_interface::CallbackReturn ImuBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("sensor_name", "imu_sensor");
    auto_declare<std::string>("frame_id", "imu_link");
    auto_declare<std::string>("topic_name", "~/imu");
  } catch (...) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ImuBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
ImuBroadcaster::state_interface_configuration() const
{
  const auto sensor_name = get_node()->get_parameter("sensor_name").as_string();

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      sensor_name + "/orientation.x",
      sensor_name + "/orientation.y",
      sensor_name + "/orientation.z",
      sensor_name + "/orientation.w",
      sensor_name + "/angular_velocity.x",
      sensor_name + "/angular_velocity.y",
      sensor_name + "/angular_velocity.z",
      sensor_name + "/linear_acceleration.x",
      sensor_name + "/linear_acceleration.y",
      sensor_name + "/linear_acceleration.z",
      sensor_name + "/sample_time.sec",
      sensor_name + "/sample_time.nanosec",
    }};
}

controller_interface::CallbackReturn ImuBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  topic_name_ = get_node()->get_parameter("topic_name").as_string();

  publisher_ = get_node()->create_publisher<sensor_msgs::msg::Imu>(
    topic_name_, rclcpp::SystemDefaultsQoS());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImuBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!publisher_) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != 12) {
    return controller_interface::CallbackReturn::ERROR;
  }

  publisher_->on_activate();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImuBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (publisher_) {
    publisher_->on_deactivate();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ImuBroadcaster::update(
  const rclcpp::Time & time,
  const rclcpp::Duration &)
{
  if (!publisher_ || !publisher_->is_activated()) {
    return controller_interface::return_type::OK;
  }

  if (state_interfaces_.size() != 12) {
    return controller_interface::return_type::OK;
  }

  sensor_msgs::msg::Imu msg;
  msg.header.stamp.sec = static_cast<int32_t>(state_interfaces_[10].get_value());
  msg.header.stamp.nanosec = static_cast<uint32_t>(state_interfaces_[11].get_value());
  if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) {
    msg.header.stamp = time;
  }

  if (
    has_last_sample_time_ &&
    msg.header.stamp.sec == last_sample_time_sec_ &&
    msg.header.stamp.nanosec == last_sample_time_nanosec_)
  {
    return controller_interface::return_type::OK;
  }

  has_last_sample_time_ = true;
  last_sample_time_sec_ = msg.header.stamp.sec;
  last_sample_time_nanosec_ = msg.header.stamp.nanosec;

  msg.header.frame_id = frame_id_;

  msg.orientation.x = state_interfaces_[0].get_value();
  msg.orientation.y = state_interfaces_[1].get_value();
  msg.orientation.z = state_interfaces_[2].get_value();
  msg.orientation.w = state_interfaces_[3].get_value();

  msg.angular_velocity.x = state_interfaces_[4].get_value();
  msg.angular_velocity.y = state_interfaces_[5].get_value();
  msg.angular_velocity.z = state_interfaces_[6].get_value();

  msg.linear_acceleration.x = state_interfaces_[7].get_value();
  msg.linear_acceleration.y = state_interfaces_[8].get_value();
  msg.linear_acceleration.z = state_interfaces_[9].get_value();

  publisher_->publish(msg);
  return controller_interface::return_type::OK;
}

}  // namespace sura_sensors

PLUGINLIB_EXPORT_CLASS(
  sura_sensors::ImuBroadcaster,
  controller_interface::ControllerInterface)
