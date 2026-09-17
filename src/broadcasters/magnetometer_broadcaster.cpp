#include "sura_sensors/broadcasters/magnetometer_broadcaster.hpp"

#include <array>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sura_sensors
{

controller_interface::CallbackReturn MagnetometerBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("sensor_name", "magnetometer_sensor");
    auto_declare<std::string>("frame_id", "imu_link");
    auto_declare<std::string>("topic_name", "~/magnetic_field");
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
      sensor_name + "/sample_time.sec",
      sensor_name + "/sample_time.nanosec",
    }};
}

controller_interface::CallbackReturn MagnetometerBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  topic_name_ = get_node()->get_parameter("topic_name").as_string();

  publisher_ = get_node()->create_publisher<sensor_msgs::msg::MagneticField>(
    topic_name_, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured MagnetometerBroadcaster: sensor_name='%s', frame_id='%s', topic='%s'",
    sensor_name_.c_str(),
    frame_id_.c_str(),
    topic_name_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MagnetometerBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!publisher_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Publisher is null");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != 5) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected 5 state interfaces, got %zu",
      state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  publisher_->on_activate();

  RCLCPP_INFO(get_node()->get_logger(), "MagnetometerBroadcaster activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MagnetometerBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (publisher_) {
    publisher_->on_deactivate();
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

  if (state_interfaces_.size() != 5) {
    return controller_interface::return_type::OK;
  }

  sensor_msgs::msg::MagneticField msg;
  msg.header.stamp.sec = static_cast<int32_t>(state_interfaces_[3].get_value());
  msg.header.stamp.nanosec = static_cast<uint32_t>(state_interfaces_[4].get_value());
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

  msg.magnetic_field.x = state_interfaces_[0].get_value();
  msg.magnetic_field.y = state_interfaces_[1].get_value();
  msg.magnetic_field.z = state_interfaces_[2].get_value();

  publisher_->publish(msg);
  return controller_interface::return_type::OK;
}

}  // namespace sura_sensors

PLUGINLIB_EXPORT_CLASS(
  sura_sensors::MagnetometerBroadcaster,
  controller_interface::ControllerInterface)
