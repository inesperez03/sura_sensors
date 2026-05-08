#include "sura_sensors/broadcasters/dvl75_altitude_broadcaster.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace sura_sensors
{

controller_interface::CallbackReturn Dvl75AltitudeBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("sensor_name", "dvl_sensor");
    auto_declare<std::string>("frame_id", "bluerov2/dvl_link");
    auto_declare<std::string>("topic_name", "~/altitude");

    auto_declare<double>("field_of_view", 0.0);
    auto_declare<double>("min_range", 0.05);
    auto_declare<double>("max_range", 50.0);
  } catch (...) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
Dvl75AltitudeBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
Dvl75AltitudeBroadcaster::state_interface_configuration() const
{
  const auto sensor_name = get_node()->get_parameter("sensor_name").as_string();

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      sensor_name + "/distance_z",
    }};
}

controller_interface::CallbackReturn Dvl75AltitudeBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  topic_name_ = get_node()->get_parameter("topic_name").as_string();

  field_of_view_ = get_node()->get_parameter("field_of_view").as_double();
  min_range_ = get_node()->get_parameter("min_range").as_double();
  max_range_ = get_node()->get_parameter("max_range").as_double();

  publisher_ = get_node()->create_publisher<sensor_msgs::msg::Range>(
    topic_name_, rclcpp::SystemDefaultsQoS());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Dvl75AltitudeBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!publisher_) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != 1) {
    return controller_interface::CallbackReturn::ERROR;
  }

  publisher_->on_activate();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Dvl75AltitudeBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (publisher_) {
    publisher_->on_deactivate();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Dvl75AltitudeBroadcaster::update(
  const rclcpp::Time & time,
  const rclcpp::Duration &)
{
  if (!publisher_ || !publisher_->is_activated()) {
    return controller_interface::return_type::OK;
  }

  const double altitude = state_interfaces_[0].get_value();

  sensor_msgs::msg::Range msg;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id_;

  msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  msg.field_of_view = field_of_view_;
  msg.min_range = min_range_;
  msg.max_range = max_range_;
  msg.range = altitude;

  publisher_->publish(msg);

  return controller_interface::return_type::OK;
}

}  // namespace sura_sensors

PLUGINLIB_EXPORT_CLASS(
  sura_sensors::Dvl75AltitudeBroadcaster,
  controller_interface::ControllerInterface)
