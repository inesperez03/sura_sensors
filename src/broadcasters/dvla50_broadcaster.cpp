#include "sura_sensors/broadcasters/dvla50_broadcaster.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <exception>
#include <string>
#include <vector>

#include "sura_msgs/msg/dvl_beam.hpp"

namespace sura_sensors
{

namespace
{

std::vector<std::string> dvl_a50_state_names(const std::string & sensor_name)
{
  std::vector<std::string> names = {
    sensor_name + "/linear_velocity.x",
    sensor_name + "/linear_velocity.y",
    sensor_name + "/linear_velocity.z",
    sensor_name + "/fom",
    sensor_name + "/altitude",
    sensor_name + "/velocity_valid",
    sensor_name + "/status",
    sensor_name + "/time",
    sensor_name + "/format_code",
  };

  for (size_t i = 0; i < 4; ++i) {
    const std::string prefix = sensor_name + "/beam" + std::to_string(i) + ".";
    names.push_back(prefix + "id");
    names.push_back(prefix + "velocity");
    names.push_back(prefix + "distance");
    names.push_back(prefix + "rssi");
    names.push_back(prefix + "nsd");
    names.push_back(prefix + "valid");
  }

  return names;
}

}  // namespace

controller_interface::CallbackReturn DvlA50Broadcaster::on_init()
{
  try {
    auto_declare<std::string>("sensor_name", "dvl_sensor");
    auto_declare<std::string>("frame_id", "dvl_link");
    auto_declare<std::string>("topic_name", "~/data");
  } catch (const std::exception &) {
    return controller_interface::CallbackReturn::ERROR;
  } catch (...) {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
DvlA50Broadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
DvlA50Broadcaster::state_interface_configuration() const
{
  const auto sensor_name = get_node()->get_parameter("sensor_name").as_string();
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    dvl_a50_state_names(sensor_name)};
}

controller_interface::CallbackReturn DvlA50Broadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  topic_name_ = get_node()->get_parameter("topic_name").as_string();

  publisher_ = get_node()->create_publisher<sura_msgs::msg::DVL>(
    topic_name_, rclcpp::SensorDataQoS());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DvlA50Broadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!publisher_) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != 33) {
    return controller_interface::CallbackReturn::ERROR;
  }

  publisher_->on_activate();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DvlA50Broadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (publisher_) {
    publisher_->on_deactivate();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DvlA50Broadcaster::update(
  const rclcpp::Time & time,
  const rclcpp::Duration &)
{
  if (!publisher_ || !publisher_->is_activated()) {
    return controller_interface::return_type::OK;
  }

  if (state_interfaces_.size() != 33) {
    return controller_interface::return_type::OK;
  }

  sura_msgs::msg::DVL msg;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id_;

  msg.velocity.x = state_interfaces_[0].get_value();
  msg.velocity.y = state_interfaces_[1].get_value();
  msg.velocity.z = state_interfaces_[2].get_value();
  msg.fom = state_interfaces_[3].get_value();
  msg.altitude = state_interfaces_[4].get_value();
  msg.velocity_valid = state_interfaces_[5].get_value() > 0.5;
  msg.status = static_cast<int64_t>(std::llround(state_interfaces_[6].get_value()));
  msg.time = state_interfaces_[7].get_value();
  msg.form = "json";

  msg.beams.reserve(4);
  size_t index = 9;
  for (size_t i = 0; i < 4; ++i) {
    sura_msgs::msg::DVLBeam beam;
    beam.id = static_cast<int64_t>(std::llround(state_interfaces_[index++].get_value()));
    beam.velocity = state_interfaces_[index++].get_value();
    beam.distance = state_interfaces_[index++].get_value();
    beam.rssi = state_interfaces_[index++].get_value();
    beam.nsd = state_interfaces_[index++].get_value();
    beam.valid = state_interfaces_[index++].get_value() > 0.5;
    msg.beams.push_back(beam);
  }

  publisher_->publish(msg);
  return controller_interface::return_type::OK;
}

}  // namespace sura_sensors

PLUGINLIB_EXPORT_CLASS(
  sura_sensors::DvlA50Broadcaster,
  controller_interface::ControllerInterface)
