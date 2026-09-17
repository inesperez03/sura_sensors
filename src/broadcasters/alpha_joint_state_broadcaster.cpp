#include "sura_sensors/broadcasters/alpha_joint_state_broadcaster.hpp"

#include <algorithm>
#include <exception>
#include <functional>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <vector>

namespace sura_sensors
{

controller_interface::CallbackReturn AlphaJointStateBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("input_topic", "/joint_states");
    auto_declare<std::string>("output_topic", "~/joint_states");
    auto_declare<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>(
      "joint_name_substrings", std::vector<std::string>{"alpha_left/", "alpha_right/"});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception in AlphaJointStateBroadcaster::on_init(): %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Unknown exception in AlphaJointStateBroadcaster::on_init()");
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
AlphaJointStateBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
AlphaJointStateBroadcaster::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn AlphaJointStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  input_topic_ = get_node()->get_parameter("input_topic").as_string();
  output_topic_ = get_node()->get_parameter("output_topic").as_string();
  joint_names_ = get_node()->get_parameter("joint_names").as_string_array();
  joint_name_substrings_ =
    get_node()->get_parameter("joint_name_substrings").as_string_array();
  joint_name_set_ = std::unordered_set<std::string>(joint_names_.begin(), joint_names_.end());

  publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    output_topic_, rclcpp::SystemDefaultsQoS());
  subscription_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    input_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&AlphaJointStateBroadcaster::joint_state_callback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured AlphaJointStateBroadcaster: input_topic='%s', output_topic='%s'",
    input_topic_.c_str(),
    output_topic_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AlphaJointStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!publisher_ || !subscription_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Publisher or subscription is null");
    return controller_interface::CallbackReturn::ERROR;
  }

  publisher_->on_activate();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AlphaJointStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (publisher_) {
    publisher_->on_deactivate();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AlphaJointStateBroadcaster::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  return controller_interface::return_type::OK;
}

bool AlphaJointStateBroadcaster::should_publish_joint(const std::string & joint_name) const
{
  if (!joint_name_set_.empty()) {
    return joint_name_set_.count(joint_name) > 0;
  }

  return std::any_of(
    joint_name_substrings_.begin(), joint_name_substrings_.end(),
    [&joint_name](const std::string & substring) {
      return !substring.empty() && joint_name.find(substring) != std::string::npos;
    });
}

void AlphaJointStateBroadcaster::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!publisher_ || !publisher_->is_activated()) {
    return;
  }

  sensor_msgs::msg::JointState filtered_msg;
  filtered_msg.header = msg->header;

  const bool has_positions = msg->position.size() == msg->name.size();
  const bool has_velocities = msg->velocity.size() == msg->name.size();
  const bool has_efforts = msg->effort.size() == msg->name.size();

  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (!should_publish_joint(msg->name[i])) {
      continue;
    }

    filtered_msg.name.push_back(msg->name[i]);
    if (has_positions) {
      filtered_msg.position.push_back(msg->position[i]);
    }
    if (has_velocities) {
      filtered_msg.velocity.push_back(msg->velocity[i]);
    }
    if (has_efforts) {
      filtered_msg.effort.push_back(msg->effort[i]);
    }
  }

  if (filtered_msg.name.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "No alpha joints found in incoming JointState message");
    return;
  }

  publisher_->publish(filtered_msg);
}

}  // namespace sura_sensors

PLUGINLIB_EXPORT_CLASS(
  sura_sensors::AlphaJointStateBroadcaster,
  controller_interface::ControllerInterface)
