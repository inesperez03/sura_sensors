#include "sura_sensors/broadcasters/pressure_broadcaster.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <string>

namespace sura_sensors
{

controller_interface::CallbackReturn PressureBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("sensor_name", "pressure_sensor");
    auto_declare<std::string>("frame_id", "bluerov2/base_link");
    auto_declare<std::string>("topic_name", "/pressure_broadcaster/pose");

    auto_declare<double>("surface_pressure_mbar", 1000.0);
    auto_declare<double>("fluid_density", 997.0);
    auto_declare<double>("gravity", 9.80665);
    auto_declare<bool>("auto_calibrate_surface_pressure", true);
    auto_declare<double>("filter_alpha", 0.1);

    auto_declare<double>("z_covariance", 0.02);
    auto_declare<double>("unused_covariance", 99999.0);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception in PressureBroadcaster::on_init(): %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  } catch (...) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Unknown exception in PressureBroadcaster::on_init()");
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PressureBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
PressureBroadcaster::state_interface_configuration() const
{
  const auto sensor_name = get_node()->get_parameter("sensor_name").as_string();

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {
      sensor_name + "/fluid_pressure",
    }};
}

controller_interface::CallbackReturn PressureBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  topic_name_ = get_node()->get_parameter("topic_name").as_string();

  surface_pressure_mbar_ =
    get_node()->get_parameter("surface_pressure_mbar").as_double();

  fluid_density_ =
    get_node()->get_parameter("fluid_density").as_double();

  gravity_ =
    get_node()->get_parameter("gravity").as_double();

  auto_calibrate_surface_pressure_ =
    get_node()->get_parameter("auto_calibrate_surface_pressure").as_bool();

  alpha_ =
    get_node()->get_parameter("filter_alpha").as_double();

  z_covariance_ =
    get_node()->get_parameter("z_covariance").as_double();

  unused_covariance_ =
    get_node()->get_parameter("unused_covariance").as_double();

  if (fluid_density_ <= 0.0) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Invalid fluid_density: %.3f",
      fluid_density_);
    return controller_interface::CallbackReturn::ERROR;
  }

  if (gravity_ <= 0.0) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Invalid gravity: %.5f",
      gravity_);
    return controller_interface::CallbackReturn::ERROR;
  }

  if (alpha_ <= 0.0 || alpha_ > 1.0) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Invalid filter_alpha %.3f. Clamping to 0.1",
      alpha_);
    alpha_ = 0.1;
  }

  if (z_covariance_ <= 0.0) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Invalid z_covariance %.6f. Setting to 0.02",
      z_covariance_);
    z_covariance_ = 0.02;
  }

  if (unused_covariance_ <= 0.0) {
    unused_covariance_ = 99999.0;
  }

  surface_pressure_calibrated_ = !auto_calibrate_surface_pressure_;
  depth_filter_initialized_ = false;
  filtered_depth_m_ = 0.0;

  publisher_ =
    get_node()->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      topic_name_,
      rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured PressureBroadcaster: sensor_name='%s', frame_id='%s', topic='%s'",
    sensor_name_.c_str(),
    frame_id_.c_str(),
    topic_name_.c_str());

  RCLCPP_INFO(
    get_node()->get_logger(),
    "PressureBroadcaster params: surface_pressure_mbar=%.3f, auto_calibrate=%s, "
    "fluid_density=%.3f, gravity=%.5f, filter_alpha=%.3f, z_covariance=%.6f",
    surface_pressure_mbar_,
    auto_calibrate_surface_pressure_ ? "true" : "false",
    fluid_density_,
    gravity_,
    alpha_,
    z_covariance_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PressureBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!publisher_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Publisher is null");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != 1) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected 1 state interface, got %zu",
      state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  publisher_->on_activate();

  RCLCPP_INFO(get_node()->get_logger(), "PressureBroadcaster activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PressureBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (publisher_) {
    publisher_->on_deactivate();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PressureBroadcaster::update(
  const rclcpp::Time & time,
  const rclcpp::Duration &)
{
  if (!publisher_ || !publisher_->is_activated()) {
    return controller_interface::return_type::OK;
  }

  if (state_interfaces_.size() != 1) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "PressureBroadcaster expected 1 state interface, got %zu",
      state_interfaces_.size());
    return controller_interface::return_type::OK;
  }

  const double pressure_mbar = state_interfaces_[0].get_value();

  if (!std::isfinite(pressure_mbar) || pressure_mbar <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Invalid pressure value: %.3f mbar",
      pressure_mbar);
    return controller_interface::return_type::OK;
  }

  if (auto_calibrate_surface_pressure_ && !surface_pressure_calibrated_) {
    surface_pressure_mbar_ = pressure_mbar;
    surface_pressure_calibrated_ = true;

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Auto-calibrated surface pressure: %.3f mbar",
      surface_pressure_mbar_);
  }

  const double delta_pressure_pa =
    (pressure_mbar - surface_pressure_mbar_) * 100.0;

  const double depth_m =
    delta_pressure_pa / (fluid_density_ * gravity_);

  if (!depth_filter_initialized_) {
    filtered_depth_m_ = depth_m;
    depth_filter_initialized_ = true;
  } else {
    filtered_depth_m_ =
      alpha_ * depth_m + (1.0 - alpha_) * filtered_depth_m_;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id_;

  msg.pose.pose.position.x = 0.0;
  msg.pose.pose.position.y = 0.0;
  msg.pose.pose.position.z = filtered_depth_m_;

  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = 0.0;
  msg.pose.pose.orientation.w = 1.0;

  for (auto & value : msg.pose.covariance) {
    value = 0.0;
  }

  msg.pose.covariance[0] = unused_covariance_;
  msg.pose.covariance[7] = unused_covariance_;
  msg.pose.covariance[14] = z_covariance_;
  msg.pose.covariance[21] = unused_covariance_;
  msg.pose.covariance[28] = unused_covariance_;
  msg.pose.covariance[35] = unused_covariance_;

  publisher_->publish(msg);

  return controller_interface::return_type::OK;
}

}  // namespace sura_sensors

PLUGINLIB_EXPORT_CLASS(
  sura_sensors::PressureBroadcaster,
  controller_interface::ControllerInterface)