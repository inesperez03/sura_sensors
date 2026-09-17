#pragma once

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace sura_sensors
{

class DvlA50TwistStampedBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  std::string sensor_name_;
  std::string frame_id_;
  std::string topic_name_;

  double min_linear_velocity_covariance_{0.01};
  double max_linear_velocity_covariance_{999.0};
  double angular_velocity_covariance_{99999.0};

  rclcpp_lifecycle::LifecyclePublisher<
    geometry_msgs::msg::TwistWithCovarianceStamped
  >::SharedPtr publisher_;

  bool has_last_logged_publish_{false};
  double last_logged_vx_{0.0};
  double last_logged_vy_{0.0};
  double last_logged_vz_{0.0};
};

}  // namespace sura_sensors
