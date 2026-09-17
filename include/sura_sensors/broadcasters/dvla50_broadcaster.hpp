#pragma once

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <sura_msgs/msg/dvl.hpp>

namespace sura_sensors
{

class DvlA50Broadcaster : public controller_interface::ControllerInterface
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

  rclcpp_lifecycle::LifecyclePublisher<sura_msgs::msg::DVL>::SharedPtr publisher_;

  bool has_last_logged_publish_{false};
  double last_logged_vx_{0.0};
  double last_logged_vy_{0.0};
  double last_logged_vz_{0.0};
  double last_logged_time_{0.0};
};

}  // namespace sura_sensors
