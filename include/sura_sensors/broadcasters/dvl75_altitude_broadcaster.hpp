#pragma once

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <sensor_msgs/msg/range.hpp>

namespace sura_sensors
{

class Dvl75AltitudeBroadcaster : public controller_interface::ControllerInterface
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

  double field_of_view_{0.0};
  double min_range_{0.05};
  double max_range_{50.0};

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
};

}  // namespace sura_sensors
