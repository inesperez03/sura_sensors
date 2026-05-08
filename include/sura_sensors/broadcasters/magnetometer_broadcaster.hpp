#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>

namespace sura_sensors
{

class MagnetometerBroadcaster : public controller_interface::ControllerInterface
{
public:
  MagnetometerBroadcaster() = default;

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
  std::string sensor_name_{"magnetometer_sensor"};
  std::string frame_id_{"imu_link"};
  std::string topic_name_{"~/magnetic_field"};
  std::string yaw_topic_name_{};
  double yaw_offset_rad_{0.0};

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::MagneticField>::SharedPtr publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr yaw_publisher_;
};

}  // namespace sura_sensors
