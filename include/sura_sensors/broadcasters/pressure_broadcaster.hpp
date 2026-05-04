#ifndef SURA_SENSORS_BROADCASTERS_PRESSURE_BROADCASTER_HPP_
#define SURA_SENSORS_BROADCASTERS_PRESSURE_BROADCASTER_HPP_

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

namespace sura_sensors
{

class PressureBroadcaster : public controller_interface::ControllerInterface
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

  double surface_pressure_mbar_{1000.0};
  double fluid_density_{997.0};
  double gravity_{9.80665};

  bool auto_calibrate_surface_pressure_{true};
  bool surface_pressure_calibrated_{false};

  double alpha_{0.1};
  bool depth_filter_initialized_{false};
  double filtered_depth_m_{0.0};

  double z_covariance_{0.02};
  double unused_covariance_{99999.0};

  rclcpp_lifecycle::LifecyclePublisher<
    geometry_msgs::msg::PoseWithCovarianceStamped
  >::SharedPtr publisher_;
};

}  // namespace sura_sensors

#endif  // SURA_SENSORS_BROADCASTERS_PRESSURE_BROADCASTER_HPP_