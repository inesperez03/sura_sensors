#ifndef SURA_SENSORS_BROADCASTERS_DVL75_GPS_BROADCASTER_HPP_
#define SURA_SENSORS_BROADCASTERS_DVL75_GPS_BROADCASTER_HPP_

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace sura_sensors
{

class Dvl75GpsBroadcaster : public controller_interface::ControllerInterface
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

  double position_covariance_{0.0};

  rclcpp_lifecycle::LifecyclePublisher<
    sensor_msgs::msg::NavSatFix
  >::SharedPtr publisher_;
};

}  // namespace sura_sensors

#endif  // SURA_SENSORS_BROADCASTERS_DVL75_GPS_BROADCASTER_HPP_