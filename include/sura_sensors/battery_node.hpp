#ifndef SURA_SENSORS__BATTERY_NODE_HPP_
#define SURA_SENSORS__BATTERY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

class BatteryNode : public rclcpp::Node
{
public:
  BatteryNode();

private:
  void timer_callback();
  double read_battery_voltage();
  double read_battery_current();

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // SURA_SENSORS__BATTERY_NODE_HPP_