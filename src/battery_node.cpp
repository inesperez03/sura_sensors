#include "sura_sensors/battery_node.hpp"

#include <chrono>
#include <limits>
#include <memory>

extern "C" {
#include "bindings.h"
}

using namespace std::chrono_literals;

static constexpr AdcChannel VOLTAGE_CHANNEL = AdcChannel::Ch3;
static constexpr AdcChannel CURRENT_CHANNEL = AdcChannel::Ch2;

static constexpr double PSM_VOLTAGE_MULTIPLIER = 11.0;
static constexpr double PSM_CURRENT_PER_VOLT = 37.8788;
static constexpr double PSM_CURRENT_OFFSET = 0.330;

BatteryNode::BatteryNode()
: Node("battery_node")
{
  init();

  battery_publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
    "/sura/sensors/battery", 10);

  timer_ = this->create_wall_timer(
    1000ms,
    std::bind(&BatteryNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Battery node started");
}

double BatteryNode::read_battery_voltage()
{
  const double adc_voltage = static_cast<double>(read_adc(VOLTAGE_CHANNEL));
  return adc_voltage * PSM_VOLTAGE_MULTIPLIER;
}

double BatteryNode::read_battery_current()
{
  const double adc_voltage = static_cast<double>(read_adc(CURRENT_CHANNEL));
  double current = (adc_voltage - PSM_CURRENT_OFFSET) * PSM_CURRENT_PER_VOLT;

  if (current < 0.0) {
    current = 0.0;
  }

  return current;
}

void BatteryNode::timer_callback()
{
  const double voltage = read_battery_voltage();
  const double current = read_battery_current();

  sensor_msgs::msg::BatteryState msg;
  msg.header.stamp = this->now();
  msg.voltage = static_cast<float>(voltage);
  msg.current = static_cast<float>(current);
  msg.percentage = std::numeric_limits<float>::quiet_NaN();
  msg.present = true;
  msg.power_supply_status =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  msg.power_supply_health =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  msg.power_supply_technology =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

  battery_publisher_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryNode>());
  rclcpp::shutdown();
  return 0;
}