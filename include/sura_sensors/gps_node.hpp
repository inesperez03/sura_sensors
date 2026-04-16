#ifndef SURA_SENSORS__GPS_NODE_HPP_
#define SURA_SENSORS__GPS_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GpsNode : public rclcpp::Node
{
public:
  GpsNode();
  ~GpsNode();

private:
  bool open_serial();
  bool read_line(std::string & line);
  bool parse_gga(const std::string & line, sensor_msgs::msg::NavSatFix & msg) const;
  double nmea_to_decimal(const std::string & value, char direction) const;
  void timer_callback();

  int fd_;
  std::string buffer_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // SURA_SENSORS__GPS_NODE_HPP_