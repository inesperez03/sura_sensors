#ifndef SURA_SENSORS__GPS_NODE_HPP_
#define SURA_SENSORS__GPS_NODE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GpsNode : public rclcpp::Node
{
public:
  GpsNode();
  ~GpsNode();

private:
  bool open_serial();
  bool read_from_serial();

  bool read_line(std::string & line);

  bool parse_gga(
    const std::string & line,
    sensor_msgs::msg::NavSatFix & msg) const;

  double nmea_to_decimal(
    const std::string & value,
    char direction) const;

  bool try_extract_ubx_packet(std::vector<uint8_t> & packet);

  bool parse_nav_pvt(
    const std::vector<uint8_t> & packet,
    sensor_msgs::msg::NavSatFix & msg) const;

  void timer_callback();

  int fd_;

  std::string serial_port_;
  int baudrate_;
  std::string protocol_;
  std::string frame_id_;
  std::string fix_topic_;

  std::string line_buffer_;
  std::vector<uint8_t> byte_buffer_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // SURA_SENSORS__GPS_NODE_HPP_