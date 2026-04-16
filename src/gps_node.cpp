#include "sura_sensors/gps_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <limits>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

namespace
{
constexpr const char * SERIAL_PORT = "/dev/serial0";
constexpr speed_t BAUD_RATE = B9600;
constexpr auto TIMER_PERIOD = 100ms;

std::array<std::string, 15> split_gga(const std::string & line)
{
  std::array<std::string, 15> fields{};
  std::stringstream ss(line);
  std::string item;
  std::size_t i = 0;

  while (std::getline(ss, item, ',') && i < fields.size()) {
    fields[i++] = item;
  }

  return fields;
}
}  // namespace

GpsNode::GpsNode()
: Node("gps_node"), fd_(-1)
{
  gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
    "/sura/sensors/gps/fix", 10);

  if (!open_serial()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open %s", SERIAL_PORT);
  }

  timer_ = this->create_wall_timer(
    TIMER_PERIOD,
    std::bind(&GpsNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "GPS node started");
}

GpsNode::~GpsNode()
{
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool GpsNode::open_serial()
{
  fd_ = open(SERIAL_PORT, O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    close(fd_);
    fd_ = -1;
    return false;
  }

  cfsetispeed(&tty, BAUD_RATE);
  cfsetospeed(&tty, BAUD_RATE);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_iflag = 0;
  tty.c_oflag = 0;
  tty.c_lflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  tcflush(fd_, TCIFLUSH);

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close(fd_);
    fd_ = -1;
    return false;
  }

  return true;
}

bool GpsNode::read_line(std::string & line)
{
  if (fd_ < 0) {
    return false;
  }

  char temp[256];
  const ssize_t bytes_read = read(fd_, temp, sizeof(temp));

  if (bytes_read > 0) {
    buffer_.append(temp, static_cast<std::size_t>(bytes_read));
  }

  const std::size_t newline_pos = buffer_.find('\n');
  if (newline_pos == std::string::npos) {
    return false;
  }

  line = buffer_.substr(0, newline_pos);
  buffer_.erase(0, newline_pos + 1);

  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }

  return true;
}

double GpsNode::nmea_to_decimal(const std::string & value, char direction) const
{
  if (value.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double raw = std::stod(value);
  const int degrees = static_cast<int>(raw / 100.0);
  const double minutes = raw - static_cast<double>(degrees * 100);
  double decimal = static_cast<double>(degrees) + minutes / 60.0;

  if (direction == 'S' || direction == 'W') {
    decimal = -decimal;
  }

  return decimal;
}

bool GpsNode::parse_gga(const std::string & line, sensor_msgs::msg::NavSatFix & msg) const
{
  if (line.rfind("$GPGGA", 0) != 0 && line.rfind("$GNGGA", 0) != 0) {
    return false;
  }

  const auto fields = split_gga(line);

  if (fields[2].empty() || fields[3].empty() || fields[4].empty() ||
      fields[5].empty() || fields[6].empty()) {
    return false;
  }

  try {
    const int fix_quality = std::stoi(fields[6]);

    msg.status.status =
      (fix_quality == 0) ?
      sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX :
      sensor_msgs::msg::NavSatStatus::STATUS_FIX;

    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    msg.latitude = nmea_to_decimal(fields[2], fields[3][0]);
    msg.longitude = nmea_to_decimal(fields[4], fields[5][0]);
    msg.altitude = fields[9].empty() ?
      std::numeric_limits<double>::quiet_NaN() :
      std::stod(fields[9]);

    msg.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  } catch (...) {
    return false;
  }

  return true;
}

void GpsNode::timer_callback()
{
  std::string line;

  while (read_line(line)) {
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "gps_link";

    if (parse_gga(line, msg)) {
      gps_publisher_->publish(msg);
      break;
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsNode>());
  rclcpp::shutdown();
  return 0;
}