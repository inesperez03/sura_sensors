#include "sura_sensors/gps_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

namespace
{

constexpr auto TIMER_PERIOD = 20ms;

bool baudrate_to_speed(const int baudrate, speed_t & speed)
{
  switch (baudrate) {
    case 4800:
      speed = B4800;
      return true;
    case 9600:
      speed = B9600;
      return true;
    case 19200:
      speed = B19200;
      return true;
    case 38400:
      speed = B38400;
      return true;
    case 57600:
      speed = B57600;
      return true;
    case 115200:
      speed = B115200;
      return true;
#ifdef B230400
    case 230400:
      speed = B230400;
      return true;
#endif
#ifdef B460800
    case 460800:
      speed = B460800;
      return true;
#endif
#ifdef B921600
    case 921600:
      speed = B921600;
      return true;
#endif
    default:
      return false;
  }
}

template<typename T>
T read_le(const std::vector<uint8_t> & data, const std::size_t offset)
{
  T value{};
  std::memcpy(&value, data.data() + offset, sizeof(T));
  return value;
}

bool check_ubx_checksum(const std::vector<uint8_t> & packet)
{
  if (packet.size() < 8) {
    return false;
  }

  uint8_t ck_a = 0;
  uint8_t ck_b = 0;

  // UBX checksum is calculated over:
  // class, id, length low, length high, payload.
  // It does not include sync bytes 0xB5 0x62 or checksum bytes.
  for (std::size_t i = 2; i < packet.size() - 2; ++i) {
    ck_a = static_cast<uint8_t>(ck_a + packet[i]);
    ck_b = static_cast<uint8_t>(ck_b + ck_a);
  }

  return ck_a == packet[packet.size() - 2] &&
         ck_b == packet[packet.size() - 1];
}

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
  protocol_ = this->declare_parameter<std::string>("protocol", "ubx");
  serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyAMA5");
  baudrate_ = this->declare_parameter<int>("baudrate", 230400);
  frame_id_ = this->declare_parameter<std::string>("frame_id", "gps_link");
  fix_topic_ = this->declare_parameter<std::string>("fix_topic", "/sura/sensors/gps/fix");

  gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
    fix_topic_,
    10);

  if (!open_serial()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Could not open GPS serial port %s at %d baud",
      serial_port_.c_str(),
      baudrate_);
  }

  timer_ = this->create_wall_timer(
    TIMER_PERIOD,
    std::bind(&GpsNode::timer_callback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "GPS node started: protocol=%s port=%s baudrate=%d topic=%s frame_id=%s",
    protocol_.c_str(),
    serial_port_.c_str(),
    baudrate_,
    fix_topic_.c_str(),
    frame_id_.c_str());
}

GpsNode::~GpsNode()
{
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool GpsNode::open_serial()
{
  speed_t speed{};
  if (!baudrate_to_speed(baudrate_, speed)) {
    RCLCPP_ERROR(this->get_logger(), "Unsupported baudrate: %d", baudrate_);
    return false;
  }

  fd_ = open(serial_port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to open %s: %s",
      serial_port_.c_str(),
      std::strerror(errno));
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "tcgetattr failed on %s: %s",
      serial_port_.c_str(),
      std::strerror(errno));
    close(fd_);
    fd_ = -1;
    return false;
  }

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
  tty.c_cflag &= ~CRTSCTS;
#endif

  tty.c_iflag = 0;
  tty.c_oflag = 0;
  tty.c_lflag = 0;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  tcflush(fd_, TCIFLUSH);

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "tcsetattr failed on %s: %s",
      serial_port_.c_str(),
      std::strerror(errno));
    close(fd_);
    fd_ = -1;
    return false;
  }

  return true;
}

bool GpsNode::read_from_serial()
{
  if (fd_ < 0) {
    return false;
  }

  bool read_any = false;
  uint8_t temp[512];

  while (true) {
    const ssize_t bytes_read = read(fd_, temp, sizeof(temp));

    if (bytes_read > 0) {
      read_any = true;

      if (protocol_ == "nmea") {
        line_buffer_.append(
          reinterpret_cast<const char *>(temp),
          static_cast<std::size_t>(bytes_read));

        if (line_buffer_.size() > 8192) {
          line_buffer_.erase(0, line_buffer_.size() - 4096);
        }
      } else {
        byte_buffer_.insert(
          byte_buffer_.end(),
          temp,
          temp + bytes_read);

        if (byte_buffer_.size() > 8192) {
          byte_buffer_.erase(byte_buffer_.begin(), byte_buffer_.end() - 4096);
        }
      }

      continue;
    }

    if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "GPS serial read error: %s",
        std::strerror(errno));
    }

    break;
  }

  return read_any;
}

bool GpsNode::read_line(std::string & line)
{
  const std::size_t newline_pos = line_buffer_.find('\n');
  if (newline_pos == std::string::npos) {
    return false;
  }

  line = line_buffer_.substr(0, newline_pos);
  line_buffer_.erase(0, newline_pos + 1);

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

bool GpsNode::parse_gga(
  const std::string & line,
  sensor_msgs::msg::NavSatFix & msg) const
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

    msg.position_covariance.fill(0.0);
    msg.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      this->get_logger(),
      "Failed to parse NMEA GGA line: %s. Error: %s",
      line.c_str(),
      ex.what());
    return false;
  }

  return true;
}

bool GpsNode::try_extract_ubx_packet(std::vector<uint8_t> & packet)
{
  while (byte_buffer_.size() >= 2) {
    while (byte_buffer_.size() >= 2 &&
           !(byte_buffer_[0] == 0xB5 && byte_buffer_[1] == 0x62)) {
      byte_buffer_.erase(byte_buffer_.begin());
    }

    if (byte_buffer_.size() < 6) {
      return false;
    }

    const uint16_t length =
      static_cast<uint16_t>(byte_buffer_[4]) |
      (static_cast<uint16_t>(byte_buffer_[5]) << 8);

    if (length > 1024) {
      byte_buffer_.erase(byte_buffer_.begin());
      continue;
    }

    const std::size_t total_size =
      6u + static_cast<std::size_t>(length) + 2u;

    if (byte_buffer_.size() < total_size) {
      return false;
    }

    packet.assign(byte_buffer_.begin(), byte_buffer_.begin() + total_size);

    if (!check_ubx_checksum(packet)) {
      byte_buffer_.erase(byte_buffer_.begin());
      continue;
    }

    byte_buffer_.erase(byte_buffer_.begin(), byte_buffer_.begin() + total_size);
    return true;
  }

  return false;
}

bool GpsNode::parse_nav_pvt(
  const std::vector<uint8_t> & packet,
  sensor_msgs::msg::NavSatFix & msg) const
{
  if (packet.size() < 8) {
    return false;
  }

  const uint8_t msg_class = packet[2];
  const uint8_t msg_id = packet[3];

  const uint16_t length =
    static_cast<uint16_t>(packet[4]) |
    (static_cast<uint16_t>(packet[5]) << 8);

  if (msg_class != 0x01 || msg_id != 0x07 || length < 92) {
    return false;
  }

  const std::size_t total_size =
    6u + static_cast<std::size_t>(length) + 2u;

  if (packet.size() < total_size) {
    return false;
  }

  const std::size_t p = 6;

  const uint8_t fix_type = packet[p + 20];
  const uint8_t flags = packet[p + 21];
  const uint8_t num_sv = packet[p + 23];

  const bool gnss_fix_ok = (flags & 0x01) != 0;
  const bool has_position_fix = gnss_fix_ok && fix_type >= 2;

  const int32_t lon_raw = read_le<int32_t>(packet, p + 24);
  const int32_t lat_raw = read_le<int32_t>(packet, p + 28);
  const int32_t h_msl_raw = read_le<int32_t>(packet, p + 36);

  const uint32_t h_acc_raw = read_le<uint32_t>(packet, p + 40);
  const uint32_t v_acc_raw = read_le<uint32_t>(packet, p + 44);

  msg.status.status = has_position_fix ?
    sensor_msgs::msg::NavSatStatus::STATUS_FIX :
    sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;

  msg.status.service =
    sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
    sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
    sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO |
    sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;

  msg.latitude = static_cast<double>(lat_raw) * 1e-7;
  msg.longitude = static_cast<double>(lon_raw) * 1e-7;
  msg.altitude = static_cast<double>(h_msl_raw) / 1000.0;

  const double h_acc_m = static_cast<double>(h_acc_raw) / 1000.0;
  const double v_acc_m = static_cast<double>(v_acc_raw) / 1000.0;

  msg.position_covariance.fill(0.0);
  msg.position_covariance[0] = h_acc_m * h_acc_m;
  msg.position_covariance[4] = h_acc_m * h_acc_m;
  msg.position_covariance[8] = v_acc_m * v_acc_m;
  msg.position_covariance_type =
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  RCLCPP_DEBUG(
    this->get_logger(),
    "UBX NAV-PVT: fix_type=%u fix_ok=%d num_sv=%u lat=%.8f lon=%.8f hacc=%.2f vacc=%.2f",
    fix_type,
    gnss_fix_ok,
    num_sv,
    msg.latitude,
    msg.longitude,
    h_acc_m,
    v_acc_m);

  return true;
}

void GpsNode::timer_callback()
{
  read_from_serial();

  if (protocol_ == "nmea") {
    std::string line;

    while (read_line(line)) {
      sensor_msgs::msg::NavSatFix msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = frame_id_;

      if (parse_gga(line, msg)) {
        gps_publisher_->publish(msg);
        break;
      }
    }

    return;
  }

  if (protocol_ == "ubx") {
    std::vector<uint8_t> packet;

    while (try_extract_ubx_packet(packet)) {
      sensor_msgs::msg::NavSatFix msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = frame_id_;

      if (parse_nav_pvt(packet, msg)) {
        gps_publisher_->publish(msg);
      }
    }

    return;
  }

  RCLCPP_ERROR_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    5000,
    "Unknown GPS protocol '%s'. Use 'ubx' or 'nmea'.",
    protocol_.c_str());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsNode>());
  rclcpp::shutdown();
  return 0;
}