#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace
{

constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_F = 1.0 / 298.257223563;
constexpr double WGS84_E2 = WGS84_F * (2.0 - WGS84_F);

struct Ecef
{
  double x;
  double y;
  double z;
};

struct Enu
{
  double e;
  double n;
  double u;
};

struct WorldPoint
{
  double x;
  double y;
  double z;
};

double deg_to_rad(const double deg)
{
  return deg * M_PI / 180.0;
}

Ecef geodetic_to_ecef(
  const double lat_deg,
  const double lon_deg,
  const double alt_m)
{
  const double lat = deg_to_rad(lat_deg);
  const double lon = deg_to_rad(lon_deg);

  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double sin_lon = std::sin(lon);
  const double cos_lon = std::cos(lon);

  const double n = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);

  return {
    (n + alt_m) * cos_lat * cos_lon,
    (n + alt_m) * cos_lat * sin_lon,
    (n * (1.0 - WGS84_E2) + alt_m) * sin_lat
  };
}

Enu geodetic_to_enu(
  const double lat_deg,
  const double lon_deg,
  const double alt_m,
  const double origin_lat_deg,
  const double origin_lon_deg,
  const double origin_alt_m)
{
  const Ecef p = geodetic_to_ecef(lat_deg, lon_deg, alt_m);
  const Ecef o = geodetic_to_ecef(origin_lat_deg, origin_lon_deg, origin_alt_m);

  const double dx = p.x - o.x;
  const double dy = p.y - o.y;
  const double dz = p.z - o.z;

  const double lat0 = deg_to_rad(origin_lat_deg);
  const double lon0 = deg_to_rad(origin_lon_deg);

  const double sin_lat = std::sin(lat0);
  const double cos_lat = std::cos(lat0);
  const double sin_lon = std::sin(lon0);
  const double cos_lon = std::cos(lon0);

  const double east =
    -sin_lon * dx + cos_lon * dy;

  const double north =
    -sin_lat * cos_lon * dx -
    sin_lat * sin_lon * dy +
    cos_lat * dz;

  const double up =
    cos_lat * cos_lon * dx +
    cos_lat * sin_lon * dy +
    sin_lat * dz;

  return {
    east,
    north,
    up
  };
}

bool is_valid_fix(const sensor_msgs::msg::NavSatFix & fix)
{
  return
    fix.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
    std::isfinite(fix.latitude) &&
    std::isfinite(fix.longitude) &&
    std::isfinite(fix.altitude);
}

bool has_known_horizontal_accuracy(const sensor_msgs::msg::NavSatFix & fix)
{
  return
    fix.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN &&
    std::isfinite(fix.position_covariance[0]) &&
    std::isfinite(fix.position_covariance[4]) &&
    fix.position_covariance[0] > 0.0 &&
    fix.position_covariance[4] > 0.0;
}

double horizontal_accuracy_m(const sensor_msgs::msg::NavSatFix & fix)
{
  const double var_x = fix.position_covariance[0];
  const double var_y = fix.position_covariance[4];
  return std::sqrt(std::max(var_x, var_y));
}

double normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

WorldPoint enu_to_world(
  const Enu & enu,
  const double world_yaw_rad)
{
  const double cos_yaw = std::cos(world_yaw_rad);
  const double sin_yaw = std::sin(world_yaw_rad);

  return {
    cos_yaw * enu.e + sin_yaw * enu.n,
    -sin_yaw * enu.e + cos_yaw * enu.n,
    enu.u
  };
}

double yaw_from_quaternion_msg(const geometry_msgs::msg::Quaternion & q_msg)
{
  tf2::Quaternion q(
    q_msg.x,
    q_msg.y,
    q_msg.z,
    q_msg.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace


class GpsAnchorNode : public rclcpp::Node
{
public:
  GpsAnchorNode()
  : Node("gps_anchor_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    gps_topic_ = this->declare_parameter<std::string>(
      "gps_topic", "/sura/sensors/gps/fix");

    world_frame_ = this->declare_parameter<std::string>(
      "world_frame", "world_enu");

    fastlio_map_frame_ = this->declare_parameter<std::string>(
      "fastlio_map_frame", "map");

    position_frame_ = this->declare_parameter<std::string>(
      "position_frame", "gps_frame");

    base_frame_ = this->declare_parameter<std::string>(
      "base_frame", "blueboat/base_link_enu");

    yaw_topic_ = this->declare_parameter<std::string>(
      "yaw_topic", "");

    use_manual_base_yaw_ = this->declare_parameter<bool>(
      "use_manual_base_yaw", false);

    origin_mode_ = this->declare_parameter<std::string>(
      "origin_mode", "manual");

    anchor_sample_count_ = this->declare_parameter<int>(
      "anchor_sample_count", 10);

    origin_latitude_ = this->declare_parameter<double>(
      "origin_latitude", 39.994335199497364);

    origin_longitude_ = this->declare_parameter<double>(
      "origin_longitude", -0.07400607762331379);

    origin_altitude_ = this->declare_parameter<double>(
      "origin_altitude", 0.0);

    yaw_offset_rad_ = this->declare_parameter<double>(
      "yaw_offset_rad", 0.0);

    manual_base_yaw_east_rad_ = this->declare_parameter<double>(
      "manual_base_yaw_east_rad", 0.0);

    world_frame_yaw_rad_ = this->declare_parameter<double>(
      "world_frame_yaw_rad", 0.0);

    max_horizontal_accuracy_m_ = this->declare_parameter<double>(
      "max_horizontal_accuracy_m", 20.0);

    publish_path_ = this->declare_parameter<bool>(
      "publish_path", true);

    max_path_size_ = this->declare_parameter<int>(
      "max_path_size", 1000);

    if (anchor_sample_count_ < 1) {
      RCLCPP_WARN(
        this->get_logger(),
        "anchor_sample_count must be >= 1. Forcing to 1.");
      anchor_sample_count_ = 1;
    }

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/sura/sensors/gps/pose_world_enu", 10);

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/sura/sensors/gps/path_world_enu", 10);

    static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic_,
      10,
      std::bind(&GpsAnchorNode::gps_callback, this, std::placeholders::_1));

    if (!yaw_topic_.empty()) {
      yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        yaw_topic_,
        10,
        std::bind(&GpsAnchorNode::yaw_callback, this, std::placeholders::_1));
    }

    if (origin_mode_ == "manual") {
      origin_ready_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "Using manual GPS origin for %s: lat=%.10f lon=%.10f alt=%.3f",
        world_frame_.c_str(),
        origin_latitude_,
        origin_longitude_,
        origin_altitude_);
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Waiting for %d valid GPS fixes to average origin for %s",
        anchor_sample_count_,
        world_frame_.c_str());
    }

    RCLCPP_INFO(
      this->get_logger(),
      "GPS anchor node started. gps_topic=%s yaw_topic=%s world_frame=%s fastlio_map_frame=%s position_frame=%s base_frame=%s anchor_sample_count=%d yaw_offset=%.3f rad world_frame_yaw=%.3f rad manual_base_yaw=%s manual_base_yaw_east=%.3f rad",
      gps_topic_.c_str(),
      yaw_topic_.empty() ? "<disabled>" : yaw_topic_.c_str(),
      world_frame_.c_str(),
      fastlio_map_frame_.c_str(),
      position_frame_.c_str(),
      base_frame_.c_str(),
      anchor_sample_count_,
      yaw_offset_rad_,
      world_frame_yaw_rad_,
      use_manual_base_yaw_ ? "true" : "false",
      manual_base_yaw_east_rad_);
  }

private:
  void yaw_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Ignoring invalid yaw sample");
      return;
    }

    latest_yaw_rad_ = normalize_angle(msg->data);
  }

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (!is_valid_fix(*msg)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Ignoring invalid GPS fix. status=%d lat=%.8f lon=%.8f",
        msg->status.status,
        msg->latitude,
        msg->longitude);
      return;
    }

    if (has_known_horizontal_accuracy(*msg)) {
      const double h_acc = horizontal_accuracy_m(*msg);

      if (h_acc > max_horizontal_accuracy_m_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          3000,
          "Ignoring GPS fix because horizontal accuracy is too poor: %.2f m > %.2f m",
          h_acc,
          max_horizontal_accuracy_m_);
        return;
      }
    }

    if (!origin_ready_) {
      collect_origin_sample(*msg);
      return;
    }

    const Enu gps_enu = geodetic_to_enu(
      msg->latitude,
      msg->longitude,
      origin_altitude_,
      origin_latitude_,
      origin_longitude_,
      origin_altitude_);

    publish_gps_pose(gps_enu, msg->header.stamp);

    if (!anchor_ready_) {
      collect_anchor_sample(gps_enu);
    }
  }

  void collect_origin_sample(const sensor_msgs::msg::NavSatFix & fix)
  {
    origin_samples_.push_back(fix);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Collecting GPS origin samples: %zu / %d",
      origin_samples_.size(),
      anchor_sample_count_);

    if (static_cast<int>(origin_samples_.size()) < anchor_sample_count_) {
      return;
    }

    double lat_sum = 0.0;
    double lon_sum = 0.0;
    double alt_sum = 0.0;

    for (const auto & sample : origin_samples_) {
      lat_sum += sample.latitude;
      lon_sum += sample.longitude;
      alt_sum += sample.altitude;
    }

    const double count = static_cast<double>(origin_samples_.size());

    origin_latitude_ = lat_sum / count;
    origin_longitude_ = lon_sum / count;
    origin_altitude_ = alt_sum / count;

    origin_ready_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "GPS origin fixed for %s from %zu samples: lat=%.10f lon=%.10f alt=%.3f",
      world_frame_.c_str(),
      origin_samples_.size(),
      origin_latitude_,
      origin_longitude_,
      origin_altitude_);
  }

  tf2::Matrix3x3 world_from_map_rotation(const double yaw_rad) const
  {
    tf2::Matrix3x3 yaw_rotation;
    yaw_rotation.setRPY(0.0, 0.0, yaw_rad);
    return yaw_rotation;
  }

  bool compute_world_to_map_translation_estimate(
    const WorldPoint & gps_world,
    const double yaw_world_map_rad,
    tf2::Vector3 & p_world_map_out)
  {
    geometry_msgs::msg::TransformStamped map_to_position_tf;

    try {
      map_to_position_tf = tf_buffer_.lookupTransform(
        fastlio_map_frame_,
        position_frame_,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Cannot collect anchor sample yet. Missing TF %s -> %s: %s",
        fastlio_map_frame_.c_str(),
        position_frame_.c_str(),
        ex.what());
      return false;
    }

    const tf2::Vector3 p_world_base(
      gps_world.x,
      gps_world.y,
      gps_world.z);

    const tf2::Vector3 p_map_position(
      map_to_position_tf.transform.translation.x,
      map_to_position_tf.transform.translation.y,
      map_to_position_tf.transform.translation.z);

    const tf2::Matrix3x3 r_world_map = world_from_map_rotation(yaw_world_map_rad);

    p_world_map_out = p_world_base - (r_world_map * p_map_position);

    return true;
  }

  bool lookup_map_to_base_yaw(double & yaw_map_base_rad)
  {
    geometry_msgs::msg::TransformStamped map_to_base_tf;

    try {
      map_to_base_tf = tf_buffer_.lookupTransform(
        fastlio_map_frame_,
        base_frame_,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Cannot collect anchor yaw yet. Missing TF %s -> %s: %s",
        fastlio_map_frame_.c_str(),
        base_frame_.c_str(),
        ex.what());
      return false;
    }

    yaw_map_base_rad = normalize_angle(
      yaw_from_quaternion_msg(map_to_base_tf.transform.rotation));
    return true;
  }

  void collect_anchor_sample(const Enu & gps_enu)
  {
    if (!use_manual_base_yaw_ && !yaw_topic_.empty() && !latest_yaw_rad_.has_value()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Waiting for initial yaw samples on %s before anchoring",
        yaw_topic_.c_str());
      return;
    }

    const WorldPoint gps_world = enu_to_world(gps_enu, world_frame_yaw_rad_);

    double yaw_map_base_rad = 0.0;
    if (!lookup_map_to_base_yaw(yaw_map_base_rad)) {
      return;
    }

    double yaw_world_base_rad = normalize_angle(-world_frame_yaw_rad_);
    if (use_manual_base_yaw_) {
      yaw_world_base_rad = normalize_angle(
        manual_base_yaw_east_rad_ - world_frame_yaw_rad_);
    } else if (latest_yaw_rad_.has_value()) {
      yaw_world_base_rad = normalize_angle(
        *latest_yaw_rad_ - world_frame_yaw_rad_ + yaw_offset_rad_);
    }

    const double yaw_world_map_rad =
      normalize_angle(yaw_world_base_rad - yaw_map_base_rad);

    tf2::Vector3 p_world_map_estimate;

    if (!compute_world_to_map_translation_estimate(
        gps_world, yaw_world_map_rad, p_world_map_estimate))
    {
      return;
    }

    anchor_translation_samples_.push_back(p_world_map_estimate);
    anchor_yaw_samples_.push_back(yaw_world_map_rad);

    RCLCPP_INFO(
      this->get_logger(),
      "Collected FAST-LIO anchor estimate %zu / %d: world->map translation estimate [%.3f, %.3f, %.3f], yaw(world->map)=%.3f rad",
      anchor_translation_samples_.size(),
      anchor_sample_count_,
      p_world_map_estimate.x(),
      p_world_map_estimate.y(),
      p_world_map_estimate.z(),
      yaw_world_map_rad);

    if (static_cast<int>(anchor_translation_samples_.size()) < anchor_sample_count_) {
      return;
    }

    publish_average_world_to_map_anchor();
  }

  void publish_average_world_to_map_anchor()
  {
    tf2::Vector3 p_world_map_sum(0.0, 0.0, 0.0);

    for (const auto & sample : anchor_translation_samples_) {
      p_world_map_sum += sample;
    }

    const double count = static_cast<double>(anchor_translation_samples_.size());
    const tf2::Vector3 p_world_map = p_world_map_sum / count;

    double yaw_sin_sum = 0.0;
    double yaw_cos_sum = 0.0;
    for (const double sample : anchor_yaw_samples_) {
      yaw_sin_sum += std::sin(sample);
      yaw_cos_sum += std::cos(sample);
    }

    const double averaged_yaw_rad = anchor_yaw_samples_.empty() ?
      yaw_offset_rad_ :
      std::atan2(yaw_sin_sum, yaw_cos_sum);

    const tf2::Matrix3x3 r_world_map = world_from_map_rotation(averaged_yaw_rad);

    tf2::Quaternion q_world_map;
    r_world_map.getRotation(q_world_map);
    q_world_map.normalize();

    geometry_msgs::msg::TransformStamped world_to_map_tf;
    world_to_map_tf.header.stamp = this->now();
    world_to_map_tf.header.frame_id = world_frame_;
    world_to_map_tf.child_frame_id = fastlio_map_frame_;

    world_to_map_tf.transform.translation.x = p_world_map.x();
    world_to_map_tf.transform.translation.y = p_world_map.y();
    world_to_map_tf.transform.translation.z = p_world_map.z();

    world_to_map_tf.transform.rotation.x = q_world_map.x();
    world_to_map_tf.transform.rotation.y = q_world_map.y();
    world_to_map_tf.transform.rotation.z = q_world_map.z();
    world_to_map_tf.transform.rotation.w = q_world_map.w();

    static_tf_broadcaster_->sendTransform(world_to_map_tf);

    anchor_ready_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Published averaged static TF %s -> %s from %zu estimates. Translation [%.3f, %.3f, %.3f], yaw=%.3f rad",
      world_frame_.c_str(),
      fastlio_map_frame_.c_str(),
      anchor_translation_samples_.size(),
      p_world_map.x(),
      p_world_map.y(),
      p_world_map.z(),
      averaged_yaw_rad);
  }

  void publish_gps_pose(
    const Enu & gps_enu,
    const builtin_interfaces::msg::Time & stamp)
  {
    const WorldPoint gps_world = enu_to_world(gps_enu, world_frame_yaw_rad_);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = world_frame_;

    pose.pose.position.x = gps_world.x;
    pose.pose.position.y = gps_world.y;
    pose.pose.position.z = gps_world.z;
    pose.pose.orientation.w = 1.0;

    pose_pub_->publish(pose);

    if (!publish_path_) {
      return;
    }

    path_.header.stamp = stamp;
    path_.header.frame_id = world_frame_;
    path_.poses.push_back(pose);

    if (static_cast<int>(path_.poses.size()) > max_path_size_) {
      path_.poses.erase(path_.poses.begin());
    }

    path_pub_->publish(path_);
  }

  std::string gps_topic_;
  std::string yaw_topic_;
  std::string world_frame_;
  std::string fastlio_map_frame_;
  std::string position_frame_;
  std::string base_frame_;
  std::string origin_mode_;

  int anchor_sample_count_;
  int max_path_size_;

  double origin_latitude_;
  double origin_longitude_;
  double origin_altitude_;
  double yaw_offset_rad_;
  double manual_base_yaw_east_rad_;
  double world_frame_yaw_rad_;
  double max_horizontal_accuracy_m_;

  bool publish_path_;
  bool use_manual_base_yaw_;

  bool origin_ready_{false};
  bool anchor_ready_{false};

  std::optional<double> latest_yaw_rad_;
  std::deque<sensor_msgs::msg::NavSatFix> origin_samples_;
  std::vector<tf2::Vector3> anchor_translation_samples_;
  std::vector<double> anchor_yaw_samples_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  nav_msgs::msg::Path path_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsAnchorNode>());
  rclcpp::shutdown();
  return 0;
}
