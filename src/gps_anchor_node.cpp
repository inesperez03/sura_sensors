#include <cmath>
#include <deque>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

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

struct Ned
{
  double n;
  double e;
  double d;
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

Ned geodetic_to_ned(
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
    north,
    east,
    -up
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
      "world_frame", "world_ned");

    fastlio_map_frame_ = this->declare_parameter<std::string>(
      "fastlio_map_frame", "map");

    base_frame_ = this->declare_parameter<std::string>(
      "base_frame", "base_link_enu");

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

    map_frame_is_enu_ = this->declare_parameter<bool>(
      "map_frame_is_enu", true);

    publish_path_ = this->declare_parameter<bool>(
      "publish_path", true);

    max_path_size_ = this->declare_parameter<int>(
      "max_path_size", 1000);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/sura/sensors/gps/pose_world_ned", 10);

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/sura/sensors/gps/path_world_ned", 10);

    static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic_,
      10,
      std::bind(&GpsAnchorNode::gps_callback, this, std::placeholders::_1));

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
      "GPS anchor node started. gps_topic=%s world_frame=%s fastlio_map_frame=%s base_frame=%s",
      gps_topic_.c_str(),
      world_frame_.c_str(),
      fastlio_map_frame_.c_str(),
      base_frame_.c_str());
  }

private:
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

    if (!origin_ready_) {
      collect_origin_sample(*msg);
      return;
    }

    const Ned gps_ned = geodetic_to_ned(
      msg->latitude,
      msg->longitude,
      msg->altitude,
      origin_latitude_,
      origin_longitude_,
      origin_altitude_);

    publish_gps_pose(gps_ned, msg->header.stamp);

    if (!anchor_ready_) {
      try_create_world_to_map_anchor(gps_ned);
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

  tf2::Matrix3x3 world_from_map_rotation() const
  {
    tf2::Matrix3x3 yaw_rotation;
    yaw_rotation.setRPY(0.0, 0.0, yaw_offset_rad_);

    if (!map_frame_is_enu_) {
      return yaw_rotation;
    }

    // FAST-LIO map assumed ENU:
    //   map.x = East
    //   map.y = North
    //   map.z = Up
    //
    // world_ned:
    //   world_ned.x = North
    //   world_ned.y = East
    //   world_ned.z = Down
    tf2::Matrix3x3 ned_from_enu(
      0.0, 1.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 0.0, -1.0);

    return yaw_rotation * ned_from_enu;
  }

  void try_create_world_to_map_anchor(const Ned & gps_ned)
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
        "Cannot anchor yet. Missing TF %s -> %s: %s",
        fastlio_map_frame_.c_str(),
        base_frame_.c_str(),
        ex.what());
      return;
    }

    const tf2::Vector3 p_world_base(
      gps_ned.n,
      gps_ned.e,
      gps_ned.d);

    const tf2::Vector3 p_map_base(
      map_to_base_tf.transform.translation.x,
      map_to_base_tf.transform.translation.y,
      map_to_base_tf.transform.translation.z);

    const tf2::Matrix3x3 r_world_map = world_from_map_rotation();

    const tf2::Vector3 p_world_map =
      p_world_base - (r_world_map * p_map_base);

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
      "Published static TF %s -> %s. Translation [%.3f, %.3f, %.3f], yaw_offset=%.3f rad",
      world_frame_.c_str(),
      fastlio_map_frame_.c_str(),
      p_world_map.x(),
      p_world_map.y(),
      p_world_map.z(),
      yaw_offset_rad_);
  }

  void publish_gps_pose(
    const Ned & gps_ned,
    const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = world_frame_;

    pose.pose.position.x = gps_ned.n;
    pose.pose.position.y = gps_ned.e;
    pose.pose.position.z = gps_ned.d;
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
  std::string world_frame_;
  std::string fastlio_map_frame_;
  std::string base_frame_;
  std::string origin_mode_;

  int anchor_sample_count_;
  int max_path_size_;

  double origin_latitude_;
  double origin_longitude_;
  double origin_altitude_;
  double yaw_offset_rad_;

  bool map_frame_is_enu_;
  bool publish_path_;

  bool origin_ready_{false};
  bool anchor_ready_{false};

  std::deque<sensor_msgs::msg::NavSatFix> origin_samples_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
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