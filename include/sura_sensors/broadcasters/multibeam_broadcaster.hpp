#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include "sura_msgs/msg/multibeam_ping.hpp"

namespace sura_sensors
{

class MultibeamBroadcaster : public controller_interface::ControllerInterface
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
  static constexpr std::size_t kMaxDetections = 64;

  struct Detection
  {
    float angle_rad{0.0F};
    float time_of_flight_s{0.0F};
    float power{0.0F};
    uint8_t point_type{0};
    uint32_t reserved{0};
  };

  struct Snapshot
  {
    uint64_t ping_sequence{0};
    rclcpp::Time reception_time{0, 0, RCL_ROS_TIME};
    uint32_t ping_number{0};
    uint32_t power_up_time_ms{0};
    uint64_t device_utc_time_ms{0};
    float listening_time_s{0.0F};
    float sound_speed_m_s{1500.0F};
    uint32_t acoustic_frequency_hz{0};
    float pulse_duration_s{0.0F};
    uint32_t flags{0};
    uint16_t reported_detection_count{0};
    uint16_t stored_detection_count{0};
    bool truncated{false};
    std::array<Detection, kMaxDetections> detections{};

    uint64_t attitude_sequence{0};
    float up_vector_x{0.0F};
    float up_vector_y{0.0F};
    float up_vector_z{1.0F};

    uint64_t water_sequence{0};
    bool water_valid{false};
    float temperature_c{0.0F};
    float pressure_bar{0.0F};

    int transport_state{0};
    bool connected{false};
    uint64_t valid_pings{0};
    uint64_t reconnect_count{0};
    uint64_t checksum_errors{0};
    uint64_t malformed_packets{0};
    uint64_t unknown_ids{0};
    uint64_t truncated_pings{0};
    uint64_t rx_bytes{0};
    uint64_t tx_bytes{0};
    int last_errno{0};
    double seconds_since_last_rx{0.0};
    double seconds_since_last_ping{0.0};
  };

  static std::vector<std::string> state_names(const std::string & sensor_name);
  static uint64_t read_u64(double hi, double lo);
  static uint64_t read_counter(double value);
  std::string resolve_topic(const std::string & value) const;
  void publish_latest();
  void publish_ping_and_points(const Snapshot & snapshot);
  void publish_attitude(const Snapshot & snapshot);
  void publish_water(const Snapshot & snapshot);
  void publish_diagnostics(const Snapshot & snapshot);

  std::string sensor_name_;
  std::string frame_id_;
  std::string points_topic_;
  std::string ping_topic_;
  std::string up_vector_topic_;
  std::string temperature_topic_;
  std::string pressure_topic_;
  double publish_period_s_{0.05};
  double stale_timeout_s_{3.0};

  std::mutex snapshot_mutex_;
  Snapshot latest_snapshot_;
  uint64_t last_published_ping_sequence_{0};
  uint64_t last_published_attitude_sequence_{0};
  uint64_t last_published_water_sequence_{0};

  rclcpp_lifecycle::LifecyclePublisher<sura_msgs::msg::MultibeamPing>::SharedPtr ping_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr up_vector_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace sura_sensors
