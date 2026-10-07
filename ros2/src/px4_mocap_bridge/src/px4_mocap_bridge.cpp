#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "px4_mocap_bridge/conversion.hpp"

namespace px4_topics
{

#define PX4_TOPIC(name, value) inline constexpr char name[] = value;
#include "px4_topics.def"
#undef PX4_TOPIC

}  // namespace px4_topics

namespace
{

using PoseStamped = geometry_msgs::msg::PoseStamped;
using VehicleOdometry = px4_msgs::msg::VehicleOdometry;

}  // namespace

/*
 * Mocap bridge runtime:
 *
 * Input:
 *   one configured geometry_msgs/PoseStamped motion-capture stream.
 *
 * Action:
 *   reject invalid poses, convert ENU/FLU -> NED/FRD, and publish PX4
 *   VehicleOdometry using sensor-data QoS.
 *
 * Timing:
 *   VehicleOdometry.timestamp is ROS publication time. timestamp_sample is the
 *   source PoseStamped header time when enabled and non-zero, otherwise the
 *   publication time is used as the measurement-time fallback.
 *
 * Output:
 *   /fmu/in/vehicle_mocap_odometry for PX4 estimator ingestion.
 */
class Px4MocapBridge : public rclcpp::Node
{
public:
  Px4MocapBridge()
  : Node("px4_mocap_bridge")
  {
    declare_parameter<std::string>("mocap_topic", "");
    declare_parameter<bool>("use_header_stamp", true);

    mocap_topic_ = get_parameter("mocap_topic").as_string();
    use_header_stamp_ = get_parameter("use_header_stamp").as_bool();

    if (mocap_topic_.empty()) {
      throw std::runtime_error(
        "mocap_topic must identify a PoseStamped motion-capture topic.");
    }

    // Motion capture is a live sensor stream: prefer the newest available
    // measurement over retransmission of stale poses. SensorDataQoS requests
    // the intended best-effort, volatile behavior for this subscription.
    subscription_ = create_subscription<PoseStamped>(
      mocap_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &Px4MocapBridge::pose_callback,
        this,
        std::placeholders::_1));

    publisher_ = create_publisher<VehicleOdometry>(
      px4_topics::IN_VEHICLE_MOCAP_ODOMETRY,
      rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "Motion capture: %s -> %s",
      mocap_topic_.c_str(),
      px4_topics::IN_VEHICLE_MOCAP_ODOMETRY);
  }

private:
  void pose_callback(const PoseStamped::SharedPtr message)
  {
    if (!px4_mocap_bridge::pose_is_valid(*message)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Ignoring invalid motion-capture pose.");
      return;
    }

    // Capture publication time once so timestamp and fallback timestamp_sample
    // refer to the same callback instant.
    const uint64_t now_us =
      static_cast<uint64_t>(
        get_clock()->now().nanoseconds() / 1000LL);

    publisher_->publish(
      px4_mocap_bridge::convert_pose(
        *message,
        now_us,
        use_header_stamp_));
  }

  std::string mocap_topic_;
  bool use_header_stamp_{true};

  rclcpp::Subscription<PoseStamped>::SharedPtr subscription_;
  rclcpp::Publisher<VehicleOdometry>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<Px4MocapBridge>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("px4_mocap_bridge"),
      "%s",
      exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
