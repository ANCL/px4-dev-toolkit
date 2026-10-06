#include "px4_mocap_bridge/conversion.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace
{

uint64_t ros_time_to_microseconds(
  const builtin_interfaces::msg::Time & stamp)
{
  const auto nanoseconds =
    static_cast<int64_t>(stamp.sec) * 1000000000LL +
    static_cast<int64_t>(stamp.nanosec);

  return static_cast<uint64_t>(nanoseconds / 1000LL);
}

bool quaternion_is_valid(const Eigen::Quaterniond & quaternion)
{
  return
    std::isfinite(quaternion.w()) &&
    std::isfinite(quaternion.x()) &&
    std::isfinite(quaternion.y()) &&
    std::isfinite(quaternion.z()) &&
    quaternion.norm() > 1e-9;
}


Eigen::Vector3d enu_to_ned(
  const Eigen::Vector3d & vector_enu)
{
  // ROS ENU -> PX4 NED: swap East/North and invert Up to Down.
  return {
    vector_enu.y(),
    vector_enu.x(),
    -vector_enu.z(),
  };
}


Eigen::Quaterniond ros_to_px4_orientation(
  const Eigen::Quaterniond & orientation_ros)
{
  // ROS expresses an FLU body orientation in ENU; PX4 expects an FRD body
  // orientation in NED. Compose the fixed ENU -> NED world transform with
  // the FLU -> FRD body transform.
  constexpr double kHalfSqrtTwo =
    0.70710678118654752440;

  const Eigen::Quaterniond ned_from_enu(
    0.0,
    kHalfSqrtTwo,
    kHalfSqrtTwo,
    0.0);

  const Eigen::Quaterniond flu_from_frd(
    0.0,
    1.0,
    0.0,
    0.0);

  return
    ned_from_enu *
    orientation_ros *
    flu_from_frd;
}

}  // namespace

namespace px4_mocap_bridge
{

bool pose_is_valid(
  const geometry_msgs::msg::PoseStamped & pose)
{
  const Eigen::Vector3d position(
    pose.pose.position.x,
    pose.pose.position.y,
    pose.pose.position.z);

  const Eigen::Quaterniond orientation(
    pose.pose.orientation.w,
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z);

  return position.allFinite() &&
    quaternion_is_valid(orientation);
}

px4_msgs::msg::VehicleOdometry convert_pose(
  const geometry_msgs::msg::PoseStamped & pose,
  uint64_t now_us,
  bool use_header_stamp)
{
  const Eigen::Vector3d position_enu(
    pose.pose.position.x,
    pose.pose.position.y,
    pose.pose.position.z);

  Eigen::Quaterniond orientation_ros(
    pose.pose.orientation.w,
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z);

  orientation_ros.normalize();

  const Eigen::Vector3d position_ned =
    enu_to_ned(position_enu);

  const Eigen::Quaterniond orientation_px4 =
    ros_to_px4_orientation(orientation_ros);

  uint64_t sample_us = now_us;

  if (
    use_header_stamp &&
    (pose.header.stamp.sec != 0 ||
    pose.header.stamp.nanosec != 0))
  {
    sample_us =
      ros_time_to_microseconds(pose.header.stamp);
  }

  const float nan =
    std::numeric_limits<float>::quiet_NaN();

  px4_msgs::msg::VehicleOdometry odometry{};

  odometry.timestamp = now_us;
  odometry.timestamp_sample = sample_us;

  odometry.pose_frame =
    px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

  odometry.position[0] =
    static_cast<float>(position_ned.x());
  odometry.position[1] =
    static_cast<float>(position_ned.y());
  odometry.position[2] =
    static_cast<float>(position_ned.z());

  odometry.q[0] =
    static_cast<float>(orientation_px4.w());
  odometry.q[1] =
    static_cast<float>(orientation_px4.x());
  odometry.q[2] =
    static_cast<float>(orientation_px4.y());
  odometry.q[3] =
    static_cast<float>(orientation_px4.z());

  odometry.velocity_frame =
    px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
  odometry.velocity = {nan, nan, nan};
  odometry.angular_velocity = {nan, nan, nan};

  odometry.position_variance = {nan, nan, nan};
  odometry.orientation_variance = {nan, nan, nan};
  odometry.velocity_variance = {nan, nan, nan};

  odometry.reset_counter = 0;
  odometry.quality = 1;

  return odometry;
}

}  // namespace px4_mocap_bridge
