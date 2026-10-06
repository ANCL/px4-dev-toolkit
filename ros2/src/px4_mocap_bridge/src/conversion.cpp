#include "px4_mocap_bridge/conversion.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

#include <Eigen/Dense>
#include <px4_ros_com/frame_transforms.h>

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
    px4_ros_com::frame_transforms::enu_to_ned_local_frame(
      position_enu);

  const Eigen::Quaterniond orientation_px4 =
    px4_ros_com::frame_transforms::ros_to_px4_orientation(
      orientation_ros);

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
