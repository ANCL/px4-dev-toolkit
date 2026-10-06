#pragma once

#include <cstdint>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

namespace px4_mocap_bridge
{

bool pose_is_valid(
  const geometry_msgs::msg::PoseStamped & pose);

px4_msgs::msg::VehicleOdometry convert_pose(
  const geometry_msgs::msg::PoseStamped & pose,
  uint64_t now_us,
  bool use_header_stamp);

}  // namespace px4_mocap_bridge
