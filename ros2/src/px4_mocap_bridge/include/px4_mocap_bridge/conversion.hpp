#pragma once

#include <cstdint>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

namespace px4_mocap_bridge
{

// Validation:
//   Accept only finite position and a finite, non-zero orientation quaternion.
bool pose_is_valid(
  const geometry_msgs::msg::PoseStamped & pose);

// Mocap-conversion contract:
//
// Inputs:
//   PoseStamped pose in ENU world / FLU body convention and publication time.
//   A valid header stamp may be used as the measurement timestamp.
//
// Logic:
//   convert ENU -> NED and FLU -> FRD, then normalize orientation.
//
// Output:
//   PX4 VehicleOdometry pose in NED/FRD with publication/sample time separated.
//   Unavailable velocity and variance fields are marked with NaN.
px4_msgs::msg::VehicleOdometry convert_pose(
  const geometry_msgs::msg::PoseStamped & pose,
  uint64_t now_us,
  bool use_header_stamp);

}  // namespace px4_mocap_bridge
