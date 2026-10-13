// Validate the staging position and yaw supplied through ROS parameters.

#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>


namespace px4_position_mode
{

// Initial Offboard setpoint in PX4 local NED coordinates:
// position [m], yaw [rad].
struct InitialSetpoint
{
  float x;
  float y;
  float z;
  float yaw;
};


// Read required initial position [m] and yaw [rad] from ROS parameters.
// Reject absent or non-finite values rather than duplicating YAML defaults.
inline InitialSetpoint load_initial_setpoint(
  rclcpp::Node & node)
{
  try {
    const double x =
      node.declare_parameter<double>("initial_x");

    const double y =
      node.declare_parameter<double>("initial_y");

    const double z =
      node.declare_parameter<double>("initial_z");

    const double yaw =
      node.declare_parameter<double>("initial_yaw");

    if (!std::isfinite(x) ||
      !std::isfinite(y) ||
      !std::isfinite(z) ||
      !std::isfinite(yaw))
    {
      throw std::runtime_error(
              "Offboard initial-setpoint parameters must be finite.");
    }

    return {
      static_cast<float>(x),
      static_cast<float>(y),
      static_cast<float>(z),
      static_cast<float>(yaw),
    };

  } catch (const std::exception & error) {
    throw std::runtime_error(
            "Could not load required Offboard initial setpoint: " +
            std::string(error.what()));
  }
}

}  // namespace px4_position_mode
