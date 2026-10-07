#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>


namespace offboard_controllers
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


// Initial-setpoint loading contract:
//
// Inputs:
//   required initial_x/y/z [m] and initial_yaw [rad] ROS parameters.
//
// Logic:
//   require every parameter and reject non-finite values. Defaults are not
//   duplicated in C++; launch configuration remains the source of values.
//
// Output:
//   validated PX4 local-NED InitialSetpoint. Missing/invalid configuration
//   terminates startup with a descriptive exception.
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

}  // namespace offboard_controllers
