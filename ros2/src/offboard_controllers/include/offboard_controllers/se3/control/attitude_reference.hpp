// Desired attitude, angular-rate, and angular-acceleration reference interfaces.

#pragma once

#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::se3::attitude_reference {

// Generate desired FRD-to-NED attitude from force [N] and yaw [rad].
RotationMatrix compute_orientation(const Vector3& force, double yaw);

// Add analytic desired body angular velocity [rad/s] in the desired FRD frame.
DesiredAttitudeRate compute_rate(const Vector3& force, const Vector3& force_derivative, double yaw,
                                 double yaw_rate);

// Also calculate desired body angular acceleration [rad/s^2].
DesiredAttitudeDynamics compute_dynamics(const Vector3& force, const Vector3& force_derivative,
                                         const Vector3& force_second_derivative, double yaw,
                                         double yaw_rate, double yaw_acceleration);

} // namespace offboard_controllers::se3::attitude_reference
