// Geometric attitude feedback in PX4-normalized torque coordinates;
// this path does not calculate a physical inertia-based moment.

#include <offboard_controllers/se3/control/normalized_torque.hpp>

#include <stdexcept>

#include <offboard_controllers/math/operations.hpp>
#include <offboard_controllers/se3/control/attitude_geometry.hpp>

namespace offboard_controllers::se3::normalized_torque {

GeometricNormalizedOutput compute(const RotationMatrix& attitude, const Vector3& angular_velocity,
                                  const DesiredAttitudeDynamics& desired,
                                  const Vector3& attitude_gain,
                                  const Vector3& angular_velocity_gain,
                                  const Vector3& acceleration_gain) {
  // All three gains map errors/feedforward into normalized PX4 torque;
  // negative coefficients are invalid for this configured control law.
  if (!is_finite(attitude_gain) || !is_finite(angular_velocity_gain) ||
      !is_finite(acceleration_gain) || attitude_gain.x < 0.0 || attitude_gain.y < 0.0 ||
      attitude_gain.z < 0.0 || angular_velocity_gain.x < 0.0 ||
      angular_velocity_gain.y < 0.0 || angular_velocity_gain.z < 0.0 ||
      acceleration_gain.x < 0.0 || acceleration_gain.y < 0.0 ||
      acceleration_gain.z < 0.0) {
    throw std::invalid_argument("SE3 normalized-torque gains must be finite and non-negative.");
  }

  if (!is_finite(attitude) || !is_finite(angular_velocity) || !is_finite(desired.attitude) ||
      !is_finite(desired.angular_velocity) || !is_finite(desired.angular_acceleration)) {
    throw std::invalid_argument("SE3 geometric-normalized torque requires finite inputs.");
  }

  // Geometric attitude error, Lee et al. (2010), Eq. (10).
  const Vector3 attitude_error = attitude_geometry::error(attitude, desired.attitude);

  const Vector3 desired_angular_velocity_current_body =
      attitude_geometry::rotate_desired_to_current_body(attitude, desired.attitude, desired.angular_velocity);

  // Lee 2010, Eq. (11):
  //
  //   e_Omega = Omega - R^T R_d Omega_d
  //
  // Omega_d is the angular velocity of R_d itself, not a body-rate command.
  const Vector3 angular_velocity_error = angular_velocity - desired_angular_velocity_current_body;

  // Time derivative of R^T R_d Omega_d, expressed in the current body frame:
  //
  //   alpha_ff = -Omega x (R^T R_d Omega_d)
  //              + R^T R_d dot(Omega_d)
  const Vector3 angular_acceleration_feedforward =
      -cross(angular_velocity, desired_angular_velocity_current_body) +
      attitude_geometry::rotate_desired_to_current_body(attitude, desired.attitude,
                                              desired.angular_acceleration);

  // Toolkit normalized geometric controller:
  //
  //   tau_n = -K_R^n e_R - K_Omega^n e_Omega
  //           + K_alpha^n alpha_ff
  //
  // This deliberately preserves Lee's SO(3) tracking errors but does not
  // implement Lee 2010 Eq. (13): there is no physical inertia J, gyroscopic
  // moment Omega x J Omega, or N*m output here. tau_n is directly in the
  // normalized coordinates expected by PX4 VehicleTorqueSetpoint.
  const Vector3 attitude_feedback = -component_product(attitude_gain, attitude_error);

  const Vector3 angular_velocity_feedback =
      -component_product(angular_velocity_gain, angular_velocity_error);

  const Vector3 acceleration_feedforward =
      component_product(acceleration_gain, angular_acceleration_feedforward);

  return {
      attitude_feedback,
      angular_velocity_feedback,
      acceleration_feedforward,
      attitude_feedback + angular_velocity_feedback + acceleration_feedforward,
  };
}

} // namespace offboard_controllers::se3::normalized_torque
