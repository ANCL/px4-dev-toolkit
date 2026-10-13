// Cascaded attitude control producing a PX4 body-rate setpoint rather
// than a direct body moment command.

#include <offboard_controllers/se3/control/attitude_control.hpp>

#include <stdexcept>

#include <offboard_controllers/math/operations.hpp>
#include <offboard_controllers/se3/control/attitude_geometry.hpp>

namespace offboard_controllers::se3::attitude_control {

Vector3 compute_rate(const RotationMatrix& attitude, const DesiredAttitudeRate& desired,
                     const Vector3& attitude_gain) {
  // The gain is a non-negative FRD body-rate feedback coefficient [1/s].
  if (!is_finite(attitude_gain) || attitude_gain.x < 0.0 || attitude_gain.y < 0.0 ||
      attitude_gain.z < 0.0) {
    throw std::invalid_argument("SE3 attitude gains must be finite and non-negative.");
  }

  if (!is_finite(attitude) || !is_finite(desired.attitude) ||
      !is_finite(desired.angular_velocity)) {
    throw std::invalid_argument("SE3 attitude-rate command requires finite inputs.");
  }

  const Vector3 attitude_error = attitude_geometry::error(attitude, desired.attitude);

  const Vector3 feedforward =
      attitude_geometry::rotate_desired_to_current_body(attitude, desired.attitude, desired.angular_velocity);

  // Cascaded attitude-to-rate law (distinct from direct moment control):
  //   Omega_sp = R^T R_d Omega_d - K_att e_R
  return feedforward - component_product(attitude_gain, attitude_error);
}

} // namespace offboard_controllers::se3::attitude_control
