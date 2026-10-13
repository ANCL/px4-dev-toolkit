// SO(3) attitude error and desired-to-current body-frame mapping.

#include <offboard_controllers/se3/control/attitude_geometry.hpp>

#include <stdexcept>

#include <offboard_controllers/math/operations.hpp>

namespace offboard_controllers::se3::attitude_geometry {

Vector3 error(const RotationMatrix& attitude, const RotationMatrix& desired_attitude) {
  if (!is_finite(attitude) || !is_finite(desired_attitude)) {
    throw std::invalid_argument("SE3 attitude error requires finite rotations.");
  }

  // Lee et al. (2010), Eq. (10): e_R = 1/2 (R_d^T R - R^T R_d)^vee.
  // Both rotations map FRD to NED; e_R is in the current FRD frame.
  return {
      0.5 * (dot(desired_attitude.b3, attitude.b2) - dot(desired_attitude.b2, attitude.b3)),
      0.5 * (dot(desired_attitude.b1, attitude.b3) - dot(desired_attitude.b3, attitude.b1)),
      0.5 * (dot(desired_attitude.b2, attitude.b1) - dot(desired_attitude.b1, attitude.b2)),
  };
}

// Transform a desired-body FRD vector through NED into the current
// vehicle FRD frame. The same mapping is used by all attitude laws.
Vector3 rotate_desired_to_current_body(const RotationMatrix& attitude,
                                       const RotationMatrix& desired_attitude,
                                       const Vector3& desired_vector) {
  return rotate_inertial_to_body(
      attitude, rotate_body_to_inertial(desired_attitude, desired_vector));
}

} // namespace offboard_controllers::se3::attitude_geometry
