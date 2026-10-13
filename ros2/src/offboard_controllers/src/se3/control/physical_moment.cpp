// Lee-style rigid-body moment law producing physical body torque.

#include <offboard_controllers/se3/control/physical_moment.hpp>

#include <stdexcept>

#include <offboard_controllers/math/operations.hpp>
#include <offboard_controllers/se3/control/attitude_geometry.hpp>

namespace offboard_controllers::se3::physical_moment {

Vector3 compute(const RotationMatrix& attitude, const Vector3& angular_velocity,
                const DesiredAttitudeDynamics& desired, const InertiaMatrix& inertia,
                const Vector3& k_r, const Vector3& k_omega) {
  if (!is_finite(attitude) || !is_finite(angular_velocity) || !is_finite(desired.attitude) ||
      !is_finite(desired.angular_velocity) || !is_finite(desired.angular_acceleration) ||
      !is_finite(inertia) || !is_finite(k_r) || !is_finite(k_omega)) {
    throw std::invalid_argument("SE3 physical moment command requires finite inputs.");
  }

  const Vector3 attitude_error = attitude_geometry::error(attitude, desired.attitude);

  const Vector3 angular_velocity_feedforward =
      attitude_geometry::rotate_desired_to_current_body(attitude, desired.attitude, desired.angular_velocity);

  const Vector3 angular_velocity_error = angular_velocity - angular_velocity_feedforward;

  const Vector3 angular_acceleration_feedforward =
      -cross(angular_velocity, angular_velocity_feedforward) +
      attitude_geometry::rotate_desired_to_current_body(attitude, desired.attitude,
                                              desired.angular_acceleration);

  const Vector3 angular_momentum = multiply(inertia, angular_velocity);

  // Lee et al. (2010), Eq. (13): physical moment [N m] with gyroscopic coupling:
  // M = -K_R e_R - K_Omega e_Omega + Omega x J Omega + J alpha_ff.
  // This is physical torque, not PX4-normalized torque.
  return -component_product(k_r, attitude_error) -
         component_product(k_omega, angular_velocity_error) +
         cross(angular_velocity, angular_momentum) +
         multiply(inertia, angular_acceleration_feedforward);
}

} // namespace offboard_controllers::se3::physical_moment
