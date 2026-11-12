// Analytic derivatives of specific force B = A/m in NED.
// No physical-mass conversions occur in the geometric reference cascade.

#include <offboard_controllers/se3/control/force_dynamics.hpp>

#include <cmath>
#include <stdexcept>

#include <offboard_controllers/constants.hpp>
#include <offboard_controllers/math/operations.hpp>

namespace offboard_controllers::se3::force_dynamics {
namespace {
void validate_gains(double kx_over_mass, double kv_over_mass) {
  if (!std::isfinite(kx_over_mass) || kx_over_mass <= 0.0 ||
      !std::isfinite(kv_over_mass) || kv_over_mass <= 0.0) {
    throw std::invalid_argument("SE3 normalized force-derivative gains must be positive and finite.");
  }
}
} // namespace

Vector3 first_derivative(const State& state, const Reference& reference,
                         const Vector3& specific_force, double kx_over_mass,
                         double kv_over_mass) {
  validate_gains(kx_over_mass, kv_over_mass);
  if (!is_finite(state.velocity) || !is_finite(state.attitude) ||
      !is_finite(reference.velocity) || !is_finite(reference.acceleration) ||
      !is_finite(reference.jerk) || !is_finite(specific_force)) {
    throw std::invalid_argument("SE3 first specific-force derivative has invalid state/reference.");
  }

  const Vector3 gravity{0.0, 0.0, kStandardGravity};
  const Vector3 body_z = state.attitude.b3;
  const double collective_acceleration = -dot(specific_force, body_z);
  const Vector3 model_acceleration = gravity - collective_acceleration * body_z;

  const Vector3 velocity_error = state.velocity - reference.velocity;
  const Vector3 acceleration_error = model_acceleration - reference.acceleration;

  // B_dot = j_d - (kx/m) e_v - (kv/m) e_a.
  return reference.jerk - kx_over_mass * velocity_error - kv_over_mass * acceleration_error;
}

Vector3 second_derivative(const State& state, const Reference& reference,
                          const Vector3& specific_force, const Vector3& first,
                          double kx_over_mass, double kv_over_mass) {
  validate_gains(kx_over_mass, kv_over_mass);
  if (!is_finite(state.attitude) || !is_finite(state.angular_velocity) ||
      !is_finite(reference.acceleration) || !is_finite(reference.jerk) ||
      !is_finite(reference.snap) || !is_finite(specific_force) || !is_finite(first)) {
    throw std::invalid_argument("SE3 second specific-force derivative has invalid state/reference.");
  }

  const Vector3 gravity{0.0, 0.0, kStandardGravity};
  const Vector3 body_z = state.attitude.b3;
  const double collective_acceleration = -dot(specific_force, body_z);
  const Vector3 model_acceleration = gravity - collective_acceleration * body_z;

  // b3_dot = R (Omega x e3), computed from the measured body rate.
  const Vector3 body_z_first = rotate_body_to_inertial(
      state.attitude, cross(state.angular_velocity, Vector3{0.0, 0.0, 1.0}));
  const double collective_first =
      -dot(first, body_z) - dot(specific_force, body_z_first);
  const Vector3 model_jerk = -collective_first * body_z - collective_acceleration * body_z_first;

  const Vector3 acceleration_error = model_acceleration - reference.acceleration;
  const Vector3 jerk_error = model_jerk - reference.jerk;

  // B_ddot = snap_d - (kx/m) e_a - (kv/m) e_j.
  return reference.snap - kx_over_mass * acceleration_error - kv_over_mass * jerk_error;
}

} // namespace offboard_controllers::se3::force_dynamics
