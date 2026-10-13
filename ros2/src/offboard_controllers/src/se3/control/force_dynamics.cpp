// First and second desired-force derivatives from rigid-body dynamics.

#include <offboard_controllers/se3/control/force_dynamics.hpp>

#include <cmath>
#include <stdexcept>

#include <offboard_controllers/constants.hpp>
#include <offboard_controllers/math/operations.hpp>

namespace offboard_controllers::se3::force_dynamics {

Vector3 first_derivative(const State& state, const Reference& reference, const Vector3& force,
                         double mass, double kx, double kv) {
  // These laws divide by mass and require positive physical gains.
  if (!std::isfinite(mass) || mass <= 0.0 || !std::isfinite(kx) || kx <= 0.0 ||
      !std::isfinite(kv) || kv <= 0.0) {
    throw std::invalid_argument("SE3 force dynamics require positive finite mass and gains.");
  }

  if (!is_finite(state.velocity) || !is_finite(state.attitude) || !is_finite(reference.velocity) ||
      !is_finite(reference.acceleration) || !is_finite(reference.jerk) || !is_finite(force)) {
    throw std::invalid_argument("SE3 force derivative requires finite state, reference, "
                                "and force data.");
  }

  const Vector3 velocity_error = state.velocity - reference.velocity;

  // Analytic force derivatives follow the Lee/FDCL model in NED:
  //
  //   A = -k_x e_x - k_v e_v - m g e3 + m x_ddot_d
  //
  // Model-based derivatives avoid differentiating noisy state estimates:
  //   f = -A . b3,  a_model = g e3 - (f/m)b3
  // FDCL reference: fdcl-gwu/uav_geometric_control at
  // f5fcb51c3b962152a4895a9d2e86e4a184655741.
  const double collective_thrust = -dot(force, state.attitude.b3);

  const Vector3 gravity{
      0.0,
      0.0,
      kStandardGravity,
  };

  const Vector3 model_acceleration = gravity - (collective_thrust / mass) * state.attitude.b3;

  const Vector3 acceleration_error = model_acceleration - reference.acceleration;

  //   A_dot = m x_d^(3) - k_x e_v - k_v e_a
  return mass * reference.jerk - kx * velocity_error - kv * acceleration_error;
}

Vector3 second_derivative(const State& state, const Reference& reference, const Vector3& force,
                          const Vector3& force_derivative, double mass, double kx, double kv) {
  // These laws divide by mass and require positive physical gains.
  if (!std::isfinite(mass) || mass <= 0.0 || !std::isfinite(kx) || kx <= 0.0 ||
      !std::isfinite(kv) || kv <= 0.0) {
    throw std::invalid_argument("SE3 force dynamics require positive finite mass and gains.");
  }

  if (!is_finite(state.attitude) || !is_finite(state.angular_velocity) ||
      !is_finite(reference.acceleration) || !is_finite(reference.jerk) ||
      !is_finite(reference.snap) || !is_finite(force) || !is_finite(force_derivative)) {
    throw std::invalid_argument("SE3 force second derivative requires finite state, reference, "
                                "and force data.");
  }

  const Vector3 body_z_axis = state.attitude.b3;

  const double collective_thrust = -dot(force, body_z_axis);

  const Vector3 gravity{
      0.0,
      0.0,
      kStandardGravity,
  };

  const Vector3 model_acceleration = gravity - (collective_thrust / mass) * body_z_axis;

  // R_dot = R hat(Omega), so
  //
  //   b3_dot = R hat(Omega) e3.
  //
  // Omega is the measured FRD body angular velocity; b3_dot is therefore
  // expressed in NED, like A and the translational reference derivatives.
  const Vector3 body_z_axis_derivative = rotate_body_to_inertial(
      state.attitude, cross(state.angular_velocity, Vector3{0.0, 0.0, 1.0}));

  // Differentiate f = -A . b3 analytically:
  //
  //   f_dot = -A_dot . b3 - A . b3_dot.
  const double collective_thrust_derivative =
      -dot(force_derivative, body_z_axis) - dot(force, body_z_axis_derivative);

  // From a_model = g e3 - (f / m)b3:
  //
  //   j_model = -(f_dot / m)b3 - (f / m)b3_dot.
  //
  // This avoids the noise amplification and delay of finite-differencing an
  // estimated acceleration signal.
  const Vector3 model_jerk = -(collective_thrust_derivative / mass) * body_z_axis -
                             (collective_thrust / mass) * body_z_axis_derivative;

  const Vector3 acceleration_error = model_acceleration - reference.acceleration;

  const Vector3 jerk_error = model_jerk - reference.jerk;

  //   A_ddot = m x_d^(4) - k_x e_a - k_v e_j
  return mass * reference.snap - kx * acceleration_error - kv * jerk_error;
}

} // namespace offboard_controllers::se3::force_dynamics
