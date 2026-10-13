// SE(3) translational tracking law. Returns kinematic acceleration
// for PX4 and a gravity-compensated physical force reference.

#include <offboard_controllers/se3/control/translation.hpp>

#include <cmath>
#include <stdexcept>

#include <offboard_controllers/constants.hpp>
#include <offboard_controllers/math/operations.hpp>

namespace offboard_controllers::se3::translation {

TranslationalOutput compute(const State& state, const Reference& reference, double mass, double kx,
                            double kv) {
  if (!std::isfinite(mass) || mass <= 0.0 || !std::isfinite(kx) || kx <= 0.0 ||
      !std::isfinite(kv) || kv <= 0.0) {
    throw std::invalid_argument("SE3 translation requires positive finite mass and gains.");
  }

  if (!is_finite(state.position) || !is_finite(state.velocity) || !is_finite(reference.position) ||
      !is_finite(reference.velocity) || !is_finite(reference.acceleration) ||
      !std::isfinite(reference.yaw)) {
    throw std::invalid_argument("SE3 state and reference values must be finite.");
  }

  // Translational tracking with feedforward acceleration:
  //   e_x = x - x_d,  e_v = v - v_d
  //   a_cmd = a_d - (k_x/m)e_x - (k_v/m)e_v
  // a_cmd excludes gravity at the PX4 acceleration-setpoint boundary.
  const Vector3 position_error = state.position - reference.position;

  const Vector3 velocity_error = state.velocity - reference.velocity;
  const Vector3 acceleration =
      reference.acceleration - (kx / mass) * position_error - (kv / mass) * velocity_error;

  const Vector3 gravity{
      0.0,
      0.0,
      kStandardGravity,
  };

  // Desired force with gravity compensation: F_d = m(a_cmd - g e_3).
  const Vector3 force_vector = mass * (acceleration - gravity);

  return {
      acceleration,
      force_vector,
  };
}

} // namespace offboard_controllers::se3::translation
