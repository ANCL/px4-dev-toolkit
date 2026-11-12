// Mass-normalized SE(3) translation: controller output is specific force.
// Vehicle mass is required only at physical actuator/diagnostic boundaries.

#include <offboard_controllers/se3/control/translation.hpp>

#include <cmath>
#include <stdexcept>

#include <offboard_controllers/constants.hpp>
#include <offboard_controllers/math/operations.hpp>

namespace offboard_controllers::se3::translation {

TranslationalOutput compute(const State& state, const Reference& reference,
                            double kx_over_mass, double kv_over_mass) {
  if (!std::isfinite(kx_over_mass) || kx_over_mass <= 0.0 ||
      !std::isfinite(kv_over_mass) || kv_over_mass <= 0.0) {
    throw std::invalid_argument("SE3 normalized translation gains must be positive and finite.");
  }

  if (!is_finite(state.position) || !is_finite(state.velocity) || !is_finite(reference.position) ||
      !is_finite(reference.velocity) || !is_finite(reference.acceleration) ||
      !std::isfinite(reference.yaw)) {
    throw std::invalid_argument("SE3 state and reference values must be finite.");
  }

  const Vector3 position_error = state.position - reference.position;
  const Vector3 velocity_error = state.velocity - reference.velocity;
  const Vector3 acceleration =
      reference.acceleration - kx_over_mass * position_error - kv_over_mass * velocity_error;

  const Vector3 gravity{0.0, 0.0, kStandardGravity};

  // B = A/m. Its direction and normalized thrust are mass-independent.
  const Vector3 specific_force = acceleration - gravity;
  return {acceleration, specific_force};
}

} // namespace offboard_controllers::se3::translation
