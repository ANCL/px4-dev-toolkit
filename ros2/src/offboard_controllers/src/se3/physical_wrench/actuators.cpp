// Converts between actuator control values and physical rotor thrust.

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

#include "internal.hpp"

namespace offboard_controllers::physical_wrench_adapter::detail
{

void require_finite(
  double value,
  const char * message)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(message);
  }
}

// Rotor thrust model: T(u) = c0 + c1*u + c2*u^2 [N].
// The normalized actuator command u is dimensionless.
double thrust_from_control(
  const Parameters & parameters,
  double control)
{
  const ActuatorThrustCurve & curve =
    parameters.actuator_thrust;

  return
    curve.constant_n +
    curve.linear_n * control +
    curve.quadratic_n * control * control;
}


// Invert the thrust curve on its monotonic configured control interval.
// Linear curves use the direct inverse; quadratic curves select the feasible root.
double control_from_thrust(
  const Parameters & parameters,
  double thrust_n)
{
  const double minimum_control =
    parameters.actuator_control_min;

  const double maximum_control =
    parameters.actuator_control_max;

  const double minimum_thrust =
    thrust_from_control(
      parameters,
      minimum_control);

  const double maximum_thrust =
    thrust_from_control(
      parameters,
      maximum_control);

  if (
    thrust_n < minimum_thrust - kFeasibilityTolerance ||
    thrust_n > maximum_thrust + kFeasibilityTolerance)
  {
    throw std::domain_error(
            "Physical rotor thrust is outside the configured "
            "actuator-thrust range.");
  }

  const double bounded_thrust =
    std::clamp(
      thrust_n,
      minimum_thrust,
      maximum_thrust);

  const ActuatorThrustCurve & curve =
    parameters.actuator_thrust;

  constexpr double coefficient_tolerance =
    1.0e-12;

  double control{};

  if (
    std::abs(curve.quadratic_n) <=
    coefficient_tolerance)
  {
    if (
      std::abs(curve.linear_n) <=
      coefficient_tolerance)
    {
      throw std::invalid_argument(
              "Physical actuator-thrust curve is not invertible.");
    }

    control =
      (
        bounded_thrust -
        curve.constant_n
      ) /
      curve.linear_n;

  } else {
    const double discriminant =
      curve.linear_n * curve.linear_n -
      4.0 *
      curve.quadratic_n *
      (
        curve.constant_n -
        bounded_thrust
      );

    if (discriminant < -coefficient_tolerance) {
      throw std::domain_error(
              "Physical actuator-thrust curve has no real inverse.");
    }

    const double root =
      std::sqrt(
        std::max(
          discriminant,
          0.0));

    const std::array<double, 2> candidates{
      (
        -curve.linear_n +
        root
      ) /
      (
        2.0 *
        curve.quadratic_n
      ),
      (
        -curve.linear_n -
        root
      ) /
      (
        2.0 *
        curve.quadratic_n
      ),
    };

    bool found = false;

    for (const double candidate : candidates) {
      if (
        candidate >=
        minimum_control -
        kFeasibilityTolerance &&
        candidate <=
        maximum_control +
        kFeasibilityTolerance)
      {
        control =
          std::clamp(
            candidate,
            minimum_control,
            maximum_control);

        found = true;
        break;
      }
    }

    if (!found) {
      throw std::domain_error(
              "Physical actuator-thrust inverse lies outside "
              "the configured control range.");
    }
  }

  return std::clamp(
    control,
    minimum_control,
    maximum_control);
}

void validate_parameters(
  const Parameters & parameters)
{
  require_finite(
    parameters.actuator_control_min,
    "Physical-wrench minimum actuator control must be finite.");

  require_finite(
    parameters.actuator_control_max,
    "Physical-wrench maximum actuator control must be finite.");

  require_finite(
    parameters.actuator_thrust.constant_n,
    "Physical-wrench thrust-curve constant must be finite.");

  require_finite(
    parameters.actuator_thrust.linear_n,
    "Physical-wrench thrust-curve linear coefficient must be finite.");

  require_finite(
    parameters.actuator_thrust.quadratic_n,
    "Physical-wrench thrust-curve quadratic coefficient must be finite.");

  if (
    parameters.actuator_control_min < 0.0 ||
    parameters.actuator_control_max >
    1.0 ||
    parameters.actuator_control_max <=
    parameters.actuator_control_min)
  {
    throw std::invalid_argument(
            "Physical-wrench actuator-control range must be "
            "a non-empty subset of [0, 1].");
  }

  const double minimum_thrust =
    thrust_from_control(
      parameters,
      parameters.actuator_control_min);

  const double maximum_thrust =
    thrust_from_control(
      parameters,
      parameters.actuator_control_max);

  // A strictly increasing thrust curve over the complete configured control
  // interval guarantees a unique physical thrust -> control inversion.
  const double derivative_at_minimum =
    parameters.actuator_thrust.linear_n +
    2.0 *
    parameters.actuator_thrust.quadratic_n *
    parameters.actuator_control_min;

  const double derivative_at_maximum =
    parameters.actuator_thrust.linear_n +
    2.0 *
    parameters.actuator_thrust.quadratic_n *
    parameters.actuator_control_max;

  if (
    !std::isfinite(minimum_thrust) ||
    !std::isfinite(maximum_thrust) ||
    minimum_thrust < 0.0 ||
    maximum_thrust <= minimum_thrust ||
    derivative_at_minimum <= 0.0 ||
    derivative_at_maximum <= 0.0)
  {
    throw std::invalid_argument(
            "Physical-wrench actuator-thrust curve must be "
            "finite, non-negative, and strictly increasing.");
  }

  // Construct both matrices here so invalid or singular vehicle data fail
  // deterministically before any conversion is attempted.
  (void)inverse(
    physical_effectiveness(
      parameters));

  (void)allocator_normalization_scale(
    allocator_effectiveness(
      parameters));
}

}  // namespace offboard_controllers::physical_wrench_adapter::detail
