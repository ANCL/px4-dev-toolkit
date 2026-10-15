/*
 * Physical wrench -> normalized PX4 multicopter control adapter.
 *
 * PX4 source of truth:
 *   ANCL/PX4-Autopilot
 *   commit 9af25b30f067c36269e94a96797be7943dd4f8e1
 *
 * Relevant PX4 paths:
 *   src/modules/control_allocator/VehicleActuatorEffectiveness/
 *     ActuatorEffectivenessRotors.cpp
 *   src/lib/control_allocation/control_allocation/
 *     ControlAllocationPseudoInverse.cpp
 *
 * This is a toolkit adapter, not part of Lee's controller. Lee's controller
 * ends with a physical collective thrust and physical body moment.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include <offboard_controllers/se3/physical_wrench_adapter.hpp>
#include <offboard_controllers/math/operations.hpp>
#include "internal.hpp"

namespace offboard_controllers::physical_wrench_adapter
{

namespace
{

using detail::Matrix4;
using detail::Vector4;
using detail::kFeasibilityTolerance;
using detail::multiply;
using detail::thrust_from_control;

struct FeasiblePhysicalAllocation
{
  Vector4 rotor_thrust{};
  double moment_scale{1.0};
  double applied_collective_thrust_n{0.0};
};


// Apply feasibility without changing the requested moment direction. Clamp
// collective thrust to the propulsion envelope first, then uniformly scale the
// complete requested moment until every rotor thrust lies inside its physical
// range. A single scale preserves roll/pitch/yaw moment ratios.
FeasiblePhysicalAllocation allocate_feasible_physical_wrench(
  const Matrix4 & inverse_physical_matrix,
  const Parameters & parameters,
  double collective_thrust_n,
  const se3::Vector3 & moment_nm)
{
  const double minimum_rotor_thrust =
    thrust_from_control(
      parameters,
      parameters.actuator_control_min);

  const double maximum_rotor_thrust =
    thrust_from_control(
      parameters,
      parameters.actuator_control_max);

  const double minimum_collective_thrust =
    static_cast<double>(kRotorCount) *
    minimum_rotor_thrust;

  const double maximum_collective_thrust =
    static_cast<double>(kRotorCount) *
    maximum_rotor_thrust;

  const double applied_collective_thrust =
    std::clamp(
      collective_thrust_n,
      minimum_collective_thrust,
      maximum_collective_thrust);

  const Vector4 collective_only =
    multiply(
      inverse_physical_matrix,
      {
        0.0,
        0.0,
        0.0,
        applied_collective_thrust,
      });

  const Vector4 requested =
    multiply(
      inverse_physical_matrix,
      {
        moment_nm.x,
        moment_nm.y,
        moment_nm.z,
        applied_collective_thrust,
      });

  double moment_scale = 1.0;

  for (std::size_t rotor = 0; rotor < kRotorCount; ++rotor) {
    const double base =
      collective_only[rotor];

    if (
      base <
      minimum_rotor_thrust -
      kFeasibilityTolerance ||
      base >
      maximum_rotor_thrust +
      kFeasibilityTolerance)
    {
      throw std::domain_error(
              "Geometric physical collective thrust is outside the "
              "configured actuator-thrust range.");
    }

    const double delta =
      requested[rotor] -
      base;

    if (delta > kFeasibilityTolerance) {
      moment_scale =
        std::min(
          moment_scale,
          (
            maximum_rotor_thrust -
            base
          ) / delta);

    } else if (delta < -kFeasibilityTolerance) {
      moment_scale =
        std::min(
          moment_scale,
          (
            base -
            minimum_rotor_thrust
          ) / -delta);
    }
  }

  moment_scale =
    std::clamp(
      moment_scale,
      0.0,
      1.0);

  Vector4 rotor_thrust{};

  for (std::size_t rotor = 0; rotor < kRotorCount; ++rotor) {
    rotor_thrust[rotor] =
      std::clamp(
        collective_only[rotor] +
        moment_scale *
        (
          requested[rotor] -
          collective_only[rotor]
        ),
        minimum_rotor_thrust,
        maximum_rotor_thrust);
  }

  return {
    rotor_thrust,
    moment_scale,
    applied_collective_thrust,
  };
}

}  // namespace

using detail::inverse;
using detail::physical_effectiveness;
using detail::allocator_effectiveness;
using detail::allocator_normalization_scale;
using detail::control_from_thrust;
using detail::validate_parameters;

Output adapt(
  const Parameters & parameters,
  double collective_thrust_n,
  const se3::Vector3 & moment_nm)
{
  validate_parameters(
    parameters);

  if (
    !std::isfinite(collective_thrust_n) ||
    collective_thrust_n < 0.0 ||
    !math::is_finite(moment_nm))
  {
    throw std::invalid_argument(
            "Geometric physical wrench must contain finite values and "
            "non-negative collective thrust.");
  }

  const Matrix4 inverse_physical_matrix =
    inverse(
      physical_effectiveness(
        parameters));

  const FeasiblePhysicalAllocation allocation =
    allocate_feasible_physical_wrench(
      inverse_physical_matrix,
      parameters,
      collective_thrust_n,
      moment_nm);

  const Vector4 & rotor_thrust =
    allocation.rotor_thrust;

  std::array<double, kRotorCount> actuator_control{};

  for (std::size_t rotor = 0; rotor < kRotorCount; ++rotor) {
    actuator_control[rotor] =
      control_from_thrust(
        parameters,
        rotor_thrust[rotor]);
  }

  const Matrix4 allocator_matrix =
    allocator_effectiveness(
      parameters);

  const Vector4 allocator_control =
    multiply(
      allocator_matrix,
      actuator_control);

  const Vector4 scale =
    allocator_normalization_scale(
      allocator_matrix);

  // Pinned PX4 first computes the pseudo-inverse of the physical
  // effectiveness matrix and then divides each mixer column by these
  // normalization scales. Therefore the normalized control setpoint that
  // reconstructs a chosen actuator vector is:
  //
  //   c_normalized = scale .* (B * u)
  //
  // where B is the unnormalized PX4 effectiveness matrix.
  const se3::Vector3 normalized_torque{
    scale[0] * allocator_control[0],
    scale[1] * allocator_control[1],
    scale[2] * allocator_control[2],
  };

  const double normalized_thrust_z =
    scale[3] *
    allocator_control[3];

  if (
    !math::is_finite(normalized_torque) ||
    !std::isfinite(normalized_thrust_z))
  {
    throw std::runtime_error(
            "Physical-wrench/PX4 adapter produced a non-finite normalized wrench.");
  }

  return {
    normalized_torque,
    normalized_thrust_z,
    rotor_thrust,
    actuator_control,
    allocation.moment_scale,
    allocation.applied_collective_thrust_n,
  };
}

}  // namespace offboard_controllers::physical_wrench_adapter
