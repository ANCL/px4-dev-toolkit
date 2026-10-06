/*
 * Physical wrench -> normalized PX4 multicopter control adapter.
 *
 * PX4 source of truth:
 *   ANCL/PX4-Autopilot
 *   commit f5083ca2c5b919350880e8667e636ef70715db17
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
#include <limits>
#include <stdexcept>

#include <offboard_controllers/se3/physical_wrench_adapter.hpp>
#include <offboard_controllers/math/operations.hpp>

namespace offboard_controllers::physical_wrench_adapter
{

namespace
{

using Matrix4 =
  std::array<
    std::array<double, kRotorCount>,
    kRotorCount>;

using Vector4 =
  std::array<double, kRotorCount>;

constexpr double kMatrixTolerance = 1.0e-12;
constexpr double kFeasibilityTolerance = 1.0e-9;


void require_finite(
  double value,
  const char * message)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(message);
  }
}


Matrix4 inverse(
  Matrix4 matrix)
{
  Matrix4 result{};

  for (std::size_t row = 0; row < kRotorCount; ++row) {
    result[row][row] = 1.0;
  }

  for (std::size_t column = 0; column < kRotorCount; ++column) {
    std::size_t pivot_row = column;

    for (
      std::size_t row = column + 1;
      row < kRotorCount;
      ++row)
    {
      if (
        std::abs(matrix[row][column]) >
        std::abs(matrix[pivot_row][column]))
      {
        pivot_row = row;
      }
    }

    if (
      !std::isfinite(matrix[pivot_row][column]) ||
      std::abs(matrix[pivot_row][column]) <=
      kMatrixTolerance)
    {
      throw std::invalid_argument(
              "Physical-wrench/PX4 adapter matrix is singular.");
    }

    if (pivot_row != column) {
      std::swap(
        matrix[pivot_row],
        matrix[column]);

      std::swap(
        result[pivot_row],
        result[column]);
    }

    const double pivot =
      matrix[column][column];

    for (
      std::size_t entry = 0;
      entry < kRotorCount;
      ++entry)
    {
      matrix[column][entry] /= pivot;
      result[column][entry] /= pivot;
    }

    for (std::size_t row = 0; row < kRotorCount; ++row) {
      if (row == column) {
        continue;
      }

      const double factor =
        matrix[row][column];

      for (
        std::size_t entry = 0;
        entry < kRotorCount;
        ++entry)
      {
        matrix[row][entry] -=
          factor *
          matrix[column][entry];

        result[row][entry] -=
          factor *
          result[column][entry];
      }
    }
  }

  return result;
}


Vector4 multiply(
  const Matrix4 & matrix,
  const Vector4 & vector)
{
  Vector4 result{};

  for (std::size_t row = 0; row < kRotorCount; ++row) {
    for (
      std::size_t column = 0;
      column < kRotorCount;
      ++column)
    {
      result[row] +=
        matrix[row][column] *
        vector[column];
    }
  }

  return result;
}


Matrix4 physical_effectiveness(
  const Parameters & parameters)
{
  Matrix4 effectiveness{};

  for (std::size_t rotor = 0; rotor < kRotorCount; ++rotor) {
    const PhysicalRotor & data =
      parameters.physical_rotors[rotor];

    if (
      !math::is_finite(data.position_frd_m) ||
      !std::isfinite(data.yaw_moment_ratio))
    {
      throw std::invalid_argument(
              "Physical-wrench/PX4 physical rotor data must be finite.");
    }

    // A rotor produces force [0, 0, -T] in FRD.
    //
    // r x F =
    //   [-y T, x T, 0]
    //
    // The signed reaction-moment ratio supplies the FRD yaw moment.
    effectiveness[0][rotor] =
      -data.position_frd_m.y;

    effectiveness[1][rotor] =
      data.position_frd_m.x;

    effectiveness[2][rotor] =
      data.yaw_moment_ratio;

    effectiveness[3][rotor] =
      1.0;
  }

  return effectiveness;
}


Matrix4 allocator_effectiveness(
  const Parameters & parameters)
{
  Matrix4 effectiveness{};

  for (std::size_t rotor = 0; rotor < kRotorCount; ++rotor) {
    const AllocatorRotor & data =
      parameters.allocator_rotors[rotor];

    if (
      !math::is_finite(data.position_frd_m) ||
      !std::isfinite(data.moment_ratio) ||
      !std::isfinite(data.thrust_coefficient) ||
      data.thrust_coefficient <= 0.0)
    {
      throw std::invalid_argument(
              "Physical-wrench/PX4 allocator rotor data are invalid.");
    }

    // Pinned PX4 ActuatorEffectivenessRotors for an upward multicopter
    // rotor whose normalized axis is [0, 0, -1]:
    //
    //   thrust = CT * axis
    //   moment = CT * position x axis - CT * KM * axis
    //
    // Active rows are roll, pitch, yaw, and body-Z thrust.
    const double ct =
      data.thrust_coefficient;

    effectiveness[0][rotor] =
      -ct * data.position_frd_m.y;

    effectiveness[1][rotor] =
      ct * data.position_frd_m.x;

    effectiveness[2][rotor] =
      ct * data.moment_ratio;

    effectiveness[3][rotor] =
      -ct;
  }

  return effectiveness;
}


Vector4 allocator_normalization_scale(
  const Matrix4 & effectiveness)
{
  // For four independent multicopter controls, the Moore-Penrose
  // pseudo-inverse is the ordinary matrix inverse.
  const Matrix4 mix =
    inverse(effectiveness);

  int nonzero_roll = 0;
  int nonzero_pitch = 0;

  double roll_norm_squared = 0.0;
  double pitch_norm_squared = 0.0;

  for (std::size_t rotor = 0; rotor < kRotorCount; ++rotor) {
    const double roll =
      mix[rotor][0];

    const double pitch =
      mix[rotor][1];

    roll_norm_squared += roll * roll;
    pitch_norm_squared += pitch * pitch;

    if (std::abs(roll) > 1.0e-3) {
      ++nonzero_roll;
    }

    if (std::abs(pitch) > 1.0e-3) {
      ++nonzero_pitch;
    }
  }

  if (nonzero_roll == 0 || nonzero_pitch == 0) {
    throw std::invalid_argument(
            "Physical-wrench/PX4 allocator cannot control roll and pitch.");
  }

  const double roll_scale =
    std::sqrt(
      roll_norm_squared /
      (
        static_cast<double>(nonzero_roll) /
        2.0
      ));

  const double pitch_scale =
    std::sqrt(
      pitch_norm_squared /
      (
        static_cast<double>(nonzero_pitch) /
        2.0
      ));

  const double roll_pitch_scale =
    std::max(
      roll_scale,
      pitch_scale);

  double yaw_scale =
    mix[0][2];

  for (std::size_t rotor = 1; rotor < kRotorCount; ++rotor) {
    yaw_scale =
      std::max(
        yaw_scale,
        mix[rotor][2]);
  }

  constexpr double float_epsilon =
    std::numeric_limits<float>::epsilon();

  if (
    !std::isfinite(roll_pitch_scale) ||
    roll_pitch_scale <= float_epsilon ||
    !std::isfinite(yaw_scale) ||
    yaw_scale <= float_epsilon)
  {
    throw std::invalid_argument(
            "Physical-wrench/PX4 allocator normalization is invalid.");
  }

  int nonzero_thrust = 0;
  double thrust_norm_sum = 0.0;

  for (std::size_t rotor = 0; rotor < kRotorCount; ++rotor) {
    const double magnitude =
      std::abs(
        mix[rotor][3]);

    thrust_norm_sum += magnitude;

    if (magnitude > float_epsilon) {
      ++nonzero_thrust;
    }
  }

  if (nonzero_thrust == 0) {
    throw std::invalid_argument(
            "Physical-wrench/PX4 allocator cannot control collective thrust.");
  }

  const double thrust_scale =
    thrust_norm_sum /
    static_cast<double>(nonzero_thrust);

  return {
    roll_pitch_scale,
    roll_pitch_scale,
    yaw_scale,
    thrust_scale,
  };
}


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


struct FeasiblePhysicalAllocation
{
  Vector4 rotor_thrust{};
  double moment_scale{1.0};
  double applied_collective_thrust_n{0.0};
};


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

}  // namespace


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
