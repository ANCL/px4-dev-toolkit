// Physical and PX4 effectiveness matrices for a four-rotor multicopter.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

#include "internal.hpp"
#include <offboard_controllers/math/operations.hpp>

namespace offboard_controllers::physical_wrench_adapter::detail
{

constexpr double kMatrixTolerance = 1.0e-12;

// Fixed-size Gauss-Jordan inversion with partial pivoting. The conventional
// quadrotor adapter has four independent wrench coordinates and four rotors,
// so both physical and PX4 effectiveness matrices are square 4x4 systems.
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
  // Reproduce the normalization applied by the pinned PX4
  // ControlAllocationPseudoInverse. Roll and pitch share the larger column
  // norm, while yaw and thrust use their respective mixer-column scales.
  //
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

}  // namespace offboard_controllers::physical_wrench_adapter::detail
