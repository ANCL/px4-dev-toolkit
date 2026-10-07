#pragma once

#include <array>

#include <offboard_controllers/math/types.hpp>

namespace offboard_controllers::px4_wrench
{

// Projected-thrust contract:
//
// Inputs:
//   desired force in NED [N], current FRD->NED attitude, vehicle mass [kg],
//   and PX4 hover-thrust calibration.
//
// Logic:
//   project force onto current FRD body -Z and normalize against hover thrust.
//
// Output:
//   positive collective magnitude in [0, 1]. Publishers apply PX4's negative
//   body-Z VehicleThrustSetpoint sign.
double normalized_projected_collective_thrust(
  const math::Vector3 & force,
  const math::RotationMatrix & attitude,
  double mass,
  double hover_thrust);

// Rotation -> quaternion contract:
//
// Input:
//   FRD-body -> NED rotation matrix.
//
// Logic:
//   convert with a numerically stable branch, normalize, and choose a
//   deterministic quaternion sign.
//
// Output:
//   PX4 Hamilton quaternion [w, x, y, z], FRD body -> NED.
std::array<double, 4> quaternion_from_rotation(
  const math::RotationMatrix & rotation);

// Quaternion -> rotation contract:
//
// Input:
//   finite, non-zero PX4 Hamilton quaternion [w, x, y, z].
//
// Logic:
//   normalize before constructing the rotation.
//
// Output:
//   FRD-body -> NED rotation matrix.
math::RotationMatrix rotation_from_quaternion(
  const std::array<double, 4> & quaternion);

}  // namespace offboard_controllers::px4_wrench
