#pragma once

#include <array>

#include <offboard_controllers/math/types.hpp>

namespace offboard_controllers::px4_wrench
{

double normalized_projected_collective_thrust(
  const math::Vector3 & force,
  const math::RotationMatrix & attitude,
  double mass,
  double hover_thrust);

// PX4 attitude quaternions use Hamilton [w, x, y, z] and rotate FRD body
// vectors into the NED inertial frame.
std::array<double, 4> quaternion_from_rotation(
  const math::RotationMatrix & rotation);

math::RotationMatrix rotation_from_quaternion(
  const std::array<double, 4> & quaternion);

}  // namespace offboard_controllers::px4_wrench
