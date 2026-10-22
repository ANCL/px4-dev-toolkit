// Trajectory reference in local NED, independent of transport or controller.

#pragma once

#include <array>

namespace trajectory_generator {

using Vector3 = std::array<double, 3>;

// Local NED reference through snap; yaw derivatives use rad/s and rad/s^2.
struct Reference
{
  Vector3 position{};
  Vector3 velocity{};
  Vector3 acceleration{};
  Vector3 jerk{};
  Vector3 snap{};

  double yaw{0.0};
  double yaw_rate{0.0};
  double yaw_acceleration{0.0};
};

}  // namespace trajectory_generator
