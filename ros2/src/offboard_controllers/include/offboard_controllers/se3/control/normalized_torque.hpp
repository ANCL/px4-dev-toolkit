// Geometric attitude feedback in PX4-normalized torque coordinates.

#pragma once

#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::se3::normalized_torque {

// Direct SO(3) control in PX4-normalized torque coordinates (not N m).
// Each of the three gain vectors is validated by this control law.
GeometricNormalizedOutput compute(const RotationMatrix& attitude, const Vector3& angular_velocity,
                                  const DesiredAttitudeDynamics& desired,
                                  const Vector3& attitude_gain,
                                  const Vector3& angular_velocity_gain,
                                  const Vector3& acceleration_gain);

} // namespace offboard_controllers::se3::normalized_torque
