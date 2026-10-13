// Geometric attitude-to-body-rate control interface.

#pragma once

#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::se3::attitude_control {

// Geometric attitude feedback with feedforward; returns FRD body-rate setpoint [rad/s].
// Negative or non-finite gain components are rejected at this interface.
Vector3 compute_rate(const RotationMatrix& attitude, const DesiredAttitudeRate& desired,
                     const Vector3& attitude_gain);

} // namespace offboard_controllers::se3::attitude_control
