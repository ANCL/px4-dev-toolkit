// SO(3) attitude-error and body-frame transformation interfaces.

#pragma once

#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::se3::attitude_geometry {

// SO(3) attitude error in the current FRD frame (Lee et al., 2010, Eq. 10).
Vector3 error(const RotationMatrix& attitude, const RotationMatrix& desired_attitude);

// Transform a desired FRD vector through NED into the current FRD frame.
Vector3 rotate_desired_to_current_body(const RotationMatrix& attitude,
                                       const RotationMatrix& desired_attitude,
                                       const Vector3& desired_vector);

} // namespace offboard_controllers::se3::attitude_geometry
