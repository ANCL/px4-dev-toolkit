// Physical geometric moment control with rigid-body inertia terms.

#pragma once

#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::se3::physical_moment {

// Rigid-body geometric moment in current FRD body frame [N m].
// Inertia [kg m^2], k_r [N m], k_omega [N m s]; inputs must be finite.
Vector3 compute(const RotationMatrix& attitude, const Vector3& angular_velocity,
                const DesiredAttitudeDynamics& desired, const InertiaMatrix& inertia,
                const Vector3& k_r, const Vector3& k_omega);

} // namespace offboard_controllers::se3::physical_moment
