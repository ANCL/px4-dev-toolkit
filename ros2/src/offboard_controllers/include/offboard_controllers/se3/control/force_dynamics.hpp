// Analytic desired-force derivative interfaces for geometric feedforward.

#pragma once

#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::se3::force_dynamics {

// Analytic A_dot [N/s] and A_ddot [N/s^2], using nominal vehicle dynamics.
// Each stage rejects non-finite or non-positive physical parameters.
Vector3 first_derivative(const State& state, const Reference& reference, const Vector3& force,
                         double mass, double kx, double kv);
Vector3 second_derivative(const State& state, const Reference& reference, const Vector3& force,
                          const Vector3& force_derivative, double mass, double kx, double kv);

} // namespace offboard_controllers::se3::force_dynamics
