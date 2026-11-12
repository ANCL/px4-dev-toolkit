// Analytic desired-force derivative interfaces for geometric feedforward.

#pragma once

#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::se3::force_dynamics {

// Analytic derivatives of B = A/m [m/s^2] using nominal vehicle dynamics.
// The gains are kx/m and kv/m, as in the native PX4 controller.
Vector3 first_derivative(const State& state, const Reference& reference,
                         const Vector3& specific_force, double kx_over_mass, double kv_over_mass);
Vector3 second_derivative(const State& state, const Reference& reference,
                          const Vector3& specific_force, const Vector3& first,
                          double kx_over_mass, double kv_over_mass);

} // namespace offboard_controllers::se3::force_dynamics
