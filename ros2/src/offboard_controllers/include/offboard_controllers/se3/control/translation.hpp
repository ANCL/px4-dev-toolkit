// Translational feedback interface producing acceleration and desired force.

#pragma once

#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::se3::translation {

// Return commanded NED acceleration and specific force B = A/m [m/s^2].
// Gains are mass-normalized: kx/m [1/s^2], kv/m [1/s].
TranslationalOutput compute(const State& state, const Reference& reference,
                            double kx_over_mass, double kv_over_mass);

} // namespace offboard_controllers::se3::translation
