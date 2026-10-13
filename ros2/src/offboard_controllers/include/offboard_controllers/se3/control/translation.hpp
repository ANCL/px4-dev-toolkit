// Translational feedback interface producing acceleration and desired force.

#pragma once

#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::se3::translation {

// Return NED acceleration [m/s^2] and desired force [N].
// Reject invalid mass [kg] and physical gains [N/m, N s/m].
TranslationalOutput compute(const State& state, const Reference& reference, double mass, double kx,
                            double kv);

} // namespace offboard_controllers::se3::translation
