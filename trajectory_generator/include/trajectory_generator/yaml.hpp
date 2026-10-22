// Host-side YAML configuration API. Not required by embedded trajectory sampling.
#pragma once

#include <string>
#include <trajectory_generator/trajectory.hpp>

namespace trajectory_generator {

// Load a named trajectory or reusable segment from YAML. Invalid definitions
// and unknown names throw std::invalid_argument before controller activation.
ConfiguredTrajectory load_trajectory(const std::string& path, const std::string& name);

}  // namespace trajectory_generator
