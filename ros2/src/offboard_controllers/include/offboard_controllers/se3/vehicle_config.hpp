#pragma once

#include <string>

#include <offboard_controllers/se3/physical_wrench_adapter.hpp>
#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::vehicle_config
{

struct PhysicalWrenchConfiguration
{
  // Physical inertia expressed in the FRD body frame expected by the
  // rotational controller.
  se3::InertiaMatrix inertia_frd{};

  // Vehicle-specific conversion from a physical body wrench to the normalized
  // wrench consumed by the pinned PX4 allocator/output chain.
  physical_wrench_adapter::Parameters adapter{};
};


// Vehicle-configuration contract:
//
// Inputs:
//   configuration directory and vehicle identifier.
//
// Logic:
//   load physical vehicle and PX4 allocator data, normalize body quantities to
//   FRD, and validate inertia, rotor geometry, signs, and propulsion mapping.
//
// Output:
//   validated physical-wrench configuration for the direct controller.
PhysicalWrenchConfiguration load_physical_wrench_configuration(
  const std::string & config_directory,
  const std::string & vehicle);

}  // namespace offboard_controllers::vehicle_config
