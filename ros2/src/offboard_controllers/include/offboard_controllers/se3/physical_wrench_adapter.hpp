#pragma once

#include <array>
#include <cstddef>

#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::physical_wrench_adapter
{

// The physical-wrench adapter currently targets a conventional quadrotor.
// All geometry and actuator data are supplied by the vehicle configuration.
constexpr std::size_t kRotorCount = 4;


struct PhysicalRotor
{
  // Rotor position relative to the vehicle center of mass in FRD [m].
  se3::Vector3 position_frd_m{};

  // Signed reaction-moment / thrust ratio [m].
  //
  // Positive means positive FRD yaw moment for positive rotor thrust.
  double yaw_moment_ratio{0.0};
};


struct AllocatorRotor
{
  // PX4 control-allocation rotor position in FRD [m].
  se3::Vector3 position_frd_m{};

  // PX4 CA_ROTOR*_CT and CA_ROTOR*_KM values.
  double thrust_coefficient{0.0};
  double moment_ratio{0.0};
};


struct ActuatorThrustCurve
{
  // Physical rotor thrust produced by a normalized PX4 motor-control value u:
  //
  //   T(u) = constant_n
  //        + linear_n * u
  //        + quadratic_n * u^2
  //
  // This curve describes the complete actuator-to-thrust relationship seen
  // downstream of the PX4 control allocator. Its source may be a simulated
  // vehicle model or the corresponding physical vehicle model; the adapter
  // itself is platform-independent.
  double constant_n{0.0};
  double linear_n{0.0};
  double quadratic_n{0.0};
};


struct Parameters
{
  std::array<PhysicalRotor, kRotorCount> physical_rotors{};
  std::array<AllocatorRotor, kRotorCount> allocator_rotors{};

  ActuatorThrustCurve actuator_thrust{};

  // Valid normalized PX4 motor-control interval represented by the thrust
  // curve above.
  double actuator_control_min{0.0};
  double actuator_control_max{1.0};
};


struct Output
{
  // Values published through PX4 VehicleTorqueSetpoint and
  // VehicleThrustSetpoint.
  se3::Vector3 normalized_torque{};
  double normalized_thrust_z{0.0};

  // Intermediate values retained for diagnostics and verification.
  std::array<double, kRotorCount> rotor_thrust_n{};
  std::array<double, kRotorCount> actuator_control{};

  // Uniform scale applied to the requested physical body moment to keep every
  // rotor inside the configured thrust envelope. A value of 1 means the
  // requested moment was feasible without saturation.
  double moment_scale{1.0};

  // Collective thrust after applying the configured propulsion envelope.
  double applied_collective_thrust_n{0.0};
};


// Physical-wrench adapter contract:
//
// Inputs:
//   collective_thrust_n  positive collective magnitude along FRD body -Z [N]
//   moment_nm             requested FRD body moment [N m]
//   parameters            physical rotors, propulsion curve, and pinned PX4
//                         allocator geometry
//
// Logic:
//   physical wrench -> feasible rotor thrusts -> normalized motor controls
//   -> pinned-PX4 allocator coordinates. Collective is bounded first; body
//   moment is uniformly scaled only when required for rotor feasibility.
//
// Output:
//   PX4-normalized torque/thrust plus physical allocation diagnostics.
//   No assumed maximum-torque constant is introduced.
Output adapt(
  const Parameters & parameters,
  double collective_thrust_n,
  const se3::Vector3 & moment_nm);

}  // namespace offboard_controllers::physical_wrench_adapter
