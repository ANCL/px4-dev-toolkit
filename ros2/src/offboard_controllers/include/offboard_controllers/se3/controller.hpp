#pragma once

#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::se3
{

// Controller mathematics are independent of ROS 2 and PX4 transport.
class Controller
{
public:
  explicit Controller(const Parameters & parameters);

  TranslationalOutput compute_translation(
    const State & state,
    const Reference & reference) const;

  // Differentiate translational control vector A analytically from the
  // nominal quadrotor dynamics; estimator acceleration is not required.
  Vector3 compute_force_derivative(
    const State & state,
    const Reference & reference,
    const Vector3 & force) const;

  // Compute A_ddot from rigid-body thrust-axis kinematics using the current
  // body angular velocity rather than a finite-difference jerk signal.
  Vector3 compute_force_second_derivative(
    const State & state,
    const Reference & reference,
    const Vector3 & force,
    const Vector3 & force_derivative) const;

  RotationMatrix compute_desired_attitude(
    const Vector3 & force,
    double yaw) const;

  DesiredAttitudeRate compute_desired_attitude_rate(
    const Vector3 & force,
    const Vector3 & force_derivative,
    double yaw,
    double yaw_rate) const;

  DesiredAttitudeDynamics compute_desired_attitude_dynamics(
    const Vector3 & force,
    const Vector3 & force_derivative,
    const Vector3 & force_second_derivative,
    double yaw,
    double yaw_rate,
    double yaw_acceleration) const;

  Vector3 compute_attitude_error(
    const RotationMatrix & attitude,
    const RotationMatrix & desired_attitude) const;

  // Cascaded attitude-to-rate law used by the attitude_rate handoff.
  // Returns an FRD body-rate setpoint.
  Vector3 compute_attitude_rate_command(
    const RotationMatrix & attitude,
    const DesiredAttitudeRate & desired) const;

  // Geometric SO(3) controller that outputs PX4-normalized torque directly,
  // rather than a physical body moment.
  GeometricNormalizedOutput compute_geometric_normalized_torque(
    const RotationMatrix & attitude,
    const Vector3 & angular_velocity,
    const DesiredAttitudeDynamics & desired) const;

  // Physical geometric moment [N m]. This is intentionally not connected
  // to PX4 VehicleTorqueSetpoint, whose xyz field is normalized/unitless.
  Vector3 compute_physical_moment_command(
    const RotationMatrix & attitude,
    const Vector3 & angular_velocity,
    const DesiredAttitudeDynamics & desired,
    const InertiaMatrix & inertia,
    const Vector3 & k_r,
    const Vector3 & k_omega) const;

private:
  Parameters parameters_;
};

}  // namespace offboard_controllers::se3
