#pragma once

#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::se3
{

// Controller mathematics are independent of ROS 2 and PX4 transport.
class Controller
{
public:
  explicit Controller(const Parameters & parameters);

  // Translational outer-loop contract:
  //   Inputs: current state and trajectory reference in local NED coordinates.
  //   Logic:  apply position/velocity feedback and acceleration feed-forward,
  //           then include gravity when constructing the desired force.
  //   Output: kinematic NED acceleration [m/s^2] and desired NED force [N].
  TranslationalOutput compute_translation(
    const State & state,
    const Reference & reference) const;

  // Desired-force derivative:
  //   Inputs: NED state/reference and the current desired force A [N].
  //   Logic:  differentiate A analytically using nominal quadrotor dynamics;
  //           estimator acceleration and numerical differentiation are avoided.
  //   Output: A_dot in NED [N/s].
  Vector3 compute_force_derivative(
    const State & state,
    const Reference & reference,
    const Vector3 & force) const;

  // Desired-force second derivative:
  //   Inputs: attitude/rates, trajectory derivatives, A, and A_dot.
  //   Logic:  use rigid-body thrust-axis kinematics and analytic
  //           differentiation rather than finite-difference jerk.
  //   Output: A_ddot in NED [N/s^2].
  Vector3 compute_force_second_derivative(
    const State & state,
    const Reference & reference,
    const Vector3 & force,
    const Vector3 & force_derivative) const;

  // Desired-attitude construction:
  //   Inputs: desired NED force [N] and yaw [rad].
  //   Logic:  align desired body -Z with force and resolve horizontal heading.
  //   Output: desired FRD-body -> NED rotation matrix.
  RotationMatrix compute_desired_attitude(
    const Vector3 & force,
    double yaw) const;

  // Desired-attitude-rate construction:
  //   Inputs: A, A_dot, yaw, and yaw rate.
  //   Logic:  analytically differentiate the desired body basis.
  //   Output: desired FRD->NED attitude and desired body angular velocity.
  DesiredAttitudeRate compute_desired_attitude_rate(
    const Vector3 & force,
    const Vector3 & force_derivative,
    double yaw,
    double yaw_rate) const;

  // Desired-attitude-dynamics construction:
  //   Inputs: A through A_ddot and yaw through yaw acceleration.
  //   Logic:  analytically differentiate the desired body basis twice.
  //   Output: desired attitude, angular velocity, and angular acceleration.
  DesiredAttitudeDynamics compute_desired_attitude_dynamics(
    const Vector3 & force,
    const Vector3 & force_derivative,
    const Vector3 & force_second_derivative,
    double yaw,
    double yaw_rate,
    double yaw_acceleration) const;

  // Geometric attitude-error contract:
  //   Inputs: current and desired FRD->NED rotation matrices.
  //   Logic:  evaluate Lee et al. Eq. (10) on SO(3).
  //   Output: dimensionless attitude-error vector e_R in the current FRD
  //           body frame.
  Vector3 compute_attitude_error(
    const RotationMatrix & attitude,
    const RotationMatrix & desired_attitude) const;

  // Attitude-to-rate handoff:
  //   Inputs: current and desired FRD->NED attitudes plus desired rate.
  //   Logic:  apply geometric attitude feedback with rotating-frame
  //           angular-rate feed-forward.
  //   Output: FRD body-rate setpoint [rad/s] for the PX4 rate controller.
  Vector3 compute_attitude_rate_command(
    const RotationMatrix & attitude,
    const DesiredAttitudeRate & desired) const;

  // Direct normalized geometric controller:
  //   Inputs: current attitude/rate and desired attitude dynamics.
  //   Logic:  combine SO(3) attitude/rate errors with angular-acceleration
  //           feed-forward directly in normalized controller coordinates.
  //   Output: dimensionless torque for PX4 VehicleTorqueSetpoint.
  GeometricNormalizedOutput compute_geometric_normalized_torque(
    const RotationMatrix & attitude,
    const Vector3 & angular_velocity,
    const DesiredAttitudeDynamics & desired) const;

  // Physical geometric moment controller:
  //   Inputs: current/desired rotational state, FRD inertia [kg m^2],
  //           and physical attitude/rate gains.
  //   Logic:  evaluate the geometric rigid-body moment law including
  //           gyroscopic and desired-angular-acceleration terms.
  //   Output: physical FRD body moment [N m], not PX4-normalized torque.
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
