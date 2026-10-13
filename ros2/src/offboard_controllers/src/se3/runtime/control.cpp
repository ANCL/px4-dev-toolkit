// Select control-cascade stages at the configured PX4 handoff boundary.

#include <offboard_controllers/se3/runtime/node.hpp>

#include <offboard_controllers/se3/control/attitude_control.hpp>
#include <offboard_controllers/se3/control/attitude_reference.hpp>
#include <offboard_controllers/se3/control/force_dynamics.hpp>
#include <offboard_controllers/se3/control/normalized_torque.hpp>
#include <offboard_controllers/se3/control/physical_moment.hpp>

namespace offboard_controllers {

double Se3::projected_collective_thrust(const se3::Vector3& force) const {
  return px4_wrench::normalized_projected_collective_thrust(force, vehicle_state_.state().attitude,
                                                            mass_, hover_thrust_);
}

// The runtime selects which mathematical stages execute for each
// handoff and which PX4 output receives their results.
// Evaluate only the stages owned by the selected handoff. In the
// px4_rate path this callback holds the outer body-rate/collective
// command; torque is computed only by the gyro-driven inner stage.
void Se3::execute_handoff(const se3::Reference& reference,
                          const se3::TranslationalOutput& output) {
  // Handoff contract:
  //   Input:  common SE(3) translation result and current vehicle state.
  //   Logic:  stop the external cascade exactly at the configured PX4
  //           ownership boundary.
  //   Output:
  //     acceleration   -> NED acceleration
  //     attitude       -> attitude + collective thrust
  //     attitude_rate  -> FRD body-rate + collective thrust
  //     px4_rate       -> reproduced PX4 normalized torque + thrust
  //     geometric_*    -> direct normalized or physical-adapted wrench
  publish_translation_diagnostics(output);

  if (handoff_ == "acceleration") {
    publish_acceleration_setpoint(output.acceleration, reference.yaw);

    return;
  }

  if (handoff_ == "attitude") {
    const se3::RotationMatrix desired_attitude =
        se3::attitude_reference::compute_orientation(output.force_vector, reference.yaw);

    publish_desired_attitude_diagnostic(desired_attitude);

    publish_attitude_setpoint(desired_attitude, output.force_vector, reference.yaw_rate);

    return;
  }

  const se3::Vector3 force_derivative =
      se3::force_dynamics::first_derivative(
          vehicle_state_.state(), reference, output.force_vector, mass_, kx_, kv_);

  if (handoff_ == "attitude_rate" || direct_controller_ == "px4_rate") {
    const se3::DesiredAttitudeRate desired = se3::attitude_reference::compute_rate(
        output.force_vector, force_derivative, reference.yaw, reference.yaw_rate);

    publish_desired_attitude_rate_diagnostics(desired);

    const se3::Vector3 body_rate_setpoint =
        se3::attitude_control::compute_rate(
            vehicle_state_.state().attitude, desired, attitude_gain_);

    publish_vector_diagnostic(body_rate_setpoint_diagnostic_pub_, body_rate_setpoint);

    if (handoff_ == "attitude_rate") {
      publish_rate_setpoint(body_rate_setpoint, output.force_vector);

      return;
    }

    // px4_rate is intentionally split across two clocks. This 100 Hz SE3
    // outer stage computes and holds the body-rate/collective setpoint; the
    // reproduced PX4 inner rate loop runs from each
    // VehicleAngularVelocity callback using its timestamp_sample.
    px4_rate_setpoint_ = body_rate_setpoint;

    px4_rate_collective_thrust_ = projected_collective_thrust(output.force_vector);

    px4_rate_setpoint_valid_ =
        math::is_finite(px4_rate_setpoint_) && std::isfinite(px4_rate_collective_thrust_);

    if (!px4_rate_setpoint_valid_) {
      throw std::runtime_error("SE3 px4_rate outer loop produced a "
                               "non-finite held setpoint.");
    }

    return;
  }

  const se3::Vector3 force_second_derivative = se3::force_dynamics::second_derivative(
      vehicle_state_.state(), reference, output.force_vector, force_derivative, mass_, kx_, kv_);

  const se3::DesiredAttitudeDynamics desired = se3::attitude_reference::compute_dynamics(
      output.force_vector, force_derivative, force_second_derivative, reference.yaw,
      reference.yaw_rate, reference.yaw_acceleration);

  publish_desired_attitude_dynamics_diagnostics(desired);

  if (direct_controller_ == "geometric_physical") {
    const se3::Vector3 physical_moment = se3::physical_moment::compute(
        vehicle_state_.state().attitude, vehicle_state_.state().angular_velocity, desired,
        physical_wrench_configuration_.inertia_frd, geometric_physical_k_r_,
        geometric_physical_k_omega_);

    // Lee's collective force f = -A . (R e3), with body Z in NED.
    const double requested_collective_thrust =
        -math::dot(output.force_vector, vehicle_state_.state().attitude.b3);

    const physical_wrench_adapter::Output adapted = physical_wrench_adapter::adapt(
        physical_wrench_configuration_.adapter, requested_collective_thrust, physical_moment);

    publish_geometric_physical_diagnostics(requested_collective_thrust, physical_moment, adapted);

    publish_normalized_wrench_setpoint(adapted.normalized_thrust_z, adapted.normalized_torque);

    return;
  }

  if (direct_controller_ == "geometric_normalized") {
    const se3::GeometricNormalizedOutput direct_output =
        se3::normalized_torque::compute(
            vehicle_state_.state().attitude, vehicle_state_.state().angular_velocity,
            desired, normalized_attitude_gain_, normalized_angular_velocity_gain_,
            normalized_angular_acceleration_gain_);

    publish_geometric_normalized_diagnostics(direct_output);

    publish_normalized_wrench_setpoint(-projected_collective_thrust(output.force_vector),
                                       direct_output.normalized_torque);

    return;
  }

  throw std::logic_error("SE3 reached an unvalidated direct-controller selection.");
}

} // namespace offboard_controllers
