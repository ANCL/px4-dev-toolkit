// Publish SE(3) PX4 setpoints and optional control diagnostics.

#include <offboard_controllers/se3/runtime/node.hpp>
#include <offboard_controllers/se3/runtime/topics.hpp>

namespace offboard_controllers {

uint64_t Se3::timestamp_us(const rclcpp::Node& node) {
  return static_cast<uint64_t>(node.get_clock()->now().nanoseconds() / 1000);
}

float Se3::finite_or_zero(double value) {
  return std::isfinite(value) ? static_cast<float>(value) : 0.0F;
}

// The selected PX4 handoff has exactly one active Offboard control
// ownership flag. The lifecycle publishes this heartbeat only after
// PX4 reports armed state and the required valid vehicle state.
void Se3::publish_offboard_control_mode() {
  px4_msgs::msg::OffboardControlMode msg{};

  msg.timestamp = timestamp_us(*this);

  msg.position = false;
  msg.velocity = false;
  msg.acceleration = handoff_ == "acceleration";
  msg.attitude = handoff_ == "attitude";
  msg.body_rate = handoff_ == "attitude_rate";
  msg.thrust_and_torque = handoff_ == "thrust_and_torque";
  msg.direct_actuator = false;

  offboard_control_mode_pub_->publish(msg);
}

void Se3::publish_trajectory_reference(const trajectory::Reference& reference) {
  px4_msgs::msg::TrajectorySetpoint msg{};

  msg.timestamp = timestamp_us(*this);

  msg.position = {
      static_cast<float>(reference.position[0]),
      static_cast<float>(reference.position[1]),
      static_cast<float>(reference.position[2]),
  };

  msg.velocity = {
      static_cast<float>(reference.velocity[0]),
      static_cast<float>(reference.velocity[1]),
      static_cast<float>(reference.velocity[2]),
  };

  msg.acceleration = {
      static_cast<float>(reference.acceleration[0]),
      static_cast<float>(reference.acceleration[1]),
      static_cast<float>(reference.acceleration[2]),
  };

  msg.jerk = {
      static_cast<float>(reference.jerk[0]),
      static_cast<float>(reference.jerk[1]),
      static_cast<float>(reference.jerk[2]),
  };

  msg.yaw = static_cast<float>(reference.yaw);

  msg.yawspeed = static_cast<float>(reference.yaw_rate);

  trajectory_reference_pub_->publish(msg);
}

// Acceleration handoff: NaNs mark unused position, velocity and jerk
// fields. Replacing these with zeros changes PX4 controller ownership.
void Se3::publish_acceleration_setpoint(const se3::Vector3& acceleration, double yaw) {
  const float nan = std::numeric_limits<float>::quiet_NaN();

  px4_msgs::msg::TrajectorySetpoint msg{};

  msg.timestamp = timestamp_us(*this);

  msg.position = {
      nan,
      nan,
      nan,
  };

  msg.velocity = {
      nan,
      nan,
      nan,
  };

  msg.acceleration = {
      static_cast<float>(acceleration.x),
      static_cast<float>(acceleration.y),
      static_cast<float>(acceleration.z),
  };

  msg.jerk = {
      nan,
      nan,
      nan,
  };

  msg.yaw = static_cast<float>(yaw);

  msg.yawspeed = nan;

  trajectory_setpoint_pub_->publish(msg);
}

// q_d maps FRD body axes to NED. PX4 thrust_body uses body-Z
// negative for upward collective thrust in the FRD convention.
void Se3::publish_attitude_setpoint(const se3::RotationMatrix& attitude, const se3::Vector3& force,
                                    double yaw_rate) {
  const auto quaternion = px4_wrench::quaternion_from_rotation(attitude);

  px4_msgs::msg::VehicleAttitudeSetpoint msg{};

  msg.timestamp = timestamp_us(*this);

  msg.q_d = {
      static_cast<float>(quaternion[0]),
      static_cast<float>(quaternion[1]),
      static_cast<float>(quaternion[2]),
      static_cast<float>(quaternion[3]),
  };

  msg.yaw_sp_move_rate = static_cast<float>(yaw_rate);

  msg.thrust_body = {
      0.0F,
      0.0F,
      -static_cast<float>(projected_collective_thrust(force)),
  };

  vehicle_attitude_setpoint_pub_->publish(msg);
}

// VehicleRatesSetpoint is expressed in current FRD body axes;
// the upstream geometric cascade has already rotated its reference.
void Se3::publish_rate_setpoint(const se3::Vector3& angular_velocity, const se3::Vector3& force) {
  px4_msgs::msg::VehicleRatesSetpoint msg{};

  msg.timestamp = timestamp_us(*this);

  msg.roll = static_cast<float>(angular_velocity.x);
  msg.pitch = static_cast<float>(angular_velocity.y);
  msg.yaw = static_cast<float>(angular_velocity.z);

  msg.thrust_body = {
      0.0F,
      0.0F,
      -static_cast<float>(projected_collective_thrust(force)),
  };

  msg.reset_integral = false;

  vehicle_rates_setpoint_pub_->publish(msg);
}

void Se3::publish_normalized_wrench_setpoint(double normalized_thrust_z,
                                             const se3::Vector3& normalized_torque) {
  if (!std::isfinite(normalized_thrust_z) || !math::is_finite(normalized_torque)) {
    throw std::invalid_argument("SE3 normalized wrench setpoint must be finite.");
  }

  const uint64_t timestamp = timestamp_us(*this);

  px4_msgs::msg::VehicleThrustSetpoint thrust{};
  thrust.timestamp = timestamp;
  thrust.xyz = {
      0.0F,
      0.0F,
      static_cast<float>(normalized_thrust_z),
  };

  px4_msgs::msg::VehicleTorqueSetpoint torque{};
  torque.timestamp = timestamp;
  torque.xyz = {
      static_cast<float>(normalized_torque.x),
      static_cast<float>(normalized_torque.y),
      static_cast<float>(normalized_torque.z),
  };

  vehicle_thrust_setpoint_pub_->publish(thrust);
  vehicle_torque_setpoint_pub_->publish(torque);
}

// The inner rate loop passes the incoming gyro timestamp_sample to
// both wrench messages. Unlike the outer-loop handoffs, these messages
// must retain the sample correlation expected by the PX4 pipeline.
void Se3::publish_px4_rate_output(uint64_t timestamp_sample, const px4_rate::Output& output) {
  const uint64_t timestamp = timestamp_us(*this);

  px4_msgs::msg::VehicleThrustSetpoint thrust{};
  thrust.timestamp = timestamp;
  thrust.timestamp_sample = timestamp_sample;
  thrust.xyz = {
      0.0F,
      0.0F,
      -static_cast<float>(px4_rate_collective_thrust_),
  };

  px4_msgs::msg::VehicleTorqueSetpoint torque{};
  torque.timestamp = timestamp;
  torque.timestamp_sample = timestamp_sample;
  torque.xyz = {
      finite_or_zero(output.normalized_torque.x),
      finite_or_zero(output.normalized_torque.y),
      finite_or_zero(output.normalized_torque.z),
  };

  // px4_rate_battery_scale_enabled mirrors the relevant PX4 controller
  // configuration. The verified baseline has MC_BAT_SCALE_EN=0, so the
  // reproduced rate stage performs no battery scaling or output constraining here.
  vehicle_thrust_setpoint_pub_->publish(thrust);

  vehicle_torque_setpoint_pub_->publish(torque);
}

// Diagnostics are observational: disabling them must not alter the
// controller outputs, selected handoff, or publication cadence.
void Se3::publish_vector_diagnostic(
    const rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr& publisher,
    const se3::Vector3& value) {
  if (!publish_diagnostics_) {
    return;
  }

  geometry_msgs::msg::Vector3 msg{};

  msg.x = value.x;
  msg.y = value.y;
  msg.z = value.z;

  publisher->publish(msg);
}

void Se3::publish_desired_attitude_diagnostic(const se3::RotationMatrix& attitude) {
  if (!publish_diagnostics_) {
    return;
  }

  const auto quaternion = px4_wrench::quaternion_from_rotation(attitude);

  px4_msgs::msg::VehicleAttitudeSetpoint msg{};

  msg.timestamp = timestamp_us(*this);

  msg.q_d = {
      static_cast<float>(quaternion[0]),
      static_cast<float>(quaternion[1]),
      static_cast<float>(quaternion[2]),
      static_cast<float>(quaternion[3]),
  };

  desired_attitude_pub_->publish(msg);
}

// Desired angular velocity is expressed in the desired FRD frame;
// rotate through NED to report it in the current vehicle FRD frame.
se3::Vector3
Se3::desired_angular_velocity_current_body(const se3::RotationMatrix& desired_attitude,
                                           const se3::Vector3& desired_angular_velocity) const {
  return math::rotate_inertial_to_body(
      vehicle_state_.state().attitude,
      math::rotate_body_to_inertial(desired_attitude, desired_angular_velocity));
}

se3::Vector3 Se3::angular_acceleration_feedforward_current_body(
    const se3::DesiredAttitudeDynamics& desired) const {
  const se3::Vector3 desired_angular_velocity =
      desired_angular_velocity_current_body(desired.attitude, desired.angular_velocity);

  return -math::cross(vehicle_state_.state().angular_velocity, desired_angular_velocity) +
         math::rotate_inertial_to_body(
             vehicle_state_.state().attitude,
             math::rotate_body_to_inertial(desired.attitude, desired.angular_acceleration));
}

void Se3::publish_translation_diagnostics(const se3::TranslationalOutput& output) {
  publish_vector_diagnostic(commanded_acceleration_pub_, output.acceleration);

  publish_vector_diagnostic(force_command_pub_, output.force_vector);
}

void Se3::publish_desired_attitude_rate_diagnostics(const se3::DesiredAttitudeRate& desired) {
  publish_desired_attitude_diagnostic(desired.attitude);

  publish_vector_diagnostic(
      desired_angular_velocity_pub_,
      desired_angular_velocity_current_body(desired.attitude, desired.angular_velocity));
}

void Se3::publish_desired_attitude_dynamics_diagnostics(
    const se3::DesiredAttitudeDynamics& desired) {
  publish_desired_attitude_diagnostic(desired.attitude);

  publish_vector_diagnostic(
      desired_angular_velocity_pub_,
      desired_angular_velocity_current_body(desired.attitude, desired.angular_velocity));

  publish_vector_diagnostic(angular_acceleration_feedforward_pub_,
                            angular_acceleration_feedforward_current_body(desired));
}

void Se3::publish_px4_rate_diagnostics(const px4_rate::Output& output) {
  publish_vector_diagnostic(px4_rate_rate_error_pub_, output.rate_error);

  publish_vector_diagnostic(px4_rate_proportional_feedback_pub_, output.proportional_feedback);

  publish_vector_diagnostic(px4_rate_integral_feedback_pub_, output.integral_feedback);

  publish_vector_diagnostic(px4_rate_derivative_feedback_pub_, output.derivative_feedback);

  publish_vector_diagnostic(px4_rate_feedforward_pub_, output.feedforward);

  publish_vector_diagnostic(px4_rate_unfiltered_torque_pub_, output.unfiltered_torque);

  publish_vector_diagnostic(px4_rate_integrator_state_pub_, output.integrator_state);
}

void Se3::publish_geometric_normalized_diagnostics(const se3::GeometricNormalizedOutput& output) {
  if (!publish_diagnostics_) {
    return;
  }

  const uint64_t timestamp = timestamp_us(*this);

  px4_msgs::msg::VehicleTorqueSetpoint attitude{};
  attitude.timestamp = timestamp;
  attitude.xyz = {
      static_cast<float>(output.attitude_feedback.x),
      static_cast<float>(output.attitude_feedback.y),
      static_cast<float>(output.attitude_feedback.z),
  };

  px4_msgs::msg::VehicleTorqueSetpoint angular_velocity{};
  angular_velocity.timestamp = timestamp;
  angular_velocity.xyz = {
      static_cast<float>(output.angular_velocity_feedback.x),
      static_cast<float>(output.angular_velocity_feedback.y),
      static_cast<float>(output.angular_velocity_feedback.z),
  };

  px4_msgs::msg::VehicleTorqueSetpoint angular_acceleration{};
  angular_acceleration.timestamp = timestamp;
  angular_acceleration.xyz = {
      static_cast<float>(output.angular_acceleration_feedforward.x),
      static_cast<float>(output.angular_acceleration_feedforward.y),
      static_cast<float>(output.angular_acceleration_feedforward.z),
  };

  geometric_normalized_attitude_feedback_pub_->publish(attitude);

  geometric_normalized_angular_velocity_feedback_pub_->publish(angular_velocity);

  geometric_normalized_angular_acceleration_feedforward_pub_->publish(angular_acceleration);
}

void Se3::publish_geometric_physical_diagnostics(double requested_collective_thrust_n,
                                                 const se3::Vector3& requested_moment_nm,
                                                 const physical_wrench_adapter::Output& output) {
  if (!publish_diagnostics_) {
    return;
  }

  publish_vector_diagnostic(geometric_physical_requested_moment_pub_, requested_moment_nm);

  const se3::Vector3 applied_moment_nm{
      output.moment_scale * requested_moment_nm.x,
      output.moment_scale * requested_moment_nm.y,
      output.moment_scale * requested_moment_nm.z,
  };

  publish_vector_diagnostic(geometric_physical_applied_moment_pub_, applied_moment_nm);

  std_msgs::msg::Float64 moment_scale{};
  moment_scale.data = output.moment_scale;

  std_msgs::msg::Float64 requested_collective{};
  requested_collective.data = requested_collective_thrust_n;

  std_msgs::msg::Float64 applied_collective{};
  applied_collective.data = output.applied_collective_thrust_n;

  geometric_physical_moment_scale_pub_->publish(moment_scale);

  geometric_physical_requested_collective_thrust_pub_->publish(requested_collective);

  geometric_physical_applied_collective_thrust_pub_->publish(applied_collective);
}

} // namespace offboard_controllers
