// ROS communication endpoint for the SE(3) Offboard experiment.
// The outer wall timer selects control stages; the PX4 rate handoff
// runs its inner loop only on gyro-sample callbacks.
#include <offboard_controllers/se3/runtime/node.hpp>
#include <offboard_controllers/se3/runtime/topics.hpp>

namespace offboard_controllers {

namespace {
constexpr char kTrajectoryReferenceTopic[] = "/se3/trajectory_reference";

constexpr char kCommandedAccelerationTopic[] = "/se3/diagnostics/commanded_acceleration";

constexpr char kForceCommandTopic[] = "/se3/diagnostics/force_command_ned";

constexpr char kDesiredAttitudeTopic[] = "/se3/diagnostics/desired_attitude";

constexpr char kDesiredAngularVelocityTopic[] = "/se3/diagnostics/desired_angular_velocity";

constexpr char kBodyRateSetpointTopic[] = "/se3/diagnostics/body_rate_setpoint";

constexpr char kAngularAccelerationFeedforwardTopic[] =
    "/se3/diagnostics/angular_acceleration_feedforward";

constexpr char kPx4RateRateErrorTopic[] = "/se3/diagnostics/px4_rate/rate_error";

constexpr char kPx4RateProportionalFeedbackTopic[] =
    "/se3/diagnostics/px4_rate/proportional_feedback";

constexpr char kPx4RateIntegralFeedbackTopic[] = "/se3/diagnostics/px4_rate/integral_feedback";

constexpr char kPx4RateDerivativeFeedbackTopic[] = "/se3/diagnostics/px4_rate/derivative_feedback";

constexpr char kPx4RateFeedforwardTopic[] = "/se3/diagnostics/px4_rate/feedforward";

constexpr char kPx4RateUnfilteredTorqueTopic[] = "/se3/diagnostics/px4_rate/unfiltered_torque";

constexpr char kPx4RateIntegratorStateTopic[] = "/se3/diagnostics/px4_rate/integrator_state";

constexpr char kGeometricNormalizedAttitudeFeedbackTopic[] =
    "/se3/diagnostics/geometric_normalized/torque_attitude_feedback";

constexpr char kGeometricNormalizedAngularVelocityFeedbackTopic[] =
    "/se3/diagnostics/geometric_normalized/torque_angular_velocity_feedback";

constexpr char kGeometricNormalizedAngularAccelerationFeedforwardTopic[] =
    "/se3/diagnostics/geometric_normalized/"
    "torque_angular_acceleration_feedforward";

constexpr char kGeometricPhysicalRequestedMomentTopic[] =
    "/se3/diagnostics/geometric_physical/requested_moment_nm";

constexpr char kGeometricPhysicalAppliedMomentTopic[] =
    "/se3/diagnostics/geometric_physical/applied_moment_nm";

constexpr char kGeometricPhysicalMomentScaleTopic[] =
    "/se3/diagnostics/geometric_physical/moment_scale";

constexpr char kGeometricPhysicalRequestedCollectiveThrustTopic[] =
    "/se3/diagnostics/geometric_physical/requested_collective_thrust_n";

constexpr char kGeometricPhysicalAppliedCollectiveThrustTopic[] =
    "/se3/diagnostics/geometric_physical/applied_collective_thrust_n";
} // namespace

// The PX4 control topics use the centralized catalog. Diagnostic topics
// retain their existing names and are created only when enabled.
void Se3::initialize_publishers() {
  offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      px4_topics::IN_OFFBOARD_CONTROL_MODE, 10);

  trajectory_setpoint_pub_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>(px4_topics::IN_TRAJECTORY_SETPOINT, 10);

  vehicle_attitude_setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
      px4_topics::IN_VEHICLE_ATTITUDE_SETPOINT, 10);

  vehicle_rates_setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
      px4_topics::IN_VEHICLE_RATES_SETPOINT, 10);

  vehicle_thrust_setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
      px4_topics::IN_VEHICLE_THRUST_SETPOINT, 10);

  vehicle_torque_setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
      px4_topics::IN_VEHICLE_TORQUE_SETPOINT, 10);

  trajectory_reference_pub_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>(kTrajectoryReferenceTopic, 10);

  if (publish_diagnostics_) {
    commanded_acceleration_pub_ =
        create_publisher<geometry_msgs::msg::Vector3>(kCommandedAccelerationTopic, 10);

    force_command_pub_ = create_publisher<geometry_msgs::msg::Vector3>(kForceCommandTopic, 10);

    desired_attitude_pub_ =
        create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(kDesiredAttitudeTopic, 10);

    desired_angular_velocity_pub_ =
        create_publisher<geometry_msgs::msg::Vector3>(kDesiredAngularVelocityTopic, 10);

    body_rate_setpoint_diagnostic_pub_ =
        create_publisher<geometry_msgs::msg::Vector3>(kBodyRateSetpointTopic, 10);

    angular_acceleration_feedforward_pub_ =
        create_publisher<geometry_msgs::msg::Vector3>(kAngularAccelerationFeedforwardTopic, 10);

    px4_rate_rate_error_pub_ =
        create_publisher<geometry_msgs::msg::Vector3>(kPx4RateRateErrorTopic, 10);

    px4_rate_proportional_feedback_pub_ =
        create_publisher<geometry_msgs::msg::Vector3>(kPx4RateProportionalFeedbackTopic, 10);

    px4_rate_integral_feedback_pub_ =
        create_publisher<geometry_msgs::msg::Vector3>(kPx4RateIntegralFeedbackTopic, 10);

    px4_rate_derivative_feedback_pub_ =
        create_publisher<geometry_msgs::msg::Vector3>(kPx4RateDerivativeFeedbackTopic, 10);

    px4_rate_feedforward_pub_ =
        create_publisher<geometry_msgs::msg::Vector3>(kPx4RateFeedforwardTopic, 10);

    px4_rate_unfiltered_torque_pub_ =
        create_publisher<geometry_msgs::msg::Vector3>(kPx4RateUnfilteredTorqueTopic, 10);

    px4_rate_integrator_state_pub_ =
        create_publisher<geometry_msgs::msg::Vector3>(kPx4RateIntegratorStateTopic, 10);

    geometric_normalized_attitude_feedback_pub_ =
        create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
            kGeometricNormalizedAttitudeFeedbackTopic, 10);

    geometric_normalized_angular_velocity_feedback_pub_ =
        create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
            kGeometricNormalizedAngularVelocityFeedbackTopic, 10);

    geometric_normalized_angular_acceleration_feedforward_pub_ =
        create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
            kGeometricNormalizedAngularAccelerationFeedforwardTopic, 10);

    geometric_physical_requested_moment_pub_ =
        create_publisher<geometry_msgs::msg::Vector3>(kGeometricPhysicalRequestedMomentTopic, 10);

    geometric_physical_applied_moment_pub_ =
        create_publisher<geometry_msgs::msg::Vector3>(kGeometricPhysicalAppliedMomentTopic, 10);

    geometric_physical_moment_scale_pub_ =
        create_publisher<std_msgs::msg::Float64>(kGeometricPhysicalMomentScaleTopic, 10);

    geometric_physical_requested_collective_thrust_pub_ = create_publisher<std_msgs::msg::Float64>(
        kGeometricPhysicalRequestedCollectiveThrustTopic, 10);

    geometric_physical_applied_collective_thrust_pub_ = create_publisher<std_msgs::msg::Float64>(
        kGeometricPhysicalAppliedCollectiveThrustTopic, 10);
  }

  vehicle_command_pub_ =
      create_publisher<px4_msgs::msg::VehicleCommand>(px4_topics::IN_VEHICLE_COMMAND, 10);
}

// Preserve PX4 sensor QoS and the distinct wall-timer cadence. The gyro
// callback runs independently on each received angular-velocity sample.
void Se3::initialize_subscriptions() {
  local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      px4_topics::OUT_VEHICLE_LOCAL_POSITION, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        vehicle_state_.update_position(*msg);
      });

  vehicle_attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      px4_topics::OUT_VEHICLE_ATTITUDE, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        vehicle_state_.update_attitude(*msg);
      });

  angular_velocity_sub_ = create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
      px4_topics::OUT_VEHICLE_ANGULAR_VELOCITY, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg) {
        update_angular_velocity(*msg);
      });

  control_allocator_status_sub_ = create_subscription<px4_msgs::msg::ControlAllocatorStatus>(
      px4_topics::OUT_CONTROL_ALLOCATOR_STATUS, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::ControlAllocatorStatus::SharedPtr msg) {
        update_control_allocator_status(*msg);
      });

  vehicle_control_mode_sub_ = create_subscription<px4_msgs::msg::VehicleControlMode>(
      px4_topics::OUT_VEHICLE_CONTROL_MODE, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
        vehicle_state_.update_control_mode(*msg);
      });

  vehicle_land_detected_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      px4_topics::OUT_VEHICLE_LAND_DETECTED, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
        vehicle_state_.update_land_detected(*msg);
      });

  vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      px4_topics::OUT_VEHICLE_STATUS_V1, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) { update_vehicle_status(*msg); });

  command_ack_sub_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(
      px4_topics::OUT_VEHICLE_COMMAND_ACK, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleCommandAck::SharedPtr msg) { handle_command_ack(*msg); });

  timer_ = create_wall_timer(runtime_timing_.control_period, [this]() { update(); });
}

// PX4-rate reproduction runs at gyro-sample cadence, not at the outer
// controller timer cadence. Observe even inactive timestamps so dt
// and integrator semantics remain consistent with the pinned PX4 code.
void Se3::update_angular_velocity(const px4_msgs::msg::VehicleAngularVelocity& msg) {
  const se3::runtime::AngularSample sample = vehicle_state_.update_angular_velocity(msg);
  const se3::Vector3& angular_velocity = sample.angular_velocity;
  const se3::Vector3& angular_acceleration = sample.angular_acceleration;

  if (!px4_rate_controller_) {
    return;
  }

  const bool rate_stage_active =
      handoff_ == "thrust_and_torque" && direct_controller_ == "px4_rate" && offboard_active_ &&
      px4_rate_setpoint_valid_ && vehicle_state_.ready(handoff_, direct_controller_);

  if (!rate_stage_active) {
    // PX4 advances _last_run on every VehicleAngularVelocity callback,
    // including callbacks for which rate control is inactive.
    px4_rate_controller_->observe_timestamp_sample(msg.timestamp_sample);

    return;
  }

  // MulticopterRateControl resets the integrator before the rate update
  // whenever the vehicle is disarmed or is not a rotary wing.
  if (!vehicle_state_.armed() || !vehicle_state_.rotary_wing()) {
    px4_rate_controller_->reset_integral();
  }

  const px4_rate::Output output =
      px4_rate_controller_->update(msg.timestamp_sample, angular_velocity, px4_rate_setpoint_,
                                   angular_acceleration, vehicle_state_.landed_or_maybe_landed());

  publish_px4_rate_diagnostics(output);

  publish_px4_rate_output(msg.timestamp_sample, output);
}

// Convert allocator residual torque into per-axis anti-windup
// saturation flags. Values within float epsilon are numerical noise.
void Se3::update_control_allocator_status(const px4_msgs::msg::ControlAllocatorStatus& msg) {
  if (!px4_rate_controller_) {
    return;
  }

  std::array<bool, 3> saturation_positive{};
  std::array<bool, 3> saturation_negative{};

  if (!msg.torque_setpoint_achieved) {
    constexpr float epsilon = std::numeric_limits<float>::epsilon();

    for (std::size_t axis = 0; axis < 3; ++axis) {
      if (msg.unallocated_torque[axis] > epsilon) {
        saturation_positive[axis] = true;

      } else if (msg.unallocated_torque[axis] < -epsilon) {
        saturation_negative[axis] = true;
      }
    }
  }

  // The latest allocator state remains active until a newer allocator
  // status arrives, matching the PX4 uORB subscription behavior.
  px4_rate_controller_->set_saturation_status(saturation_positive, saturation_negative);
}

} // namespace offboard_controllers


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<offboard_controllers::Se3>());

  rclcpp::shutdown();

  return 0;
}
