// SE(3) ROS runtime interface for state, lifecycle, and control publication.

#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <geometry_msgs/msg/vector3.hpp>
#include <limits>
#include <memory>
#include <px4_msgs/msg/control_allocator_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include <offboard_controllers/math/operations.hpp>
#include <offboard_controllers/offboard_lifecycle.hpp>
#include <offboard_controllers/offboard_trigger.hpp>
#include <offboard_controllers/px4/rate_controller.hpp>
#include <offboard_controllers/px4/wrench.hpp>
#include <offboard_controllers/runtime_timing.hpp>
#include <offboard_controllers/se3/physical_wrench_adapter.hpp>
#include <offboard_controllers/se3/runtime/vehicle_state.hpp>
#include <offboard_controllers/se3/vehicle_config.hpp>
#include <offboard_controllers/trajectory/trajectory.hpp>

namespace offboard_controllers {

// ROS coordinator for the six existing SE(3) handoff paths.
// PX4 nav_state owns Offboard transitions. The outer control timer and
// gyro-sample-triggered PX4 rate loop retain separate clocks.
class Se3 final : public rclcpp::Node {
public:
  Se3();

private:
  void initialize_publishers();
  void initialize_subscriptions();

  // PX4 observations and gyro-sample rate loop
  void update_angular_velocity(const px4_msgs::msg::VehicleAngularVelocity& msg);
  void update_control_allocator_status(const px4_msgs::msg::ControlAllocatorStatus& msg);

  // Offboard ownership, trajectory time and completion
  void update_vehicle_status(const px4_msgs::msg::VehicleStatus& msg);
  void handle_command_ack(const px4_msgs::msg::VehicleCommandAck& msg);
  void request_offboard_mode();
  void request_position_mode();
  trajectory::Reference current_trajectory_reference();
  bool state_ready_for_offboard() const;
  double trajectory_entry_yaw() const;
  void update();

  // Handoff-specific control computation
  double projected_collective_thrust(const se3::Vector3& force) const;
  void execute_handoff(const se3::Reference& reference,
                       const se3::TranslationalOutput& output);

  // PX4 setpoint construction and publication
  static uint64_t timestamp_us(const rclcpp::Node& node);
  static float finite_or_zero(double value);
  void publish_offboard_control_mode();
  void publish_trajectory_reference(const trajectory::Reference& reference);
  void publish_acceleration_setpoint(const se3::Vector3& acceleration, double yaw);
  void publish_attitude_setpoint(const se3::RotationMatrix& attitude, const se3::Vector3& force,
                                 double yaw_rate);
  void publish_rate_setpoint(const se3::Vector3& angular_velocity, const se3::Vector3& force);
  void publish_normalized_wrench_setpoint(double normalized_thrust_z,
                                          const se3::Vector3& normalized_torque);
  void publish_px4_rate_output(uint64_t timestamp_sample, const px4_rate::Output& output);

  // Experiment diagnostics
  void publish_vector_diagnostic(
      const rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr& publisher,
      const se3::Vector3& value);
  void publish_desired_attitude_diagnostic(const se3::RotationMatrix& attitude);
  se3::Vector3
  desired_angular_velocity_current_body(const se3::RotationMatrix& desired_attitude,
                                        const se3::Vector3& desired_angular_velocity) const;
  se3::Vector3
  angular_acceleration_feedforward_current_body(const se3::DesiredAttitudeDynamics& desired) const;
  void publish_translation_diagnostics(const se3::TranslationalOutput& output);
  void publish_desired_attitude_rate_diagnostics(const se3::DesiredAttitudeRate& desired);
  void publish_desired_attitude_dynamics_diagnostics(const se3::DesiredAttitudeDynamics& desired);
  void publish_px4_rate_diagnostics(const px4_rate::Output& output);
  void publish_geometric_normalized_diagnostics(const se3::GeometricNormalizedOutput& output);
  void publish_geometric_physical_diagnostics(double requested_collective_thrust_n,
                                              const se3::Vector3& requested_moment_nm,
                                              const physical_wrench_adapter::Output& output);

  // Only the PX4 rate reproduction is stateful; the geometric
  // control laws are direct, stateless mathematical functions.
  std::unique_ptr<px4_rate::Controller> px4_rate_controller_;

  vehicle_config::PhysicalWrenchConfiguration physical_wrench_configuration_{};

  std::unique_ptr<trajectory::ConfiguredTrajectory> configured_trajectory_;

  trajectory::Reference trajectory_origin_{};

  se3::runtime::VehicleState vehicle_state_{};

  bool px4_rate_setpoint_valid_{false};
  bool publish_diagnostics_{false};

  // These flags mirror PX4-confirmed ownership and the pending Position
  // return. A sent mode command does not itself set offboard_active_.
  OffboardTrigger offboard_trigger_{OffboardTrigger::AUTOMATIC};
  bool vehicle_status_received_{false};
  bool vehicle_armed_{false};
  bool offboard_active_{false};
  bool position_mode_active_{false};
  bool return_to_position_requested_{false};
  rclcpp::Time position_return_last_request_at_{0, 0, RCL_ROS_TIME};

  // Physical translation gains [N/m] and [N s/m] are computed
  // once from the configured mass and mass-normalized gains.
  double mass_{std::numeric_limits<double>::quiet_NaN()};
  double kx_{std::numeric_limits<double>::quiet_NaN()};
  double kv_{std::numeric_limits<double>::quiet_NaN()};
  se3::Vector3 attitude_gain_{};
  se3::Vector3 normalized_attitude_gain_{};
  se3::Vector3 normalized_angular_velocity_gain_{};
  se3::Vector3 normalized_angular_acceleration_gain_{};

  double hover_thrust_{std::numeric_limits<double>::quiet_NaN()};

  se3::Vector3 geometric_physical_k_r_{};
  se3::Vector3 geometric_physical_k_omega_{};

  se3::Vector3 px4_rate_setpoint_{};

  double px4_rate_collective_thrust_{std::numeric_limits<double>::quiet_NaN()};

  RuntimeTiming runtime_timing_{};
  int valid_setpoint_count_{0};

  std::string vehicle_;
  std::string trajectory_name_;
  std::string handoff_;
  std::string direct_controller_;

  rclcpp::Time trajectory_start_time_{0, 0, RCL_ROS_TIME};

  // ROS handles stay here because the Se3 node owns the communication
  // endpoints. Runtime mathematical components contain no ROS handles.
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;

  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;

  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_reference_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr commanded_acceleration_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr force_command_pub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr desired_attitude_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr desired_angular_velocity_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr body_rate_setpoint_diagnostic_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angular_acceleration_feedforward_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr px4_rate_rate_error_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr px4_rate_proportional_feedback_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr px4_rate_integral_feedback_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr px4_rate_derivative_feedback_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr px4_rate_feedforward_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr px4_rate_unfiltered_torque_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr px4_rate_integrator_state_pub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr
      vehicle_attitude_setpoint_pub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_pub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr vehicle_thrust_setpoint_pub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr vehicle_torque_setpoint_pub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr
      geometric_normalized_attitude_feedback_pub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr
      geometric_normalized_angular_velocity_feedback_pub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr
      geometric_normalized_angular_acceleration_feedforward_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr
      geometric_physical_requested_moment_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr geometric_physical_applied_moment_pub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr geometric_physical_moment_scale_pub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      geometric_physical_requested_collective_thrust_pub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      geometric_physical_applied_collective_thrust_pub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_sub_;

  rclcpp::Subscription<px4_msgs::msg::ControlAllocatorStatus>::SharedPtr
      control_allocator_status_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr command_ack_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace offboard_controllers
