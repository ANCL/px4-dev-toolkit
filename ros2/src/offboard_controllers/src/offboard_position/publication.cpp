// Offboard heartbeat, trajectory setpoint, and PX4 command publication.

#include "node.hpp"

void OffboardPosition::publish_offboard_control_mode(
  std::uint64_t timestamp)
{
  px4_msgs::msg::OffboardControlMode msg{};

  msg.timestamp = timestamp;
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.thrust_and_torque = false;
  msg.direct_actuator = false;

  offboard_control_mode_pub_->publish(msg);
}


void OffboardPosition::publish_trajectory_setpoint(
  std::uint64_t timestamp,
  const trajectory::Reference & reference)
{
  px4_msgs::msg::TrajectorySetpoint msg{};

  msg.timestamp = timestamp;

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

  msg.yaw =
    static_cast<float>(reference.yaw);

  msg.yawspeed =
    static_cast<float>(reference.yaw_rate);

  trajectory_setpoint_pub_->publish(msg);
}


void OffboardPosition::publish_offboard_stream()
{
  // Publish the Offboard heartbeat and trajectory setpoint as one logical
  // sample with the same timestamp.
  const std::uint64_t timestamp = now_us();

  const trajectory::Reference reference =
    current_trajectory_reference();

  publish_offboard_control_mode(timestamp);
  publish_trajectory_setpoint(
    timestamp,
    reference);
}


void OffboardPosition::publish_offboard_mode_command()
{
  px4_msgs::msg::VehicleCommand msg{};

  msg.param1 = kCustomModeEnabled;
  msg.param2 = kOffboardMainMode;
  msg.param3 = kOffboardSubMode;

  msg.command =
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;

  fill_command_metadata(msg);
  vehicle_command_pub_->publish(msg);

  RCLCPP_INFO(
    get_logger(),
    "Offboard mode command sent after %u prestream samples.",
    static_cast<unsigned>(offboard_samples_published_));
}


void OffboardPosition::publish_position_mode_command()
{
  px4_msgs::msg::VehicleCommand msg{};

  msg.param1 = kCustomModeEnabled;
  msg.param2 = kPositionMainMode;
  msg.param3 = 0.0F;

  msg.command =
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;

  fill_command_metadata(msg);
  vehicle_command_pub_->publish(msg);

  RCLCPP_INFO(
    get_logger(),
    "PX4 Position mode command sent.");
}


void OffboardPosition::fill_command_metadata(
  px4_msgs::msg::VehicleCommand & msg)
{
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = now_us();
}
