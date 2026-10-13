// PX4 status, local-position, and command-ack observation for Offboard position.

#include "node.hpp"

void OffboardPosition::handle_vehicle_status(
  const px4_msgs::msg::VehicleStatus & msg)
{
  if (!status_received_ ||
    msg.arming_state != arming_state_ ||
    msg.nav_state != nav_state_)
  {
    RCLCPP_INFO(
      get_logger(),
      "PX4 status: arming_state=%u, nav_state=%u",
      static_cast<unsigned>(msg.arming_state),
      static_cast<unsigned>(msg.nav_state));
  }

  arming_state_ = msg.arming_state;
  nav_state_ = msg.nav_state;
  pre_flight_checks_pass_ = msg.pre_flight_checks_pass;
  status_received_ = true;
}


void OffboardPosition::handle_local_position(
  const px4_msgs::msg::VehicleLocalPosition & msg)
{
  const bool valid =
    msg.xy_valid &&
    msg.z_valid &&
    std::isfinite(msg.x) &&
    std::isfinite(msg.y) &&
    std::isfinite(msg.z);

  if (valid) {
    local_position_ = {
      static_cast<double>(msg.x),
      static_cast<double>(msg.y),
      static_cast<double>(msg.z),
    };
  }

  if (valid && !local_position_valid_) {
    RCLCPP_INFO(
      get_logger(),
      "Local position valid: x=%.2f y=%.2f z=%.2f",
      static_cast<double>(msg.x),
      static_cast<double>(msg.y),
      static_cast<double>(msg.z));
  }

  if (!valid && local_position_valid_) {
    RCLCPP_WARN(
      get_logger(),
      "PX4 local position is no longer valid.");
  }

  local_position_valid_ = valid;
}


void OffboardPosition::handle_command_ack(
  const px4_msgs::msg::VehicleCommandAck & msg)
{
  using VehicleCommand = px4_msgs::msg::VehicleCommand;
  using VehicleCommandAck = px4_msgs::msg::VehicleCommandAck;

  if (
    msg.command != VehicleCommand::VEHICLE_CMD_DO_SET_MODE &&
    msg.command != VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM)
  {
    return;
  }

  const bool accepted =
    msg.result ==
    VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;

  if (accepted) {
    RCLCPP_INFO(
      get_logger(),
      "PX4 accepted command %u.",
      static_cast<unsigned>(msg.command));

  } else {
    RCLCPP_ERROR(
      get_logger(),
      "PX4 rejected command %u: result=%u",
      static_cast<unsigned>(msg.command),
      static_cast<unsigned>(msg.result));
  }
}
