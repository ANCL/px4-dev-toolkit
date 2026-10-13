// Issue Position, arm, and land commands to PX4.

#include "node.hpp"

void PositionTakeoffHover::publish_position_mode_command()
{
  px4_msgs::msg::VehicleCommand msg{};

  msg.param1 = kCustomModeEnabled;
  msg.param2 = kPositionMainMode;
  msg.param3 = kPositionSubMode;

  msg.command =
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;

  fill_command_metadata(msg);
  vehicle_command_pub_->publish(msg);

  RCLCPP_INFO(get_logger(), "Position-mode command sent.");
}


void PositionTakeoffHover::publish_arm_command()
{
  px4_msgs::msg::VehicleCommand msg{};

  msg.param1 =
    static_cast<float>(
    px4_msgs::msg::VehicleCommand::ARMING_ACTION_ARM);

  msg.command =
    px4_msgs::msg::VehicleCommand::
    VEHICLE_CMD_COMPONENT_ARM_DISARM;

  fill_command_metadata(msg);
  vehicle_command_pub_->publish(msg);

  RCLCPP_INFO(get_logger(), "Arm command sent.");
}


void PositionTakeoffHover::publish_land_command()
{
  px4_msgs::msg::VehicleCommand msg{};

  // Leave optional MAV_CMD_NAV_LAND parameters unspecified. PX4 therefore
  // performs its native landing behavior rather than receiving a separate
  // externally supplied landing location or heading.
  msg.param1 = NAN;
  msg.param2 = NAN;
  msg.param3 = NAN;
  msg.param4 = NAN;
  msg.param5 = NAN;
  msg.param6 = NAN;
  msg.param7 = NAN;

  msg.command =
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;

  fill_command_metadata(msg);
  vehicle_command_pub_->publish(msg);

  RCLCPP_INFO(get_logger(), "Land command sent.");
}


void PositionTakeoffHover::fill_command_metadata(
  px4_msgs::msg::VehicleCommand & msg)
{
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = now_us();
}
