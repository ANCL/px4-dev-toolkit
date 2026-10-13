// Send Position-mode and arming requests during staging.

#include "node.hpp"

void OffboardTakeoffHandoff::publish_position_mode_command()
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


void OffboardTakeoffHandoff::publish_arm_command()
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


void OffboardTakeoffHandoff::fill_command_metadata(
  px4_msgs::msg::VehicleCommand & msg)
{
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = now_us();
}
