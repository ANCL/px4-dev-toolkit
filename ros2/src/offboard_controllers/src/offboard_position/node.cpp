// ROS subscriptions, publishers, and timer setup for Offboard position.

#include "node.hpp"

OffboardPosition::OffboardPosition()
: Node("offboard_position")
{
  trajectory_name_ =
    declare_parameter<std::string>(
    "trajectory",
    "hover");

  offboard_trigger_ = offboard_controllers::parse_offboard_trigger(
    declare_parameter<std::string>("offboard_trigger", "automatic"));

  const std::string trajectory_config =
    declare_parameter<std::string>(
    "trajectory_config",
    "");

  if (trajectory_config.empty()) {
    throw std::invalid_argument(
            "Offboard position trajectory_config must not be empty.");
  }

  configured_trajectory_ =
    std::make_unique<trajectory::ConfiguredTrajectory>(
    trajectory::load_trajectory(
      trajectory_config,
      trajectory_name_));

  // PX4 uXRCE-DDS output topics are live vehicle-state streams. Use
  // SensorDataQoS so consumers prefer fresh state over retransmission of
  // stale samples.
  const auto sensor_qos = rclcpp::SensorDataQoS();

  vehicle_status_sub_ =
    create_subscription<px4_msgs::msg::VehicleStatus>(
    px4_topics::OUT_VEHICLE_STATUS_V1,
    sensor_qos,
    [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
      handle_vehicle_status(*msg);
    });

  vehicle_local_position_sub_ =
    create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    px4_topics::OUT_VEHICLE_LOCAL_POSITION,
    sensor_qos,
    [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
      handle_local_position(*msg);
    });

  vehicle_command_ack_sub_ =
    create_subscription<px4_msgs::msg::VehicleCommandAck>(
    px4_topics::OUT_VEHICLE_COMMAND_ACK,
    sensor_qos,
    [this](const px4_msgs::msg::VehicleCommandAck::SharedPtr msg) {
      handle_command_ack(*msg);
    });

  offboard_control_mode_pub_ =
    create_publisher<px4_msgs::msg::OffboardControlMode>(
    px4_topics::IN_OFFBOARD_CONTROL_MODE,
    10);

  trajectory_setpoint_pub_ =
    create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    px4_topics::IN_TRAJECTORY_SETPOINT,
    10);

  vehicle_command_pub_ =
    create_publisher<px4_msgs::msg::VehicleCommand>(
    px4_topics::IN_VEHICLE_COMMAND,
    10);

  phase_started_at_ = SteadyClock::now();

  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    [this]() {
      run();
    });

  RCLCPP_INFO(
    get_logger(),
    "Configured Offboard trajectory: %s (trigger: %s)",
    trajectory_name_.c_str(),
    offboard_trigger_ == offboard_controllers::OffboardTrigger::AUTOMATIC
      ? "automatic" : "external");

  RCLCPP_INFO(
    get_logger(),
    "Waiting for PX4 vehicle status and valid local position.");
}


int OffboardPosition::exit_code() const
{
  return exit_code_;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<OffboardPosition>();

  rclcpp::spin(node);

  const int exit_code = node->exit_code();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return exit_code;
}
