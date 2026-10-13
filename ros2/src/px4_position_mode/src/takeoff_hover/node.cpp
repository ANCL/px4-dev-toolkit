// ROS endpoints and timers for the native Position-mode reference flight.

#include "node.hpp"

PositionTakeoffHover::PositionTakeoffHover()
: Node("position_takeoff_hover")
{
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

  vehicle_land_detected_sub_ =
    create_subscription<px4_msgs::msg::VehicleLandDetected>(
    px4_topics::OUT_VEHICLE_LAND_DETECTED,
    sensor_qos,
    [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
      landed_ = msg->landed;
      land_detected_received_ = true;
    });

  vehicle_command_ack_sub_ =
    create_subscription<px4_msgs::msg::VehicleCommandAck>(
    px4_topics::OUT_VEHICLE_COMMAND_ACK,
    sensor_qos,
    [this](const px4_msgs::msg::VehicleCommandAck::SharedPtr msg) {
      handle_command_ack(*msg);
    });

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
    "Waiting for PX4 vehicle status and valid local position.");
}


int PositionTakeoffHover::exit_code() const
{
  return exit_code_;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<PositionTakeoffHover>();

  rclcpp::spin(node);

  const int exit_code = node->exit_code();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return exit_code;
}
