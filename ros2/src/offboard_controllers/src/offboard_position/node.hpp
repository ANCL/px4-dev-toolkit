// State and ROS endpoints owned by the independent Offboard-position node.

#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <offboard_controllers/offboard_trigger.hpp>
#include <offboard_controllers/trajectory/trajectory.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>


namespace px4_topics
{

#define PX4_TOPIC(name, value) inline constexpr char name[] = value;
#include "px4_topics.def"
#undef PX4_TOPIC

}  // namespace px4_topics

namespace trajectory = offboard_controllers::trajectory;


// Supplies trajectory references and lifecycle commands; PX4 owns the
// position and lower-level loops. Trajectory time starts at confirmed Offboard
// entry, while lifecycle deadlines use the steady clock.
class OffboardPosition : public rclcpp::Node
{
public:
  OffboardPosition();

  int exit_code() const;

private:
  // State-machine deadlines are safety/lifecycle timeouts and must continue
  // monotonically even if ROS/simulation time is paused or reset.
  using SteadyClock = std::chrono::steady_clock;

  enum class Phase
  {
    WAIT_READY,
    WAIT_ARM,
    PRESTREAM,
    WAIT_OFFBOARD,
    RUN,
    WAIT_POSITION,
    DONE,
  };

  // At 20 Hz, 30 prestream samples prepare automatic entry. External
  // activation keeps streaming while awaiting PX4-confirmed takeover.
  static constexpr std::uint32_t kOffboardWarmupSamples = 30;

  static constexpr float kCustomModeEnabled = 1.0F;
  static constexpr float kPositionMainMode = 3.0F;
  static constexpr float kOffboardMainMode = 6.0F;
  static constexpr float kOffboardSubMode = 0.0F;


  void handle_vehicle_status(
    const px4_msgs::msg::VehicleStatus & msg);


  void handle_local_position(
    const px4_msgs::msg::VehicleLocalPosition & msg);


  void handle_command_ack(
    const px4_msgs::msg::VehicleCommandAck & msg);


  void publish_offboard_control_mode(
    std::uint64_t timestamp);


  void publish_trajectory_setpoint(
    std::uint64_t timestamp,
    const trajectory::Reference & reference);


  trajectory::Reference current_trajectory_reference()
  {
    if (
      phase_ != Phase::RUN &&
      phase_ != Phase::WAIT_POSITION)
    {
      trajectory_origin_ =
        trajectory::stationary_reference(
        local_position_,
        trajectory_name_ == "hover" ? local_heading_ : configured_trajectory_->yaw);

      return configured_trajectory_->sequence.sample(
        0.0,
        trajectory_origin_);
    }

    const double elapsed_s =
      (get_clock()->now() - trajectory_start_time_).seconds();

    return configured_trajectory_->sequence.sample(
      elapsed_s,
      trajectory_origin_);
  }


  void publish_offboard_stream();


  void publish_offboard_mode_command();


  void publish_position_mode_command();


  void fill_command_metadata(
    px4_msgs::msg::VehicleCommand & msg);


  void start_prestream();


  void set_phase(Phase phase);


  double phase_elapsed_seconds() const;


  void complete();


  void fail(const char * reason);


  // Only confirmed Position-mode return completes the experiment.
  // Automatic return has a timeout; external return waits for the pilot.
  // Unexpected state changes and disarm terminate with failure.
  void run();


  std::uint64_t now_us() const;


  std::unique_ptr<trajectory::ConfiguredTrajectory>
    configured_trajectory_;

  trajectory::Reference trajectory_origin_{};
  trajectory::Vector3 local_position_{};
  double local_heading_{0.0};

  offboard_controllers::OffboardTrigger offboard_trigger_{
    offboard_controllers::OffboardTrigger::AUTOMATIC};
  std::string trajectory_name_;

  rclcpp::Time trajectory_start_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
    vehicle_status_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    vehicle_local_position_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr
    vehicle_command_ack_sub_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
    offboard_control_mode_pub_;

  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
    trajectory_setpoint_pub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
    vehicle_command_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  Phase phase_{Phase::WAIT_READY};
  SteadyClock::time_point phase_started_at_{};

  bool status_received_{false};
  bool local_position_valid_{false};
  bool pre_flight_checks_pass_{false};

  std::uint32_t offboard_samples_published_{0};

  std::uint8_t arming_state_{0};
  std::uint8_t nav_state_{0};

  int exit_code_{1};
};
