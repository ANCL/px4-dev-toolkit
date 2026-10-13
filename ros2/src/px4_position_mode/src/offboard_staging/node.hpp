// State and ROS interfaces for Position-mode staging before Offboard.

#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>

#include "initial_setpoint.hpp"
#include <px4_control_common/virtual_joystick_client.hpp>
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


/*
 * Optional Position-mode staging helper for Offboard tests:
 *
 *   wait for PX4 readiness and a valid local position
 *       -> set the background virtual joystick to minimum throttle
 *       -> switch to Position mode
 *       -> arm
 *       -> climb to the configured Offboard initial z
 *       -> center the joystick and hold in Position mode
 *       -> wait until an independent controller enters Offboard
 *       -> exit while MAVProxy keeps the centered joystick alive
 *
 * Ownership:
 *   MAVProxy owns the continuous MANUAL_CONTROL stream. This helper changes
 *   virtual-joystick state through the local control socket and sends PX4
 *   mode/arming commands, but never requests Offboard itself.
 */
class OffboardTakeoffHandoff : public rclcpp::Node
{
public:
  OffboardTakeoffHandoff();

  int exit_code() const;

private:
  // State-machine deadlines are safety/lifecycle timeouts and must continue
  // monotonically even if ROS/simulation time is paused or reset.
  using SteadyClock = std::chrono::steady_clock;

  enum class Phase
  {
    WAIT_READY,
    WARMUP_JOYSTICK,
    WAIT_POSITION_MODE,
    WAIT_PREFLIGHT,
    WAIT_ARM,
    CLIMB,
    WAIT_OFFBOARD,
    DONE,
  };

  static constexpr float kPrearmThrottle = -1.0F;
  static constexpr float kClimbThrottle = 0.60F;
  static constexpr double kJoystickWarmupSeconds = 1.0;

  static constexpr float kCustomModeEnabled = 1.0F;
  static constexpr float kPositionMainMode = 3.0F;
  static constexpr float kPositionSubMode = 0.0F;


  void handle_vehicle_status(
    const px4_msgs::msg::VehicleStatus & msg);


  void handle_local_position(
    const px4_msgs::msg::VehicleLocalPosition & msg);


  void handle_command_ack(
    const px4_msgs::msg::VehicleCommandAck & msg);


  bool set_joystick(double throttle);


  bool center_joystick();


  void leave_joystick_safe();


  void publish_position_mode_command();


  void publish_arm_command();


  void fill_command_metadata(
    px4_msgs::msg::VehicleCommand & msg);


  void set_phase(Phase phase);


  double phase_elapsed_seconds() const;


  void fail(const char * reason);


  void complete();


  // State-machine contract:
  //   Inputs: PX4 status/local position and MAVProxy virtual-joystick access.
  //   Action: establish manual input, enter Position mode, arm, climb to the
  //           configured staging altitude, then center the joystick.
  //   Exit: success when PX4 confirms Offboard ownership by the controller;
  //         invalid state, disarm, or timeout is a failure.
  void run();


  std::uint64_t now_us() const;


  const px4_position_mode::InitialSetpoint initial_setpoint_;
  px4_control_common::VirtualJoystickClient virtual_joystick_;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
    vehicle_status_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    vehicle_local_position_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr
    vehicle_command_ack_sub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
    vehicle_command_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  Phase phase_{Phase::WAIT_READY};
  SteadyClock::time_point phase_started_at_{};

  bool status_received_{false};
  bool local_position_valid_{false};

  bool pre_flight_checks_pass_{false};
  bool waiting_for_preflight_reported_{false};

  float local_x_{NAN};
  float local_y_{NAN};
  float local_z_{NAN};

  std::uint8_t arming_state_{0};
  std::uint8_t nav_state_{0};

  int exit_code_{1};
};
