// State and ROS interfaces for Position-mode takeoff and optional landing.

#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <px4_control_common/virtual_joystick_client.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
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
 * Minimal headless PX4 Position-mode flight sequence:
 *
 *   wait for PX4 readiness and a valid local position
 *       -> set the background virtual joystick to minimum throttle
 *       -> switch to Position mode
 *       -> arm
 *       -> command a climb of approximately 2 m
 *       -> center the joystick and hover for 20 seconds
 *       -> hold: leave PX4 armed in Position mode and exit (default)
 *       -> land: ask PX4 to land, then wait for disarm
 *
 * Ownership:
 *   MAVProxy owns the continuous MANUAL_CONTROL stream. This node changes
 *   virtual-joystick state through the local control socket and sends PX4
 *   mode/arm/land commands; it does not publish manual-control input via DDS.
 *
 * PX4 retains the native Position controller and lower control loops for the
 * entire sequence.
 */
class PositionTakeoffHover : public rclcpp::Node
{
public:
  PositionTakeoffHover();

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
    HOVER,
    WAIT_LAND,
    DONE,
  };

  static constexpr float kClimbHeightM = 2.0F;
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


  void publish_land_command();


  void fill_command_metadata(
    px4_msgs::msg::VehicleCommand & msg);


  void set_phase(Phase phase);


  double phase_elapsed_seconds() const;


  void fail(const char * reason);


  void complete();


  // State-machine contract:
  //   Inputs: PX4 status/local position/land state and virtual-joystick access.
  //   Action: enter Position mode, arm, climb, and hover for 20 seconds.
  //   Exit: hold leaves the vehicle armed with centered sticks (default);
  //         land waits for PX4 to land and disarm.
  void run();


  std::uint64_t now_us() const;


  const std::string finish_;  // "hold" (default) or "land"
  px4_control_common::VirtualJoystickClient virtual_joystick_;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
    vehicle_status_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    vehicle_local_position_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr
    vehicle_land_detected_sub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr
    vehicle_command_ack_sub_;

  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
    vehicle_command_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  Phase phase_{Phase::WAIT_READY};
  SteadyClock::time_point phase_started_at_{};

  bool status_received_{false};
  bool local_position_valid_{false};
  bool land_detected_received_{false};
  bool landed_{true};
  bool landed_reported_{false};

  bool pre_flight_checks_pass_{false};
  bool waiting_for_preflight_reported_{false};

  float local_x_{NAN};
  float local_y_{NAN};
  float local_z_{NAN};

  float start_x_{NAN};
  float start_y_{NAN};
  float start_z_{NAN};
  float target_z_{NAN};

  std::uint8_t arming_state_{0};
  std::uint8_t nav_state_{0};

  int exit_code_{1};
};
