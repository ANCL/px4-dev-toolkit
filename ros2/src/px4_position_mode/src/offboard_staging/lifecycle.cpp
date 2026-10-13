// Stage takeoff through MAVProxy manual control in PX4 Position mode;
// the helper waits for independent Offboard takeover and never requests it.

#include "node.hpp"

void OffboardTakeoffHandoff::set_phase(Phase phase)
{
  phase_ = phase;
  phase_started_at_ = SteadyClock::now();
}


double OffboardTakeoffHandoff::phase_elapsed_seconds() const
{
  return std::chrono::duration<double>(
    SteadyClock::now() - phase_started_at_).count();
}


void OffboardTakeoffHandoff::fail(const char * reason)
{
  RCLCPP_ERROR(
    get_logger(),
    "Offboard takeoff handoff failed: %s",
    reason);

  exit_code_ = 1;
  set_phase(Phase::DONE);
  leave_joystick_safe();
  timer_->cancel();
  rclcpp::shutdown();
}


void OffboardTakeoffHandoff::complete()
{
  RCLCPP_INFO(
    get_logger(),
    "PX4 Offboard mode active; virtual joystick remains centered "
    "and the staging helper is exiting.");

  exit_code_ = 0;
  set_phase(Phase::DONE);

  try {
    virtual_joystick_.center();
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      get_logger(),
      "Could not leave virtual joystick centered: %s",
      error.what());
  }

  timer_->cancel();
  rclcpp::shutdown();
}


void OffboardTakeoffHandoff::run()
{
  using VehicleStatus = px4_msgs::msg::VehicleStatus;

  if (phase_ == Phase::DONE) {
    return;
  }

  if (
    phase_ == Phase::WAIT_OFFBOARD &&
    status_received_ &&
    nav_state_ == VehicleStatus::NAVIGATION_STATE_OFFBOARD)
  {
    complete();
    return;
  }

  if (!status_received_) {
    if (
      phase_ == Phase::WAIT_READY &&
      phase_elapsed_seconds() > 20.0)
    {
      fail("PX4 vehicle status was not available.");
    }

    return;
  }

  if (!local_position_valid_) {
    if (
      phase_ != Phase::WAIT_READY &&
      phase_ != Phase::WARMUP_JOYSTICK)
    {
      fail("PX4 local position became invalid.");

    } else if (phase_elapsed_seconds() > 20.0) {
      fail("A valid PX4 local position was not available.");
    }

    return;
  }

  switch (phase_) {
    case Phase::WAIT_READY:
      if (!set_joystick(kPrearmThrottle)) {
        return;
      }

      RCLCPP_INFO(
        get_logger(),
        "Virtual joystick connected; warming manual input.");

      set_phase(Phase::WARMUP_JOYSTICK);
      return;


    case Phase::WARMUP_JOYSTICK:
      // Establish the continuously streamed MAVLink manual-control state
      // before issuing mode or arming commands.
      if (phase_elapsed_seconds() < kJoystickWarmupSeconds) {
        return;
      }

      publish_position_mode_command();
      set_phase(Phase::WAIT_POSITION_MODE);
      return;


    case Phase::WAIT_POSITION_MODE:
      if (nav_state_ == VehicleStatus::NAVIGATION_STATE_POSCTL) {
        RCLCPP_INFO(
          get_logger(),
          "PX4 Position mode active: "
          "x=%.2f y=%.2f z=%.2f target_z=%.2f",
          static_cast<double>(local_x_),
          static_cast<double>(local_y_),
          static_cast<double>(local_z_),
          static_cast<double>(initial_setpoint_.z));

        set_phase(Phase::WAIT_PREFLIGHT);
        return;
      }

      if (phase_elapsed_seconds() > 5.0) {
        fail("PX4 did not enter Position mode.");
      }

      return;


    case Phase::WAIT_PREFLIGHT:
      if (pre_flight_checks_pass_) {
        publish_arm_command();
        set_phase(Phase::WAIT_ARM);
        return;
      }

      if (!waiting_for_preflight_reported_) {
        RCLCPP_INFO(
          get_logger(),
          "Waiting for PX4 pre-flight checks.");

        waiting_for_preflight_reported_ = true;
      }

      if (phase_elapsed_seconds() > 10.0) {
        fail("PX4 pre-flight checks did not pass.");
      }

      return;


    case Phase::WAIT_ARM:
      if (arming_state_ == VehicleStatus::ARMING_STATE_ARMED) {
        if (!set_joystick(kClimbThrottle)) {
          return;
        }

        RCLCPP_INFO(
          get_logger(),
          "Vehicle armed; climbing to configured z=%.2f.",
          static_cast<double>(initial_setpoint_.z));

        set_phase(Phase::CLIMB);
        return;
      }

      if (phase_elapsed_seconds() > 5.0) {
        fail("PX4 did not arm.");
      }

      return;


    case Phase::CLIMB:
      if (arming_state_ != VehicleStatus::ARMING_STATE_ARMED) {
        fail("Vehicle disarmed during climb.");
        return;
      }

      // PX4 local NED uses positive z Down, so climbing upward decreases z.
      if (local_z_ <= initial_setpoint_.z) {
        if (!center_joystick()) {
          return;
        }

        RCLCPP_INFO(
          get_logger(),
          "Configured staging altitude reached: z=%.2f. "
          "Centering joystick and waiting for Offboard takeover.",
          static_cast<double>(local_z_));

        set_phase(Phase::WAIT_OFFBOARD);
        return;
      }

      if (phase_elapsed_seconds() > 15.0) {
        fail("Vehicle did not reach the configured staging altitude.");
      }

      return;


    case Phase::WAIT_OFFBOARD:
      if (arming_state_ != VehicleStatus::ARMING_STATE_ARMED) {
        fail("Vehicle disarmed while waiting for Offboard takeover.");
        return;
      }

      if (nav_state_ != VehicleStatus::NAVIGATION_STATE_POSCTL) {
        fail("PX4 left Position mode before Offboard takeover.");
      }

      return;


    case Phase::DONE:
    default:
      return;
  }
}


std::uint64_t OffboardTakeoffHandoff::now_us() const
{
  return static_cast<std::uint64_t>(
    get_clock()->now().nanoseconds() / 1000);
}
