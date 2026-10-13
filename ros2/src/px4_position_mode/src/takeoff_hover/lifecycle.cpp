// Native Position-mode flight: joystick climb and timed hover, followed by
// a PX4 land command and disarm confirmation.

#include "node.hpp"

void PositionTakeoffHover::set_phase(Phase phase)
{
  phase_ = phase;
  phase_started_at_ = SteadyClock::now();
}


double PositionTakeoffHover::phase_elapsed_seconds() const
{
  return std::chrono::duration<double>(
    SteadyClock::now() - phase_started_at_).count();
}


void PositionTakeoffHover::fail(const char * reason)
{
  RCLCPP_ERROR(
    get_logger(),
    "Position-mode experiment failed: %s",
    reason);

  exit_code_ = 1;
  set_phase(Phase::DONE);
  leave_joystick_safe();
  timer_->cancel();
  rclcpp::shutdown();
}


void PositionTakeoffHover::complete()
{
  RCLCPP_INFO(
    get_logger(),
    "Position-mode takeoff and hover sequence complete.");

  exit_code_ = 0;
  set_phase(Phase::DONE);

  try {
    virtual_joystick_.set(0.0, 0.0, kPrearmThrottle, 0.0);
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      get_logger(),
      "Could not reset virtual joystick after landing: %s",
      error.what());
  }

  timer_->cancel();
  rclcpp::shutdown();
}


void PositionTakeoffHover::run()
{
  using VehicleStatus = px4_msgs::msg::VehicleStatus;

  if (phase_ == Phase::DONE) {
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
        start_x_ = local_x_;
        start_y_ = local_y_;
        start_z_ = local_z_;

        // PX4 local NED uses positive z Down; an upward climb therefore
        // subtracts the requested height from the starting z coordinate.
        target_z_ = start_z_ - kClimbHeightM;

        RCLCPP_INFO(
          get_logger(),
          "PX4 Position mode active: "
          "x=%.2f y=%.2f z=%.2f target_z=%.2f",
          static_cast<double>(start_x_),
          static_cast<double>(start_y_),
          static_cast<double>(start_z_),
          static_cast<double>(target_z_));

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
          "Vehicle armed; starting climb.");

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

      // Crossing to a smaller NED z means the requested upward climb has
      // reached or passed its target altitude.
      if (local_z_ <= target_z_) {
        if (!center_joystick()) {
          return;
        }

        RCLCPP_INFO(
          get_logger(),
          "Climb target reached: z=%.2f. "
          "Centering sticks for 20-second hover.",
          static_cast<double>(local_z_));

        set_phase(Phase::HOVER);
        return;
      }

      if (phase_elapsed_seconds() > 15.0) {
        fail("Vehicle did not reach the climb target.");
      }

      return;


    case Phase::HOVER:
      if (arming_state_ != VehicleStatus::ARMING_STATE_ARMED) {
        fail("Vehicle disarmed during hover.");
        return;
      }

      if (phase_elapsed_seconds() >= 20.0) {
        RCLCPP_INFO(
          get_logger(),
          "Hover complete; asking PX4 to land.");

        publish_land_command();
        set_phase(Phase::WAIT_LAND);
      }

      return;


    case Phase::WAIT_LAND:
      if (arming_state_ == VehicleStatus::ARMING_STATE_DISARMED) {
        RCLCPP_INFO(
          get_logger(),
          "PX4 landed and disarmed.");

        complete();
        return;
      }

      if (
        land_detected_received_ &&
        landed_ &&
        !landed_reported_)
      {
        RCLCPP_INFO(
          get_logger(),
          "PX4 reports landed; waiting for automatic disarm.");

        landed_reported_ = true;
      }

      if (phase_elapsed_seconds() > 30.0) {
        fail("PX4 did not complete landing and disarm.");
      }

      return;


    case Phase::DONE:
    default:
      return;
  }
}


std::uint64_t PositionTakeoffHover::now_us() const
{
  return static_cast<std::uint64_t>(
    get_clock()->now().nanoseconds() / 1000);
}
