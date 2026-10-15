// Offboard-position mode lifecycle. A successful command alone does not
// establish PX4 mode ownership; return is confirmed before completion.

#include "node.hpp"

void OffboardPosition::start_prestream()
{
  offboard_samples_published_ = 0;

  RCLCPP_INFO(
    get_logger(),
    "Starting Offboard heartbeat and configured setpoint prestream.");

  set_phase(Phase::PRESTREAM);
}


void OffboardPosition::set_phase(Phase phase)
{
  phase_ = phase;
  phase_started_at_ = SteadyClock::now();
}


double OffboardPosition::phase_elapsed_seconds() const
{
  return std::chrono::duration<double>(
    SteadyClock::now() - phase_started_at_).count();
}


void OffboardPosition::complete()
{
  RCLCPP_INFO(
    get_logger(),
    "Offboard position experiment complete.");

  exit_code_ = 0;
  set_phase(Phase::DONE);
  timer_->cancel();
  rclcpp::shutdown();
}


void OffboardPosition::fail(const char * reason)
{
  RCLCPP_ERROR(
    get_logger(),
    "Offboard position controller failed: %s",
    reason);

  exit_code_ = 1;
  set_phase(Phase::DONE);
  timer_->cancel();
  rclcpp::shutdown();
}


void OffboardPosition::run()
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
    if (phase_ != Phase::WAIT_READY) {
      fail("PX4 local position became invalid.");

    } else if (phase_elapsed_seconds() > 20.0) {
      fail("A valid PX4 local position was not available.");
    }

    return;
  }

  switch (phase_) {
    case Phase::WAIT_READY:
      if (arming_state_ == VehicleStatus::ARMING_STATE_ARMED) {
        RCLCPP_INFO(
          get_logger(),
          "Vehicle already armed; proceeding to Offboard prestream.");

        start_prestream();
        return;
      }

      RCLCPP_INFO(get_logger(), "Waiting for pilot/staging to arm the vehicle.");
      set_phase(Phase::WAIT_ARM);
      return;


    case Phase::WAIT_ARM:
      if (arming_state_ == VehicleStatus::ARMING_STATE_ARMED) {
        RCLCPP_INFO(get_logger(), "Vehicle armed externally.");
        start_prestream();
      }
      return;


    case Phase::PRESTREAM:
      if (arming_state_ != VehicleStatus::ARMING_STATE_ARMED) {
        fail("Vehicle disarmed before Offboard takeover.");
        return;
      }

      publish_offboard_stream();
      ++offboard_samples_published_;

      if (offboard_samples_published_ >= kOffboardWarmupSamples) {
        if (offboard_controllers::requests_offboard(offboard_trigger_)) {
          publish_offboard_mode_command();
        } else {
          RCLCPP_INFO(get_logger(), "Offboard ready; waiting for pilot mode switch.");
        }
        set_phase(Phase::WAIT_OFFBOARD);
      }

      return;


    case Phase::WAIT_OFFBOARD:
      if (arming_state_ != VehicleStatus::ARMING_STATE_ARMED) {
        fail("Vehicle disarmed while entering Offboard mode.");
        return;
      }

      publish_offboard_stream();

      if (nav_state_ == VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        // Unlike phase deadlines, trajectory time intentionally follows the
        // ROS clock so simulation time and the generated reference advance
        // on the same clock used by the ROS/PX4 experiment.
        trajectory_origin_ = trajectory::stationary_reference(
          local_position_, trajectory_name_ == "hover" ? local_heading_
                                                     : configured_trajectory_->yaw);
        trajectory_start_time_ = get_clock()->now();

        RCLCPP_INFO(
          get_logger(),
          "PX4 Offboard mode active; tracking trajectory '%s'.",
          trajectory_name_.c_str());

        set_phase(Phase::RUN);
        return;
      }

      if (offboard_controllers::requests_offboard(offboard_trigger_) &&
          phase_elapsed_seconds() > 5.0) {
        fail("PX4 did not enter Offboard mode.");
      }

      return;


    case Phase::RUN:
      if (arming_state_ != VehicleStatus::ARMING_STATE_ARMED) {
        fail("Vehicle disarmed during Offboard control.");
        return;
      }

      if (nav_state_ != VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        fail("PX4 left Offboard mode.");
        return;
      }

      if (
        (get_clock()->now() - trajectory_start_time_).seconds() >=
        configured_trajectory_->sequence.duration_s())
      {
        publish_offboard_stream();

        RCLCPP_INFO(
          get_logger(),
          "Trajectory '%s' complete after %.2f s; "
          "requesting PX4 Position mode.",
          trajectory_name_.c_str(),
          configured_trajectory_->sequence.duration_s());

        publish_position_mode_command();
        set_phase(Phase::WAIT_POSITION);
        return;
      }

      publish_offboard_stream();
      return;


    case Phase::WAIT_POSITION:
      if (arming_state_ != VehicleStatus::ARMING_STATE_ARMED) {
        fail("Vehicle disarmed while returning to Position mode.");
        return;
      }

      // Continue a valid Offboard stream until PX4 confirms Position mode.
      // This makes the explicit mode command, rather than heartbeat loss, the
      // reason PX4 relinquishes Offboard control.
      publish_offboard_stream();

      if (nav_state_ == VehicleStatus::NAVIGATION_STATE_POSCTL) {
        RCLCPP_INFO(
          get_logger(),
          "PX4 Position mode active.");

        complete();
        return;
      }

      if (phase_elapsed_seconds() > 5.0) {
        fail("PX4 did not return to Position mode.");
      }

      return;


    case Phase::DONE:
    default:
      return;
  }
}


std::uint64_t OffboardPosition::now_us() const
{
  return static_cast<std::uint64_t>(
    get_clock()->now().nanoseconds() / 1000);
}
