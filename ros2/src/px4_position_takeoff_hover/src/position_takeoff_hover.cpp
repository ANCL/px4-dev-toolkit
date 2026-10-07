#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>

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
 *       -> ask PX4 to land at the current position
 *       -> wait for PX4 to land and disarm
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
  PositionTakeoffHover()
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

  int exit_code() const
  {
    return exit_code_;
  }

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
    const px4_msgs::msg::VehicleStatus & msg)
  {
    if (!status_received_ ||
      msg.arming_state != arming_state_ ||
      msg.nav_state != nav_state_)
    {
      RCLCPP_INFO(
        get_logger(),
        "PX4 status: arming_state=%u, nav_state=%u",
        static_cast<unsigned>(msg.arming_state),
        static_cast<unsigned>(msg.nav_state));
    }

    arming_state_ = msg.arming_state;
    nav_state_ = msg.nav_state;
    pre_flight_checks_pass_ = msg.pre_flight_checks_pass;
    status_received_ = true;
  }


  void handle_local_position(
    const px4_msgs::msg::VehicleLocalPosition & msg)
  {
    const bool valid =
      msg.xy_valid &&
      msg.z_valid &&
      msg.v_xy_valid &&
      msg.v_z_valid &&
      std::isfinite(msg.x) &&
      std::isfinite(msg.y) &&
      std::isfinite(msg.z);

    if (valid && !local_position_valid_) {
      RCLCPP_INFO(
        get_logger(),
        "Local position valid: x=%.2f y=%.2f z=%.2f",
        static_cast<double>(msg.x),
        static_cast<double>(msg.y),
        static_cast<double>(msg.z));
    }

    if (!valid && local_position_valid_) {
      RCLCPP_WARN(
        get_logger(),
        "PX4 local position is no longer valid.");
    }

    if (valid) {
      local_x_ = msg.x;
      local_y_ = msg.y;
      local_z_ = msg.z;
    }

    local_position_valid_ = valid;
  }


  void handle_command_ack(
    const px4_msgs::msg::VehicleCommandAck & msg)
  {
    using VehicleCommand = px4_msgs::msg::VehicleCommand;
    using VehicleCommandAck = px4_msgs::msg::VehicleCommandAck;

    if (
      msg.command != VehicleCommand::VEHICLE_CMD_DO_SET_MODE &&
      msg.command != VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM &&
      msg.command != VehicleCommand::VEHICLE_CMD_NAV_LAND)
    {
      return;
    }

    const bool accepted =
      msg.result ==
      VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;

    if (accepted) {
      RCLCPP_INFO(
        get_logger(),
        "PX4 accepted command %u.",
        static_cast<unsigned>(msg.command));

    } else {
      RCLCPP_ERROR(
        get_logger(),
        "PX4 rejected command %u: result=%u",
        static_cast<unsigned>(msg.command),
        static_cast<unsigned>(msg.result));
    }
  }


  bool set_joystick(double throttle)
  {
    try {
      virtual_joystick_.set(0.0, 0.0, throttle, 0.0);
      return true;

    } catch (const std::exception & error) {
      fail(error.what());
      return false;
    }
  }


  bool center_joystick()
  {
    try {
      virtual_joystick_.center();
      return true;

    } catch (const std::exception & error) {
      fail(error.what());
      return false;
    }
  }


  void leave_joystick_safe()
  {
    // An armed Position-mode vehicle is left with centered sticks so PX4 holds
    // its current state. Minimum throttle is restored only when disarmed.
    try {
      if (
        arming_state_ ==
        px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
      {
        virtual_joystick_.center();

      } else {
        virtual_joystick_.set(0.0, 0.0, kPrearmThrottle, 0.0);
      }

    } catch (const std::exception &) {
      // Failure reporting must not recursively fail while shutting down.
    }
  }


  void publish_position_mode_command()
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


  void publish_arm_command()
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


  void publish_land_command()
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


  void fill_command_metadata(
    px4_msgs::msg::VehicleCommand & msg)
  {
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = now_us();
  }


  void set_phase(Phase phase)
  {
    phase_ = phase;
    phase_started_at_ = SteadyClock::now();
  }


  double phase_elapsed_seconds() const
  {
    return std::chrono::duration<double>(
      SteadyClock::now() - phase_started_at_).count();
  }


  void fail(const char * reason)
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


  void complete()
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


  // State-machine contract:
  //   Inputs: PX4 status/local position/land state and virtual-joystick access.
  //   Action: enter Position mode, arm, climb, hold centered sticks, then issue
  //           PX4's native land command.
  //   Exit: success after landing and automatic disarm; invalid state,
  //         unexpected disarm, or phase timeout is a failure.
  void run()
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


  std::uint64_t now_us() const
  {
    return static_cast<std::uint64_t>(
      get_clock()->now().nanoseconds() / 1000);
  }


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
