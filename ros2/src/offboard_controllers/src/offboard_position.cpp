#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

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


/*
 * Independent PX4 Offboard position controller.
 *
 * The controller depends only on PX4's fused local state. It is therefore
 * agnostic to whether EKF2 receives external vision from Gazebo, Vicon, or
 * another estimator input.
 *
 *   wait for PX4 status and a valid local position
 *       -> arm if the vehicle is not already armed
 *       -> prestream OffboardControlMode + trajectory reference at t=0
 *       -> request Offboard
 *       -> wait until PX4 reports Offboard mode
 *       -> advance and publish the configured trajectory
 *
 * Trajectory generation is shared with the SE3 experiment. During prestream
 * the trajectory remains anchored to the current vehicle position and begins
 * advancing only after PX4 reports Offboard mode.
 */
class OffboardPosition : public rclcpp::Node
{
public:
  OffboardPosition()
  : Node("offboard_position")
  {
    trajectory_name_ =
      declare_parameter<std::string>(
      "trajectory",
      "hover");

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
      "Configured Offboard trajectory: %s",
      trajectory_name_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Waiting for PX4 vehicle status and valid local position.");
  }

  int exit_code() const
  {
    return exit_code_;
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  enum class Phase
  {
    WAIT_READY,
    WAIT_PREFLIGHT,
    WAIT_ARM,
    PRESTREAM,
    WAIT_OFFBOARD,
    RUN,
    WAIT_POSITION,
    DONE,
  };

  static constexpr std::uint32_t kOffboardWarmupSamples = 30;

  static constexpr float kCustomModeEnabled = 1.0F;
  static constexpr float kPositionMainMode = 3.0F;
  static constexpr float kOffboardMainMode = 6.0F;
  static constexpr float kOffboardSubMode = 0.0F;


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
      std::isfinite(msg.x) &&
      std::isfinite(msg.y) &&
      std::isfinite(msg.z);

    if (valid) {
      local_position_ = {
        static_cast<double>(msg.x),
        static_cast<double>(msg.y),
        static_cast<double>(msg.z),
      };
    }

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

    local_position_valid_ = valid;
  }


  void handle_command_ack(
    const px4_msgs::msg::VehicleCommandAck & msg)
  {
    using VehicleCommand = px4_msgs::msg::VehicleCommand;
    using VehicleCommandAck = px4_msgs::msg::VehicleCommandAck;

    if (
      msg.command != VehicleCommand::VEHICLE_CMD_DO_SET_MODE &&
      msg.command != VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM)
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


  void publish_offboard_control_mode(
    std::uint64_t timestamp)
  {
    px4_msgs::msg::OffboardControlMode msg{};

    msg.timestamp = timestamp;
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;

    offboard_control_mode_pub_->publish(msg);
  }


  void publish_trajectory_setpoint(
    std::uint64_t timestamp,
    const trajectory::Reference & reference)
  {
    px4_msgs::msg::TrajectorySetpoint msg{};

    msg.timestamp = timestamp;

    msg.position = {
      static_cast<float>(reference.position[0]),
      static_cast<float>(reference.position[1]),
      static_cast<float>(reference.position[2]),
    };

    msg.velocity = {
      static_cast<float>(reference.velocity[0]),
      static_cast<float>(reference.velocity[1]),
      static_cast<float>(reference.velocity[2]),
    };

    msg.acceleration = {
      static_cast<float>(reference.acceleration[0]),
      static_cast<float>(reference.acceleration[1]),
      static_cast<float>(reference.acceleration[2]),
    };

    msg.jerk = {
      static_cast<float>(reference.jerk[0]),
      static_cast<float>(reference.jerk[1]),
      static_cast<float>(reference.jerk[2]),
    };

    msg.yaw =
      static_cast<float>(reference.yaw);

    msg.yawspeed =
      static_cast<float>(reference.yaw_rate);

    trajectory_setpoint_pub_->publish(msg);
  }


  trajectory::Reference current_trajectory_reference()
  {
    if (
      phase_ != Phase::RUN &&
      phase_ != Phase::WAIT_POSITION)
    {
      trajectory_origin_ =
        trajectory::stationary_reference(
        local_position_,
        configured_trajectory_->yaw);

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


  void publish_offboard_stream()
  {
    const std::uint64_t timestamp = now_us();

    const trajectory::Reference reference =
      current_trajectory_reference();

    publish_offboard_control_mode(timestamp);
    publish_trajectory_setpoint(
      timestamp,
      reference);
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


  void publish_offboard_mode_command()
  {
    px4_msgs::msg::VehicleCommand msg{};

    msg.param1 = kCustomModeEnabled;
    msg.param2 = kOffboardMainMode;
    msg.param3 = kOffboardSubMode;

    msg.command =
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;

    fill_command_metadata(msg);
    vehicle_command_pub_->publish(msg);

    RCLCPP_INFO(
      get_logger(),
      "Offboard mode command sent after %u prestream samples.",
      static_cast<unsigned>(offboard_samples_published_));
  }


  void publish_position_mode_command()
  {
    px4_msgs::msg::VehicleCommand msg{};

    msg.param1 = kCustomModeEnabled;
    msg.param2 = kPositionMainMode;
    msg.param3 = 0.0F;

    msg.command =
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;

    fill_command_metadata(msg);
    vehicle_command_pub_->publish(msg);

    RCLCPP_INFO(
      get_logger(),
      "PX4 Position mode command sent.");
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


  void start_prestream()
  {
    offboard_samples_published_ = 0;

    RCLCPP_INFO(
      get_logger(),
      "Starting Offboard heartbeat and configured setpoint prestream.");

    set_phase(Phase::PRESTREAM);
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


  void complete()
  {
    RCLCPP_INFO(
      get_logger(),
      "Offboard position experiment complete.");

    exit_code_ = 0;
    set_phase(Phase::DONE);
    timer_->cancel();
    rclcpp::shutdown();
  }


  void fail(const char * reason)
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

        if (pre_flight_checks_pass_) {
          publish_arm_command();
          set_phase(Phase::WAIT_ARM);
          return;
        }

        RCLCPP_INFO(
          get_logger(),
          "Vehicle is disarmed; waiting for PX4 pre-flight checks.");

        set_phase(Phase::WAIT_PREFLIGHT);
        return;


      case Phase::WAIT_PREFLIGHT:
        if (arming_state_ == VehicleStatus::ARMING_STATE_ARMED) {
          RCLCPP_INFO(
            get_logger(),
            "Vehicle armed externally; proceeding to Offboard prestream.");

          start_prestream();
          return;
        }

        if (pre_flight_checks_pass_) {
          publish_arm_command();
          set_phase(Phase::WAIT_ARM);
          return;
        }

        if (phase_elapsed_seconds() > 10.0) {
          fail("PX4 pre-flight checks did not pass.");
        }

        return;


      case Phase::WAIT_ARM:
        if (arming_state_ == VehicleStatus::ARMING_STATE_ARMED) {
          RCLCPP_INFO(get_logger(), "Vehicle armed.");
          start_prestream();
          return;
        }

        if (phase_elapsed_seconds() > 5.0) {
          fail("PX4 did not arm.");
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
          publish_offboard_mode_command();
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
          trajectory_start_time_ =
            get_clock()->now();

          RCLCPP_INFO(
            get_logger(),
            "PX4 Offboard mode active; tracking trajectory '%s'.",
            trajectory_name_.c_str());

          set_phase(Phase::RUN);
          return;
        }

        if (phase_elapsed_seconds() > 5.0) {
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


  std::uint64_t now_us() const
  {
    return static_cast<std::uint64_t>(
      get_clock()->now().nanoseconds() / 1000);
  }


  std::unique_ptr<trajectory::ConfiguredTrajectory>
    configured_trajectory_;

  trajectory::Reference trajectory_origin_{};
  trajectory::Vector3 local_position_{};

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
