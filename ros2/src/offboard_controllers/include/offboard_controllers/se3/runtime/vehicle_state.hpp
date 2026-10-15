// PX4 state observations and readiness checks used by SE(3) handoffs.

#pragma once

#include <string_view>

#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::se3::runtime {

// Carries the complete gyro sample to the rate loop, including invalid samples.
// The reproduced PX4 rate stage must observe each timestamp_sample separately.
struct AngularSample {
  Vector3 angular_velocity{};
  Vector3 angular_acceleration{};
};

// Translate PX4 observations into the current validated state. Invalid samples
// clear readiness while retaining the last valid numeric observation.
class VehicleState {
public:
  void update_position(const px4_msgs::msg::VehicleLocalPosition& msg);
  void update_attitude(const px4_msgs::msg::VehicleAttitude& msg);
  AngularSample update_angular_velocity(const px4_msgs::msg::VehicleAngularVelocity& msg);
  void update_control_mode(const px4_msgs::msg::VehicleControlMode& msg);
  void update_land_detected(const px4_msgs::msg::VehicleLandDetected& msg);
  void update_vehicle_status(const px4_msgs::msg::VehicleStatus& msg);

  bool ready(std::string_view handoff, std::string_view direct_controller) const;

  const State& state() const { return state_; }
  bool armed() const { return armed_; }
  bool attitude_valid() const { return attitude_valid_; }
  bool rotary_wing() const { return rotary_wing_; }
  bool landed_or_maybe_landed() const { return landed_ || maybe_landed_; }

private:
  State state_{};
  bool position_valid_{false};
  bool attitude_valid_{false};
  bool angular_velocity_valid_{false};
  bool angular_velocity_received_{false};
  bool armed_{false};
  bool rotary_wing_{false};
  bool landed_{true};
  bool maybe_landed_{true};
};

} // namespace offboard_controllers::se3::runtime
