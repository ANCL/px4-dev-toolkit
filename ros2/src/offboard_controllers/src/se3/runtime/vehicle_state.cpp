// Update the SE(3) state estimate and verify handoff readiness.

#include <offboard_controllers/se3/runtime/vehicle_state.hpp>

#include <array>
#include <cmath>
#include <stdexcept>

#include <offboard_controllers/math/operations.hpp>
#include <offboard_controllers/px4/wrench.hpp>

namespace offboard_controllers::se3::runtime {

// PX4 validity flags gate control eligibility independently of the
// last stored numeric state; invalid updates never create fresh state.
void VehicleState::update_position(const px4_msgs::msg::VehicleLocalPosition& msg) {
  const bool valid = msg.xy_valid && msg.z_valid && msg.v_xy_valid && msg.v_z_valid &&
                     std::isfinite(msg.x) && std::isfinite(msg.y) && std::isfinite(msg.z) &&
                     std::isfinite(msg.vx) && std::isfinite(msg.vy) && std::isfinite(msg.vz);

  if (!valid) {
    position_valid_ = false;
    return;
  }

  state_.position = {static_cast<double>(msg.x), static_cast<double>(msg.y),
                     static_cast<double>(msg.z)};
  state_.velocity = {static_cast<double>(msg.vx), static_cast<double>(msg.vy),
                     static_cast<double>(msg.vz)};
  position_valid_ = true;
}

void VehicleState::update_attitude(const px4_msgs::msg::VehicleAttitude& msg) {
  try {
    state_.attitude = px4_wrench::rotation_from_quaternion(std::array<double, 4>{
        static_cast<double>(msg.q[0]),
        static_cast<double>(msg.q[1]),
        static_cast<double>(msg.q[2]),
        static_cast<double>(msg.q[3]),
    });
    attitude_valid_ = true;
  } catch (const std::invalid_argument&) {
    attitude_valid_ = false;
  }
}

// Preserve every raw gyro sample for the sample-timed inner rate loop,
// even when the latest measurement is invalid for direct geometric control.
AngularSample
VehicleState::update_angular_velocity(const px4_msgs::msg::VehicleAngularVelocity& msg) {
  const AngularSample sample{
      {static_cast<double>(msg.xyz[0]), static_cast<double>(msg.xyz[1]),
       static_cast<double>(msg.xyz[2])},
      {static_cast<double>(msg.xyz_derivative[0]), static_cast<double>(msg.xyz_derivative[1]),
       static_cast<double>(msg.xyz_derivative[2])},
  };

  angular_velocity_received_ = true;
  angular_velocity_valid_ = math::is_finite(sample.angular_velocity);
  if (angular_velocity_valid_) {
    state_.angular_velocity = sample.angular_velocity;
  }

  return sample;
}

void VehicleState::update_control_mode(const px4_msgs::msg::VehicleControlMode& msg) {
  armed_ = msg.flag_armed;
}

void VehicleState::update_land_detected(const px4_msgs::msg::VehicleLandDetected& msg) {
  landed_ = msg.landed;
  maybe_landed_ = msg.maybe_landed;
}

void VehicleState::update_vehicle_status(const px4_msgs::msg::VehicleStatus& msg) {
  rotary_wing_ = msg.vehicle_type == px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_ROTARY_WING;
}

bool VehicleState::ready(std::string_view handoff, std::string_view direct_controller) const {
  if (!position_valid_) {
    return false;
  }
  if (handoff == "acceleration") {
    return true;
  }
  if (!attitude_valid_) {
    return false;
  }
  if (handoff == "attitude" || handoff == "attitude_rate") {
    return true;
  }

  // The reproduced PX4 rate loop observes all gyro samples, even if a sample
  // is non-finite. Direct geometric torque instead requires a valid body rate.
  if (direct_controller == "px4_rate") {
    return angular_velocity_received_;
  }
  return angular_velocity_valid_;
}

} // namespace offboard_controllers::se3::runtime
