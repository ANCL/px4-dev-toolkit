#include <offboard_controllers/se3/runtime/vehicle_state.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace {

using offboard_controllers::se3::runtime::VehicleState;

void check(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

px4_msgs::msg::VehicleLocalPosition valid_position() {
  px4_msgs::msg::VehicleLocalPosition msg{};
  msg.xy_valid = true;
  msg.z_valid = true;
  msg.v_xy_valid = true;
  msg.v_z_valid = true;
  msg.x = 1.0F;
  msg.y = -2.0F;
  msg.z = -3.0F;
  msg.vx = 0.5F;
  msg.vy = -0.25F;
  msg.vz = 0.75F;
  return msg;
}

void test_state_readiness() {
  VehicleState state;
  check(!state.ready("acceleration", "geometric_normalized"), "Missing position accepted");

  auto position = valid_position();
  state.update_position(position);
  check(state.ready("acceleration", "geometric_normalized"), "Valid position rejected");
  check(!state.ready("attitude", "geometric_normalized"), "Missing attitude accepted");
  check(state.state().position.z == -3.0, "Position conversion incorrect");

  px4_msgs::msg::VehicleAttitude attitude{};
  attitude.q = {1.0F, 0.0F, 0.0F, 0.0F};
  state.update_attitude(attitude);
  check(state.ready("attitude", "geometric_normalized"), "Valid attitude rejected");
  check(state.ready("attitude_rate", "geometric_normalized"), "Attitude-rate handoff rejected");
  check(!state.ready("thrust_and_torque", "px4_rate"), "Missing gyro sample accepted");

  px4_msgs::msg::VehicleAngularVelocity gyro{};
  gyro.xyz = {0.2F, 0.3F, 0.4F};
  gyro.xyz_derivative = {0.5F, 0.6F, 0.7F};
  const auto sample = state.update_angular_velocity(gyro);
  check(std::abs(sample.angular_velocity.x - 0.2) < 1e-7, "Gyro value changed");
  check(std::abs(sample.angular_acceleration.y - 0.6) < 1e-7, "Gyro derivative changed");
  check(state.ready("thrust_and_torque", "px4_rate"), "PX4 rate handoff rejected");
  check(state.ready("thrust_and_torque", "geometric_normalized"), "Geometric handoff rejected");

  // The legacy readiness gate validates angular velocity, not its derivative.
  gyro.xyz_derivative[0] = std::numeric_limits<float>::quiet_NaN();
  const auto invalid_derivative = state.update_angular_velocity(gyro);
  check(std::isnan(invalid_derivative.angular_acceleration.x), "Gyro derivative was sanitized");
  check(state.ready("thrust_and_torque", "geometric_normalized"),
        "Gyro-derivative validation changed");
  gyro.xyz_derivative[0] = 0.5F;

  gyro.xyz[0] = std::numeric_limits<float>::quiet_NaN();
  const auto invalid_sample = state.update_angular_velocity(gyro);
  check(std::isnan(invalid_sample.angular_velocity.x), "Invalid sample was sanitized");
  check(state.ready("thrust_and_torque", "px4_rate"), "PX4 sample-received condition changed");
  check(!state.ready("thrust_and_torque", "geometric_normalized"), "Invalid gyro accepted");
  check(std::abs(state.state().angular_velocity.x - 0.2) < 1e-7,
        "Invalid gyro overwrote last valid value");

  attitude.q[0] = std::numeric_limits<float>::quiet_NaN();
  state.update_attitude(attitude);
  check(!state.ready("attitude", "geometric_normalized"), "Invalid attitude accepted");
  check(state.ready("acceleration", "geometric_normalized"), "Attitude invalidated acceleration");

  position.xy_valid = false;
  state.update_position(position);
  check(!state.ready("acceleration", "geometric_normalized"), "Invalid position accepted");
  check(state.state().position.z == -3.0, "Invalid position overwrote valid state");
}

void test_vehicle_status() {
  VehicleState state;
  check(state.landed_or_maybe_landed(), "Landing defaults changed");

  px4_msgs::msg::VehicleControlMode control{};
  control.flag_armed = true;
  state.update_control_mode(control);
  check(state.armed(), "Arming state not stored");

  px4_msgs::msg::VehicleLandDetected land{};
  land.landed = false;
  land.maybe_landed = false;
  state.update_land_detected(land);
  check(!state.landed_or_maybe_landed(), "Landing state not updated");

  px4_msgs::msg::VehicleStatus status{};
  status.vehicle_type = px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_ROTARY_WING;
  state.update_vehicle_status(status);
  check(state.rotary_wing(), "Vehicle type not decoded");
}

// The readiness gate may recover after malformed PX4 observations while
// retaining the last valid numeric state (including the old body-rate value).
void test_readiness_recovery() {
  VehicleState state;
  auto position = valid_position();
  state.update_position(position);

  px4_msgs::msg::VehicleAttitude attitude{};
  attitude.q = {1.0F, 0.0F, 0.0F, 0.0F};
  state.update_attitude(attitude);

  px4_msgs::msg::VehicleAngularVelocity gyro{};
  gyro.xyz = {0.3F, -0.2F, 0.1F};
  state.update_angular_velocity(gyro);
  check(state.ready("thrust_and_torque", "geometric_physical"),
        "Physical handoff should accept valid attitude and gyro");

  gyro.xyz[1] = std::numeric_limits<float>::quiet_NaN();
  state.update_angular_velocity(gyro);
  check(!state.ready("thrust_and_torque", "geometric_physical"),
        "Physical handoff accepted an invalid body rate");
  check(state.ready("thrust_and_torque", "px4_rate"),
        "PX4 rate handoff should still observe malformed gyro samples");
  check(std::abs(state.state().angular_velocity.y + 0.2) < 1e-6,
        "Malformed gyro sample replaced the last good body rate");

  gyro.xyz[1] = -0.25F;
  state.update_angular_velocity(gyro);
  check(state.ready("thrust_and_torque", "geometric_physical"),
        "Physical handoff did not recover after a valid gyro sample");

  attitude.q[0] = std::numeric_limits<float>::quiet_NaN();
  state.update_attitude(attitude);
  check(!state.ready("attitude_rate", "geometric_normalized"),
        "Attitude-rate readiness ignored a malformed quaternion");
  attitude.q[0] = 1.0F;
  state.update_attitude(attitude);
  check(state.ready("attitude_rate", "geometric_normalized"),
        "Attitude-rate readiness did not recover");

  position.xy_valid = false;
  state.update_position(position);
  check(!state.ready("acceleration", "geometric_normalized"),
        "Acceleration handoff ignored invalid position validity");
  position.xy_valid = true;
  state.update_position(position);
  check(state.ready("acceleration", "geometric_normalized"),
        "Acceleration readiness did not recover");
}

} // namespace

int main() {
  test_state_readiness();
  test_vehicle_status();
  test_readiness_recovery();
  return 0;
}
