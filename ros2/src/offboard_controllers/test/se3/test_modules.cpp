#include <cmath>
#include <limits>
#include <stdexcept>

#include <offboard_controllers/constants.hpp>
#include <offboard_controllers/se3/control/attitude_control.hpp>
#include <offboard_controllers/se3/control/attitude_geometry.hpp>
#include <offboard_controllers/se3/control/attitude_reference.hpp>
#include <offboard_controllers/se3/control/force_dynamics.hpp>
#include <offboard_controllers/se3/control/normalized_torque.hpp>
#include <offboard_controllers/se3/control/physical_moment.hpp>
#include <offboard_controllers/se3/control/translation.hpp>

namespace {

using namespace offboard_controllers::se3;
using offboard_controllers::kStandardGravity;

void near(double actual, double expected) {
  if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-9) {
    throw std::runtime_error("SE3 module output differs from reference value.");
  }
}

void near(const Vector3& actual, const Vector3& expected) {
  near(actual.x, expected.x);
  near(actual.y, expected.y);
  near(actual.z, expected.z);
}

template <typename F> void rejects(F&& operation) {
  try {
    operation();
  } catch (const std::invalid_argument&) {
    return;
  }
  throw std::runtime_error("Expected invalid_argument was not raised.");
}

void test_translation_and_force_dynamics() {
  State state{};
  state.attitude.b1 = {1.0, 0.0, 0.0};
  state.attitude.b2 = {0.0, 1.0, 0.0};
  state.attitude.b3 = {0.0, 0.0, 1.0};
  Reference reference{};
  reference.position = {0.0, 0.0, -2.0};
  state.position = reference.position;

  const auto output = translation::compute(state, reference, 2.0, 32.0, 11.2);
  near(output.acceleration, {0.0, 0.0, 0.0});
  near(output.force_vector, {0.0, 0.0, -2.0 * kStandardGravity});

  near(force_dynamics::first_derivative(state, reference, output.force_vector, 2.0, 32.0, 11.2),
       {0.0, 0.0, 0.0});
  near(
      force_dynamics::second_derivative(state, reference, output.force_vector, {}, 2.0, 32.0, 11.2),
      {0.0, 0.0, 0.0});

  // Translation permits zero force; desired attitude cannot normalize it.
  reference.acceleration = {0.0, 0.0, kStandardGravity};
  near(translation::compute(state, reference, 2.0, 32.0, 11.2).force_vector, {0.0, 0.0, 0.0});
  rejects([] { attitude_reference::compute_orientation({}, 0.0); });

  reference.yaw = std::numeric_limits<double>::quiet_NaN();
  rejects([&] { translation::compute(state, reference, 2.0, 32.0, 11.2); });
}

void test_attitude_reference_and_control() {
  const Vector3 force{0.0, 0.0, -10.0};
  const auto rotation = attitude_reference::compute_orientation(force, 0.0);
  near(rotation.b1, {1.0, 0.0, 0.0});
  near(rotation.b2, {0.0, 1.0, 0.0});
  near(rotation.b3, {0.0, 0.0, 1.0});
  near(attitude_geometry::error(rotation, rotation), {0.0, 0.0, 0.0});

  const auto desired_rate = attitude_reference::compute_rate(force, {}, 0.0, 0.4);
  near(desired_rate.angular_velocity, {0.0, 0.0, 0.4});
  near(attitude_control::compute_rate(rotation, desired_rate, {6.5, 6.5, 2.8}), {0.0, 0.0, 0.4});

  const auto desired_dynamics = attitude_reference::compute_dynamics(force, {}, {}, 0.0, 0.4, 0.2);
  near(desired_dynamics.angular_velocity, {0.0, 0.0, 0.4});
  near(desired_dynamics.angular_acceleration, {0.0, 0.0, 0.2});
}

void test_torque_and_moment() {
  const auto desired =
      attitude_reference::compute_dynamics({0.0, 0.0, -10.0}, {}, {}, 0.0, 0.0, 0.0);
  const Vector3 zero{};
  const auto torque =
      normalized_torque::compute(desired.attitude, zero, desired, {0.975, 0.975, 0.56},
                                 {0.15, 0.15, 0.2}, {0.003, 0.003, 0.0});
  near(torque.normalized_torque, zero);

  const InertiaMatrix inertia{0.02, 0.0, 0.0, 0.02, 0.0, 0.04};
  near(physical_moment::compute(desired.attitude, zero, desired, inertia, {5.0, 5.0, 1.0},
                                {1.0, 1.0, 0.5}),
       zero);
}

} // namespace

int main() {
  test_translation_and_force_dynamics();
  test_attitude_reference_and_control();
  test_torque_and_moment();
  return 0;
}
