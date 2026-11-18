// Checks of the physical-wrench adapter against independent physics.
//
// Synthetic symmetric X-quadrotor parameters (similar in scale to the Gazebo
// F450) are used so no vehicle configuration file is involved.
//
// For a symmetric X geometry, the pinned PX4 pseudo-inverse with its
// roll/pitch, yaw and thrust normalization reduces to the closed form
//
//   u_i = -c_z + (sqrt(2)/2) (sgn(-y_i) c_roll + sgn(x_i) c_pitch)
//         + sgn(km_i) c_yaw,
//
// which lets the test confirm that the normalized outputs reproduce the
// adapter's actuator controls without re-using the adapter's own matrices.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include <offboard_controllers/math/operations.hpp>
#include <offboard_controllers/se3/physical_wrench_adapter.hpp>

namespace adapter = offboard_controllers::physical_wrench_adapter;
namespace math = offboard_controllers::math;
using math::Vector3;

namespace {

constexpr double kArm = 0.16;
constexpr double kMomentRatio = 0.014;

// Rotor positions (FRD) and reaction-moment signs in PX4 quad-X order.
constexpr double kRotors[adapter::kRotorCount][3] = {
    {kArm, kArm, kMomentRatio},
    {-kArm, -kArm, kMomentRatio},
    {kArm, -kArm, -kMomentRatio},
    {-kArm, kArm, -kMomentRatio},
};

adapter::Parameters parameters() {
  adapter::Parameters p{};
  for (std::size_t i = 0; i < adapter::kRotorCount; ++i) {
    p.physical_rotors[i] = {{kRotors[i][0], kRotors[i][1], 0.0}, kRotors[i][2]};
    p.allocator_rotors[i] = {{kRotors[i][0], kRotors[i][1], 0.0}, 6.5, kRotors[i][2]};
  }
  p.actuator_thrust = {0.27, 3.06, 8.67};  // T(u) [N], u in [0, 1].
  p.actuator_control_min = 0.0;
  p.actuator_control_max = 1.0;
  return p;
}

double sign(double value) { return value > 0.0 ? 1.0 : -1.0; }

// Physical body wrench produced by the given rotor thrusts.
void physical_wrench(const adapter::Output& output, Vector3& moment, double& thrust) {
  moment = {};
  thrust = 0.0;
  for (std::size_t i = 0; i < adapter::kRotorCount; ++i) {
    const double t = output.rotor_thrust_n[i];
    moment = moment + Vector3{-kRotors[i][1] * t, kRotors[i][0] * t, kRotors[i][2] * t};
    thrust += t;
  }
}

void expect_near(const Vector3& actual, const Vector3& expected, double tolerance) {
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(actual.z, expected.z, tolerance);
}

}  // namespace

TEST(PhysicalWrenchAdapter, FeasibleWrenchIsReproducedExactly) {
  const adapter::Parameters p = parameters();
  const Vector3 requested_moment{0.2, -0.15, 0.03};
  const adapter::Output output = adapter::adapt(p, 20.0, requested_moment);

  EXPECT_DOUBLE_EQ(output.moment_scale, 1.0);
  EXPECT_DOUBLE_EQ(output.applied_collective_thrust_n, 20.0);

  Vector3 moment{};
  double thrust{};
  physical_wrench(output, moment, thrust);
  expect_near(moment, requested_moment, 1e-12);
  EXPECT_NEAR(thrust, 20.0, 1e-12);

  for (std::size_t i = 0; i < adapter::kRotorCount; ++i) {
    const double u = output.actuator_control[i];
    EXPECT_NEAR(0.27 + 3.06 * u + 8.67 * u * u, output.rotor_thrust_n[i], 1e-10);
  }
}

TEST(PhysicalWrenchAdapter, NormalizedOutputReproducesActuatorControlsThroughPx4Mixer) {
  const adapter::Output output = adapter::adapt(parameters(), 22.0, {-0.1, 0.25, -0.04});
  const Vector3& c = output.normalized_torque;
  const double half_sqrt2 = std::sqrt(0.5);

  EXPECT_LT(output.normalized_thrust_z, 0.0);  // Upward thrust is negative FRD z.
  for (std::size_t i = 0; i < adapter::kRotorCount; ++i) {
    const double mixed = -output.normalized_thrust_z +
                         half_sqrt2 * (sign(-kRotors[i][1]) * c.x + sign(kRotors[i][0]) * c.y) +
                         sign(kRotors[i][2]) * c.z;
    EXPECT_NEAR(mixed, output.actuator_control[i], 1e-12);
  }
}

// An infeasible moment is scaled uniformly: its direction is preserved and
// every rotor remains inside the propulsion envelope.
TEST(PhysicalWrenchAdapter, InfeasibleMomentIsScaledWithoutChangingDirection) {
  // About 9 N differential roll thrust per rotor around a 5 N hover share.
  const Vector3 requested_moment{6.0, 2.0, 0.2};
  const adapter::Output output = adapter::adapt(parameters(), 20.0, requested_moment);

  EXPECT_GT(output.moment_scale, 0.0);
  EXPECT_LT(output.moment_scale, 1.0);

  Vector3 moment{};
  double thrust{};
  physical_wrench(output, moment, thrust);
  expect_near(moment, output.moment_scale * requested_moment, 1e-10);
  EXPECT_NEAR(thrust, 20.0, 1e-10);

  const double max_thrust = 0.27 + 3.06 + 8.67;
  for (std::size_t i = 0; i < adapter::kRotorCount; ++i) {
    EXPECT_GE(output.rotor_thrust_n[i], 0.27 - 1e-12);
    EXPECT_LE(output.rotor_thrust_n[i], max_thrust + 1e-12);
  }
  // At least one rotor sits on the envelope boundary.
  const auto [low, high] = std::minmax_element(output.actuator_control.begin(), output.actuator_control.end());
  EXPECT_TRUE(std::fabs(*low) < 1e-9 || std::fabs(*high - 1.0) < 1e-9);
}
