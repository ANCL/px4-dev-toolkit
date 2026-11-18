// Consistency checks for the Offboard SE(3) control mathematics.
//
// Desired angular velocity/acceleration are compared with finite differences
// of the desired rotation. Under perfect tracking, the model-based terms must
// reduce to the reference derivatives and to the rigid-body equations.

#include <gtest/gtest.h>

#include <cmath>

#include <offboard_controllers/constants.hpp>
#include <offboard_controllers/math/operations.hpp>
#include <offboard_controllers/se3/control/attitude_control.hpp>
#include <offboard_controllers/se3/control/attitude_geometry.hpp>
#include <offboard_controllers/se3/control/attitude_reference.hpp>
#include <offboard_controllers/se3/control/force_dynamics.hpp>
#include <offboard_controllers/se3/control/normalized_torque.hpp>
#include <offboard_controllers/se3/control/physical_moment.hpp>
#include <offboard_controllers/se3/control/translation.hpp>
#include <trajectory_generator/trajectory.hpp>

namespace se3 = offboard_controllers::se3;
namespace math = offboard_controllers::math;
namespace tg = trajectory_generator;
using math::Vector3;

namespace {

constexpr double kStep = 1.0e-5;
constexpr double kPositionGain = 4.0;
constexpr double kVelocityGain = 3.0;

Vector3 vec(const tg::Vector3& value) { return {value[0], value[1], value[2]}; }

se3::Reference reference_at(double time_s) {
  // An aggressive figure-eight gives large desired rates and accelerations.
  static const tg::Segment segment = tg::Segment::figure_eight(6.0, 1.0, 0.8, 1.0);
  const tg::Reference sample = segment.sample(time_s, tg::stationary_reference({0.0, 0.0, -1.0}, 0.4));
  return {vec(sample.position), vec(sample.velocity), vec(sample.acceleration), vec(sample.jerk),
          vec(sample.snap), sample.yaw, sample.yaw_rate, sample.yaw_acceleration};
}

// With zero tracking error, B = a_d - g e3 and its derivatives are j_d, s_d.
se3::DesiredAttitudeDynamics desired_at(double time_s) {
  const se3::Reference reference = reference_at(time_s);
  const Vector3 gravity{0.0, 0.0, offboard_controllers::kStandardGravity};
  return se3::attitude_reference::compute_dynamics(reference.acceleration - gravity, reference.jerk,
                                                   reference.snap, reference.yaw, reference.yaw_rate,
                                                   reference.yaw_acceleration);
}

se3::State tracking_state(const se3::Reference& reference, const se3::DesiredAttitudeDynamics& desired) {
  return {reference.position, reference.velocity, desired.attitude, desired.angular_velocity};
}

void expect_near(const Vector3& actual, const Vector3& expected, double tolerance) {
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(actual.z, expected.z, tolerance);
}

Vector3 difference(const Vector3& after, const Vector3& before) {
  return (after - before) / (2.0 * kStep);
}

}  // namespace

TEST(Se3AttitudeReference, AngularVelocityAndAccelerationMatchFiniteDifferences) {
  for (int k = 1; k < 10; ++k) {
    const double t = 0.6 * k;  // Interior samples of the 6 s segment.
    const se3::DesiredAttitudeDynamics before = desired_at(t - kStep);
    const se3::DesiredAttitudeDynamics after = desired_at(t + kStep);
    const se3::DesiredAttitudeDynamics now = desired_at(t);

    // hat(Omega_d) = R_d^T dR_d/dt; extract the vee components.
    const math::RotationMatrix& r = now.attitude;
    const Vector3 db1 = difference(after.attitude.b1, before.attitude.b1);
    const Vector3 db2 = difference(after.attitude.b2, before.attitude.b2);
    const Vector3 db3 = difference(after.attitude.b3, before.attitude.b3);
    const Vector3 omega{math::dot(r.b3, db2), math::dot(r.b1, db3), math::dot(r.b2, db1)};

    expect_near(omega, now.angular_velocity, 1e-7);
    expect_near(difference(after.angular_velocity, before.angular_velocity), now.angular_acceleration,
                1e-6);

    // Right-handed orthonormal basis.
    EXPECT_NEAR(math::dot(r.b1, r.b2), 0.0, 1e-12);
    EXPECT_NEAR(math::norm(r.b3), 1.0, 1e-12);
    expect_near(math::cross(r.b1, r.b2), r.b3, 1e-12);
  }
}

TEST(Se3ForceDynamics, PerfectTrackingReducesToReferenceJerkAndSnap) {
  for (int k = 1; k < 12; ++k) {
    const double t = 0.5 * k;
    const se3::Reference reference = reference_at(t);
    const se3::State state = tracking_state(reference, desired_at(t));

    const se3::TranslationalOutput output =
        se3::translation::compute(state, reference, kPositionGain, kVelocityGain);
    const Vector3 first = se3::force_dynamics::first_derivative(state, reference, output.specific_force,
                                                                kPositionGain, kVelocityGain);
    const Vector3 second = se3::force_dynamics::second_derivative(
        state, reference, output.specific_force, first, kPositionGain, kVelocityGain);

    expect_near(output.acceleration, reference.acceleration, 1e-12);
    expect_near(first, reference.jerk, 1e-10);
    expect_near(second, reference.snap, 1e-9);
  }
}

// At perfect tracking, Lee's moment is the rigid-body moment that produces
// the desired angular acceleration: M = Omega x J Omega + J dOmega_d.
TEST(Se3AttitudeLaws, PerfectTrackingMomentsMatchRigidBodyDynamics) {
  const math::InertiaMatrix inertia{0.022, 0.001, 0.0, 0.023, 0.0, 0.043};
  const Vector3 k_r{6.0, 6.0, 0.4};
  const Vector3 k_omega{0.9, 0.9, 0.15};
  const Vector3 k_alpha{0.003, 0.003, 0.001};

  for (int k = 1; k < 10; ++k) {
    const double t = 0.6 * k;  // Interior samples of the 6 s segment.
    const se3::DesiredAttitudeDynamics desired = desired_at(t);
    const Vector3& omega = desired.angular_velocity;

    const Vector3 moment =
        se3::physical_moment::compute(desired.attitude, omega, desired, inertia, k_r, k_omega);
    const Vector3 expected = math::cross(omega, math::multiply(inertia, omega)) +
                             math::multiply(inertia, desired.angular_acceleration);
    expect_near(moment, expected, 1e-12);

    const se3::GeometricNormalizedOutput normalized = se3::normalized_torque::compute(
        desired.attitude, omega, desired, k_r, k_omega, k_alpha);
    expect_near(normalized.attitude_feedback, {}, 1e-12);
    expect_near(normalized.angular_velocity_feedback, {}, 1e-12);
    expect_near(normalized.normalized_torque,
                math::component_product(k_alpha, desired.angular_acceleration), 1e-12);
  }
}

// e_R sign convention: a positive rotation about body x relative to R_d
// gives a positive e_R.x, which the rate law opposes.
TEST(Se3AttitudeLaws, AttitudeErrorSignAndRateFeedback) {
  const double angle = 0.1;
  const math::RotationMatrix desired{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  // Body axes after rotating R_d by +angle about its x axis.
  const math::RotationMatrix attitude{
      {1.0, 0.0, 0.0}, {0.0, std::cos(angle), std::sin(angle)}, {0.0, -std::sin(angle), std::cos(angle)}};

  expect_near(se3::attitude_geometry::error(attitude, desired), {std::sin(angle), 0.0, 0.0}, 1e-12);

  const Vector3 gains{5.0, 5.0, 2.0};
  const Vector3 rate = se3::attitude_control::compute_rate(attitude, {desired, {}}, gains);
  expect_near(rate, {-gains.x * std::sin(angle), 0.0, 0.0}, 1e-12);
}
