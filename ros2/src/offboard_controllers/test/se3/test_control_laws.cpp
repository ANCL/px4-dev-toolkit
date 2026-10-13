#include <cmath>
#include <iostream>
#include <stdexcept>

#include <offboard_controllers/constants.hpp>
#include <offboard_controllers/math/operations.hpp>
#include <offboard_controllers/se3/control/attitude_control.hpp>
#include <offboard_controllers/se3/control/attitude_geometry.hpp>
#include <offboard_controllers/se3/control/attitude_reference.hpp>
#include <offboard_controllers/se3/control/force_dynamics.hpp>
#include <offboard_controllers/se3/control/normalized_torque.hpp>
#include <offboard_controllers/se3/control/physical_moment.hpp>
#include <offboard_controllers/se3/control/translation.hpp>

namespace
{

constexpr double kTolerance = 1.0e-9;
constexpr double kPi = 3.14159265358979323846;

using offboard_controllers::kStandardGravity;

void expect_near(
  const char * name,
  double actual,
  double expected)
{
  if (!std::isfinite(actual) || !std::isfinite(expected) ||
      std::abs(actual - expected) > kTolerance) {
    std::cerr
      << name
      << ": expected " << expected
      << ", got " << actual
      << '\n';

    throw std::runtime_error("SE3 control-law test failed.");
  }
}


void expect_near_with_tolerance(
  const char * name,
  double actual,
  double expected,
  double tolerance)
{
  if (!std::isfinite(actual) || !std::isfinite(expected) ||
      std::abs(actual - expected) > tolerance) {
    std::cerr
      << name
      << ": expected " << expected
      << ", got " << actual
      << " (tolerance " << tolerance << ")\n";

    throw std::runtime_error("SE3 control-law test failed.");
  }
}


void expect_vector_near(
  const char * name,
  const offboard_controllers::se3::Vector3 & actual,
  const offboard_controllers::se3::Vector3 & expected)
{
  expect_near(name, actual.x, expected.x);
  expect_near(name, actual.y, expected.y);
  expect_near(name, actual.z, expected.z);
}


offboard_controllers::se3::RotationMatrix yaw_rotation(
  double yaw)
{
  return {
    {std::cos(yaw), std::sin(yaw), 0.0},
    {-std::sin(yaw), std::cos(yaw), 0.0},
    {0.0, 0.0, 1.0},
  };
}


// Shared physical fixture for independent control-law regression tests.
// Mass [kg], translation gains [N/m] and [N s/m].
constexpr double kMass = 2.0;
constexpr double kKx = 8.0;
constexpr double kKv = 6.0;
const offboard_controllers::se3::Vector3 kAttitudeGain{6.5, 6.5, 2.8};
const offboard_controllers::se3::Vector3 kNormalizedAttitudeGain{0.975, 0.975, 0.56};
const offboard_controllers::se3::Vector3 kNormalizedRateGain{0.15, 0.15, 0.20};
const offboard_controllers::se3::Vector3 kNormalizedAccelerationGain{0.003, 0.003, 0.0};

namespace se3 = offboard_controllers::se3;

void test_hover_equilibrium()
{
  using namespace offboard_controllers::se3;


  State state{};
  state.position = {0.0, 0.0, -2.0};
  state.velocity = {0.0, 0.0, 0.0};

  Reference reference{};
  reference.position = {0.0, 0.0, -2.0};

  const auto output =
    se3::translation::compute(state, reference, kMass, kKx, kKv);

  expect_vector_near(
    "hover acceleration",
    output.acceleration,
    {0.0, 0.0, 0.0});

  expect_vector_near(
    "hover force",
    output.force_vector,
    {0.0, 0.0, -2.0 * kStandardGravity});
}


void test_tracking_correction()
{
  using namespace offboard_controllers::se3;


  State state{};
  state.position = {1.0, -2.0, 0.5};
  state.velocity = {0.5, -1.0, 0.25};

  Reference reference{};
  reference.acceleration = {1.0, 2.0, 3.0};

  const auto output =
    se3::translation::compute(state, reference, kMass, kKx, kKv);

  expect_vector_near(
    "tracking acceleration",
    output.acceleration,
    {-4.5, 13.0, 0.25});

  expect_vector_near(
    "tracking force",
    output.force_vector,
    {
      -9.0,
      26.0,
      2.0 * (0.25 - kStandardGravity),
    });
}


void test_force_derivative_uses_model_acceleration()
{
  using namespace offboard_controllers::se3;


  State state{};
  state.velocity = {0.5, -1.0, 0.25};
  state.attitude = yaw_rotation(0.0);

  Reference reference{};
  reference.acceleration = {1.0, 2.0, 3.0};
  reference.jerk = {0.1, 0.2, 0.3};

  // At identity attitude, A = -m g e3 implies f = m g and therefore
  // a_model = g e3 - (f / m)b3 = 0.
  const Vector3 force{
    0.0,
    0.0,
    -2.0 * kStandardGravity,
  };

  expect_vector_near(
    "model-based force derivative",
    se3::force_dynamics::first_derivative(
      state,
      reference,
      force, kMass, kKx, kKv),
    {2.2, 20.4, 16.6});
}


void test_force_second_derivative_uses_rigid_body_kinematics()
{
  using namespace offboard_controllers::se3;


  State state{};
  state.attitude = yaw_rotation(0.0);
  state.angular_velocity = {1.0, 0.0, 0.0};

  Reference reference{};

  const Vector3 force{
    0.0,
    0.0,
    -2.0 * kStandardGravity,
  };

  const Vector3 force_derivative{
    0.0,
    0.0,
    -2.0,
  };

  // With Omega = [1, 0, 0], b3_dot = [0, -1, 0]. The non-zero A_dot
  // additionally gives f_dot = 2 N/s, exercising both analytic jerk terms.
  expect_vector_near(
    "model-based force second derivative",
    se3::force_dynamics::second_derivative(
      state,
      reference,
      force,
      force_derivative, kMass, kKx, kKv),
    {0.0, -6.0 * kStandardGravity, 6.0});
}


void test_translation_allows_zero_force()
{
  using namespace offboard_controllers::se3;


  State state{};
  Reference reference{};
  reference.acceleration = {0.0, 0.0, kStandardGravity};

  const auto output =
    se3::translation::compute(state, reference, kMass, kKx, kKv);

  expect_vector_near(
    "zero force",
    output.force_vector,
    {0.0, 0.0, 0.0});
}


void test_hover_attitude_identity()
{
  using namespace offboard_controllers::se3;

  const RotationMatrix attitude =
    se3::attitude_reference::compute_orientation(
      {0.0, 0.0, -10.0},
      0.0);

  expect_vector_near("hover Rd b1", attitude.b1, {1.0, 0.0, 0.0});
  expect_vector_near("hover Rd b2", attitude.b2, {0.0, 1.0, 0.0});
  expect_vector_near("hover Rd b3", attitude.b3, {0.0, 0.0, 1.0});
}


void test_hover_attitude_yaw_90()
{
  using namespace offboard_controllers::se3;

  const RotationMatrix attitude =
    se3::attitude_reference::compute_orientation(
      {0.0, 0.0, -10.0},
      0.5 * kPi);

  expect_vector_near("yaw90 Rd b1", attitude.b1, {0.0, 1.0, 0.0});
  expect_vector_near("yaw90 Rd b2", attitude.b2, {-1.0, 0.0, 0.0});
  expect_vector_near("yaw90 Rd b3", attitude.b3, {0.0, 0.0, 1.0});
}


void test_tilted_attitude_is_orthonormal()
{
  using namespace offboard_controllers::se3;

  const RotationMatrix attitude =
    se3::attitude_reference::compute_orientation(
      {-1.0, 0.0, -1.0},
      0.0);

  const double inv_sqrt_2 =
    1.0 / std::sqrt(2.0);

  expect_vector_near(
    "tilted Rd b3",
    attitude.b3,
    {inv_sqrt_2, 0.0, inv_sqrt_2});

  expect_near("Rd b1 norm", norm(attitude.b1), 1.0);
  expect_near("Rd b2 norm", norm(attitude.b2), 1.0);
  expect_near("Rd b3 norm", norm(attitude.b3), 1.0);

  expect_near("Rd b1 dot b2", dot(attitude.b1, attitude.b2), 0.0);
  expect_near("Rd b1 dot b3", dot(attitude.b1, attitude.b3), 0.0);
  expect_near("Rd b2 dot b3", dot(attitude.b2, attitude.b3), 0.0);

  expect_vector_near(
    "Rd right handed",
    attitude.b1,
    cross(attitude.b2, attitude.b3));
}


void test_yaw_rate_feedforward()
{
  using namespace offboard_controllers::se3;


  const DesiredAttitudeRate desired =
    se3::attitude_reference::compute_rate(
      {0.0, 0.0, -10.0},
      {0.0, 0.0, 0.0},
      0.3,
      0.4);

  expect_vector_near(
    "yaw feedforward Omega_d",
    desired.angular_velocity,
    {0.0, 0.0, 0.4});

  expect_vector_near(
    "zero-error yaw feedforward command",
    se3::attitude_control::compute_rate(
      desired.attitude,
      desired, kAttitudeGain),
    {0.0, 0.0, 0.4});
}


void test_force_direction_rate_feedforward()
{
  using namespace offboard_controllers::se3;

  const DesiredAttitudeRate desired =
    se3::attitude_reference::compute_rate(
      {0.0, 0.0, -1.0},
      {-1.0, 0.0, 0.0},
      0.0,
      0.0);

  expect_vector_near(
    "tilt Omega_d",
    desired.angular_velocity,
    {0.0, 1.0, 0.0});
}


void test_rate_feedforward_is_rotated_into_current_body_frame()
{
  using namespace offboard_controllers::se3;


  const DesiredAttitudeRate desired{
    yaw_rotation(0.5 * kPi),
    {1.0, 0.0, 0.0},
  };

  expect_vector_near(
    "R^T Rd Omega_d",
    se3::attitude_control::compute_rate(
      yaw_rotation(0.0),
      desired, se3::Vector3{}),
    {0.0, 1.0, 0.0});
}


void test_desired_angular_acceleration_matches_rate_derivative()
{
  using namespace offboard_controllers::se3;


  const Vector3 force{-1.0, -0.3, -9.0};
  const Vector3 force_d1{0.4, -0.2, 0.1};
  const Vector3 force_d2{0.05, 0.02, -0.03};
  const double yaw = 0.2;
  const double yaw_rate = 0.3;
  const double yaw_acceleration = 0.4;

  const auto dynamics =
    se3::attitude_reference::compute_dynamics(
      force,
      force_d1,
      force_d2,
      yaw,
      yaw_rate,
      yaw_acceleration);

  // Oracle:
  //   Compare analytic desired angular acceleration with an independent
  //   central finite difference of desired angular velocity. The test varies
  //   input force/yaw polynomials rather than reusing internal derivatives.
  constexpr double h = 1.0e-5;

  const auto sample_rate =
    [&](double t)
    {
      const Vector3 force_t =
        force + t * force_d1 + 0.5 * t * t * force_d2;

      const Vector3 force_d1_t =
        force_d1 + t * force_d2;

      const double yaw_t =
        yaw + yaw_rate * t + 0.5 * yaw_acceleration * t * t;

      const double yaw_rate_t =
        yaw_rate + yaw_acceleration * t;

      return se3::attitude_reference::compute_rate(
        force_t,
        force_d1_t,
        yaw_t,
        yaw_rate_t).angular_velocity;
    };

  const Vector3 plus = sample_rate(h);
  const Vector3 minus = sample_rate(-h);
  const Vector3 numerical = (plus - minus) / (2.0 * h);

  expect_near_with_tolerance(
    "Omegadot x",
    dynamics.angular_acceleration.x,
    numerical.x,
    1.0e-7);
  expect_near_with_tolerance(
    "Omegadot y",
    dynamics.angular_acceleration.y,
    numerical.y,
    1.0e-7);
  expect_near_with_tolerance(
    "Omegadot z",
    dynamics.angular_acceleration.z,
    numerical.z,
    1.0e-7);
}


void test_attitude_feedback_sign()
{
  using namespace offboard_controllers::se3;

  const double yaw_error = 0.1;

  const DesiredAttitudeRate desired{
    yaw_rotation(0.0),
    {0.0, 0.0, 0.0},
  };

  const Vector3 error =
    se3::attitude_geometry::error(
      yaw_rotation(yaw_error),
      desired.attitude);

  expect_near(
    "positive yaw attitude error",
    error.z,
    std::sin(yaw_error));

  const Vector3 command =
    se3::attitude_control::compute_rate(
      yaw_rotation(yaw_error),
      desired, kAttitudeGain);

  expect_near(
    "yaw correction opposes error",
    command.z,
    -2.8 * std::sin(yaw_error));
}


void test_desired_angular_acceleration_from_yaw()
{
  using namespace offboard_controllers::se3;

  const DesiredAttitudeDynamics desired =
    se3::attitude_reference::compute_dynamics(
      {0.0, 0.0, -10.0},
      {},
      {},
      0.2,
      0.4,
      0.7);

  expect_vector_near(
    "yaw Omega_d",
    desired.angular_velocity,
    {0.0, 0.0, 0.4});

  expect_vector_near(
    "yaw Omegadot_d",
    desired.angular_acceleration,
    {0.0, 0.0, 0.7});
}


void test_geometric_normalized_torque_uses_direct_geometric_errors()
{
  using namespace offboard_controllers::se3;


  const DesiredAttitudeDynamics desired{
    yaw_rotation(0.0),
    {0.0, 0.0, 1.0},
    {1.0, 2.0, 3.0},
  };

  const GeometricNormalizedOutput output =
    se3::normalized_torque::compute(
      yaw_rotation(0.0),
      {0.0, 0.0, 0.5},
      desired, kNormalizedAttitudeGain, kNormalizedRateGain, kNormalizedAccelerationGain);

  expect_vector_near(
    "zero attitude feedback",
    output.attitude_feedback,
    {});
  expect_vector_near(
    "angular-velocity feedback",
    output.angular_velocity_feedback,
    {0.0, 0.0, 0.1});
  expect_vector_near(
    "angular-acceleration feedforward",
    output.angular_acceleration_feedforward,
    {0.003, 0.006, 0.0});
  expect_vector_near(
    "geometric-normalized torque",
    output.normalized_torque,
    {0.003, 0.006, 0.1});
}


void test_geometric_normalized_torque_uses_attitude_error_directly()
{
  using namespace offboard_controllers::se3;

  const double yaw_error = 0.1;

  const DesiredAttitudeDynamics desired{
    yaw_rotation(0.0),
    {},
    {},
  };

  const GeometricNormalizedOutput output =
    se3::normalized_torque::compute(
      yaw_rotation(yaw_error),
      {},
      desired, kNormalizedAttitudeGain, kNormalizedRateGain, kNormalizedAccelerationGain);

  expect_vector_near(
    "zero angular-velocity feedback",
    output.angular_velocity_feedback,
    {});
  expect_near(
    "direct normalized yaw attitude feedback",
    output.attitude_feedback.z,
    -0.56 * std::sin(yaw_error));
  expect_near(
    "direct normalized yaw torque",
    output.normalized_torque.z,
    -0.56 * std::sin(yaw_error));
}


void test_geometric_normalized_torque_includes_rotating_frame_acceleration()
{
  using namespace offboard_controllers::se3;


  const DesiredAttitudeDynamics desired{
    yaw_rotation(0.0),
    {0.0, 1.0, 0.0},
    {},
  };

  const GeometricNormalizedOutput output =
    se3::normalized_torque::compute(
      yaw_rotation(0.0),
      {0.0, 0.0, 1.0},
      desired, kNormalizedAttitudeGain, kNormalizedRateGain, kNormalizedAccelerationGain);

  expect_vector_near(
    "rotating-frame angular-velocity feedback",
    output.angular_velocity_feedback,
    {0.0, 0.15, -0.20});
  expect_vector_near(
    "rotating-frame angular-acceleration feedforward",
    output.angular_acceleration_feedforward,
    {0.003, 0.0, 0.0});
  expect_vector_near(
    "normalized rotating-frame torque",
    output.normalized_torque,
    {0.003, 0.15, -0.20});
}


void test_physical_moment_is_in_physical_dynamics_form()
{
  using namespace offboard_controllers::se3;


  const DesiredAttitudeDynamics desired{
    yaw_rotation(0.0),
    {},
    {},
  };

  const InertiaMatrix inertia{
    2.0, 0.0, 0.0,
    3.0, 0.0,
    4.0,
  };

  // Invariant:
  //   Non-spherical inertia and non-zero rates exercise both damping and the
  //   gyroscopic Omega x J Omega term in Lee's physical moment law.
  const Vector3 moment =
    se3::physical_moment::compute(
      yaw_rotation(0.0),
      {1.0, 2.0, 3.0},
      desired,
      inertia,
      {1.0, 1.0, 1.0},
      {0.1, 0.2, 0.3});

  expect_vector_near(
    "physical moment",
    moment,
    {5.9, -6.4, 1.1});
}


void test_physical_moment_includes_desired_angular_acceleration()
{
  using namespace offboard_controllers::se3;


  const DesiredAttitudeDynamics desired{
    yaw_rotation(0.0),
    {},
    {1.0, 0.0, 0.0},
  };

  const InertiaMatrix inertia{
    2.0, 0.0, 0.0,
    3.0, 0.0,
    4.0,
  };

  // Expected:
  //   With zero attitude/rate error and current rate, the physical moment
  //   reduces to J * Omegadot_d; Jxx=2 therefore gives 2 N m.
  expect_vector_near(
    "physical angular-acceleration feedforward",
    se3::physical_moment::compute(
      yaw_rotation(0.0),
      {},
      desired,
      inertia,
      {1.0, 1.0, 1.0},
      {1.0, 1.0, 1.0}),
    {2.0, 0.0, 0.0});
}


void test_invalid_attitude_geometry()
{
  using namespace offboard_controllers::se3;


  bool zero_force_threw = false;

  try {
    (void)se3::attitude_reference::compute_orientation(
      {0.0, 0.0, 0.0},
      0.0);
  } catch (const std::invalid_argument &) {
    zero_force_threw = true;
  }

  if (!zero_force_threw) {
    throw std::runtime_error(
            "SE3 attitude reference accepted a zero force vector.");
  }

  bool heading_threw = false;

  try {
    (void)se3::attitude_reference::compute_orientation(
      {-1.0, 0.0, 0.0},
      0.0);
  } catch (const std::invalid_argument &) {
    heading_threw = true;
  }

  if (!heading_threw) {
    throw std::runtime_error(
            "SE3 attitude reference accepted a degenerate heading.");
  }
}


// Direct mathematical interfaces reject invalid physical parameters and gains.
void test_invalid_parameters()
{
  using namespace offboard_controllers::se3;

  bool mass_rejected = false;
  try {
    (void)translation::compute(State{}, Reference{}, 0.0, kKx, kKv);
  } catch (const std::invalid_argument&) {
    mass_rejected = true;
  }
  if (!mass_rejected) {
    throw std::runtime_error("SE3 translation accepted an invalid mass.");
  }

  bool gain_rejected = false;
  try {
    const DesiredAttitudeRate desired{yaw_rotation(0.0), {}};
    (void)attitude_control::compute_rate(yaw_rotation(0.0), desired, {-1.0, 1.0, 1.0});
  } catch (const std::invalid_argument&) {
    gain_rejected = true;
  }
  if (!gain_rejected) {
    throw std::runtime_error("SE3 attitude control accepted a negative gain.");
  }

  bool normalized_gain_rejected = false;
  try {
    const DesiredAttitudeDynamics desired{yaw_rotation(0.0), {}, {}};
    (void)normalized_torque::compute(yaw_rotation(0.0), {}, desired,
                                     kNormalizedAttitudeGain, {-0.1, 0.1, 0.1},
                                     kNormalizedAccelerationGain);
  } catch (const std::invalid_argument&) {
    normalized_gain_rejected = true;
  }
  if (!normalized_gain_rejected) {
    throw std::runtime_error("SE3 normalized torque accepted a negative gain.");
  }
}

}  // namespace


int main()
{
  test_hover_equilibrium();
  test_tracking_correction();
  test_force_derivative_uses_model_acceleration();
  test_force_second_derivative_uses_rigid_body_kinematics();
  test_translation_allows_zero_force();
  test_hover_attitude_identity();
  test_hover_attitude_yaw_90();
  test_tilted_attitude_is_orthonormal();
  test_yaw_rate_feedforward();
  test_force_direction_rate_feedforward();
  test_rate_feedforward_is_rotated_into_current_body_frame();
  test_desired_angular_acceleration_matches_rate_derivative();
  test_attitude_feedback_sign();
  test_desired_angular_acceleration_from_yaw();
  test_geometric_normalized_torque_uses_direct_geometric_errors();
  test_geometric_normalized_torque_uses_attitude_error_directly();
  test_geometric_normalized_torque_includes_rotating_frame_acceleration();
  test_physical_moment_is_in_physical_dynamics_form();
  test_physical_moment_includes_desired_angular_acceleration();
  test_invalid_attitude_geometry();
  test_invalid_parameters();

  std::cout << "SE3 control-law tests passed.\n";
  return 0;
}
