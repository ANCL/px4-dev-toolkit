#include <cmath>
#include <iostream>
#include <stdexcept>

#include <offboard_controllers/px4_wrench.hpp>
#include <offboard_controllers/se3/types.hpp>

namespace
{

constexpr double kTolerance = 1.0e-9;
constexpr double kGravity = 9.80665;

void expect_near(
  const char * name,
  double actual,
  double expected)
{
  if (std::abs(actual - expected) > kTolerance) {
    std::cerr
      << name
      << ": expected " << expected
      << ", got " << actual
      << '\n';

    throw std::runtime_error("PX4 wrench test failed.");
  }
}


void test_hover_force_maps_to_hover_thrust()
{
  using offboard_controllers::px4_wrench::normalized_collective_thrust;
  using offboard_controllers::se3::Vector3;

  constexpr double mass = 2.0;
  constexpr double hover_thrust = 0.60;

  const Vector3 force{
    0.0,
    0.0,
    -mass * kGravity,
  };

  expect_near(
    "hover collective thrust",
    normalized_collective_thrust(
      force,
      mass,
      hover_thrust),
    hover_thrust);
}


void test_mapping_uses_force_magnitude()
{
  using offboard_controllers::px4_wrench::normalized_collective_thrust;
  using offboard_controllers::se3::Vector3;

  constexpr double mass = 2.0;
  constexpr double hover_thrust = 0.60;
  constexpr double weight = mass * kGravity;

  const Vector3 force{
    0.6 * weight,
    0.0,
    -0.8 * weight,
  };

  expect_near(
    "tilted collective thrust",
    normalized_collective_thrust(
      force,
      mass,
      hover_thrust),
    hover_thrust);
}


void test_mapping_does_not_hide_saturation()
{
  using offboard_controllers::px4_wrench::normalized_collective_thrust;
  using offboard_controllers::se3::Vector3;

  constexpr double mass = 2.0;
  constexpr double hover_thrust = 0.60;

  const Vector3 force{
    0.0,
    0.0,
    -2.0 * mass * kGravity,
  };

  expect_near(
    "double-weight collective thrust",
    normalized_collective_thrust(
      force,
      mass,
      hover_thrust),
    1.20);
}


void test_identity_rotation_maps_to_identity_quaternion()
{
  using offboard_controllers::px4_wrench::quaternion_from_rotation;
  using offboard_controllers::se3::RotationMatrix;

  const RotationMatrix rotation{
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0},
  };

  const auto q =
    quaternion_from_rotation(rotation);

  expect_near("identity qw", q[0], 1.0);
  expect_near("identity qx", q[1], 0.0);
  expect_near("identity qy", q[2], 0.0);
  expect_near("identity qz", q[3], 0.0);
}


void test_yaw_90_rotation_maps_to_px4_quaternion()
{
  using offboard_controllers::px4_wrench::quaternion_from_rotation;
  using offboard_controllers::se3::RotationMatrix;

  const RotationMatrix rotation{
    {0.0, 1.0, 0.0},
    {-1.0, 0.0, 0.0},
    {0.0, 0.0, 1.0},
  };

  const auto q =
    quaternion_from_rotation(rotation);

  const double half_sqrt_2 =
    std::sqrt(0.5);

  // PX4 quaternion ordering: [w, x, y, z].
  expect_near("yaw90 qw", q[0], half_sqrt_2);
  expect_near("yaw90 qx", q[1], 0.0);
  expect_near("yaw90 qy", q[2], 0.0);
  expect_near("yaw90 qz", q[3], half_sqrt_2);
}


void test_px4_quaternion_maps_to_body_to_ned_rotation()
{
  using offboard_controllers::px4_wrench::rotation_from_quaternion;

  const double half_sqrt_2 =
    std::sqrt(0.5);

  const auto rotation =
    rotation_from_quaternion({
      half_sqrt_2,
      0.0,
      0.0,
      half_sqrt_2,
    });

  expect_near("q->R b1 x", rotation.b1.x, 0.0);
  expect_near("q->R b1 y", rotation.b1.y, 1.0);
  expect_near("q->R b1 z", rotation.b1.z, 0.0);

  expect_near("q->R b2 x", rotation.b2.x, -1.0);
  expect_near("q->R b2 y", rotation.b2.y, 0.0);
  expect_near("q->R b2 z", rotation.b2.z, 0.0);

  expect_near("q->R b3 x", rotation.b3.x, 0.0);
  expect_near("q->R b3 y", rotation.b3.y, 0.0);
  expect_near("q->R b3 z", rotation.b3.z, 1.0);
}

}  // namespace


int main()
{
  test_hover_force_maps_to_hover_thrust();
  test_mapping_uses_force_magnitude();
  test_mapping_does_not_hide_saturation();
  test_identity_rotation_maps_to_identity_quaternion();
  test_yaw_90_rotation_maps_to_px4_quaternion();
  test_px4_quaternion_maps_to_body_to_ned_rotation();

  std::cout << "PX4 wrench tests passed.\n";
  return 0;
}
