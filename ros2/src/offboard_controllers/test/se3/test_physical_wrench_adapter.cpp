#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include <offboard_controllers/se3/physical_wrench_adapter.hpp>

namespace
{

constexpr double kTolerance = 1.0e-9;
constexpr double kSqrtHalf =
  0.70710678118654752440;


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

    throw std::runtime_error(
            "Physical-wrench/PX4 adapter test failed.");
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


// Fixture:
//   Mirror F450 physical propulsion and pinned PX4 allocator geometry
//   independently of adapt() for round-trip verification.
offboard_controllers::physical_wrench_adapter::Parameters
f450_parameters()
{
  using namespace offboard_controllers;

  constexpr double arm =
    0.1626345596714;

  constexpr double allocator_arm =
    0.159;

  constexpr double physical_yaw_ratio =
    0.0137;

  constexpr double allocator_yaw_ratio =
    0.014;

  constexpr double allocator_ct =
    6.5;

  return {
    {
      physical_wrench_adapter::PhysicalRotor{
        {arm, arm, -0.0115},
        physical_yaw_ratio,
      },
      physical_wrench_adapter::PhysicalRotor{
        {-arm, -arm, -0.0115},
        physical_yaw_ratio,
      },
      physical_wrench_adapter::PhysicalRotor{
        {arm, -arm, -0.0115},
        -physical_yaw_ratio,
      },
      physical_wrench_adapter::PhysicalRotor{
        {-arm, arm, -0.0115},
        -physical_yaw_ratio,
      },
    },
    {
      physical_wrench_adapter::AllocatorRotor{
        {allocator_arm, allocator_arm, 0.0},
        allocator_ct,
        allocator_yaw_ratio,
      },
      physical_wrench_adapter::AllocatorRotor{
        {-allocator_arm, -allocator_arm, 0.0},
        allocator_ct,
        allocator_yaw_ratio,
      },
      physical_wrench_adapter::AllocatorRotor{
        {allocator_arm, -allocator_arm, 0.0},
        allocator_ct,
        -allocator_yaw_ratio,
      },
      physical_wrench_adapter::AllocatorRotor{
        {-allocator_arm, allocator_arm, 0.0},
        allocator_ct,
        -allocator_yaw_ratio,
      },
    },
    {
      0.27,
      3.06,
      8.67,
    },
    0.0,
    1.0,
  };
}


double rotor_thrust_n(
  const offboard_controllers::physical_wrench_adapter::Parameters & parameters,
  double control)
{
  const auto & curve =
    parameters.actuator_thrust;

  return
    curve.constant_n +
    curve.linear_n * control +
    curve.quadratic_n * control * control;
}


struct PhysicalWrench
{
  double collective_thrust_n{0.0};
  offboard_controllers::se3::Vector3 moment_nm{};
};


// Oracle:
//   Independent forward physical model. Keeping this calculation outside the
//   adapter prevents the round-trip test from reusing the implementation it is
//   intended to verify.
PhysicalWrench physical_wrench_from_rotor_signals(
  const std::array<double, 4> & control)
{
  using namespace offboard_controllers;

  const auto parameters =
    f450_parameters();

  PhysicalWrench wrench{};

  for (std::size_t rotor = 0; rotor < 4; ++rotor) {
    const double thrust =
      rotor_thrust_n(
        parameters,
        control[rotor]);

    const auto & data =
      parameters.physical_rotors[rotor];

    wrench.collective_thrust_n +=
      thrust;

    wrench.moment_nm.x +=
      -data.position_frd_m.y *
      thrust;

    wrench.moment_nm.y +=
      data.position_frd_m.x *
      thrust;

    wrench.moment_nm.z +=
      data.yaw_moment_ratio *
      thrust;
  }

  return wrench;
}


void test_hover_round_trip()
{
  using namespace offboard_controllers;

  const std::array<double, 4> control{
    0.60,
    0.60,
    0.60,
    0.60,
  };

  const PhysicalWrench wrench =
    physical_wrench_from_rotor_signals(
      control);

  const auto output =
    physical_wrench_adapter::adapt(
      f450_parameters(),
      wrench.collective_thrust_n,
      wrench.moment_nm);

  expect_vector_near(
    "hover normalized torque",
    output.normalized_torque,
    {});

  expect_near(
    "hover normalized thrust",
    output.normalized_thrust_z,
    -0.60);

  for (std::size_t rotor = 0; rotor < 4; ++rotor) {
    expect_near(
      "hover actuator control",
      output.actuator_control[rotor],
      control[rotor]);
  }

  expect_near(
    "hover moment scale",
    output.moment_scale,
    1.0);
}


void test_roll_round_trip()
{
  using namespace offboard_controllers;

  const std::array<double, 4> control{
    0.55,
    0.65,
    0.65,
    0.55,
  };

  const PhysicalWrench wrench =
    physical_wrench_from_rotor_signals(
      control);

  const auto output =
    physical_wrench_adapter::adapt(
      f450_parameters(),
      wrench.collective_thrust_n,
      wrench.moment_nm);

  expect_vector_near(
    "roll normalized torque",
    output.normalized_torque,
    {
      0.05 / kSqrtHalf,
      0.0,
      0.0,
    });

  expect_near(
    "roll normalized thrust",
    output.normalized_thrust_z,
    -0.60);

  for (std::size_t rotor = 0; rotor < 4; ++rotor) {
    expect_near(
      "roll actuator round trip",
      output.actuator_control[rotor],
      control[rotor]);
  }
}


void test_yaw_round_trip()
{
  using namespace offboard_controllers;

  const std::array<double, 4> control{
    0.62,
    0.62,
    0.58,
    0.58,
  };

  const PhysicalWrench wrench =
    physical_wrench_from_rotor_signals(
      control);

  const auto output =
    physical_wrench_adapter::adapt(
      f450_parameters(),
      wrench.collective_thrust_n,
      wrench.moment_nm);

  expect_vector_near(
    "yaw normalized torque",
    output.normalized_torque,
    {
      0.0,
      0.0,
      0.02,
    });

  expect_near(
    "yaw normalized thrust",
    output.normalized_thrust_z,
    -0.60);

  for (std::size_t rotor = 0; rotor < 4; ++rotor) {
    expect_near(
      "yaw actuator round trip",
      output.actuator_control[rotor],
      control[rotor]);
  }
}


void test_infeasible_moment_is_scaled()
{
  using namespace offboard_controllers;

  const auto parameters =
    f450_parameters();

  const std::array<double, 4> hover_control{
    0.60,
    0.60,
    0.60,
    0.60,
  };

  const PhysicalWrench hover =
    physical_wrench_from_rotor_signals(
      hover_control);

  const se3::Vector3 requested_moment{
    10.0,
    0.0,
    0.0,
  };

  const auto output =
    physical_wrench_adapter::adapt(
      parameters,
      hover.collective_thrust_n,
      requested_moment);

  if (
    !(output.moment_scale > 0.0) ||
    !(output.moment_scale < 1.0))
  {
    throw std::runtime_error(
            "Physical-wrench/PX4 adapter did not scale an infeasible moment.");
  }

  const double minimum_rotor_thrust =
    rotor_thrust_n(
      parameters,
      parameters.actuator_control_min);

  const double maximum_rotor_thrust =
    rotor_thrust_n(
      parameters,
      parameters.actuator_control_max);

  double applied_collective = 0.0;
  se3::Vector3 applied_moment{};

  for (std::size_t rotor = 0; rotor < 4; ++rotor) {
    const double thrust =
      output.rotor_thrust_n[rotor];

    if (
      thrust < minimum_rotor_thrust - kTolerance ||
      thrust > maximum_rotor_thrust + kTolerance)
    {
      throw std::runtime_error(
              "Physical-wrench/PX4 adapter produced an infeasible rotor thrust.");
    }

    const auto & data =
      parameters.physical_rotors[rotor];

    applied_collective +=
      thrust;

    applied_moment.x +=
      -data.position_frd_m.y *
      thrust;

    applied_moment.y +=
      data.position_frd_m.x *
      thrust;

    applied_moment.z +=
      data.yaw_moment_ratio *
      thrust;
  }

  expect_near(
    "scaled collective thrust",
    applied_collective,
    hover.collective_thrust_n);

  expect_near(
    "scaled roll moment",
    applied_moment.x,
    output.moment_scale *
    requested_moment.x);

  expect_near(
    "scaled pitch moment",
    applied_moment.y,
    output.moment_scale *
    requested_moment.y);

  expect_near(
    "scaled yaw moment",
    applied_moment.z,
    output.moment_scale *
    requested_moment.z);
}


void test_collective_is_saturated()
{
  using namespace offboard_controllers;

  const auto parameters =
    f450_parameters();

  const double minimum_collective =
    static_cast<double>(
      physical_wrench_adapter::kRotorCount) *
    rotor_thrust_n(
      parameters,
      parameters.actuator_control_min);

  const double maximum_collective =
    static_cast<double>(
      physical_wrench_adapter::kRotorCount) *
    rotor_thrust_n(
      parameters,
      parameters.actuator_control_max);

  const auto low_output =
    physical_wrench_adapter::adapt(
      parameters,
      0.0,
      {});

  expect_near(
    "minimum collective saturation",
    low_output.applied_collective_thrust_n,
    minimum_collective);

  expect_near(
    "minimum collective moment scale",
    low_output.moment_scale,
    1.0);

  const auto high_output =
    physical_wrench_adapter::adapt(
      parameters,
      100.0,
      {});

  expect_near(
    "maximum collective saturation",
    high_output.applied_collective_thrust_n,
    maximum_collective);

  expect_near(
    "maximum collective moment scale",
    high_output.moment_scale,
    1.0);

  for (std::size_t rotor = 0; rotor < 4; ++rotor) {
    const double expected_min =
      rotor_thrust_n(
        parameters,
        parameters.actuator_control_min);

    const double expected_max =
      rotor_thrust_n(
        parameters,
        parameters.actuator_control_max);

    expect_near(
      "minimum rotor thrust",
      low_output.rotor_thrust_n[rotor],
      expected_min);

    expect_near(
      "maximum rotor thrust",
      high_output.rotor_thrust_n[rotor],
      expected_max);
  }
}


}  // namespace


int main()
{
  test_hover_round_trip();
  test_roll_round_trip();
  test_yaw_round_trip();
  test_infeasible_moment_is_scaled();
  test_collective_is_saturated();

  std::cout
    << "Physical-wrench adapter tests passed.\n";

  return 0;
}
