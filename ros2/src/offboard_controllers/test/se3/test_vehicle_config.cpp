#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <system_error>
#include <yaml-cpp/yaml.h>
#include <stdexcept>

#include <offboard_controllers/se3/vehicle_config.hpp>

namespace
{

constexpr double kTolerance = 1.0e-12;


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

    throw std::runtime_error(
            "Vehicle configuration test failed.");
  }
}


// Invariant:
//   Lock F450 physical-model -> FRD conversion and PX4 allocator/propulsion
//   parameters to the configured values, including frame and sign conventions.
void test_f450_physical_wrench_configuration()
{
  using namespace offboard_controllers;

  const auto configuration =
    vehicle_config::load_physical_wrench_configuration(
      VEHICLE_CONFIG_DIR,
      "gz_f450");

  const auto & inertia =
    configuration.inertia_frd;

  expect_near(
    "inertia xx",
    inertia.xx,
    0.021997);

  expect_near(
    "inertia xy FLU->FRD",
    inertia.xy,
    -1.0842e-19);

  expect_near(
    "inertia xz FLU->FRD",
    inertia.xz,
    -3.38813e-21);

  expect_near(
    "inertia yy",
    inertia.yy,
    0.0221599);

  expect_near(
    "inertia yz FLU->FRD",
    inertia.yz,
    3.38813e-21);

  expect_near(
    "inertia zz",
    inertia.zz,
    0.0433006);

  const auto & physical =
    configuration.adapter.physical_rotors;

  expect_near(
    "rotor 0 x",
    physical[0].position_frd_m.x,
    0.1626345596714);

  expect_near(
    "rotor 0 y FLU->FRD",
    physical[0].position_frd_m.y,
    0.1626345596714);

  expect_near(
    "rotor 0 z relative COM",
    physical[0].position_frd_m.z,
    -0.011220486);

  expect_near(
    "rotor 0 yaw ratio",
    physical[0].yaw_moment_ratio,
    0.0137);

  expect_near(
    "rotor 2 yaw ratio",
    physical[2].yaw_moment_ratio,
    -0.0137);

  const auto & allocator =
    configuration.adapter.allocator_rotors;

  expect_near(
    "allocator rotor 0 x",
    allocator[0].position_frd_m.x,
    0.159);

  expect_near(
    "allocator rotor 0 y",
    allocator[0].position_frd_m.y,
    0.159);

  expect_near(
    "allocator CT",
    allocator[0].thrust_coefficient,
    6.5);

  expect_near(
    "allocator KM",
    allocator[0].moment_ratio,
    0.014);

  expect_near(
    "thrust curve constant",
    configuration.adapter.actuator_thrust.constant_n,
    0.27);

  expect_near(
    "thrust curve linear",
    configuration.adapter.actuator_thrust.linear_n,
    3.06);

  expect_near(
    "thrust curve quadratic",
    configuration.adapter.actuator_thrust.quadratic_n,
    8.67);

  expect_near(
    "minimum actuator control",
    configuration.adapter.actuator_control_min,
    0.0);

  expect_near(
    "maximum actuator control",
    configuration.adapter.actuator_control_max,
    1.0);
}

struct TemporaryVehicleDirectory
{
  std::filesystem::path path;

  TemporaryVehicleDirectory()
  {
    char name[] = "/tmp/px4_vehicle_config_XXXXXX";
    const char * created = ::mkdtemp(name);

    if (created == nullptr) {
      throw std::runtime_error("Could not create temporary vehicle test directory.");
    }

    path = created;
  }

  ~TemporaryVehicleDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }

  TemporaryVehicleDirectory(const TemporaryVehicleDirectory &) = delete;
  TemporaryVehicleDirectory & operator=(const TemporaryVehicleDirectory &) = delete;
};


void test_invalid_vehicle_configuration()
{
  using namespace offboard_controllers;

  const TemporaryVehicleDirectory temporary;
  const YAML::Node baseline = YAML::LoadFile(
    std::string(VEHICLE_CONFIG_DIR) + "/gz_f450.yaml");

  auto expect_invalid = [&](const char * name) {
      try {
        (void)vehicle_config::load_physical_wrench_configuration(
          temporary.path.string(), "test_vehicle");
      } catch (const std::invalid_argument &) {
        return;
      }

      throw std::runtime_error(std::string(name) + " should be rejected.");
    };

  expect_invalid("missing vehicle file");

  auto test_case = [&](const char * name,
                       const std::function<void(YAML::Node &)> & mutate) {
      YAML::Node config = YAML::Clone(baseline);
      mutate(config);

      YAML::Emitter emitter;
      emitter << config;

      if (!emitter.good()) {
        throw std::runtime_error("Could not encode test vehicle YAML.");
      }

      const std::filesystem::path file = temporary.path / "test_vehicle.yaml";
      {
        std::ofstream output(file);
        output << emitter.c_str();

        if (!output.good()) {
          throw std::runtime_error("Could not write test vehicle YAML.");
        }
      }

      expect_invalid(name);
    };

  test_case("invalid plant frame", [](YAML::Node & config) {
      config["plant"]["frame"] = "ENU";
    });

  test_case("non-positive inertia", [](YAML::Node & config) {
      config["plant"]["inertia_kg_m2"]["xx"] = -1.0;
    });

  test_case("invalid center of mass", [](YAML::Node & config) {
      config["plant"]["center_of_mass_m"]["z"] = "not-a-number";
    });

  test_case("wrong rotor count", [](YAML::Node & config) {
      config["propulsion"]["rotor_count"] = 6;
    });

  test_case("duplicate physical rotor", [](YAML::Node & config) {
      config["propulsion"]["rotors"][1]["index"] = 0;
    });

  test_case("invalid physical rotor direction", [](YAML::Node & config) {
      config["propulsion"]["rotors"][0]["direction"] = "unknown";
    });

  test_case("invalid thrust polynomial", [](YAML::Node & config) {
      config["propulsion"]["actuator_thrust"]["polynomial_n"] =
        YAML::Load("[0.27, 3.06]");
    });

  test_case("invalid allocator frame", [](YAML::Node & config) {
      config["px4_allocator"]["frame"] = "FLU";
    });

  test_case("duplicate allocator rotor", [](YAML::Node & config) {
      config["px4_allocator"]["rotors"][1]["index"] = 0;
    });

  test_case("opposite rotor yaw direction", [](YAML::Node & config) {
      config["px4_allocator"]["rotors"][0]["km"] = -0.014;
    });
}


}  // namespace


int main()
{
  test_f450_physical_wrench_configuration();
  test_invalid_vehicle_configuration();

  std::cout
    << "Vehicle configuration tests passed.\n";

  return 0;
}
