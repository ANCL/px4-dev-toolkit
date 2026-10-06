#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include <offboard_controllers/math/operations.hpp>
#include <offboard_controllers/se3/vehicle_config.hpp>

namespace offboard_controllers::vehicle_config
{

namespace
{

enum class BodyFrame
{
  FRD,
  FLU,
};


std::string vehicle_path(
  const std::string & config_directory,
  const std::string & vehicle)
{
  return
    config_directory +
    "/" +
    vehicle +
    ".yaml";
}


YAML::Node load_vehicle_root(
  const std::string & path)
{
  try {
    return YAML::LoadFile(path);

  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(
            "Unable to load vehicle configuration '" +
            path +
            "': " +
            error.what());
  }
}


double finite_scalar(
  const YAML::Node & node,
  const std::string & name)
{
  if (!node) {
    throw std::invalid_argument(
            "Vehicle configuration does not define " +
            name +
            ".");
  }

  double value{};

  try {
    value = node.as<double>();

  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(
            "Vehicle " +
            name +
            " is invalid: " +
            error.what());
  }

  if (!std::isfinite(value)) {
    throw std::invalid_argument(
            "Vehicle " +
            name +
            " must be finite.");
  }

  return value;
}


int integer_value(
  const YAML::Node & node,
  const std::string & name)
{
  if (!node) {
    throw std::invalid_argument(
            "Vehicle configuration does not define " +
            name +
            ".");
  }

  try {
    return node.as<int>();

  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(
            "Vehicle " +
            name +
            " is invalid: " +
            error.what());
  }
}


std::string string_value(
  const YAML::Node & node,
  const std::string & name)
{
  if (!node) {
    throw std::invalid_argument(
            "Vehicle configuration does not define " +
            name +
            ".");
  }

  try {
    return node.as<std::string>();

  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(
            "Vehicle " +
            name +
            " is invalid: " +
            error.what());
  }
}


BodyFrame parse_body_frame(
  const YAML::Node & node,
  const std::string & name)
{
  const std::string frame =
    string_value(
      node,
      name);

  if (frame == "FRD") {
    return BodyFrame::FRD;
  }

  if (frame == "FLU") {
    return BodyFrame::FLU;
  }

  throw std::invalid_argument(
          "Vehicle " +
          name +
          " must be FRD or FLU.");
}


se3::Vector3 sequence_vector3(
  const YAML::Node & node,
  const std::string & name)
{
  if (
    !node ||
    !node.IsSequence() ||
    node.size() != 3)
  {
    throw std::invalid_argument(
            "Vehicle " +
            name +
            " must contain exactly three values.");
  }

  const se3::Vector3 vector{
    finite_scalar(node[0], name + "[0]"),
    finite_scalar(node[1], name + "[1]"),
    finite_scalar(node[2], name + "[2]"),
  };

  if (!math::is_finite(vector)) {
    throw std::invalid_argument(
            "Vehicle " +
            name +
            " must be finite.");
  }

  return vector;
}


se3::Vector3 allocator_position(
  const YAML::Node & node,
  const std::string & name)
{
  if (
    !node ||
    !node.IsSequence() ||
    (
      node.size() != 2 &&
      node.size() != 3
    ))
  {
    throw std::invalid_argument(
            "Vehicle " +
            name +
            " must contain two or three values.");
  }

  return {
    finite_scalar(node[0], name + "[0]"),
    finite_scalar(node[1], name + "[1]"),
    node.size() == 3 ?
    finite_scalar(node[2], name + "[2]") :
    0.0,
  };
}


se3::Vector3 named_vector3(
  const YAML::Node & node,
  const std::string & name)
{
  if (!node || !node.IsMap()) {
    throw std::invalid_argument(
            "Vehicle " +
            name +
            " must be an x/y/z mapping.");
  }

  return {
    finite_scalar(node["x"], name + ".x"),
    finite_scalar(node["y"], name + ".y"),
    finite_scalar(node["z"], name + ".z"),
  };
}


se3::Vector3 to_frd(
  const se3::Vector3 & vector,
  BodyFrame frame)
{
  if (frame == BodyFrame::FRD) {
    return vector;
  }

  // FLU -> FRD is a 180-degree rotation about body X.
  return {
    vector.x,
    -vector.y,
    -vector.z,
  };
}


se3::InertiaMatrix inertia_to_frd(
  const se3::InertiaMatrix & inertia,
  BodyFrame frame)
{
  if (frame == BodyFrame::FRD) {
    return inertia;
  }

  // J_FRD = C J_FLU C^T, C = diag(1, -1, -1).
  return {
    inertia.xx,
    -inertia.xy,
    -inertia.xz,
    inertia.yy,
    inertia.yz,
    inertia.zz,
  };
}


void validate_inertia(
  const se3::InertiaMatrix & inertia)
{
  if (!math::is_finite(inertia)) {
    throw std::invalid_argument(
            "Vehicle inertia must be finite.");
  }

  const double second_minor =
    inertia.xx * inertia.yy -
    inertia.xy * inertia.xy;

  const double determinant =
    inertia.xx *
    (
      inertia.yy * inertia.zz -
      inertia.yz * inertia.yz
    ) -
    inertia.xy *
    (
      inertia.xy * inertia.zz -
      inertia.yz * inertia.xz
    ) +
    inertia.xz *
    (
      inertia.xy * inertia.yz -
      inertia.yy * inertia.xz
    );

  if (
    inertia.xx <= 0.0 ||
    second_minor <= 0.0 ||
    determinant <= 0.0)
  {
    throw std::invalid_argument(
            "Vehicle inertia must be positive definite.");
  }
}


int direction_sign(
  const YAML::Node & node,
  const std::string & name)
{
  const std::string direction =
    string_value(
      node,
      name);

  if (direction == "ccw") {
    return 1;
  }

  if (direction == "cw") {
    return -1;
  }

  throw std::invalid_argument(
          "Vehicle " +
          name +
          " must be cw or ccw.");
}

}  // namespace


PhysicalWrenchConfiguration load_physical_wrench_configuration(
  const std::string & config_directory,
  const std::string & vehicle)
{
  const std::string path =
    vehicle_path(
      config_directory,
      vehicle);

  const YAML::Node root =
    load_vehicle_root(
      path);

  const YAML::Node plant =
    root["plant"];

  if (!plant) {
    throw std::invalid_argument(
            "Vehicle configuration '" +
            path +
            "' does not define plant.");
  }

  const BodyFrame plant_frame =
    parse_body_frame(
      plant["frame"],
      "plant.frame");

  const YAML::Node inertia_node =
    plant["inertia_kg_m2"];

  if (!inertia_node) {
    throw std::invalid_argument(
            "Vehicle configuration '" +
            path +
            "' does not define plant.inertia_kg_m2.");
  }

  const se3::InertiaMatrix inertia_native{
    finite_scalar(
      inertia_node["xx"],
      "plant.inertia_kg_m2.xx"),
    finite_scalar(
      inertia_node["xy"],
      "plant.inertia_kg_m2.xy"),
    finite_scalar(
      inertia_node["xz"],
      "plant.inertia_kg_m2.xz"),
    finite_scalar(
      inertia_node["yy"],
      "plant.inertia_kg_m2.yy"),
    finite_scalar(
      inertia_node["yz"],
      "plant.inertia_kg_m2.yz"),
    finite_scalar(
      inertia_node["zz"],
      "plant.inertia_kg_m2.zz"),
  };

  const se3::InertiaMatrix inertia_frd =
    inertia_to_frd(
      inertia_native,
      plant_frame);

  validate_inertia(
    inertia_frd);

  const se3::Vector3 center_of_mass_frd =
    to_frd(
      named_vector3(
        plant["center_of_mass_m"],
        "plant.center_of_mass_m"),
      plant_frame);

  const YAML::Node propulsion =
    root["propulsion"];

  if (!propulsion) {
    throw std::invalid_argument(
            "Vehicle configuration '" +
            path +
            "' does not define propulsion.");
  }

  if (
    integer_value(
      propulsion["rotor_count"],
      "propulsion.rotor_count") !=
    static_cast<int>(
      physical_wrench_adapter::kRotorCount))
  {
    throw std::invalid_argument(
            "Physical-wrench adapter requires exactly four rotors.");
  }

  const BodyFrame propulsion_frame =
    parse_body_frame(
      propulsion["frame"],
      "propulsion.frame");

  const double moment_ratio =
    finite_scalar(
      propulsion["moment_ratio_m"],
      "propulsion.moment_ratio_m");

  if (moment_ratio <= 0.0) {
    throw std::invalid_argument(
            "Vehicle propulsion.moment_ratio_m must be positive.");
  }

  const YAML::Node physical_rotors =
    propulsion["rotors"];

  if (
    !physical_rotors ||
    !physical_rotors.IsSequence() ||
    physical_rotors.size() !=
    physical_wrench_adapter::kRotorCount)
  {
    throw std::invalid_argument(
            "propulsion.rotors must define exactly four rotors.");
  }

  std::array<
    physical_wrench_adapter::PhysicalRotor,
    physical_wrench_adapter::kRotorCount>
  physical{};

  std::array<
    bool,
    physical_wrench_adapter::kRotorCount>
  physical_seen{};

  for (const YAML::Node & rotor : physical_rotors) {
    const int index =
      integer_value(
        rotor["index"],
        "propulsion.rotors[].index");

    if (
      index < 0 ||
      index >=
      static_cast<int>(
        physical_wrench_adapter::kRotorCount) ||
      physical_seen[
        static_cast<std::size_t>(index)])
    {
      throw std::invalid_argument(
              "Physical rotor indexes must be unique values 0..3.");
    }

    physical_seen[
      static_cast<std::size_t>(index)] = true;

    const se3::Vector3 rotor_position_frd =
      to_frd(
        sequence_vector3(
          rotor["position_m"],
          "propulsion.rotors[].position_m"),
        propulsion_frame);

    physical[
      static_cast<std::size_t>(index)] = {
      rotor_position_frd -
      center_of_mass_frd,
      static_cast<double>(
        direction_sign(
          rotor["direction"],
          "propulsion.rotors[].direction")) *
      moment_ratio,
    };
  }

  const YAML::Node actuator_thrust =
    propulsion["actuator_thrust"];

  if (!actuator_thrust) {
    throw std::invalid_argument(
            "Vehicle configuration '" +
            path +
            "' does not define propulsion.actuator_thrust.");
  }

  const double actuator_control_min =
    finite_scalar(
      actuator_thrust["control_min"],
      "propulsion.actuator_thrust.control_min");

  const double actuator_control_max =
    finite_scalar(
      actuator_thrust["control_max"],
      "propulsion.actuator_thrust.control_max");

  const se3::Vector3 thrust_polynomial =
    sequence_vector3(
      actuator_thrust["polynomial_n"],
      "propulsion.actuator_thrust.polynomial_n");

  const YAML::Node allocator =
    root["px4_allocator"];

  if (!allocator) {
    throw std::invalid_argument(
            "Vehicle configuration '" +
            path +
            "' does not define px4_allocator.");
  }

  if (
    parse_body_frame(
      allocator["frame"],
      "px4_allocator.frame") !=
    BodyFrame::FRD)
  {
    throw std::invalid_argument(
            "PX4 allocator geometry must be expressed in FRD.");
  }

  if (
    integer_value(
      allocator["airframe_type"],
      "px4_allocator.airframe_type") != 0 ||
    integer_value(
      allocator["rotor_count"],
      "px4_allocator.rotor_count") !=
    static_cast<int>(
      physical_wrench_adapter::kRotorCount))
  {
    throw std::invalid_argument(
            "Physical-wrench/PX4 adapter requires the four-rotor PX4 multirotor allocator.");
  }

  const double thrust_coefficient =
    finite_scalar(
      allocator["thrust_coefficient"],
      "px4_allocator.thrust_coefficient");

  if (thrust_coefficient <= 0.0) {
    throw std::invalid_argument(
            "PX4 allocator thrust coefficient must be positive.");
  }

  const YAML::Node allocator_rotors =
    allocator["rotors"];

  if (
    !allocator_rotors ||
    !allocator_rotors.IsSequence() ||
    allocator_rotors.size() !=
    physical_wrench_adapter::kRotorCount)
  {
    throw std::invalid_argument(
            "px4_allocator.rotors must define exactly four rotors.");
  }

  std::array<
    physical_wrench_adapter::AllocatorRotor,
    physical_wrench_adapter::kRotorCount>
  allocator_data{};

  std::array<bool, physical_wrench_adapter::kRotorCount>
  allocator_seen{};

  for (const YAML::Node & rotor : allocator_rotors) {
    const int index =
      integer_value(
        rotor["index"],
        "px4_allocator.rotors[].index");

    if (
      index < 0 ||
      index >=
      static_cast<int>(
        physical_wrench_adapter::kRotorCount) ||
      allocator_seen[
        static_cast<std::size_t>(index)])
    {
      throw std::invalid_argument(
              "PX4 allocator rotor indexes must be unique values 0..3.");
    }

    allocator_seen[
      static_cast<std::size_t>(index)] = true;

    const double km =
      finite_scalar(
        rotor["km"],
        "px4_allocator.rotors[].km");

    allocator_data[
      static_cast<std::size_t>(index)] = {
      allocator_position(
        rotor["position_m"],
        "px4_allocator.rotors[].position_m"),
      thrust_coefficient,
      km,
    };

    if (
      physical[
        static_cast<std::size_t>(index)].
      yaw_moment_ratio *
      km <= 0.0)
    {
      throw std::invalid_argument(
              "Gazebo and PX4 rotor yaw-moment directions disagree.");
    }
  }

  return {
    inertia_frd,
    {
      physical,
      allocator_data,
      {
        thrust_polynomial.x,
        thrust_polynomial.y,
        thrust_polynomial.z,
      },
      actuator_control_min,
      actuator_control_max,
    },
  };
}

}  // namespace offboard_controllers::vehicle_config
