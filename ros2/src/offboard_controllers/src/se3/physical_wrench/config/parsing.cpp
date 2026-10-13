// Parse physical-wrench parameters, frames, and inertia from YAML.

#include <cmath>
#include <stdexcept>
#include <string>

#include "internal.hpp"
#include <offboard_controllers/math/operations.hpp>

namespace offboard_controllers::vehicle_config::detail
{

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


// PX4 rotor geometry may be represented as planar [x, y] coordinates. Treat
// an omitted z component as zero while still accepting explicit [x, y, z].
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

  // For a symmetric 3x3 inertia tensor, Sylvester's criterion requires all
  // leading principal minors to be positive. This rejects non-physical or
  // malformed tensors before they reach the geometric moment controller.
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

}  // namespace offboard_controllers::vehicle_config::detail
