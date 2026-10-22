// Compile absolute PX4-local-NED YAML targets into relative segment primitives.
// The only origin passed to Sequence is the *configured* absolute start pose;
// measured vehicle position at Offboard entry never shifts the trajectory.

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <offboard_controllers/trajectory/trajectory.hpp>
#include <yaml-cpp/yaml.h>

namespace offboard_controllers::trajectory {
namespace {

Vector3 vector3(const YAML::Node& node, const std::string& label) {
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::invalid_argument(label + " must have three NED coordinates.");
  }
  Vector3 result{node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
  for (double value : result) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(label + " must contain finite numbers.");
    }
  }
  return result;
}

Vector3 delta(const Vector3& target, const Vector3& source) {
  return {target[0] - source[0], target[1] - source[1], target[2] - source[2]};
}

void require_same_position(const Vector3& current, const Vector3& required,
                           const std::string& name) {
  for (int axis = 0; axis != 3; ++axis) {
    if (std::abs(current[axis] - required[axis]) > 1.0e-8) {
      throw std::invalid_argument(
          "Trajectory segment '" + name + "' starts at another position. "
          "Add an explicit line or step to its absolute start point first.");
    }
  }
}

void require_fields(const YAML::Node& definition, const std::string& name) {
  if (!definition || !definition.IsMap() || !definition["type"] || !definition["duration"]) {
    throw std::invalid_argument("Trajectory segment '" + name +
                                "' requires type and duration.");
  }
  if (definition["offset"] || definition["yaw_offset"]) {
    throw std::invalid_argument("Trajectory segment '" + name +
                                "' uses removed relative YAML fields; use absolute "
                                "position / center / heading.");
  }
}

// Update cursor to the commanded endpoint, never to a measured vehicle pose.
Segment compile_segment(const YAML::Node& definitions, const std::string& name,
                        Vector3& cursor, double& heading) {
  const YAML::Node definition = definitions[name];
  require_fields(definition, name);
  const std::string type = definition["type"].as<std::string>();
  const double duration = definition["duration"].as<double>();

  // A misspelled/unused field must fail, never silently change the flown path.
  std::vector<std::string> allowed{"type", "duration"};
  if (type == "step" || type == "line") allowed.push_back("position");
  if (type == "yaw") allowed.push_back("heading");
  if (type == "circle" || type == "helix" || type == "figure_eight") {
    allowed.push_back("center");
    allowed.push_back("turns");
  }
  if (type == "circle" || type == "helix") allowed.push_back("radius");
  if (type == "helix") allowed.push_back("height");
  if (type == "figure_eight") {
    allowed.push_back("x_amplitude");
    allowed.push_back("y_amplitude");
  }
  for (const auto& field : definition) {
    const std::string key = field.first.as<std::string>();
    if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
      throw std::invalid_argument("Unexpected field '" + key +
                                  "' in trajectory segment '" + name + "'.");
    }
  }

  if (type == "hold") {
    return Segment::hold(duration);
  }
  if (type == "step" || type == "line") {
    const Vector3 target = vector3(definition["position"], name + " position");
    const Vector3 change = delta(target, cursor);
    cursor = target;
    return type == "step" ? Segment::step(duration, change)
                          : Segment::line(duration, change);
  }
  if (type == "yaw") {
    if (!definition["heading"]) {
      throw std::invalid_argument(name + " must specify absolute heading [rad].");
    }
    const double target = definition["heading"].as<double>();
    if (!std::isfinite(target)) {
      throw std::invalid_argument(name + " heading must be finite.");
    }
    const double difference = target - heading;
    heading = target;
    return Segment::yaw(duration, difference);
  }
  if (type == "figure_eight") {
    const Vector3 center = vector3(definition["center"], name + " center");
    require_same_position(cursor, center, name);
    const double x_amplitude = definition["x_amplitude"].as<double>();
    const double y_amplitude = definition["y_amplitude"].as<double>();
    const double turns = definition["turns"].as<double>();
    const auto segment = Segment::figure_eight(duration, x_amplitude, y_amplitude, turns);
    constexpr double kPi = 3.14159265358979323846;
    const double angle = 2.0 * kPi * turns;
    cursor = {center[0] + x_amplitude * std::sin(angle),
              center[1] + y_amplitude * std::sin(2.0 * angle), center[2]};
    return segment;
  }
  if (type == "circle" || type == "helix") {
    const Vector3 center = vector3(definition["center"], name + " center");
    const double radius = definition["radius"].as<double>();
    if (!std::isfinite(radius) || radius <= 0.0) {
      throw std::invalid_argument(name + " radius must be finite and positive.");
    }
    // Existing circle/helix sampling starts at center + (radius,0,0).
    require_same_position(cursor, {center[0] + radius, center[1], center[2]}, name);
    const double turns = definition["turns"].as<double>();
    if (type == "circle") {
      // General turn counts need not be integral, so compute terminal pose.
      constexpr double kPi = 3.14159265358979323846;
      cursor = {center[0] + radius * std::cos(2.0 * kPi * turns),
                center[1] + radius * std::sin(2.0 * kPi * turns), center[2]};
      return Segment::circle(duration, radius, turns);
    }
    const double height = definition["height"].as<double>();
    constexpr double kPi = 3.14159265358979323846;
    cursor = {center[0] + radius * std::cos(2.0 * kPi * turns),
              center[1] + radius * std::sin(2.0 * kPi * turns), center[2] + height};
    return Segment::helix(duration, radius, height, turns);
  }
  throw std::invalid_argument("Unsupported trajectory type '" + type + "' in " + name);
}

} // namespace

ConfiguredTrajectory load_trajectory(const std::string& path, const std::string& name) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& error) {
    throw std::invalid_argument("Unable to load trajectory '" + path + "': " + error.what());
  }

  if (!root["frame"] || root["frame"].as<std::string>() != "px4_local_ned") {
    throw std::invalid_argument("Trajectory YAML must declare frame: px4_local_ned.");
  }
  const YAML::Node definitions = root["segments"];
  const YAML::Node trajectories = root["trajectories"];
  if (!definitions || !definitions.IsMap() || !trajectories || !trajectories.IsMap()) {
    throw std::invalid_argument("Trajectory YAML requires segments and trajectories maps.");
  }
  Vector3 start = vector3(root["start_position"], "start_position");
  if (!root["default_yaw"]) {
    throw std::invalid_argument("Trajectory YAML requires default_yaw.");
  }
  const double yaw = root["default_yaw"].as<double>();
  if (!std::isfinite(yaw)) {
    throw std::invalid_argument("Trajectory default_yaw must be finite.");
  }

  std::vector<std::string> names;
  if (trajectories[name]) {
    const YAML::Node definition = trajectories[name];
    const YAML::Node sequence = definition["segments"];
    if (!sequence || !sequence.IsSequence() || sequence.size() == 0) {
      throw std::invalid_argument("Trajectory '" + name + "' requires a nonempty segments list.");
    }
    if (definition["start_position"]) {
      start = vector3(definition["start_position"], name + " start_position");
    }
    for (const YAML::Node& entry : sequence) {
      names.push_back(entry.as<std::string>());
    }
  } else if (definitions[name]) {
    // Standalone reusable curved segments start at their prescribed world point.
    const YAML::Node d = definitions[name];
    require_fields(d, name);
    const std::string type = d["type"].as<std::string>();
    if (type == "figure_eight") {
      start = vector3(d["center"], name + " center");
    } else if (type == "circle" || type == "helix") {
      const Vector3 center = vector3(d["center"], name + " center");
      const double radius = d["radius"].as<double>();
      start = {center[0] + radius, center[1], center[2]};
    }
    names.push_back(name);
  } else {
    throw std::invalid_argument("Unknown trajectory or segment: " + name);
  }

  Vector3 cursor = start;
  double heading = yaw;
  std::vector<Segment> output;
  output.reserve(names.size());
  for (const std::string& segment_name : names) {
    output.push_back(compile_segment(definitions, segment_name, cursor, heading));
  }
  return {Sequence(std::move(output)), yaw, start};
}

} // namespace offboard_controllers::trajectory
