// Parse named trajectories and reusable segments from YAML.
// Numerical sampling and sequence execution are implemented separately.

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <offboard_controllers/trajectory/trajectory.hpp>
#include <yaml-cpp/yaml.h>

namespace offboard_controllers::trajectory {

namespace {

Vector3 read_vector3(const YAML::Node& node, const std::string& name) {
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::invalid_argument(name + " must contain exactly three values.");
  }

  const Vector3 value{
      node[0].as<double>(),
      node[1].as<double>(),
      node[2].as<double>(),
  };

  for (const double component : value) {
    if (!std::isfinite(component)) {
      throw std::invalid_argument(name + " must be finite.");
    }
  }

  return value;
}

Segment read_segment(const YAML::Node& segments, const std::string& name) {
  const YAML::Node definition = segments[name];

  if (!definition || !definition.IsMap()) {
    throw std::invalid_argument("Unknown trajectory segment: " + name);
  }

  if (!definition["type"] || !definition["duration"]) {
    throw std::invalid_argument("Trajectory segment '" + name + "' requires type and duration.");
  }

  const std::string type = definition["type"].as<std::string>();

  const double duration = definition["duration"].as<double>();

  if (type == "hold") {
    return Segment::hold(duration);
  }

  if (type == "step") {
    return Segment::step(duration, read_vector3(definition["offset"], "trajectory step offset"));
  }

  if (type == "line") {
    return Segment::line(duration, read_vector3(definition["offset"], "trajectory line offset"));
  }

  if (type == "circle") {
    return Segment::circle(duration, definition["radius"].as<double>(),
                           definition["turns"].as<double>());
  }

  if (type == "helix") {
    return Segment::helix(duration, definition["radius"].as<double>(),
                          definition["height"].as<double>(), definition["turns"].as<double>());
  }

  if (type == "yaw") {
    return Segment::yaw(duration, definition["yaw_offset"].as<double>());
  }

  if (type == "figure_eight") {
    return Segment::figure_eight(duration, definition["x_amplitude"].as<double>(),
                                 definition["y_amplitude"].as<double>(),
                                 definition["turns"].as<double>());
  }

  throw std::invalid_argument("Unsupported trajectory type '" + type + "' in segment '" + name +
                              "'.");
}

} // namespace

ConfiguredTrajectory load_trajectory(const std::string& path, const std::string& name) {
  YAML::Node root;

  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& error) {
    throw std::invalid_argument("Unable to load trajectory configuration '" + path +
                                "': " + error.what());
  }

  const YAML::Node segments = root["segments"];

  const YAML::Node trajectories = root["trajectories"];

  if (!segments || !segments.IsMap()) {
    throw std::invalid_argument("Trajectory configuration requires a segments map.");
  }

  if (!trajectories || !trajectories.IsMap()) {
    throw std::invalid_argument("Trajectory configuration requires a trajectories map.");
  }

  if (!root["default_yaw"]) {
    throw std::invalid_argument("Trajectory configuration requires default_yaw.");
  }

  const double yaw = root["default_yaw"].as<double>();

  if (!std::isfinite(yaw)) {
    throw std::invalid_argument("trajectory yaw must be finite.");
  }

  std::vector<Segment> configured_segments;

  // A configured trajectory expands reusable motion segments.
  if (trajectories[name]) {
    const YAML::Node definition = trajectories[name];

    const YAML::Node names = definition["segments"];

    if (!names || !names.IsSequence() || names.size() == 0) {
      throw std::invalid_argument("Trajectory '" + name + "' must contain segments.");
    }

    configured_segments.reserve(names.size());

    for (const YAML::Node& segment_name : names) {
      configured_segments.push_back(read_segment(segments, segment_name.as<std::string>()));
    }

    return {
        Sequence(std::move(configured_segments)),
        yaw,
    };
  }

  // A single reusable segment may also be selected directly.
  if (segments[name]) {
    configured_segments.push_back(read_segment(segments, name));

    return {
        Sequence(std::move(configured_segments)),
        yaw,
    };
  }

  throw std::invalid_argument("Unknown trajectory or segment: " + name);
}

} // namespace offboard_controllers::trajectory
