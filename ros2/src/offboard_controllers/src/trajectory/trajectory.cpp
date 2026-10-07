/*
 * Trajectory generation flow:
 *
 *   YAML configuration
 *     -> reusable Segment definitions
 *     -> analytic segment sampling through snap
 *     -> Sequence chaining through terminal references
 *     -> complete NED position/yaw reference
 *
 * Public reference/segment/sequence contracts live in trajectory.hpp. This
 * file documents the analytic primitives and configuration expansion.
 */

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include <offboard_controllers/trajectory/trajectory.hpp>
#include <yaml-cpp/yaml.h>

namespace offboard_controllers::trajectory
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

struct TimeScale
{
  double s;
  double d1;
  double d2;
  double d3;
  double d4;
};


void require_positive_finite(
  double value,
  const char * name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(
            std::string(name) + " must be finite and positive.");
  }
}


void require_finite(
  double value,
  const char * name)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(
            std::string(name) + " must be finite.");
  }
}


void require_finite(
  const Vector3 & value,
  const char * name)
{
  for (const double component : value) {
    if (!std::isfinite(component)) {
      throw std::invalid_argument(
              std::string(name) + " must be finite.");
    }
  }
}


Vector3 add(
  const Vector3 & a,
  const Vector3 & b)
{
  return {
    a[0] + b[0],
    a[1] + b[1],
    a[2] + b[2],
  };
}


Vector3 scale(
  const Vector3 & value,
  double factor)
{
  return {
    value[0] * factor,
    value[1] * factor,
    value[2] * factor,
  };
}


// Ninth-order smooth time scaling.
//
// The time-scaling derivatives through fourth order are zero at both
// segment boundaries. This keeps consecutive trajectory segments smooth
// through the derivatives needed by the later lower-level SE(3) handoffs.
TimeScale smooth_time_scale(
  double time_s,
  double duration_s)
{
  const double t =
    std::clamp(time_s, 0.0, duration_s);

  const double u = t / duration_s;

  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  const double u6 = u5 * u;
  const double u7 = u6 * u;
  const double u8 = u7 * u;
  const double u9 = u8 * u;

  const double s =
    126.0 * u5 -
    420.0 * u6 +
    540.0 * u7 -
    315.0 * u8 +
    70.0 * u9;

  const double du1 =
    630.0 * u4 -
    2520.0 * u5 +
    3780.0 * u6 -
    2520.0 * u7 +
    630.0 * u8;

  const double du2 =
    2520.0 * u3 -
    12600.0 * u4 +
    22680.0 * u5 -
    17640.0 * u6 +
    5040.0 * u7;

  const double du3 =
    7560.0 * u2 -
    50400.0 * u3 +
    113400.0 * u4 -
    105840.0 * u5 +
    35280.0 * u6;

  const double du4 =
    15120.0 * u -
    151200.0 * u2 +
    453600.0 * u3 -
    529200.0 * u4 +
    211680.0 * u5;

  const double inv_t = 1.0 / duration_s;
  const double inv_t2 = inv_t * inv_t;
  const double inv_t3 = inv_t2 * inv_t;
  const double inv_t4 = inv_t3 * inv_t;

  return {
    s,
    du1 * inv_t,
    du2 * inv_t2,
    du3 * inv_t3,
    du4 * inv_t4,
  };
}


struct ScalarDynamics
{
  double value;
  double d1;
  double d2;
  double d3;
  double d4;
};


// Evaluate a sinusoid and its first four time derivatives by repeated chain
// rule. SE(3) attitude feed-forward needs trajectory derivatives through snap,
// so these are generated analytically rather than finite-differenced.
ScalarDynamics sinusoid_dynamics(
  double amplitude,
  double phase,
  double phase_d1,
  double phase_d2,
  double phase_d3,
  double phase_d4)
{
  const double sine = std::sin(phase);
  const double cosine = std::cos(phase);

  return {
    amplitude * sine,
    amplitude * cosine * phase_d1,
    amplitude * (-sine * phase_d1 * phase_d1 + cosine * phase_d2),
    amplitude * (
      -cosine * phase_d1 * phase_d1 * phase_d1 -
      3.0 * sine * phase_d1 * phase_d2 +
      cosine * phase_d3),
    amplitude * (
      sine * phase_d1 * phase_d1 * phase_d1 * phase_d1 -
      6.0 * cosine * phase_d1 * phase_d1 * phase_d2 -
      3.0 * sine * phase_d2 * phase_d2 -
      4.0 * sine * phase_d1 * phase_d3 +
      cosine * phase_d4),
  };
}


Reference sample_yaw(
  const Reference & origin,
  double yaw_offset_rad,
  const TimeScale & q)
{
  Reference reference =
    stationary_reference(
      origin.position,
      origin.yaw + yaw_offset_rad * q.s);

  reference.yaw_rate =
    yaw_offset_rad * q.d1;

  reference.yaw_acceleration =
    yaw_offset_rad * q.d2;

  return reference;
}


Reference sample_figure_eight(
  const Reference & origin,
  double x_amplitude_m,
  double y_amplitude_m,
  double turns,
  const TimeScale & q)
{
  const double angle =
    2.0 * kPi * turns * q.s;

  const double angle_d1 =
    2.0 * kPi * turns * q.d1;

  const double angle_d2 =
    2.0 * kPi * turns * q.d2;

  const double angle_d3 =
    2.0 * kPi * turns * q.d3;

  const double angle_d4 =
    2.0 * kPi * turns * q.d4;

  const ScalarDynamics x =
    sinusoid_dynamics(
      x_amplitude_m,
      angle,
      angle_d1,
      angle_d2,
      angle_d3,
      angle_d4);

  // y runs at twice the x phase, producing the planar 1:2 Lissajous
  // trajectory used for the configured figure-eight excitation.
  const ScalarDynamics y =
    sinusoid_dynamics(
      y_amplitude_m,
      2.0 * angle,
      2.0 * angle_d1,
      2.0 * angle_d2,
      2.0 * angle_d3,
      2.0 * angle_d4);

  Reference reference{};

  reference.position = {
    origin.position[0] + x.value,
    origin.position[1] + y.value,
    origin.position[2],
  };

  reference.velocity = {x.d1, y.d1, 0.0};
  reference.acceleration = {x.d2, y.d2, 0.0};
  reference.jerk = {x.d3, y.d3, 0.0};
  reference.snap = {x.d4, y.d4, 0.0};
  reference.yaw = origin.yaw;

  return reference;
}


Reference sample_line(
  const Reference & origin,
  const Vector3 & offset,
  const TimeScale & q)
{
  Reference reference{};

  reference.position =
    add(origin.position, scale(offset, q.s));

  reference.velocity =
    scale(offset, q.d1);

  reference.acceleration =
    scale(offset, q.d2);

  reference.jerk =
    scale(offset, q.d3);

  reference.snap =
    scale(offset, q.d4);

  reference.yaw = origin.yaw;

  return reference;
}


Reference sample_circle(
  const Reference & origin,
  double radius_m,
  double height_m,
  double turns,
  const TimeScale & q)
{
  const double angle =
    2.0 * kPi * turns * q.s;

  const double angle_d1 =
    2.0 * kPi * turns * q.d1;

  const double angle_d2 =
    2.0 * kPi * turns * q.d2;

  const double angle_d3 =
    2.0 * kPi * turns * q.d3;

  const double angle_d4 =
    2.0 * kPi * turns * q.d4;

  const double c = std::cos(angle);
  const double s = std::sin(angle);

  const double w1 = angle_d1;
  const double w2 = angle_d2;
  const double w3 = angle_d3;
  const double w4 = angle_d4;

  Reference reference{};

  // The segment starts at origin.position. The circle center lies one radius
  // in negative local-NED x, so no position jump occurs at segment entry.
  reference.position = {
    origin.position[0] + radius_m * (c - 1.0),
    origin.position[1] + radius_m * s,
    origin.position[2] + height_m * q.s,
  };

  reference.velocity = {
    -radius_m * s * w1,
    radius_m * c * w1,
    height_m * q.d1,
  };

  reference.acceleration = {
    -radius_m * (c * w1 * w1 + s * w2),
    radius_m * (-s * w1 * w1 + c * w2),
    height_m * q.d2,
  };

  reference.jerk = {
    radius_m * (
      s * w1 * w1 * w1 -
      3.0 * c * w1 * w2 -
      s * w3),
    radius_m * (
      -c * w1 * w1 * w1 -
      3.0 * s * w1 * w2 +
      c * w3),
    height_m * q.d3,
  };

  reference.snap = {
    radius_m * (
      6.0 * s * w1 * w1 * w2 -
      s * w4 +
      c * w1 * w1 * w1 * w1 -
      4.0 * c * w1 * w3 -
      3.0 * c * w2 * w2),
    radius_m * (
      s * w1 * w1 * w1 * w1 -
      4.0 * s * w1 * w3 -
      3.0 * s * w2 * w2 -
      6.0 * c * w1 * w1 * w2 +
      c * w4),
    height_m * q.d4,
  };

  reference.yaw = origin.yaw;

  return reference;
}

}  // namespace


Reference stationary_reference(
  const Vector3 & position,
  double yaw)
{
  require_finite(position, "trajectory position");
  require_finite(yaw, "trajectory yaw");

  Reference reference{};
  reference.position = position;
  reference.yaw = yaw;

  return reference;
}


Segment::Segment(
  Type type,
  double duration_s,
  const Vector3 & offset,
  double radius_m,
  double height_m,
  double turns,
  double yaw_offset_rad,
  double x_amplitude_m,
  double y_amplitude_m)
: type_(type),
  duration_s_(duration_s),
  offset_(offset),
  radius_m_(radius_m),
  height_m_(height_m),
  turns_(turns),
  yaw_offset_rad_(yaw_offset_rad),
  x_amplitude_m_(x_amplitude_m),
  y_amplitude_m_(y_amplitude_m)
{
  require_positive_finite(
    duration_s_,
    "trajectory segment duration");

  require_finite(
    offset_,
    "trajectory segment offset");

  require_finite(
    radius_m_,
    "trajectory radius");

  require_finite(
    height_m_,
    "trajectory height");

  require_finite(
    turns_,
    "trajectory turns");

  require_finite(
    yaw_offset_rad_,
    "trajectory yaw offset");

  require_finite(
    x_amplitude_m_,
    "trajectory x amplitude");

  require_finite(
    y_amplitude_m_,
    "trajectory y amplitude");
}


Segment Segment::hold(double duration_s)
{
  return Segment(
    Type::Hold,
    duration_s,
    {},
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0);
}


Segment Segment::line(
  double duration_s,
  const Vector3 & offset)
{
  return Segment(
    Type::Line,
    duration_s,
    offset,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0);
}


Segment Segment::step(
  double duration_s,
  const Vector3 & offset)
{
  return Segment(
    Type::Step,
    duration_s,
    offset,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0);
}


Segment Segment::circle(
  double duration_s,
  double radius_m,
  double turns)
{
  require_positive_finite(
    radius_m,
    "trajectory radius");

  return Segment(
    Type::Circle,
    duration_s,
    {},
    radius_m,
    0.0,
    turns,
    0.0,
    0.0,
    0.0);
}


Segment Segment::helix(
  double duration_s,
  double radius_m,
  double height_m,
  double turns)
{
  require_positive_finite(
    radius_m,
    "trajectory radius");

  return Segment(
    Type::Helix,
    duration_s,
    {},
    radius_m,
    height_m,
    turns,
    0.0,
    0.0,
    0.0);
}


Segment Segment::yaw(
  double duration_s,
  double yaw_offset_rad)
{
  require_finite(
    yaw_offset_rad,
    "trajectory yaw offset");

  return Segment(
    Type::Yaw,
    duration_s,
    {},
    0.0,
    0.0,
    0.0,
    yaw_offset_rad,
    0.0,
    0.0);
}


Segment Segment::figure_eight(
  double duration_s,
  double x_amplitude_m,
  double y_amplitude_m,
  double turns)
{
  require_positive_finite(
    x_amplitude_m,
    "trajectory x amplitude");

  require_positive_finite(
    y_amplitude_m,
    "trajectory y amplitude");

  require_positive_finite(
    turns,
    "trajectory turns");

  return Segment(
    Type::FigureEight,
    duration_s,
    {},
    0.0,
    0.0,
    turns,
    0.0,
    x_amplitude_m,
    y_amplitude_m);
}


double Segment::duration_s() const
{
  return duration_s_;
}


Reference Segment::sample(
  double time_s,
  const Reference & origin) const
{
  require_finite(time_s, "trajectory time");

  if (type_ == Type::Hold) {
    return stationary_reference(
      origin.position,
      origin.yaw);
  }

  // A step changes the position reference instantaneously at
  // segment start and then holds the new position.
  if (type_ == Type::Step) {
    return stationary_reference(
      add(origin.position, offset_),
      origin.yaw);
  }

  const TimeScale q =
    smooth_time_scale(
      time_s,
      duration_s_);

  if (type_ == Type::Line) {
    return sample_line(
      origin,
      offset_,
      q);
  }

  if (type_ == Type::Yaw) {
    return sample_yaw(
      origin,
      yaw_offset_rad_,
      q);
  }

  if (type_ == Type::FigureEight) {
    return sample_figure_eight(
      origin,
      x_amplitude_m_,
      y_amplitude_m_,
      turns_,
      q);
  }

  return sample_circle(
    origin,
    radius_m_,
    type_ == Type::Helix ? height_m_ : 0.0,
    turns_,
    q);
}


Sequence::Sequence(
  std::vector<Segment> segments)
: segments_(std::move(segments))
{
  if (segments_.empty()) {
    throw std::invalid_argument(
            "trajectory sequence must contain at least one segment.");
  }
}


double Sequence::duration_s() const
{
  double total = 0.0;

  for (const Segment & segment : segments_) {
    total += segment.duration_s();
  }

  return total;
}


Reference Sequence::sample(
  double time_s,
  const Reference & origin) const
{
  require_finite(time_s, "trajectory time");

  Reference segment_origin =
    stationary_reference(
      origin.position,
      origin.yaw);

  double remaining =
    std::max(0.0, time_s);

  for (const Segment & segment : segments_) {
    if (remaining <= segment.duration_s()) {
      return segment.sample(
        remaining,
        segment_origin);
    }

    // The terminal reference of each completed segment becomes the origin of
    // the next segment. This preserves position/yaw continuity for reusable
    // relative segments without accumulating state outside the sequence.
    segment_origin =
      segment.sample(
        segment.duration_s(),
        segment_origin);

    remaining -= segment.duration_s();
  }

  // Once all segments have completed, hold the final reference indefinitely.
  return stationary_reference(
    segment_origin.position,
    segment_origin.yaw);
}


namespace
{

Vector3 read_vector3(
  const YAML::Node & node,
  const std::string & name)
{
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::invalid_argument(
            name + " must contain exactly three values.");
  }

  const Vector3 value{
    node[0].as<double>(),
    node[1].as<double>(),
    node[2].as<double>(),
  };

  require_finite(value, name.c_str());

  return value;
}


Segment read_segment(
  const YAML::Node & segments,
  const std::string & name)
{
  const YAML::Node definition = segments[name];

  if (!definition || !definition.IsMap()) {
    throw std::invalid_argument(
            "Unknown trajectory segment: " + name);
  }

  if (!definition["type"] || !definition["duration"]) {
    throw std::invalid_argument(
            "Trajectory segment '" + name +
            "' requires type and duration.");
  }

  const std::string type =
    definition["type"].as<std::string>();

  const double duration =
    definition["duration"].as<double>();

  if (type == "hold") {
    return Segment::hold(duration);
  }

  if (type == "step") {
    return Segment::step(
      duration,
      read_vector3(
        definition["offset"],
        "trajectory step offset"));
  }

  if (type == "line") {
    return Segment::line(
      duration,
      read_vector3(
        definition["offset"],
        "trajectory line offset"));
  }

  if (type == "circle") {
    return Segment::circle(
      duration,
      definition["radius"].as<double>(),
      definition["turns"].as<double>());
  }

  if (type == "helix") {
    return Segment::helix(
      duration,
      definition["radius"].as<double>(),
      definition["height"].as<double>(),
      definition["turns"].as<double>());
  }

  if (type == "yaw") {
    return Segment::yaw(
      duration,
      definition["yaw_offset"].as<double>());
  }

  if (type == "figure_eight") {
    return Segment::figure_eight(
      duration,
      definition["x_amplitude"].as<double>(),
      definition["y_amplitude"].as<double>(),
      definition["turns"].as<double>());
  }

  throw std::invalid_argument(
          "Unsupported trajectory type '" +
          type +
          "' in segment '" +
          name +
          "'.");
}

}  // namespace


ConfiguredTrajectory load_trajectory(
  const std::string & path,
  const std::string & name)
{
  YAML::Node root;

  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(
            "Unable to load trajectory configuration '" +
            path +
            "': " +
            error.what());
  }

  const YAML::Node segments =
    root["segments"];

  const YAML::Node trajectories =
    root["trajectories"];

  if (!segments || !segments.IsMap()) {
    throw std::invalid_argument(
            "Trajectory configuration requires a segments map.");
  }

  if (!trajectories || !trajectories.IsMap()) {
    throw std::invalid_argument(
            "Trajectory configuration requires a trajectories map.");
  }

  if (!root["default_yaw"]) {
    throw std::invalid_argument(
            "Trajectory configuration requires default_yaw.");
  }

  const double yaw =
    root["default_yaw"].as<double>();

  require_finite(
    yaw,
    "trajectory yaw");

  std::vector<Segment> configured_segments;

  // A configured trajectory expands reusable motion segments.
  if (trajectories[name]) {
    const YAML::Node definition =
      trajectories[name];

    const YAML::Node names =
      definition["segments"];

    if (!names || !names.IsSequence() || names.size() == 0) {
      throw std::invalid_argument(
              "Trajectory '" +
              name +
              "' must contain segments.");
    }

    configured_segments.reserve(names.size());

    for (const YAML::Node & segment_name : names) {
      configured_segments.push_back(
        read_segment(
          segments,
          segment_name.as<std::string>()));
    }

    return {
      Sequence(std::move(configured_segments)),
      yaw,
    };
  }

  // A single reusable segment may also be selected directly.
  if (segments[name]) {
    configured_segments.push_back(
      read_segment(
        segments,
        name));

    return {
      Sequence(std::move(configured_segments)),
      yaw,
    };
  }

  throw std::invalid_argument(
          "Unknown trajectory or segment: " +
          name);
}

}  // namespace offboard_controllers::trajectory
