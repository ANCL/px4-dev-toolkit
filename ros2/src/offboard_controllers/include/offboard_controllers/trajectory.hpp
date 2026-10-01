#pragma once

#include <array>
#include <string>
#include <vector>

namespace offboard_controllers::trajectory
{

using Vector3 = std::array<double, 3>;

// References use PX4 local NED coordinates.
struct Reference
{
  Vector3 position{};
  Vector3 velocity{};
  Vector3 acceleration{};
  Vector3 jerk{};
  Vector3 snap{};

  double yaw{0.0};
  double yaw_rate{0.0};
  double yaw_acceleration{0.0};
};

Reference stationary_reference(
  const Vector3 & position,
  double yaw);

class Segment
{
public:
  static Segment hold(double duration_s);

  static Segment step(
    double duration_s,
    const Vector3 & offset);

  static Segment line(
    double duration_s,
    const Vector3 & offset);

  static Segment circle(
    double duration_s,
    double radius_m,
    double turns);

  static Segment helix(
    double duration_s,
    double radius_m,
    double height_m,
    double turns);

  static Segment yaw(
    double duration_s,
    double yaw_offset_rad);

  static Segment figure_eight(
    double duration_s,
    double x_amplitude_m,
    double y_amplitude_m,
    double turns);

  double duration_s() const;

  Reference sample(
    double time_s,
    const Reference & origin) const;

private:
  enum class Type
  {
    Hold,
    Step,
    Line,
    Circle,
    Helix,
    Yaw,
    FigureEight,
  };

  Segment(
    Type type,
    double duration_s,
    const Vector3 & offset,
    double radius_m,
    double height_m,
    double turns,
    double yaw_offset_rad,
    double x_amplitude_m,
    double y_amplitude_m);

  Type type_;
  double duration_s_;
  Vector3 offset_;
  double radius_m_;
  double height_m_;
  double turns_;
  double yaw_offset_rad_;
  double x_amplitude_m_;
  double y_amplitude_m_;
};


class Sequence
{
public:
  explicit Sequence(std::vector<Segment> segments);

  double duration_s() const;

  Reference sample(
    double time_s,
    const Reference & origin) const;

private:
  std::vector<Segment> segments_;
};


struct ConfiguredTrajectory
{
  Sequence sequence;
  double yaw;
};


ConfiguredTrajectory load_trajectory(
  const std::string & path,
  const std::string & name);

}  // namespace offboard_controllers::trajectory
