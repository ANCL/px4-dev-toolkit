/*
 * Trajectory segment construction, validation, and sequence execution.
 * At an exact boundary the ending segment owns the sample; each completed
 * segment's final position/yaw anchors the next segment. After completion
 * the sequence holds the final position and heading indefinitely.
 */

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include <trajectory_generator/trajectory.hpp>

namespace trajectory_generator {

namespace {

void require_positive_finite(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and positive.");
  }
}

void require_finite(double value, const char* name) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(name) + " must be finite.");
  }
}

void require_finite(const Vector3& value, const char* name) {
  for (const double component : value) {
    if (!std::isfinite(component)) {
      throw std::invalid_argument(std::string(name) + " must be finite.");
    }
  }
}

} // namespace

Reference stationary_reference(const Vector3& position, double yaw) {
  require_finite(position, "trajectory position");
  require_finite(yaw, "trajectory yaw");

  Reference reference{};
  reference.position = position;
  reference.yaw = yaw;

  return reference;
}

Segment::Segment(Type type, double duration_s, const Vector3& offset, double radius_m,
                 double height_m, double turns, double yaw_offset_rad, double x_amplitude_m,
                 double y_amplitude_m)
    : type_(type), duration_s_(duration_s), offset_(offset), radius_m_(radius_m),
      height_m_(height_m), turns_(turns), yaw_offset_rad_(yaw_offset_rad),
      x_amplitude_m_(x_amplitude_m), y_amplitude_m_(y_amplitude_m) {
  require_positive_finite(duration_s_, "trajectory segment duration");

  require_finite(offset_, "trajectory segment offset");

  require_finite(radius_m_, "trajectory radius");

  require_finite(height_m_, "trajectory height");

  require_finite(turns_, "trajectory turns");

  require_finite(yaw_offset_rad_, "trajectory yaw offset");

  require_finite(x_amplitude_m_, "trajectory x amplitude");

  require_finite(y_amplitude_m_, "trajectory y amplitude");
}

Segment Segment::hold(double duration_s) {
  return Segment(Type::Hold, duration_s, {}, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

Segment Segment::line(double duration_s, const Vector3& offset) {
  return Segment(Type::Line, duration_s, offset, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

Segment Segment::step(double duration_s, const Vector3& offset) {
  return Segment(Type::Step, duration_s, offset, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

Segment Segment::circle(double duration_s, double radius_m, double turns) {
  require_positive_finite(radius_m, "trajectory radius");

  return Segment(Type::Circle, duration_s, {}, radius_m, 0.0, turns, 0.0, 0.0, 0.0);
}

Segment Segment::helix(double duration_s, double radius_m, double height_m, double turns) {
  require_positive_finite(radius_m, "trajectory radius");

  return Segment(Type::Helix, duration_s, {}, radius_m, height_m, turns, 0.0, 0.0, 0.0);
}

Segment Segment::yaw(double duration_s, double yaw_offset_rad) {
  require_finite(yaw_offset_rad, "trajectory yaw offset");

  return Segment(Type::Yaw, duration_s, {}, 0.0, 0.0, 0.0, yaw_offset_rad, 0.0, 0.0);
}

Segment Segment::figure_eight(double duration_s, double x_amplitude_m, double y_amplitude_m,
                              double turns) {
  require_positive_finite(x_amplitude_m, "trajectory x amplitude");

  require_positive_finite(y_amplitude_m, "trajectory y amplitude");

  require_positive_finite(turns, "trajectory turns");

  return Segment(Type::FigureEight, duration_s, {}, 0.0, 0.0, turns, 0.0, x_amplitude_m,
                 y_amplitude_m);
}

double Segment::duration_s() const { return duration_s_; }

Sequence::Sequence(std::vector<Segment> segments) : segments_(std::move(segments)) {
  if (segments_.empty()) {
    throw std::invalid_argument("trajectory sequence must contain at least one segment.");
  }
}

double Sequence::duration_s() const {
  double total = 0.0;

  for (const Segment& segment : segments_) {
    total += segment.duration_s();
  }

  return total;
}

Reference Sequence::sample(double time_s, const Reference& origin) const {
  require_finite(time_s, "trajectory time");

  Reference segment_origin = stationary_reference(origin.position, origin.yaw);

  double remaining = std::max(0.0, time_s);

  for (const Segment& segment : segments_) {
    if (remaining <= segment.duration_s()) {
      return segment.sample(remaining, segment_origin);
    }

    // The terminal reference of each completed segment becomes the origin of
    // the next segment. This preserves position/yaw continuity for reusable
    // relative segments without accumulating state outside the sequence.
    segment_origin = segment.sample(segment.duration_s(), segment_origin);

    remaining -= segment.duration_s();
  }

  // Once all segments have completed, hold the final reference indefinitely.
  return stationary_reference(segment_origin.position, segment_origin.yaw);
}

} // namespace trajectory_generator
