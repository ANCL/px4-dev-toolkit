/*
 * Analytic NED trajectory sampling through snap.
 * Ninth-order time scaling gives smooth endpoint derivatives; a Step remains
 * deliberately discontinuous.
 */

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <trajectory_generator/trajectory.hpp>

namespace trajectory_generator {

namespace {

constexpr double kPi = 3.14159265358979323846;

struct TimeScale {
  double s;
  double d1;
  double d2;
  double d3;
  double d4;
};

Vector3 add(const Vector3& a, const Vector3& b) {
  return {
      a[0] + b[0],
      a[1] + b[1],
      a[2] + b[2],
  };
}

Vector3 scale(const Vector3& value, double factor) {
  return {
      value[0] * factor,
      value[1] * factor,
      value[2] * factor,
  };
}

// Ninth-order C4 interpolation, u = clamp(t, 0, T)/T:
//   s(u) = 126u^5 - 420u^6 + 540u^7 - 315u^8 + 70u^9
// The first four time derivatives vanish at both endpoints.
TimeScale smooth_time_scale(double time_s, double duration_s) {
  const double t = std::clamp(time_s, 0.0, duration_s);

  const double u = t / duration_s;

  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  const double u6 = u5 * u;
  const double u7 = u6 * u;
  const double u8 = u7 * u;
  const double u9 = u8 * u;

  const double s = 126.0 * u5 - 420.0 * u6 + 540.0 * u7 - 315.0 * u8 + 70.0 * u9;

  const double du1 = 630.0 * u4 - 2520.0 * u5 + 3780.0 * u6 - 2520.0 * u7 + 630.0 * u8;

  const double du2 = 2520.0 * u3 - 12600.0 * u4 + 22680.0 * u5 - 17640.0 * u6 + 5040.0 * u7;

  const double du3 = 7560.0 * u2 - 50400.0 * u3 + 113400.0 * u4 - 105840.0 * u5 + 35280.0 * u6;

  const double du4 = 15120.0 * u - 151200.0 * u2 + 453600.0 * u3 - 529200.0 * u4 + 211680.0 * u5;

  const double inv_t = 1.0 / duration_s;
  const double inv_t2 = inv_t * inv_t;
  const double inv_t3 = inv_t2 * inv_t;
  const double inv_t4 = inv_t3 * inv_t;

  return {
      s, du1 * inv_t, du2 * inv_t2, du3 * inv_t3, du4 * inv_t4,
  };
}

struct ScalarDynamics {
  double value;
  double d1;
  double d2;
  double d3;
  double d4;
};

// Evaluate a sinusoid and its first four time derivatives by repeated chain
// rule. SE(3) attitude feed-forward needs trajectory derivatives through snap,
// so these are generated analytically rather than finite-differenced.
ScalarDynamics sinusoid_dynamics(double amplitude, double phase, double phase_d1, double phase_d2,
                                 double phase_d3, double phase_d4) {
  const double sine = std::sin(phase);
  const double cosine = std::cos(phase);

  return {
      amplitude * sine,
      amplitude * cosine * phase_d1,
      amplitude * (-sine * phase_d1 * phase_d1 + cosine * phase_d2),
      amplitude * (-cosine * phase_d1 * phase_d1 * phase_d1 - 3.0 * sine * phase_d1 * phase_d2 +
                   cosine * phase_d3),
      amplitude *
          (sine * phase_d1 * phase_d1 * phase_d1 * phase_d1 -
           6.0 * cosine * phase_d1 * phase_d1 * phase_d2 - 3.0 * sine * phase_d2 * phase_d2 -
           4.0 * sine * phase_d1 * phase_d3 + cosine * phase_d4),
  };
}

Reference sample_yaw(const Reference& origin, double yaw_offset_rad, const TimeScale& q) {
  Reference reference = stationary_reference(origin.position, origin.yaw + yaw_offset_rad * q.s);

  reference.yaw_rate = yaw_offset_rad * q.d1;

  reference.yaw_acceleration = yaw_offset_rad * q.d2;

  return reference;
}

Reference sample_figure_eight(const Reference& origin, double x_amplitude_m, double y_amplitude_m,
                              double turns, const TimeScale& q) {
  const double angle = 2.0 * kPi * turns * q.s;

  const double angle_d1 = 2.0 * kPi * turns * q.d1;

  const double angle_d2 = 2.0 * kPi * turns * q.d2;

  const double angle_d3 = 2.0 * kPi * turns * q.d3;

  const double angle_d4 = 2.0 * kPi * turns * q.d4;

  const ScalarDynamics x =
      sinusoid_dynamics(x_amplitude_m, angle, angle_d1, angle_d2, angle_d3, angle_d4);

  // y runs at twice the x phase, producing the planar 1:2 Lissajous
  // trajectory used for the configured figure-eight excitation.
  const ScalarDynamics y = sinusoid_dynamics(y_amplitude_m, 2.0 * angle, 2.0 * angle_d1,
                                             2.0 * angle_d2, 2.0 * angle_d3, 2.0 * angle_d4);

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

Reference sample_line(const Reference& origin, const Vector3& offset, const TimeScale& q) {
  Reference reference{};

  reference.position = add(origin.position, scale(offset, q.s));

  reference.velocity = scale(offset, q.d1);

  reference.acceleration = scale(offset, q.d2);

  reference.jerk = scale(offset, q.d3);

  reference.snap = scale(offset, q.d4);

  reference.yaw = origin.yaw;

  return reference;
}

Reference sample_circle(const Reference& origin, double radius_m, double height_m, double turns,
                        const TimeScale& q) {
  const double angle = 2.0 * kPi * turns * q.s;

  const double angle_d1 = 2.0 * kPi * turns * q.d1;

  const double angle_d2 = 2.0 * kPi * turns * q.d2;

  const double angle_d3 = 2.0 * kPi * turns * q.d3;

  const double angle_d4 = 2.0 * kPi * turns * q.d4;

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
      radius_m * (s * w1 * w1 * w1 - 3.0 * c * w1 * w2 - s * w3),
      radius_m * (-c * w1 * w1 * w1 - 3.0 * s * w1 * w2 + c * w3),
      height_m * q.d3,
  };

  reference.snap = {
      radius_m * (6.0 * s * w1 * w1 * w2 - s * w4 + c * w1 * w1 * w1 * w1 - 4.0 * c * w1 * w3 -
                  3.0 * c * w2 * w2),
      radius_m * (s * w1 * w1 * w1 * w1 - 4.0 * s * w1 * w3 - 3.0 * s * w2 * w2 -
                  6.0 * c * w1 * w1 * w2 + c * w4),
      height_m * q.d4,
  };

  reference.yaw = origin.yaw;

  return reference;
}

} // namespace

Reference Segment::sample(double time_s, const Reference& origin) const {
  if (!std::isfinite(time_s)) {
    throw std::invalid_argument("trajectory time must be finite.");
  }

  if (type_ == Type::Hold) {
    return stationary_reference(origin.position, origin.yaw);
  }

  // A step changes the position reference instantaneously at
  // segment start and then holds the new position.
  if (type_ == Type::Step) {
    return stationary_reference(add(origin.position, offset_), origin.yaw);
  }

  const TimeScale q = smooth_time_scale(time_s, duration_s_);

  if (type_ == Type::Line) {
    return sample_line(origin, offset_, q);
  }

  if (type_ == Type::Yaw) {
    return sample_yaw(origin, yaw_offset_rad_, q);
  }

  if (type_ == Type::FigureEight) {
    return sample_figure_eight(origin, x_amplitude_m_, y_amplitude_m_, turns_, q);
  }

  return sample_circle(origin, radius_m_, type_ == Type::Helix ? height_m_ : 0.0, turns_, q);
}

} // namespace trajectory_generator
