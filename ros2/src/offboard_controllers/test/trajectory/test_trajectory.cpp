#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

#include <offboard_controllers/trajectory/trajectory.hpp>
#include <yaml-cpp/yaml.h>

namespace trajectory = offboard_controllers::trajectory;

namespace {

constexpr double kTolerance = 1.0e-8;

void require_near(double actual, double expected, const char* label,
                  double tolerance = kTolerance) {
  if (!std::isfinite(actual) || !std::isfinite(expected) ||
      std::abs(actual - expected) > tolerance) {
    std::cerr << label << ": expected " << expected << ", got " << actual << '\n';

    std::exit(EXIT_FAILURE);
  }
}

void require_vector(const trajectory::Vector3& actual, const trajectory::Vector3& expected,
                    const char* label) {
  for (std::size_t i = 0; i < 3; ++i) {
    require_near(actual[i], expected[i], label);
  }
}

void require_stationary(const trajectory::Reference& reference) {
  const trajectory::Vector3 zero{0.0, 0.0, 0.0};

  require_vector(reference.velocity, zero, "velocity");
  require_vector(reference.acceleration, zero, "acceleration");
  require_vector(reference.jerk, zero, "jerk");
  require_vector(reference.snap, zero, "snap");
}

void test_hold() {
  const auto origin = trajectory::stationary_reference({1.0, 2.0, -2.0}, 0.3);

  const trajectory::Sequence sequence({
      trajectory::Segment::hold(2.0),
  });

  const auto reference = sequence.sample(1.0, origin);

  require_vector(reference.position, origin.position, "hold position");

  require_near(reference.yaw, 0.3, "hold yaw");

  require_stationary(reference);
}

void test_step_response() {
  const auto origin = trajectory::stationary_reference({0.0, 0.0, -2.0}, 0.0);

  const trajectory::Sequence sequence({
      trajectory::Segment::hold(1.0),
      trajectory::Segment::step(4.0, {1.0, 0.0, 0.0}),
  });

  // Invariant:
  //   Step segments are right-discontinuous: the preceding segment owns the
  //   exact boundary, and the offset appears immediately afterward.
  const auto before_step = sequence.sample(1.0, origin);

  const auto after_step = sequence.sample(1.0 + 1.0e-6, origin);

  const auto final = sequence.sample(10.0, origin);

  require_vector(before_step.position, {0.0, 0.0, -2.0}, "step before");

  require_vector(after_step.position, {1.0, 0.0, -2.0}, "step after");

  require_vector(final.position, {1.0, 0.0, -2.0}, "step final");

  require_stationary(after_step);
  require_stationary(final);
}

void test_line() {
  const auto origin = trajectory::stationary_reference({0.0, 0.0, -2.0}, 0.0);

  const trajectory::Sequence sequence({
      trajectory::Segment::line(2.0, {2.0, -1.0, 0.5}),
  });

  const auto start = sequence.sample(0.0, origin);

  const auto middle = sequence.sample(1.0, origin);

  const auto end = sequence.sample(2.0, origin);

  require_vector(start.position, {0.0, 0.0, -2.0}, "line start");

  require_vector(middle.position, {1.0, -0.5, -1.75}, "line midpoint");

  require_vector(end.position, {2.0, -1.0, -1.5}, "line end");

  require_stationary(start);
  require_stationary(end);
}

void test_circle() {
  const auto origin = trajectory::stationary_reference({0.0, 0.0, -2.0}, 0.0);

  const trajectory::Sequence sequence({
      trajectory::Segment::circle(4.0, 1.0, 1.0),
  });

  const auto start = sequence.sample(0.0, origin);

  const auto middle = sequence.sample(2.0, origin);

  const auto end = sequence.sample(4.0, origin);

  require_vector(start.position, {0.0, 0.0, -2.0}, "circle start");

  require_vector(middle.position, {-2.0, 0.0, -2.0}, "circle midpoint");

  require_vector(end.position, {0.0, 0.0, -2.0}, "circle end");

  require_stationary(start);
  require_stationary(end);
}

void test_helix() {
  const auto origin = trajectory::stationary_reference({0.0, 0.0, -2.0}, 0.0);

  const trajectory::Sequence sequence({
      trajectory::Segment::helix(4.0, 1.0, -1.0, 1.0),
  });

  const auto end = sequence.sample(4.0, origin);

  require_vector(end.position, {0.0, 0.0, -3.0}, "helix end");

  require_stationary(end);
}

void test_yaw_sweep() {
  const auto origin = trajectory::stationary_reference({0.0, 0.0, -2.0}, 0.25);

  const trajectory::Sequence sequence({
      trajectory::Segment::yaw(4.0, 1.0),
  });

  const auto start = sequence.sample(0.0, origin);

  const auto middle = sequence.sample(2.0, origin);

  const auto end = sequence.sample(4.0, origin);

  require_near(start.yaw, 0.25, "yaw start");
  require_near(middle.yaw, 0.75, "yaw midpoint");
  require_near(end.yaw, 1.25, "yaw end");

  require_near(start.yaw_rate, 0.0, "yaw start rate");
  require_near(start.yaw_acceleration, 0.0, "yaw start acceleration");
  require_near(end.yaw_rate, 0.0, "yaw end rate");
  require_near(end.yaw_acceleration, 0.0, "yaw end acceleration");

  require_stationary(start);
  require_stationary(end);
}

void test_figure_eight() {
  const auto origin = trajectory::stationary_reference({1.0, -2.0, -3.0}, 0.4);

  const trajectory::Sequence sequence({
      trajectory::Segment::figure_eight(20.0, 2.0, 1.0, 1.0),
  });

  const auto start = sequence.sample(0.0, origin);

  const auto middle = sequence.sample(10.0, origin);

  const auto end = sequence.sample(20.0, origin);

  // Invariant:
  //   The 1:2 Lissajous figure eight is closed: start, midpoint, and end cross
  //   the origin while the midpoint retains non-zero velocity.
  require_vector(start.position, origin.position, "figure-eight start");
  require_vector(middle.position, origin.position, "figure-eight midpoint");
  require_vector(end.position, origin.position, "figure-eight end");

  if (!(std::abs(middle.velocity[0]) > 0.1 && std::abs(middle.velocity[1]) > 0.1)) {
    std::cerr << "figure-eight midpoint should have non-zero planar velocity\n";
    std::exit(EXIT_FAILURE);
  }

  require_near(middle.velocity[0] + middle.velocity[1], 0.0,
               "figure-eight midpoint velocity symmetry");
  require_near(middle.position[2], origin.position[2], "figure-eight altitude");
  require_near(middle.yaw, origin.yaw, "figure-eight yaw");

  require_stationary(start);
  require_stationary(end);
}

void test_sequence() {
  const auto origin = trajectory::stationary_reference({0.0, 0.0, -2.0}, 0.0);

  const trajectory::Sequence sequence({
      trajectory::Segment::hold(1.0),
      trajectory::Segment::line(2.0, {2.0, 0.0, 0.0}),
      trajectory::Segment::hold(1.0),
  });

  require_near(sequence.duration_s(), 4.0, "sequence duration");

  const auto final_reference = sequence.sample(10.0, origin);

  require_vector(final_reference.position, {2.0, 0.0, -2.0}, "sequence final position");

  require_stationary(final_reference);
}

// Editable experiment YAML is not a fixed numeric reference trajectory.
// Verify every configured segment and sequence can load and sample safely.
void test_editable_configuration() {
  const YAML::Node yaml = YAML::LoadFile(TRAJECTORY_CONFIG_PATH);
  const YAML::Node segments = yaml["segments"];
  const YAML::Node trajectories = yaml["trajectories"];
  if (!segments.IsMap() || segments.size() == 0 ||
      !trajectories.IsMap() || trajectories.size() == 0) {
    throw std::runtime_error("Trajectory YAML must define segments and trajectories.");
  }

  const auto assert_finite = [](const trajectory::Reference& r) {
    for (const auto& vector : {r.position, r.velocity, r.acceleration, r.jerk, r.snap}) {
      for (double component : vector) {
        if (!std::isfinite(component)) {
          throw std::runtime_error("Configured trajectory produced nonfinite vector data.");
        }
      }
    }
    if (!std::isfinite(r.yaw) || !std::isfinite(r.yaw_rate) ||
        !std::isfinite(r.yaw_acceleration)) {
      throw std::runtime_error("Configured trajectory produced nonfinite yaw data.");
    }
  };
  const auto check = [&](const YAML::Node& definitions) {
    for (const auto& item : definitions) {
      const auto name = item.first.as<std::string>();
      const auto configured = trajectory::load_trajectory(TRAJECTORY_CONFIG_PATH, name);
      const double duration = configured.sequence.duration_s();
      if (!std::isfinite(duration) || duration <= 0 || !std::isfinite(configured.yaw)) {
        throw std::runtime_error("Invalid configured trajectory duration or heading: " + name);
      }
      const auto start = trajectory::stationary_reference(
          configured.start_position, configured.yaw);
      assert_finite(configured.sequence.sample(0.0, start));
      assert_finite(configured.sequence.sample(duration / 2.0, start));
      assert_finite(configured.sequence.sample(duration, start));
    }
  };
  check(segments);
  check(trajectories);

  // Verify each configured step/line ends at its YAML NED target.
  // Expected values come from editable YAML, not hard-coded flight scenarios.
  for (const auto& item : trajectories) {
    const auto configured = trajectory::load_trajectory(
        TRAJECTORY_CONFIG_PATH, item.first.as<std::string>());
    const auto origin = trajectory::stationary_reference(
        configured.start_position, configured.yaw);
    double end_s = 0.0;
    for (const auto& name_node : item.second["segments"]) {
      const auto definition = segments[name_node.as<std::string>()];
      end_s += definition["duration"].as<double>();
      if (definition["position"]) {
        const auto position = definition["position"];
        const trajectory::Vector3 expected{
            position[0].as<double>(), position[1].as<double>(), position[2].as<double>()};
        require_vector(configured.sequence.sample(end_s, origin).position,
                       expected, "absolute commanded NED position");
      }
    }
  }

}

void test_analytic_segment_derivatives() {
  const auto origin = trajectory::stationary_reference({1.0, -0.5, -2.0}, 0.25);
  const std::array<trajectory::Segment, 5> segments{
      trajectory::Segment::line(4.0, {1.0, -0.5, 0.3}),
      trajectory::Segment::circle(4.0, 1.0, 1.0),
      trajectory::Segment::helix(4.0, 0.8, -0.6, 1.0),
      trajectory::Segment::yaw(4.0, 0.8),
      trajectory::Segment::figure_eight(4.0, 1.2, 0.6, 1.0),
  };

  // Compare analytic derivatives against independent centered differences.
  // Evaluate within a segment, avoiding the intentional Step discontinuity.
  constexpr double t = 1.6;
  constexpr double h = 1.0e-4;

  for (const auto& segment : segments) {
    require_stationary(segment.sample(0.0, origin));
    require_stationary(segment.sample(segment.duration_s(), origin));

    const auto before = segment.sample(t - h, origin);
    const auto at = segment.sample(t, origin);
    const auto after = segment.sample(t + h, origin);

    for (std::size_t axis = 0; axis < 3; ++axis) {
      require_near((after.position[axis] - before.position[axis]) / (2.0 * h), at.velocity[axis],
                   "analytic velocity", 2.0e-4);
      require_near((after.velocity[axis] - before.velocity[axis]) / (2.0 * h),
                   at.acceleration[axis], "analytic acceleration", 2.0e-3);
      require_near((after.acceleration[axis] - before.acceleration[axis]) / (2.0 * h),
                   at.jerk[axis], "analytic jerk", 1.0e-2);
      require_near((after.jerk[axis] - before.jerk[axis]) / (2.0 * h), at.snap[axis],
                   "analytic snap", 2.0e-2);
    }

    require_near((after.yaw - before.yaw) / (2.0 * h), at.yaw_rate, "analytic yaw rate", 2.0e-4);
    require_near((after.yaw_rate - before.yaw_rate) / (2.0 * h), at.yaw_acceleration,
                 "analytic yaw acceleration", 2.0e-3);
  }
}

void test_exact_segment_boundary() {
  const auto origin = trajectory::stationary_reference({0.0, 0.0, -2.0}, 0.0);
  const trajectory::Sequence sequence({
      trajectory::Segment::line(2.0, {1.0, 0.0, 0.0}),
      trajectory::Segment::step(1.0, {0.0, 1.0, 0.0}),
      trajectory::Segment::hold(1.0),
  });

  const auto at_boundary = sequence.sample(2.0, origin);
  const auto after_boundary = sequence.sample(2.0 + 1.0e-6, origin);
  const auto complete = sequence.sample(50.0, origin);

  // The line owns the boundary; the step appears immediately afterward.
  require_vector(at_boundary.position, {1.0, 0.0, -2.0}, "boundary line end");
  require_vector(after_boundary.position, {1.0, 1.0, -2.0}, "boundary step entry");
  require_vector(complete.position, {1.0, 1.0, -2.0}, "boundary final hold");
  require_stationary(at_boundary);
  require_stationary(after_boundary);
  require_stationary(complete);
}

void test_invalid_trajectory_inputs() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const auto rejects = [](auto&& action) {
    try {
      action();
    } catch (const std::invalid_argument&) {
      return true;
    }
    return false;
  };

  if (!rejects([] { (void)trajectory::Segment::hold(0.0); }) ||
      !rejects([] { (void)trajectory::Segment::circle(1.0, -1.0, 1.0); }) ||
      !rejects([nan] { (void)trajectory::Segment::line(1.0, {nan, 0.0, 0.0}); }) ||
      !rejects([] { (void)trajectory::Sequence({}); }) ||
      !rejects([nan] { (void)trajectory::stationary_reference({0.0, nan, 0.0}, 0.0); })) {
    throw std::runtime_error("Trajectory accepted invalid construction inputs.");
  }

  const auto origin = trajectory::stationary_reference({0.0, 0.0, -2.0}, 0.0);
  const auto segment = trajectory::Segment::line(2.0, {1.0, 0.0, 0.0});
  const trajectory::Sequence sequence({segment});

  if (!rejects([&] { (void)segment.sample(nan, origin); }) ||
      !rejects([&] { (void)sequence.sample(infinity, origin); }) || !rejects([] {
        (void)trajectory::load_trajectory(TRAJECTORY_CONFIG_PATH, "nonexistent_trajectory");
      })) {
    throw std::runtime_error("Trajectory accepted invalid sample or config inputs.");
  }
}

} // namespace

int main() {
  test_editable_configuration();
  test_step_response();
  test_hold();
  test_line();
  test_circle();
  test_helix();
  test_yaw_sweep();
  test_figure_eight();
  test_sequence();
  test_analytic_segment_derivatives();
  test_exact_segment_boundary();
  test_invalid_trajectory_inputs();

  std::cout << "Trajectory tests passed.\n";

  return EXIT_SUCCESS;
}
