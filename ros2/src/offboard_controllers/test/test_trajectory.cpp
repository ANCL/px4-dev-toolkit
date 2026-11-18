// Consistency checks for the host trajectory generator.
//
// Each analytic derivative is compared with a central finite difference of
// the next-lower analytic derivative, so errors do not compound.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <trajectory_generator/trajectory.hpp>

namespace tg = trajectory_generator;

namespace {

constexpr double kStep = 1.0e-5;

tg::Reference origin() {
  return tg::stationary_reference({0.2, -0.1, -1.0}, 0.3);
}

void expect_near(const tg::Vector3& actual, const tg::Vector3& expected, double tolerance) {
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(actual[axis], expected[axis], tolerance) << "axis " << axis;
  }
}

tg::Vector3 difference(const tg::Vector3& after, const tg::Vector3& before) {
  return {(after[0] - before[0]) / (2.0 * kStep), (after[1] - before[1]) / (2.0 * kStep),
          (after[2] - before[2]) / (2.0 * kStep)};
}

void expect_consistent_derivatives(const tg::Segment& segment) {
  const double duration = segment.duration_s();
  for (int k = 0; k < 10; ++k) {
    const double t = (0.05 + 0.1 * k) * duration;  // Interior samples only.
    const tg::Reference before = segment.sample(t - kStep, origin());
    const tg::Reference after = segment.sample(t + kStep, origin());
    const tg::Reference now = segment.sample(t, origin());

    expect_near(difference(after.position, before.position), now.velocity, 1e-6);
    expect_near(difference(after.velocity, before.velocity), now.acceleration, 1e-6);
    expect_near(difference(after.acceleration, before.acceleration), now.jerk, 1e-5);
    expect_near(difference(after.jerk, before.jerk), now.snap, 1e-4);
    EXPECT_NEAR((after.yaw - before.yaw) / (2.0 * kStep), now.yaw_rate, 1e-6);
    EXPECT_NEAR((after.yaw_rate - before.yaw_rate) / (2.0 * kStep), now.yaw_acceleration, 1e-6);
  }
}

}  // namespace

TEST(Trajectory, SmoothSegmentDerivativesAreConsistent) {
  for (const tg::Segment& segment : {
           tg::Segment::line(4.0, {0.5, -0.3, -0.25}),
           tg::Segment::circle(8.0, 0.6, 2.0),
           tg::Segment::helix(8.0, 0.6, -0.5, 2.0),
           tg::Segment::yaw(3.0, 1.2),
           tg::Segment::figure_eight(10.0, 0.75, 1.2, 2.0),
       }) {
    expect_consistent_derivatives(segment);
  }
}

TEST(Trajectory, SmoothSegmentsStartAndEndAtRest) {
  for (const tg::Segment& segment : {
           tg::Segment::line(4.0, {0.5, 0.0, 0.0}),
           tg::Segment::circle(8.0, 0.6, 1.0),
           tg::Segment::figure_eight(10.0, 0.75, 1.2, 1.0),
       }) {
    for (const double t : {0.0, segment.duration_s()}) {
      const tg::Reference reference = segment.sample(t, origin());
      expect_near(reference.velocity, {}, 1e-12);
      expect_near(reference.acceleration, {}, 1e-12);
      expect_near(reference.jerk, {}, 1e-10);
      expect_near(reference.snap, {}, 1e-9);
    }
  }
}

// Completed segments anchor the next one, and the final pose is held.
TEST(Trajectory, SequenceChainsSegmentsAndHoldsFinalPose) {
  const tg::Sequence sequence({
      tg::Segment::hold(1.0),
      tg::Segment::line(2.0, {1.0, 0.0, 0.0}),
      tg::Segment::step(1.0, {0.0, 0.5, 0.0}),
      tg::Segment::yaw(2.0, 0.5),
  });
  EXPECT_DOUBLE_EQ(sequence.duration_s(), 6.0);

  expect_near(sequence.sample(0.5, origin()).position, origin().position, 1e-12);
  expect_near(sequence.sample(3.0, origin()).position, {1.2, -0.1, -1.0}, 1e-12);
  // A step is immediate inside its segment.
  expect_near(sequence.sample(3.001, origin()).position, {1.2, 0.4, -1.0}, 1e-12);

  const tg::Reference final_reference = sequence.sample(100.0, origin());
  expect_near(final_reference.position, {1.2, 0.4, -1.0}, 1e-12);
  EXPECT_NEAR(final_reference.yaw, 0.8, 1e-12);
  expect_near(final_reference.velocity, {}, 0.0);
}
