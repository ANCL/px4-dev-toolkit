#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "px4_mocap_bridge/conversion.hpp"

namespace
{

constexpr double kTolerance = 1e-6;

void require(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    std::exit(1);
  }
}

bool near(double actual, double expected)
{
  return std::abs(actual - expected) <= kTolerance;
}

geometry_msgs::msg::PoseStamped identity_pose()
{
  geometry_msgs::msg::PoseStamped pose;

  pose.pose.position.x = 1.0;
  pose.pose.position.y = 2.0;
  pose.pose.position.z = 3.0;
  pose.pose.orientation.w = 1.0;

  return pose;
}

void test_valid_pose()
{
  const auto pose = identity_pose();

  require(
    px4_mocap_bridge::pose_is_valid(pose),
    "finite position and unit quaternion should be valid");
}

void test_invalid_pose()
{
  auto pose = identity_pose();
  pose.pose.position.x =
    std::numeric_limits<double>::quiet_NaN();

  require(
    !px4_mocap_bridge::pose_is_valid(pose),
    "NaN position should be invalid");

  pose = identity_pose();
  pose.pose.orientation.w = 0.0;

  require(
    !px4_mocap_bridge::pose_is_valid(pose),
    "zero quaternion should be invalid");
}

void test_frame_conversion()
{
  const auto odometry =
    px4_mocap_bridge::convert_pose(
      identity_pose(),
      123456,
      true);

  require(
    odometry.pose_frame ==
    px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED,
    "output pose frame should be NED");

  require(near(odometry.position[0], 2.0), "NED x");
  require(near(odometry.position[1], 1.0), "NED y");
  require(near(odometry.position[2], -3.0), "NED z");

  const double half_sqrt = std::sqrt(0.5);

  require(
    near(std::abs(odometry.q[0]), half_sqrt),
    "converted quaternion w");
  require(
    near(odometry.q[1], 0.0),
    "converted quaternion x");
  require(
    near(odometry.q[2], 0.0),
    "converted quaternion y");
  require(
    near(std::abs(odometry.q[3]), half_sqrt),
    "converted quaternion z");

  require(
    odometry.velocity_frame ==
    px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN,
    "velocity frame should be unknown");

  for (const float value : odometry.velocity) {
    require(std::isnan(value), "velocity should be NaN");
  }

  for (const float value : odometry.angular_velocity) {
    require(std::isnan(value), "angular velocity should be NaN");
  }

  for (const float value : odometry.position_variance) {
    require(std::isnan(value), "position variance should be NaN");
  }

  for (const float value : odometry.orientation_variance) {
    require(std::isnan(value), "orientation variance should be NaN");
  }

  for (const float value : odometry.velocity_variance) {
    require(std::isnan(value), "velocity variance should be NaN");
  }

  require(
    odometry.timestamp == 123456,
    "PX4 timestamp should use current ROS time");
  require(odometry.quality == 1, "quality should be valid");
}

void test_timestamp_selection()
{
  auto pose = identity_pose();

  const uint64_t now_us = 987654;

  auto odometry =
    px4_mocap_bridge::convert_pose(
      pose,
      now_us,
      true);

  require(
    odometry.timestamp_sample == now_us,
    "zero header timestamp should fall back to current time");

  pose.header.stamp.sec = 12;
  pose.header.stamp.nanosec = 3456000;

  odometry =
    px4_mocap_bridge::convert_pose(
      pose,
      now_us,
      true);

  require(
    odometry.timestamp_sample == 12003456,
    "header timestamp should be converted to microseconds");

  odometry =
    px4_mocap_bridge::convert_pose(
      pose,
      now_us,
      false);

  require(
    odometry.timestamp_sample == now_us,
    "disabled header timestamp should use current time");
}

}  // namespace

int main()
{
  test_valid_pose();
  test_invalid_pose();
  test_frame_conversion();
  test_timestamp_selection();

  std::cout << "PX4 mocap conversion tests passed.\n";
  return 0;
}
