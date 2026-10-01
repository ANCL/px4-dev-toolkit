#pragma once

namespace offboard_controllers::se3
{

// SE3 translational quantities use the PX4 local NED frame.
struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};


struct RotationMatrix
{
  // Body axes expressed in the NED inertial frame.
  Vector3 b1{};
  Vector3 b2{};
  Vector3 b3{};
};


struct InertiaMatrix
{
  // Symmetric body-frame inertia tensor [kg m^2].
  double xx{0.0};
  double xy{0.0};
  double xz{0.0};
  double yy{0.0};
  double yz{0.0};
  double zz{0.0};
};


struct State
{
  Vector3 position{};
  Vector3 velocity{};
  Vector3 acceleration{};
  Vector3 jerk{};
  RotationMatrix attitude{};
  Vector3 angular_velocity{};
};


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


struct Parameters
{
  // Physical translational gains:
  //   kx [N/m]
  //   kv [N s/m]
  double mass{0.0};
  double kx{0.0};
  double kv{0.0};

  // Geometric attitude-error gain maps attitude error [rad] to body-rate
  // correction [rad/s].
  Vector3 attitude_gain{};

  // PX4-normalized rotational-controller gains. These do not represent
  // physical moment gains in N m.
  Vector3 normalized_rate_gain{};
  Vector3 normalized_angular_acceleration_gain{};
};


struct TranslationalOutput
{
  // Kinematic acceleration command. Gravity is not included.
  Vector3 acceleration{};

  // Lee translational force:
  //   A = m(a_cmd - g e3)
  Vector3 force_vector{};
};


struct DesiredAttitudeRate
{
  RotationMatrix attitude{};

  // Desired angular velocity expressed in the desired FRD body frame.
  Vector3 angular_velocity{};
};


struct DesiredAttitudeDynamics
{
  RotationMatrix attitude{};

  // Desired angular velocity and acceleration expressed in the desired FRD
  // body frame.
  Vector3 angular_velocity{};
  Vector3 angular_acceleration{};
};

}  // namespace offboard_controllers::se3
