#pragma once

namespace offboard_controllers::math
{

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};


struct RotationMatrix
{
  // FRD body axes expressed in the NED inertial frame.
  Vector3 b1{};
  Vector3 b2{};
  Vector3 b3{};
};


struct InertiaMatrix
{
  // Symmetric FRD body-frame inertia tensor [kg m^2].
  double xx{0.0};
  double xy{0.0};
  double xz{0.0};
  double yy{0.0};
  double yz{0.0};
  double zz{0.0};
};

}  // namespace offboard_controllers::math
