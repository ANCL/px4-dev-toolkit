#pragma once

#include <cmath>

#include <offboard_controllers/se3/types.hpp>

namespace offboard_controllers::se3
{

inline Vector3 operator+(const Vector3 & a, const Vector3 & b)
{
  return {
    a.x + b.x,
    a.y + b.y,
    a.z + b.z,
  };
}


inline Vector3 operator-(const Vector3 & a, const Vector3 & b)
{
  return {
    a.x - b.x,
    a.y - b.y,
    a.z - b.z,
  };
}


inline Vector3 operator-(const Vector3 & vector)
{
  return {
    -vector.x,
    -vector.y,
    -vector.z,
  };
}


inline Vector3 operator*(double scalar, const Vector3 & vector)
{
  return {
    scalar * vector.x,
    scalar * vector.y,
    scalar * vector.z,
  };
}


inline Vector3 operator*(const Vector3 & vector, double scalar)
{
  return scalar * vector;
}


inline Vector3 operator/(const Vector3 & vector, double scalar)
{
  return {
    vector.x / scalar,
    vector.y / scalar,
    vector.z / scalar,
  };
}


inline double dot(const Vector3 & a, const Vector3 & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}


inline Vector3 cross(const Vector3 & a, const Vector3 & b)
{
  return {
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x,
  };
}


inline Vector3 component_product(
  const Vector3 & a,
  const Vector3 & b)
{
  return {
    a.x * b.x,
    a.y * b.y,
    a.z * b.z,
  };
}


inline double norm(const Vector3 & vector)
{
  return std::sqrt(dot(vector, vector));
}


inline bool is_finite(const Vector3 & vector)
{
  return
    std::isfinite(vector.x) &&
    std::isfinite(vector.y) &&
    std::isfinite(vector.z);
}


inline bool is_finite(const RotationMatrix & rotation)
{
  return
    is_finite(rotation.b1) &&
    is_finite(rotation.b2) &&
    is_finite(rotation.b3);
}


inline bool is_finite(const InertiaMatrix & inertia)
{
  return
    std::isfinite(inertia.xx) &&
    std::isfinite(inertia.xy) &&
    std::isfinite(inertia.xz) &&
    std::isfinite(inertia.yy) &&
    std::isfinite(inertia.yz) &&
    std::isfinite(inertia.zz);
}


inline Vector3 rotate_body_to_inertial(
  const RotationMatrix & rotation,
  const Vector3 & body_vector)
{
  return
    body_vector.x * rotation.b1 +
    body_vector.y * rotation.b2 +
    body_vector.z * rotation.b3;
}


inline Vector3 rotate_inertial_to_body(
  const RotationMatrix & rotation,
  const Vector3 & inertial_vector)
{
  return {
    dot(rotation.b1, inertial_vector),
    dot(rotation.b2, inertial_vector),
    dot(rotation.b3, inertial_vector),
  };
}


inline Vector3 multiply(
  const InertiaMatrix & inertia,
  const Vector3 & vector)
{
  return {
    inertia.xx * vector.x +
    inertia.xy * vector.y +
    inertia.xz * vector.z,
    inertia.xy * vector.x +
    inertia.yy * vector.y +
    inertia.yz * vector.z,
    inertia.xz * vector.x +
    inertia.yz * vector.y +
    inertia.zz * vector.z,
  };
}

}  // namespace offboard_controllers::se3
