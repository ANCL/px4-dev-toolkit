// Analytic attitude references follow Lee et al., CDC 2010, DOI: 10.1109/CDC.2010.5717652.
// Basis derivatives follow fdcl-gwu/uav_geometric_control commit
// f5fcb51c3b962152a4895a9d2e86e4a184655741.

#include <offboard_controllers/se3/control/attitude_reference.hpp>

#include <cmath>
#include <stdexcept>

#include <offboard_controllers/math/operations.hpp>

namespace offboard_controllers::se3::attitude_reference {

namespace {

constexpr double kDirectionTolerance = 1.0e-12;

struct DirectionDynamics {
  Vector3 value{};
  Vector3 derivative{};
  Vector3 second_derivative{};
};

struct BasisDynamics {
  DirectionDynamics b1{};
  DirectionDynamics b2{};
  DirectionDynamics b3{};
};

Vector3 normalized_direction(const Vector3& vector, const char* error_message);
BasisDynamics desired_basis_dynamics(const Vector3& force, const Vector3& force_derivative,
                                     const Vector3& force_second_derivative, double yaw,
                                     double yaw_rate, double yaw_acceleration);
RotationMatrix rotation_from_basis(const BasisDynamics& basis);
Vector3 desired_angular_velocity(const BasisDynamics& basis);
Vector3 desired_angular_acceleration(const BasisDynamics& basis);

} // namespace

RotationMatrix compute_orientation(const Vector3& force, double yaw) {
  if (!is_finite(force) || !std::isfinite(yaw)) {
    throw std::invalid_argument("SE3 force and yaw must be finite.");
  }

  const Vector3 b3d =
      normalized_direction(-force, "SE3 desired attitude requires a non-zero force vector.");

  const Vector3 b1c{
      std::cos(yaw),
      std::sin(yaw),
      0.0,
  };

  const Vector3 b2d =
      normalized_direction(cross(b3d, b1c), "SE3 desired attitude heading is degenerate.");

  // Complete the orthonormal right-handed basis after fixing b3_d from
  // thrust direction and b2_d from the commanded heading.
  const Vector3 b1d = cross(b2d, b3d);

  return {
      b1d,
      b2d,
      b3d,
  };
}

DesiredAttitudeRate compute_rate(const Vector3& force, const Vector3& force_derivative, double yaw,
                                 double yaw_rate) {
  const BasisDynamics basis =
      desired_basis_dynamics(force, force_derivative, {}, yaw, yaw_rate, 0.0);

  return {
      rotation_from_basis(basis),
      desired_angular_velocity(basis),
  };
}

DesiredAttitudeDynamics compute_dynamics(const Vector3& force, const Vector3& force_derivative,
                                         const Vector3& force_second_derivative, double yaw,
                                         double yaw_rate, double yaw_acceleration) {
  const BasisDynamics basis = desired_basis_dynamics(
      force, force_derivative, force_second_derivative, yaw, yaw_rate, yaw_acceleration);

  return {
      rotation_from_basis(basis),
      desired_angular_velocity(basis),
      desired_angular_acceleration(basis),
  };
}

namespace {

Vector3 normalized_direction(const Vector3& vector, const char* error_message) {
  const double magnitude = norm(vector);

  if (!std::isfinite(magnitude) || magnitude <= kDirectionTolerance) {
    throw std::invalid_argument(error_message);
  }

  return vector / magnitude;
}

// Differentiate a unit direction analytically; the direction is undefined
// if the vector magnitude falls below kDirectionTolerance.
DirectionDynamics normalized_direction_dynamics(const Vector3& vector, const Vector3& derivative,
                                                const Vector3& second_derivative,
                                                const char* error_message) {
  const double magnitude = norm(vector);

  if (!std::isfinite(magnitude) || magnitude <= kDirectionTolerance || !is_finite(derivative) ||
      !is_finite(second_derivative)) {
    throw std::invalid_argument(error_message);
  }

  // For n = v / ||v||:
  //
  //   ||v||_dot = n . v_dot
  //   n_dot     = (v_dot - ||v||_dot n) / ||v||
  //
  // Differentiate these expressions analytically again for n_ddot. This
  // avoids finite-difference noise in the desired attitude-rate and
  // attitude-acceleration feed-forward paths.
  const Vector3 direction = vector / magnitude;

  const double magnitude_derivative = dot(direction, derivative);

  const Vector3 direction_derivative = (derivative - magnitude_derivative * direction) / magnitude;

  const double magnitude_second_derivative =
      (dot(derivative, derivative) + dot(vector, second_derivative) -
       magnitude_derivative * magnitude_derivative) /
      magnitude;

  const Vector3 direction_second_derivative =
      (second_derivative - 2.0 * magnitude_derivative * direction_derivative -
       magnitude_second_derivative * direction) /
      magnitude;

  return {
      direction,
      direction_derivative,
      direction_second_derivative,
  };
}

// Build the desired FRD axes in NED and their analytic derivatives.
// Thrust direction and commanded horizontal heading cannot be parallel.
BasisDynamics desired_basis_dynamics(const Vector3& force, const Vector3& force_derivative,
                                     const Vector3& force_second_derivative, double yaw,
                                     double yaw_rate, double yaw_acceleration) {
  if (!is_finite(force) || !is_finite(force_derivative) || !is_finite(force_second_derivative) ||
      !std::isfinite(yaw) || !std::isfinite(yaw_rate) || !std::isfinite(yaw_acceleration)) {
    throw std::invalid_argument("SE3 desired-attitude dynamics require finite inputs.");
  }

  // b3_d = -A / ||A||. Differentiating the normalized force analytically
  // keeps desired angular-rate feedforward tied to the same force used for Rd.
  const DirectionDynamics b3 =
      normalized_direction_dynamics(-force, -force_derivative, -force_second_derivative,
                                    "SE3 desired attitude requires a non-zero force vector.");

  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);

  // b1c is the commanded horizontal heading direction associated with yaw.
  // Combining it with the desired thrust direction b3_d constructs the
  // right-handed desired body basis used by Lee's geometric controller.
  const Vector3 b1c{
      cosine,
      sine,
      0.0,
  };

  const Vector3 b1c_derivative{
      -sine * yaw_rate,
      cosine * yaw_rate,
      0.0,
  };

  const Vector3 b1c_second_derivative{
      -cosine * yaw_rate * yaw_rate - sine * yaw_acceleration,
      -sine * yaw_rate * yaw_rate + cosine * yaw_acceleration,
      0.0,
  };

  // b2_d = normalize(b3_d x b1c). This becomes singular when the desired
  // thrust direction is parallel to the commanded heading direction; reject
  // that geometry rather than constructing an undefined attitude.
  const Vector3 b2_raw = cross(b3.value, b1c);

  const Vector3 b2_raw_derivative = cross(b3.derivative, b1c) + cross(b3.value, b1c_derivative);

  const Vector3 b2_raw_second_derivative = cross(b3.second_derivative, b1c) +
                                           2.0 * cross(b3.derivative, b1c_derivative) +
                                           cross(b3.value, b1c_second_derivative);

  const DirectionDynamics b2 =
      normalized_direction_dynamics(b2_raw, b2_raw_derivative, b2_raw_second_derivative,
                                    "SE3 desired attitude heading is degenerate.");

  const DirectionDynamics b1{
      cross(b2.value, b3.value),
      cross(b2.derivative, b3.value) + cross(b2.value, b3.derivative),
      cross(b2.second_derivative, b3.value) + 2.0 * cross(b2.derivative, b3.derivative) +
          cross(b2.value, b3.second_derivative),
  };

  return {
      b1,
      b2,
      b3,
  };
}

RotationMatrix rotation_from_basis(const BasisDynamics& basis) {
  return {
      basis.b1.value,
      basis.b2.value,
      basis.b3.value,
  };
}

Vector3 desired_angular_velocity(const BasisDynamics& basis) {
  // R_d^T Rdot_d = hat(Omega_d). Taking the skew part suppresses only
  // floating-point loss of orthogonality; analytically it is already skew.
  return {
      0.5 * (dot(basis.b3.value, basis.b2.derivative) - dot(basis.b2.value, basis.b3.derivative)),
      0.5 * (dot(basis.b1.value, basis.b3.derivative) - dot(basis.b3.value, basis.b1.derivative)),
      0.5 * (dot(basis.b2.value, basis.b1.derivative) - dot(basis.b1.value, basis.b2.derivative)),
  };
}

Vector3 desired_angular_acceleration(const BasisDynamics& basis) {
  // d/dt(R_d^T Rdot_d) = hat(Omegadot_d). Terms quadratic in Rdot_d are
  // symmetric and disappear when the skew part is taken.
  return {
      0.5 * (dot(basis.b3.value, basis.b2.second_derivative) -
             dot(basis.b2.value, basis.b3.second_derivative)),
      0.5 * (dot(basis.b1.value, basis.b3.second_derivative) -
             dot(basis.b3.value, basis.b1.second_derivative)),
      0.5 * (dot(basis.b2.value, basis.b1.second_derivative) -
             dot(basis.b1.value, basis.b2.second_derivative)),
  };
}

} // namespace

} // namespace offboard_controllers::se3::attitude_reference
