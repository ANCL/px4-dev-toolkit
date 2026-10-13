// Internal fixed-size matrices and helpers for wrench conversion.

#pragma once

#include <array>

#include <offboard_controllers/se3/physical_wrench_adapter.hpp>

namespace offboard_controllers::physical_wrench_adapter::detail {

using Matrix4 = std::array<std::array<double, kRotorCount>, kRotorCount>;
using Vector4 = std::array<double, kRotorCount>;

inline constexpr double kFeasibilityTolerance = 1.0e-9;

Matrix4 inverse(Matrix4 matrix);
Vector4 multiply(const Matrix4 & matrix, const Vector4 & vector);
Matrix4 physical_effectiveness(const Parameters & parameters);
Matrix4 allocator_effectiveness(const Parameters & parameters);
Vector4 allocator_normalization_scale(const Matrix4 & effectiveness);

double thrust_from_control(const Parameters & parameters, double control);
double control_from_thrust(const Parameters & parameters, double thrust_n);
void validate_parameters(const Parameters & parameters);

}  // namespace offboard_controllers::physical_wrench_adapter::detail
