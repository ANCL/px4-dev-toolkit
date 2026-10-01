#pragma once

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace offboard_controllers
{

struct RuntimeTiming
{
  std::chrono::nanoseconds control_period{};
  int warmup_samples{};
  int retry_samples{};
};


inline int duration_to_samples(
  double control_rate_hz,
  double duration_s)
{
  const long long samples =
    std::llround(control_rate_hz * duration_s);

  if (
    samples < 1 ||
    samples > std::numeric_limits<int>::max())
  {
    throw std::invalid_argument(
            "Runtime timing duration produces an invalid sample count.");
  }

  return static_cast<int>(samples);
}


inline RuntimeTiming make_runtime_timing(
  double control_rate_hz,
  double warmup_duration_s,
  double retry_interval_s)
{
  if (
    !std::isfinite(control_rate_hz) ||
    control_rate_hz <= 0.0)
  {
    throw std::invalid_argument(
            "Control rate must be finite and positive.");
  }

  if (
    !std::isfinite(warmup_duration_s) ||
    warmup_duration_s <= 0.0)
  {
    throw std::invalid_argument(
            "Warmup duration must be finite and positive.");
  }

  if (
    !std::isfinite(retry_interval_s) ||
    retry_interval_s <= 0.0)
  {
    throw std::invalid_argument(
            "Offboard retry interval must be finite and positive.");
  }

  const long long period_ns =
    std::llround(1.0e9 / control_rate_hz);

  if (period_ns < 1) {
    throw std::invalid_argument(
            "Control rate produces an invalid timer period.");
  }

  return {
    std::chrono::nanoseconds{period_ns},
    duration_to_samples(
      control_rate_hz,
      warmup_duration_s),
    duration_to_samples(
      control_rate_hz,
      retry_interval_s),
  };
}

}  // namespace offboard_controllers
