#pragma once

#include <array>
#include <cstdint>

#include <offboard_controllers/math/types.hpp>

namespace offboard_controllers::px4_rate
{

struct Parameters
{
  // Raw PX4 multicopter-rate parameters. Effective P/I/D gains are
  // K multiplied component-wise by the corresponding P/I/D values.
  math::Vector3 p{};
  math::Vector3 i{};
  math::Vector3 d{};
  math::Vector3 ff{};
  math::Vector3 k{};
  math::Vector3 integrator_limit{};

  double yaw_torque_cutoff_hz{0.0};
};


struct Output
{
  double dt_s{0.0};

  math::Vector3 rate_error{};
  math::Vector3 proportional_feedback{};
  math::Vector3 integral_feedback{};
  math::Vector3 derivative_feedback{};
  math::Vector3 feedforward{};

  math::Vector3 unfiltered_torque{};
  math::Vector3 normalized_torque{};

  // Integrator state after this update. The torque above uses the integrator
  // state that existed at the beginning of the update, matching PX4.
  math::Vector3 integrator_state{};
};


class Controller
{
public:
  explicit Controller(const Parameters & parameters);

  // Allocator-saturation feedback:
  //   Inputs: per-axis positive/negative torque-saturation flags.
  //   Logic:  update() blocks only integral error that would push an already
  //           saturated axis farther into saturation.
  //   Output: anti-windup state used by subsequent rate-controller updates.
  void set_saturation_status(
    const std::array<bool, 3> & positive,
    const std::array<bool, 3> & negative);

  void reset_integral();

  // Timing-only update:
  //   Input:  gyro timestamp_sample [us].
  //   Logic:  advance the same sample clock used by PX4
  //           MulticopterRateControl while rate control is inactive.
  //   Output: no control command; the next active update sees the correct dt.
  void observe_timestamp_sample(
    uint64_t timestamp_sample_us);

  // Rate-controller contract:
  //   Inputs: timestamp_sample [us], measured/setpoint FRD body rates [rad/s],
  //           measured FRD angular acceleration [rad/s^2], and landed state.
  //   Logic:  reproduce the pinned PX4 P/I/D/feed-forward law, dt limiting,
  //           allocator-aware anti-windup, and yaw-torque filtering.
  //   Output: normalized body torque plus diagnostic terms and the
  //           post-update integrator state.
  Output update(
    uint64_t timestamp_sample_us,
    const math::Vector3 & rate,
    const math::Vector3 & rate_setpoint,
    const math::Vector3 & angular_acceleration,
    bool landed);

private:
  Parameters parameters_{};

  math::Vector3 gain_p_{};
  math::Vector3 gain_i_{};
  math::Vector3 gain_d_{};

  math::Vector3 integrator_{};

  std::array<bool, 3> saturation_positive_{};
  std::array<bool, 3> saturation_negative_{};

  uint64_t last_run_us_{0};

  double yaw_filter_time_constant_s_{0.0};
  double yaw_filter_state_{0.0};
};

}  // namespace offboard_controllers::px4_rate
