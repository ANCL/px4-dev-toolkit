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

  void set_saturation_status(
    const std::array<bool, 3> & positive,
    const std::array<bool, 3> & negative);

  void reset_integral();

  // MulticopterRateControl advances its timing state on every
  // VehicleAngularVelocity callback, even while rate control is inactive.
  // Use this for samples that do not run the controller so the next active
  // update sees the same sample-to-sample dt as PX4.
  void observe_timestamp_sample(
    uint64_t timestamp_sample_us);

  // Run one PX4-equivalent rate-controller update from a new
  // VehicleAngularVelocity sample. timestamp_sample_us drives dt exactly as
  // MulticopterRateControl does in the pinned PX4 implementation.
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
