<!-- Preserve existing section links. -->
<span id="single-run-analysis"></span>

<span id="analyzing-one-run"></span>
# Analyze a run

`px4_analyze` analyzes finalized ROS bags and PX4 ULogs, producing `summary.txt` and available plots. This page covers ROS-bag profiles; [Analysis data](../workflows/common-analysis.md) covers formats and timing.

<!-- Preserve existing section links. -->
<span id="run"></span>
<span id="output-location"></span>

## Run and outputs

In a **repository-root shell with `ros2/runtime_env.sh` sourced**, use the discovered bag path from the [complete example](../experiments/complete-example.md#4-inspect-and-analyze-the-finalized-bag):

```bash
./tools/px4_analyze "$bag_path"
```

`bag_path` must contain `metadata.yaml` and `experiment.json`. Run analysis on a workstation with the numerical dependencies. Results appear under:

```{ .text .no-copy }
<bag>/analysis/
├── summary.txt
└── *.png
```

`summary.txt` is written before plots. Required-topic errors stop analysis; absent optional topics omit their corresponding metrics. Storage backend selection comes from `metadata.yaml` and installed ROS plugins.

<!-- Preserve existing section links. -->
<span id="analysis-flow"></span>

## Profile detection

ROS-bag profiles: `position_takeoff_hover`, `offboard_takeoff_handoff`, `offboard_position`, `se3`, and `onboard_se3`. Analysis selects the profile from `experiment.json`. For ULogs, choose `px4_ulog` or `onboard_se3` through detection or `--profile`.

Offboard SE3 handoff selection comes from recorded `OffboardControlMode`; conflicting/changing levels are rejected. Direct thrust/torque law identity requires metadata because the normalized commands alone do not distinguish the geometric, physical-adapter, and PX4-rate variants.

## Reduced DDS telemetry on F450 hardware

The F450 hardware DDS profile omits `/fmu/out/sensor_combined`, `/fmu/out/battery_status`, and `/fmu/out/manual_control_setpoint` for serial bandwidth. These signals remain available internally to PX4.

The Position-mode and staging profiles use PX4-confirmed arming and mode transitions. Joystick timing and centered-stick drift are not measured by the DDS recording contract.

## Time alignment

[analysis/core.py](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/analysis/core.py) uses **rosbag receive timestamps** as the common cross-topic clock. It does not mix PX4 boot/sample timestamps with host/ROS time. Each topic is independently sampled.

The full bag establishes lifecycle events. Offboard SE3 extraction begins at confirmed Offboard entry; the common performance view clips to the interval ending at the first confirmed Position return. The profile requires recorded armed state, Offboard entry, Position return, heartbeat/diagnostic reference before entry, and selected handoff samples. An interrupted run without return may fail this profile rather than yielding a normal performance report.

The `onboard_se3` ROS-bag profile selects the first external-mode interval
(`nav_state` 23–30) and includes holds before/after motion. It does not identify
a specific external controller or infer ULog-only run phase/CRC fields. Record
one known controller experiment per bag. Use native [ULog analysis](../workflows/onboard-logging.md)
when an exact trajectory execution interval is required.

Reference signals are interpolated onto measured-signal timestamps over the overlapping interval; duplicate reference times retain the first value.

<!-- Preserve existing section links. -->
<span id="se3-analysis"></span>
<span id="interpreting-the-summary"></span>

## Read the summary first

Check profile, controller identity, bag duration, arming and mode transitions, prestream, handoff timing, and missing signals. Then read tracking and control statistics. Two statistical treatments appear in the code:

- Axis tracking tables from [`_aligned_error_stats()`](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/analysis/pipeline/statistics.py) use sample RMS $\sqrt{\frac1N\sum e_k^2}$ and maximum absolute error after interpolation.
- Common Offboard performance and maneuver metrics use time-weighted RMS from trapezoidal squared-error integration, explained in [comparison](comparison.md#metric-definitions).

Uneven sampling makes sample-RMS and time-weighted RMS differ.

<!-- Preserve existing section links. -->
<span id="source-organization"></span>
<span id="next"></span>

## Interpret the plots

**Position and velocity:** compare the desired reference to measured NED state. Position error is measured minus desired, in metres; velocity error is in m/s. Look for steady offsets, lag, overshoot, cross-axis coupling, and sudden estimator changes. Upward motion appears as decreasing Z.


**Acceleration:** separate desired kinematic acceleration, commanded acceleration, PX4/estimated signals, and gravity compensation. They are different quantities. Model-based force derivatives and reconstructed physical quantities are not additional sensors.

**Attitude and body rate:** inspect desired versus measured attitude and commanded versus measured FRD rate. Angle/rate plots can use degrees/deg/s while the controller uses rad/rad/s; read labels. Wrapped yaw error prevents a 360° jump from appearing as a large physical error.

**Mode transitions:** inspect pre-entry and return timing before judging startup transients. ROS launch time is not trajectory zero. A failed or repeated takeover can invalidate a simple single-interval comparison.

**Thrust, torque, and motors:** normalized setpoints are requested control coordinates, not measured thrust [N] or torque [N·m]. Look for clipping, allocation residuals, motor limits, and chatter. PX4-rate diagnostics separate old/new integrator state and filtered torque.

**Missing data:** an absent diagnostic/actuator stream cannot demonstrate absence of saturation or good tracking at that stage. Optional diagnostics differ by mode; the common measured response is the more consistent comparison basis.

[`enrich_summary()`](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/analysis/profiles/se3/reporting.py) assembles the Offboard SE3 reporting details. Use [Controller comparison](comparison.md) to compare completed runs.
