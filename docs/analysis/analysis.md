<!-- Preserve existing section links. -->
<span id="single-run-analysis"></span>

<span id="analyzing-one-run"></span>
# Analyze a run

`px4_analyze` loads one finalized rosbag, selects an experiment profile, and writes a textual summary plus available plots. Start with the summary to establish what happened before interpreting a curve.

<!-- Preserve existing section links. -->
<span id="run"></span>
<span id="output-location"></span>

## Run and outputs

In a **repository-root shell with `ros2/runtime_env.sh` sourced**, use the discovered bag path from the [complete example](../experiments/complete-example.md#4-inspect-and-analyze-the-finalized-bag):

```bash
./tools/px4_analyze "$bag_path"
```

Set `bag_path` to a finalized bag directory. Example result structure:

```{ .text .no-copy }
<bag>/analysis/
├── summary.txt
└── *.png
```

The summary is written before plotting. A plotting dependency failure can leave a usable summary and a warning. Required-topic or data errors can prevent analysis entirely; optional missing topics suppress the corresponding observations. The reader supports rosbag storage backends through their installed ROS plugins, using `metadata.yaml` to select the storage identifier.

<!-- Preserve existing section links. -->
<span id="analysis-flow"></span>

## Profile detection

Registered profiles are `position_takeoff_hover`, `offboard_takeoff_handoff`, `offboard_position`, and `se3`. [`detect_profile_name()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/analysis/profiles/__init__.py#L38) prefers `experiment.json`; otherwise it uses the parent directory name. Study metadata is therefore necessary for bags stored under arbitrary step names.

For SE(3), recorded `OffboardControlMode` determines the active command level. Simultaneous or changing levels are rejected. Direct-law identity comes from explicit metadata; without it, normalized geometric diagnostics can identify that law, but PX4-rate and physical modes cannot reliably be distinguished from downstream wrench messages alone. `unknown` is a data limitation, not a missing controller label to guess.

## Reduced DDS telemetry on F450 hardware

The ANCL hardware firmware intentionally omits `/fmu/out/sensor_combined`,
`/fmu/out/battery_status`, and `/fmu/out/manual_control_setpoint` to preserve
serial bandwidth. The vehicle still handles its battery and pilot controls
internally. Recorder topic lists therefore omit those unavailable ROS topics.

For historical `position_takeoff_hover` and `offboard_takeoff_handoff` bags,
manual-control samples remain an optional analysis input. When present, their
joystick-phase timing is analyzed as before. When absent, the profiles retain
PX4-confirmed arming/mode events and control-pipeline analysis but mark
joystick-dependent timing (and centered-stick hover drift) unavailable; they
must not infer stick movements from the vehicle's motion. Native PX4 logs are
needed for any later reconstruction of those discarded observations.

## Time alignment

[analysis/core.py](https://github.com/ANCL/px4-dev-toolkit/blob/main/analysis/core.py) uses **rosbag receive timestamps** as the common cross-topic clock. It does not mix PX4 boot/sample timestamps with host/ROS time. Each topic is independently sampled.

The full bag establishes lifecycle events. SE(3) extraction begins at confirmed Offboard entry; the common performance view clips to the interval ending at the first confirmed Position return. The profile requires recorded armed state, Offboard entry, Position return, heartbeat/diagnostic reference before entry, and selected handoff samples. An interrupted run without return may fail this profile rather than yielding a normal performance report.

Reference signals are interpolated onto measured-signal times over their common interval. Stable sorting retains the first reference value at duplicate times. Do not compare arrays by index just because their rates look similar.

<!-- Preserve existing section links. -->
<span id="se3-analysis"></span>
<span id="interpreting-the-summary"></span>

## Read the summary first

Check profile, controller identity, bag duration, arming and mode transitions, prestream, handoff timing, and missing signals. Then read tracking and control statistics. Two statistical treatments appear in the code:

- Axis tracking tables from [`_aligned_error_stats()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/analysis/pipeline/statistics.py#L8) use sample RMS $\sqrt{\frac1N\sum e_k^2}$ and maximum absolute error after interpolation.
- Common Offboard performance and maneuver metrics use time-weighted RMS from trapezoidal squared-error integration, explained in [comparison](comparison.md#metric-definitions).

Uneven logging rates can make these numbers differ. Read the table's context instead of applying one generic RMS interpretation to every metric.

<!-- Preserve existing section links. -->
<span id="source-organization"></span>
<span id="next"></span>

## Interpret the plots

**Position and velocity:** compare the desired reference to measured NED state. Position error is measured minus desired, in metres; velocity error is in m/s. Look for steady offsets, lag, overshoot, cross-axis coupling, and sudden estimator changes. Upward motion appears as decreasing Z.


**Acceleration:** separate desired kinematic acceleration, commanded acceleration, PX4/estimated signals, and gravity compensation. They are different quantities. Model-based force derivatives and reconstructed physical quantities are not additional sensors.

**Attitude and body rate:** inspect desired versus measured attitude and commanded versus measured FRD rate. Angle/rate plots can use degrees/deg/s while the controller uses rad/rad/s; read labels. Wrapped yaw error prevents a 360° jump from appearing as a large physical error.

**Mode transitions:** inspect pre-entry and return timing before judging startup transients. ROS launch time is not trajectory zero. A failed or repeated takeover can invalidate a simple single-interval comparison.

**Thrust, torque, and motors:** normalized setpoints are requested control coordinates, not measured thrust [N] or torque [N·m]. Look for clipping, allocation residuals, motor limits, and chatter. Physical-mode “applied moment” is the adapter's modeled $\lambda M$, not a measured moment. PX4-rate diagnostics separate old/new integrator state and filtered torque.

**Missing data:** an absent diagnostic/actuator stream cannot demonstrate absence of saturation or good tracking at that stage. Optional diagnostics differ by mode; the common measured response is the more consistent comparison basis.

[`enrich_summary()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/analysis/profiles/se3/reporting.py#L500) assembles the SE(3) reporting details. Use [SE(3) Comparison](comparison.md) to compare completed runs.
