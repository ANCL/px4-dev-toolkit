<span id="px4-flight-analysis-from-ros-bags-and-ulogs"></span>

# Analysis data

`px4_analyze` accepts finalized ROS 2 bag directories and PX4 `.ulg` files.
`px4_compare` accepts finalized ROS bags; ULog comparison has a separate
entry point below. Both formats use shared analysis functions. Run these commands on a workstation
with the SITL analysis dependencies.

```bash
source ros2/runtime_env.sh
./tools/px4_analyze bags/raw/flight.ulg
./tools/px4_analyze bags/se3/experiment_directory
./tools/px4_compare bags/se3/<run-a> bags/se3/<run-b> --output bags/comparison/rosbag_test
```

For ULog comparison, invoke the installed isolated analysis runtime directly:

```bash
.venv/onboard_analysis/bin/python3 -s analysis/onboard/cli.py compare \
  bags/raw/flight_a.ulg bags/raw/flight_b.ulg \
  --output bags/comparison/ulog_test --profile onboard_se3
```

Use `--profile px4_ulog` for generic PX4 logs, or the default `auto` to detect
matching profiles. At least two sources and an explicit output directory are
required. Native SE3 comparison requires matching nonzero plan CRCs and one
unambiguous run per source (or a `flight.ulg` directory with run-selection
metadata); it rejects stationary-hold-only comparisons. This CLI does not
combine a ROS bag with a ULog in one comparison.

Single-source ULog analysis accepts `--profile auto|px4_ulog|onboard_se3` and
`--run-id <id>` through `px4_analyze`; the run selector applies only to native
SE3. Generic single-source output is `<ulog-parent>/<ulog-stem>/analysis/`.
Native SE3 uses the same directory, or `<ulog-stem>_run<id>/analysis/` when a
run ID is explicitly selected. An imported directory containing `flight.ulg`
gets its own `analysis/`. ROS-bag output is always `<bag>/analysis/`.

For a generic ULog, `summary.txt` contains the full native PX4 control
pipeline. The shared plotting code produces position, velocity, acceleration,
attitude, rates, thrust, torque and motor figures wherever the corresponding
signals were logged. Unavailable/bypassed layers are not synthesized.

For native onboard SE3, analysis uses its reference, plan CRC, and execution
interval. Comparison requires matching trajectory CRCs.

## Signal conventions

- Position, velocity, acceleration: PX4 local NED [m, m/s, m/s²].
- Attitude: body-FRD to local-NED quaternion, displayed as Euler degrees.
- Angular rates: body FRD rad/s decoded and displayed as degrees/s.
- Thrust and torque: normalized PX4 setpoints, **not physical N or N·m**.
- Motors: normalized actuator commands, **not RPM**.
- ULog: uORB boot timestamp, converted from µs to ns; ROS bag: recorded
  receive timestamp (ns). Each recording is relative to its own start.
  Comparing recordings does **not** imply synchronization of absolute clocks.
- uORB arrays such as `q[0]...q[3]` become canonical quaternion/vector fields.
  Only uORB instance `0` is analyzed; other instances must be selected by a
  future explicit source policy rather than silently merged.
- Position validity flags and finite filtering apply to both sources. If
  fields are absent in an older ULog schema, dependent metrics are unavailable.
- The generic comparison aligns plots to recording starts. To compare actual
  controller performance on the same trajectory, use profile-specific
  comparison with recorded maneuver timing and references.

ULog analysis selects the control pipeline and relevant diagnostics. Logged
sample counts reflect recording frequency, not controller update rate. For
hardware flights, account for frames, estimation, and allocation when
interpreting tracking results.

See [Analyze a run](analysis.md) for profile interpretation and
[Controller comparison](comparison.md) for trajectory-time
alignment and metrics.
