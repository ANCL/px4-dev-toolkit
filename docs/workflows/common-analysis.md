# PX4 flight analysis from ROS bags and ULogs

`px4_analyze` and `px4_compare` support both ROS 2 bag directories and PX4
`.ulg` files. Source readers supply the PX4 observation contract to the same
`analysis/pipeline/` calculations, plots and statistics.

```bash
source ros2/runtime_env.sh
./tools/px4_analyze bags/raw/flight.ulg
./tools/px4_analyze bags/se3/experiment_directory
./tools/px4_compare bags/raw/flight_a.ulg bags/raw/flight_b.ulg --output bags/comparison/ulog_test
```

For a generic ULog, `summary.txt` contains the full native PX4 control
pipeline. The shared plotting code produces position, velocity, acceleration,
attitude, rates, thrust, torque and motor figures wherever the corresponding
signals were logged. Unavailable/bypassed layers are not synthesized.

For a native onboard SE(3) log, the controller-specific reference/CRC and
tracking analysis remains the primary profile. No unrelated whole-flight native pipeline report is generated. Controller
comparison still enforces matching trajectory CRCs.

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

ULog data inventory is source dependent. The adapter selects the control
pipeline and relevant diagnostics, not every possible sensor/estimator field.
A recorded dataset's count describes *logger sampling*, not guaranteed
controller execution frequency. For physical hardware, the source coordinate
frame, state estimator and actuator allocation must be understood before
interpreting differences as controller errors.
