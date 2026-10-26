<span id="recording-onboard-px4-experiments"></span>

# Onboard logging

PX4 ULogs are collected on the aircraft and analyzed after transfer. The toolkit analyzes `.ulg` files but does not download them through MAVLink FTP.

## Independent ROS 2 recording

With PX4 DDS active, run in a repository-root Ubuntu shell:

```bash
source ros2/runtime_env.sh
./tools/px4_record onboard_se3
```

The recorder remains in the foreground. Stop with Ctrl-C there after landing,
or use `./tools/px4_record status` and `./tools/px4_record stop` in a second
repository shell. Wait for metadata finalization before analysis.

The output is `bags/onboard_se3/<timestamp>/` with `metadata.yaml` and
`experiment.json`. Start before selecting the onboard mode and stop after landing. The standalone
metadata identifies the profile and topics; retain a separate copy of the
parameter profile, trajectory YAML/compiled plan, and firmware identity.

Recording profiles extend `config/recording/common.txt` with controller-specific topics. ROS Offboard launches also use these lists with `record:=true`.

The standard `trajectory_setpoint` publication is the desired onboard SE3
position, velocity, acceleration, jerk and yaw while SE3 is active. Its
`vehicle_rates_setpoint` and native PX4 thrust/torque/actuator topics remain.
The SE3 `se3_trajectory_status` uORB message retains exact run phase, CRC and
higher-order references in ULog; those fields are not reconstructed in ROS 2.

## Analyze

Run analysis on a workstation with the SITL analysis dependencies; the lean
companion installation does not install them. Obtain a finalized ULog using
standard PX4/QGroundControl facilities after logging has ended. In SITL, ULogs
are under the PX4 build runtime tree; on hardware they are stored on the SD
card. ROS bags and ULogs have different clocks and available fields.

```bash
source ros2/runtime_env.sh
./tools/px4_analyze bags/onboard_se3/<timestamp>
./tools/px4_analyze /path/to/flight.ulg --profile onboard_se3
```

The ULog native SE3 profile measures exact execution intervals, while a ROS
bag has no native run-phase metadata and reports the whole selected-mode
interval, including holds. The bag profile identifies the first registered external-mode
interval (`nav_state` 23–30), rather than a controller-specific SE3 ID; use a
single known onboard experiment per bag. Both share canonical native setpoint and state
analysis functions; unavailable stages are not inferred. A ULog containing
multiple trajectory run IDs requires explicit selection, for example:

```bash
./tools/px4_analyze /path/to/flight.ulg --profile onboard_se3 --run-id 1
```

The native profile can also analyze a stationary SE3 hold. See [Analysis data](data-formats.md) for outputs, units, and comparisons.
