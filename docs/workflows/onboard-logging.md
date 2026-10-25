# Recording onboard PX4 experiments

PX4's native ULog logger is retained. Obtain finished `.ulg` files manually
using standard PX4/QGroundControl facilities and pass them directly to the
existing `tools/px4_analyze` command. This toolkit intentionally does not
transfer large ULogs over MAVLink FTP.

## Independent ROS 2 recording

Source `ros2/runtime_env.sh` first, with PX4 DDS running. The standalone
recorder never arms a vehicle, changes modes or starts trajectories.

```bash
./tools/px4_record onboard_se3
./tools/px4_record status
./tools/px4_record stop
```

The output is `bags/onboard_se3/<timestamp>/` with `metadata.yaml` and
`experiment.json`. Start before selecting SE3 and stop after landing.

All experiments record the same PX4 state and control-layer signals from
`config/recording/common.txt`; each optional matching profile file only
adds required inputs or diagnostics. Existing launch-based Offboard recording
continues using `record:=true` and the same central topic configuration.

The standard `trajectory_setpoint` publication is the desired onboard SE3
position, velocity, acceleration, jerk and yaw while SE3 is active. Its
`vehicle_rates_setpoint` and native PX4 thrust/torque/actuator topics remain.
The SE3 `se3_trajectory_status` uORB message retains exact run phase, CRC and
higher-order references in ULog; those fields are not reconstructed in ROS 2.

## Analyze

```bash
source ros2/runtime_env.sh
./tools/px4_analyze bags/onboard_se3/<timestamp>
./tools/px4_analyze /path/to/flight.ulg --profile onboard_se3
```

The ULog native SE3 profile measures exact execution intervals, while a ROS
bag has no native run-phase metadata and reports the whole selected-mode
interval, including holds. Both share canonical native setpoint and state
analysis functions; unavailable stages are not inferred.
