# Troubleshooting

[Documentation](../index.md) / Reference / Troubleshooting

This page is intentionally limited to recurring actionable failures. Start from the symptom, check the stated condition, then apply the fix.

## `tools/sitl` rejects the command

**Symptom:** usage is printed immediately.

**Check:** the supported CLI is exactly:

```bash
./tools/sitl start [headless]
./tools/sitl status
./tools/sitl stop
```

**Fix:** do not use a bare `./tools/sitl` invocation.

## SITL rejects `GCS`

**Symptom:** `ERROR: GCS must be 'qgc' or 'mavproxy'.`

**Check:** `config/runtime/sitl.env`.

**Fix:** set `GCS=mavproxy` or `GCS=qgc`. The current launcher has no `none` mode.

## Gazebo command/configuration is missing

**Symptom:** the SITL launcher reports that the Gazebo Harmonic command configuration was not found or is unusable.

**Check:** whether the PX4 simulation dependencies were installed through the toolkit.

**Fix:** run the appropriate native install/profile or:

```bash
./setup/install_px4_dependencies.sh sitl
```

The launcher deliberately locates the system Gazebo command configuration rather than assuming an ambient `GZ_CONFIG_PATH` is correct.

## ROS 2 / DDS topics are missing

**Symptom:** expected `/fmu/` topics are absent.

**Check:**

```bash
./tools/sitl status
# or
./tools/experiment status

source ros2/runtime_env.sh
ros2 topic list | grep '^/fmu/'
```

For SITL, verify `START_XRCE_AGENT=true`, `XRCE_AGENT_TRANSPORT`, and `XRCE_AGENT_PORT`.

For the hardware stack, verify serial device, baud, permissions, PX4's XRCE client configuration, and that the agent process is running.

## Serial device is missing in Docker

**Symptom:** `/dev/ttyUSB0` does not exist in the hardware container.

**Check:** the actual host device:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

**Fix:** map it to the canonical container device. Example:

```bash
docker run --rm -it --network host \
  --device=/dev/ttyACM0:/dev/ttyUSB0 \
  ghcr.io/hosnooo/px4-dev-toolkit-experiment:latest bash
```

With Compose:

```bash
PX4_DOCKER_DEVICE=/dev/ttyACM0 \
  docker compose -f docker/compose.yaml run --rm experiment bash
```

## `tools/experiment` rejects the XRCE transport

**Symptom:** `experiment XRCE transport must be 'serial'.`

**Check:** `config/runtime/experiment.env`.

**Fix:** the hardware runtime wrapper currently requires `XRCE_AGENT_TRANSPORT=serial`. UDP belongs to the documented SITL setup unless the software is intentionally extended.

## Vicon/mocap data is missing

**Symptom:** the `tools/experiment` tmux session runs but there is no usable tracked pose / PX4 external-vision input.

**Check:**

- `VICON_HOSTNAME` reachability
- correct tracked object/topic in `MOCAP_TOPIC`
- Vicon namespace/frame configuration
- map transform settings
- `px4_mocap_bridge` process/log output

**Fix:** correct the site-specific settings in `config/runtime/experiment.env`. Do not change controller gains to compensate for a wrong mocap frame.

## Controller refuses a vehicle/handoff combination

**Symptom:** SE(3) reports missing `hover_thrust` or missing physical-wrench configuration.

**Check:** [SE(3) vehicle support](../controllers/se3.md#vehicle-support).

**Fix:** select a handoff supported by the curated vehicle data. Do not invent missing hover thrust, actuator curves, inertia, or geometry merely to bypass the runtime checks.

## QGroundControl is selected but unavailable

**Symptom:** SITL is configured with `GCS=qgc` but QGroundControl is not installed.

**Check:** which native installation profile was used.

**Fix:** use:

```bash
./setup/install_qgroundcontrol.sh
```

or install the `all` profile. The `sitl` profile installs MAVProxy, not QGroundControl.

## Sequence runner says `ros2` is unavailable

**Symptom:** `ros2 is not available. Source ros2/runtime_env.sh first.`

**Fix:**

```bash
source ros2/runtime_env.sh
./tools/run_sequence <sequence.yaml>
```

## Comparison output cannot be inferred

**Symptom:** `px4_compare` asks for `--output`.

**Cause:** the bags are standalone or do not share one sequence study's `bags/` directory.

**Fix:** provide:

```bash
./tools/px4_compare BAG_A BAG_B --output /path/to/comparison
```

## A stopped tmux session leaves Gazebo running

**Fix:** use the repository wrapper rather than killing tmux manually:

```bash
./tools/sitl stop
```

Its shutdown path explicitly searches for Gazebo processes associated with this repository after stopping tmux-owned processes.

## Next

- [SITL](../environments/sitl.md)
- [Hardware](../environments/hardware.md)
- [Configuration](configuration.md)
