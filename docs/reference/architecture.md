# Architecture

[Documentation](../index.md) / Reference / Architecture

This page describes the toolkit's ownership boundaries and data flow. Operational commands live in the installation/environment/controller/workflow pages.

## Repository architecture

```text
analysis/    pure Python bag analysis, profiles, plots, comparisons
config/      project-wide runtime/vehicle/topic/sequence/source/version config
docker/      SITL-only multi-stage image + Compose
px4/         PX4 build helper; fetched PX4 source lives below this directory
ros2/        ROS 2 workspace and maintained toolkit packages
setup/       native install and pinned-source reconstruction
tools/       user-facing runtime, study, and analysis entry points
```

Fetched third-party repositories are reconstructed under `px4/`, `ros2/src/`, and `tools/` from the pinned manifests in `config/sources/*.repos`. They are kept separate from toolkit-owned code.

## Reading the source code

Start at a user-facing command or ROS launch file, then follow the component
that owns the behavior. Runtime environments and experiments are independent:
starting SITL or hardware services does not arm the vehicle or execute a study.

| Task | Read from | Follow into |
|---|---|---|
| Start an environment | `tools/sitl` or `tools/hardware` | `config/runtime/`, `tools/runtime/`, `ros2/src/px4_bringup/launch/` |
| Install and reconstruct dependencies | `setup/install.sh` | `setup/fetch_sources.sh`, `config/sources/*.repos`, `ros2/build.sh` |
| Run Position-mode or Offboard control | ROS launch files in `ros2/src/` | Package `src/*/node.cpp`, then state, lifecycle, control, and publication modules |
| Understand SE(3) control | `ros2/src/offboard_controllers/src/se3/node.cpp` | `src/se3/runtime/`, then stateless `src/se3/control/` laws and the trajectory library |
| Run a recorded study | `tools/run_sequence` | `tools/sequence/`, controller launch files, `px4_control_common.recording` |
| Analyze or compare a bag | `tools/px4_analyze` or `tools/px4_compare` | `analysis/core.py`, `analysis/pipeline/`, `analysis/profiles/` |

A file-level description should explain ownership, data flow, units/frames,
process lifecycle, or a non-obvious invariant. Comments that simply restate a
function name or line of code add no useful information.

## ROS 2 packages

### `px4_bringup`

Owns reusable process launchers for services such as the uXRCE-DDS Agent and QGroundControl.

### `px4_control_common`

Owns shared control-side utilities, including the client used to control the repository MAVProxy virtual joystick.

### `px4_position_mode`

Owns native Position-mode flight and Offboard staging tools, with their recording launches.

### `offboard_controllers`

Owns the independent Offboard position controller, SE(3) controller, trajectory library, reproduced PX4 rate-control logic, physical-wrench adaptation, vehicle-config loading, launch files, and controller recording configuration.

### `px4_mocap_bridge`

Owns conversion from the selected Vicon `PoseStamped` stream to PX4 external-vision `VehicleOdometry`, plus the combined Vicon/bridge launch file.

## SITL data flow

```mermaid
flowchart LR
    G[Gazebo model / sensors] --> P[PX4 SITL]
    P <--> G
    P <--> X[uXRCE-DDS Agent]
    X <--> R[ROS 2]
    C[Toolkit controller] <--> R
    P <--> M[MAVLink]
    M <--> MP[MAVProxy or QGroundControl]
    R --> B[rosbag]
    B --> A[analysis]
```

The current SITL estimator configuration uses Gazebo odometry/external-vision aiding plus IMU according to `config/runtime/sitl.env`.

## Hardware data flow

```mermaid
flowchart LR
    V[Vicon] --> VR[vicon_receiver]
    VR --> MB[px4_mocap_bridge]
    MB --> R[ROS 2 DDS domain]
    R <--> X[XRCE Agent]
    X <--> |serial| P[PX4 hardware]
    C[Toolkit controller] <--> R
    R --> B[rosbag]
    B --> A[analysis]
```

The hardware runtime wrapper (`tools/hardware`) owns Vicon reception/bridge and XRCE agent process lifecycle; physical PX4 firmware runs on the flight controller.

## Controller pipeline ownership

The toolkit is designed to move the external-control boundary without changing the surrounding environment:

```text
trajectory reference
→ translational command
→ attitude reference
→ body-rate reference
→ thrust/torque
→ PX4 control allocation
→ actuators
```

`offboard_position` hands off at position/trajectory reference. SE(3) can hand off at acceleration, attitude, rate, or normalized thrust/torque.

## Topic catalog

PX4 ROS topic names are centralized in:

```text
config/px4_topics.def
```

C++ controller code, recording configuration, and Python analysis resolve the same catalog instead of maintaining independent literal topic-name copies.

## Source/runtime/data separation

- **source/configuration** describes what should run,
- **runtime wrappers** own process trees and lifecycle,
- **launch files/controllers** own experiment execution,
- **rosbag** captures immutable run data,
- **experiment metadata/manifests** preserve run context,
- **analysis** consumes the recorded data without changing the experiment.

## Python analysis module boundaries

`analysis/core.py` owns rosbag loading and receive-time alignment. `analysis/pipeline/` owns common PX4 signal extraction, statistics and presentation; `analysis/profiles/` owns experiment-specific semantics. The SE(3) profile and its comparison package use small modules without changing their external Python import paths. Experiment launch files retain package-specific node settings, while `px4_control_common.recording` owns rosbag lifecycle and recording-flag parsing.

## Next

- [Configuration ownership](configuration.md)
- [Extending the toolkit](extending.md)
- [Controller overview](../controllers/overview.md)
