# Configuration ownership

[Documentation](../index.md) / Reference / Configuration

Use this page when the question is: **"I want to change X; where does that configuration belong?"**

The project deliberately separates runtime connectivity, vehicle physics, controller tuning, trajectory definition, study orchestration, PX4 topic names, external source pins, and tool versions.

## Ownership map

| What you are changing | Canonical location |
|---|---|
| shared ROS runtime setting | `config/runtime/common.env` |
| SITL process/simulator/XRCE/GCS settings | `config/runtime/sitl.env` |
| hardware serial/Vicon settings | `config/runtime/experiment.env` |
| vehicle mass/inertia/propulsion/allocator facts | `config/vehicles/*.yaml` |
| SE(3) controller gains and timing | `ros2/src/offboard_controllers/config/se3/controller.yaml` |
| shared trajectory segments/names | `ros2/src/offboard_controllers/config/trajectory/trajectories.yaml` |
| basic Offboard staging setpoint | `ros2/src/offboard_controllers/config/offboard.yaml` |
| experiment/study sequence definition | `config/sequences/` |
| project-wide PX4 ROS topic catalog | `config/px4_topics.def` |
| external repository revisions | `config/sources/*.repos` |
| host/tool/application versions | `config/versions.env` |
| per-controller rosbag topic selection | package `config/recording/` files |

## Runtime configuration

### `config/runtime/common.env`

Contains settings shared by environments, currently the ROS distribution used by runtime/build helpers.

### `config/runtime/sitl.env`

Owns simulator/runtime process selection and connectivity: PX4 target, Gazebo model/world, estimator runtime overrides, XRCE endpoint, MAVProxy endpoint, selected GCS, optional service start, and tmux session.

### `config/runtime/experiment.env`

Owns the companion-computer hardware/environment boundary: serial XRCE device/baud, Vicon server/topic/frame/transform settings, optional XRCE-agent start, and tmux session.

Do not put controller gains here.

## Vehicle configuration

`config/vehicles/*.yaml` contains curated facts about a specific vehicle/model. Examples include mass, inertia, center of mass, rotor geometry/directions, PX4 allocator data, hover thrust, and, where available for that vehicle, physical actuator thrust mapping.

Missing data is meaningful. For example, the FY690S file deliberately does not invent a hover-thrust value and therefore constrains the currently supported SE(3) handoff modes.

Do not add controller tuning to a vehicle file.

## Controller tuning

`ros2/src/offboard_controllers/config/se3/controller.yaml` owns gains and controller/lifecycle timing. Parameters that reproduce the pinned PX4 rate controller are tuning values, not vehicle properties.

## Trajectories

`ros2/src/offboard_controllers/config/trajectory/trajectories.yaml` owns reusable motion definitions in local NED coordinates. It should describe requested motion, not study ordering or controller gains.

## Sequences

`config/sequences/` owns repeatable orchestration: which launch file runs, with which launch arguments, in which order, and with what settle time.

A sequence may select a controller/vehicle/trajectory, but it should not duplicate their canonical definitions.

## PX4 topics

`config/px4_topics.def` is the canonical project-wide mapping of symbolic names to `/fmu/in/*` and `/fmu/out/*` ROS topics. Prefer adding/resolving a catalog entry instead of scattering a new literal PX4 topic through C++, recorder files, and Python analysis.

## External sources and versions

`config/sources/*.repos` answers **which source revision should be fetched**.

`config/versions.env` records the host/tool/application versions used by the documented environment.

Those are different responsibilities and should remain separate.

## Rule of thumb

A fact gets one canonical home. Other pages/code should resolve or link to that fact rather than maintaining slightly different copies.

## Next

- [Architecture](architecture.md)
- [Extending the toolkit](extending.md)
