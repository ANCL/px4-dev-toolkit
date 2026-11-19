<!-- Preserve existing section links. -->

<span id="software-structure"></span>
# Architecture

The toolkit separates environment services, controllers, trajectory generation, and flight-data analysis. This page maps each responsibility to its code.

<!-- Preserve existing section links. -->
<span id="sitl-data-flow"></span>
<span id="hardware-data-flow"></span>

## How the parts connect

```mermaid
---
config:
  flowchart:
    useMaxWidth: true
    wrappingWidth: 140
    padding: 10
    rankSpacing: 22
    nodeSpacing: 24
---
flowchart TB
    G["Gazebo"] <--> P["PX4 SITL<br/>Native PX4 terminal"]
    P <--> X["DDS · UDP"]
    X <--> R["ROS 2 + recorder"]
    P <--> M["MAVLink GCS stream · 14550"]
    M <--> GCS["MAVProxy or QGroundControl"]
    P --> N["Native SE3 + built-in trajectories"]
    R --> B["Bag + metadata"]
    B --> A["Analysis"]
```

For physical flight, firmware runs on the aircraft instead of the workstation:

```mermaid
---
config:
  flowchart:
    useMaxWidth: true
    wrappingWidth: 140
    padding: 10
    rankSpacing: 22
    nodeSpacing: 24
---
flowchart TB
    V["Vicon receiver"] --> C["Pose conversion"]
    C --> R["ROS 2 + recorder"]
    R <--> X["DDS agent · serial"]
    X <--> P["PX4 flight controller"]
    P <-->|telemetry| K["Wi-Fi link"]
    K <--> GCS["QGroundControl / MAVLink Console"]
    P --> N["Native SE3 + built-in trajectories"]
    R --> B["Recorded data"]
```

In simulation, PX4 uses configured Gazebo/external-vision aiding and IMU. On hardware, the mocap bridge converts ENU/FLU pose to NED/FRD `VehicleOdometry`. The MAVLink command link and XRCE-DDS use separate communication paths. See [SITL](../operations/sitl.md) and [Companion computer](../operations/companion.md) for runtime setup.

<!-- Preserve existing section links. -->
<span id="reading-the-source-code"></span>

## Where to find the code

<div class="px4-table-scroll" markdown="1">

| Read for | Entry point | Follow into |
|---|---|---|
| Installation | [setup/install.sh](https://github.com/ANCL/px4-dev-toolkit/blob/main/setup/install.sh) | Source pins and build scripts |
| Start services | [tools/sitl](https://github.com/ANCL/px4-dev-toolkit/blob/main/tools/sitl), [tools/hardware](https://github.com/ANCL/px4-dev-toolkit/blob/main/tools/hardware) | Runtime config and process helpers |
| Position-mode reference | [px4_position_mode](https://github.com/ANCL/px4-dev-toolkit/tree/main/ros2/src/px4_position_mode) | Launch, state and joystick |
| External position reference | [offboard_position](https://github.com/ANCL/px4-dev-toolkit/tree/main/ros2/src/offboard_controllers/src/offboard_position) | Node and control lifecycle |
| Shared trajectory mathematics | [`trajectory_generator/`](https://github.com/ANCL/px4-dev-toolkit/tree/main/trajectory_generator) | C++ sampler and YAML adapter |
| ROS 2 Offboard SE3 controller | [se3/node.cpp](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/se3/node.cpp) | [`Se3::execute_handoff()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/se3/runtime/control.cpp#L23), pure `control/`, trajectories |
| Native onboard control | [PX4 SE3 runtime](https://github.com/ANCL/PX4-Autopilot/blob/lab/main/src/modules/se3_control/Runtime.cpp) | Mode registration, hold and the three `SE3_CTRL` output paths |
| Record an experiment | [recording.py](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/px4_control_common/px4_control_common/recording.py) | [`launch_experiment()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/px4_control_common/px4_control_common/recording.py#L150) and topic selection |
| Run a study | [tools/run_sequence](https://github.com/ANCL/px4-dev-toolkit/blob/main/tools/run_sequence) | [`load_sequence()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/tools/sequence/config.py#L43), [`run_step()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/tools/sequence/execution.py#L50), manifests |
| Analyze/compare | [analysis](https://github.com/ANCL/px4-dev-toolkit/tree/main/analysis) | Signals, profiles and plots |

</div>

<!-- Preserve existing section links. -->
<span id="px4_bringup"></span>
<span id="px4_control_common"></span>
<span id="px4_position_mode"></span>
<span id="offboard_controllers"></span>
<span id="px4_mocap_bridge"></span>
<span id="controller-pipeline-ownership"></span>
<span id="topic-catalog"></span>

## ROS 2 packages

`px4_bringup` provides agent/GCS launchers. `px4_control_common` provides recording and the virtual-joystick client. `px4_position_mode` contains the reference flight and independent-Offboard staging examples. `offboard_controllers` contains position-reference/SE(3) nodes, trajectory YAML loading, vehicle loading, and rate reproduction. `px4_mocap_bridge` combines Vicon launch with pose conversion.

The analytic core of `trajectory_generator/` owns reference sampling through snap without ROS or PX4 dependencies; its separate configuration layer reads YAML. Offboard nodes load shared YAML and start trajectory time at confirmed Offboard entry. For native control, PX4's module-local `src/modules/se3_control/trajectory/` catalog is configured through `TRJ_*` parameters and sampled on the flight controller. Firmware mode selection first captures the actual pose for holding.

Native SE3 registers a Commander external-mode slot **inside PX4** and computes its reference and geometric attitude at a nominal 100 Hz. `SE3_CTRL` selects the command: attitude and normalized thrust for PX4's attitude and rate controllers (ATTITUDE_PX4), body rates and normalized thrust for PX4's rate controller (RATE_PX4, default), or an SI wrench at the gyro rate that `control_allocator` allocates with the `CA_ROTOR*` rotor model (PHYSICAL). PX4 retains actuator outputs and failsafes in every path. Its implementation is described under [Onboard SE3](../controllers/onboard/se3.md); the ROS controller is described under [Offboard SE3](../controllers/offboard/se3/index.md).


<!-- Preserve existing section links. -->
<span id="repository-architecture"></span>
<span id="sourceruntimedata-separation"></span>
<span id="python-analysis-module-boundaries"></span>
<span id="next"></span>

## Source, runtime, and data

`config/` stores runtime, vehicle, sequence, source/version, and topic definitions. `docker/` provides SITL packaging. `px4/` and `ros2/` hold build helpers and source locations; external projects are reconstructed from pinned manifests rather than vendored as toolkit code.

Controllers/launches execute a run. `tools/px4_record` can independently record DDS telemetry for onboard experiments. Rosbag captures received ROS messages; PX4 ULog captures firmware logging and is a separate artifact with a separate analysis path. Manifests preserve experiment identity. `analysis/core.py` reads receive timestamps, `analysis/pipeline/` extracts common PX4 signals/statistics, and `analysis/profiles/` defines experiment-specific meaning. Analysis writes reports beside the data without modifying controller behavior.

See [Configuration](configuration.md) for the settings owned by each component.

## Validation scope

The toolkit separates an implemented interface from a configuration that has been exercised in flight:

<div class="px4-table-scroll" markdown="1">

| Environment | Confirmed evidence | Still requires verification |
|---|---|---|
| Native SITL (`gz_f450`) | All three `SE3_CTRL` paths (November 2026): passive hold, live `setpoint`, `steps5`, `circle`, `figure8`, direct switching, completion and home hold, measured-pose hold, return to Position, controller/state/allocation faults; results in the [module README](https://github.com/ANCL/PX4-Autopilot/blob/lab/main/src/modules/se3_control/README.md#validation) | PHYSICAL with the default `THR_MDL_FAC = 0` holds about 0.45 m low; gyro loss, motor failure, hardware |
| Docker SITL | SITL image built and its packaged tools passed command/dependency checks | Complete live Docker simulation and controller-flight sequence |
| Physical companion / PX4 | Native module, shell and parameter paths are implemented | Actual aircraft tuning, state estimation, control handoff, recovery procedures, and physical flight |

</div>

Physical flight requires the [hardware](../operations/companion.md) and [onboard](../operations/onboard.md) preparation procedures.

### Automated checks

A small set of automated checks covers the mathematics and configuration that flight results depend on. They are consistency checks, not flight validation.

| Scope | Checks | Command |
|---|---|---|
| ROS 2 Offboard (`offboard_controllers/test/`) | Trajectory derivatives through snap against finite differences; desired angular velocity/acceleration against finite differences of \(R_d\); perfect-tracking force derivatives equal reference jerk/snap; Lee moment equals rigid-body dynamics at zero error; attitude-error sign; physical-wrench adapter reproduces the requested wrench and PX4's normalized mixer, and scales infeasible moments uniformly | `colcon test --packages-select offboard_controllers` then `colcon test-result --verbose` (in `ros2/`, sourced) |
| Tooling (`tools/tests/`) | Study trajectories and vehicles resolve; standalone and launch recorders resolve each recording profile identically; source-revision metadata | `python3 -m pytest tools/tests` (repository root, `ros2/runtime_env.sh` sourced) |
| Native SE3 (`src/modules/se3_control/Se3MathTest.cpp` and `physical/PhysicalAllocationTest.cpp` in PX4) | The same derivative, attitude-reference, perfect-tracking, and sign checks for the single-precision firmware implementation; the four native trajectories (defaults, steps, exact turns/cycles, direct replacement); both integrals (bounds, anti-windup, resets, zero-gain equivalence); control-path ownership; physical allocation (SI effectiveness, desaturation priorities, saturation feedback, stale commands) | `make tests TESTFILTER=Se3Math` and `make tests TESTFILTER=PhysicalAllocation` (in `px4/PX4-Autopilot`) |

The Offboard and native implementations are checked independently against the same mathematical properties; neither is compiled against the other.
