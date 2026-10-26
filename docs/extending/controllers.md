# Developing a controller

For ROS 2 Offboard development, follow the steps below. Use [Offboard Position](../controllers/offboard/position.md) for a small lifecycle example or [Offboard SE3](../controllers/offboard/se3/index.md) for separated control laws. [Native PX4 controllers](#native-onboard-implementations) have their own integration path.

## 1. Choose the calculation and PX4 interface

State the inputs, frames, units, outputs, and which stages PX4 still calculates. Choose a supported `OffboardControlMode` level and matching setpoint message. Undefined/unused fields must follow that interface's semantics; the acceleration example uses NaN for unused position/velocity, not zero.

Keep pure laws independent of ROS subscriptions where practical. The [SE(3) control functions](https://github.com/ANCL/px4-dev-toolkit/tree/main/ros2/src/offboard_controllers/src/se3/control) are testable independently; [`Se3::execute_handoff()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/se3/runtime/control.cpp#L23) shows where their outputs are selected for PX4 publication. Use explicit physical versus normalized outputs rather than inferring units from a topic name.

## 2. Add package/node integration

Add a library/node under `ros2/src/offboard_controllers/` when it fits the package, or create a separate ROS package when its dependencies justify it. Register executable/library targets, headers, dependencies, and install rules in the affected package metadata/CMake. Use [CMakeLists.txt](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/CMakeLists.txt) as the reference for target and installation rules.

Store controller gains/timing in its package, aircraft data in `config/vehicles/`, and motion definitions in shared trajectories. Set the update rate from the controller's sampling requirements.

## 3. Define launch configuration and topics

A launch file should expose real choices such as vehicle, trajectory, recording, and diagnostic output. Load the installed package-share configuration and make overrides explicit. [offboard_position.launch.py](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/launch/offboard_position.launch.py) is the smaller example; [se3.launch.py](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/launch/se3.launch.py) shows vehicle/mode selection.

Resolve PX4 topic symbols through [config/px4_topics.def](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/px4_topics.def). Add a catalog entry when the interface needs one, then use it consistently in C++, recording, and analysis. Toolkit diagnostic topics can be separate names with documented units. Select QoS compatible with incoming PX4 sensor streams; the existing subscriptions use sensor-data QoS.

## 4. Make mode transitions explicit

Define readiness from the state and validity flags your law needs. Distinguish receipt, finite values, and freshness requirements. Prepare a valid heartbeat/setpoint stream appropriate to the interface, request Offboard automatically or wait for external selection, and confirm through PX4 `nav_state` rather than relying solely on command acknowledgement.

Define trajectory origin/start time, behavior on mode loss and re-entry, return behavior, and process completion. Offboard Position uses confirmation deadlines; Offboard SE3 has different pre-entry and return behavior.

Specify arming authority, stream-loss behavior, pilot handback, and PX4 failsafe interactions.

## 5. Add recording and analysis

Use [`px4_control_common.recording.launch_experiment()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/px4_control_common/px4_control_common/recording.py#L150) when its finite-run semantics fit the new node. Select required topics in shared `config/recording/` profiles and document discovery/start/stop behavior. Explicit metadata should identify the profile and effective settings for a study.

Add or extend an analysis profile only for observables meaningful for this controller. Separate measured state, computed targets, and reconstructed values; explain unavailable optional metrics. For ROS-bag comparison, implement the profile's `comparison_signals` contract and define time origin, reference sources, and compatible quantities before interpreting cross-controller metrics.

## 6. Verify the addition

Review pure laws for frame, sign, and unit correctness, including trajectory derivatives. Verify readiness and mode transitions, missing or invalid state, and published-message semantics. Use source-backed vehicle and actuator constraints rather than copying one vehicle's calibration.

In a **native development checkout at the repository root**:

```bash
./ros2/build.sh sitl
git diff --check
```

Check the launch interface with `ros2 launch ... --show-args`. For behavioral changes, run the relevant SITL experiment and inspect recording finalization and analysis. Keep a record of the tested environment. See [Configuration](../reference/configuration.md) for settings.

## Native onboard implementations

Native modules register their flight mode and arming check with Commander and implement reference, state-validity, setpoint, and fault handling inside PX4. See the [native SE3 implementation](https://github.com/ANCL/PX4-Autopilot/blob/5dd32eb1e7ab8b79b8ece462ce5f22cd75272c56/src/modules/se3_control/Runtime.cpp) for mode registration, hold-on-entry, and rate setpoint publication.

Define outputs, readiness, entry/exit, reference timing, and faults for the new module. [Onboard control](../operations/onboard.md) covers the operator workflow.

Integrate parameter profiles, transfer tools, recording topics, and analysis as needed. For new embedded trajectory formats, update the compiler and firmware loader/sampler together.
