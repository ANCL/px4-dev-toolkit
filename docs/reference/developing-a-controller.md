# Developing a controller

A controller addition needs a mathematical calculation, a ROS/PX4 interface, a predictable mode lifecycle, and evidence that the new behavior works. Use [Offboard position](../controllers/offboard-position.md) as a small transport/lifecycle reference and [SE(3)](../controllers/se3.md) for separated calculations and diagnostics.

## 1. Choose the calculation and PX4 interface

State the inputs, frames, units, outputs, and which stages PX4 still calculates. Choose a supported `OffboardControlMode` level and matching setpoint message. Undefined/unused fields must follow that interface's semantics; the acceleration example uses NaN for unused position/velocity, not zero.

Keep pure laws independent of ROS subscriptions where practical. The existing [se3/control](https://github.com/ANCL/px4-dev-toolkit/tree/main/ros2/src/offboard_controllers/src/se3/control) functions provide testable examples. Use explicit physical versus normalized outputs rather than inferring units from a topic name.

## 2. Add package/node integration

Add a library/node under `ros2/src/offboard_controllers/` when it fits the package, or create a separate ROS package when its dependencies justify it. Register executable/library targets, headers, dependencies, and install rules in the affected package metadata/CMake. Follow the current [CMakeLists.txt](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/CMakeLists.txt), not a guessed target name.

Put gains and timing in a package configuration file, physical facts in `config/vehicles/`, and motion definitions in shared trajectories. A new controller is not required to run at 100 Hz or copy F450 assumptions; justify its sampling and state needs.

## 3. Define launch configuration and topics

A launch file should expose real choices such as vehicle, trajectory, recording, and diagnostic output. Load the installed package-share configuration and make overrides explicit. [offboard_position.launch.py](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/launch/offboard_position.launch.py) is the smaller example; [se3.launch.py](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/launch/se3.launch.py) shows vehicle/mode selection.

Resolve PX4 topic symbols through [config/px4_topics.def](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/px4_topics.def). Add a catalog entry when the interface needs one, then use it consistently in C++, recording, and analysis. Toolkit diagnostic topics can be separate names with documented units. Select QoS compatible with incoming PX4 sensor streams; the existing subscriptions use sensor-data QoS.

## 4. Make mode transitions explicit

Define readiness from the state and validity flags your law needs. Distinguish receipt, finite values, and freshness requirements. Prepare a valid heartbeat/setpoint stream appropriate to the interface, request Offboard, and confirm through PX4 `nav_state` rather than relying solely on command acknowledgement.

Define trajectory origin/start time, behavior on mode loss and re-entry, return behavior, and process completion. Offboard position uses steady-clock deadlines and ROS trajectory time; SE(3) is a separate lifecycle with different pre-entry/arming behavior. Copying a launch file does not resolve those design decisions.

Specify whether the controller can arm and how an interruption affects flight. A process exit or disappearing stream is not a guaranteed landing. Preserve PX4's configured independent flight controls/failsafes.

## 5. Add recording and analysis

Use `px4_control_common.recording.launch_experiment` when its finite-run semantics fit the new node. Select required topics in a package list and document discovery/start/stop behavior. Explicit metadata should identify the profile and effective settings for a study.

Add/extend an analysis profile only for observables that are meaningful for this controller. Separate measured state, computed targets, and reconstructed values; explain unavailable optional metrics. For comparison, define time origin and common quantities before extending the current SE(3)-only CLI.

## 6. Verify the addition

Test pure laws with meaningful frame/sign/unit cases and trajectory derivatives as needed. Test readiness and mode-transition behavior, missing/invalid state, and message semantics where the addition changes them. Include source-backed vehicle/actuator constraints rather than tying unrelated tests to one fixture.

In a **native development checkout at the repository root**:

```bash
./ros2/build.sh
./ros2/test.sh
git diff --check
```

Use `ros2 launch ... --show-args` with the actual new package/file for harmless argument inspection. If behavior changes, prepare and inspect an appropriate SITL experiment, recorder finalization, and analysis. Record what was executed and what is only source-checked. Physical validation is separate; ordinary toolkit additions should not edit the pinned upstream PX4 repository.

Continue with [Configuration files](configuration.md) to find the existing settings before adding new ones.
