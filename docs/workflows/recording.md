# Recording experiments

[Documentation](../index.md) / Workflows / Recording

Recording is integrated into the launch workflows so the recorder starts before control and is finalized after the experiment process exits.

## Lifecycle

For the controller launch files that support recording:

```text
start rosbag
→ allow DDS discovery
→ start controller / flight process
→ controller owns experiment completion
→ stop rosbag
→ wait for recorder finalization
→ terminate launch
```

The launch files currently give the recorder a one-second startup window before starting the controller so early handoff traffic is not missed.

## Record a native Position-mode run

```bash
source ros2/runtime_env.sh
ros2 launch px4_position_takeoff_hover position_takeoff_hover.launch.py \
  record:=true
```

The bag is placed under the profile-specific `bags/` directory.

## Record Offboard position

```bash
ros2 launch offboard_controllers offboard_position.launch.py \
  trajectory:=step_response \
  record:=true
```

## Record SE(3)

```bash
ros2 launch offboard_controllers se3.launch.py \
  vehicle:=gz_f450 \
  trajectory:=full_excitation \
  handoff:=acceleration \
  publish_diagnostics:=true \
  record:=true
```

Standalone SE(3) recording defaults to:

```text
bags/se3/<timestamp>/
```

The SE(3) launch also exposes `recording_output:=...` so the sequence runner can provide a deterministic study-owned bag path.

## Topic ownership

Recording selections live with the controller packages, for example:

```text
ros2/src/offboard_controllers/config/recording/
```

PX4 topics in those files use symbolic names from the central catalog:

```text
config/px4_topics.def
```

Toolkit-owned ROS topics, such as SE(3) diagnostics, can be listed directly as absolute topic names.

This separation avoids scattering literal `/fmu/in/*` and `/fmu/out/*` strings through launch/analysis configuration.

## SE(3) recording scope

The SE(3) recording profile includes:

- PX4 vehicle status/state and estimator/failsafe context
- Offboard control mode and the selected handoff setpoints
- downstream PX4 trajectory/attitude/rate/thrust/torque setpoints
- control allocator status and actuator motors
- the toolkit's trajectory reference
- optional controller diagnostics when published

## Finalization contract

The sequence runner treats `metadata.yaml` as the rosbag completion contract. It only advertises a sequence bag as finalized after that file exists. It then writes `experiment.json` beside the bag so later analysis can use explicit profile, sequence/step identity, and launch arguments instead of inferring controller configuration from downstream signals.

## Analyze a bag

```bash
./tools/px4_analyze bags/<profile>/<run>
```

Sequence-study bags live under a study directory; see [Sequences](sequences.md).

## Next

- [Sequences](sequences.md)
- [Single-run analysis](../analysis/analysis.md)
