<!-- Preserve existing section links. -->
<span id="recording-experiments"></span>

# Recording

Recording preserves the messages needed to reconstruct a flight. For a ROS controller launch, add `record:=true` to start the recorder before the node and finalize it after node exit. For an onboard controller, use the independent DDS recorder below and retain the native PX4 ULog as a separate source.

<span id="lifecycle"></span>
## How recording works

1. Start `ros2 bag record` and allow **1 second** for topic discovery.
2. Start the selected flight/controller node.
3. On node exit, stop the recorder and wait for its exit. Check `metadata.yaml` and the recorded topics before analysis.

[`launch_experiment()`](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/ros2/src/px4_control_common/px4_control_common/recording.py) supervises the recorder and controller, propagating failures. Topic discovery uses a 1-second delay; check recorded topic coverage afterward.

## Record a native Position-mode run

With a ready, initially landed/disarmed **SITL** vehicle and MAVProxy, in a **second repository-root shell**:

```bash
source ros2/runtime_env.sh
ros2 launch px4_position_mode position_takeoff_hover.launch.py finish:=land record:=true
```

Output: `bags/position_takeoff_hover/<timestamp>/`. With `finish:=hold`, the aircraft remains airborne when the recorder exits. The staging example uses `offboard_takeoff_handoff.launch.py` and writes under `bags/offboard_takeoff_handoff/`.

<!-- Preserve existing section links. -->
<span id="record-offboard-position"></span>
<span id="record-se3"></span>

<span id="record-offboard-position-or-se3"></span>

## Record Offboard Position or Offboard SE3

After preparing the aircraft for the chosen controller, use **one** launch in a **separate repository-root shell**. Offboard position:

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers offboard_position.launch.py \
  trajectory:=line_x record:=true
```

Offboard SE3, as a separate experiment:

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers se3.launch.py \
  vehicle:=gz_f450 trajectory:=full_excitation handoff:=acceleration \
  publish_diagnostics:=true record:=true
```

They create `bags/offboard_position/<timestamp>/` and `bags/se3/<timestamp>/` respectively. Both launches support `recording_output` for an explicit **new** directory; the study runner uses that argument for deterministic paths. Existing output paths are rejected rather than overwritten.

<!-- Preserve existing section links. -->
<span id="topic-ownership"></span>
<span id="se3-recording-scope"></span>

## Topic selection

PX4 topic names are symbolic entries in [config/px4_topics.def](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/config/px4_topics.def). The shared [recording lists](https://github.com/ANCL/px4-dev-toolkit/tree/0b668261e04732ef97e2965dbe8f5c1ba56730f0/config/recording) combine `common.txt` with the selected profile. They select catalog symbols and, where configured, absolute ROS diagnostic topic names. Both launch-owned and independent recording use this central configuration.

The Offboard SE3 list captures state/status, estimator/failsafe context, heartbeat, handoff setpoints, downstream PX4 setpoints, allocation/motors, and the generated trajectory. Diagnostics need `publish_diagnostics:=true` to produce samples; listing a topic does not cause a publisher to run.

<!-- Preserve existing section links. -->
<span id="finalization-contract"></span>
<span id="analyze-a-bag"></span>

## Finalization and inspection

Example directory structure (storage files depend on the installed rosbag backend):

```{ .text .no-copy }
bags/se3/<timestamp>/
├── metadata.yaml
├── experiment.json
├── <rosbag storage files>
└── analysis/                 # created later by px4_analyze
```

Launch-owned `experiment.json` records profile, configuration, and controller arguments; study runs include step identity. Independent recordings contain profile and topic-selection metadata, so retain onboard parameter and plan files separately.

In a **repository-root shell with ROS sourced**, find the most recent finalized standalone Offboard SE3 bag and inspect it:

```bash
bag_path="$(find bags/se3 -mindepth 2 -maxdepth 2 -name metadata.yaml -printf '%h\n' | sort | tail -n 1)"
test -n "$bag_path" && ros2 bag info "$bag_path"
```

Check duration, topic counts, and the recorded start/end of the experiment.

<!-- Preserve existing section links. -->
<span id="next"></span>

## Independent recording

With PX4 DDS available, start recording in its own repository shell:

```bash
source ros2/runtime_env.sh
./tools/px4_record onboard_se3
```

It stays in the foreground and does not change mode, arm, or start motion.
Start before mode selection; stop with Ctrl-C after landing, or run
`./tools/px4_record stop` from another shell. `./tools/px4_record status` reports
the active recording. Wait for the saved-path message and check both
`metadata.yaml` and `experiment.json`. The wrapper tracks one independent
recorder in `bags/.px4_record.json`; a stale state after a failed exit requires
inspection of the bag and process before removal. This tracking does not
prevent a separate launch-owned recorder, so avoid accidental duplicate bags.

The companion profile supports recording but omits numerical plotting and ULog
analysis dependencies. Copy completed bags/logs to the SITL analysis workstation.
See [Onboard logging](onboard-logging.md) for the different ROS-bag and ULog
execution intervals.

## Interrupted recording

After interruption, stop the recorder and preserve the bag/logs. If `metadata.yaml` is missing, confirm the process exited before attempting backend-supported `ros2 bag reindex`; inspect the recovered topics before analysis.

For a finalized run, use [Analyze a Run](../analysis/analysis.md).
