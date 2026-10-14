# Troubleshooting

Start from the symptom, inspect the relevant state, then fix the cause. Commands below assume the stated environment and a repository-root shell. Resolve communication/frame problems before changing controller gains.

<!-- Preserve existing section links. -->
<span id="gazebo-commandconfiguration-is-missing"></span>
<span id="qgroundcontrol-is-selected-but-unavailable"></span>

## Installation

| Symptom | Check | Next action |
|---|---|---|
| `ROS 2 workspace has not been built` | Base ROS installation and `ros2/install/setup.bash` | Complete the correct native profile, then `./ros2/build.sh sitl` or `./ros2/build.sh hardware` as appropriate; source the runtime helper |
| Source fetch refuses a partial scope | Exact manifests and existing external checkouts | Preserve local modifications, inspect which source is missing, and reconstruct in a clean checkout if needed; do not reset existing work |
| Gazebo command configuration not found/unusable | Installed Gazebo Harmonic dependencies | Use the supported `sitl` installation or `./setup/install_px4_dependencies.sh sitl` |
| QGroundControl configured but missing | Installed profile and `GCS` | Install QGroundControl with `./setup/install_qgroundcontrol.sh` when `GCS=qgc`; the `sitl` installation supplies MAVProxy by default |
| Container cannot write bags/logs | Bind-mount ownership and image UID/GID | Prepare writable host directories for the container user; keep the data mounts |

The source fetcher is intentionally non-destructive. Read [Native SITL installation](../installation/native.md) or [Companion-computer installation](../installation/companion.md) and inspect the reported source path before recovery.

<!-- Preserve existing section links. -->
<span id="toolssitl-rejects-the-command"></span>
<span id="sitl-rejects-gcs"></span>
<span id="ros-2-dds-topics-are-missing"></span>
<span id="a-stopped-tmux-session-leaves-gazebo-running"></span>

## Simulation and DDS

The SITL launcher accepts `start`, `start headless`, `status`, and `stop`; a bare invocation prints usage. `GCS` accepts exactly `mavproxy` or `qgc`, without `none`. Edit the canonical runtime file only after understanding the intended service changes.

If `/fmu/` topics are absent or have no samples, in a **second simulation repository-root shell**:

```bash
source ros2/runtime_env.sh
./tools/sitl status
./tools/topic_rates
```

Inspect `START_XRCE_AGENT`, transport/port agreement, PX4 XRCE client, and selected environment. Topic discovery and sample delivery are different checks. Make sure the controller shell is inside the same container when using Docker.

For simulation staging that waits without climbing, check MAVProxy and the virtual-joystick module, PX4 Position mode, readiness, and reported phase. Selecting QGroundControl does not provide the MAVProxy-backed virtual input used by those examples.

If a stopped tmux session leaves Gazebo running, use `./tools/sitl stop` **after flight has ended**. Its repository-scoped cleanup handles detached simulator processes; stopping communication is not a flight recovery action.

<!-- Preserve existing section links. -->
<span id="companion-computer-serial-device-is-missing"></span>
<span id="toolshardware-rejects-the-xrce-transport"></span>
<span id="viconmocap-data-is-missing"></span>

## Hardware and Vicon

| Symptom | Check | Next action |
|---|---|---|
| Serial device missing/inaccessible | Actual companion device, user permissions, configured baud and PX4 client | Correct `config/runtime/hardware.env` for the real device |
| `hardware XRCE transport must be 'serial'` | Configured transport | The current wrapper starts a serial agent; use its supported setting |
| Vicon tracked pose missing | Server reachability, object/topic, namespace, mocap window logs | Correct the site-specific selection; verify incoming pose |
| PX4 external vision misaligned | ENU/FLU input convention, NED/FRD conversion, configured world transform | Correct the transform/estimator setup; do not compensate with gains |

Use [Companion Computer](../environments/hardware.md) for service and flight-readiness checks. Companion process status does not prove estimator health, RC/failsafe behavior, or a valid actuator mapping.

<!-- Preserve existing section links. -->
<span id="controller-refuses-a-vehiclehandoff-combination"></span>

## Controller and vehicle

A missing `hover_thrust` or physical configuration is a mode-compatibility issue. Choose a supported mode from [vehicle requirements](adding-a-vehicle.md); calibrate/derive missing facts in separate vehicle work. Six-rotor data cannot satisfy the current four-rotor physical adapter.

If SE(3) waits for state, inspect position/velocity validity and the selected mode's attitude/rate needs. Its heartbeat can continue while state is invalid; heartbeat presence is not proof control output is running. If Offboard is rejected or lost, inspect PX4 estimator/mode status and acknowledgements. Confirm actual `nav_state`, not only an accepted mode command.

After a failed or interrupted run, restore a known vehicle state before retrying. Consult the controller's [lifecycle description](../controllers/se3.md#when-the-controller-starts-and-stops) instead of assuming every node arms, restarts, or lands identically.

<!-- Preserve existing section links. -->
<span id="sequence-runner-says-ros2-is-unavailable"></span>
<span id="comparison-output-cannot-be-inferred"></span>
<span id="next"></span>

## Recording and analysis

| Symptom | Check | Next action |
|---|---|---|
| Recorder exits before controller | Topic configuration, writable new bag path, recorder logs | Correct the recording cause before repeating a flight |
| Sequence says `ros2` is unavailable | Current shell environment | Source `ros2/runtime_env.sh` before `tools/run_sequence` |
| Step exited but bag is not finalized | Recorder exit and `metadata.yaml` | Preserve the bag; inspect/recover storage before analysis |
| Analysis profile cannot be identified | `experiment.json` or standalone parent name | Use explicit study metadata or the documented standalone layout |
| SE(3) analysis lacks entry/return | Recorded status and experiment completion | Do not present an interrupted/incomplete interval as a normal comparison |
| Comparison requests `--output` | Shared study `bags/` parent and manifest | For standalone/different-study bags, provide an explicit destination |
| A plot/metric is unavailable | Required versus optional topics, diagnostics enabled, overlap | Interpret only supported observations; absence is not zero error |

Read [Recording](../workflows/recording.md) before attempting interrupted-bag recovery, and [Comparison](../analysis/comparison.md) for metric windows and data requirements.
