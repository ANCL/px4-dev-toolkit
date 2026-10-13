# Complete SITL experiment

This guide connects environment startup, a reference flight, an external controller, recording, analysis, and a six-mode study. Use a **native Ubuntu SITL installation**, default F450 configuration, and MAVProxy. Docker users can run the repository-root steps inside the container shells from the [Docker guide](../installation/docker.md).

The examples are source-verified. No armed simulation or measured result was produced for this documentation update. Run each demonstration only after checking the actual vehicle state; they are not an unattended script to paste in sequence.

## 1. Start and verify services

In **terminal 1 at the repository root**:

```bash
./tools/sitl start
```

In **terminal 2 at the same root**:

```bash
source ros2/runtime_env.sh
./tools/sitl status
./tools/topic_rates
```

Stop the rate display with Ctrl-C. Verify valid position/status and the intended simulated vehicle using the [first simulation checks](../getting-started/first-simulation.md#2-check-communication-and-state). Keep terminal 1 attached to logs or detach tmux without stopping it.

## 2. Record a PX4 reference flight

With the aircraft initially landed/disarmed and no competing controller, in **terminal 2**:

```bash
ros2 launch px4_position_mode position_takeoff_hover.launch.py record:=true
```

Wait for takeoff, hover, landing, and confirmed disarm. The recorder exits after the node. Inspect the printed bag path and metadata before moving on. This bag uses the `position_takeoff_hover` analysis profile; it is not a SE(3) comparison bag.

## 3. Prepare a separate Offboard demonstration

Check that the reference flight has ended and the simulated vehicle is ready again. If it failed, restore or restart the simulation in a landed/disarmed state before proceeding. In **terminal 2**:

```bash
ros2 launch px4_position_mode offboard_takeoff_handoff.launch.py
```

Wait for the staging hold phase. In **terminal 3 at the repository root**, start one independent controller:

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers se3.launch.py \
  vehicle:=gz_f450 trajectory:=line_tracking handoff:=acceleration \
  publish_diagnostics:=true record:=true
```

The staging helper exits after confirmed takeover. SE(3) follows the relative line trajectory and returns to Position mode, then recording shuts down. It does not land/disarm. An Offboard-position demonstration is an alternative controller example, described on its [own page](../controllers/offboard-position.md); do not run both together.

## 4. Inspect and analyze the finalized bag

In **terminal 3 after launch and recorder have exited**, still at the repository root with ROS sourced:

```bash
bag_path="$(find bags/se3 -mindepth 2 -maxdepth 2 -name metadata.yaml -printf '%h\n' | sort | tail -n 1)"
test -n "$bag_path" && ros2 bag info "$bag_path"
test -n "$bag_path" && ./tools/px4_analyze "$bag_path"
```

Check that this is the run just recorded, especially if older bags exist. `analysis/summary.txt` and available PNG plots are written **inside that bag**. Establish confirmed Offboard entry/return and topic completeness first, then inspect reference versus measured position, errors, rates, collective, and motor limits. The [analysis guide](../analysis/analysis.md) provides the interpretation and labeled plot slot.

## 5. Run the six-mode study as a new experiment

End the separate demonstration, land/disarm through the prepared simulation controls, and restore the required initial condition. Prepare the aircraft with the staging helper again; only begin the study once it is holding in Position mode. The study needs the full F450 physical data because its final mode uses the four-rotor adapter.

In a **separate repository-root shell**:

```bash
source ros2/runtime_env.sh
./tools/run_sequence config/sequences/se3/handoff_comparison.yaml
```

The runner executes the six recorded `full_excitation` flights in order, returning to Position mode between steps and waiting 3 s. It does not land, reset, or arm between steps. Monitor state and space for the complete path. Read [Running a series of experiments](../workflows/sequences.md) for interruption and manifests.

## 6. Compare finalized study bags

In a **repository-root shell after successful study completion**, with ROS sourced:

```bash
study_path="$(find bags/se3_handoff_comparison -mindepth 2 -maxdepth 2 -name manifest.json -printf '%h\n' | sort | tail -n 1)"
test -n "$study_path" && cat "$study_path/manifest.json"
```

Verify that this is the intended completed study and all six steps succeeded. Then, in the **same shell**:

```bash
mapfile -t study_bags < <(find "$study_path/bags" -mindepth 2 -maxdepth 2 -name metadata.yaml -printf '%h\n' | sort)
test "${#study_bags[@]}" -eq 6 && ./tools/px4_compare "${study_bags[@]}" --trajectory full_excitation
```

The command filters finalized bag directories rather than expanding unrelated files. Results go to `$study_path/comparison/summary.txt` and PNG figures. Verify the six metadata identities before interpreting overlays; [comparison](../analysis/comparison.md) aligns time to PX4-confirmed Offboard entry and explains which metrics are meaningful.

## 7. End the flight and stop services

A completed study leaves Position mode, not a landed/disarmed aircraft. Land/disarm the simulated aircraft with the prepared controls, confirm that state, then in a **repository-root shell**:

```bash
./tools/sitl stop
./tools/sitl status
```

Keep the bags, manifests, and the configurations used for the run. No performance ranking follows merely from completing this procedure; it requires valid recorded data and interpretation.
