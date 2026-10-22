<span id="complete-sitl-experiment"></span>
# SITL experiment

Run a PX4 reference flight, an SE(3) Offboard flight, and a recorded six-step handoff study. The commands assume **native Ubuntu SITL**, F450, and MAVProxy; Docker users run them in the corresponding [container shells](../installation/docker.md).

Check mode, arming, and state between flights. A finished controller is not proof the aircraft is landed or ready for another run.

## 1. Start SITL

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

## 2. Record the reference flight

With the aircraft initially landed/disarmed and no competing controller, in **terminal 2**:

```bash
ros2 launch px4_position_mode position_takeoff_hover.launch.py record:=true
```

Wait for takeoff, hover, landing, and confirmed disarm. The recorder exits after the node. Inspect the printed bag path and metadata before moving on. This bag uses the `position_takeoff_hover` analysis profile; it is not a SE(3) comparison bag.

## 3. Run SE(3) Offboard

Check that the reference flight has ended and the simulated vehicle is ready again. If it failed, restore a known landed/disarmed state before proceeding. In **terminal 2**:

```bash
ros2 launch px4_position_mode offboard_takeoff_handoff.launch.py
```

Wait for the [Offboard Staging](../controllers/offboard-staging.md) hold phase. In **terminal 3 at the repository root**, start one independent controller:

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers se3.launch.py \
  vehicle:=gz_f450 trajectory:=line_tracking handoff:=acceleration \
  publish_diagnostics:=true record:=true
```

The staging helper exits after confirmed takeover. SE(3) follows the configured absolute NED line trajectory and returns to Position mode, then recording shuts down. It does not land/disarm. An Offboard-position demonstration is an alternative controller example, described on its [own page](../controllers/offboard-position.md); do not run both together.

<span id="4-inspect-and-analyze-the-finalized-bag"></span>
## 4. Analyze the bag

In **terminal 3 after launch and recorder have exited**, still at the repository root with ROS sourced:

```bash
bag_path="$(find bags/se3 -mindepth 2 -maxdepth 2 -name metadata.yaml -printf '%h\n' | sort | tail -n 1)"
test -n "$bag_path" && ros2 bag info "$bag_path"
test -n "$bag_path" && ./tools/px4_analyze "$bag_path"
```

Check that this is the run just recorded, especially if older bags exist. `analysis/summary.txt` and available PNG plots are written **inside that bag**. Establish confirmed Offboard entry/return and topic completeness first, then inspect reference versus measured position, errors, rates, collective, and motor limits. The [analysis guide](../analysis/analysis.md) provides the interpretation and labeled plot slot.

## 5. Run the six-step handoff study

End the separate demonstration, [land/disarm using the MAVProxy SITL procedure](../environments/sitl.md#land-and-reset-the-simulated-vehicle), and restore the required initial condition. Prepare the aircraft with the staging helper again; only begin the study once it is holding in Position mode. The study uses the configured F450 vehicle but does not execute the four-rotor physical-wrench adapter.

In a **separate repository-root shell**:

```bash
source ros2/runtime_env.sh
./tools/run_sequence config/sequences/se3/handoff_comparison.yaml
```

The runner records Offboard Position followed by the five nonphysical SE(3) `full_excitation` flights, returning to native Position mode between steps and waiting 3 s. It does not land, reset, or arm between steps. Monitor state and space for the complete path. Read [Running a series of experiments](../workflows/sequences.md) for interruption and manifests.

<span id="6-compare-finalized-study-bags"></span>
## 6. Compare the study

In a **repository-root shell after successful study completion**, with ROS sourced:

```bash
study_path="$(find bags/se3_handoff_comparison -mindepth 2 -maxdepth 2 -name manifest.json -printf '%h\n' | sort | tail -n 1)"
test -n "$study_path" && cat "$study_path/manifest.json"
```

Verify that this is the intended completed study and all six steps succeeded. Then, in the **same shell**:

```bash
mapfile -t study_bags < <(find "$study_path/bags" -mindepth 2 -maxdepth 2 -name metadata.yaml -printf '%h\n' | sort)
test "${#study_bags[@]}" -eq 6 && ./tools/px4_compare "${study_bags[@]}"
```

The command filters finalized bag directories rather than expanding unrelated files. The six comparable Offboard bags produce `$study_path/comparison/summary.txt` and PNG figures. Verify the six metadata identities before interpreting overlays; [comparison](../analysis/comparison.md) aligns time to PX4-confirmed Offboard entry and explains which metrics are meaningful.

## 7. Land and stop SITL

A completed study leaves Position mode, not a landed/disarmed aircraft. [Land/disarm the simulated aircraft with MAVProxy and verify PX4 state](../environments/sitl.md#land-and-reset-the-simulated-vehicle), then in a **repository-root shell**:

```bash
./tools/sitl stop
./tools/sitl status
```

Keep the bags, manifests, and the configurations used for the run. Base comparisons on the recorded state, data coverage, and actuator limits described in the analysis chapters.
