<span id="complete-sitl-experiment"></span>
# SITL experiment

Run a PX4 reference flight, an Offboard SE3 flight, and a recorded six-step handoff study. The commands assume **native Ubuntu SITL**, F450, and MAVProxy; Docker users run them in the corresponding [container shells](../installation/docker.md).

Check actual PX4 mode, arming, and position before each subsequent flight.

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

Check valid vehicle state using [First simulation](../getting-started/first-simulation.md#2-check-communication-and-state).

## 2. Record the reference flight

With the aircraft initially landed/disarmed and no competing controller, in **terminal 2**:

```bash
ros2 launch px4_position_mode position_takeoff_hover.launch.py finish:=land record:=true
```

Wait for takeoff, hover, landing, and confirmed disarm. The recorder exits after the node. Inspect the printed bag path and metadata before moving on. This bag uses the `position_takeoff_hover` analysis profile; it uses a different lifecycle from the Offboard comparison bags.

<span id="3-run-se3-offboard"></span>

## 3. Run Offboard SE3

With the previous flight complete and the aircraft landed/disarmed, in **terminal 2**:

```bash
ros2 launch px4_position_mode position_takeoff_hover.launch.py height:=1.0 finish:=hold
```

Wait for the helper's takeoff and hover to finish, leaving the aircraft armed in Position mode. Check the actual PX4 local estimate before launching the controller: `line_x` starts at **NED `[0, 0, -1]` m, yaw `0` rad**. The helper commands a relative climb, so its completion alone does not prove that absolute pose; use pilot controls to establish the intended position and heading. The separate [Offboard staging helper](../controllers/offboard-staging.md) defaults to −2 m and also does not align X/Y/yaw. Do not hand over from that default hover without reconciling the start.

In **terminal 3 at the repository root**, start one independent controller only after this check:

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers se3.launch.py \
  vehicle:=gz_f450 trajectory:=line_x handoff:=acceleration \
  publish_diagnostics:=true record:=true
```

Offboard SE3 follows the absolute NED line trajectory, returns to Position mode, and finalizes the bag. Land separately. For the alternative position-reference example, see [Offboard Position](../controllers/offboard-position.md).

<span id="4-inspect-and-analyze-the-finalized-bag"></span>
## 4. Analyze the bag

In **terminal 3 after launch and recorder have exited**, still at the repository root with ROS sourced:

```bash
bag_path="$(find bags/se3 -mindepth 2 -maxdepth 2 -name metadata.yaml -printf '%h\n' | sort | tail -n 1)"
test -n "$bag_path" && ros2 bag info "$bag_path"
test -n "$bag_path" && ./tools/px4_analyze "$bag_path"
```

Confirm the selected bag belongs to this run. `analysis/summary.txt` and PNG figures appear inside the bag. Check Offboard entry/return, topic completeness, and tracking in the [analysis guide](../analysis/analysis.md).

## 5. Run the six-step handoff study

End the separate demonstration, [land/disarm using the MAVProxy SITL procedure](../environments/sitl.md#land-and-reset-the-simulated-vehicle), and restore the required initial condition. Repeat the `height:=1.0 finish:=hold` preparation above, then check the measured absolute **NED `[0, 0, -1]` m and yaw `0` rad** required by `full_excitation`. Only begin once the F450 is holding at that intended start in Position mode; the sequence does not reposition it before its first handoff.

In a **separate repository-root shell**:

```bash
source ros2/runtime_env.sh
./tools/run_sequence config/sequences/se3/handoff_comparison.yaml
```

The runner records Offboard Position and five Offboard SE3 `full_excitation` flights, with Position-mode return and a 3-second settle between steps. It does not reset or land between flights. See [Sequences](../workflows/sequences.md) for step definitions, recovery, and manifests.

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

The six finalized bags produce `$study_path/comparison/summary.txt` and PNG overlays. Check each bag's metadata; [Comparison](../analysis/comparison.md) explains alignment and metrics.

## 7. Land and stop SITL

A completed study leaves Position mode, not a landed/disarmed aircraft. [Land/disarm the simulated aircraft with MAVProxy and verify PX4 state](../environments/sitl.md#land-and-reset-the-simulated-vehicle), then in a **repository-root shell**:

```bash
./tools/sitl stop
./tools/sitl status
```

Keep the bags, manifests, and the configurations used for the run. Base comparisons on the recorded state, data coverage, and actuator limits described in the analysis chapters.
