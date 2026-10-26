<span id="running-offboard-controllers"></span>

# Offboard control

ROS 2 Offboard controllers stream setpoints to PX4 over uXRCE-DDS. This procedure covers [Offboard Position](../controllers/offboard/position.md) and [Offboard SE3](../controllers/offboard/se3/index.md), from takeoff through handback. For firmware modes, see [Onboard control](onboard.md).

## Prepare the runtime and aircraft

Start [SITL](sitl.md) or the [companion stack](companion.md), and confirm DDS state messages and a valid PX4 local estimate.

Before launching a controller:

1. Select the vehicle configuration, trajectory, handoff level, entry trigger, and recording output. Run only one Offboard command publisher for the aircraft.
2. Check the selected trajectory in `config/trajectories.yaml`. Positions are **absolute PX4 local NED**, metres, with positive Z down. The pinned start is `[0,0,-1]` m and yaw 0; `hover` holds that point for 10 seconds. The reference never shifts automatically to the current hover pose.
3. Prepare pilot control and the intended Position-mode recovery, landing, and disarming path. Verify estimator validity, mode selection, and the configured Offboard-loss response. Neither controller's readiness checks establish that the flight volume or failsafe settings are suitable.
4. Take off under the prepared pilot/flight-control workflow and establish an airborne Position hold near the configured start. The ROS controller will wait for arming, but it has no takeoff phase.

For the first standard **SITL** hover near the local origin with `GCS=mavproxy`, this helper performs takeoff and then exits while retaining the hold:

```bash
source ros2/runtime_env.sh
ros2 launch px4_position_mode position_takeoff_hover.launch.py \
  height:=1.0 finish:=hold
```

Run in an Ubuntu repository shell. The helper climbs 1 m relative to takeoff and holds for 10 seconds; verify the resulting position and heading. The alternative [Offboard Staging](../controllers/px4-reference/staging.md) helper targets Z = −2 m. Use one preparation helper per flight.

The MAVProxy virtual-joystick helpers are for simulation only. On physical hardware, use the aircraft’s established pilot-controlled takeoff, handoff, and recovery procedures.

## Launch the selected implementation

Open a separate Ubuntu repository-root shell in the same prepared runtime, source its ROS environment, and launch one controller. These examples use the fixed `hover` reference so that entry behavior can be inspected before selecting a moving trajectory.

**Offboard Position** keeps translational and lower feedback in PX4:

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers offboard_position.launch.py \
  trajectory:=hover offboard_trigger:=automatic record:=true
```

**Offboard SE(3)** calculates translational feedback in ROS. Its acceleration handoff leaves attitude/thrust conversion and lower feedback in PX4:

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers se3.launch.py \
  vehicle:=gz_f450 trajectory:=hover handoff:=acceleration \
  offboard_trigger:=automatic publish_diagnostics:=true record:=true
```

Select the matching vehicle file. For attitude, body-rate, or thrust/torque output, see [SE3 handoffs](../controllers/offboard/se3/index.md#choose-a-handoff) and their state/data requirements.

Both controllers load checkout YAML at startup; restart after edits. `record:=true` starts a ROS bag. See [Recording](../experiments/recording.md) for ULog and ROS-bag differences.

## Enter Offboard and run the reference

The shared trigger argument changes who requests the mode:

<div class="px4-table-scroll" markdown="1">

| Trigger | Entry | At reference completion |
|---|---|---|
| `offboard_trigger:=automatic` (default) | Controller requests Offboard after readiness/warmup | Controller requests Position |
| `offboard_trigger:=external` | Pilot selects Offboard after readiness/warmup | Controller continues final-reference output until the pilot selects Position |

</div>

For pilot-selected entry, change the example to `offboard_trigger:=external`. Wait for readiness/prestream and then use the verified RC/joystick/GCS Offboard switch. Both paths require PX4 to report `NAVIGATION_STATE_OFFBOARD`; a command ACK is not enough. Neither trigger requests arming.

The implementations deliberately differ at the transition:

<div class="px4-table-scroll" markdown="1">

| | Offboard Position | Offboard SE(3) |
|---|---|---|
| Before entry | 20 Hz heartbeat **and actual position/feedforward setpoints**; 30 warmup samples | 100 Hz default heartbeat and diagnostic trajectory reference for 1.5 s; **selected PX4 control output begins after entry** |
| Reference time | Starts on confirmed entry | Starts on confirmed, ready entry |
| Automatic mode requests | One entry request, one return request; 5 s confirmation deadlines | Entry retries every 1 s; return retries every 1 s; no fixed confirmation deadline |
| Unexpected departure during tracking | Experiment fails and exits | Stops active output; a later valid entry restarts the reference |

</div>

SE3's diagnostic reference is separate from its active PX4 setpoint. See the [SE3 lifecycle](../controllers/offboard/se3/index.md#lifecycle) for readiness and mode-loss behavior.

## Monitor, return, and land

Monitor measured local NED position/heading, arming state, and `VehicleStatus.nav_state` alongside the controller log. For SE(3), optional diagnostics help distinguish the desired reference, calculated feedback/feedforward, and actual PX4 input. Use the selected [analysis workflow](../experiments/analysis.md) to interpret tracking and actuator limits.

Confirm the Position-mode return before ending the experiment. In external mode, the controller continues tracking the final reference until handback.

Then land and disarm using the prepared flight controls. For simulation, follow [Land and reset](sitl.md#land-and-reset-the-simulated-vehicle) before a new run or environment shutdown. Controller completion, rosbag completion, and returning to Position do not themselves land the aircraft.

If tracking, state, or mode is unexpected, recover through the prepared pilot control path and inspect actual vehicle state. Interrupting ROS can remove the stream and invoke PX4's configured failsafe; it is not a landing command. The [Offboard Position lifecycle](../controllers/offboard/position.md#readiness-timing-and-return) and [SE(3) lifecycle](../controllers/offboard/se3/index.md#lifecycle) explain their different exit behavior.
