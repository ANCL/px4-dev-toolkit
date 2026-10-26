<!-- Preserve established section links. -->
<span id="geometric-se3-controller"></span>
<span id="se3-control"></span>

# Offboard SE(3)

The ROS 2 SE(3) controller tracks a named trajectory and sends PX4 one of four command levels. Each level changes the feedback loops retained by PX4. [Onboard SE3](onboard-se3.md) is a separate native firmware mode.

The geometric reference assumes an underactuated multirotor with one collective thrust axis. Position and heading determine a compatible attitude; this is not independent control of arbitrary position and attitude.

<span id="runtime-flow"></span>
<span id="control-ownership"></span>
<span id="six-configurations"></span>

## Choose a handoff

<div class="px4-table-scroll" markdown="1">

| Handoff and law | ROS command | PX4 retains |
|---|---|---|
| [`acceleration`](se3/acceleration.md) | NED acceleration and yaw | Attitude/thrust conversion, attitude/rate loops, allocation |
| [`attitude`](se3/attitude.md) | Attitude and collective thrust | Attitude/rate loops, allocation |
| [`attitude_rate`](se3/attitude-rate.md) | Body rate and collective thrust | Rate loop, allocation |
| [`thrust_and_torque`](se3/geometric-normalized.md) · `geometric_normalized` | Normalized torque and thrust | Allocation |
| [`thrust_and_torque`](se3/geometric-physical.md) · `geometric_physical` | Physical moment mapped to normalized commands | Allocation |
| [`thrust_and_torque`](se3/px4-rate.md) · `px4_rate` | Reproduced rate PID output | Allocation |

</div>

`thrust_and_torque` supports three laws. Only `geometric_physical` computes an inertia-based moment in N·m; all three publish normalized PX4 commands. See the [geometric mathematics](../mathematics/geometric-control.md).

<span id="launch"></span>
<span id="run-a-controller"></span>

## Run

Follow [Offboard control](../operations/offboard.md) to prepare state, take off, choose entry behavior, and plan recovery. This node never arms or takes off. With a matching, airborne SITL vehicle near the configured trajectory start, use a separate Ubuntu repository-root shell:

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers se3.launch.py \
  vehicle:=gz_f450 trajectory:=hover handoff:=acceleration \
  publish_diagnostics:=true
```

`hover` holds the configured NED `[0, 0, -1]` m and yaw 0 for 10 seconds. Other references are in [Trajectories](trajectories.md). Add `record:=true` to save a ROS bag under `bags/se3/`; `recording_output` overrides the destination.

<span id="when-the-controller-starts-and-stops"></span>

## Activation

`offboard_trigger:=automatic` is the default: after armed, valid state and warmup, the node requests Offboard and retries every second. `offboard_trigger:=external` waits for the pilot to select Offboard. Both start the trajectory clock only on **PX4-confirmed Offboard entry** in `VehicleStatus.nav_state`, not on launch or command acknowledgement. Neither option arms the aircraft.

During warmup SE3 publishes its heartbeat and diagnostic `/se3/trajectory_reference`, **not the selected PX4 control setpoint**. Setpoint publication begins after confirmed Offboard entry.

At trajectory completion, automatic mode requests Position, retrying every second while continuing control output. External mode holds the final trajectory reference and continues its feedback output until the pilot selects Position. A confirmed Position return completes the node. Returning to Position does not land or disarm.

## Lifecycle

The outer wall timer runs at `control_rate_hz` (100 Hz by default). Only armed, state-ready iterations count toward the configured 1.5-second heartbeat warmup. Position/velocity validity is always required; attitude and angular-rate requirements depend on the handoff. `hover` additionally requires valid attitude, including with the acceleration handoff. The reproduced PX4 rate PID uses a separate gyro-sample clock for torque publication.

If required state becomes invalid or the aircraft disarms, the timer stops publishing its heartbeat/output and resets the warmup count. There is no finite initial-readiness or mode-return deadline in this implementation. These checks do not check sample age and are not a complete flight failsafe. PX4's configured Offboard-loss behavior remains relevant.

Losing Offboard invalidates the held PX4-rate target. A later accepted, ready Offboard entry restarts the trajectory at its configured absolute start. This differs from [Offboard Position](offboard-position.md), which fails on an unexpected departure during tracking. Do not use re-entry as an automatic continuation of an interrupted trajectory. Confirm pilot control, land and disarm using the [SITL exit procedure](../environments/sitl.md#land-and-reset-the-simulated-vehicle).

## Vehicle support

The checked-in vehicle files provide mass and hover-thrust calibration for acceleration, attitude, body-rate, normalized-geometric, and PX4-rate control. `geometric_physical` additionally needs full propulsion and allocator data, currently available for `gz_f450` only; its adapter supports four rotors. See [Geometric (Physical)](se3/geometric-physical.md#vehicle-data-and-limits) and [Adding a vehicle](../reference/adding-a-vehicle.md).

<span id="frames-units-and-timing"></span>
<span id="tuning-ownership"></span>
<span id="diagnostics"></span>
<span id="recording-and-comparison"></span>
<span id="next"></span>
<span id="read-the-mathematics"></span>

## Configuration and data

The launch reads gains, trajectories, and vehicle YAML from the checkout at startup (or installed packaged files outside a checkout). Restart after YAML edits. [Offboard Staging](offboard-staging.md) uses installed configuration.

Gains and timing belong to [controller.yaml](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/ros2/src/offboard_controllers/config/se3/controller.yaml); physical airframe/allocator data belong to the vehicle file. Keep the actual YAML or its hash with each recording because later trajectory edits change comparison boundaries. Diagnostics report calculated signals; [analysis](../analysis/analysis.md) distinguishes references, measured state, and reconstructed quantities.

For implementation details, [runtime/control.cpp](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/ros2/src/offboard_controllers/src/se3/runtime/control.cpp) selects each control boundary and [runtime/lifecycle.cpp](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/ros2/src/offboard_controllers/src/se3/runtime/lifecycle.cpp) owns entry, timing, and return. The handoff pages explain the corresponding equations and message fields.
