<!-- Preserve existing section links. -->
<span id="geometric-se3-controller"></span>

# SE(3) Control

The toolkit tracks an SE(3) trajectory and hands control to PX4 at one of
four levels: acceleration, attitude, body rate, or thrust/torque. **The handoff
level determines what the toolkit calculates and what PX4 still controls.**
PX4 performs actuator allocation in every mode. The current geometric reference assumes an **underactuated multirotor** with a single collective thrust axis; it is not an independent six-degree-of-freedom position/attitude controller. The physical-wrench adapter is still restricted to four rotors.

<!-- Preserve existing section links. -->
<span id="runtime-flow"></span>
<span id="control-ownership"></span>
<span id="six-configurations"></span>

## Choose a handoff

| Toolkit output | PX4 handles | Details |
|---|---|---|
| **Acceleration + yaw** | Attitude/thrust conversion, rate control, allocation | [Acceleration](se3/acceleration.md) |
| **Attitude + thrust** | Attitude/rate control, allocation | [Attitude](se3/attitude.md) |
| **Body rate + thrust** | Rate control, allocation | [Body Rate](se3/attitude-rate.md) |
| **Thrust + torque** | Allocation | Three laws below |

The **Thrust & Torque** interface has three distinct implementations:

- **[Geometric (Normalized)](se3/geometric-normalized.md)** — computes dimensionless PX4 torque from geometric attitude/rate errors.
- **[Geometric (Physical)](se3/geometric-physical.md)** — computes a body moment in N·m and converts it through a vehicle-specific adapter.
- **[PX4 Rate PID](se3/px4-rate.md)** — reproduces the pinned PX4 rate-controller behavior inside the toolkit.

These laws all publish normalized PX4 thrust/torque setpoints; a physical body
moment is **not** sent directly to PX4. Read the [Math](se3/foundations.md)
page for frames, equations, and assumptions.

<!-- Preserve existing section links. -->
<span id="launch"></span>

<span id="run-a-controller"></span>

## Run

Prepare an airborne, state-ready SITL aircraft with
[Offboard Staging](offboard-staging.md). SE(3) does **not** arm or take off.
Ensure the selected vehicle file matches the simulator and no other Offboard
controller is running. In a second repository-root shell:

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers se3.launch.py \
  vehicle:=gz_f450 trajectory:=hover handoff:=acceleration \
  publish_diagnostics:=true
```

`hover` holds the configured absolute NED position for 10 seconds. Choose another
[trajectory](trajectories.md) for motion; add `record:=true` for a bag.
Each mode page shows its exact launch arguments.

<span id="when-the-controller-starts-and-stops"></span>

## Activation

`offboard_trigger:=automatic` (default) keeps the existing SITL mode-request
workflow; `offboard_trigger:=external` publishes its readiness heartbeat and
waits for PX4 to confirm the pilot's Offboard mode change. Neither mode arms
the vehicle. Both start the trajectory on confirmed Offboard entry. Automatic
mode requests Position on completion; external mode holds the final setpoint
until the pilot selects Position mode. The `hover` trajectory holds the configured absolute NED start position and heading; it does not use the measured entry pose. **The SE(3) handoff setpoint is not
prestreamed before Offboard**: higher-level/low-level handoff transitions
require separate SITL verification before physical flight.

## Lifecycle

1. Wait for valid state; publish the Offboard heartbeat and a diagnostic
   trajectory reference. The warmup is 1.5 s (100 Hz outer loop by default). Automatic
   activation retries mode requests every 1 s; external activation waits
   for PX4-confirmed Offboard entry.
2. **Start control output only after PX4 confirms Offboard** through
   `VehicleStatus.nav_state`. Unlike Offboard Position, SE(3) does not
   prestream its selected acceleration, attitude, rate, or wrench setpoint.
3. At trajectory completion, automatically request Position mode or hold the
   final setpoint for pilot-selected handback, depending on `offboard_trigger`.
   Finish only when PX4 confirms Position mode.

Offboard loss or invalid state can interrupt control. A subsequent confirmed
Offboard entry restarts the trajectory at the same absolute NED start; PX4-rate internal state
is invalidated after Offboard loss. Returning to Position mode **does not land
or disarm the aircraft**. Follow the [SITL landing procedure](../environments/sitl.md#land-and-reset-the-simulated-vehicle).
These readiness checks are not a complete flight failsafe.

<!-- Preserve existing section links. -->
<span id="vehicle-support"></span>

## Vehicle support

- **`gz_f450`:** all six configurations, including the four-rotor physical adapter.
- **`gz_x500`, `gz_px4vision`:** acceleration, attitude, body rate, normalized geometric torque, and PX4 Rate PID; no curated physical propulsion mapping.
- **`gz_fy690s`:** acceleration only with current curated data. Its six rotors are not supported by the four-rotor physical adapter.

Physical moment control uses a propulsion model rather than `MPC_THR_HOVER`.
See [Adding a Vehicle](../reference/adding-a-vehicle.md) before transferring
settings to another airframe.

<!-- Preserve existing section links. -->
<span id="frames-units-and-timing"></span>
<span id="tuning-ownership"></span>
<span id="diagnostics"></span>
<span id="recording-and-comparison"></span>
<span id="next"></span>

<span id="read-the-mathematics"></span>

## Configuration and data

When launched inside a toolkit checkout, SE(3) reads controller gains,
trajectory definitions, and vehicle YAML directly from that checkout at process
startup. Editing these files requires restarting the controller, **not**
rebuilding ROS 2. The exact source paths are logged at launch. Packaged YAML is
used only for a standalone installation without the checkout and is reported.
Keep a copy or hash of experiment YAML alongside each flight recording: changing
trajectory segment definitions after a flight changes comparison boundaries.


Controller gains and timing: [controller.yaml](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/config/se3/controller.yaml).
Vehicle parameters belong in vehicle files. Diagnostics report calculated
signals; [analysis](../analysis/analysis.md) distinguishes references,
measured state, and reconstructed quantities.
