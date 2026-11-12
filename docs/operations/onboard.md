<span id="running-onboard-controllers"></span>

# Onboard control

Onboard controllers run in PX4 firmware. Native SE3 owns its execution clock,
trajectory selection, and current-position hold. The toolkit supplies the
environment, logging, and analysis; no trajectory
files are uploaded. ROS 2 controllers use [Offboard control](offboard.md).

## Prepare before takeoff

Start the chosen [SITL](sitl.md) or [companion](companion.md) environment.
Confirm the PX4 firmware contains SE3, the estimator is valid, and the pilot
can switch back to Position mode.

Native SE3 gains and trajectory settings are PX4 firmware parameters.
Use `param show`, `param set` and `param save` in the PX4 shell, or QGroundControl,
to inspect and configure the aircraft while disarmed.

Native trajectories are configured with `TRJ_*` PX4 parameters, independently
of `config/trajectories.yaml` (which is for ROS 2 Offboard).

## Take off, select, execute, and return

Take off under pilot control and establish a stable Position-mode hover.
Then select SE3 and confirm PX4 actually entered the mode, not just that a
request was published. Entry captures and holds the measured position and
heading; it does not start motion.

Start a named native trajectory explicitly. It uses absolute local NED
coordinates and does **not** shift its origin to the current hover. Completion
holds the final commanded pose, while `se3_control hold` captures the current
measured pose.

To return, select Position mode using the pilot's configured control and
confirm it. Only then land and disarm. For SITL, use
[Landing](sitl.md#land-and-reset-the-simulated-vehicle).

## SE3 example: native F450 SITL

The following commands are entered in the **PX4 shell** (`pxh>`), not Ubuntu.

### 1. Inspect the native configuration

```text
se3_control status
param show TRJ_HOME_X
param show TRJ_HOME_Y
param show TRJ_HOME_Z
param show TRJ_HOME_YAW
param show TRJ_F8_T
```

On supported targets, SE3 registers at boot and its default `SE3_ENABLE`
is `1`. A registered mode is initially inactive.

### 2. Take off and verify the start pose

```text
commander takeoff
listener vehicle_local_position
```

Wait for a stable hover in a valid Position flight mode. The simulator's
default takeoff may reach NED Z about `-2.5`, while `TRJ_HOME_Z` defaults
to `-1.0`. Match the intended trajectory's absolute altitude and yaw to
the verified flight volume before execution; for example:

```text
param set TRJ_HOME_Z -2.5
```

The number is an **example**, not an automatic measurement. Confirm local
X/Y and yaw as well. The firmware intentionally has no proximity/alignment
guard on a trajectory start.

### 3. Select SE3 and confirm holding

```text
se3_control select
se3_control status
commander status
```

Require `active=1`, `faulted=0`, and `trajectory: holding` before proceeding.

### 4. Start and monitor

```text
se3_control trajectory figure_eight_tracking
se3_control status
```

The new firmware uses its built-in catalog; `se3_control trajectory <name>`
is the only trajectory start interface. To interrupt movement:

```text
se3_control hold
se3_control status
```

### 5. Return to Position and land

```text
commander mode posctl
commander status
commander land
```

Confirm Position mode, then wait for landing/disarm. Do not stop the
controller module or companion/simulator services while airborne.

## Hardware companion

Use the [companion environment](companion.md) and `./tools/px4_shell`
or the hardware `px4-shell` window. Confirm the installed firmware
includes/starts SE3 and registers its Commander mode. Hardware requires validated aircraft-specific PX4 parameters, estimator,
actuator and hover-thrust calibration. The toolkit does not provide a hardware
SE3 parameter profile.

For recording and independent ULog/ROS bag analysis, see
[Onboard logging](../experiments/onboard-logs.md).
