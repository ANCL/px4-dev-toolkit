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
A plain-text [command sheet](../workflows/onboard-commands.txt) collects the
Ubuntu, PX4-shell, and analysis commands of this procedure in one place.

### 1. Inspect the native configuration

```text
se3_control status
se3_control list
param show TRJ_*
```

On supported targets, SE3 registers at boot but is initially inactive.
The four trajectory names are `setpoint`, `steps5`, `circle`, and `figure8`.

### 2. Take off and verify the starting pose

```text
commander takeoff
listener vehicle_local_position
```

Wait for stable Position-mode hover. SITL may take off to NED Z near
`-2.5` m, while `TRJ_HOME_Z` defaults to `-0.75` m. Match the
**measured** position and intended flight volume. For example, if appropriate:

```text
param set TRJ_HOME_Z -2.5
```

Confirm X, Y, Z, and yaw. Figure-eight starts at `TRJ_HOME_*`; circle
starts at `(TRJ_HOME_X + TRJ_CIR_R, TRJ_HOME_Y, TRJ_HOME_Z)`.
Neither reference automatically aligns to the aircraft's measured pose.

### 3. Select SE3 and confirm holding

```text
se3_control select
se3_control status
commander status
```

Require `active=1`, `faulted=0`, and `trajectory: holding`. Mode entry
captures the measured pose without starting a trajectory.

### 4. Start and monitor

```text
se3_control trajectory figure8
se3_control status
```

There is no extra `start` argument. To interrupt the current trajectory
and hold the **measured** pose:

```text
se3_control hold
se3_control status
```

For a separate live test, set `TRJ_SP_*` near the measured hover pose
before `se3_control trajectory setpoint`. While it runs, changing those
parameters produces immediate position/yaw reference steps. The `steps5`
trajectory uses five fixed NED points near `(0, 0, -0.75)`, independently
of `TRJ_HOME_*`; verify their entire flight volume before starting it.

### 5. Return to Position and land

```text
commander mode posctl
commander status
commander land
```

Confirm Position mode, then wait for landing/disarm. Do not stop the
controller module or simulator/companion services while airborne.

## Hardware companion

Use QGroundControl's MAVLink Console or the PX4 MAVLink shell over an
available hardware MAVLink connection for native commands. Confirm the
installed firmware includes/starts SE3 and registers its Commander mode. Hardware requires validated aircraft-specific PX4 parameters, estimator,
actuator and hover-thrust calibration. The toolkit does not provide a hardware
SE3 parameter profile.

For recording and independent ULog/ROS bag analysis, see
[Onboard logging](../experiments/onboard-logs.md).
