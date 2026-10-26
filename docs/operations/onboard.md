<span id="running-onboard-controllers"></span>

# Onboard control

Onboard controllers execute inside PX4 firmware. This procedure covers preparation, takeoff, mode selection, trajectory execution, and return using [Onboard SE3](../controllers/onboard/se3.md). [Tools](tools.md) explains the command terminals; ROS controllers use [Offboard control](offboard.md).

## Prepare before takeoff

Start the selected [SITL](sitl.md) or [companion environment](companion.md), then confirm that PX4, its state estimate, and the operator's controls are ready. The toolkit runtime commands run in an Ubuntu repository shell; use the shell inside the running container for Docker SITL.

Before arming:

1. Confirm that the installed firmware includes and has started the chosen controller. Starting a module and selecting its flight mode are separate actions.
2. Load the controller and vehicle parameters required for this aircraft.
3. Prepare and inspect any reference or trajectory used by that controller. Trajectory upload and flight-mode selection are separate actions.
4. Check the local coordinate frame, planned flight volume, pilot handback controls, and recording setup. An absolute trajectory does not move its origin to the aircraft's current position.


## Take off, select, execute, and return

Take off under pilot control and establish a stable Position-mode hover. Keep the pilot's Position-mode control available for handback.

Request the controller mode, then confirm PX4 entered it and the controller reports active control. For the current native SE3 mode, entry captures and holds the measured position and heading. A separately staged trajectory does not start on entry.

Start the reference explicitly and monitor tracking and controller status. SE3 stays active and holds the final commanded pose on completion; `se3_control hold` instead captures and holds the measured pose.

Return to Position mode with the pilot's configured mode control, verify the transition, then land and disarm using the normal pilot procedure. In the documented MAVProxy simulation, use the [SITL landing procedure](sitl.md#land-and-reset-the-simulated-vehicle). Stop environment services only after landing and disarming.

## SE3 example: native F450 SITL

Example: `gz_f450` SITL with the `full_excitation` trajectory. [Validation scope](../reference/architecture.md#validation-scope) lists the tested environments.

### 1. Configure and upload while disarmed

Start SITL using the [environment guide](sitl.md). In a **second repository-root Ubuntu shell**, run:

```bash
./tools/onboard_params load se3 gz_f450
./tools/trajectory upload full_excitation
```

The parameter command applies `config/controllers/onboard/se3/gz_f450.parm` through MAVLink and requires `SYS_AUTOSTART=4022`. Check its acknowledgements/readbacks and final result before continuing. A failed load may have applied earlier parameters; inspect and resolve that failure before flight.

Upload compiles `config/trajectories.yaml` into `generated/trajectories/full_excitation.traj` and transfers it to `/fs/microsd/trajectories/` over MAVLink FTP. Both commands require disarm.

The pinned SITL startup already starts `se3_control` and registers its mode, even though `SE3_ENABLE` defaults to zero. In the **PX4 terminal** (`px4` tmux window), inspect:

```text
se3_control status
trajectory list
```

Look for a registered Commander mode ID and `enabled=1`. `active=0` is expected before selecting the mode. `full_excitation.traj` should appear in the onboard catalog. Registration makes SE3 available for selection after takeoff.

### 2. Take off and check the absolute reference

Use pilot controls to take off in Position mode and settle near the plan's start: **PX4 local NED `[0, 0, -1.0]` m, yaw `0` rad**. Confirm those values in the actual PX4 estimate and inspect the plan's full maneuver volume before starting it. See [Trajectories](../experiments/trajectories.md) for segment meanings and absolute coordinates.

PX4 does **not** check proximity to this start pose when accepting the trajectory-start command. Starting from another position or heading creates an immediate tracking demand; the controller neither translates the plan to the current hover nor automatically aligns the aircraft first.

### 3. Select SE3 and verify the hold

In the **PX4 terminal**:

```text
se3_control select
se3_control status
commander status
```

Confirm Commander entered SE3 and `se3_control status` reports `active=1`, `faulted=0`, and `trajectory: holding`. Entry captures the measured pose; motion starts in the next step.

If selection is rejected or the controller remains inactive, inspect its [readiness requirements](../controllers/onboard/se3.md#readiness-and-faults). Do not continue with trajectory execution until the mode and active state are confirmed.

### 4. Explicitly start and monitor movement

In the **same PX4 terminal**:

```text
trajectory start full_excitation
se3_control status
```

`trajectory start` loads the named file and starts execution while SE3 is active and holding. Check for `trajectory: running`. `trajectory status` describes the staged plan; `se3_control status` reports execution state.

The supplied plan returns to its start pose, then finishes in SE3 hold. Land and disarm after pilot handback.

### 5. Interrupt or give control back

To stop the current reference execution and hold the measured position/heading **within SE3**, use:

```text
se3_control hold
se3_control status
```

For pilot handback, select Position mode with the pilot's mode control. In SITL, the following **PX4 terminal** command also requests Position mode:

```text
commander mode posctl
commander status
se3_control status
```

Confirm Position mode and SE3 `active=0`, then land and disarm. Do not stop the controller module or terminate simulator services as a way to end an airborne experiment.

## Applying the workflow to a companion computer

On hardware, use the [companion environment](companion.md) and `px4-shell` window or `./tools/px4_shell` for PX4 commands.

Confirm that the aircraft firmware contains the native controller and trajectory modules. SITL boot registration does not prove that a hardware firmware build starts the module. If it is present but stopped, `se3_control start` is a disarmed-only registration step; check its status before takeoff.

The repository supplies only a **commented hardware template**, `config/controllers/onboard/se3/f450_02.parm.example`. It is not a usable tuned flight profile. A validated `.parm` file, measured vehicle data, matching estimator/airframe configuration, and an established pilot procedure are prerequisites for physical flight. The `gz_f450` simulation profile is not a hardware tuning recommendation.

For recording and the distinction between PX4 ULog and ROS bags, continue with [Onboard flight logging](../experiments/onboard-logs.md).
