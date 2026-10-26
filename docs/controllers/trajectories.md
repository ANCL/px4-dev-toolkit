<span id="trajectories-absolute-px4-local-ned-targets"></span>

<span id="trajectories-and-reference-motions"></span>

# Trajectories

`config/trajectories.yaml` defines named trajectories in absolute `px4_local_ned` coordinates using `start_position` and `default_yaw`. References remain in that frame when control begins; verify the PX4 local origin before execution.

## Absolute YAML commands

- `step` names the complete target `position: [north, east, down]`; it is an
  **instantaneous discontinuity**, not a smooth or speed-limited move.
- `line` names the complete target `position`; it moves smoothly from the
  previous commanded endpoint to that target.
- `hold` holds the previous *absolute* command, starting with `start_position`.
- `yaw` names an absolute `heading` in radians; no accumulation of yaw offsets.
- `figure_eight` names an absolute `center`; it starts and ends at that center.
- `circle` and `helix` name an absolute `center`; their initial position is
  `center + [radius, 0, 0]`. A circle ends at the corresponding final phase;
  a helix additionally applies its configured signed height.

A curve must start at the preceding commanded endpoint. Use an explicit
line or step to reach its start. YAML uses absolute position, center and
heading fields. A named trajectory can override `start_position`; the Offboard loader also accepts a reusable
segment name directly. The public onboard CLI accepts named entries under
`trajectories`, so wrap a segment in a named sequence for deployment.

For example:

```yaml
frame: px4_local_ned
start_position: [0.0, 0.0, -1.5]
default_yaw: 0.0
segments:
  hold_initial: {type: hold, duration: 2.0}
  north: {type: step, duration: 4.0, position: [1.0, 0.0, -1.5]}
  home: {type: line, duration: 6.0, position: [0.0, 0.0, -1.5]}
trajectories:
  test: {segments: [hold_initial, north, home]}
```

Before running a moving trajectory, check the selected plan, aircraft start pose, and maneuver volume.

Offboard controllers use the toolkit's `trajectory_generator/` C++ library
to parse YAML and sample references. Native controllers use the bounded sampler
in PX4's `src/lib/trajectory/`; `tools/trajectory` compiles the shared YAML
definitions into onboard plan files.

Smooth lines, yaw, and periodic paths use ninth-order time scaling with
analytic derivatives through snap; steps are discontinuous. Completed
sequences hold their final reference. The derivatives feed position/feedforward control and geometric attitude/rate
references; see the [shared geometric derivation](../mathematics/geometric-control.md).

## Two execution paths

The single named YAML definition is used by two runtimes:

<div class="px4-table-scroll" markdown="1">

| Path | Preparation | Execution |
|---|---|---|
| Offboard | A ROS 2 launch resolves the named YAML trajectory | The ROS controller samples it and streams setpoints through uXRCE-DDS; trajectory time starts at confirmed Offboard entry |
| Onboard | The toolkit compiles a `.traj` file and uploads it over MAVLink FTP | PX4 samples the uploaded plan after an explicit `trajectory start <name>` command |

</div>

See [Offboard](../operations/offboard.md) or [Onboard](../operations/onboard.md) for flight procedures.

<span id="compiling-trajectories-for-px4-native-flight-modes"></span>

## Compile and upload an onboard plan

In an **Ubuntu repository-root shell**, inspect the catalog and compile the
specific plan before connecting to PX4:

```bash
./tools/trajectory list
./tools/trajectory compile full_excitation
```

Compilation writes `generated/trajectories/full_excitation.traj` and needs no
PX4 connection. The artifact contains analytic segment parameters and a
checksum, rather than sampled points. The native format allows at most 32
segments with durations of at least 0.01 s, finite float32 values, continuous
curve entry positions, and a final pose matching the absolute start pose
within the compiler's closure tolerance. A deliberately discontinuous `step`
is still permitted; closure does not make that step smooth.

With the chosen [SITL](../environments/sitl.md) or
[hardware](../environments/hardware.md) environment running and PX4 disarmed:

```bash
./tools/trajectory upload full_excitation
```

Upload recompiles the plan and transfers it to `/fs/microsd/trajectories/full_excitation.traj` via MAVLink FTP. It requires a disarmed PX4 and checks transfer completion. The uploader CRC32 covers the local file; PX4's ULog `plan_crc32` covers the embedded plan body. Uploading does not start motion.

## Start an uploaded plan

Uploading a `.traj` file does **not** start motion. In the PX4 shell, `trajectory list` shows onboard files; the command `trajectory start <name>` requests execution only when the native SE3 mode is already active and holding. The Ubuntu `./tools/trajectory list` command instead lists local YAML definitions.

A plan starts at its **absolute** PX4 local NED coordinates and heading. There is no repositioning, translation to the current hover, or proximity guard. Confirm the aircraft's current pose, heading, local origin, and safe maneuver volume before execution. The [Onboard control procedure](../operations/onboard.md#se3-example-native-f450-sitl) owns the full mode-selection, start, monitoring, hold, and handback sequence; [Onboard SE3](onboard-se3.md) describes how the firmware implements it.

## Retain and remove plans

Uploaded `.traj` files persist on PX4 storage. `./tools/trajectory clean` requires disarm and removes remote catalog plans and locally compiled files. Retain the plan identity with [recordings](../workflows/onboard-logging.md).
