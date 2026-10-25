# Trajectories: absolute PX4 local NED targets

The YAML `config/trajectories.yaml`
uses `frame: px4_local_ned` and a fixed `start_position` and `default_yaw`.
**These coordinates never shift when the aircraft enters Offboard.** PX4's
local frame is estimator-relative and can reset: validate the frame and start
position before each real flight.

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
heading fields. A named trajectory can override `start_position`; a reusable
segment can also be selected directly.

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

The first position is fixed even if the vehicle begins Offboard elsewhere.
**Do not use flight excitation until the pilot verifies the aircraft's
PX4 local origin, safe maneuver volume, and the launch's selected YAML file.**

Offboard controllers use the toolkit's `trajectory_generator/` C++ library
to parse YAML and sample references. Native controllers use the bounded sampler
in PX4's `src/lib/trajectory/`; `tools/trajectory` compiles the shared YAML
definitions into onboard plan files.

Smooth lines, yaw, and periodic paths use ninth-order time scaling with
analytic derivatives through snap; steps are discontinuous. Completed
sequences hold their final reference. See
[Offboard Position](offboard-position.md) and [SE(3)](se3.md).

## Compiling trajectories for PX4-native flight modes

The same `config/trajectories.yaml` defines Offboard and onboard experiments.
The standard hover reference is `[0, 0, -1.0]` PX4 local NED, yaw zero.
Every named trajectory returns smoothly to that point and yaw before its
last hold. Vertical experiments use `-0.75 <= z <= -1.5` m; actual aircraft
motion must be evaluated against available flight space.

```bash
# Toolkit shell, after installation and PX4 SITL build:
./tools/trajectory list
./tools/trajectory upload --all
```

The uploader compiles bounded, checksummed `.traj` plans under
`generated/trajectories/` (gitignored) and transfers them through MAVProxy
using MAVLink FTP on either SITL or hardware. The files contain analytic segment
parameters, not pre-sampled points. Neither operation changes flight mode.

In the PX4 shell, while **disarmed**, start the SE(3) module after configuring
its existing parameters. Then perform the normal Position-mode takeoff and
hover. The following are separate operations:

```text
trajectory list
trajectory select circle_r05
trajectory status
se3_control select       # Request the registered mode; verify acceptance
se3_control run          # Start selected trajectory after start-pose checks
se3_control status       # Observe running / completed / holding
se3_control hold         # Optional interruption; holds measured position
commander mode posctl    # Explicit return to PX4 Position mode
```

Completion **does not exit SE(3)**. The sampler holds the final commanded
reference, equal to `[0, 0, -1.0]` for all provided named plans, until you
switch modes or command `hold`. On initial SE(3) entry, before `run`, it holds
the measured handoff pose instead. A loaded plan must begin sufficiently near
the current measured pose and heading; no implicit frame translation occurs.

See [Onboard flight logging](../workflows/onboard-logging.md) for ULog recording and analysis.

Both SITL and hardware use MAVLink FTP through the running MAVProxy
session. `tools/trajectory clean` removes compiled and remotely uploaded
trajectory files while the vehicle is disarmed. Files on PX4 persistent storage survive restart,
but selected/executing plan state does not.
