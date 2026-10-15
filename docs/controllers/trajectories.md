# Trajectories: absolute PX4 local NED targets

The YAML `ros2/src/offboard_controllers/config/trajectory/trajectories.yaml`
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

The loader refuses any curve that would start away from the preceding
commanded endpoint, and rejects legacy `offset` and `yaw_offset` keys. Use
an explicit step/line before a curve to reach its start. A named trajectory
can override `start_position`; a reusable segment can be selected directly.

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

Mathematics: smooth lines, yaw, and periodic paths use ninth-order time
scaling with analytic derivatives through snap; steps are discontinuous.
The terminal reference is held after the configured duration. Both Offboard
Position and SE(3) use the same trajectory YAML parser and sampler. See
[Offboard Position](offboard-position.md) and [SE(3)](se3.md).
