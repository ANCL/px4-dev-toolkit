<span id="trajectories-absolute-px4-local-ned-targets"></span>
<span id="trajectories-and-reference-motions"></span>

# Trajectories

Both control paths use **absolute PX4 local NED** position and heading
references, but they have **independent trajectory configurations**.

## ROS 2 Offboard trajectories

`config/trajectories.yaml` is the Offboard catalog. ROS 2 controllers load a
named sequence at startup; the host-side `trajectory_generator/` library
samples analytic position, velocity, acceleration, jerk, snap, yaw and its
derivatives. No compiled plan or MAVLink upload is required.

The YAML declares `frame: px4_local_ned`, a `start_position`, a `default_yaw`,
reusable segments, and named trajectories. A named trajectory may override
`start_position`.

- `hold` holds the previous commanded pose.
- `step` immediately changes the position reference to an absolute `position`.
- `line` smoothly reaches an absolute `position`.
- `yaw` interpolates to an absolute `heading` in radians.
- `circle` and `helix` use an absolute `center`; entry is
  `center + [radius, 0, 0]`. The helix has a signed NED height change.
- `figure_eight` starts/ends at its absolute `center`.

Curve entry must match the preceding commanded endpoint; use an explicit
line or step to reach it. Smooth segments use ninth-order time scaling with
analytic derivatives through snap. The sequence holds its final reference
after completion.

Example:

```yaml
frame: px4_local_ned
start_position: [0.0, 0.0, -1.5]
default_yaw: 0.0
segments:
  initial: {type: hold, duration: 2.0}
  north: {type: step, duration: 4.0, position: [1.0, 0.0, -1.5]}
  home: {type: line, duration: 6.0, position: [0.0, 0.0, -1.5]}
trajectories:
  test: {segments: [initial, north, home]}
```

The Offboard clock starts at confirmed Offboard entry. See
[Offboard control](../operations/offboard.md).

## Native PX4 trajectories

The firmware owns a separate **built-in catalog** and sampler in
`ANCL/PX4-Autopilot/src/lib/trajectory/`. Configure geometry, duration,
start pose and yaw with PX4 `TRJ_*` parameters. No YAML compilation, `.traj`
files, or FTP transfer are involved.

For example, in the PX4 shell:

```text
param show TRJ_HOME_Z
param show TRJ_HOME_YAW
param show TRJ_F8_T
param set TRJ_HOME_Z -2.5
```

After takeoff and confirmed SE3 entry (`active=1`, `faulted=0`):

```text
se3_control trajectory figure_eight_tracking
se3_control status
```

`se3_control hold` interrupts execution and captures the measured pose. On
completion, SE3 holds the trajectory's final commanded pose. References never
automatically align to the aircraft's measured position; verify the start
pose, heading and flight volume first. See [Onboard control](../operations/onboard.md).

## Comparing the paths

Matching trajectory **names do not guarantee matching motions**. For example,
the Offboard YAML and native `TRJ_*` parameters can specify different
figure-eight durations and step amplitudes. Match their geometry, duration,
yaw and origin explicitly before comparing controller tracking.

Do not change the Offboard YAML when tuning native parameters, or change
native parameters when tuning Offboard. Preserve both configurations and
the PX4 firmware revision with experiment results.
