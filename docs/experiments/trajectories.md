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

The firmware has a separate [SE3-local trajectory catalog](https://github.com/ANCL/PX4-Autopilot/tree/lab/main/src/modules/se3_control/trajectory/).
The four built-in names and PX4 parameters are independent of ROS 2 YAML:

| Name | Native reference |
|---|---|
| `setpoint` | Absolute `TRJ_SP_X/Y/Z/YAW`; live edits are immediate steps |
| `steps5` | Five hardcoded absolute NED points; `TRJ_STEP_HOLD` per point |
| `circle` | `TRJ_HOME_*` center and `TRJ_CIR_R/T/TURNS` |
| `figure8` | `TRJ_HOME_*` center and `TRJ_F8_AX/AY/T/TURNS` |

`steps5` uses `(0, 0, -0.75)` → `(0.25, 0, -0.75)` →
`(0, 0, -0.75)` → `(0, 0.25, -0.75)` → `(0, 0, -0.75)` metres.
It does not use `TRJ_HOME_*`. Circle begins on the +X edge of its center;
figure-eight begins at its center.

After takeoff and confirmed SE3 entry, having first checked the starting
absolute NED pose and flight volume, use the **PX4 shell**:

```text
se3_control select
se3_control status
se3_control trajectory figure8
```

SE3 must be active and holding to start a trajectory. `se3_control hold`
interrupts motion and captures the **measured** position and heading. A
completed finite trajectory holds its final **commanded** pose. Only the
active `setpoint` follows subsequent `TRJ_SP_*` edits; all other trajectory
parameters are snapshotted at the start request.

No separate sampler process, trajectory upload or native YAML configuration
is required. See [Onboard SE3](../controllers/onboard/se3.md) for defaults
and [Onboard control](../operations/onboard.md) for the procedure.

## Comparing the paths

Matching trajectory **names do not guarantee matching motions**. For example,
the Offboard YAML and native `TRJ_*` parameters can specify different
figure-eight durations and step amplitudes. Match their geometry, duration,
yaw and origin explicitly before comparing controller tracking.

Do not change the Offboard YAML when tuning native parameters, or change
native parameters when tuning Offboard. Preserve both configurations and
the PX4 firmware revision with experiment results.
