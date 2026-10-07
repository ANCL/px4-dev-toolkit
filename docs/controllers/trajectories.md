# Trajectories

[Documentation](../index.md) / Controllers / Trajectories

The shared trajectory configuration is:

```text
ros2/src/offboard_controllers/config/trajectory/trajectories.yaml
```

It is used by both the independent Offboard position controller and the SE(3) controller.

## Coordinate convention

Trajectory references use PX4 local **NED** coordinates:

- `x`: North
- `y`: East
- `z`: Down

Therefore an upward displacement has a negative `z` offset.

## Anchoring and time origin

A configured trajectory is anchored to the vehicle position when PX4 **confirms Offboard entry**.

During prestream, the reference remains at trajectory time `t = 0` and follows the current local position. The trajectory does not start advancing merely because the controller process started or because it requested Offboard.

If SE(3) loses Offboard and later regains confirmed ownership, it re-anchors and restarts the trajectory.

## Segment semantics

Supported segment types currently include:

- `hold`
- `step`
- `line`
- `circle`
- `helix`
- `yaw`
- `figure_eight`

Units in the YAML are:

```text
duration                 seconds
offset/radius/height     metres
amplitude                 metres
yaw/yaw_offset            radians
turns                     revolutions
```

At an exact segment boundary, the segment that is ending owns that sample; the next segment begins immediately afterward. After the final segment, the terminal position/yaw is held indefinitely.

## Checked-in trajectories

The current named trajectories are:

```text
hover
step_response
line_tracking
circle_tracking
helix_tracking
translation_yaw_sweep
figure_eight_tracking
full_excitation
```

Use a name directly as a launch argument:

```bash
ros2 launch offboard_controllers offboard_position.launch.py \
  trajectory:=line_tracking
```

or:

```bash
ros2 launch offboard_controllers se3.launch.py \
  vehicle:=gz_f450 \
  trajectory:=figure_eight_tracking \
  handoff:=acceleration
```

## Extending

To add new segments or named trajectories, follow [Extending the toolkit](../reference/extending.md#add-a-trajectory).


## Analysis relationship

`px4_compare --trajectory <name>` resolves this same configuration so it can compute segment-aware metrics against the configured maneuver boundaries.

## Next

- [SE(3)](se3.md)
- [Sequences](../workflows/sequences.md)
- [Comparison](../analysis/comparison.md)
