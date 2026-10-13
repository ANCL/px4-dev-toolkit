# Trajectories

A trajectory defines the requested motion, independently of controller gains. [Offboard position](offboard-position.md) and [SE(3)](se3.md) use the same [trajectories.yaml](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/config/trajectory/trajectories.yaml) and analytic sampler.

## Coordinate convention

Positions and derivatives are local NED: x North, y East, z Down. Offsets/radii/heights/amplitudes are metres, durations seconds, yaw radians, and `turns` revolutions. A negative Z offset moves upward. Segment yaw is independent of the spatial path; a circle does not automatically turn the aircraft to face its tangent.

## Anchoring and time origin

Trajectory time starts on **confirmed PX4 Offboard entry**, not process start or mode request. During preparation, the time-zero reference follows the current position. SE(3) anchors the latest position at confirmation; Offboard position retains its last pre-entry origin. Yaw comes from the selected configuration.

If SE(3) loses Offboard and later re-enters, it re-anchors and restarts. Offboard position instead fails on unexpected mode loss. This distinction matters when interpreting recorded time histories.

## Segment semantics

Each segment starts at the preceding segment's terminal position/yaw. At an exact cumulative boundary, the sampler uses the **ending segment**; the next starts immediately afterward. The sampler clamps negative time to zero and holds terminal position/yaw after the entire sequence ends. Controller lifecycle can then request a mode return.

For smooth segments, local time $t\in[0,T]$ uses $u=t/T$ and the ninth-order scale

$$
\begin{aligned}
s(u)&=126u^5-420u^6+540u^7 \\
&\quad-315u^8+70u^9.
\end{aligned}
$$

Its first four derivatives vanish at both endpoints. The code differentiates it and the path analytically through **snap** (fourth position derivative). This is not constant-speed motion or interpolation of a recorded path. The implementations are [sampling.cpp](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/trajectory/sampling.cpp) and [trajectory.cpp](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/trajectory/trajectory.cpp).

### Hold

$x_d=x_0$, with zero derivatives and fixed heading. It holds the segment origin; it does not take off to a default altitude.

### Step

$x_d=x_0+\Delta x$ immediately after the segment boundary. The code returns zero velocity/acceleration/jerk/snap for this stationary target. The position jump is intentionally discontinuous; those zeros do not imply a physically smooth displacement. A step at the beginning of an entire sequence is already applied at time zero.

### Line

$$
\begin{aligned}
x_d&=x_0+s\Delta x \\
v_d&=\dot s\Delta x \\
a_d&=\ddot s\Delta x.
\end{aligned}
$$

Jerk/snap follow the same derivatives of $s$. The path is a straight line with smooth start/stop speed.

### Circle and helix

For radius $r$ and $n$ turns, define $\theta=2\pi n s$. The path is

$$
x_d=x_0+(r(\cos\theta-1),\ r\sin\theta,\ Hs)^\top.
$$

For a circle, $H=0$; for a helix, `height` is $H$. The circle center lies one radius in **negative NED x** from the origin, so the segment starts at its origin without a jump. Smooth phase scaling changes angular speed over time. A negative helix height climbs. Non-integer turns can end at a different horizontal position.

### Yaw

$$
\begin{aligned}
x_d&=x_0 \\
\psi_d&=\psi_0+s\Delta\psi.
\end{aligned}
$$

Yaw rate/acceleration are $\dot s\Delta\psi$ and $\ddot s\Delta\psi$. Position stays fixed; yaw changes smoothly.

### Figure eight

With $\theta=2\pi n s$:

$$
x_d=x_0+(a_x\sin\theta,\ a_y\sin(2\theta),\ 0)^\top.
$$

The 1:2 phase relationship produces a planar Lissajous figure eight. Chain-rule derivatives through snap use the same scaled phase. The path crosses its origin; heading remains the segment origin yaw.

<!-- Preserve existing section links. -->
<span id="extending"></span>
<span id="analysis-relationship"></span>
<span id="next"></span>

## Checked-in trajectories

A **type** (`line`) describes the sampler. A **segment name** (`line_x_2m`) supplies parameters. A **trajectory name** (`line_tracking`) orders segment names. The loader also accepts a segment name directly as a one-segment trajectory.

This complete example uses the checked-in `line_tracking` values and can be saved as a standalone trajectory configuration:

```yaml
default_yaw: 0.0
segments:
  hold_2s:
    type: hold
    duration: 2.0
  line_x_2m:
    type: line
    duration: 4.0
    offset: [2.0, 0.0, 0.0]
trajectories:
  line_tracking:
    segments: [hold_2s, line_x_2m, hold_2s]
```

The full catalog also defines `hover`, `step_response`, `circle_tracking`, `helix_tracking`, `translation_yaw_sweep`, `figure_eight_tracking`, and `full_excitation`. Full excitation combines holds, single-axis steps, yaw changes, a figure eight, and return motion. It is a study excitation, not a universal physical-flight path.

From a **separate repository-root shell**, with a prepared airborne SITL vehicle and no competing controller:

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers se3.launch.py \
  vehicle:=gz_f450 trajectory:=line_tracking handoff:=acceleration
```

The standard launch loads the installed trajectory configuration; rebuild ROS after editing the source YAML. `px4_compare --trajectory full_excitation` resolves the canonical source YAML for segment windows, so retain the configuration used to record the study. Editing it afterward can mislabel old data. Continue with [A complete SITL experiment](../experiments/complete-example.md).
