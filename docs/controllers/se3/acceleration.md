# Acceleration

The toolkit calculates position/velocity feedback and sends a **NED acceleration request** to PX4. PX4 converts it to attitude/thrust and runs its lower feedback loops and control allocation. This mode requires the least vehicle data: mass.

## Translation law

With $e_x=x-x_d$ and $e_v=v-v_d$ in NED:

$$
a_{\mathrm{cmd}}=a_d-\frac{k_x}{m}e_x-\frac{k_v}{m}e_v.
$$

All terms are in m/s². At zero hover error the request is zero acceleration; PX4 applies gravity compensation below this interface. See the [force derivation](../../mathematics/geometric-control.md#translation-and-gravity-compensation).

`kx_over_mass` and `kv_over_mass` are the configured gains; the runtime multiplies them by vehicle mass to obtain $k_x,k_v$. It checks that mass and gains are finite and positive.

## Messages and timing

Only `OffboardControlMode.acceleration` is true; the other control-level flags are false. The active `TrajectorySetpoint` contains:

<div class="px4-table-scroll" markdown="1">

| Field | Value |
|---|---|
| `acceleration` | $a_{\mathrm{cmd}}$, NED m/s² |
| `yaw` | Desired trajectory heading, rad |
| `position`, `velocity`, `jerk` | NaN on all axes |
| `yawspeed` | NaN |
| `timestamp` | Publication time in microseconds |

</div>

**NaN means unused.** Zero position or velocity would enable a different PX4 interpretation. The separately published `/se3/trajectory_reference` contains the full desired trajectory for analysis and is not this PX4 input.

The outer wall timer publishes at `control_rate_hz` (100 Hz by default) after Offboard confirmation. Readiness requires finite position/velocity plus all PX4 validity flags; attitude/rate state is not required for this handoff, except that the runtime also requires valid attitude when the selected trajectory is `hover`. The shared [startup/return behavior](../se3.md#when-the-controller-starts-and-stops) applies.

## Run and inspect

With a prepared airborne SITL vehicle, matching mass file, and no competing controller, in a **separate repository-root shell**:

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers se3.launch.py \
  vehicle:=gz_f450 trajectory:=line_x handoff:=acceleration \
  publish_diagnostics:=true record:=true
```

Inspect position/velocity tracking, commanded acceleration, and PX4 attitude/thrust. Lower-loop response and saturation also affect tracking error.

The calculation is in [translation.cpp](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/ros2/src/offboard_controllers/src/se3/control/translation.cpp); exact fields are in [runtime/publication.cpp](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/ros2/src/offboard_controllers/src/se3/runtime/publication.cpp), selected by [runtime/control.cpp](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/ros2/src/offboard_controllers/src/se3/runtime/control.cpp).
