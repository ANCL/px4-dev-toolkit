<span id="attitude-rate"></span>

# Body Rate

The toolkit computes an FRD body-rate request and collective thrust. PX4 retains body-rate feedback and allocation; the command is not a physical moment.

## Desired rate and feedback command

The common force $A$ determines $R_d$. [Analytic force/basis derivatives](../../mathematics/geometric-control.md#analytic-force-derivatives), using desired jerk and yaw rate, determine

$$
\Omega_d=\bigl(R_d^\top\dot R_d\bigr)^\vee.
$$

Map the desired-frame angular velocity into the current FRD frame and add feedback:

$$
\begin{aligned}
e_R&=\tfrac12(R_d^\top R-R^\top R_d)^\vee \\
\Omega_{\mathrm{sp}}&=R^\top R_d\Omega_d-K_{\mathrm{att}}e_R.
\end{aligned}
$$

$K_{\mathrm{att}}$ is the diagonal vector `attitude_gain`, in s⁻¹; the output is current FRD rad/s. The desired rate alone is feedforward. The commanded rate adds a correction that PX4's rate loop then tracks. No inertia or $\Omega\times J\Omega$ term enters this law.

## Messages, scale, and timing

`VehicleRatesSetpoint.roll`, `.pitch`, and `.yaw` are commanded angular rates about forward/right/down axes, not Euler angles. `thrust_body=(0,0,-u_T)` uses

$$
u_T=\operatorname{clip}_{[0,1]}\left(h\frac{-A^\top R e_3}{mg}\right).
$$

`reset_integral=false` leaves PX4 integral reset decisions to its native logic. Only `OffboardControlMode.body_rate` is true. The message uses publication time in microseconds.

Mass and curated hover thrust are required. Readiness checks valid position/velocity and attitude; this cascaded mode does not require a measured body-rate sample for its law. The outer timer publishes at the configured rate (100 Hz by default) after confirmed entry. It uses first force derivatives; it does not use snap-based desired angular acceleration. Shared [startup and return behavior](../se3.md#when-the-controller-starts-and-stops) applies.

## Run and interpret

In a **separate repository-root shell**, with an already airborne, state-ready SITL vehicle and no competing controller:

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers se3.launch.py \
  vehicle:=gz_f450 trajectory:=figure_eight_tracking handoff:=attitude_rate \
  publish_diagnostics:=true record:=true
```

Compare desired/mapped rate, commanded rate, and measured rate to separate attitude feedback from PX4 rate tracking. Singular force/heading geometry limits analytic feedforward.

Read [attitude_control.cpp](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/ros2/src/offboard_controllers/src/se3/control/attitude_control.cpp), [attitude_reference.cpp](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/ros2/src/offboard_controllers/src/se3/control/attitude_reference.cpp), and [publication.cpp](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/ros2/src/offboard_controllers/src/se3/runtime/publication.cpp).
