# Attitude

The toolkit sends desired attitude and collective thrust to PX4. PX4 retains attitude/rate feedback and allocation. This mode requires a curated hover-thrust scale.

## Force to rotation

Use the common translation errors and acceleration law:

$$
\begin{aligned}
A&=m(a_{\mathrm{cmd}}-g e_3) \\
b_{3d}&=-A/\|A\|.
\end{aligned}
$$

Desired heading $b_{1c}=(\cos\psi_d,\sin\psi_d,0)^\top$ fixes the remaining attitude:

$$
\begin{aligned}
b_{2d}&=\frac{b_{3d}\times b_{1c}}{\|b_{3d}\times b_{1c}\|} \\
b_{1d}&=b_{2d}\times b_{3d} \\
R_d&=[b_{1d}\ b_{2d}\ b_{3d}].
\end{aligned}
$$

$R_d$ maps desired FRD axes into NED. The [foundations](../../../mathematics/index.md#desired-attitude-from-force-and-heading) explain degeneracy and the frame conventions. The runtime converts $R_d$ to a normalized scalar-first quaternion $q_d=(w,x,y,z)$, using trace/major-diagonal branches to avoid an unstable single formula near 180° rotations.

## Collective projection and message

Collective uses the **current** body axis, not the norm of desired force:

$$
\begin{aligned}
f&=-A^\top R e_3 \\
u_T&=\operatorname{clip}_{[0,1]}\left(h f/(mg)\right).
\end{aligned}
$$

`VehicleAttitudeSetpoint.q_d` contains the FRD-to-NED quaternion. `thrust_body=(0,0,-u_T)` is dimensionless; negative FRD Z means upward thrust. `yaw_sp_move_rate` contains desired trajectory yaw rate in rad/s. Only `OffboardControlMode.attitude` is enabled.

Hover scale $h$ comes from `px4_allocator.hover_thrust` in vehicle YAML. Collective projection/clipping can reduce thrust during attitude error. PX4 runs attitude/rate feedback.

## Timing, run, and diagnosis

Valid position/velocity and attitude are required. The outer timer (100 Hz by default) publishes after confirmed Offboard; jerk/snap are not needed to construct $R_d$ here. Shared [lifecycle behavior](index.md#when-the-controller-starts-and-stops) still applies.

In a **separate repository-root shell**, with a matching, already airborne SITL vehicle and no competing controller:

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers se3.launch.py \
  vehicle:=gz_f450 trajectory:=line_x handoff:=attitude \
  publish_diagnostics:=true record:=true
```

Inspect desired/measured attitude, force command, normalized collective, downstream rates, and motor limits. The law rejects invalid hover calibration and degenerate force/heading geometry.

Read [attitude_reference.cpp](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/se3/control/attitude_reference.cpp), [wrench.cpp](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/px4/wrench.cpp), and [publication.cpp](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/se3/runtime/publication.cpp).
