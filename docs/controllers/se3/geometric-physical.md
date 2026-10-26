# Geometric (Physical)

This Offboard path evaluates the classical rigid-body geometric moment in N·m, then converts the requested physical wrench to PX4-normalized thrust and torque. PX4 still performs its normal allocation. The adapter is part of the toolkit implementation, separate from the geometric feedback law.

## Physical moment

Use the [shared derivation](../../mathematics/geometric-control.md) for force $A$, desired attitude $R_d$, desired angular dynamics, and the error definitions. In current FRD coordinates:

$$
\begin{aligned}
Q&=R^\top R_d,\\
e_R&=\tfrac12(R_d^\top R-R^\top R_d)^\vee,\\
e_\Omega&=\Omega-Q\Omega_d,\\
\alpha_{\mathrm{ff}}&=-\Omega\times(Q\Omega_d)+Q\dot\Omega_d.
\end{aligned}
$$

The requested wrench is

$$
\begin{aligned}
f&=-A^\top R e_3,\\
M&=-K_R e_R-K_\Omega e_\Omega+\Omega\times(J\Omega)+J\alpha_{\mathrm{ff}}.
\end{aligned}
$$

$f$ is collective thrust in N; $M$ is a current-FRD body moment in N·m. `geometric_physical_k_r` and `geometric_physical_k_omega` are diagonal gains with units N·m and N·m·s when radians are dimensionless. $J$ is the full symmetric inertia tensor in FRD kg·m². Unlike [Geometric (Normalized)](geometric-normalized.md), this law includes inertia feedforward and gyroscopic coupling. It requires desired dynamics through snap and yaw acceleration.

## Physical wrench to PX4

The adapter performs three steps using the selected vehicle's model:

1. Solve for physical rotor thrusts using rotor arms relative to the center of mass and signed reaction-moment/thrust ratios. Bound collective to the propulsion envelope first, then uniformly scale the complete requested moment if necessary to keep every rotor within its thrust bounds.
2. Invert the monotonic actuator curve $T(u)=c_0+c_1u+c_2u^2$ [N] on its configured dimensionless control interval. This gives the normalized motor controls that produce those rotor thrusts in the model.
3. Express those controls in the pinned PX4 allocator's normalized wrench coordinates, then publish thrust/torque setpoints for PX4 allocation.

For a rotor at FRD arm $(x_i,y_i,z_i)$ with upward force $(0,0,-T_i)$, its physical contribution is $(-y_iT_i,x_iT_i,\kappa_iT_i,T_i)$ in roll moment, pitch moment, yaw moment, and positive collective. The physical effectiveness matrix and PX4 allocator matrix use distinct physical/allocator coefficients. A rotor index or yaw-sign mismatch would change the mapping.

Writing $B$ for the allocator effectiveness matrix and $S$ for its reproduced normalization scales, the published four-coordinate wrench is

$$
c_n=S\,B\,u.
$$

This matches the adapter's target inverse mixer $B^{-1}S^{-1}$ for a conventional quadrotor. It is **not** division of each physical moment by an assumed maximum torque. The adapter reproduces the relevant normalization from [PX4 `ControlAllocationPseudoInverse.cpp` at `9af25b30`](https://github.com/ANCL/PX4-Autopilot/blob/9af25b30f067c36269e94a96797be7943dd4f8e1/src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp). Matching allocator settings and motor order remain prerequisites.

The published `VehicleThrustSetpoint.xyz` is $(0,0,c_{n,z})$, with negative body Z for upward thrust; `VehicleTorqueSetpoint.xyz` contains dimensionless normalized torque, **not** $M$ in N·m. Only `OffboardControlMode.thrust_and_torque` is true. Both messages share publication time; this outer-loop path leaves `timestamp_sample` at its default.

## Vehicle data and limits

The current adapter requires exactly four parallel upward-thrust rotors with invertible physical and allocator matrices. The loader converts declared FLU plant/propulsion data into FRD, shifts physical rotor positions to the center of mass, checks positive-definite inertia, and verifies matching physical/allocator yaw signs. The actuator curve must be finite, nonnegative over its range, and strictly increasing. It does not model tilted/reversible rotors, a hexarotor, motor dynamics, or changing battery conditions.

`config/vehicles/gz_f450.yaml` currently provides the complete adapter schema. This path uses its propulsion curve rather than `px4_allocator.hover_thrust`.

Collective is clamped to four times the curve's minimum/maximum rotor thrust. The requested moment is multiplied by a single scale in $[0,1]$, preserving its roll/pitch/yaw ratios. A negative requested collective is rejected before clamping. A scale of 1 means feasible in this model; it does not prove the aircraft achieved the requested wrench. There is no integral term in this geometric law.

## Run and inspect

Follow the shared [Offboard preparation and recovery workflow](../../operations/offboard.md). With a matching, airborne `gz_f450` near the configured start and no competing controller, use an Ubuntu repository-root shell:

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers se3.launch.py \
  vehicle:=gz_f450 trajectory:=hover handoff:=thrust_and_torque \
  direct_controller:=geometric_physical publish_diagnostics:=true record:=true
```

Valid position/velocity, attitude, and measured angular velocity are required. The outer loop runs at 100 Hz by default and begins output only after confirmed Offboard entry. Automatic/external entry and handback follow the [SE(3) lifecycle](../se3.md#when-the-controller-starts-and-stops).

Diagnostics under `/se3/diagnostics/geometric_physical/` report requested/applied collective [N], requested/model-applied moment [N·m], and `moment_scale`. Compare them with normalized setpoints, allocator status, and motor outputs. They are model calculations, not thrust/torque measurements. [Validation scope](../../reference/architecture.md#validation-scope) records tested configurations.

The source boundary is explicit: [physical_moment.cpp](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/ros2/src/offboard_controllers/src/se3/control/physical_moment.cpp) ends at the physical moment; [physical_wrench/adapter.cpp](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/ros2/src/offboard_controllers/src/se3/physical_wrench/adapter.cpp) applies feasibility and normalization; [config/loader.cpp](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/ros2/src/offboard_controllers/src/se3/physical_wrench/config/loader.cpp) defines the required vehicle data.
