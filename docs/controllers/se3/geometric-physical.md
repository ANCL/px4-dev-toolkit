<span id="geometric-physical-moment"></span>

# Geometric (Physical)

This controller computes **collective thrust in newtons** and **body moment in newton-metres** from an SE(3) reference. A separate, vehicle-specific adapter converts that physical wrench into the **normalized thrust and torque setpoints** expected by PX4. PX4 still performs control allocation.

The distinction matters: the geometric control law, actuator feasibility model, and PX4 normalization are three different calculations. The current adapter is implemented for **four rotors**, not arbitrary multirotor configurations.

<span id="physical-rotational-law"></span>

## 1. Physical moment law

Let $R$ and $R_d$ map current and desired FRD body coordinates into NED. The measured angular velocity $\Omega$ is in the current body frame; $\Omega_d$ and $\dot\Omega_d$ are in the desired body frame. Define the rotation from desired-body to current-body coordinates as

$$
Q = R^\top R_d.
$$

The attitude and angular-rate errors, both expressed in the **current FRD frame**, are

$$
\begin{aligned}
e_R &= \tfrac12\left(R_d^\top R - R^\top R_d\right)^\vee, \\
e_\Omega &= \Omega - Q\Omega_d.
\end{aligned}
$$

Here $(\cdot)^\vee$ converts a skew-symmetric matrix into its corresponding three-vector; the [Math page](foundations.md#rotation-errors-and-feedforward) defines it. The feedforward angular acceleration in current-body coordinates is

$$
\alpha_{\mathrm{ff}}
= -\Omega\times(Q\Omega_d) + Q\dot\Omega_d.
$$

With body inertia $J$ and diagonal physical gains $K_R$ and $K_\Omega$, the implemented moment law is

$$
\begin{aligned}
M ={}&-K_Re_R-K_\Omega e_\Omega \\
     &+\Omega\times(J\Omega)+J\alpha_{\mathrm{ff}}.
\end{aligned}
$$

The first two terms reduce attitude and rate error. The cross-product compensates rigid-body gyroscopic coupling; the final term tracks the moving desired attitude. This is a **moment in FRD N·m**, not normalized PX4 torque. $J$ must be the inertia about the vehicle's center of mass in the same body frame. The diagonal gains are in the toolkit's SE(3) controller configuration; they are not PX4 rate-loop gains.

The translational law produces a gravity-compensated NED force $A$. Positive upward collective thrust is the projection onto the **current** body-down direction $b_3=Re_3$:

$$
f=-A^\top b_3.
$$

Thus $f$ need not equal $\|A\|$ when attitude tracking is imperfect. The runtime computes $M$ in [`physical_moment::compute()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/se3/control/physical_moment.cpp#L12-L43), obtains the projected collective, and calls the adapter from [`Se3::execute_handoff()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/se3/runtime/control.cpp#L23-L120). The desired rotation, rates, and angular acceleration originate in [`attitude_reference::compute_dynamics()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/se3/control/attitude_reference.cpp#L79).

<span id="physical-rotor-allocation"></span>

## 2. From wrench to four rotor thrusts

Rotor $i$ is located at $r_i=(x_i,y_i,z_i)$ relative to the center of mass, in FRD metres. Its upward thrust is $(0,0,-T_i)$, so the roll and pitch moments from the arm are $(-y_iT_i,x_iT_i)$. The signed reaction-moment ratio $\kappa_i$ supplies yaw moment.

For each rotor, the column of the **physical effectiveness matrix** is

$$
B_{\mathrm{ph},i}=
\begin{bmatrix}-y_i & x_i & \kappa_i & 1\end{bmatrix}^{\!\top}.
$$

Stacking these four columns gives $B_{\mathrm{ph}}\in\mathbb R^{4\times4}$. The requested physical rotor thrusts are

$$
\begin{bmatrix}T_0&T_1&T_2&T_3\end{bmatrix}^{\!\top}
=B_{\mathrm{ph}}^{-1}
\begin{bmatrix}M_x&M_y&M_z&f\end{bmatrix}^{\!\top}.
$$

This is an exact square-matrix inverse, **not a generic N-rotor allocator**. Geometry or rotor ordering that makes the matrix singular is rejected. See [`physical_effectiveness()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/se3/physical_wrench/matrices.cpp#L127) and [`inverse()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/se3/physical_wrench/matrices.cpp#L21).

The YAML loader converts supported input frames into FRD, subtracts the center of mass, validates four unique rotor indices, and checks physical/PX4 yaw-moment signs. The source of these checks is [`load_physical_wrench_configuration()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/se3/physical_wrench/config/loader.cpp#L19). The geometry and propulsion data **must match the aircraft**, including rotor order.

<span id="feasible-thrust-and-moment-scaling"></span>

## 3. Feasibility and actuator model

The physical motor model is a monotonic thrust curve over a configured control range:

$$
T_i(u_i)=c_0+c_1u_i+c_2u_i^2,
\qquad u_i\in[u_{\min},u_{\max}].
$$

The current F450 fixture uses $T(u)=0.27+3.06u+8.67u^2$ N; these are **fixture-specific coefficients**, not a generally valid thrust model. The interval determines the feasible per-rotor thrust range $[T_{\min},T_{\max}]$.

The adapter first clamps the requested collective to $[4T_{\min},4T_{\max}]$, yielding $f_a$. It then solves two allocations with the same collective:

$$
\begin{aligned}
T^0 &= B_{\mathrm{ph}}^{-1}(0,0,0,f_a)^\top,\\
T^* &= B_{\mathrm{ph}}^{-1}(M_x,M_y,M_z,f_a)^\top.
\end{aligned}
$$

$T^0$ is the zero-moment baseline, and $T^*$ requests the full moment. The adapter selects the **largest** $\lambda\in[0,1]$ for which every component of

$$
T^a=T^0+\lambda(T^*-T^0)
$$

lies in $[T_{\min},T_{\max}]$. Each positive rotor delta bounds $\lambda$ by its remaining upper-thrust margin divided by that delta; each negative delta uses the remaining lower-thrust margin. The smallest bound wins. The resulting modeled moment is $\lambda M$, preserving its roll/pitch/yaw proportions while reducing magnitude. If even $T^0$ is infeasible, the adapter rejects the request instead of silently changing the allocation.

This is a **toolkit feasibility policy**, not a consequence of Lee's unconstrained law. Its implementation is [`allocate_feasible_physical_wrench()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/se3/physical_wrench/adapter.cpp#L52-L176).

Each feasible $T_i^a$ is converted into actuator control $u_i$ by inverting the monotonic polynomial. Linear curves use a direct inverse; quadratic curves choose the root inside the configured interval. See [`thrust_from_control()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/se3/physical_wrench/actuators.cpp#L25) and [`control_from_thrust()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/se3/physical_wrench/actuators.cpp#L41).

<span id="convert-to-px4-normalized-coordinates"></span>

## 4. Convert to PX4's normalized interface

The PX4 effectiveness matrix is **not** $B_{\mathrm{ph}}$. With allocator thrust coefficient $C_{T,i}$ and signed yaw coefficient $K_{M,i}$, its rotor column is

$$
B_{\mathrm{PX4},i}=C_{T,i}
\begin{bmatrix}-y_i&x_i&K_{M,i}&-1\end{bmatrix}^{\!\top}.
$$

The final entry is **negative FRD body-Z thrust**, whereas the physical matrix's final entry is positive upward collective. The adapter constructs this matrix in [`allocator_effectiveness()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/se3/physical_wrench/matrices.cpp#L167).

Let $u$ contain the four actuator controls and let $S$ be the diagonal normalization scale derived from the pinned PX4 inverse mixer. The normalized PX4 request is

$$
c_n=S\,B_{\mathrm{PX4}}u.
$$

Its first three coordinates are normalized torque; the fourth is normalized body-Z thrust. The roll and pitch scales share the larger normalized inverse-column norm; yaw and thrust use the corresponding mixer-column rules. [`allocator_normalization_scale()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/se3/physical_wrench/matrices.cpp#L213) implements those rules, and [`adapt()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/se3/physical_wrench/adapter.cpp#L182) assembles the final command.

The intention is that the pinned PX4 allocator's corresponding normalized inverse mixer reconstructs the same modeled $u$. This correspondence depends on the **pinned PX4 allocation algorithm, coefficients, geometry, and rotor order**; it does not prove that the motors deliver the modeled thrust. See [PX4's pinned `ControlAllocationPseudoInverse.cpp`](https://github.com/ANCL/PX4-Autopilot/blob/6d31b14291a8f86e76a2e6074eca6a2c47a080dc/src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp).

<span id="parameters-timing-and-messages"></span>

## 5. Run and interpret

Use a configured F450 SITL aircraft that is already airborne, with valid position, velocity, attitude, and body angular velocity. Ensure no other Offboard controller is running. The outer SE(3) loop (100 Hz by default) needs trajectory derivatives through snap and desired yaw acceleration for this law. Selected thrust/torque setpoints start **after confirmed Offboard entry**.

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers se3.launch.py \
  vehicle:=gz_f450 trajectory:=full_excitation handoff:=thrust_and_torque \
  direct_controller:=geometric_physical publish_diagnostics:=true record:=true
```

Diagnostics distinguish **requested** collective and body moment from **applied modeled** collective and $\lambda M$, plus the moment scale $\lambda$. A scale below one indicates modeled actuator feasibility reduction—not measured motor torque or proof of closed-loop performance.

**Current limits:** exactly four modeled rotors, validated physical and PX4 rotor mappings, a monotonic identified propulsion curve, and the pinned normalization algorithm. Saturation, model mismatch, tracking transients, singular geometry, and critical SO(3) attitudes remain relevant. This is not a general hardware-ready allocation guarantee.

For shared definitions, force derivatives, and attitude dynamics, see [Math](foundations.md). For controller startup and return-to-Position behavior, see [SE(3) Control](../se3.md#when-the-controller-starts-and-stops).
