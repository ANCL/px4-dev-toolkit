<!-- Preserve established section links. -->
<span id="controller-overview"></span>

# Controllers

Choose a control path by where its feedback runs: native PX4 reference, ROS 2 Offboard, or onboard SE3.

<span id="available-workflows"></span>

<span id="what-each-method-sends"></span>

## Controller families

<div class="px4-table-scroll" markdown="1">

| Family | Controllers | Control path |
|---|---|---|
| PX4 reference | [Position](position-mode.md), [staging](offboard-staging.md) | PX4 stabilizes the aircraft; ROS helpers automate the SITL demonstration. |
| ROS 2 Offboard | [Position](offboard-position.md), [SE3](se3.md) | ROS streams setpoints via DDS; PX4 retains the loops below the selected handoff. |
| PX4 onboard | [SE3](onboard-se3.md) | PX4 runs the controller and trajectory; its rate loop and allocator remain active. |

</div>

**Control allocation** maps requested vehicle thrust and torque into actuator commands using the configured geometry and limits. Every implementation described here retains PX4 allocation. A normalized torque setpoint is an allocator input, not a physical moment in N·m.

<span id="native-position-mode-reference-experiment"></span>
<span id="position-mode-staging-helper"></span>
<span id="offboard-position-controller"></span>

<span id="begin-with-the-reference"></span>

## Begin with a reference flight

[Position mode](position-mode.md) demonstrates takeoff/hover under PX4 stabilization. [Offboard staging](offboard-staging.md) prepares an airborne SITL vehicle for handoff.

[Offboard Position](offboard-position.md) streams a trajectory's position and feedforward. See [Offboard control](../operations/offboard.md) for preparation and handback.

<span id="se3"></span>

<span id="move-the-calculation-into-the-toolkit"></span>

## Select the SE3 implementation

**SE3** describes position and orientation in three dimensions. The shared [mathematical reference](../mathematics/geometric-control.md) develops the plant and geometric law; each implementation chooses its actual software interface.

<div class="px4-table-scroll" markdown="1">

| ROS 2 Offboard SE3 handoff | External output | PX4 retains |
|---|---|---|
| `acceleration` | NED acceleration | Acceleration-to-attitude/thrust mapping, attitude/rate feedback, allocation |
| `attitude` | Attitude and normalized collective thrust | Attitude/rate feedback, allocation |
| `attitude_rate` | FRD body rates and normalized collective thrust | Rate feedback, allocation |
| `thrust_and_torque` | Normalized FRD torque/thrust | Allocation and actuator output |

</div>

For `thrust_and_torque`, choose [Geometric (Normalized)](se3/geometric-normalized.md), [Geometric (Physical)](se3/geometric-physical.md), or [PX4 Rate PID](se3/px4-rate.md). The physical law maps N·m moments to PX4-normalized commands through a vehicle adapter. See [vehicle data](../reference/adding-a-vehicle.md).

[Onboard SE3](onboard-se3.md) runs a nominal 100 Hz position/attitude-to-rate controller in firmware. See [Onboard control](../operations/onboard.md) for parameters, mode selection, trajectory start, and return.

<span id="recording-and-studies"></span>
<span id="next"></span>

## Plan the experiment

Both controllers use [absolute PX4 local NED trajectories](trajectories.md). Offboard streams references from ROS 2; onboard samples uploaded plans in PX4. See [Recording](../workflows/recording.md), [Analysis](../analysis/analysis.md), and [Validation scope](../reference/architecture.md#validation-scope).
