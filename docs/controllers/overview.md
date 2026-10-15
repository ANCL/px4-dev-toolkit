<!-- Preserve existing section links. -->
<span id="controller-overview"></span>

# Controllers

The toolkit includes a PX4 reference flight, a simulation staging helper, an external position-reference node, and a configurable SE(3) controller. Each changes a different part of the experiment.

<!-- Preserve existing section links. -->
<span id="available-workflows"></span>

## What each method sends

| Method | Command sent to PX4 | What PX4 does next |
|---|---|---|
| Position-mode reference/staging | Mode/arm sequence and simulated manual input | Position stabilization and all lower stages |
| Offboard position | Position reference with trajectory feedforward | Position feedback, attitude/rate feedback, allocation |
| SE(3) acceleration | NED acceleration request | Acceleration-to-attitude/thrust conversion, rate feedback, allocation |
| SE(3) attitude | Desired rotation and collective thrust | Attitude/rate feedback, allocation |
| SE(3) attitude rate | FRD body-rate command and collective thrust | Rate feedback, allocation |
| SE(3) direct thrust/torque (three laws) | Normalized body thrust/torque | Control allocation and motors |

**Control allocation** maps a requested vehicle thrust/torque to individual actuator commands using the configured aircraft geometry and limits. It remains in PX4 for every documented controller mode.

<!-- Preserve existing section links. -->
<span id="native-position-mode-reference-experiment"></span>
<span id="position-mode-staging-helper"></span>
<span id="offboard-position-controller"></span>

## Begin with the reference

[Position Mode](position-mode.md) demonstrates the reference takeoff, hover, landing, and disarm sequence. [Offboard Staging](offboard-staging.md) prepares a simulated aircraft for a separate controller; the staging helper does not itself request Offboard.

[Offboard position control](offboard-position.md) sends the shared trajectory position and feedforward to PX4. This is the simplest reference for an external ROS controller. It never arms, waits for confirmed PX4 mode changes, and supports automatic or pilot-triggered Offboard entry.

<!-- Preserve existing section links. -->
<span id="se3"></span>
<span id="recording-and-studies"></span>
<span id="next"></span>

## Move the calculation into the toolkit

The [SE(3) controller](se3.md) calculates position/velocity feedback and can continue through desired attitude, body rate, or direct torque. **SE(3)** is the mathematical space of three-dimensional position and rotation. Its six documented configurations use four `handoff` values, with three rotational laws for `thrust_and_torque`.

Choose a mode according to what you want to compare and the [available vehicle data](../reference/adding-a-vehicle.md). Offboard position and SE(3) share [named trajectories](trajectories.md); environment startup, arming/staging, controller selection, and [recording](../workflows/recording.md) are distinct steps.
