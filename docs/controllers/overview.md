# Controllers

The toolkit includes a PX4 reference flight, a simulation staging helper, an external position-reference node, and a configurable SE(3) controller. Each changes a different part of the experiment.

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

## Begin with the reference

[Position mode and switching to Offboard](position-mode.md) explains the reference flight and the separate staging helper. The helper prepares the simulated aircraft for an independent controller; it does not itself enter Offboard.

[Offboard position control](offboard-position.md) sends the shared trajectory position and feedforward to PX4. This is the simplest reference for an external ROS controller. It can request arming and confirms mode changes through PX4 status.

## Move the calculation into the toolkit

The [SE(3) controller](se3.md) calculates position/velocity feedback and can continue through desired attitude, body rate, or direct torque. **SE(3)** is the mathematical space of three-dimensional position and rotation. Its six documented configurations use four `handoff` values, with three rotational laws for `thrust_and_torque`.

Choose a mode according to what you want to compare and the [available vehicle data](../reference/adding-a-vehicle.md). All controllers share [named trajectories](trajectories.md); environment startup, arming/staging, controller selection, and [recording](../workflows/recording.md) are distinct steps.
