# Offboard position control

This controller sends the shared trajectory position, velocity, and acceleration feedforward to PX4. PX4 still calculates position feedback, attitude, rate feedback, and motor allocation.

Trajectory time advances after confirmed Offboard entry. Read [Trajectories](trajectories.md) for frames and segment timing.
