# Geometric normalized torque

The toolkit maps SO(3) attitude/rate errors and angular acceleration feedforward directly to normalized PX4 torque. PX4 performs motor allocation.

This law has no physical inertia term or gyroscopic moment. It needs mass and a curated hover-thrust scale.
