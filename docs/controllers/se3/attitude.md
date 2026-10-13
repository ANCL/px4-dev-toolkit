# Attitude

The toolkit constructs a desired rotation from the gravity-compensated force and yaw, then publishes VehicleAttitudeSetpoint. PX4 supplies attitude and rate feedback.

Collective thrust projects onto the current body Z axis and uses the curated hover-thrust scale; the published FRD Z component is negative.
