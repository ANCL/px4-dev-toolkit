# PX4 rate controller

The outer geometric attitude stage holds body-rate and collective commands. The reproduced PX4 inner rate stage executes on VehicleAngularVelocity timestamp_sample.

The implementation reproduces commit f5083ca2c5b919350880e8667e636ef70715db17, including integral anti-windup and yaw-torque filtering; battery scaling must remain disabled.
