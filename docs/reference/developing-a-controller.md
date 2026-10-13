# Developing a controller

Keep pure controller calculations separate from ROS/PX4 transport. Use Offboard position as a small example and SE(3) for diagnostic and mode-selection references.

A controller addition needs package/build integration, launch configuration, PX4 topics, mode-confirmation logic, and verification. Recording and analysis should reflect the new signals.
