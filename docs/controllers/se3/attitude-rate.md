# Attitude rate

The toolkit maps desired angular velocity to the current body frame and adds geometric attitude feedback. VehicleRatesSetpoint carries body-rate and collective-thrust commands.

PX4 still computes rate feedback and allocation. The desired attitude rate is analytically derived from force and yaw derivatives.
