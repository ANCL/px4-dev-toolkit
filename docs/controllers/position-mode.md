# Position mode and switching to Offboard

PX4 Position mode stabilizes the aircraft while pilot stick input requests motion. The simulation reference and staging examples imitate this input through the MAVProxy virtual joystick.

The staging helper waits for an independent controller. [Offboard position](offboard-position.md) and [SE(3)](se3.md) request and confirm their own mode change.
