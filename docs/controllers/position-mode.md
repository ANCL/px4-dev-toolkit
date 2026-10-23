<span id="position-mode-and-switching-to-offboard"></span>

# Position Mode

In **PX4 Position mode**, PX4 stabilizes the aircraft and uses position/velocity feedback while pilot stick input requests motion. A pilot normally provides stick input through an RC transmitter or joystick. In the reference simulation, MAVProxy provides repeatable virtual-joystick input.

<span id="native-position-mode-reference-experiment"></span>

## Reference flight

Use a prepared SITL environment with `GCS=mavproxy`, valid PX4 state, no competing controller, and an initially landed/disarmed simulated aircraft. In a **second repository-root shell**:

```bash
source ros2/runtime_env.sh
ros2 launch px4_position_mode position_takeoff_hover.launch.py
```

The [reference example](https://github.com/ANCL/px4-dev-toolkit/tree/main/ros2/src/px4_position_mode/src/takeoff_hover) performs readiness checks, Position-mode selection, arm, climb, and a 20-second hover. By default, `finish:=hold` exits the ROS helper **without landing or disarming**; MAVProxy keeps the virtual joystick centered and PX4 stays in Position mode. For the original takeoff-hover-land reference run, use `finish:=land`:

```bash
ros2 launch px4_position_mode position_takeoff_hover.launch.py finish:=land
```

Add `record:=true` to record the selected sequence. Neither option calculates an external position feedback law. `finish:=hold` leaves an armed aircraft in flight; land and disarm explicitly before stopping SITL.

<!-- Keep the previous staging anchor for existing links. -->
<span id="position-mode-staging-helper"></span>

To prepare an airborne SITL aircraft for an independent controller, use
[Offboard Staging](offboard-staging.md). It is a separate workflow, not part of
the reference flight.
