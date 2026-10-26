<span id="position-mode-and-switching-to-offboard"></span>
<span id="position-mode"></span>

# PX4 Position mode

PX4 Position mode runs native position/velocity control. In SITL, the toolkit's MAVProxy helper supplies virtual-joystick input for a repeatable reference flight.

<span id="native-position-mode-reference-experiment"></span>

## Reference flight

Use a prepared SITL environment with `GCS=mavproxy`, valid PX4 state, no competing controller, and an initially landed/disarmed aircraft. In a separate Ubuntu repository-root shell:

```bash
source ros2/runtime_env.sh
ros2 launch px4_position_mode position_takeoff_hover.launch.py
```

The helper checks readiness, selects Position mode, arms, climbs, and holds with centered sticks for **10 seconds**. See its [lifecycle](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/ros2/src/px4_position_mode/src/takeoff_hover/lifecycle.cpp).

`height:=1.0` is the default relative climb from the measured starting altitude, not an absolute NED position target. The launch accepts finite heights greater than 0 and at most 5 m. With `finish:=hold` (default), the ROS helper exits after hovering; MAVProxy remains centered and PX4 stays armed in Position mode. Confirm measured position and heading before using this to prepare an independent controller.

For a complete takeoff-hover-land reference:

```bash
ros2 launch px4_position_mode position_takeoff_hover.launch.py \
  height:=1.0 finish:=land record:=true
```

`finish:=land` sends PX4's land command, then waits for disarmed state, with a 30-second landing/disarm deadline. `record:=true` records the selected sequence. A timeout or interruption is not proof of landing; check the aircraft's actual state.

<!-- Preserve the previous staging anchor. -->
<span id="position-mode-staging-helper"></span>

## Preparing for Offboard

`finish:=hold` leaves PX4 armed in Position mode. Continue with [Offboard control](../operations/offboard.md) or use [Offboard Staging](offboard-staging.md) for a handoff-aware preparation helper.
