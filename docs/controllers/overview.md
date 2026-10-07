# Controller overview

[Documentation](../index.md) / Controllers / Overview

The repository contains several controller/flight examples with deliberately different ownership boundaries. Choose the one that matches what you want to test instead of treating every launch file as the same kind of controller.

## Available workflows

| Workflow | Launch file | Toolkit ownership | PX4 ownership |
|---|---|---|---|
| Native Position-mode takeoff/hover/land | `px4_position_takeoff_hover/position_takeoff_hover.launch.py` | experiment orchestration and mode/command sequence | complete native flight-control stack |
| Position-mode staging for an Offboard test | `offboard_controllers/offboard_takeoff_handoff.launch.py` | virtual-joystick staging, arm/climb, waits for independent Offboard takeover | Position-mode flight until Offboard takeover |
| Offboard position trajectory | `offboard_controllers/offboard_position.launch.py` | Offboard lifecycle + trajectory position reference | position controller and all lower loops |
| Geometric SE(3) | `offboard_controllers/se3.launch.py` | depends on selected handoff | everything below the selected handoff boundary |

## Native Position-mode reference experiment

Run the PX4-native takeoff/hover/landing experiment with:

```bash
source ros2/runtime_env.sh
ros2 launch px4_position_takeoff_hover position_takeoff_hover.launch.py
```

Add `record:=true` to record it. This is useful as a native PX4 behavior/reference run rather than an external controller.

## Position-mode staging helper

The staging helper is intentionally separate from the controller that takes Offboard ownership. Its flow is:

```text
PX4 ready + local position
→ minimum-throttle virtual joystick
→ Position mode
→ arm
→ climb to configured staging altitude
→ centered joystick hold
→ wait for independent controller to enter Offboard
→ exit
```

It relies on MAVProxy's repository virtual-joystick module for the continuous manual-control stream. The helper itself does not request Offboard mode.

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers offboard_takeoff_handoff.launch.py
```

## Offboard position controller

This controller publishes only trajectory references and Offboard/mode commands; PX4 owns the position controller and every lower control loop.

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers offboard_position.launch.py \
  trajectory:=hover
```

It can use any named trajectory from the shared [trajectory configuration](trajectories.md).

## SE(3)

SE(3) is the configurable controller family used to move the handoff boundary through the PX4 multicopter stack:

```text
acceleration
attitude
attitude_rate
thrust_and_torque + px4_rate
thrust_and_torque + geometric_normalized
thrust_and_torque + geometric_physical
```

Read [SE(3)](se3.md) before using the direct torque modes; vehicle requirements differ by handoff.

## Recording and studies

Use [Recording](../workflows/recording.md) for one run and [Sequences](../workflows/sequences.md) for ordered studies.

## Next

- [SE(3)](se3.md)
- [Trajectories](trajectories.md)
- [Recording](../workflows/recording.md)
