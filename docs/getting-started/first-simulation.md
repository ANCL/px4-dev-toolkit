# First simulation

Run a repeatable Position-mode takeoff, hover, and landing in Gazebo. PX4 performs the flight control; the example supplies simulated pilot input. This guide uses a **native SITL installation**. If you use [Docker](../installation/docker.md), run the same repository-root commands inside the container shells described there.

## 1. Prepare the simulation

Complete [native installation](../installation/native.md), including the PX4 and ROS builds. Keep the checked-in `gz_f450` simulator and `GCS=mavproxy` for this first example: its virtual joystick is needed by the Position-mode demonstration. Use simulation only for these automated commands.

In **terminal 1 at the repository root**, start the desktop simulation:

```bash
./tools/sitl start
```

Gazebo should show the aircraft, and tmux should contain `px4`, `xrce-dds`, and `mavproxy` windows. For a terminal-only host, use `./tools/sitl start headless`; inspect messages instead of a Gazebo view. Switch tmux windows with **Ctrl-b, then n**; detach with **Ctrl-b, then d** without stopping the services.

## 2. Check communication and state

In **terminal 2 at the same repository root**:

```bash
source ros2/runtime_env.sh
./tools/sitl status
./tools/topic_rates
```

Check that the expected components run and PX4 topics carry samples. Stop the rate display with Ctrl-C. Then inspect state in the **same sourced shell**:

```bash
ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort --once
ros2 topic echo /fmu/out/vehicle_status --qos-reliability best_effort --once
```

Local position should have valid horizontal/vertical estimates. PX4 status should belong to the intended simulated vehicle. If state or messages are missing, resolve [simulation/DDS checks](../reference/troubleshooting.md#simulation-and-dds) before launching a flight. A topic name without samples is insufficient.

## 3. Run the reference flight

With Gazebo ready, no other controller running, and the simulated aircraft initially landed/disarmed, use **terminal 2**:

```bash
ros2 launch px4_position_mode position_takeoff_hover.launch.py
```

The example waits for readiness, selects Position mode, arms, climbs, holds, lands, and disarms through its implemented state machine. In the GUI you should see those phases; in a headless session follow the node/PX4 logs and mode/arming state. This is a PX4 reference flight, not an external SE(3) controller.

Wait for successful completion. If it fails, inspect the reported phase and restore a landed/disarmed simulation before trying again. Ctrl-C interrupts a process; it is not a guaranteed landing command.

## 4. End the session

After landing and disarm have been confirmed, stop the simulation from a **repository-root shell**:

```bash
./tools/sitl stop
./tools/sitl status
```

The wrapper stops its tmux services and cleans up detached Gazebo processes associated with this checkout. The status should show the session stopped.

The commands and expected phases are checked against source. This documentation change did not execute an armed simulation. On a new host, confirm the actual results above before moving to a more demanding experiment.

Continue with [Position mode and switching to Offboard](../controllers/position-mode.md), then [record a flight](../workflows/recording.md) when the basic example works.
