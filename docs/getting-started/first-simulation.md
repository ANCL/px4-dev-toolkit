# First simulation

Run a Position-mode takeoff, hover, and landing in **native SITL**. For [Docker](../installation/docker.md), run the same repository commands inside the container.

## 1. Prepare the simulation

Complete [native installation](../installation/native.md), including the PX4 and ROS builds. Keep the checked-in `gz_f450` simulator and `GCS=mavproxy` for this first example: its virtual joystick is needed by the Position-mode demonstration. Use simulation only for these automated commands.

In **terminal 1 at the repository root**, start the desktop simulation:

```bash
./tools/sitl start
```

Gazebo should show the aircraft, and tmux should contain `px4`, `xrce-dds`, `mavproxy`, and `mavlink-router` windows. For a terminal-only host, use `./tools/sitl start headless`; inspect messages instead of a Gazebo view. The tmux session keeps the named services running independently of the terminal view.

## 2. Check communication and state

In **terminal 2 at the same repository root**:

```bash
source ros2/runtime_env.sh
./tools/sitl status
./tools/topic_rates
```

Check the services and arriving PX4 messages. In the same sourced shell, inspect vehicle state:

```bash
ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort --once
ros2 topic echo /fmu/out/vehicle_status_v1 --qos-reliability best_effort --once
```

Require valid local position and the expected vehicle status before flight. For missing samples, see [Simulation and DDS troubleshooting](../reference/troubleshooting.md#simulation-and-dds).

## 3. Run the reference flight

With Gazebo ready, no other controller running, and the simulated aircraft initially landed/disarmed, use **terminal 2**:

```bash
ros2 launch px4_position_mode position_takeoff_hover.launch.py finish:=land
```

`finish:=land` arms, climbs, holds, lands, and disarms under PX4 Position-mode control. The default `finish:=hold` leaves the aircraft airborne for an Offboard handoff. Follow the node/PX4 logs and mode/arming state in headless runs.

Wait for successful completion. If it fails, inspect the reported phase and restore a landed/disarmed simulation before trying again. Ctrl-C interrupts a process; it is not a guaranteed landing command.

## 4. End the session

After landing and disarm have been confirmed, stop the simulation from a **repository-root shell**:

```bash
./tools/sitl stop
./tools/sitl status
```

Confirm `status` reports the session stopped.

Confirm the expected phases and final PX4 state on your installation before moving to the Offboard examples.

Continue with [Position Mode](../controllers/position-mode.md) for the reference workflow, then [Offboard Staging](../controllers/offboard-staging.md) to prepare for an external controller. See [Recording](../workflows/recording.md) when the basic example works. For native firmware control, follow [Onboard control](../operations/onboard.md); [Tools and terminals](../operations/tools.md) explains which commands belong in Bash, PX4, or the GCS.
