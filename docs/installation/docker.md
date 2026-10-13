<!-- Preserve existing section links. -->
<span id="docker-installation-sitl"></span>

# Docker installation

The maintained Docker image provides a prebuilt PX4 SITL environment with Gazebo Harmonic, ROS 2 Jazzy, the Micro XRCE-DDS Agent, MAVProxy, controller examples, recording, and analysis. It is intended for simulation on a Linux host. It does not include the Vicon and serial services for physical hardware; see [Companion-computer installation](companion.md) for those services.

The example below runs **headless**: Gazebo simulates the aircraft and sensors, but does not open a 3D window. You can still run flights, inspect PX4/ROS 2 messages, and record data.

## Run the published image

Use a Linux host with Docker Engine installed and permission to run `docker`. Open a **host terminal** in the directory where you want to keep flight data:

```bash
mkdir -p bags log
docker pull ghcr.io/ancl/px4-dev-toolkit-sitl:latest
docker run --rm -it --name px4-sitl --network host \
  -v "$PWD/bags:/opt/px4-dev-toolkit/bags" \
  -v "$PWD/log:/opt/px4-dev-toolkit/log" \
  ghcr.io/ancl/px4-dev-toolkit-sitl:latest bash
```

The shell opens inside the container at `/opt/px4-dev-toolkit`. The image includes compiled PX4 and ROS 2 packages; you do not need to build them for this example. The container runs as UID/GID 1000, so the host `bags/` and `log/` directories must be writable by that user.

**In the first container shell**, start the simulator and support services:

```bash
./tools/sitl start headless
```

The command attaches to the `px4-sitl` tmux session containing PX4/Gazebo, the DDS agent, and MAVProxy. Switch windows with **Ctrl-b, then n**, or detach with **Ctrl-b, then d**. Detaching does not stop the services. The default MAVProxy frontend is a terminal program, not the Gazebo 3D viewer or QGroundControl.

<span id="verification"></span>

## Check the running environment

Leave the first container running. Open a **second host terminal**, then enter that same container:

```bash
docker exec -it px4-sitl bash
```

`docker exec` does not invoke the image entrypoint, so source the ROS runtime explicitly in this **second container shell**:

```bash
cd /opt/px4-dev-toolkit
source ros2/runtime_env.sh
./tools/sitl status
./tools/topic_rates
```

The rate display checks that ROS 2 is receiving messages; stop it with **Ctrl-c**. Verify the expected PX4/Gazebo and DDS processes and live topic data. A running service or a visible topic name alone does not establish that the vehicle is ready to fly. Continue with [First simulation](../getting-started/first-simulation.md) for the Position-mode demonstration and state checks.

## Save recordings and logs

The example bind-mounts the container's `bags/` and `log/` to the host directories created at the start. Flight bags and logs remain on the host after the container exits; other files written only inside the container are removed by `--rm`. MAVProxy logs are stored under `log/mavproxy/`.

The image's default user has UID/GID 1000. If recording cannot create files in the mounted folders, check their host ownership and permissions before rerunning it.

## Build from a checkout instead

To build the maintained SITL image from repository source, run in a **host terminal**:

```bash
git clone https://github.com/ANCL/px4-dev-toolkit.git
cd px4-dev-toolkit
docker compose -f docker/compose.yaml build sitl
docker compose -f docker/compose.yaml run --rm --name px4-sitl sitl bash
```

This is an alternative to `docker pull`/`docker run`, not an additional step. The Compose setup uses **named volumes** for `bags/` and `log/`, rather than the working directory's host folders, and mounts the runtime configuration read-only. Once inside the container, use the same `./tools/sitl start headless` command and second-terminal `docker exec` procedure described above. Do not run the published-image and Compose examples simultaneously under the same container name.

## Graphical simulation

The default command starts Gazebo without its 3D window. If you want the Gazebo GUI, configure host-to-container display access following the official [PX4 Gazebo Container GUI guide](https://docs.px4.io/main/en/simulation/gazebo_container_gui). That guide uses PX4's own container images, so adapt its display-related Docker options to this toolkit's image; the toolkit does not configure host-specific display forwarding for you.

After the host and container display are configured, use `./tools/sitl start` instead of `./tools/sitl start headless`. A working window also depends on suitable graphics/OpenGL support. For native desktop simulation, see [Native SITL installation](native.md).

The Gazebo 3D window is separate from ground-control software. This image starts MAVProxy by default; it does not include QGroundControl.

<span id="further-reading"></span>

## Stop the simulation

Once the simulated aircraft is **landed and disarmed**, run from the second container shell:

```bash
./tools/sitl stop
./tools/sitl status
```

The status should report the toolkit services as stopped. Exit the second container shell, detach from tmux if necessary (**Ctrl-b, then d**), and exit the first container shell. With `docker run --rm`, Docker then removes the container while leaving the bind-mounted bags and logs on the host.

For the implementation, see the [Dockerfile](https://github.com/ANCL/px4-dev-toolkit/blob/main/docker/Dockerfile), [Compose configuration](https://github.com/ANCL/px4-dev-toolkit/blob/main/docker/compose.yaml), and [entrypoint](https://github.com/ANCL/px4-dev-toolkit/blob/main/docker/entrypoint.sh).
