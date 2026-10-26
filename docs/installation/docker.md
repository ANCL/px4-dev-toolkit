<!-- Preserve existing section links. -->
<span id="docker-installation-sitl"></span>

<span id="docker-installation"></span>

# Docker SITL

The Docker image provides PX4 SITL, Gazebo Harmonic, ROS 2 Jazzy, MAVProxy, controller tools, recording, and analysis on a Linux host.

The example uses headless Gazebo; [graphical simulation](#graphical-simulation) requires host display integration.

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

The container opens at `/opt/px4-dev-toolkit` with PX4 and ROS 2 already built. The image user has UID/GID 1000; ensure it can write the mounted `bags/` and `log/` directories.

**In the first container shell**, start the simulator and support services:

```bash
./tools/sitl start headless
```

`start` attaches to the `px4-sitl` session with PX4/Gazebo, DDS, MAVProxy, and MAVLink Router. Detaching leaves the services running.

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

Check that PX4/Gazebo and DDS are running and that topics have samples. Continue with [First simulation](../getting-started/first-simulation.md).

## Save recordings and logs

The bind-mounted `bags/` and `log/` persist on the host; `--rm` removes other container-local files on exit. MAVProxy writes to `log/mavproxy/`. For write failures, check host directory ownership.

## Build from a checkout instead

To build the maintained SITL image from repository source, run in a **host terminal**:

```bash
git clone https://github.com/ANCL/px4-dev-toolkit.git
cd px4-dev-toolkit
docker compose -f docker/compose.yaml build sitl
docker compose -f docker/compose.yaml run --rm --name px4-sitl sitl bash
```

Compose replaces the published-image `docker run` example. It uses named volumes for `bags/` and `log/` and mounts runtime configuration read-only. Inside the container, follow the same SITL and second-shell commands.

## Graphical simulation

For the Gazebo GUI, configure host-to-container display access using the [PX4 Gazebo Container GUI guide](https://docs.px4.io/main/en/simulation/gazebo_container_gui), adapting its Docker options to the toolkit image.

After display forwarding is configured, use `./tools/sitl start` for the GUI. See [Native SITL](native.md) for direct desktop simulation.

MAVProxy is the default ground-control frontend in the published image.

<span id="further-reading"></span>

## Stop the simulation

Once the simulated aircraft is **landed and disarmed**, run from the second container shell:

```bash
./tools/sitl stop
./tools/sitl status
```

The status should report the toolkit services as stopped. Exit the second container shell, leave the tmux session, and exit the first container shell. With `docker run --rm`, Docker then removes the container while leaving the bind-mounted bags and logs on the host.

For the implementation, see the [Dockerfile](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/docker/Dockerfile), [Compose configuration](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/docker/compose.yaml), and [entrypoint](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/docker/entrypoint.sh).
