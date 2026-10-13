# Docker installation

The maintained image runs PX4 SITL with Gazebo. It includes Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, compiled PX4/ROS packages, the Micro XRCE-DDS Agent, MAVProxy, controller examples, recording, and analysis. The native [hardware installation](native.md) supplies Vicon and serial companion services separately.

## Published SITL image

In a **host terminal on Linux**, from a directory where you want to keep flight data:

```bash
mkdir -p bags log
docker pull ghcr.io/ancl/px4-dev-toolkit-sitl:latest
docker run --rm -it --name px4-sitl --network host \
  -v "$PWD/bags:/opt/px4-dev-toolkit/bags" \
  -v "$PWD/log:/opt/px4-dev-toolkit/log" \
  ghcr.io/ancl/px4-dev-toolkit-sitl:latest bash
```

`--network host` lets PX4, the DDS agent, and ground-control connections use the host network. Use one simulation at a time unless you explicitly configure separate endpoints. The container's default user has UID/GID 1000; the mounted directories must be writable by that user. Check permissions if recording fails to create files.

**Inside the container**, the working directory is `/opt/px4-dev-toolkit`. The entrypoint sources the ROS environment for the first shell:

```bash
./tools/sitl start headless
```

This attaches to tmux, a terminal session with one window per service. It does not open a second terminal. Open a **second host terminal**:

```bash
docker exec -it px4-sitl bash
```

`docker exec` bypasses the image entrypoint. In that **second container shell**, prepare ROS commands explicitly:

```bash
cd /opt/px4-dev-toolkit
source ros2/runtime_env.sh
./tools/sitl status
ros2 topic list
```

Expect PX4, Gazebo, MAVProxy, and the agent to be running, with `/fmu/` topics visible. Topic discovery alone does not prove samples are arriving; use `./tools/topic_rates` and the [first simulation](../getting-started/first-simulation.md) checks next.

## Persistent data

The bind mounts above keep `bags/` and `log/` in the host directory even after `--rm` removes the container. Files elsewhere in the container are lost when it is removed. MAVProxy telemetry is placed under `log/mavproxy/`.

A read-only bind mount over `config/runtime/sitl.env` can select a different runtime configuration. It does not rebuild a missing PX4 target or install another GCS. The [configuration reference](../reference/configuration.md) separates runtime settings from gains and vehicle data.

## Building the image locally

In a **host terminal**:

```bash
git clone https://github.com/ANCL/px4-dev-toolkit.git
cd px4-dev-toolkit
docker compose -f docker/compose.yaml build sitl
docker compose -f docker/compose.yaml run --rm --name px4-sitl sitl bash
```

Only the `sitl` service/build target is maintained. Compose uses **named volumes** for bags and logs, rather than the host `bags/` and `log/` directories. Those volumes persist independently of a removed container. Compose also mounts the runtime configuration read-only from the checkout. Enter a second shell with the same `docker exec` command above.

## Graphical simulation

The complete example uses headless Gazebo and the configured MAVProxy frontend. To run a GUI, the container needs access to your host display and graphics devices; the maintained Compose file does not add those mappings. Configure them for your host before using `./tools/sitl start`, or use [native installation](native.md) for the documented desktop path. `headless` disables the Gazebo GUI; it does not remove the GCS or DDS services.

## Stopping

After the simulated aircraft is landed and disarmed, run `./tools/sitl stop` in the second container shell. Detach tmux with **Ctrl-b, then d** if necessary. Exit both container shells to remove the container; the mounted data remains.

These commands are source-verified against the [Dockerfile](https://github.com/ANCL/px4-dev-toolkit/blob/main/docker/Dockerfile), [Compose file](https://github.com/ANCL/px4-dev-toolkit/blob/main/docker/compose.yaml), and [entrypoint](https://github.com/ANCL/px4-dev-toolkit/blob/main/docker/entrypoint.sh). Image pulls/builds and simulated flights were not executed as part of this documentation update.
