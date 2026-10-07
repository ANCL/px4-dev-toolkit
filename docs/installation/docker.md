# Docker installation

[Documentation](../index.md) / Installation / Docker

Use Docker when you want an isolated toolkit environment with the dependencies already prepared. There are two distinct workflows:

1. **Use the published images** — no repository checkout is required.
2. **Clone the repository and build with Compose** — use this when modifying or rebuilding the toolkit.

Compose is contributor/development infrastructure; it is not required to use the published images.

## Requirements

- Docker Engine on a Linux host
- host networking support for the documented runtime commands
- for the hardware image, access to the PX4 serial device and the Vicon network

The published images currently target Linux `amd64`.

## Files inside the container

Interactive shells start in the toolkit root:

```text
/opt/px4-dev-toolkit
```

Useful paths:

| Content | Path |
|---|---|
| PX4 source | `/opt/px4-dev-toolkit/px4/PX4-Autopilot` |
| ROS 2 workspace | `/opt/px4-dev-toolkit/ros2` |
| ROS 2 packages and pinned ROS sources | `/opt/px4-dev-toolkit/ros2/src` |
| configuration | `/opt/px4-dev-toolkit/config` |
| analysis | `/opt/px4-dev-toolkit/analysis` |
| user-facing tools | `/opt/px4-dev-toolkit/tools` |
| recorded bags | `/opt/px4-dev-toolkit/bags` |
| runtime logs | `/opt/px4-dev-toolkit/log` |

The SITL image also contains MAVProxy under `/opt/px4-dev-toolkit/tools/MAVProxy`. The hardware image contains the pinned Vicon receiver under `/opt/px4-dev-toolkit/ros2/src/vicon_receiver`.

## Option A: use the published images

### SITL image

```bash
docker pull ghcr.io/ancl/px4-dev-toolkit-sitl:latest

docker run --rm -it --network host \
  ghcr.io/ancl/px4-dev-toolkit-sitl:latest bash
```

Inside the container:

```bash
./tools/sitl start headless
```

`headless` disables the Gazebo GUI; PX4, the Gazebo server, ROS 2 connectivity, and the configured ground-control frontend still run.

If you want bags and logs to persist after `--rm`, create host directories and bind-mount them:

```bash
mkdir -p bags log

docker run --rm -it --network host \
  -v "$PWD/bags:/opt/px4-dev-toolkit/bags" \
  -v "$PWD/log:/opt/px4-dev-toolkit/log" \
  ghcr.io/ancl/px4-dev-toolkit-sitl:latest bash
```

### Hardware image

```bash
docker pull ghcr.io/ancl/px4-dev-toolkit-experiment:latest
```

The container uses `/dev/ttyUSB0` as the canonical PX4 XRCE serial device. Map the actual host device to that path. For example, if the host exposes `/dev/ttyACM0`:

```bash
docker run --rm -it --network host \
  --device=/dev/ttyACM0:/dev/ttyUSB0 \
  ghcr.io/ancl/px4-dev-toolkit-experiment:latest bash
```

No image rebuild is required when the host serial-device name changes.

The hardware stack also needs site-specific Vicon/runtime settings. First create writable host data directories:

```bash
mkdir -p bags log
```

Then create an editable host copy of the image's runtime file:

```bash
docker run --rm \
  ghcr.io/ancl/px4-dev-toolkit-experiment:latest \
  cat /opt/px4-dev-toolkit/config/runtime/experiment.env \
  > experiment.env
```

Edit `experiment.env`, then mount it over the canonical runtime file:

```bash
docker run --rm -it --network host \
  --device=/dev/ttyACM0:/dev/ttyUSB0 \
  -v "$PWD/experiment.env:/opt/px4-dev-toolkit/config/runtime/experiment.env:ro" \
  -v "$PWD/bags:/opt/px4-dev-toolkit/bags" \
  -v "$PWD/log:/opt/px4-dev-toolkit/log" \
  ghcr.io/ancl/px4-dev-toolkit-experiment:latest bash
```

Then use the [Hardware environment](../environments/hardware.md) workflow.

## Working from an editor

For an interactive development container, give the container a stable name and keep it running:

```bash
docker run -d --name px4-sitl --network host \
  ghcr.io/ancl/px4-dev-toolkit-sitl:latest \
  sleep infinity
```

You can then attach a shell with:

```bash
docker exec -it px4-sitl bash
```

VS Code's Dev Containers extension can also attach to the running container. This is an attachment workflow; the repository does not currently require a `.devcontainer` configuration.

## Option B: clone and build with Compose

Clone the repository:

```bash
git clone https://github.com/ANCL/px4-dev-toolkit.git
cd px4-dev-toolkit
```

Build one target:

```bash
docker compose -f docker/compose.yaml build sitl
```

or:

```bash
docker compose -f docker/compose.yaml build experiment
```

Build both:

```bash
docker compose -f docker/compose.yaml build sitl experiment
```

Open an interactive SITL container:

```bash
docker compose -f docker/compose.yaml run --rm sitl bash
```

For the experiment service, the Compose file maps the host device selected by `PX4_DOCKER_DEVICE` to `/dev/ttyUSB0` in the container. Example:

```bash
PX4_DOCKER_DEVICE=/dev/ttyACM0 \
  docker compose -f docker/compose.yaml run --rm experiment bash
```

Compose uses named volumes for `bags/` and `log/` and bind-mounts the repository runtime `.env` files read-only.

## What is inside each image

Both images include the toolkit source, ROS 2 workspace, analysis code, runtime tools, configuration, and pinned common sources.

The **SITL** target additionally includes the pinned SITL source set, PX4 simulation dependencies, MAVProxy, a built PX4 SITL target, and the built ROS 2 workspace.

The **experiment** target additionally includes the pinned Vicon receiver source, physical-platform PX4 dependencies, and the built ROS 2 workspace.

## Verification

For SITL, the practical verification is:

```bash
./tools/sitl start headless
```

and, from another shell in the same environment:

```bash
./tools/sitl status
```

For the hardware image, verify the serial device exists inside the container and review the Vicon settings before starting:

```bash
ls -l /dev/ttyUSB0
cat config/runtime/experiment.env
./tools/experiment status
```

## Next

- [Run SITL](../environments/sitl.md)
- [Run the hardware environment](../environments/hardware.md)
- [Troubleshooting](../reference/troubleshooting.md)
