# Docker installation — SITL

[Documentation](../index.md) / Installation / Docker

The maintained Docker distribution runs **PX4 SITL with Gazebo**. The physical
companion-computer installation is native-only; see [Native installation](native.md)
and the [Hardware environment](../environments/hardware.md).

## Published SITL image

```bash
docker pull ghcr.io/ancl/px4-dev-toolkit-sitl:latest

docker run --rm -it --network host \
  ghcr.io/ancl/px4-dev-toolkit-sitl:latest bash
```

Inside the container:

```bash
./tools/sitl start headless
```

The image includes Ubuntu 24.04, ROS 2 Jazzy, the pinned PX4-Autopilot SITL
source and build, Gazebo Harmonic, uXRCE-DDS agent, MAVProxy with its virtual
joystick, maintained controllers, recording, and analysis tools.

The image intentionally excludes the Vicon receiver, mocap bridge, hardware
launcher, serial hardware device mappings, and PX4 firmware cross-compilers.

## Persistent data

```bash
mkdir -p bags log

docker run --rm -it --network host \
  -v "$PWD/bags:/opt/px4-dev-toolkit/bags" \
  -v "$PWD/log:/opt/px4-dev-toolkit/log" \
  ghcr.io/ancl/px4-dev-toolkit-sitl:latest bash
```

Configuration can be overridden with a read-only bind mount for
`config/runtime/sitl.env`. Runtime files are under `/opt/px4-dev-toolkit/`.

## Building the image locally

```bash
git clone https://github.com/ANCL/px4-dev-toolkit.git
cd px4-dev-toolkit

docker compose -f docker/compose.yaml build sitl
docker compose -f docker/compose.yaml run --rm sitl bash
```

Only the `sitl` Compose service and Docker build target are supported.
Compose persists `bags/` and `log/` in named volumes.

## Verification

Inside the running container, check:

```bash
./tools/sitl start headless
```

In another shell attached to the same container:

```bash
./tools/sitl status
source ros2/runtime_env.sh
ros2 topic list
```

The container image is not a substitute for configuring and validating
physical PX4 flight hardware.

## Further reading

- [SITL environment](../environments/sitl.md)
- [Native companion installation](native.md)
- [Configuration ownership](../reference/configuration.md)
