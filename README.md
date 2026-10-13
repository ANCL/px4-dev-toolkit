# PX4 Development Toolkit

Develop, test, record, and analyze PX4 and ROS 2 controllers in Gazebo and on physical vehicles.

The toolkit combines PX4 simulation, native companion-computer integration, Position-mode examples, Offboard position and SE(3) controllers, rosbag recording, and flight-data analysis. External sources are pinned separately from the toolkit code.

**[Full documentation](https://ancl.github.io/px4-dev-toolkit/)** · **[Getting started](https://ancl.github.io/px4-dev-toolkit/installation/)**

## Quick start: Docker SITL

On a Linux host with Docker, open a **host terminal**. This example uses the maintained headless simulation image and saves data in the current directory:

```bash
mkdir -p bags log
docker pull ghcr.io/ancl/px4-dev-toolkit-sitl:latest
docker run --rm -it --name px4-sitl --network host \
  -v "$PWD/bags:/opt/px4-dev-toolkit/bags" \
  -v "$PWD/log:/opt/px4-dev-toolkit/log" \
  ghcr.io/ancl/px4-dev-toolkit-sitl:latest bash
```

**Inside the container**, at `/opt/px4-dev-toolkit`:

```bash
./tools/sitl start headless
```

The launcher attaches to tmux. Open a **second host terminal** and enter the same container:

```bash
docker exec -it px4-sitl bash
```

**In that container shell**, source the built ROS environment and check communication:

```bash
cd /opt/px4-dev-toolkit
source ros2/runtime_env.sh
./tools/sitl status
ros2 topic list
```

Starting the environment does not arm or fly the aircraft. Continue with [First simulation](https://ancl.github.io/px4-dev-toolkit/getting-started/first-simulation/) for a Position-mode demonstration and stopping instructions. When the simulated aircraft is landed and disarmed, run `./tools/sitl stop` inside the container, then exit both container shells.

The commands are checked against the tracked interfaces; a new host still needs its own image, graphics, and communication checks. See the [Docker guide](docs/installation/docker.md) for persistence and graphical use.

## Simulation and physical hardware

Docker supports SITL. [Native installation](docs/installation/native.md) on Ubuntu 24.04 with ROS 2 Jazzy also supports the physical companion services: Vicon pose reception, conversion to PX4 odometry, and a serial uXRCE-DDS link. The [hardware guide](docs/environments/hardware.md) explains the services and the checks required on the actual vehicle.

## Read further

- [Controllers](docs/controllers/overview.md) and [SE(3) mathematics](docs/controllers/se3/foundations.md)
- [A complete experiment](docs/experiments/complete-example.md), [analysis](docs/analysis/analysis.md), and [comparison](docs/analysis/comparison.md)
- [Adding a vehicle or controller](docs/reference/extending.md)
- [Software structure](docs/reference/architecture.md), [configuration](docs/reference/configuration.md), and [troubleshooting](docs/reference/troubleshooting.md)

For contributions, open an [issue](https://github.com/ANCL/px4-dev-toolkit/issues) or [pull request](https://github.com/ANCL/px4-dev-toolkit/pulls) with the affected behavior, source, and verification results. Keep configuration and documentation aligned with the implementation.

## License

Toolkit code uses the [BSD 3-Clause License](LICENSE). Fetched third-party projects retain their upstream licenses.
