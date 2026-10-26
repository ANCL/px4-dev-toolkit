# PX4 Development Toolkit

A PX4 and ROS 2 environment for developing and studying flight control in Gazebo simulation and on physical multirotors.

The toolkit brings together reproducible runtime setup, controller examples, experiment recording, and flight-data analysis. Simulation and companion-computer installations are independent, and external source revisions are pinned.

**[Full documentation](https://ancl.github.io/px4-dev-toolkit/)** · **[Get started](https://ancl.github.io/px4-dev-toolkit/get-started/)** · **[Operations](https://ancl.github.io/px4-dev-toolkit/operations/)** · **[Controllers](https://ancl.github.io/px4-dev-toolkit/controllers/)** · **[Experiments](https://ancl.github.io/px4-dev-toolkit/experiments/sitl-example/)**

## Choose an environment

| Environment | Purpose | Installation |
|---|---|---|
| Native SITL | PX4, Gazebo Harmonic, and ROS 2 Jazzy on Ubuntu 24.04 | [Native Ubuntu](https://ancl.github.io/px4-dev-toolkit/get-started/sitl/ubuntu/) |
| Docker SITL | Prebuilt, headless simulation with controllers and analysis | [Docker](https://ancl.github.io/px4-dev-toolkit/get-started/sitl/docker/) |
| Companion computer | ROS 2 Jazzy, Vicon pose conversion, and a serial DDS agent for a physical PX4 flight controller | [Companion computer](https://ancl.github.io/px4-dev-toolkit/get-started/companion/) |

The companion profile does not install PX4 SITL or Gazebo. The physical vehicle needs its own PX4 firmware, airframe configuration, estimator setup, RC/failsafe provisions, and flight-readiness checks.

## Quick start: Docker SITL

On a Linux host with Docker, create persistent data directories and start the published SITL image:

```bash
mkdir -p bags log
docker pull ghcr.io/ancl/px4-dev-toolkit-sitl:latest
docker run --rm -it --name px4-sitl --network host \
  -v "$PWD/bags:/opt/px4-dev-toolkit/bags" \
  -v "$PWD/log:/opt/px4-dev-toolkit/log" \
  ghcr.io/ancl/px4-dev-toolkit-sitl:latest bash
```

Inside the container, start the simulation without a Gazebo GUI:

```bash
./tools/sitl start headless
```

Leave that container running. In a **second host terminal**, open another shell in it:

```bash
docker exec -it px4-sitl bash
```

In the second container shell, check PX4 and ROS 2 communication:

```bash
cd /opt/px4-dev-toolkit
source ros2/runtime_env.sh
./tools/sitl status
./tools/topic_rates
```

Stop the rate display with Ctrl-C. Starting SITL does **not** arm or fly the vehicle. Follow [First simulation](https://ancl.github.io/px4-dev-toolkit/get-started/first-simulation/) for the reference flight, and [Running SITL](https://ancl.github.io/px4-dev-toolkit/operations/sitl/) for landing, disarming, and shutdown. The [Docker guide](https://ancl.github.io/px4-dev-toolkit/get-started/sitl/docker/) explains volumes, permissions, and graphical options.

## Controllers and experiments

The toolkit includes a PX4 Position-mode reference flight, a separate Offboard staging helper, Offboard position control, and an SE(3) controller with **six documented configurations**: acceleration, attitude, attitude rate, geometric normalized torque, geometric physical torque, and a PX4 rate-loop reproduction. Each configuration delegates different stages of control to PX4; PX4 retains actuator allocation.

Use the [controller overview](https://ancl.github.io/px4-dev-toolkit/controllers/) to choose an interface and the [SE(3) documentation](https://ancl.github.io/px4-dev-toolkit/controllers/offboard/se3/) for mathematics, implementation, and vehicle requirements. The [trajectory guide](https://ancl.github.io/px4-dev-toolkit/experiments/trajectories/) describes shared reference motions.

Experiments can record rosbags, execute an ordered six-mode study, analyze individual runs, and compare compatible controller runs. Begin with the [complete experiment](https://ancl.github.io/px4-dev-toolkit/experiments/sitl-example/); [recording](https://ancl.github.io/px4-dev-toolkit/experiments/recording/), [sequences](https://ancl.github.io/px4-dev-toolkit/experiments/sequences/), and [analysis](https://ancl.github.io/px4-dev-toolkit/experiments/analysis/) are documented separately. A finalized bag is not, by itself, evidence of a successful or comparable flight.

## Source and extensions

`trajectory_generator/` contains reusable C++ trajectory generation and analytic sampling; `ros2/` contains ROS 2 packages and controllers; `tools/` contains runtime, recording-study, and analysis commands; `analysis/` defines report and comparison logic; `config/` holds vehicle, trajectory, runtime, topic, source, and study configuration. The toolkit fetches pinned external projects rather than vendoring them into the repository.

See [Software structure](https://ancl.github.io/px4-dev-toolkit/reference/architecture/), [Configuration files](https://ancl.github.io/px4-dev-toolkit/reference/configuration/), and [Extending the toolkit](https://ancl.github.io/px4-dev-toolkit/extending/) before adding a controller, vehicle, trajectory, or analysis profile. Contributions should include the affected behavior and verification results.

## License

Toolkit code is released under the [BSD 3-Clause License](LICENSE). Fetched third-party projects retain their upstream licenses.
