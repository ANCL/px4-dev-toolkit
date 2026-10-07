# Native installation

[Documentation](../index.md) / Installation / Native

Use the native path when you want the toolkit installed directly on the workstation for development, simulation, or physical-platform work.

## Supported host

These installation instructions target:

- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy
- Gazebo Harmonic / Gazebo Sim 8 series; the documented version is recorded in `config/versions.env`

Other hosts are not currently covered by these installation instructions.

## Clone

```bash
git clone https://github.com/ANCL/px4-dev-toolkit.git
cd px4-dev-toolkit
```

## Choose an installation profile

The installer exposes four profiles:

```bash
./setup/install.sh common
./setup/install.sh sitl
./setup/install.sh experiment
./setup/install.sh all
```

| Profile | Use it for | Adds |
|---|---|---|
| `common` | shared ROS/toolkit development | ROS 2 Jazzy, common tools, common pinned sources, ROS workspace build |
| `sitl` | PX4 + Gazebo simulation | common + SITL PX4 dependencies + MAVProxy + SITL sources |
| `experiment` | companion-computer physical stack | common + experiment PX4 dependencies + Vicon source |
| `all` | workstation that needs both environments | SITL + hardware sources and dependencies + MAVProxy + QGroundControl |

`GCS=qgc` is a valid SITL runtime choice, but the `sitl` installer profile installs MAVProxy, not QGroundControl. Use `all` or run `./setup/install_qgroundcontrol.sh` if you want QGroundControl managed by the toolkit.

The setup scripts may use `sudo` to install host packages.

## Source pin behavior

External repositories are declared in:

```text
config/sources/common.repos
config/sources/sitl.repos
config/sources/experiment.repos
```

`setup/fetch_sources.sh` does not reset already-present external repositories. If a source scope is partially populated, it refuses to modify it automatically rather than creating a mixed checkout.

## Build

The top-level installer builds the ROS 2 workspace automatically. To rebuild later:

```bash
./ros2/build.sh
```

The build helper deliberately uses a small clean environment so previously sourced ROS/PX4 paths do not leak into `colcon build`.

## Test

Run toolkit-maintained Python and ROS package tests with:

```bash
./ros2/test.sh
```

The test helper covers the toolkit-maintained ROS packages and analysis regression tests; fetched third-party repositories retain their upstream test suites.

## Runtime shell

Before invoking ROS 2 commands manually:

```bash
source ros2/runtime_env.sh
```

The user-facing `tools/sitl` and `tools/experiment` wrappers source the repository runtime environment inside the tmux processes they create.

## Verification

For a SITL installation:

```bash
./tools/sitl status
```

Then start it:

```bash
./tools/sitl start headless
```

For the `experiment` installation profile, review `config/runtime/experiment.env`, verify the serial device, then use:

```bash
./tools/experiment status
```

For physical hardware, continue with the [Hardware environment](../environments/hardware.md) checklist before flight.

## Next

- [SITL environment](../environments/sitl.md)
- [Hardware environment](../environments/hardware.md)
- [Configuration](../reference/configuration.md)
