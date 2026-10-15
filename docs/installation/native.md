<!-- Preserve existing section links. -->
<span id="supported-host"></span>

# Native SITL installation

Use this page to install PX4 **software-in-the-loop** directly on **Ubuntu 24.04 (Noble)**. It installs **ROS 2 Jazzy Desktop**, **Gazebo Harmonic (Sim 8)**, PX4 SITL, MAVProxy, and the toolkit's ROS 2 controllers and analysis tools. Gazebo 8.15.0 was recorded during validation; see [tested tool versions](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/versions.env).

This is the simulation environment, not the physical companion-computer installation. For Vicon reception and serial DDS, use [Companion-computer installation](companion.md). For a prebuilt SITL environment, use [Docker](docker.md).

<!-- Preserve existing section links. -->
<span id="clone"></span>
<span id="choose-an-installation-profile"></span>

## Clone and install

From a **host terminal**:

```bash
git clone https://github.com/ANCL/px4-dev-toolkit.git
cd px4-dev-toolkit
./setup/install.sh sitl
./px4/build_sitl.sh
```

The installer installs host dependencies through `sudo`, fetches the **SITL** pinned repositories (including PX4, `px4_msgs`, Micro XRCE-DDS Agent, and MAVProxy), installs MAVProxy, and builds the SITL ROS 2 workspace. The separate `build_sitl.sh` compiles the configured PX4 SITL target; it does **not** start Gazebo or fly the vehicle. The process can take substantial time and disk space on a fresh machine.

<!-- Preserve existing section links. -->
<span id="source-pin-behavior"></span>

## Pinned external sources

The complete simulation source manifest is [config/sources/sitl.repos](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/sources/sitl.repos). Its PX4 pin is `9af25b30f067c36269e94a96797be7943dd4f8e1`.

`setup/fetch_sources.sh sitl` preserves existing full source sets rather than resetting them. If only part of a pinned source set exists, fetching stops rather than mixing revisions. Resolve partial checkouts deliberately; see [Installation troubleshooting](../reference/troubleshooting.md#installation).

<!-- Preserve existing section links. -->
<span id="build"></span>
<span id="test"></span>
<span id="runtime-shell"></span>

## Rebuild and prepare a runtime shell

The installer already builds ROS 2. After changing a maintained ROS package, from the **repository root** run:

```bash
./ros2/build.sh sitl
./ros2/test.sh sitl
```

Source the runtime environment in **each new terminal** used for manual ROS 2 commands:

```bash
source ros2/runtime_env.sh
ros2 pkg list
```

The runtime helper must be sourced, not executed. The ROS build and test commands do not launch the simulator or arm an aircraft.

<!-- Preserve existing section links. -->
<span id="verification"></span>
<span id="next"></span>

## Verify and continue

SITL uses **MAVProxy** by default (`GCS=mavproxy` in `config/runtime/sitl.env`). QGroundControl is optional: install the managed AppImage with `./setup/install_qgroundcontrol.sh` and choose `GCS=qgc` if that frontend is needed. The provided virtual-joystick staging demonstrations require the MAVProxy configuration, so retain the default for the first flight.

From the repository root:

```bash
./tools/sitl status
```

Before the first launch a stopped session is expected. Continue with [First simulation](../getting-started/first-simulation.md) to start Gazebo and PX4, verify DDS communication, and run the Position-mode reference flight.
