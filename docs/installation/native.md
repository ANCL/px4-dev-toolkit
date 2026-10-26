<!-- Preserve existing section links. -->
<span id="supported-host"></span>

<span id="native-sitl-installation"></span>

# Native SITL

Install PX4 SITL, Gazebo Harmonic, ROS 2 Jazzy Desktop, MAVProxy, controllers, and analysis tools on **Ubuntu 24.04**. Tested versions are pinned in [config/versions.env](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/config/versions.env).

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

The installer fetches pinned SITL sources and builds the ROS 2 workspace, MAVLink transfer tools, and analysis dependencies. `./px4/build_sitl.sh` builds the configured PX4 SITL target separately.

<!-- Preserve existing section links. -->
<span id="source-pin-behavior"></span>

## Pinned external sources

The [SITL source manifest](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/config/sources/sitl.repos) pins `ANCL/PX4-Autopilot` to `5dd32eb1e7ab8b79b8ece462ce5f22cd75272c56`.

`setup/fetch_sources.sh sitl` preserves existing full source sets rather than resetting them. If only part of a pinned source set exists, fetching stops rather than mixing revisions. Resolve partial checkouts deliberately; see [Installation troubleshooting](../reference/troubleshooting.md#installation).

<!-- Preserve existing section links. -->
<span id="build"></span>
<span id="runtime-shell"></span>

## Rebuild and prepare a runtime shell

The installer already builds ROS 2. After changing a maintained ROS package, from the **repository root** run:

```bash
./ros2/build.sh sitl
```

Source the runtime environment in **each new terminal** used for manual ROS 2 commands:

```bash
source ros2/runtime_env.sh
ros2 pkg list
```

The runtime helper must be sourced, not executed. Launch the simulator separately using `./tools/sitl start`.

<!-- Preserve existing section links. -->
<span id="verification"></span>
<span id="next"></span>

## Verify and continue

SITL uses **MAVProxy** by default (`GCS=mavproxy` in `config/runtime/sitl.env`). QGroundControl is optional: install the managed AppImage with `./setup/install_qgroundcontrol.sh` and choose `GCS=qgc` if that frontend is needed. The provided virtual-joystick staging demonstrations require the MAVProxy configuration, so retain the default for the first flight.

From the repository root:

```bash
./tools/sitl status
```

Continue with [First simulation](../getting-started/first-simulation.md).
