<!-- Preserve existing section links. -->
<span id="supported-host"></span>

# Native installation

Install directly on Ubuntu when you want a local development workspace or the physical companion services. The documented host is **Ubuntu 24.04 (Noble)** with **ROS 2 Jazzy** and, for SITL, **Gazebo Harmonic (Sim 8)**. The recorded validated Gazebo version is 8.15.0; tool versions are listed in [config/versions.env](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/versions.env).

<!-- Preserve existing section links. -->
<span id="clone"></span>
<span id="choose-an-installation-profile"></span>

## Clone and install

In a **host terminal**:

```bash
git clone https://github.com/ANCL/px4-dev-toolkit.git
cd px4-dev-toolkit
./setup/install.sh sitl
./px4/build_sitl.sh
```

The installer uses the repository's supported setup scripts, which can install host packages through `sudo`. It fetches the pinned sources and builds the ROS workspace. The separate `build_sitl.sh` prebuilds PX4 using the configured target; it does not launch a flight.

Choose **one** installer profile appropriate to the machine:

| Profile | What it adds |
|---|---|
| `common` | ROS-base, common tools and pinned sources, ROS workspace build |
| `sitl` | ROS desktop, common sources, PX4/Gazebo dependencies, SITL sources, MAVProxy, ROS build |
| `hardware` | ROS-base, companion tools, common/hardware sources including Vicon, ROS build |
| `all` | Both source sets, desktop/SITL dependencies, MAVProxy, QGroundControl, ROS build |

For a **companion-computer checkout**, use `./setup/install.sh hardware` in place of `sitl` and omit `build_sitl.sh`. That profile does not fetch PX4 firmware sources, simulator dependencies, or ARM cross-compilers. `common` and `hardware` use ROS-base; they do not install the graphical ROS desktop.

<!-- Preserve existing section links. -->
<span id="source-pin-behavior"></span>

## Pinned external sources

The [source manifests](https://github.com/ANCL/px4-dev-toolkit/tree/main/config/sources) specify exact revisions for PX4, PX4 ROS messages, the DDS agent, MAVProxy, and Vicon. The current PX4 pin is `f5083ca2c5b919350880e8667e636ef70715db17`.

`setup/fetch_sources.sh` leaves already-populated scopes alone and refuses partial source sets. It does not reset an existing external checkout to make an install appear successful. If it reports a partial set, inspect and preserve those checkouts before recovery; see [Troubleshooting](../reference/troubleshooting.md#installation).

<!-- Preserve existing section links. -->
<span id="build"></span>
<span id="test"></span>
<span id="runtime-shell"></span>

## Rebuild and prepare a runtime shell

The installer already builds ROS. After changing a maintained package, rebuild from the **repository root**:

```bash
./ros2/build.sh
```

The build helper starts with a small environment to avoid inherited ROS/Python paths. In **each terminal used for manual ROS commands**, source the result:

```bash
source ros2/runtime_env.sh
ros2 pkg list
```

The environment helper checks both the ROS distribution and `ros2/install/setup.bash`. It must be sourced rather than executed. A successful build and the presence of packages verify installation; they do not verify flight behavior.

<!-- Preserve existing section links. -->
<span id="verification"></span>
<span id="next"></span>

## Ground control and verification

The `sitl` profile installs MAVProxy. If you select `GCS=qgc`, install QGroundControl with `./setup/install_qgroundcontrol.sh` or use the `all` profile. The managed AppImage is pinned in `config/versions.env`; QGroundControl is not required for the headless MAVProxy example.

From the **repository root**, inspect the configured simulation:

```bash
./tools/sitl status
```

A stopped session before the first run is expected. Continue with [First simulation](../getting-started/first-simulation.md). For native hardware, review [Physical hardware](../environments/hardware.md) before starting companion services.
