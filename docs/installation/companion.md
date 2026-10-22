# Companion-computer installation

Use this installation for the **ROS 2 companion computer connected to a PX4 flight controller**. The supported host is **Ubuntu 24.04 (Noble)** with **ROS 2 Jazzy Base**. It includes the toolkit's controllers, Vicon receiver, mocap-to-PX4 odometry bridge, and Micro XRCE-DDS Agent for a serial link.

This is independent of SITL: **PX4-Autopilot sources, Gazebo, MAVProxy, and graphical ROS Desktop are not required or installed by this profile**. PX4 firmware runs on the flight controller, not the companion. For simulation use [Native SITL](native.md) or [Docker SITL](docker.md).

## Install on the companion computer

In a terminal on the **Ubuntu companion**, use a separate toolkit checkout:

```bash
git clone https://github.com/ANCL/px4-dev-toolkit.git
cd px4-dev-toolkit
./setup/install.sh hardware
```

This installs the lean ROS 2 Base environment and required build tools, fetches the pinned source repositories, and builds the hardware ROS 2 packages. The complete independent dependency set is defined in [config/sources/hardware.repos](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/sources/hardware.repos): `px4_msgs`, Micro XRCE-DDS Agent, and Vicon receiver. It does **not** build or flash PX4 firmware.

`setup/fetch_sources.sh hardware` preserves complete existing source checkouts and refuses partially populated sets. See [installation troubleshooting](../reference/troubleshooting.md#installation) if fetching stops.

## Build and source ROS 2

The installer already performs the initial build. After editing a maintained package, from the companion repository root:

```bash
./ros2/build.sh hardware
```

Source the runtime in **each terminal** used for manual ROS 2 commands:

```bash
source ros2/runtime_env.sh
ros2 pkg list
```

A successful build confirms compilation, not the serial connection, Vicon tracking, PX4 estimator, or aircraft readiness.

## Runtime YAML editing

Controller gains and trajectory YAML in the toolkit source checkout are
**authoritative at the next controller launch**. Restart the controller after
a YAML edit; a rebuild is not required. ROS code or launch-file edits still
require `./ros2/build.sh hardware` with the default copied installation.
The launch logs the selected controller, trajectory and vehicle configuration
paths. A standalone package without the checkout explicitly falls back to its
installed YAML copies.

## Configure connections

Edit [config/runtime/hardware.env](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/runtime/hardware.env) for your actual setup. Check the serial device and baud rate against the PX4 uXRCE-DDS client, and configure the Vicon server, rigid-body topic, frames, mapping transform, and timestamp behavior.

The current hardware launcher supports a **serial** DDS-agent connection. Confirm the serial device exists and is accessible to the companion user; confirm the Vicon machine is reachable and publishing the intended rigid body. Do not assume the example addresses, devices, or transforms match your lab.

## Verify services before flight

First inspect the configuration and the environment without arming a vehicle:

```bash
./tools/hardware status
```

A stopped session is normal before startup. Once the wiring, Vicon configuration, and PX4-side connection have been checked, use the [Hardware runtime guide](../environments/hardware.md) to start the companion services, inspect Vicon and converted odometry, and verify the PX4 estimator and safety setup.

Starting the companion services does **not** start firmware, arm, launch a flight controller, or make the vehicle flight-ready. Hardware operation requires real-device and pre-flight checks beyond a successful build.
