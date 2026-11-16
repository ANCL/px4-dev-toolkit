<span id="companion-computer-installation"></span>

# Companion setup

Install the companion services on **Ubuntu 24.04** with **ROS 2 Jazzy Base**. The hardware profile includes Vicon reception, pose conversion, the DDS agent and controller tools; QGroundControl telemetry is independent.

## Install on the companion computer

In a terminal on the **Ubuntu companion**, use a separate toolkit checkout:

```bash
git clone https://github.com/ANCL/px4-dev-toolkit.git
cd px4-dev-toolkit
./setup/install.sh hardware
```

The installer sets up ROS 2 Base, Vicon reception, DDS support and the hardware ROS packages from [config/sources/hardware.repos](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/sources/hardware.repos). [PX4 firmware](firmware.md) is built and flashed separately onto the flight controller. Analyze recorded bags and ULogs on an analysis workstation.

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

The DDS agent uses the configured serial link; QGroundControl connects to the flight controller independently over the existing telemetry link. The hardware launcher does not start a GCS or a MAVLink relay. DDS is separate from MAVLink.

Confirm the DDS serial device is accessible, the Vicon server publishes the intended rigid body, and the configured transforms match your laboratory setup.

## Verify services before flight

First inspect the configuration and the environment without arming a vehicle:

```bash
./tools/hardware status
```

A stopped session is normal before startup. Once the wiring, Vicon configuration, and PX4-side connection have been checked, use the [Hardware runtime guide](../operations/companion.md) to start the companion services, inspect Vicon and converted odometry, and verify the PX4 estimator and safety setup.

For native PX4 commands use QGroundControl's MAVLink Console; see [Tools](../operations/tools.md) for the terminal distinction.
