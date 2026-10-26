<span id="companion-computer-installation"></span>

# Companion setup

Install the companion services on **Ubuntu 24.04** with **ROS 2 Jazzy Base**. The hardware profile includes the Vicon receiver, pose conversion, DDS agent, controller tools, and MAVLink connectivity to PX4.

## Install on the companion computer

In a terminal on the **Ubuntu companion**, use a separate toolkit checkout:

```bash
git clone https://github.com/ANCL/px4-dev-toolkit.git
cd px4-dev-toolkit
./setup/install.sh hardware
```

The installer sets up ROS 2 Base, MAVLink Router and transfer tools, Vicon receiver, and the hardware ROS packages from [config/sources/hardware.repos](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/config/sources/hardware.repos). PX4 firmware is installed separately on the aircraft. Analyze recorded bags and ULogs on an analysis workstation.

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

Edit [config/runtime/hardware.env](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/config/runtime/hardware.env) for your actual setup. Check the serial device and baud rate against the PX4 uXRCE-DDS client, and configure the Vicon server, rigid-body topic, frames, mapping transform, and timestamp behavior.

The current hardware launcher supports a **serial** DDS-agent connection. MAVLink uses a separate device configured by `MAVLINK_MASTER` and `MAVLINK_BAUD`; it cannot share `XRCE_AGENT_DEVICE`. Prefer an explicit stable `/dev/serial/by-id/…` device. The `auto` setting accepts exactly one matching PX4 USB device after excluding the DDS device. `MAVLINK_QGC_OUT` optionally forwards MAVLink to QGroundControl; the launcher does not start a hardware GCS.

Confirm both serial devices are accessible, the Vicon server publishes the intended rigid body, and the configured transforms match your laboratory setup.

## Verify services before flight

First inspect the configuration and the environment without arming a vehicle:

```bash
./tools/hardware status
```

A stopped session is normal before startup. Once the wiring, Vicon configuration, and PX4-side connection have been checked, use the [Hardware runtime guide](../environments/hardware.md) to start the companion services, inspect Vicon and converted odometry, and verify the PX4 estimator and safety setup.

The launcher provides a PX4 MAVLink Shell; see [Tools](../operations/tools.md) for commands.
