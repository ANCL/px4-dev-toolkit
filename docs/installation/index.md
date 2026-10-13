# Getting started

The toolkit supports two independent environments: **SITL**, where PX4 and Gazebo run on a workstation, and **companion computer**, where ROS 2 communicates with PX4 firmware already running on a physical flight controller. Choose the environment for your machine; neither installation requires the other.

## SITL — simulation

**[Native Ubuntu installation](native.md)** installs PX4, Gazebo Harmonic, ROS 2 Jazzy, and the toolkit directly on Ubuntu 24.04. Choose this for desktop simulation and controller development.

**[Docker installation](docker.md)** provides a prebuilt SITL environment on a Linux host. The documented example runs headless; using the Gazebo GUI requires host-specific display configuration following the official PX4 guide linked there.

Both paths support the same SITL workflow. Start with [First simulation](../getting-started/first-simulation.md) after installing one of them.

## Companion computer — physical hardware

**[Companion-computer installation](companion.md)** installs ROS 2 Jazzy, the Vicon receiver, mocap conversion, and the serial DDS agent on Ubuntu 24.04. It does **not** install PX4 SITL, Gazebo, or MAVProxy. The PX4 flight controller and physical system require their own firmware and configuration.

For live services and flight-readiness checks, use the separate [Physical hardware guide](../environments/hardware.md). The automated SITL takeoff example is not a physical-flight procedure.

## Before installing

Native installations require Ubuntu 24.04, Git, network access to fetch pinned dependencies, sufficient disk space, and permission to install host packages with `sudo`. Docker requires a working Docker Engine on Linux and writable storage for bags and logs.

Both native profiles retrieve their own complete pinned source sets, including the shared PX4 message and DDS agent revisions. See [source pins and validated versions](../reference/configuration.md#missing-data-and-source-pins). Installation alone does not establish a flight-ready vehicle.
