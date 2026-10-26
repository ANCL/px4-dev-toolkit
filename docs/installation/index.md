# Getting started

Choose the environment that matches your setup: **SITL** runs PX4 and Gazebo on a workstation; the **companion computer** connects ROS 2 and motion capture to a physical PX4 flight controller.

## SITL — simulation

**[Native Ubuntu installation](native.md)** installs PX4, Gazebo Harmonic, ROS 2 Jazzy, and the toolkit directly on Ubuntu 24.04. Choose this for desktop simulation and controller development.

**[Docker installation](docker.md)** provides a prebuilt SITL environment on a Linux host. The documented example runs headless; using the Gazebo GUI requires host-specific display configuration following the official PX4 guide linked there.

After installing either option, begin with [First simulation](../getting-started/first-simulation.md).

## Companion computer — physical hardware

**[Companion-computer installation](companion.md)** installs ROS 2 Jazzy, the Vicon receiver, mocap conversion, and the serial DDS agent on Ubuntu 24.04. Configure the aircraft firmware and flight systems separately.

For setup checks and flight preparation, use the [Companion Computer guide](../environments/hardware.md).

## Before installing

Native installations require Ubuntu 24.04, Git, network access to fetch pinned dependencies, sufficient disk space, and permission to install host packages with `sudo`. Docker requires a working Docker Engine on Linux and writable storage for bags and logs.

Both native profiles retrieve their own complete pinned source sets, including the shared PX4 message and DDS agent revisions. See [source pins and validated versions](../reference/configuration.md#missing-data-and-source-pins).
