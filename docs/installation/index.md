# Getting started

The toolkit connects PX4 flight software to ROS 2 controllers, records the messages from a flight, and analyzes them afterward. Begin with **software-in-the-loop (SITL)**: Gazebo supplies the simulated aircraft and sensors, while PX4 runs the flight software on your computer.

## Choose an installation

[Docker installation](docker.md) gives you the maintained, compiled SITL environment. It is useful when you want a consistent simulation setup without installing ROS and Gazebo on the host. The documented commands assume Linux host networking; graphical use also needs a working host display connection. The image does not contain the physical Vicon/serial companion stack.

[Native installation](native.md) targets **Ubuntu 24.04 and ROS 2 Jazzy**. Use it to develop directly on the workstation, run Gazebo, or prepare a companion computer for a physical vehicle. The four installer profiles let you install only the pieces needed by that machine.

## Before the first run

You need Git for a native checkout or local image build, enough disk space for source/build files, network access for dependencies, and a terminal. Docker needs a working Docker engine. A graphical simulation needs a supported display/graphics setup; headless operation runs the simulator without its GUI.

The [source manifests and tool versions](../reference/configuration.md#missing-data-and-source-pins) identify the supported dependencies. After installation, verify communication and PX4 state before starting a flight example; graphics and networking depend on the host.

Once installed, continue with [First simulation](../getting-started/first-simulation.md).
