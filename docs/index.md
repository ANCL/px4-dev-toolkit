# PX4 Development Toolkit documentation

Use this page to go directly to the task you are trying to complete. You should not need to browse the repository tree to discover the workflow.

## Install the toolkit

Choose one installation method:

- **[Docker](installation/docker.md)** — use the published images immediately, or clone the repository and build the toolkit images with Compose.
- **[Native](installation/native.md)** — install ROS 2, PX4 dependencies, pinned sources, and the toolkit workspace directly on Ubuntu 24.04.

Docker and native installation are alternatives. The runtime pages below are the same conceptual workflows after installation.

## Run an environment

- **[SITL](environments/sitl.md)** — PX4 SITL + Gazebo + uXRCE-DDS + MAVProxy/QGroundControl.
- **[Hardware](environments/hardware.md)** — physical PX4 + serial uXRCE-DDS + Vicon/mocap bridge.

## Run a controller

Start with **[Controller overview](controllers/overview.md)** to choose the right workflow.

- **[SE(3)](controllers/se3.md)** — controller architecture, handoff modes, direct rotational controllers, frames, timing, and vehicle requirements.
- **[Trajectories](controllers/trajectories.md)** — shared NED trajectory definitions and timing/anchoring semantics.

## Run repeatable experiments

- **[Recording](workflows/recording.md)** — how controller launch files own rosbag lifecycle and bag placement.
- **[Sequences](workflows/sequences.md)** — run ordered controller studies with deterministic bag paths, logs, manifests, and settle intervals.

## Analyze results

- **[Single-run analysis](analysis/analysis.md)** — `px4_analyze`, profile detection, summaries, plots, and timing alignment.
- **[Comparison](analysis/comparison.md)** — `px4_compare`, confirmed-Offboard alignment, controller labels, and trajectory-segment metrics.

## Understand or extend the toolkit

These pages are for development and deeper system work; they are not prerequisites for the basic run workflow.

- **[Architecture](reference/architecture.md)** — how the repository, runtime environments, ROS 2 packages, and PX4 data flow fit together.
- **[Configuration](reference/configuration.md)** — where each kind of configuration belongs.
- **[Extending](reference/extending.md)** — add controllers, trajectories, vehicles, sequences, analysis profiles, tools, ROS packages, or external sources.
- **[Troubleshooting](reference/troubleshooting.md)** — recurring actionable failures and checks.

## Suggested first journey

For a new simulation user:

1. [Docker installation](installation/docker.md) or [Native installation](installation/native.md)
2. [Run SITL](environments/sitl.md)
3. [Controller overview](controllers/overview.md)
4. [Run SE(3)](controllers/se3.md)
5. [Record](workflows/recording.md) or [run a sequence](workflows/sequences.md)
6. [Analyze](analysis/analysis.md) and [compare](analysis/comparison.md)

[Back to repository README](../README.md)
