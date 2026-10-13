---
hide:
  - toc
---

<div class="px4-hero" markdown="1">

# PX4 Development Toolkit

PX4 and ROS 2 flight-control experiments, in simulation and on hardware.

Run controller examples, record flight messages, and examine the results. The documentation connects the six SE(3) control laws to their implementation and explains how to analyze the recorded data.

<div class="px4-actions" markdown="1">

[Get started](installation/index.md){ .md-button .md-button--primary }
[Read the controller guide](controllers/overview.md)

</div>
</div>

<!-- Replace each image with approved muted/loop/playsinline video and a poster.
     Keep the 16:9 frame; respect reduced motion and provide pause/play. -->
<div class="px4-media-pair">
  <figure class="px4-media">
    <div class="px4-media-frame">
      <img src="assets/hero-gazebo-placeholder.svg" width="640" height="360" alt="Schematic quadrotor in a simulation volume; flight footage has not been supplied">
    </div>
    <figcaption><strong>Gazebo SITL</strong>Simulation flight · video placeholder</figcaption>
  </figure>
  <figure class="px4-media">
    <div class="px4-media-frame">
      <img src="assets/hero-f450-placeholder.svg" width="640" height="360" alt="Schematic quadrotor in an indoor flight area; lab footage has not been supplied">
    </div>
    <figcaption><strong>Physical flight in the lab</strong>Lab flight · video placeholder</figcaption>
  </figure>
</div>

<!-- Preserve existing section links. -->
<span id="choose-an-installation"></span>
<span id="suggested-first-journey"></span>
<span id="documentation"></span>

## Start here

<div class="px4-start-list" markdown="1">

- [Getting started](installation/index.md) — choose Docker or native installation, then run the first simulation.
- [Running the toolkit](environments/sitl.md) — start simulation services or prepare a [physical companion](environments/hardware.md).
- [Controllers and trajectories](controllers/overview.md) — see what each controller sends to PX4 and follow the derivations.
- [A complete experiment](experiments/complete-example.md) — record a flight, inspect the data, and compare SE(3) modes.
- [Adding to the toolkit](reference/extending.md) — add a vehicle, controller, trajectory, or analysis profile.
- [Technical reference](reference/architecture.md) — find the source, configuration files, and troubleshooting checks.

</div>

<!-- Preserve existing section links. -->
<span id="from-simulation-to-hardware"></span>
<span id="from-an-idea-to-a-recorded-flight"></span>

## Simulation and hardware

**Software-in-the-loop (SITL)** runs PX4 against the aircraft and sensors simulated by Gazebo. The maintained Docker image provides a compiled simulation environment. Native Ubuntu supports both SITL and companion-computer services for a physical vehicle, including motion-capture pose conversion and the serial DDS connection to PX4.

Environment services start separately from controllers and recording. On hardware, the pilot uses real RC or joystick controls; the virtual joystick supplies simulated manual input for the SITL examples.

<span id="understand-what-each-controller-calculates"></span>

## Controller interfaces

PX4 can track a position reference or accept acceleration, attitude, body-rate, or thrust-and-torque commands. The toolkit's SE(3) controller has six configurations that calculate different parts of this control stack. Start with the [controller overview](controllers/overview.md), then read the [mathematical foundations](controllers/se3/foundations.md) and the chapter for the selected mode.

Source code and contributions are on [GitHub](https://github.com/ANCL/px4-dev-toolkit).
