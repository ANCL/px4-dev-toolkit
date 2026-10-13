---
hide:
  - toc
---

<div class="px4-hero" markdown="1">

# PX4 Development Toolkit

Develop, test, record, and analyze PX4 and ROS 2 controllers in Gazebo and on physical vehicles. Keep the flight software, controller examples, and tools for understanding a flight together.

<div class="px4-actions" markdown="1">

[Get started](installation/index.md){ .md-button .md-button--primary }
[Explore the documentation](#start-here)
[GitHub repository](https://github.com/ANCL/px4-dev-toolkit)

</div>
</div>

<!-- Both frames are static, honest placeholders. Replace an img with approved
     muted/loop/playsinline video and a poster without changing the paired layout.
     Any future autoplay must respect reduced motion and offer pause/play. -->
<div class="px4-media-pair">
  <figure class="px4-media">
    <div class="px4-media-frame">
      <img src="assets/hero-gazebo-placeholder.svg" width="640" height="360" alt="Neutral simulation graphic; placeholder for a future Gazebo SITL flight video">
    </div>
    <figcaption><strong>Gazebo SITL</strong>Simulation flight video coming soon.</figcaption>
  </figure>
  <figure class="px4-media">
    <div class="px4-media-frame">
      <img src="assets/hero-f450-placeholder.svg" width="640" height="360" alt="Neutral indoor-flight graphic; placeholder for a future lab flight video">
    </div>
    <figcaption><strong>Lab flight</strong>Physical flight video coming soon.</figcaption>
  </figure>
</div>

## From an idea to a recorded flight

Set up the environment, run a flight, record its signals, then read the results. **Software-in-the-loop (SITL)** runs PX4, the aircraft's flight-control software, against a Gazebo simulation. On a physical vehicle, a native companion computer connects **ROS 2**, the message-based robotics framework, to PX4 and the motion-capture system.

Docker provides the maintained simulation image. Native Ubuntu supports both simulation and physical companion services; each environment starts independently of the controller.

## Understand what each controller calculates

PX4 can track a position reference, or accept commands farther down its control stack. The toolkit's **SE(3)** controller describes position and rotation together and offers six configurations, from acceleration requests to direct thrust and torque. The [controller overview](controllers/overview.md) explains the choices; the [mathematical chapters](controllers/se3/foundations.md) connect them to the implementation.

## Start here

<div class="px4-start-list" markdown="1">

- [Getting started](installation/index.md) — install the environment and run your first simulation.
- [Running the toolkit](environments/sitl.md) — inspect simulation services or prepare a [physical companion](environments/hardware.md).
- [Controllers and trajectories](controllers/overview.md) — follow a reference and understand how commands reach PX4.
- [A complete experiment](experiments/complete-example.md) — record, analyze, and compare runs.
- [Adding to the toolkit](reference/extending.md) — introduce a vehicle or controller with documented verification.
- [Technical reference](reference/architecture.md) — find the source, configuration, and troubleshooting checks.

</div>
