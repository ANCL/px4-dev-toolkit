# Trajectory Generator

`trajectory_generator` provides transport-independent C++ trajectory generation
and analytic sampling for the PX4 development toolkit.

- **Current method: analytic.** Segments support hold, step, smooth line, yaw,
  circle, helix and figure-eight trajectories. Ninth-order time scaling gives
  zero first-through-fourth derivatives at smooth segment endpoints.
- **Reference:** local PX4 NED position, velocity, acceleration, jerk, snap,
  yaw, yaw rate and yaw acceleration. It is *not* a full 6-DoF attitude reference.
- **Configuration:** `config/trajectories.yaml` is the single toolkit-owned
  definition. The `trajectory_generator_yaml` target loads it on the host.
- **Core:** `trajectory_generator` needs neither YAML nor ROS/PX4 headers.
  `Sequence::sample(t, origin)` evaluates an already prepared sequence.

## Build only the core

```sh
cmake -S trajectory_generator -B /tmp/trajectory-generator-build
cmake --build /tmp/trajectory-generator-build
```

Host applications can set `-DTRAJECTORY_GENERATOR_WITH_YAML=ON` to build the
YAML loader; the ROS 2 package enables that target automatically.

## Integration boundaries

ROS controllers link the generator directly: no added ROS topic, clock change,
or DDS traffic. The controller owns trajectory activation and sampling time.
A future general ROS reference publisher can be added as an *optional adapter*;
PX4 uORB integration belongs to an onboard module rather than this library.
Future optimized trajectories (for example, minimum snap) can create their own
representation or compiled segment plan while sharing the reference contract.

The current sequence uses `std::vector` and throws on invalid configuration;
embedded PX4 integration will need bounded prevalidated storage and nonthrowing
sampling semantics. Do not treat this host library as flight-controller-ready
until that interface is defined and verified.
