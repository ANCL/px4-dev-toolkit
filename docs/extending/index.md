<!-- Preserve existing section links. -->
<span id="extension-map"></span>
<span id="adding-to-the-toolkit"></span>

# Extending the toolkit

Choose the component to extend. [Configuration](../reference/configuration.md) maps existing settings to their owners.

## Choose an extension

- **[Vehicles](vehicles.md):** vehicle YAML, mass, frames, geometry, and calibration.
- **[Controllers](controllers.md):** ROS or native PX4 controller implementation and integration.
- **Other components:** follow the sections below.

<span id="add-a-controller"></span>
<span id="add-a-vehicle"></span>

## Other extension points

### Add a trajectory

Add a segment in [trajectories.yaml](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/trajectories.yaml). A new segment type needs an analytic sampler in `trajectory_generator/src/analytic/` and parser support in `trajectory_generator/src/config/yaml.cpp`. For onboard use, also update compiler serialization and the firmware loader/sampler. Check derivatives, units, segment joins, and [NED/time conventions](../experiments/trajectories.md).

### Add recording topics

Add topic symbols in [config/px4_topics.def](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/px4_topics.def) and select them through `config/recording/` profiles. Toolkit diagnostic topics may use absolute names. Update required/optional topic declarations in the analysis profile.

### Add a sequence

Add a YAML definition under `config/sequences/` with launch steps and arguments. The [handoff study](../experiments/sequences.md) illustrates recording paths, manifests, and settle time.

<!-- Preserve existing section links. -->
<span id="extend-comparison-analysis"></span>

### Add an analysis profile

Profiles live in [analysis/profiles](https://github.com/ANCL/px4-dev-toolkit/tree/main/analysis/profiles). Define `PROFILE_NAME`, required/optional topics, `analyze`, and reporting; register it in [`PROFILES`](https://github.com/ANCL/px4-dev-toolkit/blob/main/analysis/profiles/__init__.py#L17). Every recorded bag's `experiment.json` identifies its profile. Reuse bag loading and shared signal/statistics helpers.

For `px4_compare`, implement `comparison_signals` with a reference source, common quantities, time origin, and compatible saved trajectories. ULog comparison has its own onboard analysis path.

<!-- Preserve existing section links. -->
<span id="add-a-user-facing-tool"></span>
<span id="add-a-ros-2-package"></span>
<span id="add-or-update-an-external-source"></span>

### Add a tool, package, or external source

Place user-facing commands in `tools/` and component helpers beside their implementation. New ROS packages need package/CMake metadata and the appropriate build-profile inclusion. Pin external sources under `config/sources/*.repos`.

<!-- Preserve existing section links. -->
<span id="before-committing-an-extension"></span>
<span id="related"></span>

## Verification before contributing

In a **native development checkout at the repository root**:

```bash
./ros2/build.sh sitl
git diff --check
```

Test changed interfaces and behavior in the applicable runtime. Record the environment and observed results.
