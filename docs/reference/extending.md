<!-- Preserve existing section links. -->
<span id="extending-the-toolkit"></span>
<span id="extension-map"></span>

<span id="adding-to-the-toolkit"></span>
# Extending the toolkit

Start with the closest working example. Keep controller logic, vehicle data, trajectories, runtime services, and analysis in their owning components; use the [configuration map](configuration.md) before adding new settings.

## Add a controller

Follow [Developing a controller](developing-a-controller.md) for package/node integration, pure calculations, mode transitions, topics, recording, and verification. Offboard position is a small reference; SE(3) shows several command levels and diagnostics. A new controller need not use the same gains, timing, or F450 fixture.

## Add a vehicle

[Adding a vehicle](adding-a-vehicle.md) explains schema, physical measurements, frame conversion, and mode requirements. Record only verified facts. A valid YAML is not a flight-validation result, and additional rotors can require an adapter change rather than more configuration.

## Add a trajectory

Prefer composing existing named segments in [trajectories.yaml](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/config/trajectory/trajectories.yaml). A new segment type also needs sampler/parser support and tests for derivatives, units, endpoints, and boundary behavior. Keep the [documented NED and time conventions](../controllers/trajectories.md).

## Add recording topics

Use the package's recording list and symbolic [PX4 topic catalog](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/px4_topics.def). Toolkit diagnostic topics may be direct absolute names where the recording helper permits them. Record the signals the experiment needs, and explain which are required versus optional in analysis.

## Add a sequence

Create a YAML under `config/sequences/` selecting existing launches and arguments. Use the [six-mode study](../workflows/sequences.md) for recording paths, manifests, and settle semantics. Do not duplicate controller gains or vehicle physics in the sequence.

<!-- Preserve existing section links. -->
<span id="extend-comparison-analysis"></span>

## Add an analysis profile

Profiles live in [analysis/profiles](https://github.com/ANCL/px4-dev-toolkit/tree/main/analysis/profiles). Define `PROFILE_NAME`, required/optional topics, `analyze`, and reporting; register it in [`PROFILES`](https://github.com/ANCL/px4-dev-toolkit/blob/main/analysis/profiles/__init__.py#L14). [`detect_profile_name()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/analysis/profiles/__init__.py#L38) resolves study metadata or the standalone bag layout. Reuse bag loading and shared signal/statistics helpers. Include meaningful analysis regression tests under `analysis/tests/`.

Comparison support requires an explicit definition of comparable quantities and time origin. `px4_compare` currently accepts SE(3) only; registering a new single-run profile does not automatically create valid comparison semantics.

<!-- Preserve existing section links. -->
<span id="add-a-user-facing-tool"></span>
<span id="add-a-ros-2-package"></span>
<span id="add-or-update-an-external-source"></span>

## Add a tool, package, or external source

Put user-facing commands in `tools/` and internal helpers with their component. Resolve repository configuration without duplicating it, validate arguments, and stop any process trees the command creates. For a new ROS package, provide normal package/CMake metadata and choose its dependencies; update the maintained-package test selection only when the package needs inclusion.

Pin external repositories in the appropriate `config/sources/*.repos` scope. Ordinary controller development does not require changing pinned upstream PX4.

<!-- Preserve existing section links. -->
<span id="before-committing-an-extension"></span>
<span id="related"></span>

## Verification before contributing

In a **native development checkout at the repository root**:

```bash
./ros2/build.sh sitl
./ros2/test.sh sitl
git diff --check
```

Run affected harmless/source-level checks and, when behavior changes, validate the relevant simulation in a properly prepared environment. Write tests for the new behavior rather than changing unrelated tests to make it fit an old fixture. Document the exercised environment, expected result, and remaining real-flight validation.
