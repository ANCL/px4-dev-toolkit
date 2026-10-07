# Extending the toolkit

[Documentation](../index.md) / Reference / Extending

Use this page when you want to add a new capability without breaking the repository's existing ownership boundaries. It points to the canonical location and the nearest existing example; detailed behavior stays in the subsystem-specific pages.

## Extension map

| Add | Start here | Existing example |
|---|---|---|
| controller or Offboard behavior | `ros2/src/offboard_controllers/` | `offboard_position`, `se3` |
| trajectory | `ros2/src/offboard_controllers/config/trajectory/trajectories.yaml` and trajectory library | `line`, `circle`, `figure_eight` |
| vehicle data | `config/vehicles/<vehicle>.yaml` | `gz_f450.yaml` |
| recording topics | package `config/recording/` | `offboard_controllers/config/recording/se3.txt` |
| experiment sequence | `config/sequences/` | `config/sequences/se3/handoff_comparison.yaml` |
| analysis profile | `analysis/profiles/` | `analysis/profiles/se3/` |
| comparison/analysis feature | `analysis/` | `analysis/profiles/se3/comparison.py` |
| user-facing command | `tools/` | `sitl`, `experiment`, `run_sequence`, `px4_analyze` |
| ROS 2 package | `ros2/src/` | toolkit-maintained packages listed in Architecture |
| external source | `config/sources/*.repos` | common/SITL/experiment manifests |

Before adding a new file, check [Configuration](configuration.md) so the setting or data does not duplicate an existing canonical source.

## Add a controller

Keep controller mathematics separate from ROS/PX4 transport where practical. Reuse the central PX4 topic catalog instead of adding literal `/fmu/in/*` or `/fmu/out/*` names throughout the code.

A typical controller addition is:

1. add the controller library/node under `ros2/src/offboard_controllers/`;
2. add its CMake target and dependencies;
3. provide a launch file with explicit runtime arguments;
4. put controller tuning in package controller configuration, not vehicle files;
5. add unit/runtime tests under the package `test/` tree;
6. add recording and analysis support only for signals the new workflow needs.

Use `offboard_position` as the simpler lifecycle/reference example and `se3` as the full controller/diagnostics example. See [Controller overview](../controllers/overview.md).

## Add a trajectory

Shared trajectory definitions live in:

```text
ros2/src/offboard_controllers/config/trajectory/trajectories.yaml
```

Prefer composing reusable named segments. If a new motion requires a new segment type, extend the trajectory library and its tests as well as the YAML configuration. Keep NED units and boundary semantics consistent with [Trajectories](../controllers/trajectories.md).

After changing trajectory behavior, run the toolkit tests:

```bash
./ros2/test.sh
```

## Add a vehicle

Create:

```text
config/vehicles/<vehicle>.yaml
```

Store vehicle facts there: mass, inertia, center of mass, allocator/rotor geometry, hover thrust, and physical propulsion mapping when available. Do not copy controller gains into the vehicle file.

Only provide properties you can actually source for that vehicle. Controller modes that require missing properties should remain unavailable rather than using invented defaults. See [SE(3) vehicle support](../controllers/se3.md#vehicle-support).

## Add recording topics

Recording selections belong with the package/workflow that owns the recording. PX4 topics should reference symbolic names from:

```text
config/px4_topics.def
```

Toolkit-owned ROS topics may be listed directly when appropriate. Keep the recording set focused on what the corresponding analysis or experiment needs; do not record the entire ROS graph by default.

See [Recording](../workflows/recording.md).

## Add a sequence

Create a YAML definition under `config/sequences/`. A sequence should select existing launch files and launch arguments; it should not duplicate controller, vehicle, or trajectory configuration.

Use:

```text
config/sequences/se3/handoff_comparison.yaml
```

as the reference for ordered steps, recording output ownership, and settle time. Run it with:

```bash
source ros2/runtime_env.sh
./tools/run_sequence <sequence.yaml>
```

See [Sequences](../workflows/sequences.md) for the schema and study layout.

## Add an analysis profile

Experiment-specific analysis lives in `analysis/profiles/`; bag loading and common analysis helpers stay in the shared analysis modules.

A profile should:

1. define a stable `PROFILE_NAME`;
2. declare required and optional topics;
3. implement the experiment-specific analysis and summary;
4. add plots only where they add useful interpretation;
5. register the profile in `analysis/profiles/__init__.py`;
6. add regression tests under `analysis/tests/`.

`tools/px4_analyze` will then resolve the registered profile from `experiment.json` when present, or from the bag directory layout for standalone runs.

Use the SE(3) profile as the most complete example and the Position/Offboard profiles as smaller examples. See [Single-run analysis](../analysis/analysis.md).

## Extend comparison analysis

Shared comparison helpers live in `analysis/comparison.py`; profile-specific comparison semantics belong with the profile. The current `px4_compare` CLI intentionally supports SE(3) only.

If another experiment family needs comparison support, define what quantities and time origin are meaningful for that profile before broadening the CLI. Do not assume all profiles can be compared using the SE(3) metrics. See [SE(3) comparison](../analysis/comparison.md).

## Add a user-facing tool

Put complete user workflows in `tools/`. Internal helpers should stay with the subsystem that owns them instead of becoming top-level commands.

A top-level tool should:

- locate/use repository configuration rather than duplicating it;
- provide a clear usage/error contract;
- own any process tree it starts and shut it down predictably;
- avoid depending on the caller's current directory when it can resolve the repository root itself.

Use `tools/sitl` and `tools/experiment` for runtime/process ownership, `tools/run_sequence` for orchestration, and `tools/px4_analyze` for a small CLI around library code.

## Add a ROS 2 package

Toolkit-owned packages live under `ros2/src/`. Add normal ROS package metadata/build files and keep dependencies scoped to the package that needs them.

`ros2/build.sh` builds the workspace. If the new package is maintained by this toolkit, add it to the `TOOLKIT_PACKAGES` list in `ros2/test.sh` so the standard test command covers it:

```bash
./ros2/build.sh
./ros2/test.sh
```

## Add or update an external source

External repositories are pinned through:

```text
config/sources/common.repos
config/sources/sitl.repos
config/sources/experiment.repos
```

Choose the manifest based on which environment needs the dependency. `setup/fetch_sources.sh` reconstructs these source sets and deliberately avoids resetting existing checkouts.

Do not vendor a fetched third-party repository into toolkit-owned source unless there is a separate reason to change that architecture.

## Before committing an extension

At minimum:

```bash
./ros2/build.sh
./ros2/test.sh
git diff --check
```

Also run the affected user workflow when the change alters runtime behavior. Keep documentation updates beside the subsystem they change rather than creating a second description of the same interface.

## Related

- [Architecture](architecture.md)
- [Configuration](configuration.md)
- [Troubleshooting](troubleshooting.md)
