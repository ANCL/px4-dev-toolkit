<!-- Preserve existing section links. -->
<span id="experiment-sequences"></span>

<span id="running-a-series-of-experiments"></span>
# Sequences

`tools/run_sequence` runs ordered ROS launches from YAML. It preserves each launch's output, gives recorded steps deterministic bag paths, and records the study result in a manifest. Environment startup and preparing the aircraft remain separate.

## Run a sequence

Prepare a matching airborne F450 SITL vehicle in Position mode, valid state/DDS, adequate recording space, and no competing controller. The SE(3) launches do not arm or reset the aircraft. Check that the whole study path is appropriate for the simulation before beginning.

In a **separate repository-root shell**:

```bash
source ros2/runtime_env.sh
./tools/run_sequence config/sequences/se3/handoff_comparison.yaml
```

The runner accepts one positional YAML path. First prepare and arm the SITL aircraft using the Offboard staging helper; the runner itself does not arm or take off. It validates the sequence and prints the actual study directory before launching steps. It stops on a failed step or a successful recorded step whose expected bag lacks `metadata.yaml`.

## Sequence schema

The [checked-in study](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/sequences/se3/handoff_comparison.yaml) defines six recorded steps: Offboard Position and five SE(3) controller configurations. The study-level `profile: se3` remains the default, but the first step specifies `profile: offboard_position`. This **fragment** shows its first step:

```{ .yaml .no-copy }
name: se3_handoff_comparison
profile: se3
recording_output_argument: recording_output
settle_seconds: 3.0
steps:
  - name: position
    profile: offboard_position
    package: offboard_controllers
    launch: offboard_position.launch.py
    arguments:
      trajectory: full_excitation
      record: true
```

Names permit letters, numbers, `.`, `_`, and `-`. Argument values must be strings, numbers, or booleans. The runner overrides the named recording argument for recorded steps; it does not change vehicle/gain/trajectory definitions. [`load_sequence()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/tools/sequence/config.py#L43) validates the whole definition before creating the study directory. [`run_step()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/tools/sequence/execution.py#L50) starts and supervises each launch.

## Study layout

Example directory structure, with timestamp chosen at runtime:

```{ .text .no-copy }
bags/se3_handoff_comparison/<timestamp>/
├── manifest.json
├── logs/
│   ├── 01-position.log
│   └── ...
└── bags/
    ├── 01-position/
    │   ├── metadata.yaml
    │   ├── experiment.json
    │   └── <storage files>
    └── ...
```

[`write_manifest()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/tools/sequence/manifest.py#L9) records progress, commands, times, exits, logs, and finalized bags. [`write_experiment_metadata()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/tools/sequence/manifest.py#L21) adds study and step identity without replacing the recorder's configuration. The shared recorder writes the experiment configuration on finalization; the sequence runner adds study identity, including for a finalized failed step; finalization and successful flight are different facts.

<!-- Preserve existing section links. -->
<span id="process-ownership"></span>
<span id="settle-time"></span>
<span id="handoff-comparison-study"></span>

## Ordering, settling, and interruption

The checked-in study executes **Offboard Position first**, then SE(3) acceleration, attitude, body rate, geometric normalized torque, and PX4 rate PID, all using `full_excitation`. The five SE(3) steps select `gz_f450` with diagnostics enabled. Each step returns to native PX4 Position mode before ending, and the runner waits **3 seconds between successful steps**. The manifest records each step's analysis profile.

That delay is not a vehicle reset, landing, arming action, or a flight-readiness check. Position can drift between steps, and each controller restarts its fixed absolute NED reference. Confirm the required initial condition and watch the study; do not treat it as a guarantee of independent identical starting states.

Each launch has a process group. Ctrl-C stops the launch tree and preserves per-step logs and the manifest. It can interrupt a live controller stream: recover the aircraft through prepared flight controls and verify mode/arming state before stopping services. A completed controller is not proof the recorder exited cleanly.

<!-- Preserve existing section links. -->
<span id="next"></span>

## Compare the completed study

Analyze each bag with `px4_analyze`, then compare the compatible recorded Offboard flights together using `px4_compare`. Use the **actual directory printed by the runner**, check all expected steps and metadata, and follow [Comparing SE(3) controllers](../analysis/comparison.md). Output defaults to the study's `comparison/` directory when its finalized bags share the expected parent and manifest. The [complete example](../experiments/complete-example.md) shows how to find the completed study directory.
