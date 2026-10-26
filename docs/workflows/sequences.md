<!-- Preserve existing section links. -->
<span id="experiment-sequences"></span>

<span id="running-a-series-of-experiments"></span>
<span id="sequences"></span>

<span id="running-experiment-sequences"></span>

# Sequences

`tools/run_sequence` executes ordered ROS launches from YAML, storing step logs, bags, and a study manifest.

## Run a sequence

Prepare an airborne `gz_f450` SITL vehicle in Position mode, with valid DDS state, sufficient recording space, and no competing controller.

In a **separate repository-root shell**:

```bash
source ros2/runtime_env.sh
./tools/run_sequence config/sequences/se3/handoff_comparison.yaml
```

The runner validates the YAML, prints the study directory, then launches steps. It stops on a failed step or missing finalized bag metadata. Takeoff and staging must happen first.

## Sequence schema

The [checked-in study](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/config/sequences/se3/handoff_comparison.yaml) records Offboard Position plus five Offboard SE3 configurations, excluding `geometric_physical`. The study defaults to profile `se3`; the first step overrides it with `offboard_position`:

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

Names permit letters, numbers, `.`, `_`, and `-`. Argument values must be strings, numbers, or booleans. The runner overrides the named recording argument for recorded steps; it does not change vehicle/gain/trajectory definitions. [`load_sequence()`](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/tools/sequence/config.py) validates the whole definition before creating the study directory. [`run_step()`](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/tools/sequence/execution.py) starts and supervises each launch.

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

[`manifest.py`](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/tools/sequence/manifest.py) records commands, timestamps, exits, logs, and bag paths. Each finalized bag's `experiment.json` includes its study/step identity.

<!-- Preserve existing section links. -->
<span id="process-ownership"></span>
<span id="settle-time"></span>
<span id="handoff-comparison-study"></span>

## Ordering, settling, and interruption

The checked-in study executes **Offboard Position first**, then Offboard SE3 acceleration, attitude, body rate, geometric normalized torque, and PX4 rate PID, all using `full_excitation`. The five Offboard SE3 steps select `gz_f450` with diagnostics enabled. Each step returns to native PX4 Position mode before ending, and the runner waits **3 seconds between successful steps**. The manifest records each step's analysis profile.

The 3-second settle does not reset the aircraft: position can drift, while every controller restarts its absolute NED reference. Monitor the starting state between steps.

Ctrl-C stops the launch process tree while preserving logs and the manifest. If interrupted during flight, recover through pilot control and confirm PX4 mode/arming before shutdown.

<!-- Preserve existing section links. -->
<span id="next"></span>

## Compare the completed study

Analyze each bag with `px4_analyze`, then compare the compatible recorded Offboard flights together using `px4_compare`. Use the **actual directory printed by the runner**, check all expected steps and metadata, and follow [Controller comparison](../analysis/comparison.md). Output defaults to the study's `comparison/` directory when its finalized bags share the expected parent and manifest. The [complete example](../experiments/complete-example.md) shows how to find the completed study directory.
