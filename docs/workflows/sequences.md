# Experiment sequences

[Documentation](../index.md) / Workflows / Sequences

`tools/run_sequence` executes an ordered YAML study, preserves per-step logs, owns deterministic bag locations, and writes a manifest describing the complete run.

## Run a sequence

Source the ROS runtime first:

```bash
source ros2/runtime_env.sh
```

Then:

```bash
./tools/run_sequence config/sequences/se3/handoff_comparison.yaml
```

The runner accepts one positional argument: the YAML sequence configuration.

## Sequence schema

A sequence defines:

```yaml
name: study_name
profile: se3
recording_output_argument: recording_output
settle_seconds: 3.0

steps:
  - name: one_step
    package: offboard_controllers
    launch: se3.launch.py
    arguments:
      vehicle: gz_f450
      trajectory: full_excitation
      handoff: acceleration
      publish_diagnostics: true
      record: true
```

Names are restricted to letters, numbers, `.`, `_`, and `-`. Launch argument values must be strings, numbers, or booleans. The runner validates the complete sequence before creating the study directory or launching a process.

## Study layout

A run creates:

```text
bags/<sequence-name>/<timestamp>/
├── manifest.json
├── logs/
│   ├── 01-<step>.log
│   └── ...
└── bags/
    ├── 01-<step>/
    └── ...
```

For recorded steps, the runner overrides the launch file's `recording_output` argument so each bag has a deterministic path inside the study.

After a bag is finalized, `experiment.json` is written inside it with the profile, sequence, step, and effective launch arguments.

## Process ownership

Every `ros2 launch` step runs in its own process group. On interruption, the runner stops the complete launch tree rather than only the parent process.

Combined stdout/stderr is streamed to the terminal and to the step log.

## Settle time

`settle_seconds` is applied between successful steps. It is intended for the environment/vehicle to settle before the next experiment, not as part of a controller's trajectory timing.

## Handoff comparison study

The checked-in SE(3) study:

```text
config/sequences/se3/handoff_comparison.yaml
```

runs the same `full_excitation` trajectory through:

```text
acceleration
attitude
attitude_rate
thrust_and_torque + geometric_normalized
thrust_and_torque + px4_rate
thrust_and_torque + geometric_physical
```

using `gz_f450`, diagnostics, and recording for every step.

## Compare the completed study

The bags are directly compatible with `px4_compare`. If all bags come from the same study `bags/` directory, comparison output defaults to:

```text
<study>/comparison/
```

See [Comparison](../analysis/comparison.md).

## Next

- [Recording](recording.md)
- [Comparison](../analysis/comparison.md)
- [Extending sequences](../reference/extending.md#add-a-sequence)
