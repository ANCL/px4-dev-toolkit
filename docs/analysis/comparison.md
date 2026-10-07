# SE(3) comparison

[Documentation](../index.md) / Analysis / Comparison

`tools/px4_compare` compares two or more recorded runs from one analysis profile. The current CLI intentionally supports the `se3` profile only because its controller-comparison semantics are explicitly defined.

## Basic usage

```bash
./tools/px4_compare BAG_A BAG_B [BAG_C ...]
```

At least two bags are required.

## Output directory

If all bags are direct children of one sequence study's `bags/` directory and that study contains `manifest.json`, output defaults to:

```text
<study>/comparison/
```

For standalone bags or bags from different locations, provide an explicit output directory:

```bash
./tools/px4_compare \
  /path/to/bag-a \
  /path/to/bag-b \
  --output /path/to/comparison
```

## Offboard alignment

Each bag is analyzed independently, then comparison runs are aligned to **confirmed PX4 Offboard entry**. This avoids treating process start, recorder start, or mode-request time as the common physical origin.

That alignment is especially important because prestream and handoff timing can differ slightly between runs.

## Labels

When experiment metadata is available, comparison uses the configured handoff/direct-controller identity. The standard labels correspond to:

```text
Acceleration
Attitude
Rate
Geometric
PX4 rate
Geometric physical
```

Duplicate labels are disambiguated using bag information rather than silently merging runs.

## Trajectory-aware metrics

Add the configured trajectory name to resolve segment boundaries from the canonical trajectory YAML:

```bash
./tools/px4_compare \
  BAG_A BAG_B BAG_C \
  --trajectory full_excitation
```

This enables segment-aware metrics in addition to the common whole-run physical comparison.

## Handoff study example

After:

```bash
source ros2/runtime_env.sh
./tools/run_sequence config/sequences/se3/handoff_comparison.yaml
```

compare the finalized bags from that study, for example:

```bash
./tools/px4_compare \
  bags/se3_handoff_comparison/<timestamp>/bags/* \
  --trajectory full_excitation
```

The shell expansion should resolve only finalized bag directories from the study.

## Outputs

Comparison writes:

```text
comparison/
├── summary.txt
└── *.png
```

The plot set focuses on common physical quantities that remain meaningful across different handoff boundaries, with segment context when requested.

## Next

- [SE(3)](../controllers/se3.md)
- [Sequences](../workflows/sequences.md)
- [Single-run analysis](analysis.md)
