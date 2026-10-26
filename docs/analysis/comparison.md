<!-- Preserve existing section links. -->
<span id="se3-comparison"></span>

<span id="comparing-se3-controllers"></span>
# Controller comparison

`px4_compare` compares finalized ROS bags using each profile's reference and measured PX4 signals. Runs must have compatible saved trajectories. For ULog comparison and plan CRC matching, see [Analysis data](../workflows/common-analysis.md).

<!-- Preserve existing section links. -->
<span id="basic-usage"></span>
<span id="output-directory"></span>
<span id="handoff-study-example"></span>
<span id="outputs"></span>

## Compare a completed study

Use the actual finalized six-step study from the [complete example](../experiments/complete-example.md#6-compare-finalized-study-bags). In its **repository-root shell**, with ROS sourced and `study_bags` already checked:

```bash
./tools/px4_compare "${study_bags[@]}"
```

For bags from one manifested study, output defaults to `<study>/comparison/`. Standalone bags require `--output`. The result contains `summary.txt` and overlay PNGs. Example with standalone bag paths:

```bash
./tools/px4_compare bags/se3/<run-a> bags/se3/<run-b> --output bags/comparison/example
```

<!-- Preserve existing section links. -->
<span id="labels"></span>

## Offboard alignment

Runs align at **$t=0$ = PX4-confirmed Offboard entry**, using rosbag receive timestamps. The selected-mode interval ends at confirmed Position return.

Each series retains its sample times; errors use interpolation over the overlap. Labels and reference sources come from the analysis profile and saved metadata.

## Common quantities and limits

The common view compares measured position/velocity tracking, yaw error, body-rate magnitude, normalized thrust/torque, and the maximum absolute motor command envelope. Different command levels do not expose identical internal signals: for acceleration mode PX4 creates attitude/rate commands, while direct modes create torque externally.

Normalized torque is not N·m. Hover calibration, actuator mapping, frames, PX4 configuration, and saturation affect the relationship between requests and motion. Lower tracking error accompanied by much greater actuator activity can be a relevant tradeoff; without measured data there is no controller ranking.


<!-- Preserve existing section links. -->
<span id="comparison-implementation"></span>

## Metric definitions

[`timed_stats()`](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/analysis/comparison.py), [`activity_stats()`](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/analysis/comparison.py), and [`effort_stats()`](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/analysis/comparison.py) compute the shared time-weighted and actuator-activity statistics. For scalar magnitude/error samples $q_k$ and positive finite intervals $\Delta t_k$:

$$
q_{\mathrm{RMS}}=\sqrt{
\frac{\sum_k\tfrac12(q_k^2+q_{k+1}^2)\Delta t_k}
{\sum_k\Delta t_k}}.
$$

Position/velocity metrics use Euclidean error norms; wrench and rate metrics use signal magnitudes. Peak is maximum absolute value. Fewer than two samples gives zero covered duration and a single absolute-value RMS; when no valid interval exists, the code falls back to sample RMS. Treat these edge cases as insufficient coverage, not a robust performance estimate.

Roughness uses time-weighted RMS of finite-difference signal slopes:

$$
q_{\mathrm{rough}}=\sqrt{
\frac{\sum_k\left(\frac{q_{k+1}-q_k}{\Delta t_k}\right)^2\Delta t_k}
{\sum_k\Delta t_k}}.
$$

Motor metrics use the maximum absolute motor envelope, not measured electrical power. Near-limit duration/fraction integrate the binary condition at or above **0.99 of nominal limit 1.0**. This reveals requested actuator activity; it is not battery energy consumption or a motor thrust measurement.

## Trajectory-aware metrics

The selected trajectory in each bag's `experiment.json` determines maneuver boundaries. Comparison rejects mismatched recorded trajectories. See [`step_response_metrics()`](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/analysis/profiles/se3/comparison/metrics.py), [`yaw_segment_metrics()`](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/analysis/profiles/se3/comparison/metrics.py), and [`figure_eight_metrics()`](https://github.com/ANCL/px4-dev-toolkit/blob/0b668261e04732ef97e2965dbe8f5c1ba56730f0/analysis/profiles/se3/comparison/metrics.py):

<div class="px4-table-scroll" markdown="1">

| Maneuver | Implemented interpretation |
|---|---|
| Single-axis step | Sampled 10–90% rise time, overshoot, settling, integral absolute error, final residual RMS, uncommanded-axis RMS |
| Yaw | Wrapped yaw RMS/peak and final residual RMS in degrees |
| Figure eight | Time-weighted position/velocity error, measured body-rate activity, normalized wrench, and motor effort/roughness |

</div>

Step response normalizes measured motion by the signed configured amplitude. The target is the first reference **strictly after** the boundary. The window includes one immediately following hold when present. Settling requires all remaining sampled errors within the larger of **0.02 m or 5% of step amplitude**. Residual uses the final 0.5 s; integral absolute error uses trapezoidal integration in m·s. Multi-axis steps are excluded from single-axis step metrics.

Yaw/figure-eight windows use start-inclusive/end-exclusive intervals. Yaw errors wrap to $[-180,180)$ degrees. Threshold crossings are observed sample times, not interpolated synthetic crossing times. Missing crossings or coverage can yield unavailable/NaN values.

<!-- Preserve existing section links. -->
<span id="next"></span>

## Before drawing a conclusion

Confirm all expected study steps succeeded, data is complete, each run used the same canonical trajectory/vehicle, and the Offboard window covers the intended segments. Inspect state quality and allocation/motor limits alongside errors. Account for drift and estimator-frame resets between steps, transport/logging rate differences, and real-world environmental variation. A source-compatible launch and a finalized bag do not by themselves establish a controlled scientific comparison.

To add a controller or a metric, use [Extending the Toolkit](../reference/extending.md).

## Tracking and error figures

The comparison generates paired figures for compatible position, velocity,
attitude, body-rate and trajectory-yaw observations. Tracking figures overlay
each run's measured and recorded reference components; error figures overlay
`actual - reference` on the same three component axes (yaw error wrapped to
`[-180, 180)` degrees). There is **no fourth vector-error-norm panel**.
The existing time-weighted vector-norm statistics remain in `summary.txt`.

The plots use the common 300-DPI analysis style. Missing setpoints result in
actual-only plots, **not** invented error curves. Existing thrust/torque,
body-rate magnitude and motor envelope overlays remain separate.

## Trajectory-time alignment

Comparison time zero is the trajectory start. For these Offboard controllers,
the controller starts the trajectory when PX4 confirms Offboard entry.
For native SE3 ROS bags, trajectory execution is *not* directly
recorded by the standard DDS recording contract; its start is estimated by fitting the full position/yaw
trajectory reference against a confirmed reference run. Ambiguous
or incompatible fits are rejected, not plotted. The saved summary
labels observed versus inferred timestamps explicitly.

Identical PX4 topic names do not guarantee populated streams: each
controller may bypass layers. Missing references remain unavailable.
The comparison requires at least one recorded trajectory configuration
and one mode-confirmed Offboard reference run for that inference. Current
standalone `px4_record onboard_se3` metadata contains a topic list, not a
trajectory snapshot: pair it with a compatible recorded Offboard reference
only when the reference fit and coverage checks succeed. An ambiguous hover or
shortened execution can be rejected. For exact native run timing, use ULog.


## Comparison signal provenance

`px4_compare` uses `comparison_signals(experiment)` from the existing
`analysis.profiles` modules. This is a read-only mapping from controller
configuration to the *physical meaning* and source topic of each signal.
The comparison engine is shared and never publishes flight commands.

For an Offboard SE3 attitude-rate handoff, the desired attitude is
`/se3/diagnostics/desired_attitude` (when diagnostics were recorded),
not the inactive PX4 attitude-controller output. For the PX4-rate
reproduction the desired rate is the Offboard SE3 diagnostic body-rate setpoint.
Direct geometric torque handoffs have no equivalent commanded body-rate
reference. Missing observations remain unavailable, without fabricated
values or fallback to a different stage of the control cascade.

The summary reports the selected source names and alignment evidence.
