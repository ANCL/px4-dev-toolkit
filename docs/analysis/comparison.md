<!-- Preserve existing section links. -->
<span id="se3-comparison"></span>

<span id="comparing-se3-controllers"></span>
# Controller comparison

`px4_compare` compares two or more flights using equivalent physical quantities. Each analysis profile supplies the reference, PX4 measurements and confirmed flight interval; different controllers or gains do not prevent comparison. The tool requires the same recorded trajectory configuration and reports unavailable signals rather than inventing measurements.

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

If bags are direct children of one study `bags/` directory with a `manifest.json`, output defaults to `<study>/comparison/`. For separately recorded bags, define an array containing their actual paths and provide `--output`; do not assume directory naming supplies experiment identity. The tool writes `summary.txt` and overlay PNGs. It does not reset or execute controllers.

<!-- Preserve existing section links. -->
<span id="labels"></span>

## Offboard alignment

Each run is analyzed independently. The comparison shifts time so **$t=0$ is recorded PX4-confirmed Offboard entry**, and clips to its confirmed Position return. It uses rosbag receive time, not controller process start, recorder start, command acknowledgement, or gyro `timestamp_sample`.

Each series keeps its own time samples. Measured/reference errors use interpolation over their overlapping interval. Missing or differing run durations must be considered when reading overlays. Mode labels come from recorded heartbeat and metadata; duplicate labels are disambiguated by bag identity rather than silently merged.

## Common quantities and limits

The common view compares measured position/velocity tracking, yaw error, body-rate magnitude, normalized thrust/torque, and the maximum absolute motor command envelope. Different command levels do not expose identical internal signals: for acceleration mode PX4 creates attitude/rate commands, while direct modes create torque externally.

Normalized torque is not N·m. Hover calibration, actuator mapping, frames, PX4 configuration, and saturation affect the relationship between requests and motion. Lower tracking error accompanied by much greater actuator activity can be a relevant tradeoff; without measured data there is no controller ranking.


<!-- Preserve existing section links. -->
<span id="comparison-implementation"></span>

## Metric definitions

[`timed_stats()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/analysis/comparison.py#L246), [`activity_stats()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/analysis/comparison.py#L336), and [`effort_stats()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/analysis/comparison.py#L408) compute the shared time-weighted and actuator-activity statistics. For scalar magnitude/error samples $q_k$ and positive finite intervals $\Delta t_k$:

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

The selected trajectory in each bag's `experiment.json` determines maneuver boundaries. Comparison rejects mismatched recorded trajectories. See [`step_response_metrics()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/analysis/profiles/se3/comparison/metrics.py#L280), [`yaw_segment_metrics()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/analysis/profiles/se3/comparison/metrics.py#L533), and [`figure_eight_metrics()`](https://github.com/ANCL/px4-dev-toolkit/blob/main/analysis/profiles/se3/comparison/metrics.py#L585):

| Maneuver | Implemented interpretation |
|---|---|
| Single-axis step | Sampled 10–90% rise time, overshoot, settling, integral absolute error, final residual RMS, uncommanded-axis RMS |
| Yaw | Wrapped yaw RMS/peak and final residual RMS in degrees |
| Figure eight | Time-weighted position/velocity error, measured body-rate activity, normalized wrench, and motor effort/roughness |

Step response normalizes measured motion by the signed configured amplitude. The target is the first reference **strictly after** the boundary. The window includes one immediately following hold when present. Settling requires all remaining sampled errors within the larger of **0.02 m or 5% of step amplitude**. Residual uses the final 0.5 s; integral absolute error uses trapezoidal integration in m·s. Multi-axis steps are excluded from single-axis step metrics.

Yaw/figure-eight windows use start-inclusive/end-exclusive intervals. Yaw errors wrap to $[-180,180)$ degrees. Threshold crossings are observed sample times, not interpolated synthetic crossing times. Missing crossings or coverage can yield unavailable/NaN values.

<!-- Preserve existing section links. -->
<span id="next"></span>

## Before drawing a conclusion

Confirm all expected study steps succeeded, data is complete, each run used the same canonical trajectory/vehicle, and the Offboard window covers the intended segments. Inspect state quality and allocation/motor limits alongside errors. Account for drift/re-anchoring between steps, transport/logging rate differences, and real-world environmental variation. A source-compatible launch and a finalized bag do not by themselves establish a controlled scientific comparison.

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
For legacy native SE3 ROS bags, trajectory execution is *not* directly
recorded; its start is estimated by fitting the full position/yaw
trajectory reference against a confirmed reference run. Ambiguous
or incompatible fits are rejected, not plotted. The saved summary
labels observed versus inferred timestamps explicitly.

Identical PX4 topic names do not guarantee populated streams: each
controller may bypass layers. Missing references remain unavailable.
The comparison requires at least one recorded trajectory configuration
and one mode-confirmed reference run for legacy inference.


## Comparison signal provenance

`px4_compare` uses `comparison_signals(experiment)` from the existing
`analysis.profiles` modules. This is a read-only mapping from controller
configuration to the *physical meaning* and source topic of each signal.
The comparison engine is shared and never publishes flight commands.

For an SE3 attitude-rate handoff, the desired attitude is
`/se3/diagnostics/desired_attitude` (when diagnostics were recorded),
not the inactive PX4 attitude-controller output. For the PX4-rate
reproduction the desired rate is the SE3 diagnostic body-rate setpoint.
Direct geometric torque handoffs have no equivalent commanded body-rate
reference. Missing observations remain unavailable, without fabricated
values or fallback to a different stage of the control cascade.

The summary reports the selected source names and alignment evidence.
