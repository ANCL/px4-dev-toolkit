<!-- Preserve existing section links. -->
<span id="se3-comparison"></span>

# Comparing SE(3) controllers

A useful comparison holds the vehicle, trajectory, tuning context, and test conditions consistent, then examines both tracking and actuation. The current `px4_compare` CLI supports **two or more SE(3) bags**; other experiment profiles are not interchangeable with these metrics.

<!-- Preserve existing section links. -->
<span id="basic-usage"></span>
<span id="output-directory"></span>
<span id="handoff-study-example"></span>
<span id="outputs"></span>

## Compare a completed study

Use the actual finalized six-mode study from the [complete example](../experiments/complete-example.md#6-compare-finalized-study-bags). In its **repository-root shell**, with ROS sourced and `study_bags` already checked:

```bash
./tools/px4_compare "${study_bags[@]}" --trajectory full_excitation
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

<figure class="px4-figure-slot">
  <img src="../../assets/measured-plot-placeholder.svg" width="760" height="190" alt="Empty axes reserved for controller comparison; measured data has not been supplied">
  <figcaption><strong>Controller comparison · figure placeholder.</strong> NED position-error magnitude [m] and motor envelope [normalized], aligned to confirmed Offboard entry [s], with maneuver windows labeled. An approved study is needed before this figure can compare controller responses.</figcaption>
</figure>

<!-- Preserve existing section links. -->
<span id="comparison-implementation"></span>

## Metric definitions

[analysis/comparison.py](https://github.com/ANCL/px4-dev-toolkit/blob/main/analysis/comparison.py) defines the shared statistics. For scalar magnitude/error samples $q_k$ and positive finite intervals $\Delta t_k$:

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

`--trajectory full_excitation` resolves the canonical [trajectory YAML](../controllers/trajectories.md) into cumulative segment times. It must match the configuration used for recording. See [comparison/metrics.py](https://github.com/ANCL/px4-dev-toolkit/blob/main/analysis/profiles/se3/comparison/metrics.py):

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

Continue with [Adding to the toolkit](../reference/extending.md) when you need another controller or analysis view.
