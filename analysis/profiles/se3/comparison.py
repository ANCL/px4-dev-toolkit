"""Multi-run comparison for SE3 trajectory-tracking experiments."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from analysis.comparison import (
    activity_stats,
    effort_stats,
    interval_scalar,
    interval_series,
    motor_envelope,
    shift_motor_series,
    shift_vector_series,
    timed_stats,
    tracking_error_series,
    vector_magnitude_series,
)
from analysis.plots import (
    _decorate_axis,
    _matplotlib,
)


@dataclass
class ComparisonRun:
    """Common physical comparison view of one analyzed SE3 run."""

    label: str
    bag_path: Path
    handoff: str
    direct_controller: str
    duration_s: float

    position_actual: dict[str, list[float]]
    position_reference: dict[str, list[float]]
    position_error: dict[str, list[float]]
    velocity_error: dict[str, list[float]]
    yaw_error: dict[str, list[float]]

    thrust: dict[str, list[float]]
    torque: dict[str, list[float]]
    body_rates: dict[str, list[float]]

    motors: dict[
        str,
        tuple[list[float], list[float]],
    ]

def build_run(
    result,
    bag_path: Path,
    label: str,
) -> ComparisonRun:
    """Build one common physical view from analyzed SE3 data."""
    layers = result.plot_data["layers"]
    pipeline = result.plot_data["pipeline"]

    origin = float(
        result.metrics["offboard_entry_s"]
    )

    duration = float(
        result.metrics["offboard_duration_s"]
    )

    end = (
        origin
        + duration
    )

    return ComparisonRun(
        label=label,
        bag_path=bag_path,
        handoff=str(
            result.metrics["handoff"]
        ),
        direct_controller=str(
            result.metrics[
                "direct_controller"
            ]
        ),
        duration_s=duration,
        position_actual=shift_vector_series(
            pipeline["position"]["actual"],
            origin,
            duration,
        ),
        position_reference=shift_vector_series(
            layers["reference"]["position"],
            origin,
            duration,
        ),
        position_error=tracking_error_series(
            pipeline["position"]["actual"],
            layers["reference"]["position"],
            time_origin_s=origin,
            end_time_s=end,
        ),
        velocity_error=tracking_error_series(
            pipeline["velocity"]["actual"],
            layers["reference"]["velocity"],
            time_origin_s=origin,
            end_time_s=end,
        ),
        yaw_error=tracking_error_series(
            pipeline["attitude"]["actual"],
            layers["reference"]["yaw"],
            time_origin_s=origin,
            end_time_s=end,
            components=("z",),
            wrap_degrees=True,
        ),
        thrust=shift_vector_series(
            pipeline["thrust"]["setpoint"],
            origin,
            duration,
        ),
        torque=shift_vector_series(
            pipeline["torque"]["setpoint"],
            origin,
            duration,
        ),
        body_rates=shift_vector_series(
            pipeline["rates"]["actual"],
            origin,
            duration,
        ),
        motors=shift_motor_series(
            pipeline["motors"],
            origin,
            duration,
        ),
    )

def trajectory_segments(
    config_path: Path,
    trajectory_name: str,
) -> list[dict[str, float | str]]:
    """Resolve configured segment boundaries relative to Offboard entry."""
    data = yaml.safe_load(
        config_path.read_text()
    )
    segments = data.get("segments", {})
    trajectories = data.get(
        "trajectories",
        {},
    )

    if trajectory_name not in trajectories:
        raise ValueError(
            f"Unknown trajectory: {trajectory_name}"
        )

    elapsed = 0.0
    result = []

    for index, segment_name in enumerate(
        trajectories[trajectory_name]["segments"],
        start=1,
    ):
        if segment_name not in segments:
            raise ValueError(
                f"Trajectory {trajectory_name} references "
                f"unknown segment {segment_name}"
            )

        segment = segments[segment_name]
        duration = float(
            segment["duration"]
        )

        item: dict[str, object] = {
            "index": index,
            "name": segment_name,
            "type": str(
                segment["type"]
            ),
            "start_s": elapsed,
            "end_s": elapsed + duration,
        }

        if "offset" in segment:
            item["offset"] = [
                float(value)
                for value in segment["offset"]
            ]

        if "yaw_offset" in segment:
            item["yaw_offset"] = float(
                segment["yaw_offset"]
            )

        result.append(item)
        elapsed += duration

    return result


_COMPONENTS = (
    "x",
    "y",
    "z",
)

_AXIS_NAMES = (
    "North",
    "East",
    "Down",
)


def _ordered_component(
    series: dict[str, list[float]],
    component: str,
) -> tuple[np.ndarray, np.ndarray]:
    times = np.asarray(
        series["times_s"],
        dtype=float,
    )

    values = np.asarray(
        series[component],
        dtype=float,
    )

    if times.size == 0:
        return times, values

    order = np.argsort(times)

    times = times[order]
    values = values[order]

    times, unique = np.unique(
        times,
        return_index=True,
    )

    return (
        times,
        values[unique],
    )


def _interp_component(
    series: dict[str, list[float]],
    component: str,
    times: np.ndarray,
) -> np.ndarray:
    source_times, source_values = (
        _ordered_component(
            series,
            component,
        )
    )

    if source_times.size == 0:
        return np.full(
            times.shape,
            np.nan,
        )

    return np.interp(
        times,
        source_times,
        source_values,
    )


def _first_crossing(
    times: np.ndarray,
    values: np.ndarray,
    threshold: float,
    *,
    not_before: float | None = None,
) -> float | None:
    mask = (
        values
        >= threshold
    )

    if not_before is not None:
        mask &= (
            times
            >= not_before
        )

    indices = np.flatnonzero(
        mask
    )

    if indices.size == 0:
        return None

    return float(
        times[
            indices[0]
        ]
    )


def _settling_time(
    times: np.ndarray,
    error: np.ndarray,
    *,
    start_s: float,
    tolerance: float,
) -> float:
    """Return settling time over the configured response window."""
    for index in range(len(times)):
        if (
            times[index] < start_s
            or abs(error[index]) > tolerance
        ):
            continue

        if np.all(
            np.abs(
                error[index:]
            )
            <= tolerance
        ):
            return float(
                times[index]
                - start_s
            )

    return float("nan")


def _integral_absolute_error(
    times: np.ndarray,
    error: np.ndarray,
) -> float:
    if times.size < 2:
        return float("nan")

    dt = np.diff(times)

    valid = (
        np.isfinite(dt)
        & (dt > 0.0)
    )

    if not np.any(valid):
        return float("nan")

    return float(
        np.sum(
            0.5
            * (
                np.abs(
                    error[:-1][valid]
                )
                + np.abs(
                    error[1:][valid]
                )
            )
            * dt[valid]
        )
    )


def _step_axis(
    segment: dict[str, object],
) -> tuple[int, float] | None:
    offset = segment.get(
        "offset"
    )

    if not isinstance(
        offset,
        list,
    ):
        return None

    active = [
        index
        for index, value
        in enumerate(offset)
        if abs(
            float(value)
        ) > 1.0e-12
    ]

    if len(active) != 1:
        return None

    axis = active[0]

    return (
        axis,
        float(
            offset[axis]
        ),
    )


def _step_response_end(
    segments: list[
        dict[str, object]
    ],
    index: int,
) -> float:
    """Include the hold immediately following a configured step."""
    segment = segments[index]

    end = float(
        segment["end_s"]
    )

    next_index = (
        index + 1
    )

    if (
        next_index < len(segments)
        and segments[
            next_index
        ]["type"] == "hold"
    ):
        end = float(
            segments[
                next_index
            ]["end_s"]
        )

    return end


def step_response_metrics(
    run: ComparisonRun,
    segment: dict[str, object],
    response_end_s: float,
) -> dict[str, float | str]:
    """Measure one configured position step and its following hold."""
    axis_info = _step_axis(
        segment
    )

    if axis_info is None:
        return {}

    axis, amplitude = axis_info

    component = _COMPONENTS[
        axis
    ]

    start = float(
        segment["start_s"]
    )

    actual_times, actual_values = (
        _ordered_component(
            run.position_actual,
            component,
        )
    )

    mask = (
        (actual_times >= start)
        & (
            actual_times
            <= response_end_s
        )
    )

    times = actual_times[
        mask
    ]

    response = actual_values[
        mask
    ]

    if times.size < 2:
        return {}

    reference_times, reference_values = (
        _ordered_component(
            run.position_reference,
            component,
        )
    )

    target_indices = np.flatnonzero(
        reference_times
        >= start
    )

    if target_indices.size == 0:
        return {}

    target = float(
        reference_values[
            target_indices[0]
        ]
    )

    baseline = (
        target
        - amplitude
    )

    progress = (
        response
        - baseline
    ) / amplitude

    t10 = _first_crossing(
        times,
        progress,
        0.10,
    )

    if t10 is None:
        t90 = None
    else:
        t90 = _first_crossing(
            times,
            progress,
            0.90,
            not_before=t10,
        )

    if (
        t10 is None
        or t90 is None
    ):
        rise_s = float(
            "nan"
        )
    else:
        rise_s = (
            t90
            - t10
        )

    overshoot_percent = (
        max(
            0.0,
            float(
                np.max(progress)
                - 1.0
            ),
        )
        * 100.0
    )

    error = (
        response
        - target
    )

    tolerance = max(
        0.02,
        0.05
        * abs(amplitude),
    )

    settling_s = _settling_time(
        times,
        error,
        start_s=start,
        tolerance=tolerance,
    )

    iae = _integral_absolute_error(
        times,
        error,
    )

    tail_start = max(
        start,
        response_end_s
        - 0.5,
    )

    tail_mask = (
        times
        >= tail_start
    )

    residual = timed_stats(
        times[
            tail_mask
        ].tolist(),
        error[
            tail_mask
        ].tolist(),
    )["rms"]

    cross_errors = []

    for cross_axis, cross_component in enumerate(
        _COMPONENTS
    ):
        if cross_axis == axis:
            continue

        actual_cross = _interp_component(
            run.position_actual,
            cross_component,
            times,
        )

        reference_cross = _interp_component(
            run.position_reference,
            cross_component,
            times,
        )

        cross_errors.append(
            actual_cross
            - reference_cross
        )

    cross_norm = np.sqrt(
        np.sum(
            np.square(
                np.vstack(
                    cross_errors
                )
            ),
            axis=0,
        )
    )

    cross_rms = timed_stats(
        times.tolist(),
        cross_norm.tolist(),
    )["rms"]

    return {
        "axis":
            _AXIS_NAMES[axis],
        "amplitude_m":
            amplitude,
        "rise_s":
            rise_s,
        "settling_s":
            settling_s,
        "overshoot_percent":
            overshoot_percent,
        "iae_m_s":
            iae,
        "residual_rms_m":
            residual,
        "cross_axis_rms_m":
            cross_rms,
    }


def yaw_segment_metrics(
    run: ComparisonRun,
    segment: dict[str, object],
) -> dict[str, float]:
    """Measure wrapped yaw tracking over one configured yaw maneuver."""
    start = float(
        segment["start_s"]
    )

    end = float(
        segment["end_s"]
    )

    yaw = interval_series(
        run.yaw_error,
        start,
        end,
    )

    overall = timed_stats(
        yaw["times_s"],
        yaw["z"],
    )

    tail = interval_series(
        run.yaw_error,
        max(
            start,
            end - 0.5,
        ),
        end,
    )

    residual = timed_stats(
        tail["times_s"],
        tail["z"],
    )

    return {
        "rms_deg":
            overall["rms"],
        "peak_deg":
            overall["peak"],
        "residual_rms_deg":
            residual["rms"],
    }


def figure_eight_metrics(
    run: ComparisonRun,
    segment: dict[str, object],
) -> dict[str, float]:
    """Measure common tracking and actuation over a figure-eight."""
    start = float(
        segment["start_s"]
    )

    end = float(
        segment["end_s"]
    )

    position = interval_series(
        run.position_error,
        start,
        end,
    )

    velocity = interval_series(
        run.velocity_error,
        start,
        end,
    )

    thrust_times, thrust_magnitude = (
        vector_magnitude_series(
            run.thrust
        )
    )

    thrust_times, thrust_magnitude = (
        interval_scalar(
            thrust_times,
            thrust_magnitude,
            start,
            end,
        )
    )

    torque_times, torque_magnitude = (
        vector_magnitude_series(
            run.torque
        )
    )

    torque_times, torque_magnitude = (
        interval_scalar(
            torque_times,
            torque_magnitude,
            start,
            end,
        )
    )

    rate_times, rate_magnitude = (
        vector_magnitude_series(
            run.body_rates
        )
    )

    rate_times, rate_magnitude = (
        interval_scalar(
            rate_times,
            rate_magnitude,
            start,
            end,
        )
    )

    motor_times, motor_magnitude = (
        motor_envelope(
            run.motors
        )
    )

    motor_times, motor_magnitude = (
        interval_scalar(
            motor_times,
            motor_magnitude,
            start,
            end,
        )
    )

    return {
        "position_rms_m":
            timed_stats(
                position["times_s"],
                position["norm"],
            )["rms"],
        "velocity_rms_m_s":
            timed_stats(
                velocity["times_s"],
                velocity["norm"],
            )["rms"],
        "body_rate_rms_deg_s":
            activity_stats(
                rate_times,
                rate_magnitude,
            )["rms"],
        "thrust_rms":
            activity_stats(
                thrust_times,
                thrust_magnitude,
            )["rms"],
        "torque_rms":
            activity_stats(
                torque_times,
                torque_magnitude,
            )["rms"],
        "motor_rms":
            effort_stats(
                motor_times,
                motor_magnitude,
            )["rms"],
        "motor_roughness":
            effort_stats(
                motor_times,
                motor_magnitude,
            )["roughness_rms"],
    }


def summarize_run(
    run: ComparisonRun,
) -> dict[str, object]:
    """Compute common whole-Offboard tracking and activity metrics."""
    thrust_times, thrust_magnitude = (
        vector_magnitude_series(
            run.thrust
        )
    )

    torque_times, torque_magnitude = (
        vector_magnitude_series(
            run.torque
        )
    )

    rate_times, rate_magnitude = (
        vector_magnitude_series(
            run.body_rates
        )
    )

    motor_times, motor_magnitude = (
        motor_envelope(
            run.motors
        )
    )

    return {
        "position":
            timed_stats(
                run.position_error[
                    "times_s"
                ],
                run.position_error[
                    "norm"
                ],
            ),
        "velocity":
            timed_stats(
                run.velocity_error[
                    "times_s"
                ],
                run.velocity_error[
                    "norm"
                ],
            ),
        "thrust":
            activity_stats(
                thrust_times,
                thrust_magnitude,
            ),
        "torque":
            activity_stats(
                torque_times,
                torque_magnitude,
            ),
        "body_rates":
            activity_stats(
                rate_times,
                rate_magnitude,
            ),
        "motors":
            effort_stats(
                motor_times,
                motor_magnitude,
            ),
    }


def _format_float(
    value: float,
    width: int = 10,
) -> str:
    if not np.isfinite(
        value
    ):
        return f"{'n/a':>{width}}"

    return f"{value:>{width}.4f}"


def single_run_summary(
    run: ComparisonRun,
    segments: list[
        dict[str, object]
    ] | None = None,
) -> str:
    """Build detailed performance metrics for one SE3 run."""
    metrics = summarize_run(
        run
    )

    def value(
        number: float,
    ) -> str:
        if not np.isfinite(number):
            return "n/a"

        return f"{number:.4f}"

    lines = [
        "SE3 Performance Metrics",
        "=======================",
        "",
        "statistics: time-weighted over the PX4 Offboard interval",
        "",
        "Whole-run tracking",
        "------------------",
        (
            "position-error norm RMS [m]: "
            + value(
                metrics[
                    "position"
                ]["rms"]
            )
        ),
        (
            "position-error norm peak [m]: "
            + value(
                metrics[
                    "position"
                ]["peak"]
            )
        ),
        (
            "velocity-error norm RMS [m/s]: "
            + value(
                metrics[
                    "velocity"
                ]["rms"]
            )
        ),
        (
            "velocity-error norm peak [m/s]: "
            + value(
                metrics[
                    "velocity"
                ]["peak"]
            )
        ),
        "",
        "Whole-run actuation and response",
        "--------------------------------",
        (
            "normalized thrust magnitude RMS [-]: "
            + value(
                metrics[
                    "thrust"
                ]["rms"]
            )
        ),
        (
            "normalized thrust magnitude peak [-]: "
            + value(
                metrics[
                    "thrust"
                ]["peak"]
            )
        ),
        (
            "normalized thrust roughness [1/s]: "
            + value(
                metrics[
                    "thrust"
                ]["roughness_rms"]
            )
        ),
        (
            "normalized torque magnitude RMS [-]: "
            + value(
                metrics[
                    "torque"
                ]["rms"]
            )
        ),
        (
            "normalized torque magnitude peak [-]: "
            + value(
                metrics[
                    "torque"
                ]["peak"]
            )
        ),
        (
            "normalized torque roughness [1/s]: "
            + value(
                metrics[
                    "torque"
                ]["roughness_rms"]
            )
        ),
        (
            "measured body-rate magnitude RMS [deg/s]: "
            + value(
                metrics[
                    "body_rates"
                ]["rms"]
            )
        ),
        (
            "measured body-rate magnitude peak [deg/s]: "
            + value(
                metrics[
                    "body_rates"
                ]["peak"]
            )
        ),
        (
            "motor-envelope RMS [-]: "
            + value(
                metrics[
                    "motors"
                ]["rms"]
            )
        ),
        (
            "motor-envelope peak [-]: "
            + value(
                metrics[
                    "motors"
                ]["peak"]
            )
        ),
        (
            "motor near-limit duration [s]: "
            + value(
                metrics[
                    "motors"
                ][
                    "near_limit_duration_s"
                ]
            )
        ),
        (
            "motor near-limit time [%]: "
            + value(
                100.0
                * metrics[
                    "motors"
                ][
                    "near_limit_fraction"
                ]
            )
        ),
        (
            "motor roughness [1/s]: "
            + value(
                metrics[
                    "motors"
                ]["roughness_rms"]
            )
        ),
    ]

    if not segments:
        return (
            "\n".join(lines)
            + "\n"
        )

    lines.extend(
        [
            "",
            "Configured maneuver performance",
            "-------------------------------",
        ]
    )

    for index, segment in enumerate(
        segments
    ):
        if segment["type"] != "step":
            continue

        axis_info = _step_axis(
            segment
        )

        if axis_info is None:
            continue

        axis, amplitude = axis_info

        response_end = (
            _step_response_end(
                segments,
                index,
            )
        )

        result = (
            step_response_metrics(
                run,
                segment,
                response_end,
            )
        )

        lines.extend(
            [
                "",
                (
                    f"Step - {segment['name']} "
                    f"({_AXIS_NAMES[axis]} "
                    f"{amplitude:+.2f} m)"
                ),
                (
                    "  rise time [s]: "
                    + value(
                        float(
                            result.get(
                                "rise_s",
                                float("nan"),
                            )
                        )
                    )
                ),
                (
                    "  settling time [s]: "
                    + value(
                        float(
                            result.get(
                                "settling_s",
                                float("nan"),
                            )
                        )
                    )
                ),
                (
                    "  overshoot [%]: "
                    + value(
                        float(
                            result.get(
                                "overshoot_percent",
                                float("nan"),
                            )
                        )
                    )
                ),
                (
                    "  IAE [m s]: "
                    + value(
                        float(
                            result.get(
                                "iae_m_s",
                                float("nan"),
                            )
                        )
                    )
                ),
                (
                    "  residual RMS [m]: "
                    + value(
                        float(
                            result.get(
                                "residual_rms_m",
                                float("nan"),
                            )
                        )
                    )
                ),
                (
                    "  cross-axis RMS [m]: "
                    + value(
                        float(
                            result.get(
                                "cross_axis_rms_m",
                                float("nan"),
                            )
                        )
                    )
                ),
            ]
        )

    for segment in segments:
        if segment["type"] != "yaw":
            continue

        result = yaw_segment_metrics(
            run,
            segment,
        )

        lines.extend(
            [
                "",
                (
                    "Yaw - "
                    + str(
                        segment["name"]
                    )
                ),
                (
                    "  wrapped tracking RMS [deg]: "
                    + value(
                        result[
                            "rms_deg"
                        ]
                    )
                ),
                (
                    "  wrapped tracking peak [deg]: "
                    + value(
                        result[
                            "peak_deg"
                        ]
                    )
                ),
                (
                    "  end residual RMS [deg]: "
                    + value(
                        result[
                            "residual_rms_deg"
                        ]
                    )
                ),
            ]
        )

    for segment in segments:
        if (
            segment["type"]
            != "figure_eight"
        ):
            continue

        result = (
            figure_eight_metrics(
                run,
                segment,
            )
        )

        lines.extend(
            [
                "",
                (
                    "Figure eight - "
                    + str(
                        segment["name"]
                    )
                ),
                (
                    "  position-error RMS [m]: "
                    + value(
                        result[
                            "position_rms_m"
                        ]
                    )
                ),
                (
                    "  velocity-error RMS [m/s]: "
                    + value(
                        result[
                            "velocity_rms_m_s"
                        ]
                    )
                ),
                (
                    "  body-rate magnitude RMS [deg/s]: "
                    + value(
                        result[
                            "body_rate_rms_deg_s"
                        ]
                    )
                ),
                (
                    "  normalized thrust magnitude RMS [-]: "
                    + value(
                        result[
                            "thrust_rms"
                        ]
                    )
                ),
                (
                    "  normalized torque magnitude RMS [-]: "
                    + value(
                        result[
                            "torque_rms"
                        ]
                    )
                ),
                (
                    "  motor-envelope RMS [-]: "
                    + value(
                        result[
                            "motor_rms"
                        ]
                    )
                ),
                (
                    "  motor roughness [1/s]: "
                    + value(
                        result[
                            "motor_roughness"
                        ]
                    )
                ),
            ]
        )

    return (
        "\n".join(lines)
        + "\n"
    )


def comparison_summary(
    runs: list[ComparisonRun],
    segments: list[
        dict[str, object]
    ] | None = None,
) -> str:
    """Build the focused common-quantity SE3 comparison report."""
    metrics = {
        run.label:
            summarize_run(
                run
            )
        for run in runs
    }

    lines = [
        "SE3 Controller Comparison",
        "=========================",
        "",
        f"runs: {len(runs)}",
        "alignment: PX4 Offboard entry = 0 s",
        (
            "scope: common physical tracking and downstream "
            "actuation only"
        ),
        (
            "statistics: time-weighted over the recorded "
            "Offboard interval"
        ),
        "",
        "Whole-run tracking",
        "------------------",
        (
            "  controller              "
            "pos RMS[m] pos peak[m] "
            "vel RMS[m/s] vel peak[m/s]"
        ),
    ]

    for run in runs:
        position = (
            metrics[
                run.label
            ]["position"]
        )

        velocity = (
            metrics[
                run.label
            ]["velocity"]
        )

        lines.append(
            f"  {run.label:<22}"
            f"{_format_float(position['rms'], 11)}"
            f"{_format_float(position['peak'], 12)}"
            f"{_format_float(velocity['rms'], 13)}"
            f"{_format_float(velocity['peak'], 14)}"
        )

    lines.extend(
        [
            "",
            "Whole-run actuation and response",
            "--------------------------------",
            (
                "  controller              "
                "thrust RMS torque RMS rate RMS "
                "motor RMS motor peak limit[s] roughness"
            ),
        ]
    )

    for run in runs:
        row = metrics[
            run.label
        ]

        lines.append(
            f"  {run.label:<22}"
            f"{_format_float(row['thrust']['rms'], 11)}"
            f"{_format_float(row['torque']['rms'], 11)}"
            f"{_format_float(row['body_rates']['rms'], 9)}"
            f"{_format_float(row['motors']['rms'], 10)}"
            f"{_format_float(row['motors']['peak'], 11)}"
            f"{_format_float(row['motors']['near_limit_duration_s'], 9)}"
            f"{_format_float(row['motors']['roughness_rms'], 10)}"
        )

    if not segments:
        return (
            "\n".join(lines)
            + "\n"
        )

    for index, segment in enumerate(
        segments
    ):
        if segment["type"] != "step":
            continue

        axis_info = _step_axis(
            segment
        )

        if axis_info is None:
            continue

        axis, amplitude = axis_info

        response_end = (
            _step_response_end(
                segments,
                index,
            )
        )

        title = (
            "Step response — "
            f"{segment['name']} "
            f"({_AXIS_NAMES[axis]} "
            f"{amplitude:+.2f} m)"
        )

        lines.extend(
            [
                "",
                title,
                "-" * len(title),
                (
                    "  controller              "
                    "rise[s] settle[s] overshoot[%] "
                    "IAE[m s] residual[m] cross-axis[m]"
                ),
            ]
        )

        for run in runs:
            result = (
                step_response_metrics(
                    run,
                    segment,
                    response_end,
                )
            )

            lines.append(
                f"  {run.label:<22}"
                f"{_format_float(float(result.get('rise_s', float('nan'))), 9)}"
                f"{_format_float(float(result.get('settling_s', float('nan'))), 10)}"
                f"{_format_float(float(result.get('overshoot_percent', float('nan'))), 13)}"
                f"{_format_float(float(result.get('iae_m_s', float('nan'))), 10)}"
                f"{_format_float(float(result.get('residual_rms_m', float('nan'))), 12)}"
                f"{_format_float(float(result.get('cross_axis_rms_m', float('nan'))), 14)}"
            )

    for segment in segments:
        if segment["type"] != "yaw":
            continue

        title = (
            "Yaw tracking — "
            + str(
                segment["name"]
            )
        )

        lines.extend(
            [
                "",
                title,
                "-" * len(title),
                (
                    "  controller              "
                    "RMS[deg] peak[deg] "
                    "end residual RMS[deg]"
                ),
            ]
        )

        for run in runs:
            result = (
                yaw_segment_metrics(
                    run,
                    segment,
                )
            )

            lines.append(
                f"  {run.label:<22}"
                f"{_format_float(result['rms_deg'], 10)}"
                f"{_format_float(result['peak_deg'], 10)}"
                f"{_format_float(result['residual_rms_deg'], 22)}"
            )

    for segment in segments:
        if (
            segment["type"]
            != "figure_eight"
        ):
            continue

        title = (
            "Figure-eight tracking — "
            + str(
                segment["name"]
            )
        )

        lines.extend(
            [
                "",
                title,
                "-" * len(title),
                (
                    "  controller              "
                    "pos RMS vel RMS rate RMS "
                    "thrust RMS torque RMS motor RMS roughness"
                ),
            ]
        )

        for run in runs:
            result = (
                figure_eight_metrics(
                    run,
                    segment,
                )
            )

            lines.append(
                f"  {run.label:<22}"
                f"{_format_float(result['position_rms_m'], 8)}"
                f"{_format_float(result['velocity_rms_m_s'], 8)}"
                f"{_format_float(result['body_rate_rms_deg_s'], 9)}"
                f"{_format_float(result['thrust_rms'], 11)}"
                f"{_format_float(result['torque_rms'], 11)}"
                f"{_format_float(result['motor_rms'], 10)}"
                f"{_format_float(result['motor_roughness'], 10)}"
            )

    return (
        "\n".join(lines)
        + "\n"
    )

def _segment_markers(
    axis,
    segments,
) -> None:
    for segment in segments or []:
        axis.axvline(
            float(
                segment["start_s"]
            ),
            linestyle="--",
            linewidth=0.7,
            alpha=0.35,
        )


def _save_scalar_overlay(
    output_path: Path,
    runs: list[ComparisonRun],
    getter,
    title: str,
    ylabel: str,
    segments,
) -> None:
    """Overlay one common scalar quantity for all compared runs."""
    plt = _matplotlib()

    fig, axis = plt.subplots(
        figsize=(
            10.5,
            4.8,
        )
    )

    plotted = 0

    for run in runs:
        times, values = getter(
            run
        )

        if (
            not times
            or not values
        ):
            continue

        axis.plot(
            times,
            values,
            label=run.label,
        )

        plotted += 1

    if plotted == 0:
        plt.close(fig)
        return

    _segment_markers(
        axis,
        segments,
    )

    _decorate_axis(
        axis
    )

    axis.set_xlabel(
        "Time from Offboard entry [s]"
    )

    axis.set_ylabel(
        ylabel
    )

    axis.set_title(
        title,
        pad=28,
    )

    handles, labels = (
        axis.get_legend_handles_labels()
    )

    if handles:
        fig.legend(
            handles,
            labels,
            frameon=False,
            loc="upper center",
            bbox_to_anchor=(
                0.5,
                0.94,
            ),
            ncol=min(
                3,
                len(labels),
            ),
        )

    fig.tight_layout(
        rect=(
            0.02,
            0.02,
            0.99,
            0.86,
        )
    )

    fig.savefig(
        output_path,
        bbox_inches="tight",
    )

    plt.close(
        fig
    )


def write_comparison_plots(
    runs: list[ComparisonRun],
    output_dir: Path,
    segments: list[
        dict[str, object]
    ] | None = None,
) -> list[Path]:
    """Write focused common-quantity SE3 comparison figures."""
    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    for old_plot in output_dir.glob(
        "*.png"
    ):
        old_plot.unlink()

    specifications = [
        (
            "01_position_error_norm.png",
            lambda run: (
                run.position_error[
                    "times_s"
                ],
                run.position_error[
                    "norm"
                ],
            ),
            "Position-Tracking Error Norm",
            "Position error norm [m]",
        ),
        (
            "02_velocity_error_norm.png",
            lambda run: (
                run.velocity_error[
                    "times_s"
                ],
                run.velocity_error[
                    "norm"
                ],
            ),
            "Velocity-Tracking Error Norm",
            "Velocity error norm [m/s]",
        ),
        (
            "03_yaw_error.png",
            lambda run: (
                run.yaw_error[
                    "times_s"
                ],
                run.yaw_error[
                    "z"
                ],
            ),
            "Wrapped Yaw-Tracking Error",
            "Yaw error [deg]",
        ),
        (
            "04_thrust_magnitude.png",
            lambda run:
                vector_magnitude_series(
                    run.thrust
                ),
            "Normalized Thrust Magnitude",
            "Thrust magnitude [-]",
        ),
        (
            "05_torque_magnitude.png",
            lambda run:
                vector_magnitude_series(
                    run.torque
                ),
            "Normalized Torque Magnitude",
            "Torque magnitude [-]",
        ),
        (
            "06_motor_envelope.png",
            lambda run:
                motor_envelope(
                    run.motors
                ),
            "Motor-Command Envelope",
            "Maximum |motor command| [-]",
        ),
        (
            "07_body_rate_magnitude.png",
            lambda run:
                vector_magnitude_series(
                    run.body_rates
                ),
            "Measured Body-Rate Magnitude",
            "Body-rate magnitude [deg/s]",
        ),
    ]

    generated = []

    for (
        filename,
        getter,
        title,
        ylabel,
    ) in specifications:
        plot_path = (
            output_dir
            / filename
        )

        _save_scalar_overlay(
            plot_path,
            runs,
            getter,
            title,
            ylabel,
            segments,
        )

        if plot_path.exists():
            generated.append(
                plot_path
            )

    return generated
