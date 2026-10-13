"""Calculate maneuver-response and time-weighted comparison metrics."""

from __future__ import annotations

import numpy as np
from analysis.comparison import (
    activity_stats,
    effort_stats,
    interval_scalar,
    interval_series,
    motor_envelope,
    timed_stats,
    vector_magnitude_series,
)
from .model import ComparisonRun, _AXIS_NAMES, _COMPONENTS


def _ordered_component(
    series: dict[str, list[float]],
    component: str,
) -> tuple[np.ndarray, np.ndarray]:
    """Prepare one recorded component for interpolation.

    Inputs:
        One component and its recorded timestamps.

    Method:
        Stable-sort by time and retain the first recorded value at duplicate
        timestamps, yielding a deterministic strictly increasing time base.

    Returns:
        Ordered unique timestamps and matching component values.
    """
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

    order = np.argsort(times, kind="stable")

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
    """Interpolate one recorded component onto an explicit comparison grid."""
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
    """Find the first sampled threshold crossing.

    Inputs:
        Sampled response, threshold, and optional earliest allowed time.

    Method:
        Search recorded samples directly; no synthetic crossing time is
        interpolated.

    Returns:
        Crossing timestamp, or None when the threshold is never reached.
    """
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
    """Measure settling time within the supplied response window.

    Inputs:
        Response timestamps, tracking error, step start, and tolerance.

    Method:
        Find the first post-step sample for which that sample and every
        remaining sample stay inside the tolerance band.

    Returns:
        Settling time relative to step start, or NaN when not settled.
    """
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
    """Integrate |tracking error| over time with the trapezoidal rule."""
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
    """Return commanded axis and signed amplitude for a single-axis step.

    Step-response metrics are defined only for segments with exactly one
    non-zero translational offset. Multi-axis steps are deliberately excluded
    rather than assigning their coupled response to an arbitrary axis.
    """
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
    """Measure one single-axis position-step response.

    Inputs:
        Offboard-aligned run, configured step metadata, and response-window end.

    Method:
        Use the configured step amplitude to normalize the measured response,
        compute sampled 10-90 % rise time, overshoot, settling, IAE, final
        residual, and uncommanded-axis coupling. The immediately following hold
        may extend the response window.

    Returns:
        Step-response metrics, or an empty mapping when the segment/data cannot
        support a single-axis response measurement.
    """
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

    # Sequence semantics give the exact boundary sample to the preceding
    # segment. Select the first reference strictly after the step boundary so
    # an exactly sampled boundary cannot be mistaken for the new step target.
    target_indices = np.flatnonzero(
        reference_times
        > start
    )

    if target_indices.size == 0:
        return {}

    target = float(
        reference_values[
            target_indices[0]
        ]
    )

    # Normalize the measured response from the pre-step baseline to the
    # configured target. This gives a dimensionless response where 0 is the
    # starting value and 1 is the requested step.
    baseline = (
        target
        - amplitude
    )

    progress = (
        response
        - baseline
    ) / amplitude

    # Standard 10-90 percent rise time, measured from the actual response
    # crossings rather than from the configured segment boundaries.
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

    # Settling requires the response to remain inside the larger of a
    # 2-centimetre absolute band or a 5-percent step-amplitude band.
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

    # Residual error is measured over the final 0.5 s of the response window,
    # including the post-step hold when one is configured.
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

    # Cross-axis RMS exposes coupling that a single commanded-axis metric
    # would hide. Compare the two uncommanded axes on the commanded-axis time
    # grid and combine them as a Euclidean norm.
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
    """Measure wrapped yaw tracking over one configured yaw maneuver.

    The metric window is [segment start, segment end), and angular error is
    already wrapped to [-180, 180) degrees in build_run().
    """
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
    """Measure common physical performance over a figure-eight window.

    Returns time-weighted position/velocity tracking, measured body-rate,
    normalized wrench, and motor-envelope metrics that are comparable across
    all controller handoff choices.
    """
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
    """Compute common metrics over one recorded Offboard interval.

    Tracking uses actual-reference error norms. Actuation/response metrics use
    signal magnitudes and time weighting so different logging rates do not
    change the interpretation of a run.
    """
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
