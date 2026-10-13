"""Compute time-aligned tracking errors and observation ranges."""

from __future__ import annotations

import numpy as np


def _aligned_error_stats(
    actual: dict[str, list[float]],
    reference: dict[str, list[float]],
    component: str,
    *,
    wrap_degrees: bool = False,
) -> tuple[float, float] | None:
    """Compute tracking-error statistics on independently timed signals.

    Inputs:
        Actual and reference series plus the component to compare.

    Method:
        Restrict to their common interval and interpolate the reference onto
        actual-signal timestamps. Angular errors are wrapped when requested.

    Returns:
        RMS and maximum absolute tracking error, or None when no overlap exists.
    """
    actual_times = np.asarray(actual["times_s"], dtype=float)
    reference_times = np.asarray(reference["times_s"], dtype=float)

    if actual_times.size == 0 or reference_times.size == 0:
        return None

    actual_values = np.asarray(actual[component], dtype=float)
    reference_values = np.asarray(reference[component], dtype=float)

    order = np.argsort(reference_times, kind="stable")
    reference_times = reference_times[order]
    reference_values = reference_values[order]

    reference_times, unique = np.unique(
        reference_times,
        return_index=True,
    )
    reference_values = reference_values[unique]

    start = max(actual_times[0], reference_times[0])
    end = min(actual_times[-1], reference_times[-1])

    mask = (
        (actual_times >= start)
        & (actual_times <= end)
    )

    if not np.any(mask):
        return None

    times = actual_times[mask]

    error = actual_values[mask] - np.interp(
        times,
        reference_times,
        reference_values,
    )

    if wrap_degrees:
        error = (error + 180.0) % 360.0 - 180.0

    return (
        float(np.sqrt(np.mean(error ** 2))),
        float(np.max(np.abs(error))),
    )


def _axis_range(
    series: dict[str, list[float]],
    component: str,
) -> tuple[float, float, float] | None:
    """Return minimum, maximum, and peak absolute value for one signal axis."""
    values = np.asarray(series[component], dtype=float)

    if values.size == 0:
        return None

    return (
        float(np.min(values)),
        float(np.max(values)),
        float(np.max(np.abs(values))),
    )
