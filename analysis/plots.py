"""Plotting helpers for PX4 flight-data analysis.

Inputs are already expressed on the analysis rosbag clock, usually as seconds
from bag start. Functions may shift that clock to a profile-specific origin
such as confirmed Offboard entry.

When measured and reference signals have independent timestamps, plotting uses
the same common-interval/interpolation semantics as the numerical analysis.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np


def _matplotlib():
    """Configure a consistent publication-style Matplotlib backend."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        matplotlib.rcParams.update(
            {
                "font.family": "STIXGeneral",
                "mathtext.fontset": "stix",
                "font.size": 11.5,
                "axes.titlesize": 13,
                "axes.labelsize": 11.5,
                "legend.fontsize": 10.5,
                "xtick.labelsize": 10.5,
                "ytick.labelsize": 10.5,
                "axes.spines.top": False,
                "axes.spines.right": False,
                "lines.linewidth": 1.6,
                "savefig.dpi": 300,
            }
        )

        import matplotlib.pyplot as plt

    except ImportError as exc:
        raise RuntimeError(
            "matplotlib is not installed; summary.txt was still generated."
        ) from exc

    return plt


def _decorate_axis(axis) -> None:
    """Apply the common time-history axis appearance."""
    axis.grid(
        True,
        which="major",
        linewidth=0.5,
        alpha=0.30,
    )
    axis.margins(x=0.01)


def _add_markers(
    axis,
    markers: list[tuple[float, str]] | None,
) -> None:
    """Draw experiment-event boundaries without adding legend entries."""
    for marker_time_s, _ in markers or []:
        axis.axvline(
            marker_time_s,
            linestyle="--",
            linewidth=0.9,
            alpha=0.65,
        )


def _add_marker_labels(
    axis,
    markers: list[tuple[float, str]] | None,
) -> None:
    """Label event boundaries once on the first subplot."""
    for marker_time_s, label in markers or []:
        axis.annotate(
            label,
            xy=(marker_time_s, 1.0),
            xycoords=("data", "axes fraction"),
            xytext=(-4, -5),
            textcoords="offset points",
            rotation=90,
            ha="right",
            va="top",
            fontsize=8,
        )


def _tracking_error(
    actual_times: list[float],
    actual_values: list[float],
    reference_times: list[float],
    reference_values: list[float],
    *,
    wrap_degrees: bool = False,
) -> tuple[list[float], list[float]]:
    """Compute aligned actual-reference error for plotting.

    Inputs:
        Independently timestamped actual/reference scalar histories.

    Method:
        Restrict to their common interval and interpolate reference values onto
        actual-signal timestamps. Degree-valued angles may be wrapped.

    Returns:
        Actual timestamps and actual - reference error.
    """
    if not actual_times or not reference_times:
        return [], []

    actual_t = np.asarray(actual_times, dtype=float)
    actual_y = np.asarray(actual_values, dtype=float)
    reference_t = np.asarray(reference_times, dtype=float)
    reference_y = np.asarray(reference_values, dtype=float)

    order = np.argsort(reference_t, kind="stable")
    reference_t = reference_t[order]
    reference_y = reference_y[order]

    reference_t, unique = np.unique(
        reference_t,
        return_index=True,
    )
    reference_y = reference_y[unique]

    start = max(actual_t[0], reference_t[0])
    end = min(actual_t[-1], reference_t[-1])

    mask = (actual_t >= start) & (actual_t <= end)

    if not np.any(mask):
        return [], []

    times = actual_t[mask]
    error = actual_y[mask] - np.interp(
        times,
        reference_t,
        reference_y,
    )

    if wrap_degrees:
        error = (error + 180.0) % 360.0 - 180.0

    return times.tolist(), error.tolist()


def save_series_plot(
    output_path: Path,
    *,
    times_s: list[float],
    series: dict[str, list[float]],
    title: str,
    ylabel: str,
    markers: list[tuple[float, str]] | None = None,
) -> None:
    """Save a clean single-axis time-history figure."""
    plt = _matplotlib()

    fig, axis = plt.subplots(figsize=(10, 4.6))

    for label, values in series.items():
        axis.plot(times_s, values, label=label)

    _add_markers(axis, markers)
    _add_marker_labels(axis, markers)
    _decorate_axis(axis)

    axis.set_xlabel(r"Time from bag start, $t$ [s]")
    axis.set_ylabel(ylabel)
    axis.set_title(title, pad=12)

    axis.legend(
        frameon=False,
        loc="best",
    )

    fig.tight_layout()
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)


def save_tracking_plot(
    output_path: Path,
    *,
    components: list[tuple[str, dict[str, tuple[list[float], list[float]]]]],
    title: str,
    unit: str,
    markers: list[tuple[float, str]] | None = None,
    time_origin_s: float = 0.0,
    xlabel: str = "Time from bag start [s]",
) -> Path | None:
    """Save observed/reference histories and, if comparable, a separate error figure.

    Both files share the same measured-sample time axis. No vector-error norm
    subplot is added. Missing reference channels never produce invented errors.
    The return value is the companion error path (or None).
    """
    if not components:
        return None

    plt = _matplotlib()
    from matplotlib.ticker import MaxNLocator

    labels = {
        "measured": "-", "actual": "-", "command": "--",
        "setpoint": "--", "desired": "--", "trajectory": ":",
        "controller setpoint": "-.", "reference": "--",
    }
    reference_order = (
        "command", "setpoint", "desired", "trajectory",
        "controller setpoint", "reference",
    )
    # Prefer the desired trajectory over an intermediate native controller
    # setpoint when both are recorded for the same physical quantity.
    errors: list[tuple[str, list[float], list[float], str]] = []
    rows = len(components)
    fig, axes = plt.subplots(rows, 1, sharex=True, figsize=(11.8, 2.5 * rows + 1.35), squeeze=False)
    for axis, (component_name, signals) in zip(axes[:, 0], components):
        for label, (times, values) in signals.items():
            axis.plot(
                np.asarray(times, dtype=float) - time_origin_s,
                values, label=label, linestyle=labels.get(label, "-"),
            )
        _add_markers(axis, [
            (stamp - time_origin_s, label) for stamp, label in (markers or [])
        ])
        _decorate_axis(axis)
        axis.set_ylabel(f"{component_name} {unit}")
        axis.yaxis.set_major_locator(MaxNLocator(nbins=5))

        actual = signals.get("measured") or signals.get("actual")
        ref_name = next((key for key in reference_order if key in signals), None)
        if actual is None or ref_name is None:
            continue
        ref = signals[ref_name]
        angular = unit == "[deg]" and "yaw" in component_name.lower()
        times, values = _tracking_error(
            actual[0], actual[1], ref[0], ref[1], wrap_degrees=angular,
        )
        if times:
            errors.append((component_name, times, values, ref_name))

    if markers:
        _add_marker_labels(
            axes[0, 0], [(stamp - time_origin_s, label) for stamp, label in markers]
        )
    axes[-1, 0].set_xlabel(xlabel)
    handles, legend_labels = axes[0, 0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, legend_labels, loc="upper center", ncol=min(len(handles), 4),
                   bbox_to_anchor=(0.5, 0.968), frameon=False)
    fig.suptitle(title, y=0.997)
    fig.tight_layout(rect=(0.02, 0.025, 0.99, 0.91 if handles else 0.94))
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)

    if not errors:
        return None

    # A companion error plot shares the same per-component semantics as the
    # numerical statistics: actual minus interpolated reference. Yaw wraps.
    error_path = output_path.with_name(
        output_path.stem.replace("_tracking", "_error") + output_path.suffix
    )
    if error_path == output_path:
        error_path = output_path.with_name(output_path.stem + "_error" + output_path.suffix)
    fig, axes = plt.subplots(len(errors), 1, sharex=True,
                            figsize=(11.8, 2.5 * len(errors) + 1.35), squeeze=False)
    for axis, (component_name, times, values, ref_name) in zip(axes[:, 0], errors):
        axis.plot(np.asarray(times) - time_origin_s, values, label="actual - " + ref_name)
        axis.axhline(0.0, color="0.45", linewidth=0.85, linestyle="--")
        _decorate_axis(axis)
        axis.set_ylabel(f"{component_name} error {unit}")
        axis.yaxis.set_major_locator(MaxNLocator(nbins=5))
        _add_markers(axis, [
            (stamp - time_origin_s, label) for stamp, label in (markers or [])
        ])
    axes[-1, 0].set_xlabel(xlabel)
    fig.suptitle(title.replace("Tracking", "Tracking Error") + " — actual − reference", y=0.987)
    fig.tight_layout(rect=(0.02, 0.025, 0.99, 0.94))
    fig.savefig(error_path, bbox_inches="tight")
    plt.close(fig)
    return error_path

def save_mode_timeline(
    output_path: Path,
    *,
    transitions: list[tuple[float, int]],
    state_names: dict[int, str],
) -> None:
    """Save the PX4 navigation-state history."""
    if not transitions:
        return

    plt = _matplotlib()

    times_s = [time for time, _ in transitions]
    states = [state for _, state in transitions]
    unique_states = sorted(set(states))

    fig, axis = plt.subplots(figsize=(10, 4.5))

    axis.step(
        times_s,
        states,
        where="post",
        linewidth=1.5,
    )

    axis.set_yticks(
        unique_states,
        [
            f"{state_names.get(state, 'STATE')} ({state})"
            for state in unique_states
        ],
    )

    axis.set_xlabel(r"Time from bag start, $t$ [s]")
    axis.set_ylabel("PX4 navigation state")
    axis.set_title(
        "PX4 Navigation-State Timeline",
        pad=12,
    )

    _decorate_axis(axis)

    fig.tight_layout()
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)
