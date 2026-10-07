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
                "font.size": 10,
                "axes.titlesize": 11,
                "axes.labelsize": 10,
                "legend.fontsize": 9,
                "xtick.labelsize": 9,
                "ytick.labelsize": 9,
                "axes.spines.top": False,
                "axes.spines.right": False,
                "lines.linewidth": 1.35,
                "savefig.dpi": 200,
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
    components: list[
        tuple[
            str,
            dict[str, tuple[list[float], list[float]]],
        ]
    ],
    title: str,
    unit: str,
    markers: list[tuple[float, str]] | None = None,
    time_origin_s: float = 0.0,
    xlabel: str = "Time from bag start [s]",
) -> None:
    """Save a multi-component tracking figure.

    Inputs:
        Per-component signal histories, display metadata, optional event
        markers, and the time origin to subtract for presentation.

    Method:
        Plot recorded signals on their own timestamps. Where an actual and
        reference signal both exist, interpolate the reference onto actual
        timestamps over their common interval. Component errors are then
        aligned to a common grid before forming the vector error norm.

    Output:
        One figure containing component histories and, when available, a final
        actual-reference error-norm panel.
    """
    if not components:
        return

    plt = _matplotlib()

    line_styles = {
        "measured": "-",
        "actual": "-",
        "command": "--",
        "setpoint": "--",
        "desired": "--",
        "trajectory": ":",
        "controller setpoint": "-.",
    }

    errors = []

    for component_name, signals in components:
        actual = (
            signals.get("measured")
            or signals.get("actual")
        )

        if actual is None:
            continue

        reference_label = None

        for candidate in (
            "command",
            "controller setpoint",
            "setpoint",
            "desired",
            "trajectory",
        ):
            if candidate in signals:
                reference_label = candidate
                break

        if reference_label is None:
            continue

        reference = signals[
            reference_label
        ]

        wrap_degrees = (
            unit == "[deg]"
            and "yaw" in component_name.lower()
        )

        error_times, error_values = (
            _tracking_error(
                actual[0],
                actual[1],
                reference[0],
                reference[1],
                wrap_degrees=wrap_degrees,
            )
        )

        if error_times:
            errors.append(
                (
                    component_name,
                    error_times,
                    error_values,
                )
            )

    axis_count = (
        len(components)
        + (1 if errors else 0)
    )

    fig, axes = plt.subplots(
        axis_count,
        1,
        sharex=True,
        figsize=(
            10.5,
            1.95 * axis_count + 1.25,
        ),
    )

    if axis_count == 1:
        axes = [axes]

    for axis, (
        component_name,
        signals,
    ) in zip(
        axes,
        components,
    ):
        for label, (
            times_s,
            values,
        ) in signals.items():
            plot_times = (
                np.asarray(
                    times_s,
                    dtype=float,
                )
                - time_origin_s
            )

            axis.plot(
                plot_times,
                values,
                label=label,
                linestyle=line_styles.get(
                    label,
                    "-",
                ),
            )

        if markers:
            shifted_markers = [
                (
                    time_s - time_origin_s,
                    label,
                )
                for time_s, label in markers
            ]
        else:
            shifted_markers = None

        _add_markers(
            axis,
            shifted_markers,
        )
        _decorate_axis(axis)

        axis.set_ylabel(
            f"{component_name} {unit}"
        )

    handles, labels = (
        axes[0].get_legend_handles_labels()
    )

    if errors:
        error_axis = axes[-1]

        # Component errors may originate from independently timestamped topics.
        # Restrict them to one common interval and interpolate onto the first
        # component's grid before forming the Euclidean error norm.
        start_time = max(
            values[0]
            for _, values, _ in errors
        )
        end_time = min(
            values[-1]
            for _, values, _ in errors
        )

        base_times = np.asarray(
            errors[0][1],
            dtype=float,
        )

        grid = base_times[
            (
                base_times >= start_time
            )
            & (
                base_times <= end_time
            )
        ]

        if grid.size:
            aligned_errors = []

            for _, times_s, values in errors:
                aligned_errors.append(
                    np.interp(
                        grid,
                        np.asarray(
                            times_s,
                            dtype=float,
                        ),
                        np.asarray(
                            values,
                            dtype=float,
                        ),
                    )
                )

            error_norm = np.sqrt(
                np.sum(
                    np.square(
                        np.vstack(
                            aligned_errors
                        )
                    ),
                    axis=0,
                )
            )

            error_axis.plot(
                grid - time_origin_s,
                error_norm,
            )

        _decorate_axis(
            error_axis
        )

        error_axis.set_ylabel(
            f"||e|| {unit}"
        )

    axes[-1].set_xlabel(
        xlabel
    )

    fig.suptitle(
        title,
        fontsize=13,
        y=0.995,
    )

    if handles:
        fig.legend(
            handles,
            labels,
            frameon=False,
            loc="upper center",
            ncol=min(
                4,
                len(labels),
            ),
            bbox_to_anchor=(
                0.5,
                0.955,
            ),
        )

    top = (
        0.90
        if handles
        else 0.94
    )

    fig.tight_layout(
        rect=(
            0.02,
            0.02,
            0.99,
            top,
        )
    )

    fig.savefig(
        output_path,
        bbox_inches="tight",
    )
    plt.close(fig)


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
