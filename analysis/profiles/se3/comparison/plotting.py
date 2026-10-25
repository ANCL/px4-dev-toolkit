"""Comparison figures using the same tracking/error convention as single runs.

For each compatible recorded quantity:
  tracking: the measured and commanded component histories for each run;
  error: the recorded actual-minus-reference component errors for each run.

Each run remains independently aligned to confirmed PX4 Offboard entry. Plotting
only consumes ComparisonRun data; it never changes comparisons or their metrics.
"""
from __future__ import annotations

from pathlib import Path
from analysis.comparison import motor_envelope, vector_magnitude_series
from analysis.plots import _decorate_axis, _matplotlib
from .model import ComparisonRun

_COMPONENTS = ("x", "y", "z")


def _segment_markers(axis, segments) -> None:
    """Indicate configured maneuver boundaries, relative to Offboard entry."""
    for segment in segments or []:
        axis.axvline(float(segment["start_s"]), linestyle="--",
                     linewidth=0.75, color="0.55", alpha=0.40)


def _vector_plots(output_dir: Path, runs: list[ComparisonRun], *,
                  prefix: str, title: str, unit: str,
                  actual_field: str, reference_field: str, error_field: str,
                  component_names: tuple[str, ...], segments,
                  xlabel: str = "Time from Offboard entry [s]") -> list[Path]:
    """Overlay each controller's actual/reference and separately its errors.

    Each run's own recorded desired signal is displayed rather than assuming
    all controllers produced identical sampled references. No vector norm
    is shown in the error figure; numerical norm metrics remain unchanged.
    """
    plt = _matplotlib()
    from matplotlib.ticker import MaxNLocator

    available = []
    for run in runs:
        actual = getattr(run, actual_field)
        reference = getattr(run, reference_field)
        errors = getattr(run, error_field)
        if actual["times_s"] or reference["times_s"] or errors["times_s"]:
            available.append((run, actual, reference, errors))

    if not available:
        return []

    plotted = []
    # Keep one panel per physical component, matching single-flight figures.
    fig, axes = plt.subplots(len(component_names), 1, sharex=True,
                            figsize=(11.8, 2.55 * len(component_names) + 1.5),
                            squeeze=False)
    component_keys = ("z",) if component_names == ("yaw",) else _COMPONENTS
    for component, key, axis in zip(component_names, component_keys, axes[:, 0]):
        for run, actual, reference, _ in available:
            if actual["times_s"]:
                line, = axis.plot(actual["times_s"], actual[key],
                                  label=f"{run.label} actual")
                color = line.get_color()
            else:
                color = None
            if reference["times_s"]:
                axis.plot(reference["times_s"], reference[key],
                          label=f"{run.label} reference", color=color,
                          linestyle="--", alpha=0.85)
        _segment_markers(axis, segments)
        _decorate_axis(axis)
        axis.axhline(0, color="0.55", linewidth=0.5, alpha=0.25)
        axis.yaxis.set_major_locator(MaxNLocator(nbins=5))
        axis.set_ylabel(f"{component} {unit}")
    axes[-1, 0].set_xlabel(xlabel)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc="upper center", frameon=False,
                   bbox_to_anchor=(0.5, 0.967), ncol=min(4, len(labels)))
    fig.suptitle(f"{title} — tracking", y=0.999)
    fig.tight_layout(rect=(0.02, 0.02, 0.99, 0.90 if handles else 0.94))
    tracking_path = output_dir / f"{prefix}_tracking.png"
    fig.savefig(tracking_path, bbox_inches="tight")
    plt.close(fig)
    plotted.append(tracking_path)

    # Only record an error figure if an actual and reference overlap exists.
    has_error = any(errors["times_s"] for _, _, _, errors in available)
    if not has_error:
        return plotted

    fig, axes = plt.subplots(len(component_names), 1, sharex=True,
                            figsize=(11.8, 2.55 * len(component_names) + 1.5),
                            squeeze=False)
    for component, key, axis in zip(component_names, component_keys, axes[:, 0]):
        for run, _, _, errors in available:
            if errors["times_s"]:
                axis.plot(errors["times_s"], errors[key], label=run.label)
        axis.axhline(0, color="0.5", linewidth=0.9, linestyle="--")
        _segment_markers(axis, segments)
        _decorate_axis(axis)
        axis.yaxis.set_major_locator(MaxNLocator(nbins=5))
        axis.set_ylabel(f"{component} error {unit}")
    axes[-1, 0].set_xlabel(xlabel)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc="upper center", frameon=False,
                   bbox_to_anchor=(0.5, 0.965), ncol=min(4, len(labels)))
    fig.suptitle(f"{title} — tracking error (actual − reference)", y=0.998)
    fig.tight_layout(rect=(0.02, 0.02, 0.99, 0.90 if handles else 0.94))
    error_path = output_dir / f"{prefix}_error.png"
    fig.savefig(error_path, bbox_inches="tight")
    plt.close(fig)
    plotted.append(error_path)
    return plotted


def _save_scalar_overlay(output_path: Path, runs: list[ComparisonRun],
                         getter, title: str, ylabel: str, segments,
                         xlabel: str = "Time from Offboard entry [s]") -> None:
    """Overlay controller-response and actuator-effort scalar magnitudes."""
    plt = _matplotlib()
    fig, axis = plt.subplots(figsize=(11.8, 4.8))
    for run in runs:
        times, values = getter(run)
        if times and values:
            axis.plot(times, values, label=run.label)
    handles, labels = axis.get_legend_handles_labels()
    if not handles:
        plt.close(fig)
        return
    _segment_markers(axis, segments)
    _decorate_axis(axis)
    axis.set(xlabel=xlabel, ylabel=ylabel, title=title)
    fig.legend(handles, labels, frameon=False, loc="upper center",
               bbox_to_anchor=(0.5, 0.94), ncol=min(4, len(labels)))
    fig.tight_layout(rect=(0.02, 0.02, 0.99, 0.87))
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)


def write_comparison_plots(runs: list[ComparisonRun], output_dir: Path,
                           segments: list[dict[str, object]] | None = None, *,
                           xlabel: str = "Time from Offboard entry [s]") -> list[Path]:
    """Generate per-axis comparison pairs and existing scalar effort figures.

    The existing comparison time windows, statistics, and maneuver boundaries
    are retained. Remove only old files owned by this plotting module.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    previous = (
        "01_position_error_norm.png", "02_velocity_error_norm.png",
        "03_yaw_error.png", "04_thrust_magnitude.png",
        "05_torque_magnitude.png", "06_motor_envelope.png",
        "07_body_rate_magnitude.png",
    )
    for filename in previous:
        (output_dir / filename).unlink(missing_ok=True)
    for stem in ("01_position", "02_velocity", "03_attitude",
                 "04_body_rates", "05_yaw"):
        for suffix in ("_tracking.png", "_error.png"):
            (output_dir / (stem + suffix)).unlink(missing_ok=True)

    generated = []
    specifications = (
        ("01_position", "Position (NED)", "[m]", "position_actual",
         "position_reference", "position_error", ("North", "East", "Down")),
        ("02_velocity", "Velocity (NED)", "[m/s]", "velocity_actual",
         "velocity_reference", "velocity_error", ("North", "East", "Down")),
        ("03_attitude", "Attitude", "[deg]", "attitude_actual",
         "attitude_reference", "attitude_error", ("roll", "pitch", "yaw")),
        ("04_body_rates", "Body rates (FRD)", "[deg/s]", "body_rates",
         "body_rate_reference", "body_rate_error", ("p", "q", "r")),
        ("05_yaw", "Trajectory yaw", "[deg]", "yaw_actual",
         "yaw_reference", "yaw_error", ("yaw",)),
    )
    for prefix, title, unit, actual_field, reference_field, error_field, names in specifications:
        generated.extend(_vector_plots(output_dir, runs, prefix=prefix,
                                       title=title, unit=unit, actual_field=actual_field,
                                       reference_field=reference_field, error_field=error_field,
                                       component_names=names, segments=segments, xlabel=xlabel))

    scalars = (
        ("06_thrust_magnitude.png", lambda run: vector_magnitude_series(run.thrust),
         "Normalized thrust magnitude", "Thrust magnitude [-]"),
        ("07_torque_magnitude.png", lambda run: vector_magnitude_series(run.torque),
         "Normalized torque magnitude", "Torque magnitude [-]"),
        ("08_motor_envelope.png", lambda run: motor_envelope(run.motors),
         "Motor-command envelope", "Maximum |motor command| [-]"),
        ("09_body_rate_magnitude.png", lambda run: vector_magnitude_series(run.body_rates),
         "Measured body-rate magnitude", "Body-rate magnitude [deg/s]"),
    )
    for filename, getter, title, unit in scalars:
        path = output_dir / filename
        _save_scalar_overlay(path, runs, getter, title, unit, segments, xlabel=xlabel)
        if path.is_file():
            generated.append(path)
    return generated
