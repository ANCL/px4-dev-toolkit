"""Render SE(3) comparison overlays and maneuver markers."""

from __future__ import annotations

from pathlib import Path
from analysis.comparison import motor_envelope, vector_magnitude_series
from analysis.plots import _decorate_axis, _matplotlib
from .model import ComparisonRun


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
    """Write Offboard-aligned common-quantity comparison figures.

    Inputs:
        Comparison runs plus optional configured maneuver boundaries.

    Output:
        Overlay plots for tracking error, body response, normalized wrench, and
        motor effort; controller-specific diagnostics are deliberately omitted.
    """
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
