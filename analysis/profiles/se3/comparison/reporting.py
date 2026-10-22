"""Present common physical metrics for single and compared SE(3) runs.

Whole-run statistics cover confirmed Offboard operation; maneuver results use
configured trajectory windows. A missing quantity is shown as unavailable,
not inferred from an unrelated controller handoff.
"""

from __future__ import annotations

import numpy as np
from .model import ComparisonRun, _AXIS_NAMES
from .metrics import (
    _step_axis,
    _step_response_end,
    figure_eight_metrics,
    step_response_metrics,
    summarize_run,
    yaw_segment_metrics,
)


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
    """Build the detailed performance report for one SE(3) run.

    Whole-run statistics cover the confirmed Offboard interval. When segment
    metadata is available, maneuver-specific metrics use their configured
    Offboard-relative windows.
    """
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
    """Build a common-quantity comparison across SE(3) controllers.

    Each run is independently aligned to its confirmed Offboard entry. Only
    quantities available with the same physical meaning across controllers are
    compared; controller-specific internal terms remain diagnostic only.
    """
    metrics = {
        run.label:
            summarize_run(
                run
            )
        for run in runs
    }

    lines = [
        "Controller Comparison",
        "=====================",
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
