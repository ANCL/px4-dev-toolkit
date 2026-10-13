"""Report measured PX4 pipeline layers and time-aligned tracking errors.

Signal extraction belongs to pipeline/signals.py and pipeline/analysis.py.
Bypassed or unobserved layers remain unavailable; reporting never fabricates
a downstream setpoint to fill gaps in recorded data.
"""

from __future__ import annotations

import numpy as np
from .statistics import _aligned_error_stats


def control_pipeline_summary(
    pipeline: dict[str, object],
) -> list[str]:
    """Summarize the extracted native PX4 control pipeline.

    Inputs:
        Layer-organized output from analyze_control_pipeline().

    Method:
        Report signal ranges and actual-reference tracking statistics at each
        available control layer. Missing/bypassed layers remain explicitly
        unavailable rather than being inferred.

    Returns:
        Human-readable summary lines in controller-cascade order.
    """

    def tracking_lines(
        actual,
        reference,
        components,
    ):
        """Align each measured axis with its own reference time series."""
        result = [
            "                         RMS error    max |error|"
        ]

        for label, component, wrap in components:
            stats = _aligned_error_stats(
                actual,
                reference,
                component,
                wrap_degrees=wrap,
            )

            if stats is None:
                result.append(
                    f"  {label:<20} unavailable"
                )
                continue

            rms, maximum = stats
            result.append(
                f"  {label:<20} "
                f"{rms:>10.4f}    {maximum:>10.4f}"
            )

        return result

    def range_lines(series, components):
        """Describe the available values, without reconstructing missing data."""
        result = [
            "                         minimum      maximum"
            "        mean    peak |value|"
        ]

        for label, component in components:
            values = np.asarray(
                series[component],
                dtype=float,
            )

            if values.size == 0:
                result.append(
                    f"  {label:<20} unavailable"
                )
                continue

            result.append(
                f"  {label:<20} "
                f"{np.min(values):>10.4f}   "
                f"{np.max(values):>10.4f}   "
                f"{np.mean(values):>10.4f}   "
                f"{np.max(np.abs(values)):>10.4f}"
            )

        return result

    lines = [
        "PX4 Control Pipeline Summary",
        "============================",
        "",
        "Layer 1 - Trajectory reference",
        "------------------------------",
        (
            "trajectory_setpoint samples: "
            f"{len(pipeline['position']['trajectory']['times_s'])}"
        ),
        "",
        "Position [m]",
        *range_lines(
            pipeline["position"]["trajectory"],
            [
                ("x / North", "x"),
                ("y / East", "y"),
                ("z / Down", "z"),
            ],
        ),
        "",
        "Velocity [m/s]",
        *range_lines(
            pipeline["velocity"]["trajectory"],
            [
                ("vx / North", "x"),
                ("vy / East", "y"),
                ("vz / Down", "z"),
            ],
        ),
        "",
        "Acceleration [m/s^2]",
        *range_lines(
            pipeline["acceleration"]["trajectory"],
            [
                ("ax / North", "x"),
                ("ay / East", "y"),
                ("az / Down", "z"),
            ],
        ),
        "",
        "Layer 2 - Position controller",
        "-----------------------------",
        (
            "vehicle_local_position_setpoint samples: "
            f"{len(pipeline['position']['controller setpoint']['times_s'])}"
        ),
        (
            "vehicle_local_position samples: "
            f"{len(pipeline['position']['actual']['times_s'])}"
        ),
        "",
        "Trajectory -> controller setpoint",
        "",
        "Position [m]",
        *tracking_lines(
            pipeline["position"]["controller setpoint"],
            pipeline["position"]["trajectory"],
            [
                ("x / North", "x", False),
                ("y / East", "y", False),
                ("z / Down", "z", False),
            ],
        ),
        "",
        "Velocity [m/s]",
        *tracking_lines(
            pipeline["velocity"]["controller setpoint"],
            pipeline["velocity"]["trajectory"],
            [
                ("vx / North", "x", False),
                ("vy / East", "y", False),
                ("vz / Down", "z", False),
            ],
        ),
        "",
        "Acceleration [m/s^2]",
        *tracking_lines(
            pipeline["acceleration"]["controller setpoint"],
            pipeline["acceleration"]["trajectory"],
            [
                ("ax / North", "x", False),
                ("ay / East", "y", False),
                ("az / Down", "z", False),
            ],
        ),
        "",
        "Controller setpoint -> actual",
        "",
        "Position [m]",
        *tracking_lines(
            pipeline["position"]["actual"],
            pipeline["position"]["controller setpoint"],
            [
                ("x / North", "x", False),
                ("y / East", "y", False),
                ("z / Down", "z", False),
            ],
        ),
        "",
        "Velocity [m/s]",
        *tracking_lines(
            pipeline["velocity"]["actual"],
            pipeline["velocity"]["controller setpoint"],
            [
                ("vx / North", "x", False),
                ("vy / East", "y", False),
                ("vz / Down", "z", False),
            ],
        ),
        "",
        "Acceleration [m/s^2]",
        *tracking_lines(
            pipeline["acceleration"]["actual"],
            pipeline["acceleration"]["controller setpoint"],
            [
                ("ax / North", "x", False),
                ("ay / East", "y", False),
                ("az / Down", "z", False),
            ],
        ),
        "",
        "Layer 3 - Attitude controller",
        "-----------------------------",
        (
            "vehicle_attitude_setpoint samples: "
            f"{len(pipeline['attitude']['setpoint']['times_s'])}"
        ),
        (
            "vehicle_attitude samples: "
            f"{len(pipeline['attitude']['actual']['times_s'])}"
        ),
        "",
        "Attitude tracking [deg]",
        *tracking_lines(
            pipeline["attitude"]["actual"],
            pipeline["attitude"]["setpoint"],
            [
                ("roll", "x", False),
                ("pitch", "y", False),
                ("yaw", "z", True),
            ],
        ),
        "",
        "Layer 4 - Rate controller",
        "-------------------------",
        (
            "vehicle_rates_setpoint samples: "
            f"{len(pipeline['rates']['setpoint']['times_s'])}"
        ),
        (
            "vehicle_angular_velocity samples: "
            f"{len(pipeline['rates']['actual']['times_s'])}"
        ),
        "",
        "Body-rate tracking [deg/s]",
        *tracking_lines(
            pipeline["rates"]["actual"],
            pipeline["rates"]["setpoint"],
            [
                ("p / roll", "x", False),
                ("q / pitch", "y", False),
                ("r / yaw", "z", False),
            ],
        ),
        "",
        "Layer 5 - Controller output",
        "---------------------------",
        (
            "vehicle_thrust_setpoint samples: "
            f"{len(pipeline['thrust']['setpoint']['times_s'])}"
        ),
        "",
        "Normalized thrust setpoint",
        *range_lines(
            pipeline["thrust"]["setpoint"],
            [
                ("x / body", "x"),
                ("y / body", "y"),
                ("z / body", "z"),
            ],
        ),
        "",
        (
            "vehicle_torque_setpoint samples: "
            f"{len(pipeline['torque']['setpoint']['times_s'])}"
        ),
        "",
        "Normalized torque setpoint",
        *range_lines(
            pipeline["torque"]["setpoint"],
            [
                ("x / roll", "x"),
                ("y / pitch", "y"),
                ("z / yaw", "z"),
            ],
        ),
        "",
        "Layer 6 - Control allocation",
        "----------------------------",
        f"Active motor channels: {len(pipeline['motors'])}",
    ]

    motor_sample_count = max(
        (
            len(times)
            for times, _ in pipeline["motors"].values()
        ),
        default=0,
    )

    lines.append(
        f"actuator_motors samples: {motor_sample_count}"
    )
    lines.append("")

    for label, (_times, values) in pipeline["motors"].items():
        array = np.asarray(values, dtype=float)

        if array.size == 0:
            continue

        lines.append(
            f"  {label:<12} "
            f"min {np.min(array):>8.4f}   "
            f"max {np.max(array):>8.4f}   "
            f"mean {np.mean(array):>8.4f}   "
            f"peak {np.max(np.abs(array)):>8.4f}"
        )

    return lines
