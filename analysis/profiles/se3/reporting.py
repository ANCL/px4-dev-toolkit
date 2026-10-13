"""Summarize recorded SE(3) tracking, feasibility, and Offboard behavior.

Native PX4 observations and toolkit handoff layers are reported separately.
Tracking metrics explicitly time-align actual/reference series; missing or
bypassed channels are not substituted with calculated setpoints.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from analysis.core import BagData
from analysis.pipeline import _aligned_error_stats


def _boolean_activity_duration(
    times_s: list[float],
    active: list[bool],
) -> tuple[float, float]:
    """Measure time spent in a sampled boolean state.

    Inputs:
        Sample timestamps and corresponding boolean activity values.

    Method:
        Integrate the boolean history trapezoidally so irregular sampling does
        not bias the result toward regions containing more messages.

    Returns:
        Active duration and active fraction of covered time.
    """
    count = min(
        len(times_s),
        len(active),
    )

    if count < 2:
        return (
            0.0,
            float("nan"),
        )

    total_duration = 0.0
    active_duration = 0.0

    for index in range(
        count - 1
    ):
        dt = (
            times_s[index + 1]
            - times_s[index]
        )

        if (
            not math.isfinite(dt)
            or dt <= 0.0
        ):
            continue

        total_duration += dt

        active_duration += (
            0.5
            * (
                float(active[index])
                + float(
                    active[index + 1]
                )
            )
            * dt
        )

    if total_duration <= 0.0:
        return (
            0.0,
            float("nan"),
        )

    return (
        active_duration,
        active_duration
        / total_duration,
    )


def _geometric_physical_summary(
    diagnostics: dict[str, object],
) -> list[str]:
    """Summarize physical-wrench feasibility intervention.

    Inputs:
        Requested/applied collective thrust and moment-scale diagnostics.

    Method:
        Interpret moment_scale < 1 as uniform moment reduction and a
        requested/applied collective difference as collective saturation.

    Returns:
        Human-readable saturation duration, fraction, scale, and thrust ranges.
    """
    title = "Geometric physical-wrench feasibility"

    lines = [
        title,
        "-" * len(title),
    ]

    required = (
        "moment_scale",
        "requested_collective_thrust",
        "applied_collective_thrust",
    )

    if not all(
        name in diagnostics
        for name in required
    ):
        lines.append(
            "Geometric physical feasibility diagnostics unavailable."
        )
        return lines

    scale_series = diagnostics[
        "moment_scale"
    ]

    requested_series = diagnostics[
        "requested_collective_thrust"
    ]

    applied_series = diagnostics[
        "applied_collective_thrust"
    ]

    moment_scale = scale_series[
        "values"
    ]

    requested = requested_series[
        "values"
    ]

    applied = applied_series[
        "values"
    ]

    if (
        not moment_scale
        or not requested
        or not applied
    ):
        lines.append(
            "Geometric physical feasibility diagnostics contain no data."
        )
        return lines

    moment_active = [
        scale < 1.0 - 1.0e-9
        for scale in moment_scale
    ]

    moment_duration, moment_fraction = (
        _boolean_activity_duration(
            scale_series["times_s"],
            moment_active,
        )
    )

    collective_count = min(
        len(requested),
        len(applied),
    )

    collective_active = [
        abs(
            requested[index]
            - applied[index]
        )
        > 1.0e-9
        for index in range(
            collective_count
        )
    ]

    collective_duration, collective_fraction = (
        _boolean_activity_duration(
            requested_series[
                "times_s"
            ][:collective_count],
            collective_active,
        )
    )

    def percent(
        fraction: float,
    ) -> str:
        if not math.isfinite(
            fraction
        ):
            return "n/a"

        return (
            f"{100.0 * fraction:.4f}"
        )

    lines.extend(
        [
            (
                "moment saturation duration [s]: "
                f"{moment_duration:.4f}"
            ),
            (
                "moment saturation time [%]: "
                + percent(
                    moment_fraction
                )
            ),
            (
                "minimum moment scale: "
                f"{min(moment_scale):.6f}"
            ),
            (
                "collective saturation duration [s]: "
                f"{collective_duration:.4f}"
            ),
            (
                "collective saturation time [%]: "
                + percent(
                    collective_fraction
                )
            ),
            (
                "requested collective range [N]: "
                f"{min(requested):.4f} .. "
                f"{max(requested):.4f}"
            ),
            (
                "applied collective range [N]: "
                f"{min(applied):.4f} .. "
                f"{max(applied):.4f}"
            ),
        ]
    )

    return lines


def _message_count(series: dict[str, object]) -> int:
    return int(
        series.get(
            "message_count",
            len(series["times_s"]),
        )
    )


def _finite_count(series: dict[str, object]) -> int:
    return int(
        series.get(
            "finite_count",
            len(series["times_s"]),
        )
    )


def _tracking_lines(
    actual,
    reference,
    components,
) -> list[str]:
    """Report recorded-axis errors after timestamp alignment and yaw wrapping."""
    result = [
        "                         RMS value    max |value|"
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


def _se3_summary(
    layers: dict[str, object],
    pipeline: dict[str, object],
    handoff_mode: str,
    direct_controller: str | None,
) -> list[str]:
    """Order handoff commands and measured downstream PX4 layers for readers."""
    reference_position = layers["reference"]["position"]
    reference_velocity = layers["reference"]["velocity"]
    reference_acceleration = layers["reference"]["acceleration"]

    actual_position = pipeline["position"]["actual"]
    actual_velocity = pipeline["velocity"]["actual"]
    attitude_actual = pipeline["attitude"]["actual"]
    attitude_setpoint = pipeline["attitude"]["setpoint"]
    rate_actual = pipeline["rates"]["actual"]
    rate_setpoint = pipeline["rates"]["setpoint"]
    thrust = pipeline["thrust"]["setpoint"]
    torque = pipeline["torque"]["setpoint"]

    motor_sample_count = max(
        (
            len(times)
            for times, _ in pipeline["motors"].values()
        ),
        default=0,
    )

    if direct_controller is None:
        title = f"SE3 {handoff_mode}-Handoff Pipeline Summary"
    else:
        title = (
            f"SE3 {handoff_mode} / "
            f"{direct_controller} Pipeline Summary"
        )
    lines = [
        title,
        "=" * len(title),
        "",
        "Layer 1 - Generated trajectory reference",
        "----------------------------------------",
        (
            "generated reference messages: "
            f"{_message_count(reference_position)}"
        ),
        (
            "finite position references: "
            f"{_finite_count(reference_position)}"
        ),
        (
            "finite velocity references: "
            f"{_finite_count(reference_velocity)}"
        ),
        (
            "finite acceleration references: "
            f"{_finite_count(reference_acceleration)}"
        ),
        "",
        "Position tracking [m]",
        *_tracking_lines(
            actual_position,
            reference_position,
            [
                ("x / North", "x", False),
                ("y / East", "y", False),
                ("z / Down", "z", False),
            ],
        ),
        "",
        "Velocity tracking [m/s]",
        *_tracking_lines(
            actual_velocity,
            reference_velocity,
            [
                ("vx / North", "x", False),
                ("vy / East", "y", False),
                ("vz / Down", "z", False),
            ],
        ),
        "",
        "Layer 2 - Toolkit handoff",
        "-------------------------",
    ]

    handoff = layers["handoff"]

    if handoff_mode == "acceleration":
        handoff_acceleration = handoff["acceleration"]
        lines.extend(
            [
                (
                    "acceleration handoff messages: "
                    f"{_message_count(handoff_acceleration)}"
                ),
                (
                    "finite acceleration handoff samples: "
                    f"{_finite_count(handoff_acceleration)}"
                ),
                "",
                (
                    "SE3 feedback correction "
                    "(handoff - generated feed-forward) [m/s^2]"
                ),
                *_tracking_lines(
                    handoff_acceleration,
                    reference_acceleration,
                    [
                        ("ax / North", "x", False),
                        ("ay / East", "y", False),
                        ("az / Down", "z", False),
                    ],
                ),
            ]
        )
    elif handoff_mode == "attitude":
        attitude = handoff["attitude"]
        lines.extend(
            [
                f"attitude handoff samples: {len(attitude['times_s'])}",
                "",
                "Attitude handoff tracking [deg]",
                *_tracking_lines(
                    attitude_actual,
                    attitude,
                    [
                        ("roll", "x", False),
                        ("pitch", "y", False),
                        ("yaw", "z", True),
                    ],
                ),
            ]
        )
    elif handoff_mode == "attitude_rate":
        rates = handoff["rates"]
        lines.extend(
            [
                f"body-rate handoff samples: {_message_count(rates)}",
                "",
                "Body-rate handoff tracking [deg/s]",
                *_tracking_lines(
                    rate_actual,
                    rates,
                    [
                        ("p / roll", "x", False),
                        ("q / pitch", "y", False),
                        ("r / yaw", "z", False),
                    ],
                ),
            ]
        )
    else:
        handoff_thrust = handoff["thrust"]
        handoff_torque = handoff["torque"]
        lines.extend(
            [
                (
                    "normalized thrust handoff messages: "
                    f"{_message_count(handoff_thrust)}"
                ),
                (
                    "normalized torque handoff messages: "
                    f"{_message_count(handoff_torque)}"
                ),
            ]
        )

    lines.extend(
        [
            "",
            "PX4 downstream control pipeline",
            "-------------------------------",
            (
                "vehicle_attitude_setpoint samples: "
                f"{len(attitude_setpoint['times_s'])}"
            ),
            (
                "vehicle_rates_setpoint samples: "
                f"{_message_count(rate_setpoint)}"
            ),
            (
                "vehicle_angular_velocity samples: "
                f"{_message_count(rate_actual)}"
            ),
            (
                "vehicle_thrust_setpoint messages: "
                f"{_message_count(thrust)}"
            ),
            (
                "vehicle_torque_setpoint messages: "
                f"{_message_count(torque)}"
            ),
            f"Active motor channels: {len(pipeline['motors'])}",
            f"actuator_motors samples: {motor_sample_count}",
        ]
    )

    return lines


def enrich_summary(
    result,
    bag: BagData,
) -> str:
    """Append Offboard-interval performance metrics to one SE(3) summary.

    Inputs:
        Single-run analysis result and its source bag.

    Method:
        Build the common physical comparison view aligned to confirmed Offboard
        entry. When experiment metadata identifies the configured trajectory,
        also resolve maneuver boundaries for step/yaw/figure-eight metrics.

    Returns:
        Lifecycle/controller summary followed by common performance metrics.
    """
    from .comparison import (
        build_run,
        single_run_summary,
        trajectory_segments,
    )

    handoff = str(
        result.metrics[
            "handoff"
        ]
    )

    direct_controller = str(
        result.metrics[
            "direct_controller"
        ]
    )

    if handoff == "thrust_and_torque":
        label = direct_controller
    else:
        label = handoff

    run = build_run(
        result,
        bag.path,
        label,
    )

    segments = None

    experiment_path = (
        bag.path
        / "experiment.json"
    )

    if experiment_path.is_file():
        experiment = json.loads(
            experiment_path.read_text()
        )

        arguments = experiment.get(
            "arguments",
            {},
        )

        trajectory_name = (
            arguments.get(
                "trajectory"
            )
            if isinstance(
                arguments,
                dict,
            )
            else None
        )

        if (
            isinstance(
                trajectory_name,
                str,
            )
            and trajectory_name
        ):
            root = (
                Path(__file__)
                .resolve()
                .parents[3]
            )

            trajectory_path = (
                root
                / "ros2"
                / "src"
                / "offboard_controllers"
                / "config"
                / "trajectory"
                / "trajectories.yaml"
            )

            segments = (
                trajectory_segments(
                    trajectory_path,
                    trajectory_name,
                )
            )

    performance = (
        single_run_summary(
            run,
            segments,
        )
    )

    return (
        result.summary.rstrip()
        + "\n\n"
        + performance
    )
