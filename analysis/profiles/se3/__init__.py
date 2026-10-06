"""Analysis profile for geometric SE3 trajectory tracking."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import shutil
from pathlib import Path

from analysis.core import (
    BagData,
    TimedSample,
    first_nav_state_time,
    mode_transitions,
    native_constant_names,
)
from analysis.pipeline import (
    CONTROL_PIPELINE_TOPICS,
    _aligned_error_stats,
    _attitude_series,
    _series3,
    analyze_control_pipeline,
)
from analysis.plots import (
    save_mode_timeline,
    save_tracking_plot,
)
from analysis.px4 import TOPICS


PROFILE_NAME = "se3"

STATUS_TOPIC = TOPICS["OUT_VEHICLE_STATUS_V1"]
OFFBOARD_MODE_TOPIC = TOPICS["IN_OFFBOARD_CONTROL_MODE"]

ACCELERATION_HANDOFF_TOPIC = TOPICS["IN_TRAJECTORY_SETPOINT"]
ATTITUDE_HANDOFF_TOPIC = TOPICS["IN_VEHICLE_ATTITUDE_SETPOINT"]
ATTITUDE_RATE_HANDOFF_TOPIC = TOPICS["IN_VEHICLE_RATES_SETPOINT"]
THRUST_HANDOFF_TOPIC = TOPICS["IN_VEHICLE_THRUST_SETPOINT"]
TORQUE_HANDOFF_TOPIC = TOPICS["IN_VEHICLE_TORQUE_SETPOINT"]

HANDOFF_TOPICS = (
    ACCELERATION_HANDOFF_TOPIC,
    ATTITUDE_HANDOFF_TOPIC,
    ATTITUDE_RATE_HANDOFF_TOPIC,
    THRUST_HANDOFF_TOPIC,
    TORQUE_HANDOFF_TOPIC,
)

HANDOFF_MODE_TOPICS = {
    "acceleration": (ACCELERATION_HANDOFF_TOPIC,),
    "attitude": (ATTITUDE_HANDOFF_TOPIC,),
    "attitude_rate": (ATTITUDE_RATE_HANDOFF_TOPIC,),
    "thrust_and_torque": (THRUST_HANDOFF_TOPIC, TORQUE_HANDOFF_TOPIC),
}

OFFBOARD_MODE_FIELDS = {
    "acceleration": "acceleration",
    "attitude": "attitude",
    "attitude_rate": "body_rate",
    "thrust_and_torque": "thrust_and_torque",
}

TRAJECTORY_REFERENCE_TOPIC = (
    "/se3/trajectory_reference"
)

COMMANDED_ACCELERATION_TOPIC = (
    "/se3/diagnostics/commanded_acceleration"
)

FORCE_COMMAND_TOPIC = (
    "/se3/diagnostics/force_command_ned"
)

DESIRED_ATTITUDE_TOPIC = (
    "/se3/diagnostics/desired_attitude"
)

DESIRED_ANGULAR_VELOCITY_TOPIC = (
    "/se3/diagnostics/desired_angular_velocity"
)

BODY_RATE_SETPOINT_TOPIC = (
    "/se3/diagnostics/body_rate_setpoint"
)

ANGULAR_ACCELERATION_FEEDFORWARD_TOPIC = (
    "/se3/diagnostics/angular_acceleration_feedforward"
)

PX4_RATE_RATE_ERROR_TOPIC = (
    "/se3/diagnostics/px4_rate/rate_error"
)

PX4_RATE_PROPORTIONAL_FEEDBACK_TOPIC = (
    "/se3/diagnostics/px4_rate/proportional_feedback"
)

PX4_RATE_INTEGRAL_FEEDBACK_TOPIC = (
    "/se3/diagnostics/px4_rate/integral_feedback"
)

PX4_RATE_DERIVATIVE_FEEDBACK_TOPIC = (
    "/se3/diagnostics/px4_rate/derivative_feedback"
)

PX4_RATE_FEEDFORWARD_TOPIC = (
    "/se3/diagnostics/px4_rate/feedforward"
)

PX4_RATE_UNFILTERED_TORQUE_TOPIC = (
    "/se3/diagnostics/px4_rate/unfiltered_torque"
)

PX4_RATE_INTEGRATOR_STATE_TOPIC = (
    "/se3/diagnostics/px4_rate/integrator_state"
)

GEOMETRIC_NORMALIZED_ATTITUDE_FEEDBACK_TOPIC = (
    "/se3/diagnostics/geometric_normalized/"
    "torque_attitude_feedback"
)

GEOMETRIC_NORMALIZED_ANGULAR_VELOCITY_FEEDBACK_TOPIC = (
    "/se3/diagnostics/geometric_normalized/"
    "torque_angular_velocity_feedback"
)

GEOMETRIC_NORMALIZED_ANGULAR_ACCELERATION_FEEDFORWARD_TOPIC = (
    "/se3/diagnostics/geometric_normalized/"
    "torque_angular_acceleration_feedforward"
)

GEOMETRIC_PHYSICAL_REQUESTED_MOMENT_TOPIC = (
    "/se3/diagnostics/geometric_physical/requested_moment_nm"
)

GEOMETRIC_PHYSICAL_APPLIED_MOMENT_TOPIC = (
    "/se3/diagnostics/geometric_physical/applied_moment_nm"
)

GEOMETRIC_PHYSICAL_MOMENT_SCALE_TOPIC = (
    "/se3/diagnostics/geometric_physical/moment_scale"
)

GEOMETRIC_PHYSICAL_REQUESTED_COLLECTIVE_THRUST_TOPIC = (
    "/se3/diagnostics/geometric_physical/requested_collective_thrust_n"
)

GEOMETRIC_PHYSICAL_APPLIED_COLLECTIVE_THRUST_TOPIC = (
    "/se3/diagnostics/geometric_physical/applied_collective_thrust_n"
)

DIAGNOSTIC_TOPICS = {
    COMMANDED_ACCELERATION_TOPIC,
    FORCE_COMMAND_TOPIC,
    DESIRED_ATTITUDE_TOPIC,
    DESIRED_ANGULAR_VELOCITY_TOPIC,
    BODY_RATE_SETPOINT_TOPIC,
    ANGULAR_ACCELERATION_FEEDFORWARD_TOPIC,
    PX4_RATE_RATE_ERROR_TOPIC,
    PX4_RATE_PROPORTIONAL_FEEDBACK_TOPIC,
    PX4_RATE_INTEGRAL_FEEDBACK_TOPIC,
    PX4_RATE_DERIVATIVE_FEEDBACK_TOPIC,
    PX4_RATE_FEEDFORWARD_TOPIC,
    PX4_RATE_UNFILTERED_TORQUE_TOPIC,
    PX4_RATE_INTEGRATOR_STATE_TOPIC,
    GEOMETRIC_NORMALIZED_ATTITUDE_FEEDBACK_TOPIC,
    GEOMETRIC_NORMALIZED_ANGULAR_VELOCITY_FEEDBACK_TOPIC,
    GEOMETRIC_NORMALIZED_ANGULAR_ACCELERATION_FEEDFORWARD_TOPIC,
    GEOMETRIC_PHYSICAL_REQUESTED_MOMENT_TOPIC,
    GEOMETRIC_PHYSICAL_APPLIED_MOMENT_TOPIC,
    GEOMETRIC_PHYSICAL_MOMENT_SCALE_TOPIC,
    GEOMETRIC_PHYSICAL_REQUESTED_COLLECTIVE_THRUST_TOPIC,
    GEOMETRIC_PHYSICAL_APPLIED_COLLECTIVE_THRUST_TOPIC,
}

REQUIRED_TOPICS = {
    STATUS_TOPIC,
    OFFBOARD_MODE_TOPIC,
    *HANDOFF_TOPICS,
    TRAJECTORY_REFERENCE_TOPIC,
    *CONTROL_PIPELINE_TOPICS,
}

OPTIONAL_TOPICS = DIAGNOSTIC_TOPICS


DIRECT_CONTROLLERS = {
    "geometric_normalized",
    "px4_rate",
    "geometric_physical",
}


@dataclass
class AnalysisResult:
    summary: str
    metrics: dict[str, float | bool | str]
    plot_data: dict[str, object]


def _required(
    bag: BagData,
    topic: str,
) -> list[TimedSample]:
    samples = bag.samples[topic]

    if not samples:
        raise RuntimeError(
            f"No samples recorded on {topic}"
        )

    return samples


def _handoff_samples(
    bag: BagData,
    offboard_mode: list[TimedSample],
) -> tuple[str, list[tuple[str, list[TimedSample]]]]:
    active_modes = set()

    for sample in offboard_mode:
        active = [
            mode
            for mode, field in OFFBOARD_MODE_FIELDS.items()
            if bool(getattr(sample.message, field, False))
        ]

        if len(active) > 1:
            raise RuntimeError(
                "SE3 OffboardControlMode enables multiple handoff levels"
            )

        if active:
            active_modes.add(active[0])

    if not active_modes:
        raise RuntimeError(
            "SE3 OffboardControlMode does not identify a handoff level"
        )

    if len(active_modes) > 1:
        raise RuntimeError(
            "SE3 bag contains multiple Offboard handoff modes"
        )

    handoff_mode = next(iter(active_modes))
    inputs = []

    for topic in HANDOFF_MODE_TOPICS[handoff_mode]:
        samples = bag.samples.get(topic, [])

        if not samples:
            raise RuntimeError(
                f"No SE3 {handoff_mode} handoff samples recorded on {topic}"
            )

        inputs.append((topic, samples))

    return handoff_mode, inputs


def _first_armed_time(
    samples: list[TimedSample],
    armed_state: int,
) -> int | None:
    for sample in samples:
        if int(sample.message.arming_state) == armed_state:
            return sample.timestamp_ns

    return None


def _direct_controller_name(
    bag: BagData,
    handoff_mode: str,
) -> str | None:
    """Identify the direct controller without guessing from PX4 outputs."""
    if handoff_mode != "thrust_and_torque":
        return None

    metadata_path = bag.path / "experiment.json"

    if metadata_path.is_file():
        try:
            metadata = json.loads(
                metadata_path.read_text()
            )
        except json.JSONDecodeError as exc:
            raise RuntimeError(
                f"Invalid experiment metadata JSON: {metadata_path}"
            ) from exc

        if not isinstance(metadata, dict):
            raise RuntimeError(
                f"Experiment metadata must be a JSON object: {metadata_path}"
            )

        arguments = metadata.get("arguments", {})

        if not isinstance(arguments, dict):
            raise RuntimeError(
                f"Experiment arguments must be a JSON object: {metadata_path}"
            )

        metadata_handoff = arguments.get("handoff")

        if (
            metadata_handoff is not None
            and metadata_handoff != handoff_mode
        ):
            raise RuntimeError(
                "SE3 sequence metadata handoff does not match "
                "the recorded OffboardControlMode."
            )

        direct_controller = arguments.get(
            "direct_controller"
        )

        if direct_controller is not None:
            if direct_controller not in DIRECT_CONTROLLERS:
                raise RuntimeError(
                    "Unknown SE3 direct controller in experiment metadata: "
                    f"{direct_controller}"
                )

            return str(direct_controller)

    # Older geometric-normalized bags can still be identified from their
    # controller-owned diagnostic topics. px4_rate and geometric_physical cannot
    # be distinguished reliably from native PX4 wrench topics alone.
    if any(
        bag.samples.get(topic)
        for topic in (
            GEOMETRIC_NORMALIZED_ATTITUDE_FEEDBACK_TOPIC,
            GEOMETRIC_NORMALIZED_ANGULAR_VELOCITY_FEEDBACK_TOPIC,
            GEOMETRIC_NORMALIZED_ANGULAR_ACCELERATION_FEEDFORWARD_TOPIC,
        )
    ):
        return "geometric_normalized"

    return "unknown"


def extract_se3_layers(
    bag: BagData,
    handoff_mode: str = "acceleration",
) -> dict[str, object]:
    """Extract toolkit-owned trajectory and selected handoff signals."""

    trajectory_samples = bag.samples[TRAJECTORY_REFERENCE_TOPIC]

    reference = {
        "position": _series3(
            bag,
            TRAJECTORY_REFERENCE_TOPIC,
            lambda msg: msg.position,
        ),
        "velocity": _series3(
            bag,
            TRAJECTORY_REFERENCE_TOPIC,
            lambda msg: msg.velocity,
        ),
        "acceleration": _series3(
            bag,
            TRAJECTORY_REFERENCE_TOPIC,
            lambda msg: msg.acceleration,
        ),
        "yaw": _series3(
            bag,
            TRAJECTORY_REFERENCE_TOPIC,
            lambda msg: (
                0.0,
                0.0,
                msg.yaw,
            ),
            scale=(
                180.0
                / 3.14159265358979323846
            ),
        ),
        "jerk": None,
    }

    if any(
        hasattr(sample.message, "jerk")
        for sample in trajectory_samples
    ):
        reference["jerk"] = _series3(
            bag,
            TRAJECTORY_REFERENCE_TOPIC,
            lambda msg: msg.jerk,
        )

    if handoff_mode == "acceleration":
        handoff = {
            "acceleration": _series3(
                bag,
                ACCELERATION_HANDOFF_TOPIC,
                lambda msg: msg.acceleration,
            ),
        }

    elif handoff_mode == "attitude":
        handoff = {
            "attitude": _attitude_series(
                bag,
                ATTITUDE_HANDOFF_TOPIC,
                "q_d",
            ),
        }

    elif handoff_mode == "attitude_rate":
        handoff = {
            "rates": _series3(
                bag,
                ATTITUDE_RATE_HANDOFF_TOPIC,
                lambda msg: (msg.roll, msg.pitch, msg.yaw),
                scale=180.0 / 3.14159265358979323846,
            ),
        }

    elif handoff_mode == "thrust_and_torque":
        handoff = {
            "thrust": _series3(
                bag,
                THRUST_HANDOFF_TOPIC,
                lambda msg: msg.xyz,
            ),
            "torque": _series3(
                bag,
                TORQUE_HANDOFF_TOPIC,
                lambda msg: msg.xyz,
            ),
        }

    else:
        raise RuntimeError(
            f"Unsupported SE3 handoff mode: {handoff_mode}"
        )

    return {
        "reference": reference,
        "handoff": handoff,
    }


def _vector3_diagnostic_series(
    bag: BagData,
    topic: str,
    *,
    scale: float = 1.0,
) -> dict[str, object]:
    return _series3(
        bag,
        topic,
        lambda msg: (
            msg.x,
            msg.y,
            msg.z,
        ),
        scale=scale,
    )


def extract_common_diagnostics(
    bag: BagData,
) -> dict[str, object]:
    """Extract optional controller-wide SE3 internal signals."""
    diagnostics = {}

    vector_topics = {
        "commanded_acceleration":
            COMMANDED_ACCELERATION_TOPIC,
        "force_command":
            FORCE_COMMAND_TOPIC,
    }

    for name, topic in vector_topics.items():
        if bag.samples.get(topic):
            diagnostics[name] = (
                _vector3_diagnostic_series(
                    bag,
                    topic,
                )
            )

    if bag.samples.get(
        DESIRED_ATTITUDE_TOPIC
    ):
        diagnostics["desired_attitude"] = (
            _attitude_series(
                bag,
                DESIRED_ATTITUDE_TOPIC,
                "q_d",
            )
        )

    degrees = (
        180.0 /
        3.14159265358979323846
    )

    if bag.samples.get(
        DESIRED_ANGULAR_VELOCITY_TOPIC
    ):
        diagnostics[
            "desired_angular_velocity"
        ] = _vector3_diagnostic_series(
            bag,
            DESIRED_ANGULAR_VELOCITY_TOPIC,
            scale=degrees,
        )

    if bag.samples.get(
        BODY_RATE_SETPOINT_TOPIC
    ):
        diagnostics[
            "body_rate_setpoint"
        ] = _vector3_diagnostic_series(
            bag,
            BODY_RATE_SETPOINT_TOPIC,
            scale=degrees,
        )

    if bag.samples.get(
        ANGULAR_ACCELERATION_FEEDFORWARD_TOPIC
    ):
        diagnostics[
            "angular_acceleration_feedforward"
        ] = _vector3_diagnostic_series(
            bag,
            ANGULAR_ACCELERATION_FEEDFORWARD_TOPIC,
            scale=degrees,
        )

        angular_velocity_topic = TOPICS[
            "OUT_VEHICLE_ANGULAR_VELOCITY"
        ]

        if bag.samples.get(
            angular_velocity_topic
        ):
            diagnostics[
                "measured_angular_acceleration"
            ] = _series3(
                bag,
                angular_velocity_topic,
                lambda msg: msg.xyz_derivative,
                scale=degrees,
            )

    return diagnostics


def extract_geometric_normalized_diagnostics(
    bag: BagData,
) -> dict[str, object]:
    """Extract optional normalized geometric-controller terms."""
    diagnostics = {}

    topics = {
        "attitude_feedback":
            GEOMETRIC_NORMALIZED_ATTITUDE_FEEDBACK_TOPIC,
        "angular_velocity_feedback":
            GEOMETRIC_NORMALIZED_ANGULAR_VELOCITY_FEEDBACK_TOPIC,
        "angular_acceleration_feedforward":
            GEOMETRIC_NORMALIZED_ANGULAR_ACCELERATION_FEEDFORWARD_TOPIC,
    }

    for name, topic in topics.items():
        if bag.samples.get(topic):
            diagnostics[name] = _series3(
                bag,
                topic,
                lambda msg: msg.xyz,
            )

    return diagnostics


def extract_px4_rate_diagnostics(
    bag: BagData,
) -> dict[str, object]:
    """Extract optional reproduced PX4 rate-loop internals."""
    diagnostics = {}

    degrees = (
        180.0 /
        3.14159265358979323846
    )

    topics = {
        "proportional_feedback":
            PX4_RATE_PROPORTIONAL_FEEDBACK_TOPIC,
        "integral_feedback":
            PX4_RATE_INTEGRAL_FEEDBACK_TOPIC,
        "derivative_feedback":
            PX4_RATE_DERIVATIVE_FEEDBACK_TOPIC,
        "feedforward":
            PX4_RATE_FEEDFORWARD_TOPIC,
        "unfiltered_torque":
            PX4_RATE_UNFILTERED_TORQUE_TOPIC,
        "integrator_state":
            PX4_RATE_INTEGRATOR_STATE_TOPIC,
    }

    if bag.samples.get(
        PX4_RATE_RATE_ERROR_TOPIC
    ):
        diagnostics["rate_error"] = (
            _vector3_diagnostic_series(
                bag,
                PX4_RATE_RATE_ERROR_TOPIC,
                scale=degrees,
            )
        )

    for name, topic in topics.items():
        if bag.samples.get(topic):
            diagnostics[name] = (
                _vector3_diagnostic_series(
                    bag,
                    topic,
                )
            )

    return diagnostics


def _scalar_series(
    bag: BagData,
    topic: str,
) -> dict[str, object]:
    samples = bag.samples.get(topic, [])

    values = [
        float(sample.message.data)
        for sample in samples
    ]

    return {
        "times_s": [
            bag.relative_seconds(
                sample.timestamp_ns
            )
            for sample in samples
        ],
        "values": values,
        "message_count": len(values),
        "finite_count": sum(
            math.isfinite(value)
            for value in values
        ),
    }


def extract_geometric_physical_diagnostics(
    bag: BagData,
) -> dict[str, object]:
    """Extract optional Geometric physical-wrench and feasibility signals."""
    diagnostics = {}

    vector_topics = {
        "requested_moment":
            GEOMETRIC_PHYSICAL_REQUESTED_MOMENT_TOPIC,
        "applied_moment":
            GEOMETRIC_PHYSICAL_APPLIED_MOMENT_TOPIC,
    }

    for name, topic in vector_topics.items():
        if bag.samples.get(topic):
            diagnostics[name] = (
                _vector3_diagnostic_series(
                    bag,
                    topic,
                )
            )

    scalar_topics = {
        "moment_scale":
            GEOMETRIC_PHYSICAL_MOMENT_SCALE_TOPIC,
        "requested_collective_thrust":
            GEOMETRIC_PHYSICAL_REQUESTED_COLLECTIVE_THRUST_TOPIC,
        "applied_collective_thrust":
            GEOMETRIC_PHYSICAL_APPLIED_COLLECTIVE_THRUST_TOPIC,
    }

    for name, topic in scalar_topics.items():
        if bag.samples.get(topic):
            diagnostics[name] = _scalar_series(
                bag,
                topic,
            )

    return diagnostics


def _boolean_activity_duration(
    times_s: list[float],
    active: list[bool],
) -> tuple[float, float]:
    """Return trapezoidal active duration and fraction."""
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


def analyze(bag: BagData) -> AnalysisResult:
    """Analyze the selected SE3 handoff through the PX4 control pipeline."""

    status = _required(bag, STATUS_TOPIC)
    offboard_mode = _required(bag, OFFBOARD_MODE_TOPIC)
    handoff_mode, handoff_inputs = _handoff_samples(
        bag,
        offboard_mode,
    )

    direct_controller = _direct_controller_name(
        bag,
        handoff_mode,
    )

    handoff_topics = [topic for topic, _ in handoff_inputs]
    handoff_topic = " + ".join(handoff_topics)
    reference = _required(
        bag,
        TRAJECTORY_REFERENCE_TOPIC,
    )

    status_type = type(status[0].message)

    armed_state = int(
        status_type.ARMING_STATE_ARMED
    )
    offboard_state = int(
        status_type.NAVIGATION_STATE_OFFBOARD
    )

    position_state = int(
        status_type.NAVIGATION_STATE_POSCTL
    )

    started_armed = (
        int(status[0].message.arming_state)
        == armed_state
    )

    armed_ns = _first_armed_time(
        status,
        armed_state,
    )

    if armed_ns is None:
        raise RuntimeError(
            "Armed state was not recorded"
        )

    offboard_ns = first_nav_state_time(
        status,
        offboard_state,
    )

    if offboard_ns is None:
        raise RuntimeError(
            "Offboard-mode entry was not recorded"
        )

    position_return_ns = first_nav_state_time(
        status,
        position_state,
        not_before_ns=offboard_ns,
    )

    if position_return_ns is None:
        raise RuntimeError(
            "Position-mode return after Offboard was not recorded"
        )

    offboard_mode_start_ns = offboard_mode[0].timestamp_ns
    reference_start_ns = reference[0].timestamp_ns
    handoff_start_ns = min(
        samples[0].timestamp_ns
        for _, samples in handoff_inputs
    )

    if offboard_mode_start_ns >= offboard_ns:
        raise RuntimeError(
            "OffboardControlMode prestream did not precede "
            "Offboard entry"
        )

    if reference_start_ns >= offboard_ns:
        raise RuntimeError(
            "Generated trajectory-reference prestream did not "
            "precede Offboard entry"
        )

    metrics = {
        "bag_duration_s": bag.duration_s,
        "handoff": handoff_mode,
        "handoff_topic": handoff_topic,
        "direct_controller": direct_controller or "n/a",
        "started_armed": started_armed,
        "armed_s": bag.relative_seconds(armed_ns),
        "offboard_mode_start_s": bag.relative_seconds(
            offboard_mode_start_ns
        ),
        "reference_start_s": bag.relative_seconds(
            reference_start_ns
        ),
        "offboard_entry_s": bag.relative_seconds(
            offboard_ns
        ),
        "handoff_start_s": bag.relative_seconds(
            handoff_start_ns
        ),
        "position_return_s": bag.relative_seconds(
            position_return_ns
        ),
        "offboard_mode_prestream_s": (
            offboard_ns - offboard_mode_start_ns
        ) / 1e9,
        "reference_prestream_s": (
            offboard_ns - reference_start_ns
        ) / 1e9,
        "handoff_offset_from_offboard_s": (
            handoff_start_ns - offboard_ns
        ) / 1e9,
        "offboard_duration_s": (
            position_return_ns - offboard_ns
        ) / 1e9,
    }

    state_names = native_constant_names(
        status_type,
        "NAVIGATION_STATE_",
    )

    transitions = mode_transitions(status)

    # Controller-pipeline metrics begin only after PX4 actually enters
    # Offboard. Lifecycle analysis above still uses the complete bag.
    tracking_bag = BagData(
        path=bag.path,
        start_ns=bag.start_ns,
        end_ns=bag.end_ns,
        samples={
            topic: [
                sample
                for sample in samples
                if sample.timestamp_ns >= offboard_ns
            ]
            for topic, samples in bag.samples.items()
        },
    )

    layers = extract_se3_layers(
        tracking_bag,
        handoff_mode,
    )

    # Keep the native PX4 pipeline on its native topics. In acceleration
    # handoff, OUT_TRAJECTORY_SETPOINT is not the toolkit's generated
    # geometric trajectory reference.
    pipeline = analyze_control_pipeline(
        tracking_bag
    )

    diagnostics = {
        "common": extract_common_diagnostics(
            tracking_bag
        ),
        "geometric_normalized":
            extract_geometric_normalized_diagnostics(
                tracking_bag
            ),
        "px4_rate":
            extract_px4_rate_diagnostics(
                tracking_bag
            ),
        "geometric_physical":
            extract_geometric_physical_diagnostics(
                tracking_bag
            ),
    }

    diagnostics_present = any(
        tracking_bag.samples.get(topic)
        for topic in DIAGNOSTIC_TOPICS
    )

    lines = [
        f"Profile: {PROFILE_NAME}",
        f"Bag: {bag.path}",
        f"Bag duration: {bag.duration_s:.3f} s",
        "",
        "PX4 navigation-state transitions:",
    ]

    for timestamp_ns, state in transitions:
        lines.append(
            f"  {bag.relative_seconds(timestamp_ns):8.3f} s  "
            f"{state_names.get(state, f'STATE_{state}')} ({state})"
        )

    lines.extend(
        [
            "",
            "SE3 Offboard lifecycle:",
            f"  handoff input: {handoff_topic}",
            *(
                [f"  direct controller: {direct_controller}"]
                if direct_controller is not None
                else []
            ),
            f"  started armed: "
            f"{'yes' if started_armed else 'no'}",
            f"  armed state first recorded: "
            f"{metrics['armed_s']:.3f} s",
            f"  OffboardControlMode start: "
            f"{metrics['offboard_mode_start_s']:.3f} s",
            f"  generated reference start: "
            f"{metrics['reference_start_s']:.3f} s",
            f"  Offboard entry: "
            f"{metrics['offboard_entry_s']:.3f} s",
            f"  OffboardControlMode before entry: "
            f"{metrics['offboard_mode_prestream_s']:.3f} s",
            f"  generated reference before entry: "
            f"{metrics['reference_prestream_s']:.3f} s",
            f"  selected handoff start: "
            f"{metrics['handoff_start_s']:.3f} s",
            f"  handoff offset from Offboard entry: "
            f"{metrics['handoff_offset_from_offboard_s']:+.3f} s",
            f"  Position return: "
            f"{metrics['position_return_s']:.3f} s",
            f"  time in Offboard: "
            f"{metrics['offboard_duration_s']:.3f} s",
            "",
            *_se3_summary(
                layers,
                pipeline,
                handoff_mode,
                direct_controller,
            ),
        ]
    )

    lines.extend(
        [
            "",
            (
                "Internal diagnostics: "
                + (
                    "recorded"
                    if diagnostics_present
                    else "not recorded"
                )
            ),
        ]
    )

    if direct_controller == "geometric_physical":
        lines.extend(
            [
                "",
                *_geometric_physical_summary(
                    diagnostics["geometric_physical"]
                ),
            ]
        )

    markers = [
        (
            metrics["reference_start_s"],
            "REFERENCE PRESTREAM",
        ),
        (
            metrics["offboard_entry_s"],
            "OFFBOARD ENTRY",
        ),
        (
            metrics["handoff_start_s"],
            "HANDOFF START",
        ),
        (
            metrics["position_return_s"],
            "POSITION RETURN",
        ),
    ]

    if not started_armed:
        markers.insert(
            0,
            (
                metrics["armed_s"],
                "ARMED",
            ),
        )

    return AnalysisResult(
        summary="\n".join(lines) + "\n",
        metrics=metrics,
        plot_data={
            "layers": layers,
            "pipeline": pipeline,
            "diagnostics": diagnostics,
            "diagnostics_present":
                diagnostics_present,
            "handoff_mode": handoff_mode,
            "markers": markers,
            "state_names": state_names,
            "transitions": [
                (
                    bag.relative_seconds(timestamp_ns),
                    state,
                )
                for timestamp_ns, state in transitions
            ],
        },
    )


def _vector_components(
    series_by_label,
    component_names,
):
    components = []

    for component_name, key in component_names:
        signals = {}

        for label, series in series_by_label.items():
            if series["times_s"]:
                signals[label] = (
                    series["times_s"],
                    series[key],
                )

        if signals:
            components.append(
                (component_name, signals)
            )

    return components


def write_plots(
    result: AnalysisResult,
    output_dir: Path,
) -> list[Path]:
    """Write general SE3 analysis plus optional controller diagnostics."""
    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    for old_plot in output_dir.glob("*.png"):
        old_plot.unlink()

    diagnostics_dir = (
        output_dir / "diagnostics"
    )

    if diagnostics_dir.exists():
        shutil.rmtree(
            diagnostics_dir
        )

    pipeline = result.plot_data["pipeline"]
    layers = result.plot_data["layers"]
    diagnostics = result.plot_data["diagnostics"]

    handoff_mode = str(
        result.metrics["handoff"]
    )

    direct_controller = str(
        result.metrics["direct_controller"]
    )

    origin_s = float(
        result.metrics["offboard_entry_s"]
    )

    time_label = (
        "Time from Offboard entry [s]"
    )

    generated = []

    def save_vector(
        directory: Path,
        filename: str,
        signals,
        components,
        title: str,
        unit: str,
    ) -> None:
        path = directory / filename

        save_tracking_plot(
            path,
            components=_vector_components(
                signals,
                components,
            ),
            title=title,
            unit=unit,
            time_origin_s=origin_s,
            xlabel=time_label,
        )

        if path.exists():
            generated.append(path)

    # --------------------------------------------------------
    # General analysis: only signals that are actually part of
    # the operational experiment/control path.
    # --------------------------------------------------------

    save_vector(
        output_dir,
        "01_position_tracking.png",
        {
            "measured":
                pipeline["position"]["actual"],
            "trajectory":
                layers["reference"]["position"],
        },
        [
            ("North", "x"),
            ("East", "y"),
            ("Down", "z"),
        ],
        "Position Tracking — NED",
        "[m]",
    )

    save_vector(
        output_dir,
        "02_velocity_tracking.png",
        {
            "measured":
                pipeline["velocity"]["actual"],
            "trajectory":
                layers["reference"]["velocity"],
        },
        [
            ("North", "x"),
            ("East", "y"),
            ("Down", "z"),
        ],
        "Velocity Tracking — NED",
        "[m/s]",
    )

    if handoff_mode == "acceleration":
        save_vector(
            output_dir,
            "03_acceleration_tracking.png",
            {
                "measured":
                    pipeline["acceleration"]["actual"],
                "command":
                    layers["handoff"]["acceleration"],
            },
            [
                ("North", "x"),
                ("East", "y"),
                ("Down", "z"),
            ],
            "Acceleration Handoff Tracking — NED",
            "[m/s²]",
        )

    if handoff_mode == "acceleration":
        attitude_reference = (
            pipeline["attitude"]["setpoint"]
        )
        attitude_label = "setpoint"
        attitude_title = (
            "Attitude Tracking — PX4 Generated Setpoint"
        )

    elif handoff_mode == "attitude":
        attitude_reference = (
            layers["handoff"]["attitude"]
        )
        attitude_label = "command"
        attitude_title = (
            "Attitude Tracking — SE3 Handoff"
        )

    else:
        attitude_reference = None

    if attitude_reference is not None:
        save_vector(
            output_dir,
            "04_attitude_tracking.png",
            {
                "measured":
                    pipeline["attitude"]["actual"],
                attitude_label:
                    attitude_reference,
            },
            [
                ("Roll", "x"),
                ("Pitch", "y"),
                ("Yaw", "z"),
            ],
            attitude_title,
            "[deg]",
        )

    if handoff_mode in {
        "acceleration",
        "attitude",
    }:
        rate_reference = (
            pipeline["rates"]["setpoint"]
        )
        rate_label = "setpoint"
        rate_title = (
            "Body-Rate Tracking — PX4 Rate Setpoint"
        )

    elif handoff_mode == "attitude_rate":
        rate_reference = (
            layers["handoff"]["rates"]
        )
        rate_label = "command"
        rate_title = (
            "Body-Rate Tracking — SE3 Handoff"
        )

    else:
        rate_reference = None

    if rate_reference is not None:
        save_vector(
            output_dir,
            "05_body_rate_tracking.png",
            {
                "measured":
                    pipeline["rates"]["actual"],
                rate_label:
                    rate_reference,
            },
            [
                ("p", "x"),
                ("q", "y"),
                ("r", "z"),
            ],
            rate_title + " — FRD",
            "[deg/s]",
        )

    save_vector(
        output_dir,
        "06_thrust_command.png",
        {
            "command":
                pipeline["thrust"]["setpoint"],
        },
        [
            ("x", "x"),
            ("y", "y"),
            ("z", "z"),
        ],
        "Normalized Thrust Command — FRD",
        "[normalized]",
    )

    save_vector(
        output_dir,
        "07_torque_command.png",
        {
            "command":
                pipeline["torque"]["setpoint"],
        },
        [
            ("Roll", "x"),
            ("Pitch", "y"),
            ("Yaw", "z"),
        ],
        "Normalized Torque Command — FRD",
        "[normalized]",
    )

    if pipeline["motors"]:
        motor_path = (
            output_dir
            / "08_motor_commands.png"
        )

        save_tracking_plot(
            motor_path,
            components=[
                (
                    "Motor command",
                    pipeline["motors"],
                ),
            ],
            title=(
                "PX4 Control Allocation — Motor Commands"
            ),
            unit="[normalized]",
            time_origin_s=origin_s,
            xlabel=time_label,
        )

        if motor_path.exists():
            generated.append(
                motor_path
            )

    mode_path = (
        output_dir
        / "09_mode_timeline.png"
    )

    save_mode_timeline(
        mode_path,
        transitions=result.plot_data[
            "transitions"
        ],
        state_names=result.plot_data[
            "state_names"
        ],
    )

    generated.append(
        mode_path
    )

    # --------------------------------------------------------
    # Optional diagnostics. No directory is created when the
    # experiment did not publish diagnostic instrumentation.
    # --------------------------------------------------------

    if not result.plot_data[
        "diagnostics_present"
    ]:
        return generated

    diagnostics_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    common = diagnostics["common"]

    commanded_acceleration = common.get(
        "commanded_acceleration"
    )

    if commanded_acceleration:
        save_vector(
            diagnostics_dir,
            "01_translational_controller.png",
            {
                "measured":
                    pipeline["acceleration"]["actual"],
                "command":
                    commanded_acceleration,
                "trajectory":
                    layers["reference"]["acceleration"],
            },
            [
                ("North", "x"),
                ("East", "y"),
                ("Down", "z"),
            ],
            "SE3 Translational Controller — Acceleration",
            "[m/s²]",
        )

    force_command = common.get(
        "force_command"
    )

    if force_command:
        save_vector(
            diagnostics_dir,
            "02_force_command.png",
            {
                "command": force_command,
            },
            [
                ("North", "x"),
                ("East", "y"),
                ("Down", "z"),
            ],
            "SE3 Force Command — NED",
            "[N]",
        )

    desired_attitude = common.get(
        "desired_attitude"
    )

    if (
        desired_attitude
        and handoff_mode != "attitude"
    ):
        save_vector(
            diagnostics_dir,
            "03_desired_attitude.png",
            {
                "measured":
                    pipeline["attitude"]["actual"],
                "desired":
                    desired_attitude,
            },
            [
                ("Roll", "x"),
                ("Pitch", "y"),
                ("Yaw", "z"),
            ],
            "SE3 Desired Attitude Diagnostic",
            "[deg]",
        )

    desired_angular_velocity = common.get(
        "desired_angular_velocity"
    )

    if desired_angular_velocity:
        save_vector(
            diagnostics_dir,
            "04_desired_angular_velocity.png",
            {
                "measured":
                    pipeline["rates"]["actual"],
                "desired":
                    desired_angular_velocity,
            },
            [
                ("p", "x"),
                ("q", "y"),
                ("r", "z"),
            ],
            (
                "SE3 Desired Angular Velocity "
                "vs Vehicle Body Rate — FRD"
            ),
            "[deg/s]",
        )

    body_rate_setpoint = common.get(
        "body_rate_setpoint"
    )

    if (
        body_rate_setpoint
        and handoff_mode != "attitude_rate"
    ):
        save_vector(
            diagnostics_dir,
            "05_body_rate_setpoint.png",
            {
                "measured":
                    pipeline["rates"]["actual"],
                "command":
                    body_rate_setpoint,
            },
            [
                ("p", "x"),
                ("q", "y"),
                ("r", "z"),
            ],
            "SE3 Body-Rate Setpoint Tracking — FRD",
            "[deg/s]",
        )

    angular_acceleration_feedforward = (
        common.get(
            "angular_acceleration_feedforward"
        )
    )

    measured_angular_acceleration = (
        common.get(
            "measured_angular_acceleration"
        )
    )

    if angular_acceleration_feedforward:
        angular_acceleration_signals = {
            "desired":
                angular_acceleration_feedforward,
        }

        if measured_angular_acceleration:
            angular_acceleration_signals[
                "measured"
            ] = measured_angular_acceleration

        save_vector(
            diagnostics_dir,
            "06_angular_acceleration.png",
            angular_acceleration_signals,
            [
                ("p dot", "x"),
                ("q dot", "y"),
                ("r dot", "z"),
            ],
            "Angular-Acceleration Reference — FRD",
            "[deg/s²]",
        )

    geometric = diagnostics[
        "geometric_normalized"
    ]

    if geometric:
        terms = {}

        for label, key in (
            (
                "attitude feedback",
                "attitude_feedback",
            ),
            (
                "rate feedback",
                "angular_velocity_feedback",
            ),
            (
                "acceleration feed-forward",
                "angular_acceleration_feedforward",
            ),
        ):
            if key in geometric:
                terms[label] = geometric[key]

        if "torque" in layers["handoff"]:
            terms["command"] = (
                layers["handoff"]["torque"]
            )

        save_vector(
            diagnostics_dir,
            "10_geometric_torque_terms.png",
            terms,
            [
                ("Roll", "x"),
                ("Pitch", "y"),
                ("Yaw", "z"),
            ],
            (
                "Geometric-Normalized "
                "Torque Decomposition"
            ),
            "[normalized]",
        )

    px4_rate = diagnostics["px4_rate"]

    if "rate_error" in px4_rate:
        save_vector(
            diagnostics_dir,
            "10_px4_rate_error.png",
            {
                "error":
                    px4_rate["rate_error"],
            },
            [
                ("p", "x"),
                ("q", "y"),
                ("r", "z"),
            ],
            "Reproduced PX4 Rate Error — FRD",
            "[deg/s]",
        )

    px4_terms = {}

    for label, key in (
        ("P", "proportional_feedback"),
        ("I", "integral_feedback"),
        ("D", "derivative_feedback"),
        ("FF", "feedforward"),
        ("unfiltered", "unfiltered_torque"),
    ):
        if key in px4_rate:
            px4_terms[label] = px4_rate[key]

    if px4_terms:
        save_vector(
            diagnostics_dir,
            "11_px4_rate_terms.png",
            px4_terms,
            [
                ("Roll", "x"),
                ("Pitch", "y"),
                ("Yaw", "z"),
            ],
            "Reproduced PX4 Rate-Controller Terms",
            "[normalized]",
        )

    if "integrator_state" in px4_rate:
        save_vector(
            diagnostics_dir,
            "12_px4_rate_integrator.png",
            {
                "integrator":
                    px4_rate[
                        "integrator_state"
                    ],
            },
            [
                ("Roll", "x"),
                ("Pitch", "y"),
                ("Yaw", "z"),
            ],
            "Reproduced PX4 Rate Integrator",
            "[normalized]",
        )

    physical = diagnostics["geometric_physical"]

    if (
        "requested_moment" in physical
        and "applied_moment" in physical
    ):
        save_vector(
            diagnostics_dir,
            "10_geometric_physical_moment_feasibility.png",
            {
                "requested":
                    physical["requested_moment"],
                "applied":
                    physical["applied_moment"],
            },
            [
                ("Roll", "x"),
                ("Pitch", "y"),
                ("Yaw", "z"),
            ],
            "Geometric Physical Moment — Requested vs Applied",
            "[N m]",
        )

    requested_collective = physical.get(
        "requested_collective_thrust"
    )
    applied_collective = physical.get(
        "applied_collective_thrust"
    )

    if (
        requested_collective
        and applied_collective
    ):
        collective_path = (
            diagnostics_dir
            / "11_geometric_physical_collective_feasibility.png"
        )

        save_tracking_plot(
            collective_path,
            components=[
                (
                    "Collective thrust",
                    {
                        "requested": (
                            requested_collective[
                                "times_s"
                            ],
                            requested_collective[
                                "values"
                            ],
                        ),
                        "applied": (
                            applied_collective[
                                "times_s"
                            ],
                            applied_collective[
                                "values"
                            ],
                        ),
                    },
                ),
            ],
            title=(
                "Geometric Physical Collective Thrust — "
                "Requested vs Applied"
            ),
            unit="[N]",
            time_origin_s=origin_s,
            xlabel=time_label,
        )

        generated.append(
            collective_path
        )

    moment_scale = physical.get(
        "moment_scale"
    )

    if moment_scale:
        scale_path = (
            diagnostics_dir
            / "12_geometric_physical_moment_scale.png"
        )

        save_tracking_plot(
            scale_path,
            components=[
                (
                    "Moment scale",
                    {
                        "scale": (
                            moment_scale["times_s"],
                            moment_scale["values"],
                        ),
                    },
                ),
            ],
            title="Geometric Physical Moment Feasibility Scale",
            unit="[-]",
            time_origin_s=origin_s,
            xlabel=time_label,
        )

        generated.append(
            scale_path
        )

    return generated


def enrich_summary(
    result,
    bag: BagData,
) -> str:
    """Append common performance metrics to one SE3 bag summary."""
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
