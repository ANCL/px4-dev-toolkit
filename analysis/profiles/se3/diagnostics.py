"""Extract optional internal SE(3) and reproduced PX4-rate diagnostics.

These channels are instrumentation, not prerequisites of the flight-control
path. Missing diagnostics remain absent rather than being inferred from PX4
setpoints, so analysis of runs without instrumentation is still meaningful.
"""

from __future__ import annotations

import math
from analysis.core import BagData
from analysis.pipeline import _attitude_series, _series3
from analysis.px4 import TOPICS
from .model import (
    ANGULAR_ACCELERATION_FEEDFORWARD_TOPIC,
    BODY_RATE_SETPOINT_TOPIC,
    COMMANDED_ACCELERATION_TOPIC,
    DESIRED_ANGULAR_VELOCITY_TOPIC,
    DESIRED_ATTITUDE_TOPIC,
    FORCE_COMMAND_TOPIC,
    GEOMETRIC_NORMALIZED_ANGULAR_ACCELERATION_FEEDFORWARD_TOPIC,
    GEOMETRIC_NORMALIZED_ANGULAR_VELOCITY_FEEDBACK_TOPIC,
    GEOMETRIC_NORMALIZED_ATTITUDE_FEEDBACK_TOPIC,
    GEOMETRIC_PHYSICAL_APPLIED_COLLECTIVE_THRUST_TOPIC,
    GEOMETRIC_PHYSICAL_APPLIED_MOMENT_TOPIC,
    GEOMETRIC_PHYSICAL_MOMENT_SCALE_TOPIC,
    GEOMETRIC_PHYSICAL_REQUESTED_COLLECTIVE_THRUST_TOPIC,
    GEOMETRIC_PHYSICAL_REQUESTED_MOMENT_TOPIC,
    PX4_RATE_DERIVATIVE_FEEDBACK_TOPIC,
    PX4_RATE_FEEDFORWARD_TOPIC,
    PX4_RATE_INTEGRAL_FEEDBACK_TOPIC,
    PX4_RATE_INTEGRATOR_STATE_TOPIC,
    PX4_RATE_PROPORTIONAL_FEEDBACK_TOPIC,
    PX4_RATE_RATE_ERROR_TOPIC,
    PX4_RATE_UNFILTERED_TORQUE_TOPIC,
)


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
