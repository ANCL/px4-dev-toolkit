"""SE(3) analysis topic declarations, handoff selections, and result schema."""

from __future__ import annotations

from dataclasses import dataclass
from analysis.pipeline import CONTROL_PIPELINE_TOPICS
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
    """Single-run SE(3) analysis consumed by reporting/comparison code."""

    summary: str
    metrics: dict[str, float | bool | str]
    plot_data: dict[str, object]
