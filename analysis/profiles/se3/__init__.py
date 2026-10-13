"""Expose geometric SE(3) analysis and controller diagnostics."""

from .model import (
    ACCELERATION_HANDOFF_TOPIC,
    ANGULAR_ACCELERATION_FEEDFORWARD_TOPIC,
    ATTITUDE_HANDOFF_TOPIC,
    ATTITUDE_RATE_HANDOFF_TOPIC,
    BODY_RATE_SETPOINT_TOPIC,
    COMMANDED_ACCELERATION_TOPIC,
    DESIRED_ANGULAR_VELOCITY_TOPIC,
    DESIRED_ATTITUDE_TOPIC,
    DIAGNOSTIC_TOPICS,
    DIRECT_CONTROLLERS,
    FORCE_COMMAND_TOPIC,
    GEOMETRIC_NORMALIZED_ANGULAR_ACCELERATION_FEEDFORWARD_TOPIC,
    GEOMETRIC_NORMALIZED_ANGULAR_VELOCITY_FEEDBACK_TOPIC,
    GEOMETRIC_NORMALIZED_ATTITUDE_FEEDBACK_TOPIC,
    GEOMETRIC_PHYSICAL_APPLIED_COLLECTIVE_THRUST_TOPIC,
    GEOMETRIC_PHYSICAL_APPLIED_MOMENT_TOPIC,
    GEOMETRIC_PHYSICAL_MOMENT_SCALE_TOPIC,
    GEOMETRIC_PHYSICAL_REQUESTED_COLLECTIVE_THRUST_TOPIC,
    GEOMETRIC_PHYSICAL_REQUESTED_MOMENT_TOPIC,
    HANDOFF_MODE_TOPICS,
    HANDOFF_TOPICS,
    OFFBOARD_MODE_FIELDS,
    OFFBOARD_MODE_TOPIC,
    OPTIONAL_TOPICS,
    PROFILE_NAME,
    PX4_RATE_DERIVATIVE_FEEDBACK_TOPIC,
    PX4_RATE_FEEDFORWARD_TOPIC,
    PX4_RATE_INTEGRAL_FEEDBACK_TOPIC,
    PX4_RATE_INTEGRATOR_STATE_TOPIC,
    PX4_RATE_PROPORTIONAL_FEEDBACK_TOPIC,
    PX4_RATE_RATE_ERROR_TOPIC,
    PX4_RATE_UNFILTERED_TORQUE_TOPIC,
    REQUIRED_TOPICS,
    STATUS_TOPIC,
    THRUST_HANDOFF_TOPIC,
    TORQUE_HANDOFF_TOPIC,
    TRAJECTORY_REFERENCE_TOPIC,
    AnalysisResult,
)

from .lifecycle import (
    _required,
    _handoff_samples,
    _first_armed_time,
    _direct_controller_name,
)

from .extraction import (
    extract_se3_layers,
)

from .diagnostics import (
    _vector3_diagnostic_series,
    extract_common_diagnostics,
    extract_geometric_normalized_diagnostics,
    extract_px4_rate_diagnostics,
    _scalar_series,
    extract_geometric_physical_diagnostics,
)

from .reporting import (
    _boolean_activity_duration,
    _geometric_physical_summary,
    _message_count,
    _finite_count,
    _tracking_lines,
    _se3_summary,
    enrich_summary,
)

from .analysis import (
    analyze,
)

from .plotting import (
    _vector_components,
    write_plots,
)
