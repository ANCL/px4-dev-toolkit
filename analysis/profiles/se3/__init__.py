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


def comparison_input(result: AnalysisResult, _bag):
    """Adapt an SE(3) flight without changing its controller diagnostics."""
    from analysis.comparison import ComparisonInput

    handoff = str(result.metrics["handoff"])
    controller = str(result.metrics["direct_controller"])
    identity = controller if handoff == "thrust_and_torque" else handoff
    names = {
        "acceleration": "Acceleration",
        "attitude": "Attitude",
        "attitude_rate": "Rate",
        "geometric_normalized": "Geometric",
        "px4_rate": "PX4 rate",
        "geometric_physical": "Geometric physical",
    }
    return ComparisonInput(
        label=names.get(identity, identity),
        metrics=result.metrics,
        plot_data=result.plot_data,
    )


def comparison_signals(experiment: dict) -> dict:
    """Physical signal provenance for SE3, chosen by its existing handoff.

    This is only an observation contract. It does not publish control messages
    or reinterpret downstream PX4 setpoints as an upstream SE3 reference.
    """
    from analysis.px4 import TOPICS

    handoff = experiment.get("handoff")
    if handoff not in HANDOFF_MODE_TOPICS:
        raise ValueError(f"Unsupported SE3 comparison handoff: {handoff!r}")

    if handoff == "acceleration":
        # PX4's attitude and rate controllers are active downstream.
        attitude = (TOPICS["OUT_VEHICLE_ATTITUDE_SETPOINT"], "q_d")
        rate = (TOPICS["OUT_VEHICLE_RATES_SETPOINT"], "rpy")
    elif handoff == "attitude":
        attitude = (ATTITUDE_HANDOFF_TOPIC, "q_d")
        rate = (TOPICS["OUT_VEHICLE_RATES_SETPOINT"], "rpy")
    elif handoff == "attitude_rate":
        # PX4 attitude control is bypassed: SE3's desired R_d is diagnostic.
        attitude = (DESIRED_ATTITUDE_TOPIC, "q_d")
        rate = (ATTITUDE_RATE_HANDOFF_TOPIC, "rpy")
    else:
        controller = experiment.get("direct_controller", "geometric_normalized")
        if controller not in DIRECT_CONTROLLERS:
            raise ValueError(f"Unsupported SE3 direct controller: {controller!r}")
        attitude = (DESIRED_ATTITUDE_TOPIC, "q_d")
        # Only the reproduced PX4 rate cascade computes a commanded rate.
        rate = (BODY_RATE_SETPOINT_TOPIC, "xyz") if controller == "px4_rate" else None

    return {
        "trajectory": TRAJECTORY_REFERENCE_TOPIC,
        "attitude": attitude,
        "rates": rate,
    }
