"""Expose native PX4 control-pipeline analysis and presentation."""

from .topics import (
    ACTUATOR_MOTORS_TOPIC,
    ANGULAR_VELOCITY_TOPIC,
    ATTITUDE_SETPOINT_TOPIC,
    ATTITUDE_TOPIC,
    CONTROL_PIPELINE_TOPICS,
    LOCAL_POSITION_SETPOINT_TOPIC,
    LOCAL_POSITION_TOPIC,
    RATES_SETPOINT_TOPIC,
    THRUST_SETPOINT_TOPIC,
    TORQUE_SETPOINT_TOPIC,
    TRAJECTORY_TOPIC,
)

from .signals import (
    _finite3,
    _series3,
    _valid_position,
    _valid_velocity,
    _attitude_series,
    _yaw_series,
    _motor_series,
)

from .statistics import (
    _aligned_error_stats,
    _axis_range,
)

from .analysis import (
    analyze_control_pipeline,
)

from .reporting import (
    control_pipeline_summary,
)

from .plotting import (
    _components,
    write_control_pipeline_plots,
)
