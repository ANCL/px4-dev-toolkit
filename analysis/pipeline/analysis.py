"""Assemble the recorded PX4 control-pipeline layers."""

from __future__ import annotations

import math

from analysis.core import BagData

from .topics import (
    ANGULAR_VELOCITY_TOPIC,
    ATTITUDE_SETPOINT_TOPIC,
    ATTITUDE_TOPIC,
    LOCAL_POSITION_SETPOINT_TOPIC,
    LOCAL_POSITION_TOPIC,
    RATES_SETPOINT_TOPIC,
    THRUST_SETPOINT_TOPIC,
    TORQUE_SETPOINT_TOPIC,
    TRAJECTORY_TOPIC,
)
from .signals import (
    _attitude_series,
    _motor_series,
    _series3,
    _valid_position,
    _valid_velocity,
)


def analyze_control_pipeline(
    bag: BagData,
) -> dict[str, object]:
    """Extract a common view of the native PX4 multicopter control pipeline.

    Inputs:
        BagData containing any available native PX4 observation topics.

    Method:
        Convert each available layer into finite, rosbag-time-aligned position,
        velocity, attitude, rate, wrench, and motor series. Missing layers are
        preserved as empty series because some handoffs intentionally bypass
        portions of the native cascade.

    Returns:
        A layer-organized dictionary used by summaries, plots, and comparisons.
    """
    actual_position = _series3(
        bag,
        LOCAL_POSITION_TOPIC,
        lambda msg: (msg.x, msg.y, msg.z),
        valid=_valid_position,
    )
    actual_velocity = _series3(
        bag,
        LOCAL_POSITION_TOPIC,
        lambda msg: (msg.vx, msg.vy, msg.vz),
        valid=_valid_velocity,
    )
    actual_acceleration = _series3(
        bag,
        LOCAL_POSITION_TOPIC,
        lambda msg: (msg.ax, msg.ay, msg.az),
    )

    trajectory_position = _series3(
        bag,
        TRAJECTORY_TOPIC,
        lambda msg: msg.position,
    )
    trajectory_velocity = _series3(
        bag,
        TRAJECTORY_TOPIC,
        lambda msg: msg.velocity,
    )
    trajectory_acceleration = _series3(
        bag,
        TRAJECTORY_TOPIC,
        lambda msg: msg.acceleration,
    )

    controller_position = _series3(
        bag,
        LOCAL_POSITION_SETPOINT_TOPIC,
        lambda msg: (msg.x, msg.y, msg.z),
    )
    controller_velocity = _series3(
        bag,
        LOCAL_POSITION_SETPOINT_TOPIC,
        lambda msg: (msg.vx, msg.vy, msg.vz),
    )
    controller_acceleration = _series3(
        bag,
        LOCAL_POSITION_SETPOINT_TOPIC,
        lambda msg: msg.acceleration,
    )

    attitude_actual = _attitude_series(
        bag,
        ATTITUDE_TOPIC,
        "q",
    )
    attitude_setpoint = _attitude_series(
        bag,
        ATTITUDE_SETPOINT_TOPIC,
        "q_d",
    )

    degrees = 180.0 / math.pi

    rate_actual = _series3(
        bag,
        ANGULAR_VELOCITY_TOPIC,
        lambda msg: msg.xyz,
        scale=degrees,
    )
    rate_setpoint = _series3(
        bag,
        RATES_SETPOINT_TOPIC,
        lambda msg: (msg.roll, msg.pitch, msg.yaw),
        scale=degrees,
    )

    thrust = _series3(
        bag,
        THRUST_SETPOINT_TOPIC,
        lambda msg: msg.xyz,
    )
    torque = _series3(
        bag,
        TORQUE_SETPOINT_TOPIC,
        lambda msg: msg.xyz,
    )

    return {
        "position": {
            "actual": actual_position,
            "trajectory": trajectory_position,
            "controller setpoint": controller_position,
        },
        "velocity": {
            "actual": actual_velocity,
            "trajectory": trajectory_velocity,
            "controller setpoint": controller_velocity,
        },
        "acceleration": {
            "actual": actual_acceleration,
            "trajectory": trajectory_acceleration,
            "controller setpoint": controller_acceleration,
        },
        "attitude": {
            "actual": attitude_actual,
            "setpoint": attitude_setpoint,
        },
        "rates": {
            "actual": rate_actual,
            "setpoint": rate_setpoint,
        },
        "thrust": {
            "setpoint": thrust,
        },
        "torque": {
            "setpoint": torque,
        },
        "motors": _motor_series(bag),
    }
