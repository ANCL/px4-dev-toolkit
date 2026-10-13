"""Extract measured PX4 signals while retaining each topic's time grid.

PX4 publishes states and controller layers at different rates. Signal
extraction filters invalid samples but does not synchronize or resample them;
alignment is performed later only where a particular metric requires it.
"""

from __future__ import annotations

import math
from analysis.core import BagData
from analysis.px4 import quaternion_to_euler
from .topics import ACTUATOR_MOTORS_TOPIC


def _finite3(values) -> bool:
    return (
        len(values) >= 3
        and all(
            math.isfinite(float(values[index]))
            for index in range(3)
        )
    )


def _series3(
    bag: BagData,
    topic: str,
    getter,
    *,
    valid=None,
    scale: float = 1.0,
) -> dict[str, list[float]]:
    """Extract a finite three-component time series from one PX4 topic."""
    samples = bag.samples.get(topic, [])
    selected = []

    for sample in samples:
        message = sample.message

        if valid is not None and not valid(message):
            continue

        values = getter(message)

        if not _finite3(values):
            continue

        selected.append((sample, values))

    return {
        "message_count": len(samples),
        "finite_count": len(selected),
        "times_s": [
            bag.relative_seconds(sample.timestamp_ns)
            for sample, _ in selected
        ],
        "x": [
            float(values[0]) * scale
            for _, values in selected
        ],
        "y": [
            float(values[1]) * scale
            for _, values in selected
        ],
        "z": [
            float(values[2]) * scale
            for _, values in selected
        ],
    }


def _valid_position(message) -> bool:
    return (
        bool(message.xy_valid)
        and bool(message.z_valid)
        and all(
            math.isfinite(float(value))
            for value in (message.x, message.y, message.z)
        )
    )


def _valid_velocity(message) -> bool:
    return (
        bool(message.v_xy_valid)
        and bool(message.v_z_valid)
        and all(
            math.isfinite(float(value))
            for value in (message.vx, message.vy, message.vz)
        )
    )


def _attitude_series(
    bag: BagData,
    topic: str,
    field: str,
) -> dict[str, list[float]]:
    """Extract roll, pitch, yaw in degrees from a PX4 quaternion stream."""
    times_s = []
    roll = []
    pitch = []
    yaw = []

    for sample in bag.samples.get(topic, []):
        quaternion = getattr(sample.message, field)

        if len(quaternion) < 4:
            continue

        if not all(math.isfinite(float(value)) for value in quaternion):
            continue

        norm_squared = sum(float(value) ** 2 for value in quaternion)

        if norm_squared <= 1e-12:
            continue

        r, p, y = quaternion_to_euler(quaternion)

        times_s.append(
            bag.relative_seconds(sample.timestamp_ns)
        )
        roll.append(math.degrees(r))
        pitch.append(math.degrees(p))
        yaw.append(math.degrees(y))

    return {
        "times_s": times_s,
        "x": roll,
        "y": pitch,
        "z": yaw,
    }


def _yaw_series(
    bag: BagData,
    topic: str,
    field: str,
) -> dict[str, list[float]]:
    """Extract yaw in degrees from a PX4 quaternion stream."""
    series = _attitude_series(bag, topic, field)

    return {
        "times_s": series["times_s"],
        "yaw": series["z"],
    }


def _motor_series(
    bag: BagData,
) -> dict[str, tuple[list[float], list[float]]]:
    """Extract finite PX4 control-allocation motor commands."""
    samples = bag.samples.get(ACTUATOR_MOTORS_TOPIC, [])

    if not samples:
        return {}

    motors = {}
    channel_count = len(samples[0].message.control)

    for index in range(channel_count):
        times_s = []
        values = []

        for sample in samples:
            value = float(sample.message.control[index])

            if not math.isfinite(value):
                continue

            times_s.append(
                bag.relative_seconds(sample.timestamp_ns)
            )
            values.append(value)

        if values:
            motors[f"motor {index + 1}"] = (
                times_s,
                values,
            )

    return motors
