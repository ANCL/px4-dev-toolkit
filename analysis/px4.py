"""Common PX4 conventions used by flight-data analysis."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
import math
import re


_TOPIC_FILE = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "px4_topics.def"
)

_TOPIC_PATTERN = re.compile(
    r'^PX4_TOPIC\(([A-Z0-9_]+),\s*"([^"]+)"\)$'
)


def _load_topics() -> dict[str, str]:
    """Load the project-wide PX4 ROS topic catalog."""
    topics: dict[str, str] = {}

    for raw_line in _TOPIC_FILE.read_text().splitlines():
        line = raw_line.strip()

        if not line or line.startswith("//"):
            continue

        match = _TOPIC_PATTERN.fullmatch(line)

        if match is None:
            raise RuntimeError(
                f"Invalid PX4 topic definition: {raw_line}"
            )

        name, topic = match.groups()
        topics[name] = topic

    return topics


TOPICS = _load_topics()


def quaternion_to_euler(
    quaternion: Sequence[float],
) -> tuple[float, float, float]:
    """Convert a PX4 attitude quaternion to aerospace Euler angles.

    Input:
        Hamilton quaternion [w, x, y, z] mapping FRD body vectors into NED.

    Method:
        Apply the corresponding roll/pitch/yaw decomposition, clamping the
        pitch argument for numerical robustness.

    Returns:
        Roll, pitch, and yaw [rad].
    """
    w, x, y, z = (
        float(value)
        for value in quaternion
    )

    norm = math.sqrt(
        w * w
        + x * x
        + y * y
        + z * z
    )

    if norm <= 1.0e-12:
        raise ValueError(
            "Quaternion norm must be non-zero."
        )

    w /= norm
    x /= norm
    y /= norm
    z /= norm

    sin_roll = 2.0 * (
        w * x + y * z
    )
    cos_roll = 1.0 - 2.0 * (
        x * x + y * y
    )
    roll = math.atan2(
        sin_roll,
        cos_roll,
    )

    sin_pitch = 2.0 * (
        w * y - z * x
    )

    if abs(sin_pitch) >= 1.0:
        pitch = math.copysign(
            math.pi / 2.0,
            sin_pitch,
        )
    else:
        pitch = math.asin(
            sin_pitch
        )

    sin_yaw = 2.0 * (
        w * z + x * y
    )
    cos_yaw = 1.0 - 2.0 * (
        y * y + z * z
    )
    yaw = math.atan2(
        sin_yaw,
        cos_yaw,
    )

    return roll, pitch, yaw
