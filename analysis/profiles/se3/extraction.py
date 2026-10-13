"""Extract SE(3) commanded layers independently of the native PX4 pipeline.

Only the selected Offboard handoff is commanded by the toolkit; downstream
PX4 controller signals are observed separately by analysis/pipeline/.
"""

from __future__ import annotations

from analysis.core import BagData
from analysis.pipeline import _attitude_series, _series3
from .model import (
    ACCELERATION_HANDOFF_TOPIC,
    ATTITUDE_HANDOFF_TOPIC,
    ATTITUDE_RATE_HANDOFF_TOPIC,
    THRUST_HANDOFF_TOPIC,
    TORQUE_HANDOFF_TOPIC,
    TRAJECTORY_REFERENCE_TOPIC,
)


def extract_se3_layers(
    bag: BagData,
    handoff_mode: str = "acceleration",
) -> dict[str, object]:
    """Extract toolkit-owned reference and selected handoff signals.

    Inputs:
        BagData and the handoff resolved from OffboardControlMode.

    Method:
        Preserve the generated SE(3) trajectory separately from native PX4
        pipeline topics, then extract only the signal actually handed to PX4
        at the selected ownership boundary.

    Returns:
        Generated trajectory-reference series and selected-handoff series.
    """

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
