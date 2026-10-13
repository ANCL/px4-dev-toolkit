"""Resolve PX4 arming, handoff ownership, and controller metadata."""

from __future__ import annotations

import json
from analysis.core import BagData, TimedSample
from .model import (
    DIRECT_CONTROLLERS,
    GEOMETRIC_NORMALIZED_ANGULAR_ACCELERATION_FEEDFORWARD_TOPIC,
    GEOMETRIC_NORMALIZED_ANGULAR_VELOCITY_FEEDBACK_TOPIC,
    GEOMETRIC_NORMALIZED_ATTITUDE_FEEDBACK_TOPIC,
    HANDOFF_MODE_TOPICS,
    OFFBOARD_MODE_FIELDS,
)


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
    """Resolve the single PX4 Offboard handoff used by the run.

    Inputs:
        Recorded OffboardControlMode history and bag topics.

    Method:
        Treat OffboardControlMode as authoritative. Reject simultaneous or
        changing handoff levels instead of inferring ownership downstream.

    Returns:
        Handoff name and the required recorded input streams.
    """
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
    """Resolve the controller used below a thrust-and-torque handoff.

    Inputs:
        Bag metadata, selected Offboard handoff, and optional toolkit
        diagnostic topics.

    Method:
        Prefer explicit experiment metadata. When metadata is absent, identify
        geometric_normalized only from its controller-owned diagnostic topics;
        do not guess px4_rate versus geometric_physical from native PX4 wrench
        output alone.

    Returns:
        Direct-controller name, "unknown" when it cannot be distinguished, or
        None when the selected handoff does not use a direct controller.
    """
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

    # Without explicit direct-controller metadata, geometric_normalized can
    # still be identified from its controller-owned diagnostic topics. px4_rate
    # and geometric_physical cannot be distinguished reliably from native PX4
    # wrench topics alone.
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
