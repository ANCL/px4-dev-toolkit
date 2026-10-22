"""Resolve PX4 arming, handoff ownership, and controller metadata."""

from __future__ import annotations

import json
from analysis.core import BagData, TimedSample
from .model import (
    DIRECT_CONTROLLERS,
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


def _direct_controller_name(bag: BagData, handoff_mode: str) -> str | None:
    """Read the recorded direct controller; do not infer its law from PX4 outputs."""
    if handoff_mode != "thrust_and_torque":
        return None
    path = bag.path / "experiment.json"
    if not path.is_file():
        raise RuntimeError(f"Missing experiment.json in SE3 bag: {bag.path}")
    metadata = json.loads(path.read_text())
    if metadata.get("handoff") != handoff_mode:
        raise RuntimeError("Recorded handoff does not match PX4 OffboardControlMode")
    direct_controller = metadata.get("direct_controller")
    if direct_controller not in DIRECT_CONTROLLERS:
        raise RuntimeError("SE3 bag lacks valid direct controller identity")
    return str(direct_controller)
