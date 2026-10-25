"""Experiment-specific analysis profiles and profile discovery."""

from __future__ import annotations

import json
from pathlib import Path

from . import (
    offboard_position,
    onboard_se3,
    offboard_takeoff_handoff,
    position_takeoff_hover,
    se3,
)


PROFILES = {
    onboard_se3.PROFILE_NAME: onboard_se3,
    position_takeoff_hover.PROFILE_NAME: position_takeoff_hover,
    offboard_position.PROFILE_NAME: offboard_position,
    offboard_takeoff_handoff.PROFILE_NAME: offboard_takeoff_handoff,
    se3.PROFILE_NAME: se3,
}


def get_profile(name: str):
    """Return one registered experiment profile by its stable name."""
    try:
        return PROFILES[name]
    except KeyError as exc:
        available = ", ".join(
            sorted(PROFILES)
        )
        raise ValueError(
            f"unknown analysis profile {name!r}; "
            f"available: {available}"
        ) from exc


def read_experiment(bag_path: Path) -> dict[str, object]:
    """Read a recorded run's required experiment configuration."""
    path = bag_path / "experiment.json"
    if not path.is_file():
        raise RuntimeError(f"Missing experiment.json in bag: {bag_path}")
    try:
        data = json.loads(path.read_text())
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise RuntimeError(f"Invalid experiment.json: {path}") from exc
    if not isinstance(data, dict) or not isinstance(data.get("profile"), str):
        raise RuntimeError(f"Invalid experiment.json profile: {path}")
    if not isinstance(data.get("config"), dict):
        raise RuntimeError(f"Invalid experiment.json config: {path}")
    return data


def detect_profile_name(bag_path: Path) -> str:
    """Use the required run metadata, never the bag directory name."""
    return str(read_experiment(bag_path)["profile"])
