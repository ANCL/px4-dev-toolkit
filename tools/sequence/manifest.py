"""Persist study state and finalized rosbag analysis metadata."""

from __future__ import annotations

import json
from pathlib import Path


def write_manifest(path: Path, manifest: dict[str, object]) -> None:
    """Persist the study-level progress/result document as readable JSON."""
    path.write_text(
        json.dumps(
            manifest,
            indent=2,
            sort_keys=False,
        )
        + "\n"
    )


def write_experiment_metadata(
    bag_path: Path,
    profile: str,
    sequence_name: str,
    step: dict[str, object],
) -> None:
    """Attach study identity to a finalized run's experiment.json.

    The shared recorder owns experiment settings. The sequence runner
    adds study identity without replacing the recorded configuration.
    """
    experiment_path = bag_path / "experiment.json"
    if not experiment_path.is_file():
        raise RuntimeError(f"Missing experiment.json in finalized bag: {bag_path}")
    experiment = json.loads(experiment_path.read_text())
    if experiment.get("profile") != profile:
        raise RuntimeError(f"Sequence profile does not match recorded run: {bag_path}")
    experiment["study"] = {"name": sequence_name, "step": str(step["name"])}
    experiment_path.write_text(
        json.dumps(
            experiment,
            indent=2,
        )
        + "\n"
    )
