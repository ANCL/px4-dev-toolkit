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
    """Write immutable analysis context beside one finalized sequence bag.

    The metadata records profile, sequence/step identity, and effective launch
    arguments so later analysis does not infer controller configuration from
    downstream PX4 topics.
    """
    experiment_path = (
        bag_path / "experiment.json"
    )

    experiment = {
        "profile": profile,
        "sequence": sequence_name,
        "step": str(step["name"]),
        "arguments": dict(step["arguments"]),
    }

    experiment_path.write_text(
        json.dumps(
            experiment,
            indent=2,
        )
        + "\n"
    )
