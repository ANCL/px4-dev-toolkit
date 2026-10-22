"""Validate sequence YAML before any ROS process or bag directory exists.

Normalized launch arguments are strings, and step/study names are restricted
to a predictable character set before paths or launch commands are built.
"""

from __future__ import annotations

from pathlib import Path
import re
import yaml


NAME_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+$")

def validate_name(value: object, field: str) -> str:
    """Restrict study and step identifiers to the configured name characters."""
    if (
        not isinstance(value, str)
        or not value
        or NAME_PATTERN.fullmatch(value) is None
    ):
        raise ValueError(
            f"{field} must contain only letters, numbers, '.', '_' or '-'."
        )

    return value


def launch_value(value: object) -> str:
    """Serialize YAML values as ROS launch arguments, not Python literals."""
    if isinstance(value, bool):
        return "true" if value else "false"

    if isinstance(value, (str, int, float)):
        return str(value)

    raise ValueError(
        "Launch argument values must be strings, numbers, or booleans."
    )


def load_sequence(path: Path) -> dict[str, object]:
    """Load and normalize one sequence definition.

    Inputs:
        YAML path describing profile, recording argument, settle time, and
        ordered launch steps.

    Method:
        Validate names and launch arguments before any study directory or
        process is created.

    Returns:
        Normalized sequence data suitable for deterministic execution.
    """
    if not path.is_file():
        raise ValueError(f"Sequence file does not exist: {path}")

    data = yaml.safe_load(path.read_text())

    if not isinstance(data, dict):
        raise ValueError("Sequence configuration must be a YAML mapping.")

    name = validate_name(data.get("name"), "Sequence name")

    profile = validate_name(
        data.get("profile"),
        "Sequence profile",
    )

    recording_output_argument = validate_name(
        data.get("recording_output_argument"),
        "Sequence recording_output_argument",
    )

    settle_seconds = data.get("settle_seconds", 0.0)

    if (
        not isinstance(settle_seconds, (int, float))
        or isinstance(settle_seconds, bool)
        or settle_seconds < 0.0
    ):
        raise ValueError(
            "settle_seconds must be a non-negative number."
        )

    raw_steps = data.get("steps")

    if not isinstance(raw_steps, list) or not raw_steps:
        raise ValueError("Sequence must define at least one step.")

    steps = []

    for index, raw_step in enumerate(raw_steps, start=1):
        if not isinstance(raw_step, dict):
            raise ValueError(
                f"Step {index} must be a YAML mapping."
            )

        step_name = validate_name(
            raw_step.get("name"),
            f"Step {index} name",
        )

        package = raw_step.get("package")
        launch = raw_step.get("launch")

        if not isinstance(package, str) or not package:
            raise ValueError(
                f"Step '{step_name}' requires a package."
            )

        if not isinstance(launch, str) or not launch:
            raise ValueError(
                f"Step '{step_name}' requires a launch file."
            )

        arguments = raw_step.get("arguments", {})

        if not isinstance(arguments, dict):
            raise ValueError(
                f"Step '{step_name}' arguments must be a mapping."
            )

        normalized_arguments = {
            str(key): launch_value(value)
            for key, value in arguments.items()
        }

        step_profile = validate_name(
            raw_step.get("profile", profile),
            f"Step {index} profile",
        )

        steps.append(
            {
                "name": step_name,
                "profile": step_profile,
                "package": package,
                "launch": launch,
                "arguments": normalized_arguments,
            }
        )

    return {
        "name": name,
        "profile": profile,
        "recording_output_argument":
            recording_output_argument,
        "settle_seconds": float(settle_seconds),
        "steps": steps,
    }
