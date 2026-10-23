#!/usr/bin/env python3
"""Compile absolute-NED YAML definitions into bounded PX4 onboard plans.

Called by ``./tools/trajectory compile <name>``. Each generated PX4_TRAJ_V2
file contains analytic segment parameters and an integrity checksum, not
sampled setpoints. Compilation never selects a PX4 mode or commands flight.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import struct
import zlib
import yaml

TYPE_IDS = {
    "hold": 0,
    "step": 1,
    "line": 2,
    "circle": 3,
    "helix": 4,
    "yaw": 5,
    "figure_eight": 6,
}
MAX_SEGMENTS = 32
TOLERANCE = 1.0e-6


def real(value: object, field: str) -> float:
    """Require finite float32 values, matching the PX4 sampler's storage."""
    if isinstance(value, bool):
        raise ValueError(f"{field}: expected number, not boolean")
    try:
        raw = float(value)
        result = struct.unpack("<f", struct.pack("<f", raw))[0]
    except (TypeError, ValueError, OverflowError, struct.error) as error:
        raise ValueError(f"{field}: invalid number {value!r}") from error
    if not math.isfinite(result):
        raise ValueError(f"{field}: non-finite float32 value")
    return result


def vec(value: object, field: str) -> list[float]:
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        raise ValueError(f"{field}: expected three NED coordinates")
    return [real(x, field) for x in value]


def equal_position(first: list[float], second: list[float], name: str) -> None:
    if any(abs(a - b) > TOLERANCE for a, b in zip(first, second)):
        raise ValueError(
            f"{name}: curve starts away from previous endpoint; "
            "add a line or step before it"
        )


def compile_plan(config: dict, selected: str) -> str:
    """Resolve absolute YAML definitions into the native relative primitives."""
    if not isinstance(config, dict) or config.get("frame") != "px4_local_ned":
        raise ValueError("expected frame: px4_local_ned")
    definitions = config.get("segments")
    trajectories = config.get("trajectories")
    if not isinstance(definitions, dict) or not isinstance(trajectories, dict):
        raise ValueError("segments and trajectories must be maps")

    start = vec(config.get("start_position"), "start_position")
    yaw = real(config.get("default_yaw"), "default_yaw")
    if selected in trajectories:
        definition = trajectories[selected]
        if not isinstance(definition, dict):
            raise ValueError(f"{selected}: expected trajectory map")
        names = definition.get("segments")
        if not isinstance(names, list) or not names:
            raise ValueError(f"{selected}: expected nonempty segment list")
        if "start_position" in definition:
            start = vec(definition["start_position"], "trajectory start_position")
    elif selected in definitions:
        names = [selected]
        d = definitions[selected]
        if not isinstance(d, dict):
            raise ValueError(f"{selected}: expected segment map")
        if d.get("type") == "figure_eight":
            start = vec(d.get("center"), "figure_eight center")
        elif d.get("type") in ("circle", "helix"):
            center = vec(d.get("center"), "circle center")
            radius = real(d.get("radius"), "radius")
            start = [real(center[0] + radius, "start x"), center[1], center[2]]
    else:
        raise ValueError(f"unknown trajectory or segment: {selected}")

    if len(names) > MAX_SEGMENTS:
        raise ValueError(f"trajectory has more than {MAX_SEGMENTS} segments")

    cursor = start.copy()
    heading = yaw
    lines: list[str] = []
    for name in names:
        if not isinstance(name, str) or name not in definitions:
            raise ValueError(f"undefined segment: {name!r}")
        d = definitions[name]
        if not isinstance(d, dict) or "type" not in d or "duration" not in d:
            raise ValueError(f"{name}: expected type and duration")
        kind = d["type"]
        if kind not in TYPE_IDS:
            raise ValueError(f"{name}: unsupported type {kind!r}")
        allowed = {"type", "duration"}
        if kind in ("step", "line"):
            allowed.add("position")
        if kind == "yaw":
            allowed.add("heading")
        if kind in ("circle", "helix", "figure_eight"):
            allowed.update(("center", "turns"))
        if kind in ("circle", "helix"):
            allowed.add("radius")
        if kind == "helix":
            allowed.add("height")
        if kind == "figure_eight":
            allowed.update(("x_amplitude", "y_amplitude"))
        unexpected = d.keys() - allowed
        if unexpected:
            raise ValueError(f"{name}: unexpected fields {sorted(unexpected)}")

        duration = real(d["duration"], f"{name} duration")
        if duration < 0.01:
            raise ValueError(f"{name}: duration must be >= 0.01 s")
        offset = [0.0, 0.0, 0.0]
        radius = height = turns = yaw_offset = xamp = yamp = 0.0

        if kind in ("step", "line"):
            target = vec(d.get("position"), f"{name} position")
            offset = [real(t - p, f"{name} offset") for t, p in zip(target, cursor)]
            cursor = target
        elif kind == "yaw":
            new_heading = real(d.get("heading"), f"{name} heading")
            yaw_offset = real(new_heading - heading, f"{name} yaw offset")
            heading = new_heading
        elif kind == "figure_eight":
            center = vec(d.get("center"), f"{name} center")
            equal_position(cursor, center, name)
            xamp = real(d.get("x_amplitude"), f"{name} x amplitude")
            yamp = real(d.get("y_amplitude"), f"{name} y amplitude")
            turns = real(d.get("turns"), f"{name} turns")
            if min(xamp, yamp, turns) <= 0:
                raise ValueError(f"{name}: amplitudes and turns must be positive")
            theta = 2 * math.pi * turns
            cursor = [
                real(center[0] + xamp * math.sin(theta), f"{name} final x"),
                real(center[1] + yamp * math.sin(2 * theta), f"{name} final y"),
                center[2],
            ]
        elif kind in ("circle", "helix"):
            center = vec(d.get("center"), f"{name} center")
            radius = real(d.get("radius"), f"{name} radius")
            turns = real(d.get("turns"), f"{name} turns")
            if radius <= 0:
                raise ValueError(f"{name}: radius must be positive")
            entry = [real(center[0] + radius, f"{name} entry x"), center[1], center[2]]
            equal_position(cursor, entry, name)
            if kind == "helix":
                height = real(d.get("height"), f"{name} height")
            theta = 2 * math.pi * turns
            cursor = [
                real(center[0] + radius * math.cos(theta), f"{name} final x"),
                real(center[1] + radius * math.sin(theta), f"{name} final y"),
                real(center[2] + height, f"{name} final z"),
            ]

        row = [duration, *offset, radius, height, turns, yaw_offset, xamp, yamp]
        lines.append(f"{TYPE_IDS[kind]} " + " ".join(format(x, ".9g") for x in row))

    if any(abs(a-b) > 0.01 for a, b in zip(cursor, start)) or \
            abs(math.atan2(math.sin(heading-yaw), math.cos(heading-yaw))) > 0.01:
        raise ValueError(f"{selected}: final pose must equal absolute start pose")

    body = ("PX4_TRAJ_V2\n" + " ".join(format(x, ".9g") for x in [*start, yaw]) + "\n"
            + str(len(lines)) + "\n" + "\n".join(lines) + "\n")
    return body + f"CRC32 {zlib.crc32(body.encode('ascii')):08X}\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name in trajectories.yaml, or one reusable segment")
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[2] / "config/trajectories.yaml")
    parser.add_argument("--output", type=Path, required=True, help="onboard .traj output path")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    output = compile_plan(config, args.name)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(output)
    print(f"Compiled {args.name} -> {args.output} ({output.splitlines()[2]} segments)")


if __name__ == "__main__":
    main()
