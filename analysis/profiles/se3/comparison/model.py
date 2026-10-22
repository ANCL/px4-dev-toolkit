"""Construct a common measured-signal view for SE(3) controller runs.

Each recorded run retains its own time series. Trajectory windows are resolved
relative to confirmed Offboard entry before being passed to maneuver metrics,
so results do not depend on when rosbag recording happened to begin.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
from analysis.comparison import (
    shift_motor_series,
    shift_vector_series,
    tracking_error_series,
)


_COMPONENTS = (
    "x",
    "y",
    "z",
)


_AXIS_NAMES = (
    "North",
    "East",
    "Down",
)


@dataclass
class ComparisonRun:
    """Common physical comparison view of one analyzed SE(3) run."""

    label: str
    bag_path: Path
    handoff: str
    direct_controller: str
    duration_s: float

    position_actual: dict[str, list[float]]
    position_reference: dict[str, list[float]]
    position_error: dict[str, list[float]]
    velocity_error: dict[str, list[float]]
    yaw_error: dict[str, list[float]]

    thrust: dict[str, list[float]]
    torque: dict[str, list[float]]
    body_rates: dict[str, list[float]]

    motors: dict[
        str,
        tuple[list[float], list[float]],
    ]


def build_run(
    result,
    bag_path: Path,
    label: str,
) -> ComparisonRun:
    """Build the common comparison view for one analyzed SE(3) run.

    Inputs:
        Single-run analysis result, bag path, and display label.

    Method:
        Shift physical signals so t=0 is PX4's confirmed Offboard entry. This
        removes arming/prestream timing from controller-performance comparison.

    Returns:
        ComparisonRun containing aligned tracking, wrench, rate, and motor data.
    """
    layers = result.plot_data["layers"]
    pipeline = result.plot_data["pipeline"]

    origin = float(
        result.metrics["offboard_entry_s"]
    )

    duration = float(
        result.metrics["offboard_duration_s"]
    )

    end = (
        origin
        + duration
    )

    return ComparisonRun(
        label=label,
        bag_path=bag_path,
        handoff=str(
            result.metrics["handoff"]
        ),
        direct_controller=str(
            result.metrics[
                "direct_controller"
            ]
        ),
        duration_s=duration,
        position_actual=shift_vector_series(
            pipeline["position"]["actual"],
            origin,
            duration,
        ),
        position_reference=shift_vector_series(
            layers["reference"]["position"],
            origin,
            duration,
        ),
        position_error=tracking_error_series(
            pipeline["position"]["actual"],
            layers["reference"]["position"],
            time_origin_s=origin,
            end_time_s=end,
        ),
        velocity_error=tracking_error_series(
            pipeline["velocity"]["actual"],
            layers["reference"]["velocity"],
            time_origin_s=origin,
            end_time_s=end,
        ),
        yaw_error=tracking_error_series(
            pipeline["attitude"]["actual"],
            layers["reference"]["yaw"],
            time_origin_s=origin,
            end_time_s=end,
            components=("z",),
            wrap_degrees=True,
        ),
        thrust=shift_vector_series(
            pipeline["thrust"]["setpoint"],
            origin,
            duration,
        ),
        torque=shift_vector_series(
            pipeline["torque"]["setpoint"],
            origin,
            duration,
        ),
        body_rates=shift_vector_series(
            pipeline["rates"]["actual"],
            origin,
            duration,
        ),
        motors=shift_motor_series(
            pipeline["motors"],
            origin,
            duration,
        ),
    )


def trajectory_segments(configuration: dict[str, object]) -> list[dict[str, object]]:
    """Resolve recorded absolute-NED segments relative to Offboard entry."""
    if configuration.get("frame") != "px4_local_ned":
        raise ValueError("Recorded trajectory frame must be px4_local_ned")

    elapsed = 0.0
    result: list[dict[str, object]] = []
    current = [float(v) for v in configuration["start_position"]]
    heading = float(configuration["default_yaw"])

    for index, definition in enumerate(configuration["segments"], start=1):
        kind = str(definition["type"])
        duration = float(definition["duration"])
        item: dict[str, object] = {
            "index": index,
            "name": definition["name"],
            "type": kind,
            "start_s": elapsed,
            "end_s": elapsed + duration,
        }
        if "position" in definition:
            target = [float(v) for v in definition["position"]]
            # Internal step amplitude: YAML positions remain absolute NED.
            item["offset"] = [target[i] - current[i] for i in range(3)]
            item["position"] = target
            current = target
        if "center" in definition:
            center = [float(v) for v in definition["center"]]
            item["center"] = center
            if kind in ("circle", "helix"):
                angle = 2.0 * math.pi * float(definition["turns"])
                radius = float(definition["radius"])
                height = float(definition["height"]) if kind == "helix" else 0.0
                current = [center[0] + radius * math.cos(angle),
                           center[1] + radius * math.sin(angle), center[2] + height]
            elif kind == "figure_eight":
                angle = 2.0 * math.pi * float(definition["turns"])
                current = [center[0] + float(definition["x_amplitude"]) * math.sin(angle),
                           center[1] + float(definition["y_amplitude"]) * math.sin(2 * angle),
                           center[2]]
        if "heading" in definition:
            heading = float(definition["heading"])
            item["heading"] = heading
        result.append(item)
        elapsed += duration
    return result
