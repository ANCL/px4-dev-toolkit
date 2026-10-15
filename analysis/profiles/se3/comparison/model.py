"""Construct a common measured-signal view for SE(3) controller runs.

Each recorded run retains its own time series. Trajectory windows are resolved
relative to confirmed Offboard entry before being passed to maneuver metrics,
so results do not depend on when rosbag recording happened to begin.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import yaml
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


def trajectory_segments(
    config_path: Path,
    trajectory_name: str,
) -> list[dict[str, object]]:
    """Resolve trajectory segments relative to confirmed Offboard entry.

    Inputs:
        Trajectory YAML and the same trajectory/segment name passed to runtime.

    Method:
        Expand a named trajectory into reusable segments. A reusable segment
        selected directly is treated as a one-segment trajectory, matching
        trajectory::load_trajectory() runtime behavior.

    Returns:
        Ordered segment metadata with cumulative start/end times beginning at
        Offboard-relative t=0.
    """
    data = yaml.safe_load(
        config_path.read_text()
    )
    segments = data.get("segments", {})
    trajectories = data.get(
        "trajectories",
        {},
    )

    if trajectory_name in trajectories:
        segment_names = trajectories[
            trajectory_name
        ]["segments"]
    elif trajectory_name in segments:
        segment_names = [
            trajectory_name
        ]
    else:
        raise ValueError(
            f"Unknown trajectory or segment: {trajectory_name}"
        )

    elapsed = 0.0
    result = []
    current = [float(v) for v in data["start_position"]]
    if trajectory_name in trajectories and "start_position" in trajectories[trajectory_name]:
        current = [float(v) for v in trajectories[trajectory_name]["start_position"]]
    heading = float(data["default_yaw"])

    for index, segment_name in enumerate(
        segment_names,
        start=1,
    ):
        if segment_name not in segments:
            raise ValueError(
                f"Trajectory {trajectory_name} references "
                f"unknown segment {segment_name}"
            )

        segment = segments[segment_name]
        duration = float(
            segment["duration"]
        )

        item: dict[str, object] = {
            "index": index,
            "name": segment_name,
            "type": str(
                segment["type"]
            ),
            "start_s": elapsed,
            "end_s": elapsed + duration,
        }

        if "position" in segment:
            target = [float(v) for v in segment["position"]]
            # Backward-compatible metric key: this is a DERIVED signed step
            # amplitude, not a relative YAML command.
            item["offset"] = [target[i] - current[i] for i in range(3)]
            item["position"] = target
            current = target
        if "center" in segment:
            item["center"] = [float(v) for v in segment["center"]]
        if segment["type"] in {"circle", "helix"}:
            import math
            center = item["center"]
            angle = 2.0 * math.pi * float(segment["turns"])
            r = float(segment["radius"])
            current = [center[0] + r * math.cos(angle),
                       center[1] + r * math.sin(angle),
                       center[2] + (float(segment["height"]) if segment["type"] == "helix" else 0.0)]
        if "heading" in segment:
            new_heading = float(segment["heading"])
            item["yaw_offset"] = new_heading - heading
            item["heading"] = new_heading
            heading = new_heading

        result.append(item)
        elapsed += duration

    return result
