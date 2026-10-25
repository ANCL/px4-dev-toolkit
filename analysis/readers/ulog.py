"""Adapt recorded PX4 uORB datasets to the common analysis input contract.

ROS 2 and uORB use the same PX4 message fields for the native control layers.
This reader normalizes the recording envelope (topic names, flattened arrays,
timestamps and validity flags), so pipeline calculations are reused unchanged.

All ULog times remain PX4 boot-relative, converted from microseconds to ns.
ROS bag timestamps remain rosbag receive times. No clocks are mixed between
recordings, and no cross-source offset is invented.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
import re
from types import SimpleNamespace
from typing import Any

import numpy as np

from analysis.core import BagData, TimedSample
from analysis.px4 import TOPICS
from analysis.pipeline.topics import CONTROL_PIPELINE_TOPICS

_ARRAY = re.compile(r"^(.+)\[(\d+)\]$")

# Only the messages consumed by the shared pipeline need decoding. Everything
# else may be present in the file without becoming an analysis dependency.
UORB_TO_ROS = {
    path.rsplit("/", 1)[-1].removesuffix("_v1"): path
    for path in CONTROL_PIPELINE_TOPICS
}
STATUS = "vehicle_status"
OTHER_TOPICS = (
    STATUS, "vehicle_land_detected", "rate_ctrl_status",
    "control_allocator_status", "se3_trajectory_status",
    "vehicle_local_position_groundtruth", "vehicle_attitude_groundtruth",
    "vehicle_angular_velocity_groundtruth", "vehicle_visual_odometry",
)


@dataclass
class FlightULog:
    """One recording plus the existing pipeline's normalized message view."""

    path: Path
    bag: BagData
    datasets: dict[str, dict[str, np.ndarray]]
    counts: dict[str, int]
    firmware: str
    dropouts: int
    time_basis: str = "PX4 boot-time timestamps (uORB timestamp, microseconds)"


def _message_arrays(data: dict[str, Any]) -> dict[str, list[str]]:
    """Resolve pyulog's `q[0]`/`control[1]` columns into ordered vectors."""
    groups: dict[str, dict[int, str]] = defaultdict(dict)
    for key in data:
        match = _ARRAY.fullmatch(key)
        if match:
            groups[match.group(1)][int(match.group(2))] = key
    return {
        base: [columns[i] for i in sorted(columns)]
        for base, columns in groups.items()
        if sorted(columns) == list(range(len(columns)))
    }


def _record(data: dict[str, Any], index: int, arrays: dict[str, list[str]], name: str) -> SimpleNamespace:
    """Translate one flattened ULog row to the PX4 ROS/uORB field contract."""
    fields = {}
    for key, column in data.items():
        if key != "timestamp" and "[" not in key:
            value = column[index]
            fields[key] = value.item() if isinstance(value, np.generic) else value
    for base, keys in arrays.items():
        fields[base] = [float(data[key][index]) for key in keys]
    # Older PX4 builds may omit these booleans. In that case require finite
    # components downstream, and identify the absent flags in the report.
    if name == "vehicle_local_position":
        fields.setdefault("xy_valid", {"x", "y"}.issubset(data))
        fields.setdefault("z_valid", "z" in data)
        fields.setdefault("v_xy_valid", {"vx", "vy"}.issubset(data))
        fields.setdefault("v_z_valid", "vz" in data)
    # A required field absent from an old uORB version is explicitly invalid;
    # downstream finite filters suppress the affected series.
    scalar_fields = {
        "vehicle_local_position": ("x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az"),
        "vehicle_local_position_setpoint": ("x", "y", "z", "vx", "vy", "vz"),
        "vehicle_rates_setpoint": ("roll", "pitch", "yaw"),
    }
    vector_fields = {
        "trajectory_setpoint": ("position", "velocity", "acceleration"),
        "vehicle_local_position_setpoint": ("acceleration",),
        "vehicle_attitude": ("q",),
        "vehicle_attitude_setpoint": ("q_d",),
        "vehicle_angular_velocity": ("xyz",),
        "vehicle_thrust_setpoint": ("xyz",),
        "vehicle_torque_setpoint": ("xyz",),
    }
    for field in scalar_fields.get(name, ()):
        fields.setdefault(field, float("nan"))
    for field in vector_fields.get(name, ()):
        fields.setdefault(field, [float("nan")] * (4 if field in ("q", "q_d") else 3))
    return SimpleNamespace(**fields)


def _stream(data: dict[str, Any], name: str) -> list[TimedSample]:
    stamp = data.get("timestamp")
    if stamp is None:
        return []
    arrays = _message_arrays(data)
    return [
        TimedSample(timestamp_ns=int(value) * 1000,
                    message=_record(data, index, arrays, name))
        for index, value in enumerate(stamp)
    ]


def read_ulog(path: Path, *, ulog: Any | None = None) -> FlightULog:
    """Read all available native control layers without requiring any one.

    This function never interprets trajectory plans or synthesizes desired
    outputs. Experiment profiles retain responsibility for those operations.
    PX4 native topic instance 0 is selected; other instances remain visible in
    the inventory but are not silently merged into a different sensor frame.
    """
    path = Path(path).expanduser().resolve()
    if ulog is None:
        try:
            from pyulog import ULog
        except ImportError as exc:
            raise RuntimeError("pyulog is required for ULog reading") from exc
        names = set(UORB_TO_ROS) | set(OTHER_TOPICS)
        ulog = ULog(str(path), message_name_filter_list=sorted(names))
    if getattr(ulog, "file_corruption", False):
        raise ValueError(f"ULog corruption detected: {path}")

    datasets: dict[str, dict[str, np.ndarray]] = {}
    counts: dict[str, int] = {}
    for dataset in ulog.data_list:
        # Track the different multi-instances explicitly in coverage reporting.
        name = dataset.name
        key = f"{name}[{dataset.multi_id}]"
        counts[key] = len(dataset.data.get("timestamp", ()))
        if dataset.multi_id == 0 and name not in datasets:
            datasets[name] = dataset.data

    streams = {
        ros_topic: _stream(datasets[name], name)
        for name, ros_topic in UORB_TO_ROS.items()
        if name in datasets
    }
    # If a stream's original field name changes with a PX4 upgrade, it should
    # produce unavailable metrics, not a silently substituted alternative.
    status_topic = TOPICS.get("OUT_VEHICLE_STATUS_V1")
    if status_topic and STATUS in datasets:
        streams[status_topic] = _stream(datasets[STATUS], STATUS)

    stamps = [s.timestamp_ns for group in streams.values() for s in (group[:1] + group[-1:])]
    for data in datasets.values():
        ts = data.get("timestamp")
        if ts is not None and len(ts):
            stamps += [int(ts[0]) * 1000, int(ts[-1]) * 1000]
    if not stamps:
        raise ValueError(f"ULog contains no supported timestamped data: {path}")
    start, end = min(stamps), max(stamps)
    bag = BagData(path=path, start_ns=start, end_ns=end, samples=streams)
    return FlightULog(
        path=path, bag=bag, datasets=datasets, counts=counts,
        firmware=str(ulog.msg_info_dict.get("ver_sw", "unknown")),
        dropouts=len(ulog.dropouts),
    )
