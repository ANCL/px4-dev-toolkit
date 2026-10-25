"""Read onboard SE(3) flights from PX4 ULog on the PX4 boot-time axis."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np

TOPICS = (
    "se3_trajectory_status", "vehicle_local_position", "vehicle_attitude",
    "vehicle_attitude_setpoint",
    "vehicle_angular_velocity", "vehicle_rates_setpoint", "actuator_motors",
    "vehicle_torque_setpoint", "vehicle_status", "control_allocator_status",
)


@dataclass
class Run:
    """One explicitly started trajectory within one ULog recording."""

    source: Path
    metadata: dict[str, Any]
    run_id: int
    checksum: int
    start_s: float
    end_s: float
    completed: bool
    reference: dict[str, np.ndarray]
    position: dict[str, np.ndarray]
    rates: dict[str, np.ndarray] | None
    rate_sp: dict[str, np.ndarray] | None
    motors: dict[str, np.ndarray] | None
    torque: dict[str, np.ndarray] | None
    allocator: dict[str, np.ndarray] | None
    dropouts: int
    parameters: dict[str, Any]
    firmware_version: str
    attitude: dict[str, np.ndarray] | None = None
    attitude_sp: dict[str, np.ndarray] | None = None


def resolve_log(source: Path) -> tuple[Path, dict[str, Any]]:
    """Accept a ULog path or an imported onboard experiment directory."""
    source = Path(source).expanduser().resolve()
    if source.is_file() and source.suffix.lower() == ".ulg":
        return source, {}
    if not source.is_dir() or not (source / "flight.ulg").is_file():
        raise ValueError(f"expected a .ulg file or onboard flight directory: {source}")
    info = source / "experiment.json"
    return source / "flight.ulg", json.loads(info.read_text()) if info.is_file() else {}


def _topic(ulog: Any, name: str, required: bool = False) -> dict[str, np.ndarray] | None:
    try:
        fields = ulog.get_dataset(name).data
    except (KeyError, ValueError, IndexError):
        if required:
            raise ValueError(f"missing ULog topic {name}; verify PX4 logger configuration") from None
        return None
    if "timestamp" not in fields or not len(fields["timestamp"]):
        if required:
            raise ValueError(f"ULog topic {name} has no samples")
        return None
    return {k: np.asarray(v) for k, v in fields.items()}


def _vector(data: dict[str, np.ndarray], key: str, components: int = 3) -> np.ndarray:
    """pyulog expands uORB fixed arrays as field[0], field[1], etc."""
    try:
        return np.column_stack([data[f"{key}[{i}]"] for i in range(components)]).astype(float)
    except KeyError as error:
        raise ValueError(f"ULog field missing: {error.args[0]}") from None


def _series(data: dict[str, np.ndarray] | None, fields: tuple[str, ...],
            arrays: tuple[str, ...] = ()) -> dict[str, np.ndarray] | None:
    if data is None:
        return None
    result: dict[str, np.ndarray] = {"time": np.asarray(data["timestamp"], dtype=float) * 1e-6}
    for name in fields:
        if name not in data:
            raise ValueError(f"ULog field missing: {name}")
        result[name] = np.asarray(data[name], dtype=float)
    for name in arrays:
        result[name] = _vector(data, name, 4 if name in ("control", "q", "q_d") else 3)
    return result


def _reference(data: dict[str, np.ndarray], indices: np.ndarray) -> dict[str, np.ndarray]:
    output = {"time": np.asarray(data["timestamp"][indices], dtype=float) * 1e-6}
    for field in ("position", "velocity", "acceleration", "jerk", "snap"):
        output[field] = _vector(data, field)[indices]
    for field in ("yaw", "yaw_rate", "yaw_acceleration", "elapsed_s", "phase"):
        output[field] = np.asarray(data[field][indices], dtype=float)
    return output


def _window(series: dict[str, np.ndarray] | None, start: float,
            end: float) -> dict[str, np.ndarray] | None:
    if series is None:
        return None
    times = series["time"]
    mask = np.isfinite(times) & (times >= start) & (times <= end)
    return {key: value[mask] for key, value in series.items()}


def read_run(source: Path, run_id: int | None = None) -> Run:
    """Load one run; reject ambiguous multi-run logs unless the run ID is given."""
    log_path, metadata = resolve_log(source)
    try:
        from pyulog import ULog
    except ImportError as error:
        raise RuntimeError(
            "pyulog is required for ULog analysis"
        ) from error

    ulog = ULog(str(log_path), message_name_filter_list=list(TOPICS))
    if ulog.file_corruption:
        raise ValueError(f"ULog corruption detected: {log_path}")
    data = _topic(ulog, "se3_trajectory_status", required=True)
    assert data is not None
    phases = np.asarray(data["phase"], dtype=int)
    ids = np.asarray(data["run_id"], dtype=int)
    crcs = np.asarray(data["plan_crc32"], dtype=np.uint32)
    eligible = (ids > 0) & np.isin(phases, (2, 3)) & (crcs != 0)
    choices = sorted(int(v) for v in np.unique(ids[eligible]))
    hold_only = not choices
    if hold_only:
        # An explicit SE(3) mode handoff can be evaluated before any run.
        indexes = np.where((ids == 0) & (phases == 1))[0]
        if len(indexes) < 2:
            raise ValueError("no active SE(3) reference samples in ULog")
        selected, checksum = 0, 0
        if run_id not in (None, 0):
            raise ValueError("no trajectory run recorded; only SE(3) stationary hold is available")
    else:
        metadata_id = metadata.get("run_id")
        selected = run_id if run_id is not None else metadata_id
        if selected is None and len(choices) != 1:
            raise ValueError(f"ULog contains runs {choices}; select one using --run-id")
        selected = int(choices[0] if selected is None else selected)
        if selected not in choices:
            raise ValueError(f"run_id {selected} is not in ULog; recorded runs: {choices}")
        indexes = np.where(eligible & (ids == selected))[0]
        checksums = {int(v) for v in crcs[indexes]}
        if len(checksums) != 1:
            raise ValueError(f"run {selected} contains inconsistent trajectory checksums")
        checksum = checksums.pop()
    expected = metadata.get("plan_crc32")
    if expected is not None and int(expected) != checksum:
        raise ValueError(f"metadata plan checksum differs from ULog run {selected}")

    reference = _reference(data, indexes)
    start = float(reference["time"][0])
    end = float(reference["time"][-1])
    completed = bool(np.any(reference["phase"] == 3))
    if start >= end:
        raise ValueError(f"run {selected} has insufficient timestamp coverage")

    position = _series(_topic(ulog, "vehicle_local_position", True),
                       ("x", "y", "z", "vx", "vy", "vz", "heading"))
    assert position is not None
    parameters = dict(ulog.initial_parameters)
    for changed_at, name, value in ulog.changed_parameters:
        if float(changed_at) * 1e-6 <= start:
            parameters[name] = value

    return Run(
        source=log_path, metadata=metadata, run_id=selected, checksum=checksum,
        start_s=start, end_s=end, completed=completed, reference=reference,
        position=_window(position, start, end),
        rates=_window(_series(_topic(ulog, "vehicle_angular_velocity"), (), ("xyz",)), start, end),
        attitude=_window(_series(_topic(ulog, "vehicle_attitude"), (), ("q",)), start, end),
        attitude_sp=_window(_series(_topic(ulog, "vehicle_attitude_setpoint"), (), ("q_d",)), start, end),
        rate_sp=_window(_series(_topic(ulog, "vehicle_rates_setpoint"),
                                ("roll", "pitch", "yaw")), start, end),
        motors=_window(_series(_topic(ulog, "actuator_motors"), (), ("control",)), start, end),
        torque=_window(_series(_topic(ulog, "vehicle_torque_setpoint"), (), ("xyz",)), start, end),
        allocator=_window(_series(_topic(ulog, "control_allocator_status"),
                                  ("torque_setpoint_achieved", "thrust_setpoint_achieved")), start, end),
        dropouts=len(ulog.dropouts), parameters=parameters,
        firmware_version=str(ulog.msg_info_dict.get("ver_sw", "unknown")),
    )
