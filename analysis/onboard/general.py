"""General ULog reporting using the toolkit's shared PX4 control pipeline.

The source adapter maps PX4 uORB arrays and timestamps to the same observation
contract used by rosbag profiles. Both formats share physical units, finite
sample filtering, tracking statistics and plots. A recording does not need a
trajectory or a controller diagnostic topic to generate an overview.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from analysis.readers.ulog import read_ulog
from analysis.pipeline import (
    analyze_control_pipeline, control_pipeline_summary,
    write_control_pipeline_plots,
)
from analysis.plots import save_tracking_plot


def resolve(path: Path):
    """Accept an original ULog or a legacy imported flight directory."""
    path = Path(path).expanduser().resolve()
    if path.is_file() and path.suffix.lower() == ".ulg":
        return path, {}
    if path.is_dir() and (path / "flight.ulg").is_file():
        metadata = path / "experiment.json"
        return path / "flight.ulg", json.loads(metadata.read_text()) if metadata.is_file() else {}
    raise ValueError("expected a .ulg or imported PX4 experiment directory")


def load(path: Path, all_topics: bool = False):
    """Retain the original ULog-reader API for profile detection and tests."""
    try:
        from pyulog import ULog
    except ImportError as error:
        raise RuntimeError("install the onboard analysis dependencies") from error
    from analysis.readers.ulog import UORB_TO_ROS, OTHER_TOPICS
    source, metadata = resolve(path)
    selected = None if all_topics else sorted(set(UORB_TO_ROS) | set(OTHER_TOPICS))
    ulog = ULog(str(source), message_name_filter_list=selected)
    if ulog.file_corruption:
        raise ValueError("ULog is corrupt: " + str(source))
    datasets = {v.name: v.data for v in ulog.data_list if v.multi_id == 0}
    return source, metadata, ulog, datasets


def detect(path: Path) -> str:
    _, metadata, _, datasets = load(path)
    profile = metadata.get("profile")
    if profile in ("onboard_se3", "px4_ulog"):
        return profile
    status = datasets.get("se3_trajectory_status", {})
    return "onboard_se3" if len(status.get("timestamp", ())) >= 2 else "px4_ulog"


def _output_dir(path: Path) -> Path:
    source, _ = resolve(path)
    return (path / "analysis") if path.is_dir() else (source.parent / source.stem / "analysis")


def _position_overview(flight) -> list[str]:
    data = flight.datasets.get("vehicle_local_position", {})
    required = ("timestamp", "x", "y", "z")
    if not all(key in data for key in required):
        return ["Position: not logged"]
    t = np.asarray(data["timestamp"], dtype=float) * 1e-6
    xyz = np.column_stack([data[key] for key in ("x", "y", "z")]).astype(float)
    valid = np.isfinite(t) & np.all(np.isfinite(xyz), axis=1)
    if "xy_valid" in data:
        valid &= np.asarray(data["xy_valid"], dtype=bool)
    if "z_valid" in data:
        valid &= np.asarray(data["z_valid"], dtype=bool)
    t, xyz = t[valid], xyz[valid]
    if not len(t):
        return ["Position: no valid samples"]
    return [
        f"Position samples: {len(t)}",
        f"Measured interval: {t[-1] - t[0]:.3f} s",
        f"Position start NED [m]: {xyz[0].tolist()}",
        f"Position end NED [m]: {xyz[-1].tolist()}",
        f"Position range NED [m]: {(np.max(xyz, axis=0)-np.min(xyz, axis=0)).tolist()}",
    ]


def _flight_events(flight) -> list[str]:
    states = flight.datasets.get("vehicle_status", {})
    if "nav_state" not in states or "timestamp" not in states:
        return ["Navigation-state transitions: unavailable"]
    result = ["Navigation-state transitions:"]
    previous = None
    for stamp, state in zip(states["timestamp"], states["nav_state"]):
        value = int(state)
        if value != previous:
            result.append(f"  {(int(stamp)*1000-flight.bag.start_ns)/1e9:.3f}s: nav_state={value}")
            previous = value
    return result


def _coverage(flight) -> list[str]:
    from analysis.readers.ulog import UORB_TO_ROS
    output = ["", "Recorded source coverage", "------------------------",
              "Counts are logged samples, not controller execution frequencies."]
    for name in sorted(UORB_TO_ROS):
        output.append(f"  {name:<37} {flight.counts.get(name + '[0]', 0):>7} samples")
    output += ["", "Additional diagnostic datasets:"]
    for name in sorted(set(flight.datasets) - set(UORB_TO_ROS)):
        output.append(f"  {name:<37} {flight.counts.get(name + '[0]', 0):>7} samples")
    return output


def _pipeline_lines(flight, pipeline) -> list[str]:
    return ["", *control_pipeline_summary(pipeline)]


def _analyze(path: Path):
    source, metadata, ulog, _ = load(path)
    flight = read_ulog(source, ulog=ulog)
    pipeline = analyze_control_pipeline(flight.bag)
    return source, metadata, flight, pipeline


def summary(path: Path, output: Path | None = None) -> str:
    """Produce the same multilayer numerical and graphical report as rosbags."""
    source, metadata, flight, pipeline = _analyze(path)
    native_se3 = len(flight.datasets.get("se3_trajectory_status", {}).get("timestamp", ())) >= 2
    profile = metadata.get("profile") or ("onboard_se3" if native_se3 else "px4_ulog")
    lines = [
        "PX4 ULog flight analysis", "========================",
        f"Source: {source}",
        f"Profile: {profile}",
        f"Firmware: {flight.firmware}",
        f"Timestamp domain: {flight.time_basis}",
        f"Dropout records: {flight.dropouts}",
        f"Selected topics with samples: {len(flight.datasets)}",
        "",
        *_position_overview(flight),
        *_flight_events(flight),
        *_pipeline_lines(flight, pipeline),
        *_coverage(flight),
        "",
        "Reference caution: missing desired signals are unavailable; no setpoints are inferred.",
    ]
    report = "\n".join(lines) + "\n"
    directory = output or _output_dir(Path(path))
    directory.mkdir(parents=True, exist_ok=True)
    try:
        figures = write_control_pipeline_plots(pipeline, directory)
    except RuntimeError as error:
        # Numerical report is still useful on machines without Matplotlib.
        figures = []
        report += f"Plot warning: {error}\n"
    if figures:
        report += "\nGenerated figures:\n"
        report += "".join(f"  {figure.name}\n" for figure in figures)
    (directory / "summary.txt").write_text(report)
    return report


def _series(pipeline: dict, category: str, name: str):
    return pipeline[category][name]


def _comparison_plots(runs: list[tuple[Path, dict]], output: Path) -> list[Path]:
    """Overlay observed and setpoint layers separately, using shared styles."""
    figures = []
    axes = (("position", "[m]"), ("velocity", "[m/s]"),
            ("acceleration", "[m/s²]"), ("attitude", "[deg]"),
            ("rates", "[deg/s]"), ("thrust", "[normalized]"),
            ("torque", "[normalized]"))
    for category, unit in axes:
        for source_type in ("actual", "controller setpoint", "trajectory", "setpoint"):
            if category in ("thrust", "torque") and source_type != "setpoint":
                continue
            components = []
            for axis, key in zip(("x", "y", "z"), ("x", "y", "z")):
                signals = {}
                for source, pipeline in runs:
                    group = pipeline[category]
                    if source_type not in group:
                        continue
                    series = group[source_type]
                    if series["times_s"]:
                        signals[source.stem] = (series["times_s"], series[key])
                if signals:
                    components.append((axis, signals))
            if not components:
                continue
            name = f"{category}_{source_type.replace(' ', '_')}_overlay.png"
            location = output / name
            save_tracking_plot(
                location, components=components,
                title=f"PX4 {category}: {source_type} comparison", unit=unit,
                xlabel="Time relative to ULog start [s]",
            )
            figures.append(location)
    # Per-channel motor overlays; channels may differ across configurations.
    channels = sorted(set().union(*(set(p["motors"]) for _, p in runs)))
    for channel in channels:
        signals = {}
        for source, pipeline in runs:
            if channel in pipeline["motors"]:
                signals[source.stem] = pipeline["motors"][channel]
        if signals:
            location = output / f"{channel.replace(' ', '_')}_overlay.png"
            save_tracking_plot(location, components=[(channel, signals)],
                               title=f"PX4 motor comparison: {channel}",
                               unit="[normalized]", xlabel="Time relative to ULog start [s]")
            figures.append(location)
    return figures


def compare(paths: list[Path], output: Path) -> str:
    """Compare compatible recorded signals, without assuming identical flights."""
    if len(paths) < 2:
        raise ValueError("comparison requires at least two logs")
    flights = [(Path(path), *_analyze(Path(path))) for path in paths]
    output.mkdir(parents=True, exist_ok=True)
    report = ["PX4 ULog control-pipeline comparison", "================================",
              "Overlays are aligned to recording start, not trajectory start.",
              "Tracking statistics within each flight use its own recorded references.",
              "Runs without a common plan must not be interpreted as matched maneuvers.", ""]
    for path, source, metadata, flight, pipeline in flights:
        report.extend((f"{source}: duration={flight.bag.duration_s:.3f}s, "
                       f"dropout_records={flight.dropouts}",
                       *control_pipeline_summary(pipeline), ""))
    figure_paths = _comparison_plots([(src, p) for _, src, _, _, p in flights], output)
    report.append(f"Generated overlay figures: {len(figure_paths)}")
    text = "\n".join(report) + "\n"
    (output / "summary.txt").write_text(text)
    return text
