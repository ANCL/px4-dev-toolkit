"""Profile-independent, trajectory-time comparison of standard PX4 telemetry.

The same recorded PX4 state and controller topics are processed for every run.
Only the reference source and execution-start evidence vary by recording:
  * a confirmed Offboard transition starts ROS Offboard trajectories;
  * a native SE3 recording needs an explicit event, or a strictly validated
    reference-fit estimate for legacy bags without the event.

A reference fit is an *estimate*, never represented as an exact PX4 event.
No assumptions about specific trajectory names or specific maneuver steps.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path

import numpy as np

from analysis.comparison import (
    shift_motor_series,
    shift_vector_series,
    tracking_error_series,
)
from analysis.core import BagData, read_bag
from analysis.pipeline import (
    CONTROL_PIPELINE_TOPICS,
    analyze_control_pipeline,
    _series3,
    _attitude_series,
)
from analysis.px4 import TOPICS
from analysis.profiles import read_experiment, get_profile
from analysis.profiles.se3.comparison.model import ComparisonRun, trajectory_segments
from analysis.profiles.se3.comparison.plotting import write_comparison_plots
from analysis.profiles.se3.comparison.reporting import comparison_summary

STATUS = TOPICS["OUT_VEHICLE_STATUS_V1"]
# Observation and setpoint source selection belongs to existing experiment
# profiles, not the common comparison engine. Every profile returns topic names
# and explicit message field conventions; no guessed fallback to another layer.

def comparison_sources(metadata: dict) -> dict:
    profile = get_profile(str(metadata.get("profile", "")))
    resolver = getattr(profile, "comparison_signals", None)
    if not callable(resolver):
        raise ValueError(f"Profile {metadata.get('profile')!r} does not define comparison signals")
    sources = resolver(metadata)
    if not isinstance(sources, dict) or not isinstance(sources.get("trajectory"), str):
        raise ValueError("Invalid comparison source definition: missing trajectory topic")
    return sources


def source_topics(sources: dict) -> set[str]:
    """List exact topic names declared by a profile (including optional signals)."""
    result = set()
    for source in sources.values():
        if isinstance(source, str):
            result.add(source)
        elif isinstance(source, tuple) and len(source) == 2:
            result.add(source[0])
        elif source is not None:
            raise ValueError(f"Unsupported comparison source specification: {source!r}")
    return result


@dataclass
class RawRun:
    path: Path
    metadata: dict
    bag: BagData
    pipeline: dict
    position_reference: dict
    velocity_reference: dict
    yaw_reference: dict
    mode_start_s: float
    mode_end_s: float
    mode: str
    reference_topic: str
    origin_s: float | None = None
    start_evidence: str = ""
    match_position_rms_m: float = float("nan")
    match_yaw_rms_deg: float = float("nan")
    sources: dict = field(default_factory=dict)


def _mode_interval(bag: BagData, *, native: bool) -> tuple[float, float]:
    """Identify a single continuous selected-control interval from PX4 state."""
    states = bag.samples.get(STATUS, [])
    if not states:
        raise ValueError("Missing recorded PX4 vehicle_status_v1")
    status_class = type(states[0].message)
    offboard = int(status_class.NAVIGATION_STATE_OFFBOARD)

    # Reuse the native-mode predicate owned by the onboard analysis profile.
    # Resolve once, rather than repeatedly importing during status iteration.
    native_selected = get_profile("onboard_se3")._external if native else None

    def selected(sample):
        nav = int(sample.message.nav_state)
        return native_selected(nav) if native_selected is not None else nav == offboard

    start = end = None
    entered = False
    for sample in states:
        if selected(sample):
            if not entered:
                if start is not None:
                    raise ValueError("Multiple control intervals; select one complete experiment bag")
                start = sample.timestamp_ns
                entered = True
        elif entered:
            end = sample.timestamp_ns
            entered = False
    if entered:
        end = states[-1].timestamp_ns
    if start is None or end is None or end <= start:
        raise ValueError("No complete native SE3/Offboard control interval")
    return bag.relative_seconds(start), bag.relative_seconds(end)


def _selected_bag(bag: BagData, start: float, end: float) -> BagData:
    start_ns = bag.start_ns + int(round(start * 1e9))
    end_ns = bag.start_ns + int(round(end * 1e9))
    return BagData(bag.path, bag.start_ns, bag.end_ns, {
        topic: [s for s in samples if start_ns <= s.timestamp_ns <= end_ns]
        for topic, samples in bag.samples.items()
    })


def _reference(bag: BagData, topic: str):
    """Extract only the trajectory reference declared by this experiment profile."""
    if not bag.samples.get(topic):
        raise ValueError(f"Missing profile-declared trajectory reference topic: {topic}")
    position = _series3(bag, topic, lambda msg: msg.position)
    velocity = _series3(bag, topic, lambda msg: msg.velocity)
    yaw = _series3(bag, topic, lambda msg: (0.0, 0.0, msg.yaw),
                   scale=180.0 / math.pi)
    if min(len(position["times_s"]), len(velocity["times_s"]),
           len(yaw["times_s"])) <= 20:
        raise ValueError(f"Incomplete trajectory reference (position, velocity, yaw): {topic}")
    return position, velocity, yaw


def load_run(path: Path) -> RawRun:
    metadata = read_experiment(path)
    sources = comparison_sources(metadata)
    native = metadata["profile"] == "onboard_se3"
    bag = read_bag(path, {STATUS}, CONTROL_PIPELINE_TOPICS | source_topics(sources))
    mode_start, mode_end = _mode_interval(bag, native=native)
    sliced = _selected_bag(bag, mode_start, mode_end)
    position, velocity, yaw = _reference(sliced, sources["trajectory"])
    pipeline = analyze_control_pipeline(sliced)
    return RawRun(path=path, metadata=metadata, bag=sliced,
                  pipeline=pipeline, position_reference=position,
                  velocity_reference=velocity, yaw_reference=yaw,
                  mode_start_s=mode_start, mode_end_s=mode_end,
                  mode="native SE3" if native else "Offboard",
                  reference_topic=sources["trajectory"], sources=sources,
                  origin_s=None if native else mode_start,
                  start_evidence=("inferred reference fit" if native
                                  else "confirmed PX4 Offboard entry"))


def _optional_attitude(bag: BagData | None, spec) -> dict:
    """Return desired attitude only from the profile's declared quaternion topic."""
    if bag is None or spec is None:
        return _series_or_empty(None)
    topic, field_name = spec
    if not bag.samples.get(topic):
        return _series_or_empty(None)
    if field_name != "q_d":
        raise ValueError(f"Unsupported attitude field specification: {field_name}")
    return _attitude_series(bag, topic, field_name)


def _optional_rate(bag: BagData | None, spec) -> dict:
    """Decode rad/s into deg/s, with the profile specifying message layout."""
    if bag is None or spec is None:
        return _series_or_empty(None)
    topic, field_name = spec
    if not bag.samples.get(topic):
        return _series_or_empty(None)
    if field_name == "rpy":
        getter = lambda msg: (msg.roll, msg.pitch, msg.yaw)
    elif field_name == "xyz":
        getter = lambda msg: (msg.x, msg.y, msg.z)
    else:
        raise ValueError(f"Unsupported body-rate field specification: {field_name}")
    return _series3(bag, topic, getter, scale=180.0 / math.pi)


def _interp(series: dict, component: str, t: np.ndarray, *, angular: bool = False) -> np.ndarray:
    times = np.asarray(series["times_s"], dtype=float)
    values = np.asarray(series[component], dtype=float)
    if times.size < 2:
        raise ValueError(f"Insufficient {component} reference")
    order = np.argsort(times, kind="stable")
    times, values = times[order], values[order]
    times, unique = np.unique(times, return_index=True)
    values = values[unique]
    if angular:
        values = np.rad2deg(np.unwrap(np.deg2rad(values)))
    if t.min() < times[0] - 0.015 or t.max() > times[-1] + 0.015:
        raise ValueError(f"Recorded {component} reference does not cover the trajectory interval")
    return np.interp(t, times, values)


def _reference_differences(template: RawRun, target: RawRun,
                           template_start: float, target_start: float,
                           duration: float) -> tuple[float, float]:
    """Check independently logged trajectories on the common motion timeline."""
    # Omit samples immediately at the limits to allow one dropped DDS sample.
    relative = np.arange(0.25, duration - 0.50, 0.25)
    if relative.size < 15:
        raise ValueError("Trajectory too short for validated reference alignment")
    t0, t1 = template_start + relative, target_start + relative
    residuals = []
    for key in ("x", "y", "z"):
        residuals.append(_interp(template.position_reference, key, t0)
                         - _interp(target.position_reference, key, t1))
    pos_rms = float(np.sqrt(np.mean(np.sum(np.square(residuals), axis=0))))
    yaw = _interp(template.yaw_reference, "z", t0, angular=True) - _interp(target.yaw_reference, "z", t1, angular=True)
    yaw = (yaw + 180.0) % 360.0 - 180.0
    yaw_rms = float(np.sqrt(np.mean(np.square(yaw))))
    return pos_rms, yaw_rms


def infer_start(template: RawRun, target: RawRun, duration: float) -> tuple[float, float, float]:
    """Infer native execution onset from the whole recorded reference sequence.

    This works for arbitrary sufficiently distinctive paths and includes their
    initial hold. We do not assume a named trajectory or a particular X step.
    Abort on poor match, inadequate coverage, or ambiguous candidate shifts.
    """
    if template.origin_s is None:
        raise ValueError("A confirmed trajectory-start reference is required for inference")
    if target.mode_end_s - target.mode_start_s < duration - 0.50:
        raise ValueError("Native selected-mode interval is too short for the shared trajectory")

    first = max(target.mode_start_s, float(target.position_reference["times_s"][0]))
    last = min(target.mode_end_s, float(target.position_reference["times_s"][-1]))
    # Allow up to 0.55 s of end-reference transport gaps, not an arbitrary
    # partial maneuver. An observed initial hold is allowed before start.
    maximum = last - duration + 0.55
    if maximum < first - 0.06:
        raise ValueError("Not enough recorded reference to locate native trajectory start")
    maximum = max(first, maximum)

    # Score an entire path, not one characteristic step. A coarse grid keeps
    # longer trajectories efficient while retaining step/yaw/curve signatures.
    relative = np.arange(0.35, duration - 0.65, 0.30)
    reference_times = template.origin_s + relative
    desired = np.column_stack([
        _interp(template.position_reference, k, reference_times)
        for k in ("x", "y", "z")
    ])
    desired_yaw = _interp(template.yaw_reference, "z", reference_times, angular=True)
    if (np.max(np.linalg.norm(desired - desired[0], axis=1)) < 0.12
            and np.ptp(desired_yaw) < 10.0):
        raise ValueError("Reference is too constant to infer execution start")

    grid = np.arange(first, maximum + 1e-9, 0.05)
    if grid.size == 0 or grid[-1] < maximum - 0.015:
        grid = np.append(grid, maximum)

    target_times = np.asarray(target.position_reference["times_s"])
    target_yaw_times = np.asarray(target.yaw_reference["times_s"])
    target_yaw_values = np.rad2deg(
        np.unwrap(np.deg2rad(target.yaw_reference["z"]))
    )
    # Require that candidate end points are actually recorded and not silently
    # clamped by interpolation.
    candidates = []
    for candidate in grid:
        stamps = candidate + relative
        if (stamps[0] < target_times[0] or stamps[-1] > target_times[-1]
                or stamps[0] < target_yaw_times[0] or stamps[-1] > target_yaw_times[-1]):
            continue
        actual = np.column_stack([
            np.interp(stamps, target_times, target.position_reference[k])
            for k in ("x", "y", "z")
        ])
        actual_yaw = np.interp(stamps, target_yaw_times, target_yaw_values)
        pos_rms = float(np.sqrt(np.mean(np.sum((desired - actual) ** 2, axis=1))))
        yaw_error = (desired_yaw - actual_yaw + 180.0) % 360.0 - 180.0
        yaw_rms = float(np.sqrt(np.mean(yaw_error ** 2)))
        candidates.append((pos_rms + yaw_rms / 200.0, float(candidate)))
    if not candidates:
        raise ValueError("No complete native reference window supports trajectory alignment")
    candidates.sort()
    _, onset = candidates[0]
    pos_rms, yaw_rms = _reference_differences(
        template, target, template.origin_s, onset, duration)
    if pos_rms > 0.12 or yaw_rms > 10.0:
        raise ValueError(f"Native trajectory reference does not match: position RMS={pos_rms:.3f} m, "
                         f"yaw RMS={yaw_rms:.2f} deg")

    # A distinctly separate alignment with virtually equal score means the
    # trajectory does not uniquely identify when the initial hold began.
    alternative = [score for score, t in candidates if abs(t - onset) >= 0.35]
    if alternative and min(alternative) < candidates[0][0] + 0.020:
        raise ValueError("Ambiguous native trajectory start: multiple similar reference fits")
    return onset, pos_rms, yaw_rms


def _series_or_empty(series: dict | None) -> dict:
    return series if series is not None else {"times_s": [], "x": [], "y": [], "z": []}


def build_common_run(raw: RawRun, label: str, duration: float) -> ComparisonRun:
    """Produce the existing common comparison schema from recorded PX4 topics."""
    origin = raw.origin_s
    if origin is None:
        raise ValueError("Cannot build comparison without an accepted trajectory start")
    pipeline = raw.pipeline
    attitude = pipeline["attitude"]["actual"]
    # A missing declared observation remains unavailable. Never substitute
    # downstream PX4 output, another controller, or a fabricated zero signal.
    sources = raw.sources or (comparison_sources(raw.metadata) if raw.bag is not None else {})
    attitude_sp = _optional_attitude(raw.bag, sources.get("attitude"))
    rate_sp = _optional_rate(raw.bag, sources.get("rates"))

    def shifted(series):
        return shift_vector_series(_series_or_empty(series), origin, duration)

    def error(measured, reference, *, yaw_only=False, angular=False):
        return tracking_error_series(measured, reference, time_origin_s=origin,
                                     end_time_s=origin + duration,
                                     components=("z",) if yaw_only else ("x", "y", "z"),
                                     wrap_degrees=angular)

    p = pipeline["position"]["actual"]
    v = pipeline["velocity"]["actual"]
    rates = pipeline["rates"]["actual"]
    return ComparisonRun(
        label=label, bag_path=raw.path,
        handoff=str(raw.metadata.get("handoff", "onboard_rate")),
        direct_controller=str(raw.metadata.get("direct_controller", "native_se3"
                          if raw.mode == "native SE3" else "PX4")),
        duration_s=duration,
        position_actual=shifted(p),
        position_reference=shifted(raw.position_reference),
        position_error=error(p, raw.position_reference),
        velocity_actual=shifted(v),
        velocity_reference=shifted(raw.velocity_reference),
        velocity_error=error(v, raw.velocity_reference),
        attitude_actual=shifted(attitude),
        attitude_reference=shifted(attitude_sp),
        attitude_error=error(attitude, attitude_sp, angular=True),
        yaw_actual=shifted(attitude),
        yaw_reference=shifted(raw.yaw_reference),
        yaw_error=error(attitude, raw.yaw_reference, yaw_only=True, angular=True),
        thrust=shifted(pipeline["thrust"]["setpoint"]),
        torque=shifted(pipeline["torque"]["setpoint"]),
        body_rates=shifted(rates),
        body_rate_reference=shifted(rate_sp),
        body_rate_error=error(rates, rate_sp),
        motors=shift_motor_series(pipeline["motors"], origin, duration),
    )


def _configuration(runs: list[RawRun]):
    """Use independently saved trajectory config, never infer it from a label."""
    configurations = [r.metadata.get("config", {}).get("trajectory") for r in runs]
    known = [c for c in configurations if isinstance(c, dict)]
    if not known:
        raise ValueError("No embedded trajectory configuration to determine start/duration; "
                         "record an explicit start event or provide a compatible reference bag")
    if any(c != known[0] for c in known[1:]):
        raise ValueError("Recorded trajectory configurations differ; cannot compare")
    segments = trajectory_segments(known[0])
    duration = float(segments[-1]["end_s"]) if segments else 0.0
    if duration < 3.0:
        raise ValueError("Recorded trajectory has invalid/too short duration")
    return segments, duration


def compare_bags(paths: list[Path], output_dir: Path) -> None:
    """Compare arbitrary runs with a common recorded trajectory and clock."""
    if len(paths) < 2:
        raise ValueError("Specify at least two ROS bags")
    runs = [load_run(path) for path in paths]
    segments, duration = _configuration(runs)
    template = next((r for r in runs if r.origin_s is not None), None)
    if template is None:
        raise ValueError("At least one run needs confirmed trajectory-start evidence "
                         "to align legacy recordings")

    from analysis.comparison import unique_labels
    profile_labels = []
    for run in runs:
        info = run.metadata
        controller = info.get("handoff") or info.get("profile")
        if info.get("direct_controller"):
            controller += f"/{info['direct_controller']}"
        profile_labels.append(str(controller))
    labels = unique_labels(profile_labels, paths)

    for run in runs:
        if run.origin_s is None:
            run.origin_s, run.match_position_rms_m, run.match_yaw_rms_deg = (
                infer_start(template, run, duration)
            )
        else:
            run.match_position_rms_m, run.match_yaw_rms_deg = _reference_differences(
                template, run, template.origin_s, run.origin_s, duration)
        if run.match_position_rms_m > 0.12 or run.match_yaw_rms_deg > 10.0:
            raise ValueError(f"Trajectory reference mismatch for {run.path}: "
                             f"position RMS={run.match_position_rms_m:.3f} m, "
                             f"yaw RMS={run.match_yaw_rms_deg:.2f} deg")

    # Compare the same commanded trajectory duration for every run. Do not
    # include controller prestream, SE3 idle hold, or post-run Position hold.
    for run in runs:
        if run.mode_end_s - float(run.origin_s) < duration - 0.75:
            raise ValueError(f"Recording ended before the comparable trajectory completed: {run.path}")

    normalized = [build_common_run(run, label, duration)
                  for run, label in zip(runs, labels)]
    original_summary = comparison_summary(normalized, segments)
    # The existing metric engine is reused unchanged; update only its obsolete
    # Offboard-specific description for this trajectory-aligned presentation.
    original_summary = original_summary.replace(
        "alignment: PX4 Offboard entry = 0 s",
        "alignment: trajectory start = 0 s (per-run provenance below)",
    ).replace("Offboard interval", "common trajectory interval")

    details = ["Trajectory-start evidence", "-------------------------"]
    for run, label in zip(runs, labels):
        details.extend([
            f"{label}: {run.path}",
            f"  mode: {run.mode}",
            f"  reference: {run.reference_topic}",
            f"  attitude source: {run.sources.get('attitude') or 'unavailable'}",
            f"  rate command source: {run.sources.get('rates') or 'unavailable'}",
            f"  start time from bag beginning: {run.origin_s:.3f} s",
            f"  source: {run.start_evidence}",
            (f"  reference agreement: position RMS {run.match_position_rms_m:.4f} m, "
             f"yaw RMS {run.match_yaw_rms_deg:.3f} deg"),
        ])
    details += ["", f"Common trajectory interval: 0 .. {duration:.3f} s", ""]
    if any(r.start_evidence.startswith("inferred") for r in runs):
        details.extend([
            "CAUTION: Legacy native start was estimated from recorded setpoints.",
            "This is not an observed execution-phase event or CRC validation.",
            "Do not interpret sub-sample timing or the inferred start as exact.", "",
        ])
    summary = original_summary.rstrip() + "\n\n" + "\n".join(details) + "\n"

    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.txt").write_text(summary)
    print(summary, end="")
    print(f"Summary: {output_dir / 'summary.txt'}")
    for path in write_comparison_plots(
        normalized, output_dir, segments, xlabel="Time from trajectory start [s]"
    ):
        print(f"Plot: {path}")
