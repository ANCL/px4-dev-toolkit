"""Analyze PX4 Position-mode takeoff, hover, and optional landing."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

from analysis.core import (
    BagData,
    TimedSample,
    first_nav_state_time,
    mode_transitions,
    native_constant_names,
)
from analysis.pipeline import (
    CONTROL_PIPELINE_TOPICS,
    analyze_control_pipeline,
    control_pipeline_summary,
    write_control_pipeline_plots,
)
from analysis.plots import save_mode_timeline
from analysis.px4 import TOPICS


PROFILE_NAME = "position_takeoff_hover"

STATUS_TOPIC = TOPICS["OUT_VEHICLE_STATUS_V1"]
LOCAL_POSITION_TOPIC = TOPICS["OUT_VEHICLE_LOCAL_POSITION"]

REQUIRED_TOPICS = {
    STATUS_TOPIC,
    LOCAL_POSITION_TOPIC,
    *CONTROL_PIPELINE_TOPICS,
}


def _finish_action(bag: BagData) -> str:
    """Read recorded finish action; older runs used automatic landing."""
    metadata = bag.path / "experiment.json"
    finish = "land"

    if metadata.is_file():
        config = json.loads(metadata.read_text()).get("config", {})
        finish = config.get("finish", "land")

    if finish not in ("hold", "land"):
        raise RuntimeError(f"Unknown Position-mode finish action: {finish}")

    return finish

@dataclass
class AnalysisResult:
    """Profile output consumed by the global analyzer CLI."""

    summary: str
    metrics: dict[str, float]
    plot_data: dict[str, object]


def _first_arming_state_time(
    samples: list[TimedSample],
    state: int,
    *,
    not_before_ns: int | None = None,
) -> int | None:
    """Find the first occurrence of one PX4 arming state."""

    for sample in samples:
        if (
            not_before_ns is not None
            and sample.timestamp_ns < not_before_ns
        ):
            continue

        if int(sample.message.arming_state) == state:
            return sample.timestamp_ns

    return None


def _first_landing_state_time(
    samples: list[TimedSample],
    status_type: type,
    *,
    not_before_ns: int,
) -> tuple[int, int] | None:
    """Find PX4 Land mode or its local Descend fallback."""

    landing_states = {
        int(status_type.NAVIGATION_STATE_AUTO_LAND),
        int(status_type.NAVIGATION_STATE_DESCEND),
    }

    for sample in samples:
        if sample.timestamp_ns < not_before_ns:
            continue

        state = int(sample.message.nav_state)

        if state in landing_states:
            return sample.timestamp_ns, state

    return None


def analyze(bag: BagData) -> AnalysisResult:
    """Analyze one native Position-mode takeoff/hover experiment.

    Inputs:
        BagData containing PX4 lifecycle, local-position, and pipeline topics.

    Method:
        Resolve confirmed Position/arming events and the recorded finish;
        require landing/disarm only if finish=land.

    Returns:
        Lifecycle/hover metrics, textual summary, and plot-ready pipeline data.
    """

    status_samples = bag.samples[STATUS_TOPIC]
    local_position_samples = bag.samples[LOCAL_POSITION_TOPIC]

    if not status_samples:
        raise RuntimeError(f"No samples recorded on {STATUS_TOPIC}")

    if not local_position_samples:
        raise RuntimeError(
            f"No samples recorded on {LOCAL_POSITION_TOPIC}"
        )

    status_type = type(status_samples[0].message)

    position_state = int(
        status_type.NAVIGATION_STATE_POSCTL
    )
    armed_state = int(
        status_type.ARMING_STATE_ARMED
    )
    disarmed_state = int(
        status_type.ARMING_STATE_DISARMED
    )

    position_entry_ns = first_nav_state_time(
        status_samples,
        position_state,
    )

    if position_entry_ns is None:
        raise RuntimeError("Position-mode entry was not recorded")

    armed_ns = _first_arming_state_time(
        status_samples,
        armed_state,
        not_before_ns=position_entry_ns,
    )

    if armed_ns is None:
        raise RuntimeError("Armed state was not recorded")

    finish = _finish_action(bag)
    metrics = {
        "bag_duration_s": bag.duration_s,
        "position_entry_s": bag.relative_seconds(position_entry_ns),
        "armed_s": bag.relative_seconds(armed_ns),
    }

    if finish == "land":
        landing = _first_landing_state_time(
            status_samples, status_type, not_before_ns=armed_ns,
        )
        if landing is None:
            raise RuntimeError("PX4 Land/Descend entry after hover was not recorded")

        landing_ns, landing_state = landing
        disarmed_ns = _first_arming_state_time(
            status_samples, disarmed_state, not_before_ns=landing_ns,
        )
        if disarmed_ns is None:
            raise RuntimeError("Final disarmed state was not recorded")

        metrics["landing_entry_s"] = bag.relative_seconds(landing_ns)
        metrics["disarmed_s"] = bag.relative_seconds(disarmed_ns)

    else:
        final_status = status_samples[-1]
        if (int(final_status.message.nav_state) != position_state or
                int(final_status.message.arming_state) != armed_state):
            raise RuntimeError("Hold recording did not end armed in Position mode")
        metrics["hold_end_s"] = bag.relative_seconds(final_status.timestamp_ns)

    state_names = native_constant_names(
        status_type,
        "NAVIGATION_STATE_",
    )
    if finish == "land":
        landing_name = state_names.get(
            landing_state,
            f"STATE_{landing_state}",
        )
    transitions = mode_transitions(status_samples)
    pipeline = analyze_control_pipeline(bag)

    lines = [
        f"Profile: {PROFILE_NAME}",
        f"Bag: {bag.path}",
        f"Bag duration: {metrics['bag_duration_s']:.3f} s",
        "",
        "PX4 navigation-state transitions:",
    ]

    for timestamp_ns, state in transitions:
        lines.append(
            f"  {bag.relative_seconds(timestamp_ns):8.3f} s  "
            f"{state_names.get(state, f'STATE_{state}')} ({state})"
        )

    lines.extend([
        "",
        "Position-mode flight:",
        f"  Position-mode entry: {metrics['position_entry_s']:.3f} s",
        f"  armed: {metrics['armed_s']:.3f} s",
    ])

    markers = [
        (metrics["position_entry_s"], "POSCTL ENTRY"),
        (metrics["armed_s"], "ARMED"),
    ]

    if finish == "land":
        lines.extend([
            f"  native landing entry: {metrics['landing_entry_s']:.3f} s ({landing_name})",
            f"  disarmed: {metrics['disarmed_s']:.3f} s",
        ])
        markers.append((metrics["landing_entry_s"], "LAND"))

    else:
        lines.append(f"  Position-mode hold confirmed: {metrics['hold_end_s']:.3f} s")
        lines.append("  Vehicle remains armed; land separately after recording")

    lines.extend(["", *control_pipeline_summary(pipeline)])
    return AnalysisResult(
        summary="\n".join(lines) + "\n",
        metrics=metrics,
        plot_data={
            "pipeline": pipeline,
            "markers": markers,
            "state_names": state_names,
            "transitions": [
                (bag.relative_seconds(timestamp_ns), state)
                for timestamp_ns, state in transitions
            ],
        },
    )


def write_plots(
    result: AnalysisResult,
    output_dir: Path,
) -> list[Path]:
    """Write common PX4 pipeline plots plus navigation-state history."""

    output_dir.mkdir(parents=True, exist_ok=True)

    # Each analyzer run owns its generated figures. Remove stale PNGs before
    # writing the report so output never mixes figures from separate analyses.
    for old_plot in output_dir.glob("*.png"):
        old_plot.unlink()

    generated = write_control_pipeline_plots(
        result.plot_data["pipeline"],
        output_dir,
        markers=result.plot_data["markers"],
    )

    mode_path = output_dir / "09_mode_timeline.png"
    save_mode_timeline(
        mode_path,
        transitions=result.plot_data["transitions"],
        state_names=result.plot_data["state_names"],
    )
    generated.append(mode_path)

    return generated
