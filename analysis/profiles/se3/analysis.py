"""Orchestrate SE(3) flight-data analysis on the recorded bag clock.

Lifecycle establishes PX4-confirmed Offboard entry first; extraction gathers
actual references, PX4 pipeline observations, and optional diagnostics;
reporting and plots consume those results without altering their time origins.
"""

from __future__ import annotations

from analysis.core import (
    BagData,
    first_nav_state_time,
    mode_transitions,
    native_constant_names,
)
from analysis.pipeline import analyze_control_pipeline
from .model import (
    AnalysisResult,
    DIAGNOSTIC_TOPICS,
    OFFBOARD_MODE_TOPIC,
    PROFILE_NAME,
    STATUS_TOPIC,
    TRAJECTORY_REFERENCE_TOPIC,
)
from .lifecycle import (
    _direct_controller_name,
    _first_armed_time,
    _handoff_samples,
    _required,
)
from .reporting import _geometric_physical_summary, _se3_summary
from .diagnostics import (
    extract_common_diagnostics,
    extract_geometric_normalized_diagnostics,
    extract_geometric_physical_diagnostics,
    extract_px4_rate_diagnostics,
)
from .extraction import extract_se3_layers


def analyze(bag: BagData) -> AnalysisResult:
    """Analyze one recorded SE(3) Offboard run.

    Inputs:
        BagData containing PX4 lifecycle, OffboardControlMode, generated
        reference, handoff, native pipeline, and optional diagnostic topics.

    Method:
        Resolve the authoritative handoff and direct-controller identity,
        measure lifecycle events on the rosbag clock, then extract controller
        and PX4 pipeline data beginning at confirmed Offboard entry.

    Returns:
        Lifecycle metrics, human-readable summary, and structured data used by
        plots and Offboard-interval performance comparison.
    """

    status = _required(bag, STATUS_TOPIC)
    offboard_mode = _required(bag, OFFBOARD_MODE_TOPIC)
    handoff_mode, handoff_inputs = _handoff_samples(
        bag,
        offboard_mode,
    )

    direct_controller = _direct_controller_name(
        bag,
        handoff_mode,
    )

    handoff_topics = [topic for topic, _ in handoff_inputs]
    handoff_topic = " + ".join(handoff_topics)
    reference = _required(
        bag,
        TRAJECTORY_REFERENCE_TOPIC,
    )

    status_type = type(status[0].message)

    armed_state = int(
        status_type.ARMING_STATE_ARMED
    )
    offboard_state = int(
        status_type.NAVIGATION_STATE_OFFBOARD
    )

    position_state = int(
        status_type.NAVIGATION_STATE_POSCTL
    )

    started_armed = (
        int(status[0].message.arming_state)
        == armed_state
    )

    armed_ns = _first_armed_time(
        status,
        armed_state,
    )

    if armed_ns is None:
        raise RuntimeError(
            "Armed state was not recorded"
        )

    offboard_ns = first_nav_state_time(
        status,
        offboard_state,
    )

    if offboard_ns is None:
        raise RuntimeError(
            "Offboard-mode entry was not recorded"
        )

    position_return_ns = first_nav_state_time(
        status,
        position_state,
        not_before_ns=offboard_ns,
    )

    if position_return_ns is None:
        raise RuntimeError(
            "Position-mode return after Offboard was not recorded"
        )

    offboard_mode_start_ns = offboard_mode[0].timestamp_ns
    reference_start_ns = reference[0].timestamp_ns
    handoff_start_ns = min(
        samples[0].timestamp_ns
        for _, samples in handoff_inputs
    )

    if offboard_mode_start_ns >= offboard_ns:
        raise RuntimeError(
            "OffboardControlMode prestream did not precede "
            "Offboard entry"
        )

    if reference_start_ns >= offboard_ns:
        raise RuntimeError(
            "Generated trajectory-reference prestream did not "
            "precede Offboard entry"
        )

    metrics = {
        "bag_duration_s": bag.duration_s,
        "handoff": handoff_mode,
        "handoff_topic": handoff_topic,
        "direct_controller": direct_controller or "n/a",
        "started_armed": started_armed,
        "armed_s": bag.relative_seconds(armed_ns),
        "offboard_mode_start_s": bag.relative_seconds(
            offboard_mode_start_ns
        ),
        "reference_start_s": bag.relative_seconds(
            reference_start_ns
        ),
        "offboard_entry_s": bag.relative_seconds(
            offboard_ns
        ),
        "handoff_start_s": bag.relative_seconds(
            handoff_start_ns
        ),
        "position_return_s": bag.relative_seconds(
            position_return_ns
        ),
        "offboard_mode_prestream_s": (
            offboard_ns - offboard_mode_start_ns
        ) / 1e9,
        "reference_prestream_s": (
            offboard_ns - reference_start_ns
        ) / 1e9,
        "handoff_offset_from_offboard_s": (
            handoff_start_ns - offboard_ns
        ) / 1e9,
        "offboard_duration_s": (
            position_return_ns - offboard_ns
        ) / 1e9,
    }

    state_names = native_constant_names(
        status_type,
        "NAVIGATION_STATE_",
    )

    transitions = mode_transitions(status)

    # Controller/pipeline extraction begins only after PX4 confirms Offboard.
    # Lifecycle analysis above still uses the complete bag. Performance
    # comparison later clips these series to the recorded Offboard interval.
    tracking_bag = BagData(
        path=bag.path,
        start_ns=bag.start_ns,
        end_ns=bag.end_ns,
        samples={
            topic: [
                sample
                for sample in samples
                if sample.timestamp_ns >= offboard_ns
            ]
            for topic, samples in bag.samples.items()
        },
    )

    layers = extract_se3_layers(
        tracking_bag,
        handoff_mode,
    )

    # Keep the native PX4 pipeline on its native topics. In acceleration
    # handoff, OUT_TRAJECTORY_SETPOINT is not the toolkit's generated
    # geometric trajectory reference.
    pipeline = analyze_control_pipeline(
        tracking_bag
    )

    diagnostics = {
        "common": extract_common_diagnostics(
            tracking_bag
        ),
        "geometric_normalized":
            extract_geometric_normalized_diagnostics(
                tracking_bag
            ),
        "px4_rate":
            extract_px4_rate_diagnostics(
                tracking_bag
            ),
        "geometric_physical":
            extract_geometric_physical_diagnostics(
                tracking_bag
            ),
    }

    diagnostics_present = any(
        tracking_bag.samples.get(topic)
        for topic in DIAGNOSTIC_TOPICS
    )

    lines = [
        f"Profile: {PROFILE_NAME}",
        f"Bag: {bag.path}",
        f"Bag duration: {bag.duration_s:.3f} s",
        "",
        "PX4 navigation-state transitions:",
    ]

    for timestamp_ns, state in transitions:
        lines.append(
            f"  {bag.relative_seconds(timestamp_ns):8.3f} s  "
            f"{state_names.get(state, f'STATE_{state}')} ({state})"
        )

    lines.extend(
        [
            "",
            "SE3 Offboard lifecycle:",
            f"  handoff input: {handoff_topic}",
            *(
                [f"  direct controller: {direct_controller}"]
                if direct_controller is not None
                else []
            ),
            f"  started armed: "
            f"{'yes' if started_armed else 'no'}",
            f"  armed state first recorded: "
            f"{metrics['armed_s']:.3f} s",
            f"  OffboardControlMode start: "
            f"{metrics['offboard_mode_start_s']:.3f} s",
            f"  generated reference start: "
            f"{metrics['reference_start_s']:.3f} s",
            f"  Offboard entry: "
            f"{metrics['offboard_entry_s']:.3f} s",
            f"  OffboardControlMode before entry: "
            f"{metrics['offboard_mode_prestream_s']:.3f} s",
            f"  generated reference before entry: "
            f"{metrics['reference_prestream_s']:.3f} s",
            f"  selected handoff start: "
            f"{metrics['handoff_start_s']:.3f} s",
            f"  handoff offset from Offboard entry: "
            f"{metrics['handoff_offset_from_offboard_s']:+.3f} s",
            f"  Position return: "
            f"{metrics['position_return_s']:.3f} s",
            f"  time in Offboard: "
            f"{metrics['offboard_duration_s']:.3f} s",
            "",
            *_se3_summary(
                layers,
                pipeline,
                handoff_mode,
                direct_controller,
            ),
        ]
    )

    lines.extend(
        [
            "",
            (
                "Internal diagnostics: "
                + (
                    "recorded"
                    if diagnostics_present
                    else "not recorded"
                )
            ),
        ]
    )

    if direct_controller == "geometric_physical":
        lines.extend(
            [
                "",
                *_geometric_physical_summary(
                    diagnostics["geometric_physical"]
                ),
            ]
        )

    markers = [
        (
            metrics["reference_start_s"],
            "REFERENCE PRESTREAM",
        ),
        (
            metrics["offboard_entry_s"],
            "OFFBOARD ENTRY",
        ),
        (
            metrics["handoff_start_s"],
            "HANDOFF START",
        ),
        (
            metrics["position_return_s"],
            "POSITION RETURN",
        ),
    ]

    if not started_armed:
        markers.insert(
            0,
            (
                metrics["armed_s"],
                "ARMED",
            ),
        )

    return AnalysisResult(
        summary="\n".join(lines) + "\n",
        metrics=metrics,
        plot_data={
            "layers": layers,
            "pipeline": pipeline,
            "diagnostics": diagnostics,
            "diagnostics_present":
                diagnostics_present,
            "handoff_mode": handoff_mode,
            "markers": markers,
            "state_names": state_names,
            "transitions": [
                (
                    bag.relative_seconds(timestamp_ns),
                    state,
                )
                for timestamp_ns, state in transitions
            ],
        },
    )
