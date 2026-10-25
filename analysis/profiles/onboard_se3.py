"""ROS-bag view of onboard SE3 using standard PX4 uORB/DDS topics.

The ULog-only execution phase/run ID is deliberately not guessed from standard
trajectory_setpoint samples. This profile reports the entire active SE3-mode
interval (including hold) and uses the existing PX4 pipeline calculations.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from analysis.core import BagData, TimedSample
from analysis.pipeline import (
    CONTROL_PIPELINE_TOPICS, analyze_control_pipeline,
    write_control_pipeline_plots,
)
from analysis.pipeline.statistics import _aligned_error_stats
from analysis.px4 import TOPICS

PROFILE_NAME = 'onboard_se3'
STATUS = TOPICS['OUT_VEHICLE_STATUS_V1']
TRAJECTORY = TOPICS['OUT_TRAJECTORY_SETPOINT']
POSITION = TOPICS['OUT_VEHICLE_LOCAL_POSITION']
RATE = TOPICS['OUT_VEHICLE_RATES_SETPOINT']
REQUIRED_TOPICS = {STATUS, TRAJECTORY, POSITION, RATE}
OPTIONAL_TOPICS = CONTROL_PIPELINE_TOPICS - REQUIRED_TOPICS


@dataclass
class AnalysisResult:
    summary: str
    metrics: dict[str, float]
    plot_data: dict[str, object]


def _external(state: int) -> bool:
    # Registered PX4 external flight modes are NAVIGATION_STATE_EXTERNAL1..8.
    return 23 <= state <= 30


def _active_interval(status: list[TimedSample]) -> tuple[int, int]:
    if not status:
        raise ValueError('vehicle_status_v1 has no samples')
    entered = None
    for sample in status:
        if _external(int(sample.message.nav_state)):
            if entered is None:
                entered = sample.timestamp_ns
        elif entered is not None:
            return entered, sample.timestamp_ns
    if entered is not None:
        return entered, status[-1].timestamp_ns
    raise ValueError('No PX4 external-mode interval found; was SE3 selected?')



def _vector_tracking(actual: dict, reference: dict) -> tuple[float, float] | None:
    """Time-weighted vector RMS/peak using the shared finite PX4 pipeline streams."""
    ta = np.asarray(actual['times_s'], dtype=float)
    tr = np.asarray(reference['times_s'], dtype=float)
    if len(ta) < 2 or len(tr) < 2:
        return None
    a = np.column_stack([actual[axis] for axis in 'xyz'])
    r = np.column_stack([reference[axis] for axis in 'xyz'])
    order = np.argsort(tr, kind='stable')
    tr, unique = np.unique(tr[order], return_index=True)
    r = r[order][unique]
    mask = (ta >= tr[0]) & (ta <= tr[-1])
    ta, a = ta[mask], a[mask]
    if len(ta) < 2:
        return None
    wanted = np.column_stack([np.interp(ta, tr, r[:, i]) for i in range(3)])
    norm = np.linalg.norm(a - wanted, axis=1)
    dt = np.diff(ta)
    valid = np.isfinite(dt) & (dt > 0)
    if not np.any(valid):
        return None
    integral = np.sum(0.5 * (norm[:-1]**2 + norm[1:]**2)[valid] * dt[valid])
    return float(np.sqrt(integral / np.sum(dt[valid]))), float(np.max(norm))


def analyze(bag: BagData) -> AnalysisResult:
    """Analyze only the selected external-mode interval, not takeoff/landing."""
    start, end = _active_interval(bag.samples[STATUS])
    if start >= end:
        raise ValueError('External-mode interval has zero duration')
    sliced = BagData(bag.path, start, end, {
        name: [sample for sample in samples if start <= sample.timestamp_ns <= end]
        for name, samples in bag.samples.items()
    })
    for name in (TRAJECTORY, POSITION, RATE):
        if not sliced.samples[name]:
            raise ValueError(f'No onboard SE3 samples in active mode: {name}')
    pipeline = analyze_control_pipeline(sliced)
    pos = pipeline['position']
    vel = pipeline['velocity']
    rates = pipeline['rates']
    metrics = {'active_mode_s': (end - start) * 1e-9}
    lines = [
        'Profile: onboard_se3',
        f'ROS bag: {bag.path}',
        f'SE3 selected-mode duration: {metrics["active_mode_s"]:.3f} s',
        'Interval includes hold before/after the trajectory.',
        'ULog-only run phase and checksum are not inferred from DDS.',
        '',
    ]
    groups = (
        ('Position [m]', pos['actual'], pos['trajectory']),
        ('Velocity [m/s]', vel['actual'], vel['trajectory']),
        ('Attitude [deg]', pipeline['attitude']['actual'], pipeline['attitude']['setpoint']),
        ('Body rate [deg/s]', rates['actual'], rates['setpoint']),
    )
    for label, actual, reference in groups:
        lines.append(label)
        for axis in 'xyz':
            stats = _aligned_error_stats(
                actual, reference, axis,
                wrap_degrees=(label == 'Attitude [deg]' and axis == 'z'),
            )
            if stats is None:
                lines.append(f'  {axis}: unavailable')
            else:
                rms, peak = stats
                lines.append(f'  {axis}: RMS={rms:.5f} max={peak:.5f}')
                metrics[f'{label}_{axis}_rms'] = rms
        lines.append('')
    for label, actual, reference, unit, key in (
        ('Position vector', pos['actual'], pos['trajectory'], 'm', 'position'),
        ('Velocity vector', vel['actual'], vel['trajectory'], 'm/s', 'velocity'),
        ('Body-rate vector', rates['actual'], rates['setpoint'], 'deg/s', 'rates'),
    ):
        stats = _vector_tracking(actual, reference)
        if stats is None:
            lines.append(f'{label}: unavailable')
        else:
            rms, peak = stats
            metrics[f'{key}_vector_rms'] = rms
            metrics[f'{key}_vector_peak'] = peak
            lines.append(f'{label}: RMS={rms:.5f} {unit}, peak={peak:.5f} {unit}')
    lines.append('')
    for name in ('thrust', 'torque'):
        series = pipeline[name]['setpoint']
        lines.append(f'{name} samples: {len(series["times_s"])}')
    lines.append(f'Motor channels: {len(pipeline["motors"])}')
    return AnalysisResult('\n'.join(lines) + '\n', metrics, {'pipeline': pipeline})


def write_plots(result: AnalysisResult, output_dir: Path) -> list[Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    return write_control_pipeline_plots(result.plot_data['pipeline'], output_dir)


def comparison_signals(_experiment: dict) -> dict:
    """Read native SE3 trajectory and desired setpoints from PX4 outputs."""
    return {
        "trajectory": TRAJECTORY,
        "attitude": (TOPICS["OUT_VEHICLE_ATTITUDE_SETPOINT"], "q_d"),
        "rates": (RATE, "rpy"),
    }
