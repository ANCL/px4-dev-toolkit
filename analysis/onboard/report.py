"""Tracking statistics and plots for PX4-native SE(3) ULog experiments."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .ulog import Run, read_run


def _aligned(run: Run, measurement: dict, measured: np.ndarray,
             desired: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Interpolate references onto valid measurement times within their overlap."""
    t = np.asarray(measurement["time"], dtype=float)
    r = np.asarray(run.reference["time"], dtype=float)
    # Evaluate maneuver tracking through the first completion sample.
    # Do not include an arbitrary amount of post-completion hover in RMS.
    active = np.asarray(run.reference["phase"] == (1 if run.run_id == 0 else 2))
    completed = np.where(run.reference["phase"] == 3)[0]
    if len(completed):
        active[completed[0]] = True
    valid_ref = active & np.isfinite(r) & np.all(np.isfinite(desired), axis=1)
    r = r[valid_ref]
    desired = desired[valid_ref]
    if len(r) < 2:
        raise ValueError("too few finite reference samples")
    order = np.argsort(r, kind="stable")
    r, unique = np.unique(r[order], return_index=True)
    desired = desired[order][unique]
    mask = (t >= r[0]) & (t <= r[-1]) & np.isfinite(t) & np.all(np.isfinite(measured), axis=1)
    t, measured = t[mask], measured[mask]
    if len(t) < 2:
        raise ValueError("measured data do not overlap SE(3) trajectory samples")
    interp = np.column_stack([np.interp(t, r, desired[:, i]) for i in range(desired.shape[1])])
    return t, measured, interp


def _rms(t: np.ndarray, values: np.ndarray) -> float:
    """Time-weighted RMS on the measurement time axis."""
    if len(t) == 0:
        return float("nan")
    if len(t) == 1 or t[-1] <= t[0]:
        return float(np.sqrt(np.mean(values**2)))
    dt = np.diff(t)
    keep = np.isfinite(dt) & (dt > 0)
    if not np.any(keep):
        return float(np.sqrt(np.mean(values**2)))
    integral = np.sum(0.5 * (values[1:]**2 + values[:-1]**2)[keep] * dt[keep])
    return float(np.sqrt(integral / np.sum(dt[keep])))


def _time_fraction(times: np.ndarray, active: np.ndarray) -> float:
    """Fraction of recorded time satisfying a boolean condition."""
    dt = np.diff(times)
    mask = np.isfinite(dt) & (dt > 0)
    if not np.any(mask):
        return float(np.mean(active)) if len(active) else float("nan")
    intervals = (active[:-1].astype(float) + active[1:].astype(float)) * 0.5
    return float(np.sum(intervals[mask] * dt[mask]) / np.sum(dt[mask]))


def tracking(run: Run) -> dict:
    """Return aligned position, velocity and yaw errors, with coverage diagnostics."""
    measured = np.column_stack([run.position[k] for k in ("x", "y", "z")])
    measured_velocity = np.column_stack([run.position[k] for k in ("vx", "vy", "vz")])
    t, actual, target = _aligned(run, run.position, measured, run.reference["position"])
    tv, velocity, target_velocity = _aligned(
        run, run.position, measured_velocity, run.reference["velocity"])
    position_error = actual - target
    velocity_error = velocity - target_velocity
    error_norm = np.linalg.norm(position_error, axis=1)
    velocity_norm = np.linalg.norm(velocity_error, axis=1)
    tyaw, measured_yaw, target_yaw = _aligned(
        run, run.position, run.position["heading"][:, None], run.reference["yaw"][:, None])
    yaw_error_deg = np.degrees(np.arctan2(np.sin(measured_yaw - target_yaw),
                                         np.cos(measured_yaw - target_yaw))).ravel()
    gaps = np.diff(t)
    running = run.reference["phase"] == 2
    completed = run.reference["phase"] == 3
    first_done = np.flatnonzero(completed)
    active_end = float(run.reference["time"][first_done[0]]) if len(first_done) else float(run.end_s)
    result = {
        "time": t, "measured": actual, "target": target,
        "position_error": position_error, "error_norm": error_norm,
        "velocity_time": tv, "velocity_error": velocity_error,
        "velocity_measured": velocity, "velocity_target": target_velocity,
        "yaw_time": tyaw,
        "yaw_measured_deg": np.degrees(measured_yaw.ravel()),
        "yaw_target_deg": np.degrees(target_yaw.ravel()),
        "yaw_error_deg": yaw_error_deg,
        "position_rms_m": _rms(t, error_norm),
        "position_peak_m": float(np.max(error_norm)),
        "velocity_rms_mps": _rms(tv, velocity_norm),
        "yaw_rms_deg": _rms(t, yaw_error_deg),
        "max_sample_gap_s": float(np.max(gaps)) if len(gaps) else float("nan"),
        "reference_running_samples": int(np.count_nonzero(running)),
        "reference_hold_samples": int(np.count_nonzero(completed)),
        "measured_samples": int(len(t)),
        "execution_duration_s": active_end - run.start_s,
        "final_hold_duration_s": run.end_s - active_end,
    }
    if run.rates is not None and run.rate_sp is not None:
        actual_rates = np.asarray(run.rates["xyz"], dtype=float)
        commanded = np.column_stack([run.rate_sp[k] for k in ("roll", "pitch", "yaw")])
        mt = run.rates["time"]
        st = run.rate_sp["time"]
        mask = (mt >= st[0]) & (mt <= st[-1]) if len(st) > 1 else np.zeros(len(mt), dtype=bool)
        if np.count_nonzero(mask) > 1:
            matched = np.column_stack([np.interp(mt[mask], st, commanded[:, i]) for i in range(3)])
            result["body_rate_error_rms_radps"] = _rms(
                mt[mask], np.linalg.norm(actual_rates[mask] - matched, axis=1))
    if run.torque is not None and len(run.torque["time"]) > 1:
        magnitude = np.linalg.norm(run.torque["xyz"], axis=1)
        result["torque_rms_normalized"] = _rms(run.torque["time"], magnitude)
        result["torque_peak_normalized"] = float(np.max(magnitude))
    if run.motors is not None and len(run.motors["time"]) > 1:
        values = run.motors["control"]
        result["motor_peak_normalized"] = float(np.max(np.abs(values)))
        result["motor_near_limit_fraction"] = _time_fraction(
            run.motors["time"], np.max(np.abs(values), axis=1) >= 0.99)
    if run.allocator is not None and len(run.allocator["time"]) > 1:
        not_achieved = ((run.allocator["torque_setpoint_achieved"] < 0.5) |
                        (run.allocator["thrust_setpoint_achieved"] < 0.5))
        result["allocation_unachieved_fraction"] = _time_fraction(
            run.allocator["time"], not_achieved)
    return result



def _plot_run(run: Run, stats: dict, output: Path) -> list[Path]:
    """Generate trajectory-only tracking and per-axis error plots using the common style."""
    from analysis.plots import _matplotlib, save_tracking_plot
    import matplotlib.pyplot as plt

    plots: list[Path] = []
    # Replace only figures owned by this analyzer; preserve source ULog and
    # any unrelated user files inside the analysis directory.
    for filename in (
        "position.png", "position_error.png", "yaw_error.png", "body_rates.png",
        "motors.png", "position_tracking.png", "velocity_tracking.png",
        "velocity_error.png", "yaw_tracking.png",
        "attitude_tracking.png", "attitude_error.png", "body_rates_tracking.png",
        "body_rates_error.png",
    ):
        (output / filename).unlink(missing_ok=True)

    def write(name, title, unit, labels, timed_actual, actual, timed_ref, desired):
        components = [
            (axis, {
                "measured": (timed_actual.tolist(), actual[:, i].tolist()),
                "trajectory": (timed_ref.tolist(), desired[:, i].tolist()),
            }) for i, axis in enumerate(labels)
        ]
        path = output / (name + "_tracking.png")
        error = save_tracking_plot(
            path, components=components, title=title, unit=unit,
            time_origin_s=run.start_s, xlabel="Time after trajectory start [s]",
        )
        plots.append(path)
        if error is not None:
            plots.append(error)

    # These are precisely the samples used by the run-specific RMS/peak report.
    # Do not add hold after the first completion sample.
    write("position", f"Onboard SE3 run {run.run_id}: position", "[m]",
          ("North", "East", "Down"), stats["time"], stats["measured"],
          stats["time"], stats["target"])
    write("velocity", f"Onboard SE3 run {run.run_id}: velocity", "[m/s]",
          ("North", "East", "Down"), stats["velocity_time"], stats["velocity_measured"],
          stats["velocity_time"], stats["velocity_target"])
    write("yaw", f"Onboard SE3 run {run.run_id}: yaw", "[deg]",
          ("yaw",), stats["yaw_time"], stats["yaw_measured_deg"][:, None],
          stats["yaw_time"], stats["yaw_target_deg"][:, None])

    # Attitude is optional in old ULogs. The standard q_d publisher is
    # introduced by this patch and exists only in subsequent recordings.
    if run.attitude is not None and run.attitude_sp is not None:
        from analysis.px4 import quaternion_to_euler
        measured_q = np.asarray(run.attitude["q"], dtype=float)
        wanted_q = np.asarray(run.attitude_sp["q_d"], dtype=float)
        measured_good = np.isfinite(measured_q).all(axis=1) & (np.linalg.norm(measured_q, axis=1) > 0.5)
        wanted_good = np.isfinite(wanted_q).all(axis=1) & (np.linalg.norm(wanted_q, axis=1) > 0.5)
        if np.any(measured_good) and np.any(wanted_good):
            measured_rpy = np.degrees(np.asarray([
                quaternion_to_euler(q) for q in measured_q[measured_good]
            ], dtype=float))
            wanted_rpy = np.degrees(np.asarray([
                quaternion_to_euler(q) for q in wanted_q[wanted_good]
            ], dtype=float))
            write("attitude", "PX4 attitude tracking", "[deg]",
                  ("roll", "pitch", "yaw"),
                  np.asarray(run.attitude["time"])[measured_good], measured_rpy,
                  np.asarray(run.attitude_sp["time"])[wanted_good], wanted_rpy)

    if run.rates is not None and run.rate_sp is not None and len(run.rate_sp["time"]) >= 2:
        rt = np.asarray(run.rates["time"], dtype=float)
        sp_t = np.asarray(run.rate_sp["time"], dtype=float)
        valid = (rt >= sp_t[0]) & (rt <= sp_t[-1])
        if np.count_nonzero(valid) >= 2:
            sp = np.column_stack([run.rate_sp[k] for k in ("roll", "pitch", "yaw")])
            measured = np.asarray(run.rates["xyz"])[valid]
            write("body_rates", "PX4 body-rate tracking", "[deg/s]",
                  ("p / roll rate", "q / pitch rate", "r / yaw rate"),
                  rt[valid], np.degrees(measured), sp_t, np.degrees(sp))

    if run.motors is not None:
        _matplotlib()
        fig, axis = plt.subplots(figsize=(11.8, 4.6))
        for i in range(run.motors["control"].shape[1]):
            axis.plot(run.motors["time"] - run.start_s, run.motors["control"][:, i],
                      label=f"motor {i + 1}")
        axis.set(xlabel="Time after trajectory start [s]", ylabel="Normalized output",
                 title="PX4 actuator motor commands")
        axis.grid(True, alpha=0.3)
        axis.legend(ncol=4, frameon=False)
        fig.tight_layout()
        path = output / "motors.png"
        fig.savefig(path, bbox_inches="tight")
        plt.close(fig)
        plots.append(path)
    return plots

def summarize(run: Run, stats: dict) -> str:
    name = run.metadata.get("trajectory") or ("stationary hold" if run.run_id == 0 else "not specified")
    warnings = []
    if not run.completed and run.run_id > 0:
        warnings.append("Trajectory completion was not recorded")
    if stats["max_sample_gap_s"] > 0.1:
        warnings.append(f"Measured-position gap: {stats['max_sample_gap_s']:.3f} s")
    if run.dropouts:
        warnings.append(f"ULog dropout records: {run.dropouts}")
    optional = []
    for key, label, unit in (
        ("body_rate_error_rms_radps", "Body-rate RMS error", "rad/s"),
        ("torque_rms_normalized", "Torque-command RMS norm", "normalized"),
        ("torque_peak_normalized", "Torque-command peak norm", "normalized"),
        ("motor_peak_normalized", "Motor-output peak", "normalized"),
        ("motor_near_limit_fraction", "Motor near-limit fraction", ""),
        ("allocation_unachieved_fraction", "Allocation unachieved fraction", ""),
    ):
        optional.append(f"{label}: {stats[key]:.4f} {unit}\n" if key in stats else
                        f"{label}: not logged\n")
    interval = (f"Mode hold interval: {stats['execution_duration_s']:.2f} s\n"
                if run.run_id == 0 else
                f"Execution interval: {stats['execution_duration_s']:.2f} s\n"
                f"Post-completion final hold: {stats['final_hold_duration_s']:.2f} s\n")
    return (f"Onboard SE(3) flight\n===================\n"
            f"Log: {run.source}\nTrajectory: {name}\n"
            f"Run ID: {run.run_id}\nPlan CRC32: {run.checksum:08X}\n"
            + interval +
            f"Execution completed: {'n/a (mode hold)' if run.run_id == 0 else ('yes' if run.completed else 'no')}\n"
            f"Position RMS error: {stats['position_rms_m']:.4f} m\n"
            f"Position peak error: {stats['position_peak_m']:.4f} m\n"
            f"Velocity RMS error: {stats['velocity_rms_mps']:.4f} m/s\n"
            f"Yaw RMS error: {stats['yaw_rms_deg']:.3f} deg\n"
            f"Measured position samples: {stats['measured_samples']}\n"
            f"Reference samples: {len(run.reference['time'])}\n"
            f"Post-completion hold samples: {stats['reference_hold_samples']}\n"
            f"ULog dropout records: {run.dropouts}\n"
            + "".join(optional)
            + ("Warnings:\n" + "".join(f"  - {w}\n" for w in warnings) if warnings else ""))


def analyze(source: Path, run_id: int | None = None) -> str:
    run = read_run(source, run_id)
    stats = tracking(run)
    out = (source if source.is_dir() else source.parent / (source.stem + (f"_run{run_id}" if run_id is not None else ""))) / "analysis"
    out.mkdir(parents=True, exist_ok=True)
    summary = summarize(run, stats)
    (out / "summary.txt").write_text(summary)
    plots = _plot_run(run, stats, out)
    return summary + "".join(f"Plot: {p}\n" for p in plots)


def compare(sources: list[Path], output: Path) -> str:
    runs = [read_run(path) for path in sources]
    if any(r.run_id == 0 for r in runs):
        raise ValueError("trajectory comparison requires explicit run commands, not hold-only flights")
    if len({r.checksum for r in runs}) != 1:
        raise ValueError("selected trajectories have different CRC32 checksums")
    stats = [tracking(run) for run in runs]
    output.mkdir(parents=True, exist_ok=True)
    lines = ["Onboard SE(3) trajectory comparison", "=================================",
             f"Plan CRC32: {runs[0].checksum:08X}", ""]
    for run, s in zip(runs, stats):
        label = run.metadata.get("trajectory", "unknown")
        lines.append(f"{run.source.parent.name}/{run.source.name}  {label}  "
                     f"pos_RMS={s['position_rms_m']:.4f}m  "
                     f"pos_peak={s['position_peak_m']:.4f}m  "
                     f"vel_RMS={s['velocity_rms_mps']:.4f}m/s  "
                     f"rate_RMS={s.get('body_rate_error_rms_radps', float('nan')):.4f}rad/s  "
                     f"motor_peak={s.get('motor_peak_normalized', float('nan')):.3f}  "
                     f"completed={run.completed}")
    text = "\n".join(lines) + "\n"
    (output / "summary.txt").write_text(text)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(10, 5))
    for run, s in zip(runs, stats):
        ax.plot(s["time"] - run.start_s, s["error_norm"],
                label=run.source.parent.name)
    ax.set(xlabel="time after run start [s]", ylabel="position error norm [m]",
           title="Onboard SE(3): matched trajectory")
    ax.grid(True, alpha=.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output / "position_error_overlay.png", dpi=140)
    plt.close(fig)
    plotted = [output / "position_error_overlay.png"]

    for filename, field, times_field, label, title in (
        ("velocity_error_overlay.png", "velocity_error", "velocity_time", "velocity error norm [m/s]", "Velocity tracking"),
        ("yaw_error_overlay.png", "yaw_error_deg", "time", "yaw error [deg]", "Heading tracking"),
    ):
        fig, ax = plt.subplots(figsize=(10, 5))
        for run, s in zip(runs, stats):
            error = (np.linalg.norm(s[field], axis=1) if field == "velocity_error" else s[field])
            ax.plot(s[times_field] - run.start_s, error, label=run.source.parent.name)
        ax.set(xlabel="time after run start [s]", ylabel=label, title=title)
        ax.grid(True, alpha=.3)
        ax.legend()
        fig.tight_layout()
        path = output / filename
        fig.savefig(path, dpi=140)
        plt.close(fig)
        plotted.append(path)
    return text + "".join(f"Plot: {path}\n" for path in plotted)
