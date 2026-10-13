"""Render native PX4 control-pipeline plots from analyzed signals."""

from __future__ import annotations

from pathlib import Path
from analysis.plots import save_tracking_plot


def _components(
    section: dict[str, dict[str, list[float]]],
    names: list[tuple[str, str]],
):
    components = []

    for component_name, key in names:
        signals = {}

        for label, series in section.items():
            if series["times_s"]:
                signals[label] = (
                    series["times_s"],
                    series[key],
                )

        if signals:
            components.append((component_name, signals))

    return components


def write_control_pipeline_plots(
    pipeline: dict[str, object],
    output_dir: Path,
    *,
    markers: list[tuple[float, str]] | None = None,
) -> list[Path]:
    """Write the standard native-PX4 pipeline figures.

    Inputs:
        Extracted pipeline, output directory, and optional bag-relative event
        markers.

    Method:
        Plot each available controller layer with its measured/setpoint signals
        using the same timestamp-alignment rules as the numerical analysis.

    Returns:
        Paths of figures actually generated.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    generated = []

    specifications = [
        (
            "position",
            "01_position_tracking.png",
            "PX4 Position Tracking (NED)",
            "[m]",
            [
                ("x / North", "x"),
                ("y / East", "y"),
                ("z / Down", "z"),
            ],
        ),
        (
            "velocity",
            "02_velocity_tracking.png",
            "PX4 Velocity Tracking (NED)",
            "[m/s]",
            [
                ("vx / North", "x"),
                ("vy / East", "y"),
                ("vz / Down", "z"),
            ],
        ),
        (
            "acceleration",
            "03_acceleration_tracking.png",
            "PX4 Acceleration Tracking (NED)",
            "[m/s²]",
            [
                ("ax / North", "x"),
                ("ay / East", "y"),
                ("az / Down", "z"),
            ],
        ),
        (
            "attitude",
            "04_attitude_tracking.png",
            "PX4 Attitude Tracking",
            "[deg]",
            [
                ("roll", "x"),
                ("pitch", "y"),
                ("yaw", "z"),
            ],
        ),
        (
            "rates",
            "05_rate_tracking.png",
            "PX4 Body-Rate Tracking (FRD)",
            "[deg/s]",
            [
                ("p / roll rate", "x"),
                ("q / pitch rate", "y"),
                ("r / yaw rate", "z"),
            ],
        ),
        (
            "thrust",
            "06_thrust_setpoint.png",
            "PX4 Thrust Setpoint (FRD)",
            "[normalized]",
            [
                ("x / body", "x"),
                ("y / body", "y"),
                ("z / body", "z"),
            ],
        ),
        (
            "torque",
            "07_torque_setpoint.png",
            "PX4 Torque Setpoint (FRD)",
            "[normalized]",
            [
                ("x / roll", "x"),
                ("y / pitch", "y"),
                ("z / yaw", "z"),
            ],
        ),
    ]

    for key, filename, title, unit, names in specifications:
        components = _components(pipeline[key], names)

        if not components:
            continue

        path = output_dir / filename
        save_tracking_plot(
            path,
            components=components,
            title=title,
            unit=unit,
            markers=markers,
        )
        generated.append(path)

    motors = pipeline["motors"]

    if motors:
        path = output_dir / "08_actuator_motors.png"
        save_tracking_plot(
            path,
            components=[
                ("motor command", motors),
            ],
            title="PX4 Control Allocation: Actuator Motors",
            unit="[normalized]",
            markers=markers,
        )
        generated.append(path)

    return generated
