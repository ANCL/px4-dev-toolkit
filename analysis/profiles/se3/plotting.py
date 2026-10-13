"""Plot SE(3) tracking, controller output, and feasibility diagnostics."""

from __future__ import annotations

import shutil
from pathlib import Path
from analysis.plots import save_mode_timeline, save_tracking_plot
from .model import AnalysisResult


def _vector_components(
    series_by_label,
    component_names,
):
    components = []

    for component_name, key in component_names:
        signals = {}

        for label, series in series_by_label.items():
            if series["times_s"]:
                signals[label] = (
                    series["times_s"],
                    series[key],
                )

        if signals:
            components.append(
                (component_name, signals)
            )

    return components


def write_plots(
    result: AnalysisResult,
    output_dir: Path,
) -> list[Path]:
    """Write the single-run SE(3) figures.

    Inputs:
        Structured analysis result and output directory.

    Method:
        Shift presentation time to confirmed Offboard entry, plot only signals
        relevant to the selected handoff, and place controller-specific
        internals under the diagnostics directory.

    Returns:
        Paths of figures actually generated.
    """
    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    for old_plot in output_dir.glob("*.png"):
        old_plot.unlink()

    diagnostics_dir = (
        output_dir / "diagnostics"
    )

    if diagnostics_dir.exists():
        shutil.rmtree(
            diagnostics_dir
        )

    pipeline = result.plot_data["pipeline"]
    layers = result.plot_data["layers"]
    diagnostics = result.plot_data["diagnostics"]

    handoff_mode = str(
        result.metrics["handoff"]
    )

    direct_controller = str(
        result.metrics["direct_controller"]
    )

    origin_s = float(
        result.metrics["offboard_entry_s"]
    )

    time_label = (
        "Time from Offboard entry [s]"
    )

    generated = []

    def save_vector(
        directory: Path,
        filename: str,
        signals,
        components,
        title: str,
        unit: str,
    ) -> None:
        path = directory / filename

        save_tracking_plot(
            path,
            components=_vector_components(
                signals,
                components,
            ),
            title=title,
            unit=unit,
            time_origin_s=origin_s,
            xlabel=time_label,
        )

        if path.exists():
            generated.append(path)

    # --------------------------------------------------------
    # General analysis: only signals that are actually part of
    # the operational experiment/control path.
    # --------------------------------------------------------

    save_vector(
        output_dir,
        "01_position_tracking.png",
        {
            "measured":
                pipeline["position"]["actual"],
            "trajectory":
                layers["reference"]["position"],
        },
        [
            ("North", "x"),
            ("East", "y"),
            ("Down", "z"),
        ],
        "Position Tracking — NED",
        "[m]",
    )

    save_vector(
        output_dir,
        "02_velocity_tracking.png",
        {
            "measured":
                pipeline["velocity"]["actual"],
            "trajectory":
                layers["reference"]["velocity"],
        },
        [
            ("North", "x"),
            ("East", "y"),
            ("Down", "z"),
        ],
        "Velocity Tracking — NED",
        "[m/s]",
    )

    if handoff_mode == "acceleration":
        save_vector(
            output_dir,
            "03_acceleration_tracking.png",
            {
                "measured":
                    pipeline["acceleration"]["actual"],
                "command":
                    layers["handoff"]["acceleration"],
            },
            [
                ("North", "x"),
                ("East", "y"),
                ("Down", "z"),
            ],
            "Acceleration Handoff Tracking — NED",
            "[m/s²]",
        )

    if handoff_mode == "acceleration":
        attitude_reference = (
            pipeline["attitude"]["setpoint"]
        )
        attitude_label = "setpoint"
        attitude_title = (
            "Attitude Tracking — PX4 Generated Setpoint"
        )

    elif handoff_mode == "attitude":
        attitude_reference = (
            layers["handoff"]["attitude"]
        )
        attitude_label = "command"
        attitude_title = (
            "Attitude Tracking — SE3 Handoff"
        )

    else:
        attitude_reference = None

    if attitude_reference is not None:
        save_vector(
            output_dir,
            "04_attitude_tracking.png",
            {
                "measured":
                    pipeline["attitude"]["actual"],
                attitude_label:
                    attitude_reference,
            },
            [
                ("Roll", "x"),
                ("Pitch", "y"),
                ("Yaw", "z"),
            ],
            attitude_title,
            "[deg]",
        )

    if handoff_mode in {
        "acceleration",
        "attitude",
    }:
        rate_reference = (
            pipeline["rates"]["setpoint"]
        )
        rate_label = "setpoint"
        rate_title = (
            "Body-Rate Tracking — PX4 Rate Setpoint"
        )

    elif handoff_mode == "attitude_rate":
        rate_reference = (
            layers["handoff"]["rates"]
        )
        rate_label = "command"
        rate_title = (
            "Body-Rate Tracking — SE3 Handoff"
        )

    else:
        rate_reference = None

    if rate_reference is not None:
        save_vector(
            output_dir,
            "05_body_rate_tracking.png",
            {
                "measured":
                    pipeline["rates"]["actual"],
                rate_label:
                    rate_reference,
            },
            [
                ("p", "x"),
                ("q", "y"),
                ("r", "z"),
            ],
            rate_title + " — FRD",
            "[deg/s]",
        )

    save_vector(
        output_dir,
        "06_thrust_command.png",
        {
            "command":
                pipeline["thrust"]["setpoint"],
        },
        [
            ("x", "x"),
            ("y", "y"),
            ("z", "z"),
        ],
        "Normalized Thrust Command — FRD",
        "[normalized]",
    )

    save_vector(
        output_dir,
        "07_torque_command.png",
        {
            "command":
                pipeline["torque"]["setpoint"],
        },
        [
            ("Roll", "x"),
            ("Pitch", "y"),
            ("Yaw", "z"),
        ],
        "Normalized Torque Command — FRD",
        "[normalized]",
    )

    if pipeline["motors"]:
        motor_path = (
            output_dir
            / "08_motor_commands.png"
        )

        save_tracking_plot(
            motor_path,
            components=[
                (
                    "Motor command",
                    pipeline["motors"],
                ),
            ],
            title=(
                "PX4 Control Allocation — Motor Commands"
            ),
            unit="[normalized]",
            time_origin_s=origin_s,
            xlabel=time_label,
        )

        if motor_path.exists():
            generated.append(
                motor_path
            )

    mode_path = (
        output_dir
        / "09_mode_timeline.png"
    )

    save_mode_timeline(
        mode_path,
        transitions=result.plot_data[
            "transitions"
        ],
        state_names=result.plot_data[
            "state_names"
        ],
    )

    generated.append(
        mode_path
    )

    # --------------------------------------------------------
    # Optional diagnostics. No directory is created when the
    # experiment did not publish diagnostic instrumentation.
    # --------------------------------------------------------

    if not result.plot_data[
        "diagnostics_present"
    ]:
        return generated

    diagnostics_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    common = diagnostics["common"]

    commanded_acceleration = common.get(
        "commanded_acceleration"
    )

    if commanded_acceleration:
        save_vector(
            diagnostics_dir,
            "01_translational_controller.png",
            {
                "measured":
                    pipeline["acceleration"]["actual"],
                "command":
                    commanded_acceleration,
                "trajectory":
                    layers["reference"]["acceleration"],
            },
            [
                ("North", "x"),
                ("East", "y"),
                ("Down", "z"),
            ],
            "SE3 Translational Controller — Acceleration",
            "[m/s²]",
        )

    force_command = common.get(
        "force_command"
    )

    if force_command:
        save_vector(
            diagnostics_dir,
            "02_force_command.png",
            {
                "command": force_command,
            },
            [
                ("North", "x"),
                ("East", "y"),
                ("Down", "z"),
            ],
            "SE3 Force Command — NED",
            "[N]",
        )

    desired_attitude = common.get(
        "desired_attitude"
    )

    if (
        desired_attitude
        and handoff_mode != "attitude"
    ):
        save_vector(
            diagnostics_dir,
            "03_desired_attitude.png",
            {
                "measured":
                    pipeline["attitude"]["actual"],
                "desired":
                    desired_attitude,
            },
            [
                ("Roll", "x"),
                ("Pitch", "y"),
                ("Yaw", "z"),
            ],
            "SE3 Desired Attitude Diagnostic",
            "[deg]",
        )

    desired_angular_velocity = common.get(
        "desired_angular_velocity"
    )

    if desired_angular_velocity:
        save_vector(
            diagnostics_dir,
            "04_desired_angular_velocity.png",
            {
                "measured":
                    pipeline["rates"]["actual"],
                "desired":
                    desired_angular_velocity,
            },
            [
                ("p", "x"),
                ("q", "y"),
                ("r", "z"),
            ],
            (
                "SE3 Desired Angular Velocity "
                "vs Vehicle Body Rate — FRD"
            ),
            "[deg/s]",
        )

    body_rate_setpoint = common.get(
        "body_rate_setpoint"
    )

    if (
        body_rate_setpoint
        and handoff_mode != "attitude_rate"
    ):
        save_vector(
            diagnostics_dir,
            "05_body_rate_setpoint.png",
            {
                "measured":
                    pipeline["rates"]["actual"],
                "command":
                    body_rate_setpoint,
            },
            [
                ("p", "x"),
                ("q", "y"),
                ("r", "z"),
            ],
            "SE3 Body-Rate Setpoint Tracking — FRD",
            "[deg/s]",
        )

    angular_acceleration_feedforward = (
        common.get(
            "angular_acceleration_feedforward"
        )
    )

    measured_angular_acceleration = (
        common.get(
            "measured_angular_acceleration"
        )
    )

    if angular_acceleration_feedforward:
        angular_acceleration_signals = {
            "desired":
                angular_acceleration_feedforward,
        }

        if measured_angular_acceleration:
            angular_acceleration_signals[
                "measured"
            ] = measured_angular_acceleration

        save_vector(
            diagnostics_dir,
            "06_angular_acceleration.png",
            angular_acceleration_signals,
            [
                ("p dot", "x"),
                ("q dot", "y"),
                ("r dot", "z"),
            ],
            "Angular-Acceleration Reference — FRD",
            "[deg/s²]",
        )

    geometric = diagnostics[
        "geometric_normalized"
    ]

    if geometric:
        terms = {}

        for label, key in (
            (
                "attitude feedback",
                "attitude_feedback",
            ),
            (
                "rate feedback",
                "angular_velocity_feedback",
            ),
            (
                "acceleration feed-forward",
                "angular_acceleration_feedforward",
            ),
        ):
            if key in geometric:
                terms[label] = geometric[key]

        if "torque" in layers["handoff"]:
            terms["command"] = (
                layers["handoff"]["torque"]
            )

        save_vector(
            diagnostics_dir,
            "10_geometric_torque_terms.png",
            terms,
            [
                ("Roll", "x"),
                ("Pitch", "y"),
                ("Yaw", "z"),
            ],
            (
                "Geometric-Normalized "
                "Torque Decomposition"
            ),
            "[normalized]",
        )

    px4_rate = diagnostics["px4_rate"]

    if "rate_error" in px4_rate:
        save_vector(
            diagnostics_dir,
            "10_px4_rate_error.png",
            {
                "error":
                    px4_rate["rate_error"],
            },
            [
                ("p", "x"),
                ("q", "y"),
                ("r", "z"),
            ],
            "Reproduced PX4 Rate Error — FRD",
            "[deg/s]",
        )

    px4_terms = {}

    for label, key in (
        ("P", "proportional_feedback"),
        ("I", "integral_feedback"),
        ("D", "derivative_feedback"),
        ("FF", "feedforward"),
        ("unfiltered", "unfiltered_torque"),
    ):
        if key in px4_rate:
            px4_terms[label] = px4_rate[key]

    if px4_terms:
        save_vector(
            diagnostics_dir,
            "11_px4_rate_terms.png",
            px4_terms,
            [
                ("Roll", "x"),
                ("Pitch", "y"),
                ("Yaw", "z"),
            ],
            "Reproduced PX4 Rate-Controller Terms",
            "[normalized]",
        )

    if "integrator_state" in px4_rate:
        save_vector(
            diagnostics_dir,
            "12_px4_rate_integrator.png",
            {
                "integrator":
                    px4_rate[
                        "integrator_state"
                    ],
            },
            [
                ("Roll", "x"),
                ("Pitch", "y"),
                ("Yaw", "z"),
            ],
            "Reproduced PX4 Rate Integrator",
            "[normalized]",
        )

    physical = diagnostics["geometric_physical"]

    if (
        "requested_moment" in physical
        and "applied_moment" in physical
    ):
        save_vector(
            diagnostics_dir,
            "10_geometric_physical_moment_feasibility.png",
            {
                "requested":
                    physical["requested_moment"],
                "applied":
                    physical["applied_moment"],
            },
            [
                ("Roll", "x"),
                ("Pitch", "y"),
                ("Yaw", "z"),
            ],
            "Geometric Physical Moment — Requested vs Applied",
            "[N m]",
        )

    requested_collective = physical.get(
        "requested_collective_thrust"
    )
    applied_collective = physical.get(
        "applied_collective_thrust"
    )

    if (
        requested_collective
        and applied_collective
    ):
        collective_path = (
            diagnostics_dir
            / "11_geometric_physical_collective_feasibility.png"
        )

        save_tracking_plot(
            collective_path,
            components=[
                (
                    "Collective thrust",
                    {
                        "requested": (
                            requested_collective[
                                "times_s"
                            ],
                            requested_collective[
                                "values"
                            ],
                        ),
                        "applied": (
                            applied_collective[
                                "times_s"
                            ],
                            applied_collective[
                                "values"
                            ],
                        ),
                    },
                ),
            ],
            title=(
                "Geometric Physical Collective Thrust — "
                "Requested vs Applied"
            ),
            unit="[N]",
            time_origin_s=origin_s,
            xlabel=time_label,
        )

        generated.append(
            collective_path
        )

    moment_scale = physical.get(
        "moment_scale"
    )

    if moment_scale:
        scale_path = (
            diagnostics_dir
            / "12_geometric_physical_moment_scale.png"
        )

        save_tracking_plot(
            scale_path,
            components=[
                (
                    "Moment scale",
                    {
                        "scale": (
                            moment_scale["times_s"],
                            moment_scale["values"],
                        ),
                    },
                ),
            ],
            title="Geometric Physical Moment Feasibility Scale",
            unit="[-]",
            time_origin_s=origin_s,
            xlabel=time_label,
        )

        generated.append(
            scale_path
        )

    return generated
