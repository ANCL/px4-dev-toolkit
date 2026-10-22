"""Start a configured SE(3) controller handoff with optional rosbag capture.

recording_output places study-managed bags; standalone recording uses bags/se3.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from px4_control_common.recording import (
    controller_configuration_paths, launch_experiment, parse_recording_flag,
    ros_parameters, trajectory_configuration, trajectory_configuration_path, vehicle_configuration,
)


def make_control_node(controller_config: Path, vehicle_config: Path) -> Node:
    """Create the configured SE(3) node used by recorded and unrecorded runs."""

    return Node(
        package="offboard_controllers",
        executable="se3",
        name="se3",
        output="screen",
        emulate_tty=True,
        parameters=[
            str(controller_config / "se3" / "controller.yaml"),
            {
                "vehicle":
                    LaunchConfiguration("vehicle"),
                "trajectory":
                    LaunchConfiguration("trajectory"),
                "handoff":
                    LaunchConfiguration("handoff"),
                "offboard_trigger":
                    LaunchConfiguration("offboard_trigger"),
                "direct_controller":
                    LaunchConfiguration(
                        "direct_controller"
                    ),
                "publish_diagnostics":
                    LaunchConfiguration(
                        "publish_diagnostics"
                    ),
                "vehicle_config_dir":
                    str(vehicle_config),
                "trajectory_config":
                    str(trajectory_configuration_path(controller_config, __file__)),
            },
        ],
    )


def launch_setup(context):
    record = parse_recording_flag(LaunchConfiguration("record").perform(context))
    recording_output = LaunchConfiguration("recording_output").perform(context).strip()
    controller_config, vehicle_config, origin = controller_configuration_paths(__file__)
    experiment = None
    if record:
        args = {name: LaunchConfiguration(name).perform(context) for name in (
            "vehicle", "trajectory", "handoff", "offboard_trigger", "direct_controller",
            "publish_diagnostics",
        )}
        controller_file = controller_config / "se3/controller.yaml"
        trajectory_file = trajectory_configuration_path(controller_config, __file__)
        experiment = {
            "profile": "se3",
            "vehicle": args["vehicle"],
            "trajectory": args["trajectory"],
            "handoff": args["handoff"],
            "offboard_trigger": args["offboard_trigger"],
            "publish_diagnostics": parse_recording_flag(args["publish_diagnostics"]),
            "config": {
                "controller": ros_parameters(controller_file, "se3"),
                "vehicle": vehicle_configuration(vehicle_config / (args["vehicle"] + ".yaml")),
                "trajectory": trajectory_configuration(trajectory_file, args["trajectory"]),
            },
        }
        if args["handoff"] == "thrust_and_torque":
            experiment["direct_controller"] = args["direct_controller"]
    return [
        LogInfo(msg=f"SE3 YAML: {origin}; controller={controller_config / 'se3/controller.yaml'}; "
                    f"trajectory={trajectory_configuration_path(controller_config, __file__)}; "
                    f"vehicles={vehicle_config}"),
        *launch_experiment(
            node=make_control_node(controller_config, vehicle_config),
            record=record,
            launch_file=__file__,
            package="offboard_controllers",
            topic_file="recording/se3.txt",
            bag_name="se3",
            process_name="se3",
            complete_message='SE3 controller complete; stopping recording.',
            failure_message='SE3 controller failed; stopping recording.',
            recording_output=recording_output,
            allow_direct_topics=True,
            experiment=experiment,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle",
                default_value="gz_f450",
                description="Vehicle configuration selected by the SE(3) controller.",
            ),
            DeclareLaunchArgument(
                "trajectory",
                default_value="hover",
                description="Named trajectory from the shared trajectory configuration.",
            ),
            DeclareLaunchArgument(
                "handoff",
                default_value="acceleration",
                description=(
                    "PX4 Offboard handoff: acceleration, attitude, attitude_rate, or "
                    "thrust_and_torque."
                ),
            ),
            DeclareLaunchArgument(
                "offboard_trigger",
                default_value="automatic",
                description="Offboard entry: automatic or external (pilot mode switch).",
            ),
            DeclareLaunchArgument(
                "direct_controller",
                default_value="geometric_normalized",
                description=(
                    "Rotational controller used when "
                    "handoff:=thrust_and_torque."
                ),
            ),
            DeclareLaunchArgument(
                "publish_diagnostics",
                default_value="false",
                description=(
                    "Publish optional SE3 internal "
                    "controller diagnostics."
                ),
            ),
            DeclareLaunchArgument(
                "record",
                default_value="false",
                description=(
                    "Record the SE3 experiment."
                ),
            ),
            DeclareLaunchArgument(
                "recording_output",
                default_value="",
                description=(
                    "Optional explicit rosbag output directory. "
                    "An empty value preserves the standalone "
                    "bags/se3/<timestamp> layout."
                ),
            ),
            OpaqueFunction(
                function=launch_setup
            ),
        ]
    )
