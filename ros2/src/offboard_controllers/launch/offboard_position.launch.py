"""Start independent PX4 Offboard position tracking, with optional rosbag capture."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from px4_control_common.recording import (
    controller_configuration_paths, launch_experiment, parse_recording_flag,
    trajectory_configuration, trajectory_configuration_path,
)


def make_control_node(controller_config: Path) -> Node:
    """Create the Offboard position controller node used by both launch modes."""

    return Node(
        package="offboard_controllers",
        executable="offboard_position",
        name="offboard_position",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "trajectory":
                    LaunchConfiguration("trajectory"),
                "offboard_trigger":
                    LaunchConfiguration("offboard_trigger"),
                "trajectory_config":
                    str(trajectory_configuration_path(controller_config, __file__)),
            },
        ],
    )


def launch_setup(context):
    record = parse_recording_flag(LaunchConfiguration("record").perform(context))
    controller_config, _, origin = controller_configuration_paths(__file__)
    experiment = None
    if record:
        trajectory = LaunchConfiguration("trajectory").perform(context)
        experiment = {
            "profile": "offboard_position",
            "trajectory": trajectory,
            "offboard_trigger": LaunchConfiguration("offboard_trigger").perform(context),
            "config": {
                "trajectory": trajectory_configuration(
                    trajectory_configuration_path(controller_config, __file__), trajectory
                ),
            },
        }
    return [
        LogInfo(msg=f"Offboard Position YAML: {origin}; "
                    f"trajectory={trajectory_configuration_path(controller_config, __file__)}"),
        *launch_experiment(
            node=make_control_node(controller_config),
            record=record,
            launch_file=__file__,
            package="offboard_controllers",
            topic_file="recording/offboard_position.txt",
            bag_name="offboard_position",
            process_name="offboard_position",
            recording_output=LaunchConfiguration("recording_output").perform(context).strip(),
            complete_message='Offboard position controller complete; stopping recording.',
            failure_message='Offboard position controller failed; stopping recording.',
            experiment=experiment,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "trajectory",
                default_value="hover",
                description=(
                    "Named trajectory from the shared trajectory configuration."
                ),
            ),
            DeclareLaunchArgument(
                "offboard_trigger",
                default_value="automatic",
                description="Offboard entry: automatic or external (pilot mode switch).",
            ),
            DeclareLaunchArgument(
                "record",
                default_value="false",
                description=(
                    "Record the Offboard position controller run."
                ),
            ),
            DeclareLaunchArgument(
                "recording_output",
                default_value="",
                description="Optional explicit rosbag output directory.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
