"""Stage native Position-mode takeoff before independent Offboard ownership."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from px4_control_common.recording import (
    launch_experiment, parse_recording_flag, ros_parameters,
)


def make_control_node() -> Node:
    """Create the Offboard takeoff handoff node."""
    package_share = Path(
        get_package_share_directory("px4_position_mode")
    )
    config_file = package_share / "config" / "offboard.yaml"

    if not config_file.is_file():
        raise RuntimeError(
            f"Missing Offboard configuration: {config_file}"
        )

    return Node(
        package="px4_position_mode",
        executable="offboard_takeoff_handoff",
        name="offboard_takeoff_handoff",
        output="screen",
        emulate_tty=True,
        parameters=[str(config_file)],
    )


def launch_setup(context):
    record = parse_recording_flag(LaunchConfiguration("record").perform(context))
    return launch_experiment(
        node=make_control_node(),
        record=record,
        launch_file=__file__,
        package="px4_position_mode",
        topic_file="recording/offboard_takeoff_handoff.txt",
        bag_name="offboard_takeoff_handoff",
        process_name="offboard_takeoff_handoff",
        complete_message='Offboard takeoff handoff complete; stopping recording.',
        failure_message='Offboard takeoff handoff failed; stopping recording.',
        experiment={
            "profile": "offboard_takeoff_handoff",
            "config": {
                "staging": ros_parameters(
                    Path(get_package_share_directory("px4_position_mode")) / "config/offboard.yaml",
                    "/**",
                ),
            },
        } if record else None,
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "record",
                default_value="false",
                description=(
                    "Record the Offboard takeoff handoff run."
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
