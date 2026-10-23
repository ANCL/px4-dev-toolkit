"""Take off in Position mode, then hold (default) or land."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from px4_control_common.recording import launch_experiment, parse_recording_flag


def make_flight_node(finish: str) -> Node:
    """Create the Position-mode takeoff node with its finish action."""
    return Node(
        package="px4_position_mode",
        executable="position_takeoff_hover",
        name="position_takeoff_hover",
        output="screen",
        emulate_tty=True,
        parameters=[{"finish": finish}],
    )


def launch_setup(context):
    record = parse_recording_flag(LaunchConfiguration("record").perform(context))
    finish = LaunchConfiguration("finish").perform(context)
    if finish not in ("hold", "land"):
        raise ValueError("finish must be 'hold' or 'land'")

    return launch_experiment(
        node=make_flight_node(finish),
        record=record,
        launch_file=__file__,
        package="px4_position_mode",
        topic_file="recording_topics.txt",
        bag_name="position_takeoff_hover",
        process_name="position_takeoff_hover",
        complete_message='Position-mode flight complete; stopping recording.',
        failure_message='Position-mode flight failed; stopping recording.',
        experiment={"profile": "position_takeoff_hover", "config": {"finish": finish}} if record else None,
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "finish",
                default_value="hold",
                description="After the 20-second hover: hold (stay armed in Position mode) or land.",
            ),
            DeclareLaunchArgument(
                "record",
                default_value="false",
                description=(
                    "Record the Position-mode takeoff and selected finish action."
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
