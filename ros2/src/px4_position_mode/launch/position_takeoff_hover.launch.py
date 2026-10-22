"""Run native Position-mode takeoff, hover and landing, optionally recorded."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from px4_control_common.recording import launch_experiment, parse_recording_flag


def make_flight_node() -> Node:
    """Create the finite Position-mode flight node."""
    return Node(
        package="px4_position_mode",
        executable="position_takeoff_hover",
        name="position_takeoff_hover",
        output="screen",
        emulate_tty=True,
    )


def launch_setup(context):
    record = parse_recording_flag(LaunchConfiguration("record").perform(context))
    return launch_experiment(
        node=make_flight_node(),
        record=record,
        launch_file=__file__,
        package="px4_position_mode",
        topic_file="recording_topics.txt",
        bag_name="position_takeoff_hover",
        process_name="position_takeoff_hover",
        complete_message='Position-mode flight complete; stopping recording.',
        failure_message='Position-mode flight failed; stopping recording.',
        experiment={"profile": "position_takeoff_hover", "config": {}} if record else None,
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "record",
                default_value="false",
                description=(
                    "Record the complete Position-mode takeoff, hover, "
                    "landing, and disarm sequence."
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
