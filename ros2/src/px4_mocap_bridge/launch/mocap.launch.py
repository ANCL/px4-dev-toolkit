"""Launch Vicon reception and PX4 motion-capture odometry bridging."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    hostname = LaunchConfiguration("hostname")
    buffer_size = LaunchConfiguration("buffer_size")
    topic_namespace = LaunchConfiguration("topic_namespace")
    world_frame = LaunchConfiguration("world_frame")
    vicon_frame = LaunchConfiguration("vicon_frame")
    map_xyz = LaunchConfiguration("map_xyz")
    map_rpy = LaunchConfiguration("map_rpy")
    map_rpy_in_degrees = LaunchConfiguration("map_rpy_in_degrees")
    mocap_topic = LaunchConfiguration("mocap_topic")
    use_header_stamp = LaunchConfiguration("use_header_stamp")

    vicon_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("vicon_receiver"),
                    "launch",
                    "client.launch.py",
                ]
            )
        ),
        launch_arguments={
            "hostname": hostname,
            "buffer_size": buffer_size,
            "topic_namespace": topic_namespace,
            "world_frame": world_frame,
            "vicon_frame": vicon_frame,
            "map_xyz": map_xyz,
            "map_rpy": map_rpy,
            "map_rpy_in_degrees": map_rpy_in_degrees,
        }.items(),
    )

    bridge_node = Node(
        package="px4_mocap_bridge",
        executable="px4_mocap_bridge",
        name="px4_mocap_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "mocap_topic": mocap_topic,
                "use_header_stamp": use_header_stamp,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hostname"),
            DeclareLaunchArgument("buffer_size", default_value="200"),
            DeclareLaunchArgument(
                "topic_namespace",
                default_value="vicon",
            ),
            DeclareLaunchArgument(
                "world_frame",
                default_value="map",
            ),
            DeclareLaunchArgument(
                "vicon_frame",
                default_value="vicon",
            ),
            DeclareLaunchArgument(
                "map_xyz",
                default_value="[0.0,0.0,0.0]",
            ),
            DeclareLaunchArgument(
                "map_rpy",
                default_value="[0.0,0.0,0.0]",
            ),
            DeclareLaunchArgument(
                "map_rpy_in_degrees",
                default_value="false",
            ),
            DeclareLaunchArgument("mocap_topic"),
            DeclareLaunchArgument(
                "use_header_stamp",
                default_value="true",
            ),
            vicon_launch,
            bridge_node,
        ]
    )
