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
            DeclareLaunchArgument(
                "hostname",
                description="Hostname or IP address of the Vicon DataStream server.",
            ),
            DeclareLaunchArgument(
                "buffer_size",
                default_value="200",
                description="Vicon receiver frame buffer size.",
            ),
            DeclareLaunchArgument(
                "topic_namespace",
                default_value="vicon",
                description="ROS topic namespace used by the Vicon receiver.",
            ),
            DeclareLaunchArgument(
                "world_frame",
                default_value="map",
                description="World-frame name passed to the Vicon receiver.",
            ),
            DeclareLaunchArgument(
                "vicon_frame",
                default_value="vicon",
                description="Vicon-frame name passed to the Vicon receiver.",
            ),
            DeclareLaunchArgument(
                "map_xyz",
                default_value="[0.0,0.0,0.0]",
                description=(
                    "Translation parameter passed to the Vicon receiver for frame "
                    "mapping."
                ),
            ),
            DeclareLaunchArgument(
                "map_rpy",
                default_value="[0.0,0.0,0.0]",
                description=(
                    "Roll, pitch, yaw parameter passed to the Vicon receiver for frame "
                    "mapping."
                ),
            ),
            DeclareLaunchArgument(
                "map_rpy_in_degrees",
                default_value="false",
                description="Whether the frame-mapping RPY values are in degrees.",
            ),
            DeclareLaunchArgument(
                "mocap_topic",
                description="Input PoseStamped topic used by the PX4 motion-capture bridge.",
            ),
            DeclareLaunchArgument(
                "use_header_stamp",
                default_value="true",
                description="Use PoseStamped header timestamps for PX4 odometry conversion.",
            ),
            vicon_launch,
            bridge_node,
        ]
    )
