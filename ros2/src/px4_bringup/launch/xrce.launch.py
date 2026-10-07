"""Launch the Micro XRCE-DDS Agent for PX4 <-> ROS 2 communication."""

import shutil

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction


def launch_agent(context):
    """Construct one validated PX4 uXRCE-DDS Agent process.

    Inputs:
        udp4/udp6 port or serial device/baud launch arguments.

    Output:
        One foreground MicroXRCEAgent process owned by ros2 launch.
    """
    agent = shutil.which("MicroXRCEAgent")

    if agent is None:
        raise RuntimeError(
            "MicroXRCEAgent was not found. "
            "Source the ROS workspace before launching."
        )

    transport = LaunchConfiguration("transport").perform(context)
    port = LaunchConfiguration("port").perform(context)
    device = LaunchConfiguration("device").perform(context)
    baud = LaunchConfiguration("baud").perform(context)

    if transport in {"udp4", "udp6"}:
        command = [agent, transport, "-p", port]
        description = f"{transport} port {port}"

    elif transport == "serial":
        if not device:
            raise RuntimeError(
                "XRCE serial transport requires device:=..."
            )

        command = [
            agent,
            "serial",
            "--dev",
            device,
            "-b",
            baud,
        ]
        description = f"serial {device} @ {baud} baud"

    else:
        raise RuntimeError(
            "XRCE transport must be udp4, udp6, or serial."
        )

    return [
        LogInfo(msg=["XRCE Agent: ", description]),
        ExecuteProcess(
            cmd=command,
            output="screen",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "transport",
                default_value="udp4",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="8888",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="",
            ),
            DeclareLaunchArgument(
                "baud",
                default_value="921600",
            ),
            OpaqueFunction(
                function=launch_agent,
            ),
        ]
    )
