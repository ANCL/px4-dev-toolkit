"""Launch Position-mode staging through confirmed Offboard takeover.

Without recording:
    launch the staging helper and propagate its exit status.

With recording:
    start rosbag first, allow DDS discovery, then start the helper. The helper
    owns experiment completion; its exit stops rosbag, and launch terminates
    only after recording has finalized.
"""

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown, matches_action
from launch.events.process import ShutdownProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ros2 bag is started before the node. Give DDS discovery a short window before
# control starts so the beginning of the handoff is not missed.
RECORDER_STARTUP_SECONDS = 1.0


def find_repo_root() -> Path:
    """
    Locate the toolkit root from either the source or installed launch path.

    colcon installs this launch file below ros2/install/, which is still inside
    the repository. Repository markers keep bag output anchored to the
    repository's bags/ directory instead of depending on the shell's current
    working directory.
    """
    start = Path(__file__).resolve()

    for path in (start.parent, *start.parents):
        if (
            (path / "config" / "px4_topics.def").is_file()
            and (path / "config").is_dir()
            and (path / "ros2").is_dir()
        ):
            return path

    raise RuntimeError(
        f"Could not locate toolkit repository root from {start}"
    )


def read_topic_catalog(path: Path) -> dict[str, str]:
    """Load the project-wide PX4 topic-name catalog."""
    topics = {}

    for raw_line in path.read_text().splitlines():
        line = raw_line.strip()

        if not line or line.startswith("//"):
            continue

        prefix = "PX4_TOPIC("
        if not line.startswith(prefix) or not line.endswith(")"):
            raise RuntimeError(
                f"Invalid PX4 topic definition: {raw_line}"
            )

        name, value = line[len(prefix):-1].split(",", 1)
        topics[name.strip()] = value.strip().strip('"')

    return topics


def read_recording_topics(
    path: Path,
    topic_catalog: dict[str, str],
) -> list[str]:
    """Resolve this package's recording selection through the topic catalog."""
    topics = []

    for raw_line in path.read_text().splitlines():
        name = raw_line.strip()

        if not name or name.startswith("#"):
            continue

        if name not in topic_catalog:
            raise RuntimeError(
                f"Unknown PX4 topic name in {path}: {name}"
            )

        topics.append(topic_catalog[name])

    if not topics:
        raise RuntimeError(f"Recording topic list is empty: {path}")

    return topics


def launch_argument_is_true(context, name: str) -> bool:
    """Interpret a ROS launch boolean argument using the usual truthy forms."""
    value = LaunchConfiguration(name).perform(context).strip().lower()
    return value in {"1", "true", "yes", "on"}


def make_control_node() -> Node:
    """Create the Offboard takeoff handoff node used by both launch modes."""
    package_share = Path(
        get_package_share_directory("offboard_controllers")
    )
    config_file = package_share / "config" / "offboard.yaml"

    if not config_file.is_file():
        raise RuntimeError(
            f"Missing Offboard configuration: {config_file}"
        )

    return Node(
        package="offboard_controllers",
        executable="offboard_takeoff_handoff",
        name="offboard_takeoff_handoff",
        output="screen",
        emulate_tty=True,
        parameters=[str(config_file)],
    )


def launch_setup(context):
    """
    Build either the simple control launch or the recording orchestration.

    Without recording, the launch contains only the control node. With
    recording, rosbag starts first and stops when the control node exits.
    """
    record = launch_argument_is_true(context, "record")
    control_node = make_control_node()

    if not record:
        def on_control_exit(event, _context):
            if event.returncode != 0:
                returncode = event.returncode

                def fail_launch(_context):
                    raise RuntimeError(
                        "offboard_takeoff_handoff exited with status "
                        f"{returncode}."
                    )

                return [
                    OpaqueFunction(
                        function=fail_launch
                    )
                ]

            return [
                EmitEvent(
                    event=Shutdown(
                        reason="offboard_takeoff_handoff completed."
                    )
                )
            ]

        return [
            control_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=control_node,
                    on_exit=on_control_exit,
                )
            ),
        ]

    repo_root = find_repo_root()
    package_share = Path(
        get_package_share_directory("offboard_controllers")
    )
    topic_file = package_share / "config" / "recording/offboard_takeoff_handoff.txt"
    topic_catalog_file = repo_root / "config" / "px4_topics.def"

    topic_catalog = read_topic_catalog(topic_catalog_file)
    topics = read_recording_topics(topic_file, topic_catalog)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_root = repo_root / "bags" / "offboard_takeoff_handoff"
    bag_path = bag_root / timestamp

    bag_root.mkdir(parents=True, exist_ok=True)

    control_exit_code = {
        "value": None,
    }

    bag_process = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--output",
            str(bag_path),
            "--topics",
            *topics,
        ],
        name="offboard_takeoff_handoff_recorder",
        output="screen",
        emulate_tty=True,
    )

    def on_control_exit(event, _context):
        control_exit_code["value"] = event.returncode

        if event.returncode == 0:
            message = (
                "Offboard takeoff handoff complete; stopping recording."
            )

        else:
            message = (
                "Offboard takeoff handoff failed; stopping recording."
            )

        return [
            LogInfo(msg=message),
            EmitEvent(
                event=ShutdownProcess(
                    process_matcher=matches_action(bag_process)
                )
            ),
        ]

    def on_bag_exit(_event, _context):
        # The recorder is deliberately stopped with SIGINT after the flight
        # process exits. A signal-based recorder return code is therefore not
        # itself an experiment failure. Failure is determined by premature
        # recorder exit or by the flight/controller process return code.
        actions = [
            LogInfo(msg=["Bag output: ", str(bag_path)]),
        ]

        controller_returncode = control_exit_code["value"]

        if controller_returncode is None:
            failure = (
                "Offboard takeoff handoff recorder exited "
                "before the controller."
            )
        elif controller_returncode != 0:
            failure = (
                "offboard_takeoff_handoff exited with status "
                f"{controller_returncode}."
            )
        else:
            failure = None

        if failure is not None:
            def fail_launch(_context):
                raise RuntimeError(failure)

            actions.append(
                OpaqueFunction(
                    function=fail_launch
                )
            )

            return actions

        actions.append(
            EmitEvent(
                event=Shutdown(
                    reason="ROS bag recorder stopped."
                )
            )
        )

        return actions

    return [
        LogInfo(msg=["Recording bag: ", str(bag_path)]),

        # Start rosbag first. The short timer begins only after the recorder
        # process has actually started, keeping early control traffic in the bag.
        bag_process,
        RegisterEventHandler(
            OnProcessStart(
                target_action=bag_process,
                on_start=[
                    TimerAction(
                        period=RECORDER_STARTUP_SECONDS,
                        actions=[control_node],
                    )
                ],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=control_node,
                on_exit=on_control_exit,
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=bag_process,
                on_exit=on_bag_exit,
            )
        ),
    ]


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
