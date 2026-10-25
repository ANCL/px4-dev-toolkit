"""Recording and exit handling shared by finite PX4 ROS 2 experiments."""

from datetime import datetime
import json
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction,
    RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown, matches_action
from launch.events.process import ShutdownProcess

RECORDER_STARTUP_SECONDS = 1.0


def parse_recording_flag(value: str) -> bool:
    """Apply the launch files' existing case-insensitive recording convention."""
    return value.strip().lower() in {"1", "true", "yes", "on"}


def ros_parameters(path: Path, node: str) -> dict[str, object]:
    """Read the exact ROS parameter values supplied by a launch YAML file."""
    data = yaml.safe_load(path.read_text())
    return dict(data[node]["ros__parameters"])


def vehicle_configuration(path: Path) -> dict[str, object]:
    """Retain physical/allocator data, not the YAML source provenance."""
    data = yaml.safe_load(path.read_text())
    return {key: data[key] for key in ("plant", "propulsion", "px4_allocator") if key in data}


def trajectory_configuration(path: Path, name: str) -> dict[str, object]:
    """Resolve the selected absolute-NED path without copying unused maneuvers."""
    data = yaml.safe_load(path.read_text())
    available = data["segments"]
    named = data["trajectories"]
    start = list(data["start_position"])
    if name in named:
        selected = named[name]
        names = selected["segments"]
        start = list(selected.get("start_position", start))
    elif name in available:
        names = [name]
        primitive = available[name]
        if primitive["type"] == "figure_eight":
            start = list(primitive["center"])
        elif primitive["type"] in ("circle", "helix"):
            start = list(primitive["center"])
            start[0] += primitive["radius"]
    else:
        raise ValueError(f"Unknown trajectory or segment: {name}")
    return {
        "frame": data["frame"],
        "start_position": start,
        "default_yaw": data["default_yaw"],
        "segments": [{"name": segment, **available[segment]} for segment in names],
    }


def find_repo_root(launch_file: str) -> Path:
    start = Path(launch_file).resolve()
    for path in (start.parent, *start.parents):
        if ((path / "config" / "px4_topics.def").is_file()
                and (path / "config").is_dir() and (path / "ros2").is_dir()):
            return path
    raise RuntimeError(f"Could not locate toolkit repository root from {start}")



def trajectory_configuration_path(controller_config: Path, launch_file: str) -> Path:
    """Resolve the toolkit-owned trajectory YAML or packaged fallback."""
    try:
        root = find_repo_root(launch_file)
    except RuntimeError:
        return controller_config / "trajectory/trajectories.yaml"
    return root / "config/trajectories.yaml"


def controller_configuration_paths(launch_file: str) -> tuple[Path, Path, str]:
    """Load live checkout YAML; allow packaged fallback only without a checkout.

    Returns (controller_config_dir, vehicle_config_dir, origin_description).
    All controller configuration is read once when a new node is launched.
    """
    try:
        root = find_repo_root(launch_file)
    except RuntimeError:
        package_config = Path(get_package_share_directory("offboard_controllers")) / "config"
        controller_config = package_config
        vehicle_config = package_config / "vehicles"
        origin = "packaged (no toolkit checkout found)"
    else:
        controller_config = root / "ros2/src/offboard_controllers/config"
        vehicle_config = root / "config/vehicles"
        origin = "repository source"

    for file in (controller_config / "se3/controller.yaml",
                 trajectory_configuration_path(controller_config, launch_file)):
        if not file.is_file():
            raise RuntimeError(f"Controller configuration missing: {file}")
    if not vehicle_config.is_dir():
        raise RuntimeError(f"Vehicle configuration directory missing: {vehicle_config}")

    return controller_config, vehicle_config, origin


# Only profiles with allow_direct=True may record non-catalog ROS topics.
# Otherwise topic names must resolve through the shared PX4 catalog.
def read_topics(topic_file: Path, catalog_file: Path,
                allow_direct: bool = False) -> list[str]:
    catalog = {}
    for raw in catalog_file.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("//"):
            continue
        prefix = "PX4_TOPIC("
        if not line.startswith(prefix) or not line.endswith(")"):
            raise RuntimeError(f"Invalid PX4 topic definition: {raw}")
        name, value = line[len(prefix):-1].split(",", 1)
        catalog[name.strip()] = value.strip().strip('"')

    topics = []
    for raw in topic_file.read_text().splitlines():
        name = raw.strip()
        if not name or name.startswith("#"):
            continue
        if name.startswith("/") and allow_direct:
            topics.append(name)
        elif name in catalog:
            topics.append(catalog[name])
        else:
            raise RuntimeError(f"Unknown PX4 topic name in {topic_file}: {name}")
    if not topics:
        raise RuntimeError(f"Recording topic list is empty: {topic_file}")
    return topics


def _failure(message: str):
    def raise_failure(_context):
        raise RuntimeError(message)
    return OpaqueFunction(function=raise_failure)


def launch_experiment(*, node, record: bool, launch_file: str,
                      package: str, topic_file: str, bag_name: str,
                      process_name: str, complete_message: str,
                      failure_message: str, recording_output: str = "",
                      experiment: dict[str, object] | None = None,
                      allow_direct_topics: bool = False):
    """Run the node directly, or stop/finalize rosbag when it exits.

    Recorder SIGINT is expected after a completed controller run. Early
    recorder exit or a nonzero controller exit always fails the launch.
    """
    if not record:
        def on_node_exit(event, _context):
            if event.returncode != 0:
                return [_failure(f"{process_name} exited with status {event.returncode}.")]
            return [EmitEvent(event=Shutdown(reason=f"{process_name} completed."))]
        return [node, RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=on_node_exit))]

    if experiment is None or not experiment.get("profile"):
        raise ValueError("Recorded experiments require profile and configuration metadata")

    root = find_repo_root(launch_file)
    share = Path(get_package_share_directory(package))
    topic_root = root / "config" / "recording"
    profile_file = topic_root / (str(experiment["profile"]) + ".txt")
    common_file = topic_root / "common.txt"
    catalog_file = root / "config" / "px4_topics.def"
    if common_file.is_file() and profile_file.is_file():
        topics = read_topics(common_file, catalog_file)
        # Allow PX4 catalog tokens and explicit ROS controller diagnostics.
        # The shared parser validates every entry; it does not silently skip
        # an unknown symbolic topic.
        extra_entries = [line.strip() for line in profile_file.read_text().splitlines()
                         if line.strip() and not line.lstrip().startswith("#")]
        if extra_entries:
            topics.extend(read_topics(profile_file, catalog_file, allow_direct=True))
        topics = list(dict.fromkeys(topics))
    else:
        # Preserve older experiments without a shared topic profile.
        topics = read_topics(share / "config" / topic_file,
                            catalog_file, allow_direct_topics)
    if recording_output:
        bag_path = Path(recording_output).expanduser().resolve()
        if bag_path.exists():
            raise RuntimeError(f"Recording output already exists: {bag_path}")
        bag_path.parent.mkdir(parents=True, exist_ok=True)
    else:
        bag_root = root / "bags" / bag_name
        bag_path = bag_root / datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_root.mkdir(parents=True, exist_ok=True)

    recorder = ExecuteProcess(
        cmd=["ros2", "bag", "record", "--output", str(bag_path), "--topics", *topics],
        name=f"{process_name}_recorder", output="screen", emulate_tty=True,
    )
    node_exit_code = {"value": None}

    def on_node_exit(event, _context):
        node_exit_code["value"] = event.returncode
        message = complete_message if event.returncode == 0 else failure_message
        return [LogInfo(msg=message), EmitEvent(event=ShutdownProcess(
            process_matcher=matches_action(recorder)))]

    def on_recorder_exit(_event, _context):
        if (bag_path / "metadata.yaml").is_file():
            (bag_path / "experiment.json").write_text(
                json.dumps(experiment, indent=2) + "\n"
            )
        actions = [LogInfo(msg=["Bag output: ", str(bag_path)])]
        if node_exit_code["value"] is None:
            return actions + [_failure(f"{process_name} recorder exited before the node.")]
        if node_exit_code["value"] != 0:
            return actions + [_failure(
                f"{process_name} exited with status {node_exit_code['value']}.")]
        return actions + [EmitEvent(event=Shutdown(reason="ROS bag recorder stopped."))]

    return [
        LogInfo(msg=["Recording bag: ", str(bag_path)]),
        recorder,
        RegisterEventHandler(OnProcessStart(
            target_action=recorder,
            on_start=[TimerAction(period=RECORDER_STARTUP_SECONDS, actions=[node])])),
        RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=on_node_exit)),
        RegisterEventHandler(OnProcessExit(
            target_action=recorder, on_exit=on_recorder_exit)),
    ]
