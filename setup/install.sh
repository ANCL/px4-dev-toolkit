#!/usr/bin/env bash
# Install one native toolkit scope: common ROS utilities, SITL/Gazebo,
# hardware companion services, or both. Source pins live in config/sources/;
# this script delegates each installation stage and then builds the ROS workspace.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
profile="${1:-}"

case "${profile}" in
    common|sitl|hardware|all)
        ;;
    *)
        echo "Usage: ./setup/install.sh common|sitl|hardware|all" >&2
        exit 2
        ;;
esac

# A companion computer does not require PX4 firmware sources, simulators,
# cross-compilers, or graphical ROS desktop applications.
if [[ "${profile}" == "hardware" || "${profile}" == "common" ]]; then
    ros_profile=base
    tools_profile="${profile}"
else
    ros_profile=desktop
    tools_profile=sitl
fi

"${ROOT}/setup/install_ros2_jazzy.sh" "${ros_profile}"
"${ROOT}/setup/install_common_tools.sh" "${tools_profile}"
"${ROOT}/setup/fetch_sources.sh" "${profile}"

case "${profile}" in
    sitl|all)
        "${ROOT}/setup/install_px4_dependencies.sh" sitl
        "${ROOT}/setup/install_mavproxy.sh"
        ;;
esac

if [[ "${profile}" == "all" ]]; then
    "${ROOT}/setup/install_qgroundcontrol.sh"
fi

"${ROOT}/ros2/build.sh"
mkdir -p "${ROOT}/bags" "${ROOT}/log"

echo "PX4 ${profile} toolkit environment ready."
