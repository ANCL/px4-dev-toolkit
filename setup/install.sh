#!/usr/bin/env bash
# Install either a SITL workstation or a physical-vehicle companion computer.
# These are separate environments; both resolve their own pinned sources.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
profile="${1:-}"

if (( $# != 1 )); then
    echo "Usage: ./setup/install.sh sitl|hardware" >&2
    exit 2
fi

case "${profile}" in
    sitl)
        # SITL needs PX4/Gazebo, the graphical ROS distribution, and MAVProxy.
        "${ROOT}/setup/install_ros2_jazzy.sh" desktop
        "${ROOT}/setup/install_tools.sh" sitl
        "${ROOT}/setup/fetch_sources.sh" sitl
        "${ROOT}/setup/install_px4_dependencies.sh" sitl
        "${ROOT}/setup/install_mavproxy.sh"
        "${ROOT}/setup/install_mavlink.sh"
        ;;
    hardware)
        # The companion uses ROS-base, Vicon, and a serial DDS agent.
        # It does not install PX4 firmware sources or Gazebo dependencies.
        "${ROOT}/setup/install_ros2_jazzy.sh" base
        "${ROOT}/setup/install_tools.sh" hardware
        "${ROOT}/setup/fetch_sources.sh" hardware
        "${ROOT}/setup/install_mavlink.sh"
        ;;
    *)
        echo "Usage: ./setup/install.sh sitl|hardware" >&2
        exit 2
        ;;
esac

"${ROOT}/ros2/build.sh" "${profile}"
mkdir -p "${ROOT}/bags" "${ROOT}/log"
echo "PX4 ${profile} toolkit environment ready."
