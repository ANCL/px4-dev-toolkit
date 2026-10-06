#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
    echo "Usage: ./setup/install.sh common|sitl|experiment|all"
}

profile="${1:-}"

case "${profile}" in
    common|sitl|experiment|all)
        ;;
    *)
        usage
        exit 2
        ;;
esac

echo "========================================"
echo " Installing PX4 environment: ${profile}"
echo "========================================"

# Shared host environment and pinned sources.
"${ROOT}/setup/install_ros2_jazzy.sh"
"${ROOT}/setup/install_common_tools.sh"
"${ROOT}/setup/fetch_sources.sh" "${profile}"

case "${profile}" in
    common)
        ;;

    sitl)
        "${ROOT}/setup/install_px4_dependencies.sh" sitl
        "${ROOT}/setup/install_mavproxy.sh"
        ;;

    experiment)
        "${ROOT}/setup/install_px4_dependencies.sh" experiment
        ;;

    all)
        "${ROOT}/setup/install_px4_dependencies.sh" all
        "${ROOT}/setup/install_mavproxy.sh"
        "${ROOT}/setup/install_qgroundcontrol.sh"
        ;;
esac

"${ROOT}/ros2/build.sh"

echo
echo "========================================"
echo " PX4 environment ready: ${profile}"
echo "========================================"
