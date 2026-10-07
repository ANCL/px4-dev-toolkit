#!/usr/bin/env bash
set -euo pipefail

ROOT=/opt/px4-dev-toolkit

# Container commands receive the same ROS 2 + toolkit environment used by
# native interactive shells.
source "${ROOT}/ros2/runtime_env.sh"

if (( $# == 0 )); then
    set -- bash
fi

exec "$@"
