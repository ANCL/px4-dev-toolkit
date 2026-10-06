#!/usr/bin/env bash

# Shared interactive runtime environment for ROS 2 + PX4.
# Source this file before running ROS 2 commands from a normal terminal.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "ERROR: source ros2/runtime_env.sh instead of executing it."
    exit 1
fi

_PX4_ENV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source "${_PX4_ENV_ROOT}/config/runtime/common.env"

_PX4_ENV_ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
_PX4_ENV_WS_SETUP="${_PX4_ENV_ROOT}/ros2/install/setup.bash"

# Fail early with a useful message instead of letting a later ros2 command
# fail because the base ROS install or this workspace has not been prepared.
if [[ ! -f "${_PX4_ENV_ROS_SETUP}" ]]; then
    echo "ERROR: ROS 2 ${ROS_DISTRO} is not installed." >&2
    return 1
fi

if [[ ! -f "${_PX4_ENV_WS_SETUP}" ]]; then
    echo "ERROR: ROS 2 workspace has not been built." >&2
    return 1
fi

# ROS setup scripts may inspect unset variables. Preserve the caller's
# nounset state while sourcing them.
_PX4_ENV_NOUNSET_WAS_SET=0

case "$-" in
    *u*)
        _PX4_ENV_NOUNSET_WAS_SET=1
        set +u
        ;;
esac

if ! source "${_PX4_ENV_ROS_SETUP}"; then
    if [[ "${_PX4_ENV_NOUNSET_WAS_SET}" -eq 1 ]]; then
        set -u
    fi
    return 1
fi

if ! source "${_PX4_ENV_WS_SETUP}"; then
    if [[ "${_PX4_ENV_NOUNSET_WAS_SET}" -eq 1 ]]; then
        set -u
    fi
    return 1
fi

if [[ "${_PX4_ENV_NOUNSET_WAS_SET}" -eq 1 ]]; then
    set -u
fi


# Do not leave helper variables in the caller's interactive shell.
unset _PX4_ENV_ROOT
unset _PX4_ENV_ROS_SETUP
unset _PX4_ENV_WS_SETUP
unset _PX4_ENV_NOUNSET_WAS_SET
