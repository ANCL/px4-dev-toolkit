#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Build packages appropriate to the installed environment. Common ROS 2
# libraries are shared; SITL does not need the Vicon bridge and the companion
# does not need the simulated-pilot Position-mode examples.
if (( $# != 1 )); then
    echo "Usage: ./ros2/build.sh sitl|hardware" >&2
    exit 2
fi
case "$1" in
    sitl|hardware) profile="$1" ;;
    *) echo "Usage: ./ros2/build.sh sitl|hardware" >&2; exit 2 ;;
esac

source "${ROOT}/config/runtime/common.env"

# Build in a deliberately small environment so an already-sourced ROS/PX4
# shell cannot leak paths or Python settings into colcon. Preserve only the
# host identity, a deterministic PATH, locale, and proxy settings.
CLEAN_ENV=(
    "HOME=${HOME}"
    "USER=${USER:-$(id -un)}"
    "LOGNAME=${LOGNAME:-${USER:-$(id -un)}}"
    "SHELL=/bin/bash"
    "TERM=${TERM:-xterm}"
    "PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    "PX4_ENV_ROOT=${ROOT}"
    "ROS_DISTRO=${ROS_DISTRO}"
    "PX4_INSTALL_PROFILE=${profile}"
)

for name in \
    LANG \
    LC_ALL \
    http_proxy \
    https_proxy \
    HTTP_PROXY \
    HTTPS_PROXY \
    ALL_PROXY \
    NO_PROXY \
    no_proxy
do
    if [[ -n "${!name-}" ]]; then
        CLEAN_ENV+=("${name}=${!name}")
    fi
done

# ROS setup scripts may inspect unset variables, so nounset is disabled only
# while sourcing the distro environment and re-enabled for the actual build.
env -i "${CLEAN_ENV[@]}" \
    bash --noprofile --norc <<'EOF_BUILD'
set -eo pipefail

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

cd "${PX4_ENV_ROOT}/ros2"
case "${PX4_INSTALL_PROFILE}" in
    sitl) colcon build --packages-skip px4_mocap_bridge ;;
    hardware) colcon build --packages-skip px4_position_mode ;;
esac
EOF_BUILD

echo
echo "ROS 2 workspace built successfully."
