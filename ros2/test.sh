#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source "${ROOT}/config/runtime/common.env"

CLEAN_ENV=(
    "HOME=${HOME}"
    "USER=${USER:-$(id -un)}"
    "LOGNAME=${LOGNAME:-${USER:-$(id -un)}}"
    "SHELL=/bin/bash"
    "TERM=${TERM:-xterm}"
    "PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    "PX4_ENV_ROOT=${ROOT}"
    "ROS_DISTRO=${ROS_DISTRO}"
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

env -i "${CLEAN_ENV[@]}" \
    bash --noprofile --norc <<'EOF_TEST'
set -eo pipefail

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${PX4_ENV_ROOT}/ros2/install/setup.bash"
set -u

cd "${PX4_ENV_ROOT}/ros2"

# Test only packages maintained by this toolkit. Fetched third-party
# repositories retain their own upstream test suites.
TOOLKIT_PACKAGES=(
    offboard_controllers
    px4_bringup
    px4_control_common
    px4_mocap_bridge
    px4_position_takeoff_hover
)

RESULT_BASE="build/toolkit_test_results"

rm -rf "${RESULT_BASE}"

# Test-runner contract:
#   Action: run pure-Python analysis regressions and maintained ROS package
#           tests, then always print verbose collected colcon results.
#   Failure: preserve the first failing suite after all available diagnostics
#            have been emitted.
set +e

cd "${PX4_ENV_ROOT}"

python3 -m unittest discover \
    -s analysis/tests \
    -p 'test_*.py'
analysis_status=$?

cd "${PX4_ENV_ROOT}/ros2"

colcon test \
    --packages-select "${TOOLKIT_PACKAGES[@]}" \
    --test-result-base "${RESULT_BASE}" \
    --return-code-on-test-failure
test_status=$?

colcon test-result \
    --test-result-base "${RESULT_BASE}" \
    --verbose
result_status=$?

set -e

if (( analysis_status != 0 )); then
    exit "${analysis_status}"
fi

if (( test_status != 0 )); then
    exit "${test_status}"
fi

exit "${result_status}"
EOF_TEST

echo
echo "Toolkit ROS 2 tests passed."
