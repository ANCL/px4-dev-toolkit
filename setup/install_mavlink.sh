#!/usr/bin/env bash
# Install the same MAVLink tools for SITL and the companion computer.
# No MAVProxy dependency, PX4 parameter changes or flight operations.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_DIR="${ROOT}/.venv/mavlink"
source "${ROOT}/config/sources/px4.env"
PIN="5e38c7a32391a1ba75cde05c61825257ad617411"
SOURCE="${HOME}/.cache/px4-dev-toolkit/mavlink-router"

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    git curl meson ninja-build pkg-config \
    gcc g++ libsystemd-dev python3-venv

python3 -m venv "${ENV_DIR}"

"${ENV_DIR}/bin/python" -m pip install \
    --disable-pip-version-check \
    "pymavlink==2.4.49" \
    "pyserial==3.5" \
    "PyYAML==6.0.2"

# Use the installed PX4 revision when present; otherwise resolve the selected
# remote branch (the companion does not install PX4 firmware sources).
if [[ -d "${ROOT}/px4/PX4-Autopilot/.git" ]]; then
    PX4_REV="$(git -C "${ROOT}/px4/PX4-Autopilot" rev-parse HEAD)"
else
    PX4_REV="$(git ls-remote "${PX4_REPO}" "refs/heads/${PX4_REF}" | awk 'NR == 1 { print $1 }')"
    [[ -n "${PX4_REV}" ]] || PX4_REV="${PX4_REF}"
fi

PX4_SLUG="${PX4_REPO#https://github.com/}"
PX4_SLUG="${PX4_SLUG%.git}"
[[ "${PX4_SLUG}" != "${PX4_REPO}" ]] || {
    echo "ERROR: PX4_REPO must be a github.com HTTPS URL." >&2
    exit 1
}

curl --fail --location --silent --show-error \
    "https://raw.githubusercontent.com/${PX4_SLUG}/${PX4_REV}/Tools/mavlink_shell.py" \
    --output "${ENV_DIR}/bin/mavlink_shell.py"

"${ENV_DIR}/bin/python" -m py_compile \
    "${ENV_DIR}/bin/mavlink_shell.py"

if [[ ! -x "${ENV_DIR}/bin/mavlink-routerd" ]]; then
    if [[ ! -d "${SOURCE}/.git" ]]; then
        mkdir -p "$(dirname "${SOURCE}")"
        git clone https://github.com/mavlink-router/mavlink-router.git \
            "${SOURCE}"
    fi

    git -C "${SOURCE}" fetch origin "${PIN}"
    git -C "${SOURCE}" checkout --detach "${PIN}"
    git -C "${SOURCE}" submodule update --init --recursive

    if [[ ! -d "${SOURCE}/build" ]]; then
        meson setup "${SOURCE}/build" "${SOURCE}" \
            --buildtype=release
    fi

    ninja -C "${SOURCE}/build" src/mavlink-routerd

    install -m 755 \
        "${SOURCE}/build/src/mavlink-routerd" \
        "${ENV_DIR}/bin/mavlink-routerd"
fi

"${ENV_DIR}/bin/python" -c \
    'import pymavlink, serial, yaml; print("MAVLink Python ready")'

echo "MAVLink shell and router installed."
