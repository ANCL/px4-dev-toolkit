#!/usr/bin/env bash
# Install the same MAVLink tools for SITL and the companion computer.
# No MAVProxy dependency, PX4 parameter changes or flight operations.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_DIR="${ROOT}/.venv/mavlink"
PIN="5e38c7a32391a1ba75cde05c61825257ad617411"
PX4_PIN="5dd32eb1e7ab8b79b8ece462ce5f22cd75272c56"
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

# Use the upstream PX4 shell client from our pinned firmware revision.
curl --fail --location --silent --show-error \
    "https://raw.githubusercontent.com/ANCL/PX4-Autopilot/${PX4_PIN}/Tools/mavlink_shell.py" \
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
