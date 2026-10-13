#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4="${ROOT}/px4/PX4-Autopilot"

if [[ ! -f "${PX4}/Tools/setup/ubuntu.sh" ]]; then
    echo "ERROR: PX4 source is missing."
    echo "Run fetch_sources.sh first."
    exit 1
fi

if [[ "${1:-}" != "sitl" ]]; then
    echo "Usage: ./setup/install_px4_dependencies.sh sitl" >&2
    exit 2
fi

echo "Installing PX4 SITL dependencies (without firmware toolchains)..."
bash "${PX4}/Tools/setup/ubuntu.sh" --no-nuttx
echo "PX4 SITL dependencies ready."
