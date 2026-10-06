#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4="${ROOT}/px4/PX4-Autopilot"

if [[ ! -f "${PX4}/Tools/setup/ubuntu.sh" ]]; then
    echo "ERROR: PX4 source is missing."
    echo "Run fetch_sources.sh first."
    exit 1
fi

# Keep the user-facing profiles positive. PX4's own installer expresses these
# selections as exclusions, so translate the toolkit profile here.
case "${1:-}" in
    sitl)
        px4_args=(--no-nuttx)
        ;;
    experiment)
        px4_args=(--no-sim-tools)
        ;;
    all)
        px4_args=()
        ;;
    *)
        echo "Usage: ./setup/install_px4_dependencies.sh sitl|experiment|all" >&2
        exit 2
        ;;
esac

echo "Installing PX4 ${1} development dependencies..."

bash "${PX4}/Tools/setup/ubuntu.sh" "${px4_args[@]}"

echo "PX4 ${1} dependencies ready."
