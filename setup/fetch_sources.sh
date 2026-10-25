#!/usr/bin/env bash
# Fetch one independent, pinned source set without resetting existing work.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
profile="${1:-}"

if (( $# != 1 )); then
    echo "Usage: ./setup/fetch_sources.sh sitl|hardware" >&2
    exit 2
fi

case "${profile}" in
    sitl|hardware) ;;
    *)
        echo "Usage: ./setup/fetch_sources.sh sitl|hardware" >&2
        exit 2
        ;;
esac

manifest="${ROOT}/config/sources/${profile}.repos"
if [[ ! -f "${manifest}" ]]; then
    echo "ERROR: Source manifest not found: ${manifest}" >&2
    exit 1
fi
if ! command -v vcs >/dev/null 2>&1; then
    echo "ERROR: vcs is not installed." >&2
    exit 1
fi

mapfile -t repositories < <(
    awk '
        /^  [^[:space:]].*:$/ {
            sub(/^  /, "")
            sub(/:$/, "")
            print
        }
    ' "${manifest}"
)

if (( ${#repositories[@]} == 0 )); then
    echo "ERROR: Source manifest contains no repositories: ${manifest}" >&2
    exit 1
fi

existing=0
for relative in "${repositories[@]}"; do
    if [[ -d "${ROOT}/${relative}" ]]; then
        existing=$((existing + 1))
    fi
done

if (( existing == 0 )); then
    echo "Fetching ${profile} sources..."
    vcs import --input "${manifest}" "${ROOT}"
elif (( existing == ${#repositories[@]} )); then
    # Existing source checkouts are intentionally not reset or updated.
    echo "${profile} sources already exist; keeping current checkouts."
elif [[ "${profile}" == "hardware" ]] \
     && (( existing == ${#repositories[@]} - 1 )) \
     && [[ ! -e "${ROOT}/tools/MAVProxy" ]] \
     && [[ -d "${ROOT}/ros2/src/px4_msgs" ]] \
     && [[ -d "${ROOT}/ros2/src/Micro-XRCE-DDS-Agent" ]] \
     && [[ -d "${ROOT}/ros2/src/vicon_receiver" ]]; then
    # Upgrade an existing three-source companion checkout by fetching ONLY
    # the newly required pinned MAVProxy source. Never reset other sources.
    temporary_manifest="$(mktemp --suffix=.repos)"
    trap 'rm -f "${temporary_manifest}"' EXIT
    python3 - "${manifest}" "${temporary_manifest}" <<'PY'
from pathlib import Path
import sys
import yaml

config = yaml.safe_load(Path(sys.argv[1]).read_text())
repositories = config.get("repositories", {})
if "tools/MAVProxy" not in repositories:
    raise SystemExit("hardware source manifest is missing tools/MAVProxy")
Path(sys.argv[2]).write_text(yaml.safe_dump({
    "repositories": {"tools/MAVProxy": repositories["tools/MAVProxy"]}
}, sort_keys=False))
PY
    echo "Fetching pinned MAVProxy for existing hardware checkout..."
    vcs import --input "${temporary_manifest}" "${ROOT}"
    rm -f "${temporary_manifest}"
    trap - EXIT
else
    echo "ERROR: ${profile} source tree is partially populated." >&2
    echo "Refusing to modify it automatically." >&2
    exit 1
fi

if [[ "${profile}" == "sitl" ]]; then
    echo "Initializing PX4 nested submodules..."
    git -C "${ROOT}/px4/PX4-Autopilot" submodule update --init --recursive
fi

echo "${profile} source repositories ready."
