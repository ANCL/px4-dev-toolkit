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
