#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
    echo "Usage: ./setup/fetch_sources.sh common|sitl|experiment|all"
}

fetch_scope() {
    local scope="$1"
    local manifest="${ROOT}/config/sources/${scope}.repos"
    local -a repositories=()
    local existing=0
    local relative
    local repository

    mapfile -t repositories < <(
        awk '
            /^  [^[:space:]].*:$/ {
                sub(/^  /, "")
                sub(/:$/, "")
                print
            }
        ' "${manifest}"
    )

    # Treat each manifest as an all-or-nothing checkout set. Automatically
    # importing into a partially populated source tree could mix user-modified
    # or manually checked-out repositories with different pinned revisions.
    for relative in "${repositories[@]}"; do
        repository="${ROOT}/${relative}"

        if [[ -d "${repository}" ]]; then
            existing=$((existing + 1))
        fi
    done

    if (( existing == 0 )); then
        echo "Fetching ${scope} sources..."
        vcs import --input "${manifest}" "${ROOT}"
    elif (( existing == ${#repositories[@]} )); then
        # Existing repositories are deliberately not reset or updated here.
        # fetch_sources.sh must never discard local work in external checkouts.
        echo "${scope} sources already exist; keeping current checkouts."
    else
        echo "ERROR: ${scope} source tree is partially populated." >&2
        echo "Refusing to modify it automatically." >&2
        return 1
    fi
}

if ! command -v vcs >/dev/null 2>&1; then
    echo "ERROR: vcs is not installed." >&2
    exit 1
fi

case "${1:-}" in
    common)
        fetch_scope common
        ;;
    sitl)
        fetch_scope common
        fetch_scope sitl
        ;;
    experiment)
        fetch_scope common
        fetch_scope experiment
        ;;
    all)
        fetch_scope common
        fetch_scope sitl
        fetch_scope experiment
        ;;
    *)
        usage
        exit 2
        ;;
esac

if [[ "${1}" == "sitl" || "${1}" == "experiment" || "${1}" == "all" ]]; then
    echo "Initializing PX4 nested submodules..."

    git -C "${ROOT}/px4/PX4-Autopilot" \
        submodule update --init --recursive
fi

echo "Source repositories ready."
