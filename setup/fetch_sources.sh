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
    # PX4 is selected centrally; the other repositories remain vcs-pinned.
    source "${ROOT}/config/sources/px4.env"
    PX4_DIR="${ROOT}/px4/PX4-Autopilot"

    [[ -n "${PX4_REPO:-}" && -n "${PX4_REF:-}" ]] || {
        echo "ERROR: PX4_REPO and PX4_REF must be configured." >&2
        exit 1
    }

    if [[ ! -e "${PX4_DIR}" ]]; then
        mkdir -p "$(dirname "${PX4_DIR}")"
        git clone "${PX4_REPO}" "${PX4_DIR}"
        git -C "${PX4_DIR}" fetch origin "${PX4_REF}"
        git -C "${PX4_DIR}" checkout --detach FETCH_HEAD
    elif [[ ! -d "${PX4_DIR}/.git" ]]; then
        echo "ERROR: PX4 source path is not a Git checkout: ${PX4_DIR}" >&2
        exit 1
    else
        echo "PX4 source already exists; keeping its current commit."
    fi

    git -C "${PX4_DIR}" submodule update --init --recursive
fi

echo "${profile} source repositories ready."
