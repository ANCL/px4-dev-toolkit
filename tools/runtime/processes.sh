#!/usr/bin/env bash
# Shared process-tree and bounded shutdown primitives for both runtimes.
# Source from the concrete SITL/hardware launcher, not independently.

pid_is_alive() {
    local pid="$1"
    local state

    [[ -r "/proc/${pid}/stat" ]] || return 1

    state="$(
        awk '{print $3}' "/proc/${pid}/stat" 2>/dev/null || true
    )"

    [[ -n "${state}" && "${state}" != "Z" ]]
}

collect_process_tree() {
    local root_pid="$1"
    local -a queue=("${root_pid}")
    local index=0
    local pid
    local child

    while (( index < ${#queue[@]} )); do
        pid="${queue[$index]}"
        index=$((index + 1))

        echo "${pid}"

        while IFS= read -r child; do
            if [[ -n "${child}" ]]; then
                queue+=("${child}")
            fi
        done < <(
            pgrep -P "${pid}" 2>/dev/null || true
        )
    done
}

any_pid_alive() {
    local pid

    for pid in "$@"; do
        if pid_is_alive "${pid}"; then
            return 0
        fi
    done

    return 1
}

wait_for_pids() {
    local timeout_seconds="$1"
    shift

    local elapsed=0

    while (( elapsed < timeout_seconds )); do
        if ! any_pid_alive "$@"; then
            return 0
        fi

        sleep 1
        elapsed=$((elapsed + 1))
    done

    ! any_pid_alive "$@"
}

signal_alive_pids() {
    local signal="$1"
    shift

    local pid

    for pid in "$@"; do
        if pid_is_alive "${pid}"; then
            kill -s "${signal}" "${pid}" 2>/dev/null || true
        fi
    done
}
