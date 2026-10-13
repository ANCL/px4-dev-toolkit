#!/usr/bin/env bash
# Graceful shutdown of one named tmux session and its owned process tree.
# A launcher's environment-specific cleanup runs after this shared helper.

stop_owned_session() {
    local session="$1"
    if ! tmux has-session -t "${session}" 2>/dev/null; then
        echo "tmux session '${session}' is not running."
        return 0
    fi

    local -a pane_pids=()
    local -a pane_targets=()
    local -a stack_pids=()
    local pane_pid
    local pid
    local target

    mapfile -t pane_pids < <(
        tmux list-panes -s -t "${session}" -F '#{pane_pid}'
    )
    mapfile -t pane_targets < <(
        tmux list-panes -s -t "${session}" \
            -F '#{session_name}:#{window_name}.#{pane_index}'
    )

    for pane_pid in "${pane_pids[@]}"; do
        while IFS= read -r pid; do
            [[ -n "${pid}" ]] && stack_pids+=("${pid}")
        done < <(collect_process_tree "${pane_pid}")
    done

    if (( ${#stack_pids[@]} > 0 )); then
        mapfile -t stack_pids < <(
            printf '%s\n' "${stack_pids[@]}" | sort -n -u
        )
    fi

    echo "Stopping tmux session '${session}'..."
    for target in "${pane_targets[@]}"; do
        tmux send-keys -t "${target}" C-c 2>/dev/null || true
    done

    if (( ${#stack_pids[@]} > 0 )) &&
        ! wait_for_pids 15 "${stack_pids[@]}"; then
        echo "Processes still running; sending SIGINT."
        signal_alive_pids INT "${stack_pids[@]}"
        if ! wait_for_pids 15 "${stack_pids[@]}"; then
            echo "Processes still running; sending SIGTERM."
            signal_alive_pids TERM "${stack_pids[@]}"
            if ! wait_for_pids 2 "${stack_pids[@]}"; then
                echo "Processes still running; sending SIGKILL."
                signal_alive_pids KILL "${stack_pids[@]}"
            fi
        fi
    fi

    tmux kill-session -t "${session}" 2>/dev/null || true
    echo "tmux session '${session}' stopped."
}
