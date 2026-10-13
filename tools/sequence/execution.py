"""Run and interrupt one ROS 2 launch process group safely."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import sys


def stop_process(process: subprocess.Popen[str]) -> None:
    """Stop one ROS launch process group.

    Input:
        Popen object whose PID is also the process-group leader.

    Method:
        Escalate SIGINT -> SIGTERM -> SIGKILL only when the group survives the
        previous shutdown stage.

    Returns:
        Nothing; the process is reaped before returning.
    """
    if process.poll() is not None:
        return

    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=5.0)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass

    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=3.0)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass

    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass

    process.wait()


def run_step(
    command: list[str],
    log_path: Path,
) -> int:
    """Run one experiment step while preserving process and log ownership.

    Inputs:
        Complete ros2 launch command and the step log path.

    Method:
        Start the launch in its own process group, stream combined output to
        both terminal and log, and stop the whole group on interruption.

    Returns:
        Launch-process exit status.
    """
    print()
    print("$", " ".join(command))
    print()

    with log_path.open("w") as log:
        # ros2 launch can create several descendants. A new session gives the
        # step its own process group so interruption can stop the complete
        # launch tree rather than only the parent process.
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )

        assert process.stdout is not None

        try:
            for line in process.stdout:
                sys.stdout.write(line)
                sys.stdout.flush()
                log.write(line)
                log.flush()

            return process.wait()

        except KeyboardInterrupt:
            stop_process(process)
            raise
