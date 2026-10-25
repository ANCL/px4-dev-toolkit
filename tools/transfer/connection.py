"""Reusable PX4 MAVLink-FTP client for either toolkit runtime.

The MAVLink router owns the PX4 connection. Transfer tools communicate
over their own local UDP endpoint without opening the serial device.
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
import re
import time
import zlib

ENDPOINT = "udpin:127.0.0.1:14662"
TRAJECTORY_DIR = "/fs/microsd/trajectories"
_NAME = re.compile(r"^[A-Za-z0-9_-]+\.traj$")


class TransferError(RuntimeError):
    """A transfer was rejected or was not positively verified."""


class MissingDirectory(TransferError):
    """The remote FTP catalog is absent, not merely unreachable."""


def successful(response) -> bool:
    """Interpret the pinned pymavlink MAVFTP return value without guessing."""
    from pymavlink.mavftp import FtpError
    return response is not None and response.error_code == FtpError.Success


def require(response, operation: str) -> None:
    if not successful(response):
        code = getattr(response, "error_code", "no response")
        raise TransferError(f"{operation}: MAVLink FTP error {code}")


@dataclass(frozen=True)
class RemoteEntry:
    path: str
    size: int
    is_dir: bool


class Connection:
    """One bounded CLI operation over an independent MAVLink endpoint."""

    def __init__(self, master, heartbeat, *, init_ftp=True):
        from pymavlink.mavftp import MAVFTP
        self.master = master
        self.system_id = heartbeat.get_srcSystem()
        self.component_id = heartbeat.get_srcComponent()
        self.armed = bool(heartbeat.base_mode & 128)
        self.ftp = MAVFTP(master, self.system_id, self.component_id) if init_ftp else None

    @staticmethod
    def open(endpoint=ENDPOINT, timeout=12, *, init_ftp=True):
        from pymavlink import mavutil
        master = mavutil.mavlink_connection(
            endpoint, source_system=254, source_component=190,
            autoreconnect=False,
        )
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline:
                msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
                if msg is None or msg.get_srcSystem() == 254:
                    continue
                if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                    continue
                return Connection(master, msg, init_ftp=init_ftp)
            raise TransferError("no PX4 heartbeat from the toolkit MAVLink router")
        except BaseException:
            master.close()
            raise

    def close(self):
        self.master.close()

    def require_disarmed(self):
        if self.armed:
            raise TransferError("PX4 is armed; finish the flight and disarm first")

    def entries(self, folder):
        from pymavlink.mavftp import FtpError
        response = self.ftp.cmd_list([folder])
        if response is not None and response.error_code == FtpError.FileNotFound:
            raise MissingDirectory(folder)
        require(response, f"list {folder}")
        return [RemoteEntry(folder.rstrip("/") + "/" + e.name, e.size_b, e.is_dir)
                for e in self.ftp.list_result if e.name not in (".", "..")]

    def ensure_directory(self, folder):
        try:
            self.entries(folder)
            return
        except MissingDirectory:
            require(self.ftp.cmd_mkdir([folder]), f"mkdir {folder}")
            self.entries(folder)

    def remove(self, remote):
        require(self.ftp.cmd_rm([remote]), f"remove {remote}")

    def upload(self, local: Path, remote: str):
        """Require MAVLink FTP completion without downloading the file."""
        payload = local.read_bytes()
        if len(payload) > 131072:
            raise TransferError("trajectory file unexpectedly exceeds 128 KiB")

        completed = []

        def on_complete(length):
            completed.append(length)

        require(
            self.ftp.cmd_put([str(local), remote], callback=on_complete),
            f"start upload {remote}",
        )
        require(
            self.ftp.process_ftp_reply("CreateFile", timeout=90),
            f"upload {remote}",
        )

        if completed != [len(payload)]:
            raise TransferError(
                f"{remote}: FTP did not confirm the full transfer"
            )

        return zlib.crc32(payload) & 0xFFFFFFFF


def trajectory_names(entries):
    """Only delete plain .traj catalog entries, never arbitrary PX4 files."""
    return sorted(entry.path for entry in entries if not entry.is_dir
                  and _NAME.fullmatch(entry.path.rsplit("/", 1)[-1]))
