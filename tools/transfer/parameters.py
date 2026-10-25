"""Explicit PX4 controller-parameter loading through the toolkit MAVLink link.

PX4 encodes MAV_PARAM_TYPE_INT32 parameters as raw int32 bits in the float32
PARAM_VALUE / PARAM_SET wire field, while REAL32 uses numeric float values.
Read and write using the type actually reported by the connected firmware.
"""

from __future__ import annotations

import math
from pathlib import Path
import re
import struct
import time

from .connection import TransferError

_PARAM_NAME = re.compile(r"^[A-Z][A-Z0-9_]{0,15}$")


def read_profile(path: Path) -> dict[str, float]:
    """Validate a MAVProxy-compatible NAME VALUE .parm profile before I/O."""
    if not path.is_file():
        raise TransferError(f"parameter profile does not exist: {path}")
    values: dict[str, float] = {}
    for lineno, text in enumerate(path.read_text().splitlines(), 1):
        line = text.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.replace("\t", " ").split()
        if len(parts) != 2 or not _PARAM_NAME.fullmatch(parts[0]):
            raise TransferError(f"{path.name}:{lineno}: expected PX4 NAME VALUE")
        name, source = parts
        if name in values:
            raise TransferError(f"{path.name}:{lineno}: duplicate parameter {name}")
        try:
            value = float(source)
        except ValueError as exc:
            raise TransferError(f"{path.name}:{lineno}: invalid {name} value") from exc
        if not math.isfinite(value):
            raise TransferError(f"{path.name}:{lineno}: nonfinite {name} value")
        values[name] = value
    if not values:
        raise TransferError(f"parameter profile has no uncommented values: {path}")
    return values


def int32_as_float(value: int) -> float:
    """PX4 PARAM_SET payload for an integer, respecting bitwise encoding."""
    if not -(2**31) <= value < 2**31:
        raise TransferError("integer parameter out of int32 range")
    return struct.unpack("<f", struct.pack("<i", value))[0]


def float_as_int32(value: float) -> int:
    return struct.unpack("<i", struct.pack("<f", value))[0]


def decode_value(message, mavlink) -> float | int:
    if message.param_type == mavlink.MAV_PARAM_TYPE_INT32:
        return float_as_int32(message.param_value)
    if message.param_type == mavlink.MAV_PARAM_TYPE_REAL32:
        return float(message.param_value)
    raise TransferError(f"unsupported parameter type {message.param_type}")


def desired_wire_value(value: float, param_type: int, mavlink):
    if param_type == mavlink.MAV_PARAM_TYPE_INT32:
        if not value.is_integer() or not -(2**31) <= value < 2**31:
            raise TransferError(f"expected an int32 value, received {value:g}")
        return int32_as_float(int(value))
    if param_type == mavlink.MAV_PARAM_TYPE_REAL32:
        try:
            encoded = struct.unpack("<f", struct.pack("<f", value))[0]
        except (OverflowError, struct.error) as exc:
            raise TransferError(f"float32 parameter out of range: {value:g}") from exc
        if not math.isfinite(encoded):
            raise TransferError(f"float32 parameter out of range: {value:g}")
        return encoded
    raise TransferError(f"unsupported PX4 parameter type {param_type}")


def equivalent(actual: float | int, desired: float, param_type: int, mavlink) -> bool:
    wire = desired_wire_value(desired, param_type, mavlink)
    if param_type == mavlink.MAV_PARAM_TYPE_INT32:
        return actual == int(desired)
    return math.isclose(float(actual), wire, rel_tol=1e-6, abs_tol=1e-6)


def name_of(message) -> str:
    raw = message.param_id
    if isinstance(raw, bytes):
        return raw.split(b"\0", 1)[0].decode("ascii", errors="replace")
    return str(raw).split("\0", 1)[0]


class ParameterLink:
    """Read and apply named PX4 parameters using the existing forwarded link."""

    def __init__(self, connection):
        from pymavlink import mavutil
        self.connection = connection
        self.master = connection.master
        self.mavlink = mavutil.mavlink

    def _read_matching(self, name: str, timeout: float = 2.5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.master.recv_match(
                type="PARAM_VALUE", blocking=True,
                timeout=min(0.5, max(0.01, deadline - time.monotonic())),
            )
            if (msg is not None and
                msg.get_srcSystem() == self.connection.system_id and
                msg.get_srcComponent() == self.connection.component_id and
                name_of(msg) == name):
                return msg
        return None

    def read(self, name: str):
        for _ in range(3):
            self.master.mav.param_request_read_send(
                self.connection.system_id, self.connection.component_id,
                name.encode("ascii"), -1,
            )
            message = self._read_matching(name)
            if message is not None:
                return message
        raise TransferError(f"no PX4 parameter response for {name}")

    def confirm_disarmed(self):
        """Do not rely on an old heartbeat captured at initial connection."""
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            msg = self.master.recv_match(
                type="HEARTBEAT", blocking=True,
                timeout=min(0.8, max(0.01, deadline - time.monotonic())),
            )
            if (msg is None or msg.get_srcSystem() != self.connection.system_id or
                msg.get_srcComponent() != self.connection.component_id):
                continue
            if msg.base_mode & self.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                raise TransferError("PX4 is armed; refusing parameter load")
            return
        raise TransferError("PX4 disarmed state could not be confirmed")

    def load(self, values: dict[str, float]) -> list[str]:
        """Preflight-check every requested parameter before the first write."""
        snapshot = {}
        for name, value in values.items():
            msg = self.read(name)
            current = decode_value(msg, self.mavlink)
            desired_wire_value(value, msg.param_type, self.mavlink)
            snapshot[name] = (current, msg.param_type)
        self.confirm_disarmed()
        changed = []
        for name, requested in values.items():
            current, kind = snapshot[name]
            if equivalent(current, requested, kind, self.mavlink):
                print(f"unchanged: {name} = {current}")
                continue
            self.confirm_disarmed()
            encoded = desired_wire_value(requested, kind, self.mavlink)
            self.master.mav.param_set_send(
                self.connection.system_id, self.connection.component_id,
                name.encode("ascii"), encoded, kind,
            )
            # PX4 must acknowledge the write through a new read request.
            verified = self.read(name)
            if verified.param_type != kind or not equivalent(
                decode_value(verified, self.mavlink), requested, kind, self.mavlink
            ):
                raise TransferError(
                    f"{name}: PX4 readback differs; earlier writes may have applied"
                )
            changed.append(name)
            print(f"verified: {name}: {current} -> {decode_value(verified, self.mavlink)}")
        return changed
