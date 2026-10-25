"""Recording readers with one control-pipeline observation contract.

Both supported readers return the same BagData/TimedSample interface to
analysis/pipeline. Native ROS bag profiles may request additional topics;
generic flight analysis asks for whatever PX4 control layers were recorded.
Other telemetry transports can implement the same interface without rewriting
statistics or plotting.
"""
from __future__ import annotations

from pathlib import Path


def read_flight(path: Path):
    """Open a ULog or ROS 2 bag as PX4 pipeline observations.

    Each source preserves its own monotonic/receive timestamp domain;
    this function never attempts cross-device clock synchronization.
    """
    from analysis.pipeline.topics import CONTROL_PIPELINE_TOPICS
    from analysis.core import read_bag
    from .ulog import read_ulog

    path = Path(path).expanduser().resolve()
    if path.is_file() and path.suffix.lower() == ".ulg":
        return read_ulog(path).bag
    if path.is_dir() and (path / "metadata.yaml").is_file():
        return read_bag(path, (), CONTROL_PIPELINE_TOPICS)
    raise ValueError(f"not a PX4 ULog or ROS 2 bag: {path}")
