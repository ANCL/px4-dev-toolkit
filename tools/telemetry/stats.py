"""Online callback-interval statistics on a monotonic host clock."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class TopicStats:
    """Online arrival-time statistics for one ROS 2 topic."""

    samples: int = 0
    first_ns: int | None = None
    last_ns: int | None = None
    interval_count: int = 0
    mean_interval_ns: float = 0.0
    interval_m2: float = 0.0
    min_interval_ns: int | None = None
    max_interval_ns: int | None = None

    def add(self, now_ns: int) -> None:
        """
        Incorporate one topic-arrival timestamp.

        Inputs:
            Monotonic callback-arrival time [ns].

        Method:
            Update sample count, interval extrema, and Welford online interval
            variance without retaining the complete arrival history.

        Effects:
            Mutates this TopicStats instance; derived rate/jitter properties
            consume the accumulated state.
        """
        self.samples += 1

        if self.first_ns is None:
            self.first_ns = now_ns
            self.last_ns = now_ns
            return

        assert self.last_ns is not None

        interval_ns = now_ns - self.last_ns
        self.last_ns = now_ns
        self.interval_count += 1

        if (
            self.min_interval_ns is None
            or interval_ns < self.min_interval_ns
        ):
            self.min_interval_ns = interval_ns

        if (
            self.max_interval_ns is None
            or interval_ns > self.max_interval_ns
        ):
            self.max_interval_ns = interval_ns

        # Welford's online update accumulates interval variance without
        # retaining every arrival interval or suffering the cancellation of
        # E[x^2] - E[x]^2.
        delta = interval_ns - self.mean_interval_ns
        self.mean_interval_ns += delta / self.interval_count
        self.interval_m2 += (
            delta * (interval_ns - self.mean_interval_ns)
        )

    @property
    def rate_hz(self) -> float | None:
        if (
            self.interval_count == 0
            or self.first_ns is None
            or self.last_ns is None
            or self.last_ns <= self.first_ns
        ):
            return None

        elapsed_s = (self.last_ns - self.first_ns) / 1e9
        return self.interval_count / elapsed_s

    @property
    def min_interval_ms(self) -> float | None:
        if self.min_interval_ns is None:
            return None

        return self.min_interval_ns / 1e6

    @property
    def max_interval_ms(self) -> float | None:
        if self.max_interval_ns is None:
            return None

        return self.max_interval_ns / 1e6

    @property
    def std_interval_ms(self) -> float | None:
        if self.interval_count == 0:
            return None

        variance_ns2 = self.interval_m2 / self.interval_count
        return math.sqrt(max(variance_ns2, 0.0)) / 1e6
