"""Regression tests for SE(3) comparison boundary semantics."""

from pathlib import Path
import tempfile
import unittest

from analysis.profiles.se3.comparison import (
    ComparisonRun,
    step_response_metrics,
    trajectory_segments,
)


def vector_series(
    times: list[float],
    x: list[float],
    y: list[float] | None = None,
    z: list[float] | None = None,
) -> dict[str, list[float]]:
    count = len(times)

    return {
        "times_s": times,
        "x": x,
        "y": y if y is not None else [0.0] * count,
        "z": z if z is not None else [0.0] * count,
    }


def empty_vector_series() -> dict[str, list[float]]:
    return {
        "times_s": [],
        "x": [],
        "y": [],
        "z": [],
    }


class TrajectorySegmentTests(unittest.TestCase):
    def test_direct_reusable_segment_matches_runtime_selection(self) -> None:
        """A reusable segment may be selected directly, not only by trajectory."""

        configuration = """
segments:
  primitive:
    type: line
    duration: 2.5
    offset: [1.0, 0.0, 0.0]

trajectories:
  wrapped:
    segments:
      - primitive
"""

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trajectories.yaml"
            path.write_text(configuration)

            segments = trajectory_segments(
                path,
                "primitive",
            )

        self.assertEqual(len(segments), 1)
        self.assertEqual(segments[0]["name"], "primitive")
        self.assertEqual(segments[0]["type"], "line")
        self.assertEqual(segments[0]["start_s"], 0.0)
        self.assertEqual(segments[0]["end_s"], 2.5)
        self.assertEqual(
            segments[0]["offset"],
            [1.0, 0.0, 0.0],
        )


class StepResponseTests(unittest.TestCase):
    def test_exact_boundary_reference_belongs_to_previous_segment(self) -> None:
        """Step target comes from the first reference after the boundary."""

        # Oracle:
        #   Runtime Sequence semantics give t=2.0 to the preceding hold.
        #   The new +1 m step therefore appears only after t=2.0.
        reference = vector_series(
            [1.9, 2.0, 2.001, 3.0],
            [0.0, 0.0, 1.0, 1.0],
        )

        actual = vector_series(
            [2.0, 2.1, 2.2, 2.7],
            [0.0, 0.5, 1.0, 1.0],
        )

        empty = empty_vector_series()

        run = ComparisonRun(
            label="test",
            bag_path=Path("/tmp/test"),
            handoff="acceleration",
            direct_controller="n/a",
            duration_s=3.0,
            position_actual=actual,
            position_reference=reference,
            position_error=empty,
            velocity_error=empty,
            yaw_error=empty,
            thrust=empty,
            torque=empty,
            body_rates=empty,
            motors={},
        )

        metrics = step_response_metrics(
            run,
            {
                "name": "step_x_1m",
                "type": "step",
                "start_s": 2.0,
                "end_s": 3.0,
                "offset": [1.0, 0.0, 0.0],
            },
            3.0,
        )

        # Expected:
        #   With target=1 m, normalized response is [0, 0.5, 1, 1].
        #   Sampled 10-90 % crossings are therefore 2.1 s and 2.2 s.
        self.assertAlmostEqual(
            float(metrics["rise_s"]),
            0.1,
            places=12,
        )
        self.assertAlmostEqual(
            float(metrics["overshoot_percent"]),
            0.0,
            places=12,
        )
        self.assertAlmostEqual(
            float(metrics["residual_rms_m"]),
            0.0,
            places=12,
        )


if __name__ == "__main__":
    unittest.main()
