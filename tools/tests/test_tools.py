"""Pure-Python contracts for refactored toolkit command helpers."""

from pathlib import Path
import sys
import tempfile
import unittest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))

from sequence.config import load_sequence, launch_value, validate_name
from sequence.manifest import write_manifest, write_experiment_metadata
from telemetry.stats import TopicStats


class SequenceContracts(unittest.TestCase):
    def test_name_validation(self):
        self.assertEqual(validate_name("step_1", "name"), "step_1")
        for invalid in ("", "../foo", "hello world", "a/b"):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    validate_name(invalid, "name")

    def test_launch_values(self):
        self.assertEqual(launch_value(True), "true")
        self.assertEqual(launch_value(False), "false")
        self.assertEqual(launch_value(42), "42")
        with self.assertRaises(ValueError):
            launch_value([1, 2])

    def test_valid_sequence(self):
        with tempfile.TemporaryDirectory() as temp:
            config = Path(temp) / "study.yaml"
            config.write_text(
                "name: test\nprofile: se3\nrecording_output_argument: recording_output\n"
                "steps:\n  - name: first\n    package: offboard_controllers\n"
                "    launch: se3.launch.py\n    arguments:\n      record: true\n"
            )
            loaded = load_sequence(config)
            self.assertEqual(loaded["steps"][0]["arguments"]["record"], "true")

    def test_bag_metadata(self):
        with tempfile.TemporaryDirectory() as temp:
            bag = Path(temp)
            write_experiment_metadata(bag, "se3", "study", {
                "name": "first", "arguments": {"handoff": "acceleration"}
            })
            self.assertIn('"handoff": "acceleration"', (bag / "experiment.json").read_text())
            manifest = bag / "manifest.json"
            write_manifest(manifest, {"result": "success"})
            self.assertIn('"result": "success"', manifest.read_text())


class TopicRateContracts(unittest.TestCase):
    def test_online_rates(self):
        stats = TopicStats()
        self.assertIsNone(stats.rate_hz)
        for timestamp in (0, 100_000_000, 200_000_000):
            stats.add(timestamp)
        self.assertEqual(stats.samples, 3)
        self.assertAlmostEqual(stats.rate_hz, 10.0)
        self.assertAlmostEqual(stats.min_interval_ms, 100.0)
        self.assertAlmostEqual(stats.max_interval_ms, 100.0)
        self.assertAlmostEqual(stats.std_interval_ms, 0.0)


if __name__ == "__main__":
    unittest.main()
