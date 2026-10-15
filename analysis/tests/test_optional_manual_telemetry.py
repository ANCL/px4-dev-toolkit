"""Regression coverage for reduced hardware DDS telemetry.

These tests use synthetic rosbag samples without depending on a live Pixhawk.
"""

from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from analysis.core import BagData, TimedSample
from analysis.profiles import offboard_takeoff_handoff, position_takeoff_hover


class VehicleStatus:
    ARMING_STATE_DISARMED = 1
    ARMING_STATE_ARMED = 2
    NAVIGATION_STATE_POSCTL = 2
    NAVIGATION_STATE_OFFBOARD = 14
    NAVIGATION_STATE_AUTO_LAND = 18
    NAVIGATION_STATE_DESCEND = 12

    def __init__(self, nav_state: int, arming_state: int):
        self.nav_state = nav_state
        self.arming_state = arming_state


def item(time_s: float, message: object) -> TimedSample:
    return TimedSample(int(time_s * 1e9), message)


def bag(samples: dict) -> BagData:
    return BagData(
        path=Path("/tmp/synthetic-bag"),
        start_ns=0,
        end_ns=int(11e9),
        samples=samples,
    )


def analyze_without_pipeline(profile, data):
    with (
        patch.object(profile, "analyze_control_pipeline", return_value={}),
        patch.object(profile, "control_pipeline_summary", return_value=["pipeline measured"]),
    ):
        return profile.analyze(data)


class ReducedTelemetryAnalysisTests(unittest.TestCase):
    def position_bag(self, include_manual: bool) -> BagData:
        profile = position_takeoff_hover
        messages = {
            profile.STATUS_TOPIC: [
                item(0, VehicleStatus(2, 1)),
                item(1, VehicleStatus(2, 2)),
                item(8, VehicleStatus(18, 2)),
                item(10, VehicleStatus(18, 1)),
            ],
            profile.LOCAL_POSITION_TOPIC: [
                item(4.5, SimpleNamespace(x=0.0, y=0.0, z=0.0)),
                item(5, SimpleNamespace(x=0.3, y=0.0, z=-0.1)),
                item(7, SimpleNamespace(x=0.1, y=0.0, z=-0.03)),
            ],
        }
        if include_manual:
            messages[profile.MANUAL_CONTROL_TOPIC] = [
                item(2, SimpleNamespace(throttle=0.5)),
                item(4, SimpleNamespace(throttle=0.0)),
            ]
        return bag(messages)

    def handoff_bag(self, include_manual: bool) -> BagData:
        profile = offboard_takeoff_handoff
        messages = {
            profile.STATUS_TOPIC: [
                item(0, VehicleStatus(2, 1)),
                item(1, VehicleStatus(2, 2)),
                item(6, VehicleStatus(14, 2)),
            ],
            profile.OFFBOARD_MODE_TOPIC: [item(4, SimpleNamespace())],
            profile.TRAJECTORY_INPUT_TOPIC: [item(4.5, SimpleNamespace())],
        }
        if include_manual:
            messages[profile.MANUAL_CONTROL_TOPIC] = [
                item(2, SimpleNamespace(throttle=0.5)),
                item(3, SimpleNamespace(throttle=0.0)),
            ]
        return bag(messages)

    def test_position_without_manual_has_no_invented_hover_metrics(self):
        profile = position_takeoff_hover
        self.assertIn(profile.MANUAL_CONTROL_TOPIC, profile.OPTIONAL_TOPICS)
        self.assertNotIn(profile.MANUAL_CONTROL_TOPIC, profile.REQUIRED_TOPICS)
        result = analyze_without_pipeline(profile, self.position_bag(False))
        self.assertAlmostEqual(result.metrics["landing_entry_s"], 8.0)
        self.assertAlmostEqual(result.metrics["disarmed_s"], 10.0)
        self.assertNotIn("hover_start_s", result.metrics)
        self.assertNotIn("max_xy_hover_drift_m", result.metrics)
        self.assertIn("unavailable (not recorded)", result.summary)
        self.assertNotIn("STICKS CENTERED", [name for _, name in result.plot_data["markers"]])

    def test_position_legacy_manual_metrics_preserved(self):
        profile = position_takeoff_hover
        result = analyze_without_pipeline(profile, self.position_bag(True))
        self.assertAlmostEqual(result.metrics["climb_start_s"], 2.0)
        self.assertAlmostEqual(result.metrics["hover_start_s"], 4.0)
        self.assertAlmostEqual(result.metrics["hover_duration_s"], 4.0)
        self.assertAlmostEqual(result.metrics["max_xy_hover_drift_m"], 0.3)
        self.assertIn("STICKS CENTERED", [name for _, name in result.plot_data["markers"]])

    def test_handoff_without_manual_uses_confirmed_offboard(self):
        profile = offboard_takeoff_handoff
        self.assertIn(profile.MANUAL_CONTROL_TOPIC, profile.OPTIONAL_TOPICS)
        self.assertNotIn(profile.MANUAL_CONTROL_TOPIC, profile.REQUIRED_TOPICS)
        result = analyze_without_pipeline(profile, self.handoff_bag(False))
        self.assertAlmostEqual(result.metrics["armed_s"], 1.0)
        self.assertAlmostEqual(result.metrics["offboard_entry_s"], 6.0)
        self.assertNotIn("centered_s", result.metrics)
        self.assertIn("unavailable (not recorded)", result.summary)
        self.assertNotIn("STICKS CENTERED", [name for _, name in result.plot_data["markers"]])

    def test_handoff_legacy_manual_metrics_preserved(self):
        profile = offboard_takeoff_handoff
        result = analyze_without_pipeline(profile, self.handoff_bag(True))
        self.assertAlmostEqual(result.metrics["climb_start_s"], 2.0)
        self.assertAlmostEqual(result.metrics["centered_s"], 3.0)
        self.assertAlmostEqual(result.metrics["centered_to_offboard_s"], 3.0)
        self.assertIn("STICKS CENTERED", [name for _, name in result.plot_data["markers"]])

    def test_bad_legacy_manual_stream_still_rejected(self):
        profile = offboard_takeoff_handoff
        data = self.handoff_bag(True)
        data.samples[profile.MANUAL_CONTROL_TOPIC] = [
            item(2, SimpleNamespace(throttle=0.0)),
        ]
        with self.assertRaisesRegex(RuntimeError, "Climb-stick command"):
            analyze_without_pipeline(profile, data)

    def test_hardware_recorder_lists_omit_disabled_dds_topics(self):
        root = Path(__file__).resolve().parents[2]
        filenames = [
            "ros2/src/offboard_controllers/config/recording/offboard_position.txt",
            "ros2/src/offboard_controllers/config/recording/se3.txt",
            "ros2/src/px4_position_mode/config/recording_topics.txt",
            "ros2/src/px4_position_mode/config/recording/offboard_takeoff_handoff.txt",
        ]
        for filename in filenames:
            with self.subTest(file=filename):
                contents = (root / filename).read_text()
                self.assertNotIn("OUT_BATTERY_STATUS", contents)
                self.assertNotIn("OUT_MANUAL_CONTROL_SETPOINT", contents)


if __name__ == "__main__":
    unittest.main()
