"""Verify ROS recording-topic resolution and launch flag semantics."""

from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from px4_control_common.recording import (
    controller_configuration_paths, parse_recording_flag, read_topics,
)


class RecordingTests(unittest.TestCase):
    def test_topic_catalog_recording_contract(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            catalog = folder / "px4_topics.def"
            topics = folder / "topics.txt"
            catalog.write_text('PX4_TOPIC(OUT_TEST, "/fmu/out/test")\n')
            topics.write_text('OUT_TEST\n')
            self.assertEqual(read_topics(topics, catalog), ["/fmu/out/test"])

            topics.write_text('/se3/diagnostics/force_command_ned\n')
            with self.assertRaises(RuntimeError):
                read_topics(topics, catalog)
            self.assertEqual(read_topics(topics, catalog, allow_direct=True),
                             ["/se3/diagnostics/force_command_ned"])

    def test_recording_boolean_semantics(self):
        for value in ("1", "true", "yes", "on", " TRUE ", "On"):
            with self.subTest(value=value):
                self.assertTrue(parse_recording_flag(value))
        for value in ("0", "false", "no", "off", "", "invalid"):
            with self.subTest(value=value):
                self.assertFalse(parse_recording_flag(value))

    def test_checkout_yaml_is_authoritative(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "config/vehicles").mkdir(parents=True)
            (root / "config/px4_topics.def").touch()
            source = root / "ros2/src/offboard_controllers/config"
            (source / "se3").mkdir(parents=True)
            (source / "trajectory").mkdir(parents=True)
            (source / "se3/controller.yaml").touch()
            (source / "trajectory/trajectories.yaml").touch()
            launch = root / "ros2/install/offboard_controllers/share/launch/se3.launch.py"
            launch.parent.mkdir(parents=True)
            launch.touch()

            actual = controller_configuration_paths(str(launch))
            self.assertEqual(actual, (source, root / "config/vehicles", "repository source"))

    def test_standalone_install_uses_explicitly_reported_package_yaml(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            packaged = root / "share/offboard_controllers/config"
            (packaged / "se3").mkdir(parents=True)
            (packaged / "trajectory").mkdir(parents=True)
            (packaged / "vehicles").mkdir()
            (packaged / "se3/controller.yaml").touch()
            (packaged / "trajectory/trajectories.yaml").touch()
            with patch("px4_control_common.recording.get_package_share_directory",
                       return_value=str(packaged.parent)):
                actual = controller_configuration_paths(str(root / "isolated/launch.py"))
            self.assertEqual(actual, (packaged, packaged / "vehicles",
                                      "packaged (no toolkit checkout found)"))


if __name__ == "__main__":
    unittest.main()
