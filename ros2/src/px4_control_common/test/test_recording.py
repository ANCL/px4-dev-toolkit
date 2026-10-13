"""Verify ROS recording-topic resolution and launch flag semantics."""

from pathlib import Path
import tempfile
import unittest

from px4_control_common.recording import parse_recording_flag, read_topics


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


if __name__ == "__main__":
    unittest.main()
