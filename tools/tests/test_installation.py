"""Installer contracts; no network, apt, ROS installation, or source download."""

import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class InstallerProfiles(unittest.TestCase):
    def test_public_profiles_are_only_sitl_and_hardware(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "setup").mkdir()
            (root / "ros2").mkdir()
            shutil.copy2(ROOT / "setup/install.sh", root / "setup/install.sh")
            for name in (
                "install_ros2_jazzy.sh", "install_tools.sh", "fetch_sources.sh",
                "install_px4_dependencies.sh", "install_mavproxy.sh",
            ):
                script = root / "setup" / name
                script.write_text(
                    '#!/usr/bin/env bash\n'
                    f'echo "{name} $*" >> "$INSTALL_TEST_LOG"\n'
                )
                script.chmod(0o755)
            build = root / "ros2/build.sh"
            build.write_text(
                '#!/usr/bin/env bash\n'
                'echo "ros2/build.sh $*" >> "$INSTALL_TEST_LOG"\n'
            )
            build.chmod(0o755)
            logfile = root / "calls.txt"
            env = {**os.environ, "INSTALL_TEST_LOG": str(logfile)}

            cases = {
                "sitl": [
                    "install_ros2_jazzy.sh desktop", "install_tools.sh sitl",
                    "fetch_sources.sh sitl", "install_px4_dependencies.sh sitl",
                    "install_mavproxy.sh ", "ros2/build.sh sitl",
                ],
                "hardware": [
                    "install_ros2_jazzy.sh base", "install_tools.sh hardware",
                    "fetch_sources.sh hardware", "ros2/build.sh hardware",
                ],
            }
            for profile, expected in cases.items():
                with self.subTest(profile=profile):
                    logfile.unlink(missing_ok=True)
                    result = subprocess.run(
                        ["bash", str(root / "setup/install.sh"), profile],
                        env=env, capture_output=True, text=True,
                    )
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertEqual(logfile.read_text().splitlines(), expected)
                    self.assertTrue((root / "bags").is_dir())
                    self.assertTrue((root / "log").is_dir())

            for invalid_args in ([], ["common"], ["all"], ["other"], ["sitl", "hardware"]):
                with self.subTest(invalid_args=invalid_args):
                    logfile.unlink(missing_ok=True)
                    result = subprocess.run(
                        ["bash", str(root / "setup/install.sh"), *invalid_args],
                        env=env, capture_output=True, text=True,
                    )
                    self.assertEqual(result.returncode, 2)
                    self.assertFalse(logfile.exists())

    def test_manifests_are_independent_and_pinned_identically(self):
        source_dir = ROOT / "config/sources"
        self.assertFalse((source_dir / "common.repos").exists())
        def entries(profile):
            data = (source_dir / f"{profile}.repos").read_text()
            self.assertTrue(data.startswith("repositories:\n"))
            sections = re.split(r"(?=^  [^ \n]+:\s*$)", data, flags=re.M)
            result = {}
            for section in sections:
                match = re.match(r"  ([^ \n]+):\s*$", section, flags=re.M)
                if not match:
                    continue
                revision = re.search(r"^    version: (\S+)$", section, flags=re.M)
                self.assertIsNotNone(revision, match.group(1))
                result[match.group(1)] = revision.group(1)
            self.assertEqual(len(result), 4 if profile == "sitl" else 3)
            return result
        sitl = entries("sitl")
        hardware = entries("hardware")
        shared = {"ros2/src/px4_msgs", "ros2/src/Micro-XRCE-DDS-Agent"}
        self.assertEqual(set(sitl) & set(hardware), shared)
        for package in shared:
            self.assertEqual(sitl[package], hardware[package])
        self.assertIn("px4/PX4-Autopilot", sitl)
        self.assertIn("tools/MAVProxy", sitl)
        self.assertIn("ros2/src/vicon_receiver", hardware)

    def test_no_removed_installer_paths_remain(self):
        self.assertFalse((ROOT / "setup/install_common_tools.sh").exists())
        for source in (
            "setup/install.sh", "setup/fetch_sources.sh", "setup/install_tools.sh",
            "docker/Dockerfile", ".github/workflows/pr-tests.yml",
        ):
            text = (ROOT / source).read_text()
            self.assertNotIn("install_common_tools.sh", text, source)
            self.assertNotIn("common.repos", text, source)
            self.assertNotIn("fetch_sources.sh common", text, source)
        docker = (ROOT / "docker/Dockerfile").read_text()
        self.assertIn("RUN ./setup/fetch_sources.sh sitl", docker)
        self.assertIn("setup/install_tools.sh sitl", docker)
        ci = (ROOT / ".github/workflows/pr-tests.yml").read_text()
        self.assertIn("./setup/fetch_sources.sh hardware", ci)
        self.assertIn("./ros2/build.sh hardware", ci)
        self.assertIn("./ros2/test.sh hardware", ci)
        build = (ROOT / "ros2/build.sh").read_text()
        self.assertIn("colcon build --packages-skip px4_mocap_bridge", build)
        self.assertIn("colcon build --packages-skip px4_position_mode", build)
        test = (ROOT / "ros2/test.sh").read_text()
        self.assertIn("TOOLKIT_PACKAGES+=(px4_position_mode)", test)
        self.assertIn("TOOLKIT_PACKAGES+=(px4_mocap_bridge)", test)
        self.assertIn('if [[ "${PX4_INSTALL_PROFILE}" == "sitl" ]]; then', test)
        self.assertIn('analysis_status=0', test)
        self.assertIn('Run analysis regressions', ci)
        self.assertIn("-p test_installation.py -v", ci)

    def test_shell_syntax(self):
        for filename in ("install.sh", "install_tools.sh", "fetch_sources.sh"):
            result = subprocess.run(
                ["bash", "-n", str(ROOT / "setup" / filename)],
                capture_output=True, text=True,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
        for filename in ("ros2/build.sh", "ros2/test.sh"):
            result = subprocess.run(
                ["bash", "-n", str(ROOT / filename)],
                capture_output=True, text=True,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            for arguments in ([], ["common"], ["all"], ["invalid"]):
                check = subprocess.run(
                    ["bash", str(ROOT / filename), *arguments],
                    capture_output=True, text=True,
                )
                self.assertEqual(check.returncode, 2, check.stderr)


if __name__ == "__main__":
    unittest.main()
