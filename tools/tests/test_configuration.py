"""Checks that checked-in configuration resolves through the tooling.

Run from the repository root with the ROS workspace built and sourced
(px4_control_common is imported from the install space, so rebuild after
editing it):

    source ros2/runtime_env.sh
    python3 -m pytest tools/tests
"""
from __future__ import annotations

from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
# Loading extensionless scripts would otherwise leave tools/__pycache__ files.
sys.dont_write_bytecode = True

from px4_control_common.recording import (  # noqa: E402
    read_topics, source_revisions, trajectory_configuration,
)
from sequence.config import load_sequence  # noqa: E402


def load_script(name: str):
    """Import an extensionless tool script as a module."""
    loader = SourceFileLoader(name, str(ROOT / "tools" / name))
    module = module_from_spec(spec_from_loader(name, loader))
    loader.exec_module(module)
    return module


SEQUENCES = sorted((ROOT / "config/sequences").rglob("*.yaml"))
PROFILES = sorted(p.stem for p in (ROOT / "config/recording").glob("*.txt") if p.stem != "common")


@pytest.mark.parametrize("path", SEQUENCES, ids=lambda p: p.stem)
def test_sequence_steps_reference_existing_trajectories_and_vehicles(path):
    sequence = load_sequence(path)
    for step in sequence["steps"]:
        arguments = step["arguments"]
        if "trajectory" in arguments:
            resolved = trajectory_configuration(ROOT / "config/trajectories.yaml", arguments["trajectory"])
            assert resolved["segments"], step["name"]
        if "vehicle" in arguments:
            assert (ROOT / "config/vehicles" / f"{arguments['vehicle']}.yaml").is_file(), step["name"]


@pytest.mark.parametrize("profile", PROFILES)
def test_recording_profiles_resolve_through_topic_catalog(profile):
    # Standalone recorder and launch-owned recorder must accept the same files.
    standalone = load_script("px4_record").topics_for(profile)
    catalog = ROOT / "config/px4_topics.def"
    profile_file = ROOT / "config/recording" / f"{profile}.txt"
    launch = read_topics(ROOT / "config/recording/common.txt", catalog)
    # A comment-only profile adds nothing (read_topics rejects empty files).
    if any(line.strip() and not line.lstrip().startswith("#")
           for line in profile_file.read_text().splitlines()):
        launch += read_topics(profile_file, catalog, allow_direct=True)
    assert standalone == list(dict.fromkeys(launch))
    assert all(topic.startswith("/") for topic in standalone)


def test_source_revisions_report_commit_and_modification(tmp_path):
    def git(*args):
        return subprocess.run(["git", "-C", str(tmp_path), *args], check=True,
                              capture_output=True, text=True).stdout.strip()

    git("init", "-q", "-b", "main")
    (tmp_path / "file.txt").write_text("a\n")
    git("add", "file.txt")
    git("-c", "user.name=test", "-c", "user.email=test@example.invalid", "commit", "-q", "-m", "init")

    clean = source_revisions(tmp_path)
    assert clean == {"toolkit": {"commit": git("rev-parse", "HEAD"), "branch": "main", "modified": False}}

    (tmp_path / "file.txt").write_text("b\n")
    assert source_revisions(tmp_path)["toolkit"]["modified"] is True

    # Untracked files do not mark a checkout modified.
    (tmp_path / "file.txt").write_text("a\n")
    (tmp_path / "scratch.txt").write_text("x\n")
    assert source_revisions(tmp_path)["toolkit"]["modified"] is False


def test_source_revisions_outside_git_is_null(tmp_path):
    assert source_revisions(tmp_path) == {"toolkit": None}
