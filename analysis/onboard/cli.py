"""ULog dispatch to shared PX4 pipeline and native onboard SE(3) profiles."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from analysis.onboard.general import (  # noqa: E402
    compare as compare_general, detect, summary,
)
from analysis.onboard.report import analyze as analyze_se3, compare as compare_se3  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    cmds = parser.add_subparsers(dest="command", required=True)
    single = cmds.add_parser("analyze")
    single.add_argument("source", type=Path)
    single.add_argument("--run-id", type=int)
    single.add_argument("--profile", choices=("auto", "px4_ulog", "onboard_se3"), default="auto")
    batch = cmds.add_parser("compare")
    batch.add_argument("sources", type=Path, nargs="+")
    batch.add_argument("--output", type=Path, required=True)
    batch.add_argument("--profile", choices=("auto", "px4_ulog", "onboard_se3"), default="auto")
    args = parser.parse_args()
    try:
        if args.command == "analyze":
            profile = detect(args.source) if args.profile == "auto" else args.profile
            if profile == "onboard_se3":
                # Use the native execution interval, not unrelated PX4
                # takeoff/landing phases from this ULog.
                print(analyze_se3(args.source, args.run_id), end="")
            else:
                if args.run_id is not None:
                    raise ValueError("--run-id applies only to onboard_se3")
                print(summary(args.source), end="")
        else:
            if len(args.sources) < 2:
                raise ValueError("comparison requires at least two recordings")
            profiles = {detect(src) for src in args.sources} if args.profile == "auto" else {args.profile}
            if len(profiles) != 1:
                raise ValueError(f"ULog comparison requires matching profiles: {sorted(profiles)}")
            fn = compare_se3 if profiles == {"onboard_se3"} else compare_general
            print(fn(args.sources, args.output), end="")
    except (OSError, RuntimeError, ValueError, KeyError) as err:
        print(f"ERROR: {err}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
