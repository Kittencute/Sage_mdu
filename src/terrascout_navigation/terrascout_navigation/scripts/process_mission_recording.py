#!/usr/bin/env python3
"""Generate the FAST-LIO comparison and mission report for the newest bag."""

import argparse
import json
from datetime import datetime, timezone
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
from zoneinfo import ZoneInfo


WORKSPACE_ROOT = Path(__file__).resolve().parents[4]
RECORDINGS_ROOT = WORKSPACE_ROOT / ".sage" / "run" / "1" / "recordings" / "terrascout1"
ARCHIVED_RECORDINGS_ROOT = WORKSPACE_ROOT / "runs"
RESULTS_ROOT = WORKSPACE_ROOT / "src" / "terrascout_mission_tester" / "mission_results"
SCRIPT_DIR = Path(__file__).resolve().parent
STOCKHOLM = ZoneInfo("Europe/Stockholm")


def bag_start_datetime(bag: Path) -> datetime:
    match = re.search(r"rosbag2_(\d{4}_\d{2}_\d{2}-\d{2}_\d{2}_\d{2})", bag.name)
    if not match:
        raise ValueError(f"Cannot parse recording timestamp from bag directory: {bag}")
    return datetime.strptime(match.group(1), "%Y_%m_%d-%H_%M_%S").replace(tzinfo=timezone.utc)


def select_results_dir(bag: Path, requested: Path | None = None) -> Path:
    if requested is not None:
        return requested if requested.is_absolute() else WORKSPACE_ROOT / requested

    bag_start = bag_start_datetime(bag)
    # Prefer a bundle already tagged by its report with this exact recording.
    for report_path in RESULTS_ROOT.glob("mission_*/mission_report.json"):
        try:
            report = json.loads(report_path.read_text(encoding="utf-8"))
        except (OSError, ValueError, json.JSONDecodeError):
            continue
        recording = Path(report.get("recording", ""))
        if recording.name == bag.name:
            return report_path.parent

    duration_match = re.search(
        r"^\s+duration:\s*\n\s+nanoseconds:\s*(\d+)",
        (bag / "metadata.yaml").read_text(encoding="utf-8"),
        re.MULTILINE,
    )
    bag_end = bag_start
    if duration_match:
        from datetime import timedelta

        bag_end += timedelta(microseconds=int(duration_match.group(1)) / 1000)
    candidates = []
    for metadata_path in RESULTS_ROOT.glob("mission_*/metadata.json"):
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            created = datetime.fromisoformat(metadata["bundle_created_at"])
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
            continue
        created_utc = created.astimezone(timezone.utc)
        if bag_start <= created_utc <= bag_end:
            candidates.append((created, metadata_path.parent))

    if candidates:
        return max(candidates, key=lambda item: item[0])[1]

    local_stamp = bag_start.astimezone(STOCKHOLM).strftime("%Y%m%d_%H%M%S")
    return RESULTS_ROOT / f"mission_{local_stamp}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, help="bag directory; defaults to newest completed recording")
    parser.add_argument("--out-dir", type=Path, help="explicit results bundle directory; by default match the latest tester bundle created before the recording")
    parser.add_argument("--state-topic", default="/terrascout1/mission_state")
    parser.add_argument("--mission-id", type=int, help="select a mission ID in the recording")
    parser.add_argument("--mission-instance", type=int, help="select a run instance for the mission ID")
    parser.add_argument("--show-trajectory", action="store_true")
    parser.add_argument(
        "--headless", action="store_true",
        help="skip interactive plotting and run the analyses without opening a GUI",
    )
    args = parser.parse_args()

    bag = args.bag
    if bag is None:
        metadata_files = list(RECORDINGS_ROOT.glob("**/metadata.yaml"))
        metadata_files.extend(
            ARCHIVED_RECORDINGS_ROOT.glob("*/recordings/terrascout1/**/metadata.yaml")
        )
        if not metadata_files:
            parser.error(
                "No completed bag found. Expected metadata.yaml under "
                f"{RECORDINGS_ROOT} or archived runs in {ARCHIVED_RECORDINGS_ROOT}"
            )
        newest_metadata = max(metadata_files, key=lambda path: path.stat().st_mtime)
        bag = newest_metadata.parent
    bag = bag.resolve()
    if not (bag / "metadata.yaml").is_file():
        parser.error(f"Bag directory has no metadata.yaml: {bag}")

    out_dir = select_results_dir(bag, args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Bag: {bag}")
    print(f"Results bundle: {out_dir}")
    comparison = [sys.executable, str(SCRIPT_DIR / "compare_fast_lio_gt.py"), str(bag), "--out-dir", str(out_dir)]
    if args.show_trajectory and not args.headless:
        comparison.append("--show-trajectory")
    comparison_result = subprocess.run(
        comparison, cwd=WORKSPACE_ROOT, check=False, capture_output=True, text=True
    )
    print(comparison_result.stdout, end="")
    if comparison_result.stderr:
        print(comparison_result.stderr, file=sys.stderr, end="")

    try:
        bag_in_container = Path("/workspace") / bag.relative_to(WORKSPACE_ROOT)
        out_dir_in_container = Path("/workspace") / out_dir.relative_to(WORKSPACE_ROOT)
    except ValueError:
        parser.error("--bag and --out-dir must be inside the workspace for containerized report generation")
    report = [
        "python3", "/workspace/src/terrascout_navigation/terrascout_navigation/scripts/report_mission_bag.py", str(bag_in_container),
        "--out-dir", str(out_dir_in_container), "--state-topic", args.state_topic,
    ]
    if args.mission_id is not None:
        report.extend(["--mission-id", str(args.mission_id)])
    if args.mission_instance is not None:
        report.extend(["--mission-instance", str(args.mission_instance)])
    container_command = [
        "docker", "run", "--rm", "--user", f"{os.getuid()}:{os.getgid()}",
        "--workdir", "/workspace", "--volume", f"{WORKSPACE_ROOT}:/workspace",
        "sage-sim:latest", "bash", "-lc",
        "source /opt/ros/jazzy/setup.bash && source install/setup.bash && " + shlex.join(report),
    ]
    report_result = subprocess.run(
        container_command, cwd=WORKSPACE_ROOT, check=False, capture_output=True, text=True
    )
    print(report_result.stdout, end="")
    if report_result.stderr:
        print(report_result.stderr, file=sys.stderr, end="")

    report_path = out_dir / "mission_report.md"
    processing_details = [
        "",
        "## Processing and FAST-LIO comparison details",
        "",
        f"- Bag: `{bag}`",
        f"- Results bundle: `{out_dir}`",
        f"- Comparison command: `{shlex.join(comparison)}`",
        f"- Comparison exit code: `{comparison_result.returncode}`",
        f"- Report command: `{shlex.join(report)}`",
        f"- Report/setup exit code: `{report_result.returncode}`",
        "",
        "### Analysis output",
        "",
        "Complete standard output and diagnostics from FAST-LIO/ground-truth comparison and mission-report processing:",
        "",
        "#### FAST-LIO / ground-truth comparison — stdout",
        "",
        "```text",
        comparison_result.stdout.strip() or "(no output)",
        "```",
        "",
        "#### FAST-LIO / ground-truth comparison — stderr",
        "",
        "```text",
        comparison_result.stderr.strip() or "(no diagnostics)",
        "```",
        "",
        "#### Mission report generation — stdout",
        "",
        "```text",
        report_result.stdout.strip() or "(no output)",
        "```",
        "",
        "#### Mission report generation — stderr / setup diagnostics",
        "",
        "```text",
        report_result.stderr.strip() or "(no diagnostics)",
        "```",
        "",
    ]
    report_path.parent.mkdir(parents=True, exist_ok=True)
    base_report = report_path.read_text(encoding="utf-8") if report_path.exists() else ""
    marker = "## Processing and FAST-LIO comparison details"
    if marker in base_report:
        base_report = base_report.split(marker, maxsplit=1)[0].rstrip() + "\n"
    with report_path.open("w", encoding="utf-8") as stream:
        stream.write(base_report.rstrip() + "\n\n" + "\n".join(processing_details).lstrip())
    if comparison_result.returncode != 0 or report_result.returncode != 0:
        print(
            "Processing completed with errors; see mission_report.md for captured diagnostics.",
            file=sys.stderr,
        )
        return 1
    print(f"Completed both analyses in {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
