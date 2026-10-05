"""
Command line: build the capture manifest (procedure document, Section 9) from
the robot's pose log, the capture files and the plan.

    python3 -m sensorperf.cli.make_manifest --pose-log pose_log.csv --captures Characterization_20261005/ \
        --plan plan/poses.csv --registration registration.json --sensor-config-id cfgA \
        --out Characterization_20261005/manifest.csv

What it computes
----------------
One manifest row per captured frame (``sensorperf.io.manifest.MANIFEST_COLUMNS``):
the capture file is matched to the plan by its Section 9 name
(``<proc>_<target>_G<gap>_Z<zzzz>_F<field>_P<pose>_f<frame>.mc``), the plan
supplies the sub-series, seed, lateral offsets, tilt, step, visit and level, the
pose log supplies the read-back flange pose (flange -> robot base) and the optional
readings, and the registration turns the read-back pose into the ground-truth
target pose in the camera frame (target -> camera). The capture files are not
opened: only their names are used.

Inputs
    --pose-log PATH   CSV, one row per frame (a ``file`` column) or one row per
                      pose (the Section 9 identity columns and ``frames``). Robot
                      pose columns x_mm, y_mm, z_mm, rotation_type, r1..r9;
                      optional timestamp, sensor_temp_c, air_temp_c,
                      ambient_ir. rotation_type is one of none, quaternion_wxyz,
                      quaternion_xyzw, euler_zyx_deg, euler_xyz_deg, fixed_xyz_deg,
                      rotvec_deg, matrix (the table of sensorperf.acquisition.pose_log,
                      taken from the calibration repository's make_manifest).
    --captures DIR    the session root (or any folder); sub-folders are searched
                      for .mc files.
    --plan PATH       poses.csv written by plan_stations.
    --registration PATH   registration.json (camera_to_base, target_to_flange).
    --sensor-config-id TEXT   identifier written to every row (the sensor
                      configuration of sensor_config.json).

Output: --out (default MANIFEST.csv in the captures folder).

Problems are listed by name: capture files with no log row or plan row, planned
poses without captures, logged frames without files, poses read back far from
the plan. Warnings are listed but do not fail unless --strict.

Exit code: 0 on success, 2 when the log, plan or captures must be fixed (every
problem is listed and no manifest is written).
"""
from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path

from sensorperf.acquisition.pose_log import ROTATION_VALUE_COUNTS, build_manifest_with_report
from sensorperf.geometry.registration import Registration
from sensorperf.io.manifest import MANIFEST_FILE_NAME, write_manifest_csv

EXIT_OK = 0
EXIT_INPUT_ERROR = 2
"""Exit codes: success, an input the technician must fix."""


def build_parser() -> argparse.ArgumentParser:
    """The argument parser (every option has help text)."""
    parser = argparse.ArgumentParser(
        description="Build the capture manifest (manifest.csv) from a robot pose log, the capture files and the plan "
                    "(poses.csv of plan_stations), through the registration.")
    parser.add_argument("--pose-log", required=True, type=Path, metavar="PATH",
                        help="robot pose log CSV: one row per frame (file, x_mm, y_mm, z_mm, rotation_type, r1..r9) or "
                             "one row per pose (procedure, target_id, gap_mm, station_z_mm, field, pose_index, frames, "
                             "x_mm, ...); rotation_type: " + ", ".join(ROTATION_VALUE_COUNTS))
    parser.add_argument("--captures", required=True, type=Path, metavar="DIR",
                        help="session root or folder holding the .mc files (sub-folders are searched)")
    parser.add_argument("--plan", required=True, type=Path, metavar="PATH", help="poses.csv written by plan_stations")
    parser.add_argument("--registration", required=True, type=Path, metavar="PATH",
                        help="registration.json (camera_to_base and target_to_flange)")
    parser.add_argument("--sensor-config-id", default="", metavar="TEXT",
                        help="identifier of the sensor configuration, written to every row")
    parser.add_argument("--out", type=Path, metavar="PATH",
                        help=f"manifest file to write (default: {MANIFEST_FILE_NAME} in the --captures folder)")
    parser.add_argument("--strict", action="store_true",
                        help="treat warnings (unmatched files or plan rows, frame count mismatches, poses read back "
                             "far from the plan) as errors")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the manifest builder; returns the exit code."""
    args = build_parser().parse_args(argv)
    try:
        registration = Registration.load(args.registration)
    except (OSError, ValueError, KeyError) as error:
        print(f"ERROR: cannot read the registration {args.registration}: {error}", file=sys.stderr)
        return EXIT_INPUT_ERROR
    records, messages = build_manifest_with_report(args.pose_log, args.captures, args.plan, registration,
                                                   args.sensor_config_id, strict=args.strict)
    for warning in messages.warnings:
        print(f"WARNING: {warning}", file=sys.stderr)
    for error in messages.errors:
        print(f"ERROR: {error}", file=sys.stderr)
    if messages.errors:
        print(f"Manifest not written: {len(messages.errors)} error(s), {len(messages.warnings)} warning(s). "
              "Fix the items above and run again.", file=sys.stderr)
        return EXIT_INPUT_ERROR
    if not records:
        print("ERROR: no frame could be matched, so there is nothing to write; check that the file names follow the "
              "Section 9 rule and that the pose log names them.", file=sys.stderr)
        return EXIT_INPUT_ERROR
    out = args.out if args.out is not None else args.captures / MANIFEST_FILE_NAME
    write_manifest_csv(out, records)
    per_series = Counter(r.procedure for r in records)
    poses = len({r.pose_key() for r in records})
    print(f"Wrote {out}: {len(records)} frames in {poses} poses ("
          + ", ".join(f"{k}: {v}" for k, v in sorted(per_series.items())) + f"); {len(messages.warnings)} warning(s).")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
