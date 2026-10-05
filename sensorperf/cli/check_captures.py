"""
Command line: a quick-look check of a capture session before the long analyses.

    python3 -m sensorperf.cli.check_captures --session Characterization_20261005/ --out check.json
    python3 -m sensorperf.cli.check_captures --session Characterization_20261005/ --pilot 750

What it checks, per pose (all frames of one commanded pose)
    frames          number of capture files of the pose.
    valid fraction  fraction of the pixels that the registered target should
                    occupy (from the manifest's target pose and targets.json) that
                    were read, averaged over the frames.
    border          reads within --border-margin-px of the image border (information)
                    and, flagged, the number of feature outlines of the target that
                    cross that border band (the target is partly out of view).
    front plane     a robust plane fitted to the pixels that should see the front
                    surface: RMS residual, signed distance to the registered front
                    plane (positive = nearer the sensor) and the angle between the
                    normals.
    back plane      the same for the back plate, where the target has one.

``--pilot Z`` instead prints the pilot detection counts of Section 8, Step 1 (the
rule of Section 13, Step 2) on the first frame of each C pose at station Z: per
array and gap the detection fraction per diameter level, the pilot D_50, the
pilot D_0 and the post-site detection fraction. The values feed
``plan_stations --pilot-d50-mm / --pilot-d0-mm``.

Inputs: a session folder (sensor_config.json, targets.json, manifest.csv,
optionally parameters.json and registration.json; Section 9).
Output: a console table (and, with --out, a JSON report). Units: millimeters,
degrees, pixels.

Exit code: 0 when no pose is flagged, 1 when at least one is, 2 when the session
cannot be read (or, with --pilot, when no C poses match).
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

from sensorperf.acquisition.check import (
    EXIT_FLAGGED, EXIT_INPUT_ERROR, EXIT_OK, REPORT_JSON_INDENT, CheckParameters, _json_safe, check_session,
    format_pilot_table, pilot_detection_counts,
)
from sensorperf.io.manifest import SUBSERIES_JITTER
from sensorperf.io.session import Session


def build_parser() -> argparse.ArgumentParser:
    """The argument parser (every option has help text)."""
    d = CheckParameters()
    parser = argparse.ArgumentParser(
        description="Quick-look check of a capture session: valid fraction, border contact and the front and back "
                    "plane fits against the registered target, or (with --pilot) the D pilot detection counts.")
    parser.add_argument("--session", required=True, type=Path, metavar="DIR",
                        help="session folder with sensor_config.json, targets.json and manifest.csv")
    parser.add_argument("--out", type=Path, metavar="PATH", help="write a JSON report here")
    parser.add_argument("--min-valid-fraction", type=float, default=d.min_valid_fraction,
                        help="flag a pose in which fewer than this fraction of the expected target pixels were read")
    parser.add_argument("--border-margin-px", type=int, default=d.border_margin_px,
                        help="reads and feature outlines within this many pixels of the border count as border contact")
    parser.add_argument("--plane-residual-warn-mm", type=float, default=d.plane_residual_warn_mm,
                        help="flag a plane whose fit RMS exceeds this")
    parser.add_argument("--pose-residual-warn-mm", type=float, default=d.pose_residual_warn_mm,
                        help="flag a plane whose distance to the registered plane exceeds this")
    parser.add_argument("--normal-warn-deg", type=float, default=d.normal_warn_deg,
                        help="flag a plane whose normal differs from the registered one by more than this")
    parser.add_argument("--classification-margin-px", type=int, default=d.classification_margin_px,
                        help="erosion of the expected front and back masks before a plane is fitted")
    parser.add_argument("--min-plane-pixels", type=int, default=d.min_plane_pixels,
                        help="fewest pixels a plane fit is attempted with")
    parser.add_argument("--pilot", type=float, metavar="Z_MM", default=None,
                        help="print the D pilot detection counts for the C poses at this station (mm) instead of the check")
    parser.add_argument("--pilot-subseries", nargs="+", default=[SUBSERIES_JITTER], metavar="LABEL",
                        help="sub-series of the C poses used by --pilot (default: jitter)")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the check; returns the exit code."""
    args = build_parser().parse_args(argv)
    try:
        session = Session.load(args.session)
    except (OSError, ValueError, KeyError) as error:
        print(f"ERROR: cannot read the session {args.session}: {error}. The folder needs sensor_config.json, "
              "targets.json and manifest.csv (plan_stations and make_manifest write them).", file=sys.stderr)
        return EXIT_INPUT_ERROR
    if not session.records:
        print(f"ERROR: the manifest of {args.session} lists no captures.", file=sys.stderr)
        return EXIT_INPUT_ERROR
    if args.pilot is not None:
        results = pilot_detection_counts(session, session.params, session.geometry, args.pilot,
                                         tuple(args.pilot_subseries))
        print(format_pilot_table(results))
        if args.out is not None:
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.out.write_text(json.dumps(_json_safe({f"{t}@{g}": asdict(r) for (t, g), r in results.items()}),
                                           indent=REPORT_JSON_INDENT), encoding="utf-8")
        return EXIT_OK if results else EXIT_INPUT_ERROR
    params = CheckParameters(
        min_valid_fraction=args.min_valid_fraction, border_margin_px=args.border_margin_px,
        plane_residual_warn_mm=args.plane_residual_warn_mm, pose_residual_warn_mm=args.pose_residual_warn_mm,
        normal_warn_deg=args.normal_warn_deg, classification_margin_px=args.classification_margin_px,
        min_plane_pixels=args.min_plane_pixels)
    try:
        report = check_session(session, params)
    except (OSError, ValueError, KeyError) as error:
        print(f"ERROR: the check failed: {error}", file=sys.stderr)
        return EXIT_INPUT_ERROR
    print(report.format_table())
    for note in report.notes:
        print(f"NOTE: {note}")
    print(report.verdict())
    if args.out is not None:
        report.write_json(args.out)
    return EXIT_FLAGGED if report.flagged else EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
