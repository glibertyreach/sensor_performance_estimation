"""
Command line: a quick-look check of a capture session before the long analyses.

    python3 -m sensorperf.cli.check_captures --session Characterization_20261005/ --out check.json
    python3 -m sensorperf.cli.check_captures --session Characterization_20261005/ --pilot 800
    python3 -m sensorperf.cli.check_captures --mount-check Characterization_20261005/mount_check/ \\
        --registration Characterization_20261005/registration.json --target T3a

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

The limits are for gross errors and loose on purpose. The depth noise of a stereo sensor grows with the square of
the distance, so the residual and distance limits are multiplied by (Z / --limit-reference-z-mm) to the power
--limit-depth-exponent for a target farther than the reference (never reduced below the stated values). The
tilt limit of a plane is at least --normal-sigma-factor standard errors of its fitted tilt; where that exceeds
--normal-limit-cap-deg the tilt is not testable from the frames of the pose (noted, not flagged). The limits applied
are written to the JSON report.

``--pilot Z`` instead runs the post check of Section 8, Step 1 (formerly the D pilot), which keeps only the post
check (the rule of Section 13, Step 2 on the first frame of each C pose at station Z, normally
the reference station): per disk plate and gap the threshold and the fraction of post-only
sites that are detected (a bare post must not be). The detection levels are no longer chosen
from a pilot D_50.

``--mount-check FRAMES_OR_FOLDER`` instead runs the mount check of Step 4.8 (``acquisition.mount_check``) on the depth frames
of the capture made at the reference station after a target was mounted (no session manifest is needed): it fits the
mounted target's front plane and compares its Z and tilt with the registered pose (``registration_residual_accept_mm``
and ``mount_tilt_tolerance_deg``), and locates one feature of the target and compares its H and V with the as-built
datum offsets (``frame_check_px``). It prints PASS or FAIL for each check with the measured value and the tolerance and
writes mount_check.json next to the frames. ``--registration`` and ``--target`` are required with it. The target
definitions (targets.json), the as-built record (targets_asbuilt.csv) and parameters.json are taken from the folder of
registration.json unless given. The registered pose is the commanded one (the target centered and fronto-parallel at
``mount_check_depth_mm``) unless the read-back flange pose of the capture is given with ``--flange-pose``.

Inputs: a session folder (sensor_config.json, targets.json, manifest.csv,
optionally parameters.json and registration.json; Section 9).
Output: a console table (and, with --out, a JSON report). Units: millimeters,
degrees, pixels.

Exit code: 0 when no pose is flagged, 1 when at least one is, 2 when the session
cannot be read (or, with --pilot, when no C poses match). With --mount-check: 0 when every check
passes, 1 when one fails, 2 when the frames, the registration or the target definition cannot be read.
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

from sensorperf.acquisition.check import (
    EXIT_FLAGGED, EXIT_INPUT_ERROR, EXIT_OK, REPORT_JSON_INDENT, UNREADABLE_ERRORS, CheckParameters, check_session,
    format_post_check_table, json_safe, pilot_post_check,
)
from sensorperf.acquisition.mount_check import (
    MountCheckInputError, MountTolerances, check_mount, frame_paths, registered_pose, report_location,
)
from sensorperf.geometry.registration import Registration
from sensorperf.geometry.targets import ASBUILT_FILE_NAME, TARGETS_FILE_NAME, TargetSet, load_targets_asbuilt
from sensorperf.io.manifest import ROBOT_POSE_COLUMNS, SUBSERIES_JITTER, six_to_pose
from sensorperf.io.session import PARAMETERS_FILE_NAME, Session
from sensorperf.parameters import CharacterizationParameters


def build_parser() -> argparse.ArgumentParser:
    """The argument parser (every option has help text)."""
    d = CheckParameters()
    parser = argparse.ArgumentParser(
        description="Quick-look check of a capture session: valid fraction, border contact and the front and back "
                    "plane fits against the registered target; or (with --pilot) the post check of Section 8, Step 1; "
                    "or (with --mount-check) the mount check of Step 4.8 of a freshly mounted target.")
    parser.add_argument("--session", type=Path, metavar="DIR",
                        help="session folder with sensor_config.json, targets.json and manifest.csv (required unless "
                             "--mount-check is given)")
    parser.add_argument("--out", type=Path, metavar="PATH",
                        help="write a JSON report here (with --mount-check: instead of mount_check.json next to the frames)")
    parser.add_argument("--min-valid-fraction", type=float, default=d.min_valid_fraction,
                        help="flag a pose in which fewer than this fraction of the expected target pixels were read")
    parser.add_argument("--border-margin-px", type=int, default=d.border_margin_px,
                        help="reads and feature outlines within this many pixels of the border count as border contact")
    parser.add_argument("--plane-residual-warn-mm", type=float, default=d.plane_residual_warn_mm,
                        help="flag a plane whose fit RMS exceeds this (mm) at the reference distance; farther away the "
                             "limit grows with the depth noise (see --limit-depth-exponent)")
    parser.add_argument("--pose-residual-warn-mm", type=float, default=d.pose_residual_warn_mm,
                        help="flag a plane whose distance to the registered plane exceeds this (mm) at the reference "
                             "distance; farther away the limit grows with the depth noise (see --limit-depth-exponent)")
    parser.add_argument("--normal-warn-deg", type=float, default=d.normal_warn_deg,
                        help="flag a plane whose normal differs from the registered one by more than this (degrees), "
                             "or by more than --normal-sigma-factor standard errors of the fitted tilt when that is larger")
    parser.add_argument("--limit-reference-z-mm", type=float, default=None, metavar="MM",
                        help="target distance (mm) up to which the residual and distance limits hold as stated; beyond "
                             "it they grow with depth (default: Z_REFERENCE_MM of the session's parameters.json, "
                             f"{d.limit_reference_z_mm:g})")
    parser.add_argument("--limit-depth-exponent", type=float, default=d.limit_depth_exponent,
                        help="the residual and distance limits are multiplied by (Z / reference distance) to this power "
                             "beyond the reference distance, never below 1; 2 is the depth-noise law of a stereo sensor, "
                             "0 keeps the limits fixed")
    parser.add_argument("--normal-sigma-factor", type=float, default=d.normal_sigma_factor,
                        help="a plane's tilt limit is at least this many standard errors of its fitted tilt, so that a "
                             "plane fitted from few noisy pixels is not judged against an angle it cannot resolve")
    parser.add_argument("--normal-limit-cap-deg", type=float, default=d.normal_limit_cap_deg,
                        help="where a plane's tilt limit would exceed this (degrees) its tilt cannot be tested from the "
                             "frames of that pose: no tilt flag is raised and the pose line says 'tilt not testable'")
    parser.add_argument("--classification-margin-px", type=int, default=d.classification_margin_px,
                        help="erosion of the expected front and back masks before a plane is fitted")
    parser.add_argument("--min-plane-pixels", type=int, default=d.min_plane_pixels,
                        help="fewest pixels a plane fit is attempted with")
    parser.add_argument("--pilot", type=float, metavar="Z_MM", default=None,
                        help="print the post check (Section 8, Step 1) for the C poses at this station (mm, normally Z_REFERENCE_MM) instead of the check")
    parser.add_argument("--pilot-subseries", nargs="+", default=[SUBSERIES_JITTER], metavar="LABEL",
                        help="sub-series of the C poses used by --pilot (default: jitter)")
    mount = parser.add_argument_group(
        "mount check (Section 4, Step 4.8)",
        "python3 -m sensorperf.cli.check_captures --mount-check FRAMES_OR_FOLDER --registration registration.json "
        "--target ID")
    d_params = CharacterizationParameters()
    mount.add_argument("--mount-check", nargs="+", type=Path, metavar="FRAMES_OR_FOLDER", default=None,
                       help="check a freshly mounted target from the depth frames (.mc files, or the folder that holds "
                            "them) captured at the reference station: fit its front plane and compare Z and tilt with "
                            "the registered pose, locate a feature and compare its H and V with the as-built datum "
                            "offsets; prints PASS or FAIL per check and writes mount_check.json next to the frames")
    mount.add_argument("--registration", type=Path, metavar="PATH",
                       help="registration.json (required with --mount-check); targets.json, targets_asbuilt.csv and "
                            "parameters.json are looked for in its folder")
    mount.add_argument("--target", metavar="ID",
                       help="id of the mounted target, e.g. T2, T3a, T3b, T4, T5 (required with --mount-check)")
    mount.add_argument("--gap-mm", type=float, default=None, metavar="MM",
                       help="gap of the mounted target when it differs from the one in targets.json")
    mount.add_argument("--flange-pose", type=float, nargs=len(ROBOT_POSE_COLUMNS), default=None, metavar="V",
                       help="read-back flange pose of the capture (" + " ".join(ROBOT_POSE_COLUMNS)
                            + "; mm and a rotation vector in degrees, as in the manifest); without it the registered "
                              "pose is the commanded one, the target centered and fronto-parallel at MOUNT_CHECK_DEPTH_MM")
    mount.add_argument("--targets", type=Path, metavar="PATH", help=f"target definitions (default: {TARGETS_FILE_NAME} "
                                                                    "next to registration.json)")
    mount.add_argument("--asbuilt", type=Path, metavar="PATH",
                       help=f"as-built record (default: {ASBUILT_FILE_NAME} next to registration.json, if present)")
    mount.add_argument("--parameters", type=Path, metavar="PATH",
                       help=f"parameter overrides (default: {PARAMETERS_FILE_NAME} next to registration.json, if present)")
    mount.add_argument("--z-tolerance-mm", type=float, default=None, metavar="MM",
                       help="largest Z difference from the registered pose (default REGISTRATION_RESIDUAL_ACCEPT_MM, "
                            f"{d_params.registration_residual_accept_mm:g})")
    mount.add_argument("--tilt-tolerance-deg", type=float, default=None, metavar="DEG",
                       help="largest tilt difference from the registered pose (default MOUNT_TILT_TOLERANCE_DEG, "
                            f"{d_params.mount_tilt_tolerance_deg:g})")
    mount.add_argument("--frame-check-px", type=float, default=None, metavar="PX",
                       help=f"largest H or V difference of the located feature (default FRAME_CHECK_PX, {d_params.frame_check_px:g})")
    return parser


def run_mount_check(args: argparse.Namespace) -> int:
    """The mount check of Step 4.8 (``--mount-check``); returns the exit code."""
    try:
        paths = frame_paths(args.mount_check)
        folder = args.registration.parent
        try:
            registration = Registration.load(args.registration)
        except (OSError, ValueError, KeyError) as error:
            raise MountCheckInputError(f"cannot read the registration {args.registration}: {error}") from None
        parameters_path = args.parameters or folder / PARAMETERS_FILE_NAME
        if args.parameters is not None or parameters_path.exists():
            params = CharacterizationParameters.from_json(parameters_path)
        else:
            params = CharacterizationParameters()
        targets_path = args.targets or folder / TARGETS_FILE_NAME
        try:
            targets = TargetSet.load(targets_path)
        except (OSError, ValueError, KeyError) as error:
            raise MountCheckInputError(f"cannot read the target definitions {targets_path}: {error}; plan_stations "
                                       f"writes {TARGETS_FILE_NAME}, or give --targets") from None
        asbuilt_path = args.asbuilt or folder / ASBUILT_FILE_NAME
        notes = []
        if args.asbuilt is not None or asbuilt_path.exists():
            try:
                targets.apply_asbuilt(load_targets_asbuilt(asbuilt_path))
            except (OSError, ValueError, KeyError) as error:
                raise MountCheckInputError(f"cannot read the as-built record {asbuilt_path}: {error}") from None
        else:
            notes.append(f"no {ASBUILT_FILE_NAME}: the nominal feature offsets of {targets_path.name} stand in for the "
                         "as-built datum offsets")
        if args.target not in targets.targets:
            raise MountCheckInputError(f"target {args.target!r} is not in {targets_path}; it has "
                                       f"{', '.join(sorted(targets.targets))}")
        target = targets.get(args.target, args.gap_mm)
        flange = None if args.flange_pose is None else six_to_pose(args.flange_pose)
        pose, pose_note = registered_pose(params, registration, flange)
        defaults = MountTolerances.from_parameters(params)
        tolerances = MountTolerances(
            z_mm=defaults.z_mm if args.z_tolerance_mm is None else args.z_tolerance_mm,
            tilt_deg=defaults.tilt_deg if args.tilt_tolerance_deg is None else args.tilt_tolerance_deg,
            frame_px=defaults.frame_px if args.frame_check_px is None else args.frame_check_px,
            z_name=defaults.z_name if args.z_tolerance_mm is None else "option --z-tolerance-mm",
            tilt_name=defaults.tilt_name if args.tilt_tolerance_deg is None else "option --tilt-tolerance-deg",
            frame_name=defaults.frame_name if args.frame_check_px is None else "option --frame-check-px")
        report = check_mount(paths, target, pose, pose_note, tolerances)
    except (MountCheckInputError, OSError, ValueError, KeyError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return EXIT_INPUT_ERROR
    if registration.accepted is False:
        report.notes.append("the registration was not accepted (registration.json: accepted = false): the registered pose "
                            "is only as good as the registration")
    report.notes = notes + report.notes
    print(report.format_text())
    report.write_json(args.out or report_location(args.mount_check))
    return EXIT_FLAGGED if not report.passed else EXIT_OK


def main(argv: list[str] | None = None) -> int:
    """Run the check; returns the exit code."""
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.mount_check is not None:
        if args.registration is None or args.target is None:
            parser.error("--mount-check needs --registration (registration.json) and --target (the id of the mounted target)")
        if args.pilot is not None or args.session is not None:
            parser.error("--mount-check is a check of its own; do not combine it with --session or --pilot")
        return run_mount_check(args)
    if args.session is None:
        parser.error("--session is required (or use --mount-check)")
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
        results = pilot_post_check(session, session.params, session.geometry, args.pilot,
                                         tuple(args.pilot_subseries))
        print(format_post_check_table(results))
        if args.out is not None:
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.out.write_text(json.dumps(json_safe({f"{t}@{g}": asdict(r) for (t, g), r in results.items()}),
                                           indent=REPORT_JSON_INDENT), encoding="utf-8")
        return EXIT_OK if results else EXIT_INPUT_ERROR
    params = CheckParameters(
        min_valid_fraction=args.min_valid_fraction, border_margin_px=args.border_margin_px,
        plane_residual_warn_mm=args.plane_residual_warn_mm, pose_residual_warn_mm=args.pose_residual_warn_mm,
        normal_warn_deg=args.normal_warn_deg, classification_margin_px=args.classification_margin_px,
        min_plane_pixels=args.min_plane_pixels,
        limit_reference_z_mm=session.params.z_reference_mm if args.limit_reference_z_mm is None
        else args.limit_reference_z_mm,
        limit_depth_exponent=args.limit_depth_exponent, normal_sigma_factor=args.normal_sigma_factor,
        normal_limit_cap_deg=args.normal_limit_cap_deg)
    try:
        report = check_session(session, params)
    except UNREADABLE_ERRORS as error:
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
