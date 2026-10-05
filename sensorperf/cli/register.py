"""
Command line: solve the robot-to-sensor registration (procedure document,
Section 4, Steps 6 and 7) from the registration observations.

    python3 -m sensorperf.cli.register --observations planes.csv --out registration.json
    python3 -m sensorperf.cli.register --observations poses.csv --method fiducial --accept-mm 0.15

What it computes
----------------
The two transforms that turn a read-back robot pose into a ground-truth target
pose in the camera frame: ``camera_to_base`` (camera -> robot base) and
``target_to_flange`` (target frame -> flange). Target pose in the camera frame =
camera_to_base^-1 . flange_to_base . target_to_flange. The hand-eye equation
A_i X = Y B_i is solved by ``sensorperf.geometry.registration``. The default is the
plane-only solve (``--method planes``, ``solve_from_planes``): the registration target is
the patternless noise plate T2 (the patterned plate T1 no longer exists, redesign note
Section 3), so each pose gives the plate's front plane from a depth-plane fit. camera_to_base
is then fully observable; the in-plane position of the plate on the flange and its rotation
about its normal are not, and are not needed for T2; a constant depth offset is absorbed into
camera_to_base (Section 15). ``--method fiducial`` keeps the closed-form start and nonlinear
least squares from full target poses for observations that come from another source.

Input: --observations, a CSV with one row per registration pose
    pose_id
    x_mm, y_mm, z_mm, rotation_type, r1 .. r9    the read-back flange pose (flange ->
                    base; rotation_type is one of none, quaternion_wxyz, quaternion_xyzw,
                    euler_zyx_deg, euler_xyz_deg, fixed_xyz_deg, rotvec_deg, matrix, as in
                    sensorperf.acquisition.pose_log)
    and, for the target in the camera frame, either
    tx_mm, ty_mm, tz_mm, trx_deg, try_deg, trz_deg   the fiducial solve's target pose
                    (target -> camera): position in mm, rotation vector in degrees
    or
    nx, ny, nz, distance_mm   the depth-plane fit of the target's front plane in the
                    camera frame: unit normal toward the camera and the distance such that
                    n . p + distance = 0 for points p on the plane.

Output: --out (default registration.json), with the residual RMS in mm, the
standard error of the camera's Z offset in mm (plane method only: from the fit covariance, along the mean plate
normal; null/NaN for the fiducial method), the rotation residual in degrees, the pose count, the method and the
accept verdict; the same numbers are printed. --accept-mm is the acceptance limit (default
REGISTRATION_RESIDUAL_ACCEPT_MM of the parameter table).

Exit code: 0 when the residual is within the accept limit, 1 when it is not (the
file is still written, with accepted = false), 2 when the observations cannot be
read or solved.
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

from sensorperf.acquisition.pose_log import Messages, robot_pose_from_columns
from sensorperf.geometry.registration import (
    MIN_OBSERVATIONS_FULL_POSE, MIN_OBSERVATIONS_PLANES, PlaneObservation, plane_of_pose, solve_from_planes,
    solve_hand_eye,
)
from sensorperf.io.manifest import six_to_pose
from sensorperf.parameters import CharacterizationParameters

EXIT_OK = 0
EXIT_NOT_ACCEPTED = 1
EXIT_INPUT_ERROR = 2
"""Exit codes: accepted, solved but not accepted, input error."""
METHOD_FIDUCIAL = "fiducial"
METHOD_PLANES = "planes"
"""Values of --method."""
POSE_COLUMNS = ("tx_mm", "ty_mm", "tz_mm", "trx_deg", "try_deg", "trz_deg")
"""Target pose columns (target -> camera): position in mm, rotation vector in degrees."""
PLANE_COLUMNS = ("nx", "ny", "nz", "distance_mm")
"""Plane observation columns: unit normal toward the camera and the plane distance."""
NORMAL_COMPONENTS = 3
"""The first three plane columns are the normal's components."""
NORMAL_NORM_TOLERANCE = 1.0e-3
"""A plane normal whose length differs from 1 by more than this is an input error (it is normalized otherwise)."""
DEFAULT_OUT_NAME = "registration.json"
"""Default output file name."""


class ObservationError(Exception):
    """The observations file has to be fixed; the message says what to change."""


def read_observations(path: Path, method: str):
    """(pose ids, flange poses, target poses or None, plane observations or None) from the observations CSV. Raises
    ObservationError naming the row and column of a problem."""
    try:
        handle = path.open("r", newline="", encoding="utf-8-sig")
    except OSError as error:
        raise ObservationError(f"cannot read {path}: {error.strerror}") from None
    with handle:
        reader = csv.DictReader(handle)
        columns = [c.strip() for c in (reader.fieldnames or [])]
        rows = [{(k or "").strip(): v for k, v in row.items()} for row in reader]
    if not rows:
        raise ObservationError(f"{path} has no data rows")
    has_pose = all(c in columns for c in POSE_COLUMNS)
    has_plane = all(c in columns for c in PLANE_COLUMNS)
    if method == METHOD_FIDUCIAL and not has_pose:
        raise ObservationError(f"--method fiducial needs the columns {', '.join(POSE_COLUMNS)} (the target pose from the "
                               "fiducial solve); use --method planes with the columns nx, ny, nz, distance_mm")
    if method == METHOD_PLANES and not (has_plane or has_pose):
        raise ObservationError(f"--method planes needs the columns {', '.join(PLANE_COLUMNS)} (or the target pose columns)")
    messages = Messages()
    ids, flanges, targets, planes = [], [], [], []
    for line, row in enumerate(rows, start=2):
        pose_id = (row.get("pose_id") or f"line{line}").strip()
        where = f"observations line {line} (pose {pose_id})"
        flange = robot_pose_from_columns(row, where, messages)
        if flange is None:
            continue
        try:
            if has_pose and (method == METHOD_FIDUCIAL or not has_plane):
                values = [float(row[c]) for c in POSE_COLUMNS]
                target = six_to_pose(values)
                targets.append(target)
                planes.append(plane_of_pose(target))
            else:
                normal = np.array([float(row[c]) for c in PLANE_COLUMNS[:NORMAL_COMPONENTS]])
                if abs(np.linalg.norm(normal) - 1.0) > NORMAL_NORM_TOLERANCE:
                    raise ValueError(f"the plane normal ({', '.join(PLANE_COLUMNS[:NORMAL_COMPONENTS])}) must be a unit vector, got length "
                                     f"{np.linalg.norm(normal):.4f}")
                planes.append(PlaneObservation(normal / np.linalg.norm(normal), float(row["distance_mm"])))
        except (ValueError, KeyError, TypeError) as error:
            raise ObservationError(f"{where}: {error}; enter plain numbers") from None
        ids.append(pose_id)
        flanges.append(flange)
    if messages.errors:
        raise ObservationError("\n".join(messages.errors))
    for warning in messages.warnings:
        print(f"WARNING: {warning}", file=sys.stderr)
    return ids, flanges, (targets if method == METHOD_FIDUCIAL else None), planes


def build_parser() -> argparse.ArgumentParser:
    """The argument parser (every option has help text)."""
    parser = argparse.ArgumentParser(
        description="Solve the robot-to-sensor registration (camera_to_base, target_to_flange) from registration "
                    "observations: the read-back flange poses with depth-plane fits of the noise plate (the "
                    "default) or with full target poses (hand-eye). Writes registration.json and prints the residual and the accept verdict.")
    parser.add_argument("--observations", required=True, type=Path, metavar="CSV",
                        help="one row per registration pose: pose_id, x_mm, y_mm, z_mm, rotation_type, r1..r9, and either "
                             + ", ".join(POSE_COLUMNS) + " (rotation vector in degrees) or " + ", ".join(PLANE_COLUMNS))
    parser.add_argument("--method", choices=(METHOD_FIDUCIAL, METHOD_PLANES), default=METHOD_PLANES,
                        help="planes: from depth-plane fits of the noise plate T2 only (the standard, no pattern "
                             "needed); fiducial: hand-eye from full target poses (default %(default)s)")
    parser.add_argument("--accept-mm", type=float, default=None, metavar="MM",
                        help="acceptance limit of the RMS residual in mm (default: REGISTRATION_RESIDUAL_ACCEPT_MM, "
                             f"{CharacterizationParameters().registration_residual_accept_mm:g})")
    parser.add_argument("--out", type=Path, default=Path(DEFAULT_OUT_NAME), metavar="PATH",
                        help="registration file to write (default %(default)s)")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the solve; returns the exit code."""
    args = build_parser().parse_args(argv)
    accept = args.accept_mm if args.accept_mm is not None else CharacterizationParameters().registration_residual_accept_mm
    try:
        ids, flanges, targets, planes = read_observations(args.observations, args.method)
        minimum = MIN_OBSERVATIONS_FULL_POSE if args.method == METHOD_FIDUCIAL else MIN_OBSERVATIONS_PLANES
        if len(flanges) < minimum:
            raise ObservationError(f"--method {args.method} needs at least {minimum} observations, got {len(flanges)}")
        if args.method == METHOD_FIDUCIAL:
            registration = solve_hand_eye(flanges, targets, accept_rms_mm=accept)
        else:
            registration = solve_from_planes(flanges, planes, accept_rms_mm=accept)
    except (ObservationError, ValueError, np.linalg.LinAlgError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return EXIT_INPUT_ERROR
    registration.notes["observations"] = str(args.observations)
    registration.notes["accept_mm"] = accept
    args.out.parent.mkdir(parents=True, exist_ok=True)
    registration.save(args.out)
    verdict = "ACCEPTED" if registration.accepted else "NOT ACCEPTED"
    z_offset_se = ("not available" if not np.isfinite(registration.camera_z_offset_se_mm)
                   else f"{registration.camera_z_offset_se_mm:.4f} mm")
    print(f"Registration ({registration.method}) from {registration.pose_count} poses: residual RMS "
          f"{registration.residual_rms_mm:.4f} mm, camera Z offset standard error {z_offset_se}, "
          f"rotation residual {registration.rotation_residual_rms_deg:.4f} deg; "
          f"limit {accept:g} mm -> {verdict}. Wrote {args.out}")
    if not registration.accepted:
        print("The residual is above the limit: re-check the fiducial solve, the tool frame and the flange pose "
              "convention (rotation_type), or capture more registration poses.", file=sys.stderr)
    return EXIT_OK if registration.accepted else EXIT_NOT_ACCEPTED


if __name__ == "__main__":
    sys.exit(main())
