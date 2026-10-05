"""
Robot pose log -> capture manifest (document, Section 9).

The capture PC records, for every captured frame or pose, where the robot
flange really was (the read-back pose) and which file the sensor wrote.
:func:`build_manifest` matches those entries with the capture files on disk and
with the plan (poses.csv from ``acquisition/plan.py``), and writes one
:class:`sensorperf.io.manifest.FrameRecord` per frame: the plan supplies the
sub-series, seed, lateral offsets, tilt, step, visit and level; the pose log
supplies the robot pose and the optional readings; the registration turns the
read-back flange pose into the ground-truth target pose in the camera frame
(``Registration.target_to_camera``). The capture files are never opened: only
their names matter.

Pose log formats (CSV, one header row, positions in millimeters in the robot
base frame, the read-back flange pose = flange -> base)

    Format 1, one row per frame (recognized by a ``file`` column)
        file, x_mm, y_mm, z_mm, rotation_type, r1 .. r9 [, optional columns]
        ``file`` is the Section 9 file name (a folder part is ignored), for example
        ``C_T5_G15_Z0800_F0_P017_f03.mc``; it identifies the pose and the frame.

    Format 2, one row per pose (recognized by the identity columns and ``frames``)
        procedure, target_id, gap_mm, station_z_mm, field, pose_index, frames,
        x_mm, y_mm, z_mm, rotation_type, r1 .. r9 [, optional columns]
        The row applies to frames 0 .. frames-1 of that pose (``frames`` may be
        left out: the planned frame count is used). ``gap_mm`` is empty for a
        target without a back plate.

    Format 2b, one row per frame with the identity columns (``frame_index`` in
        place of ``frames``) is also accepted.

    The ``poses.csv`` written by ``write_plan`` with a registration is accepted
    too (columns base_x_mm .. base_z_mm and r00..r22): it reads as a pose log
    in which the robot reached every commanded pose exactly, which is how the
    synthetic sessions and the tests use it.

    Optional columns, all of which end up in the manifest when present: timestamp,
    sensor_temp_c, air_temp_c (logged every TEMPERATURE_LOG_INTERVAL_MIN).

    Resolution of the logged position: for series Z the read-back pose is the ground truth of the depth step, so
    x_mm, y_mm, z_mm must be written with at least ``MIN_POSE_LOG_DECIMALS`` decimals (0.01 mm). A log rounded to 0.1 mm
    makes the small ladder rungs meaningless; ``build_manifest`` warns, naming the problem, when every z_mm value of the
    series-Z rows has fewer decimals.

Rotation conventions (adapted from ``sphcal/cli/make_manifest.py`` of the
depth-calibration repository, which this table reproduces verbatim; all through
scipy.spatial.transform.Rotation; the result is the flange-to-base rotation
matrix; angles in degrees)

    rotation_type      r1 r2 r3 (r4 ...)      meaning                        scipy call
    none               (unused)               identity                       Rotation.identity()
    quaternion_wxyz    w x y z                unit quaternion, scalar first  from_quat([x, y, z, w])
    quaternion_xyzw    x y z w                unit quaternion, scalar last   from_quat([x, y, z, w])
    euler_zyx_deg      A B C                  intrinsic Z-Y-X: rotate about    from_euler("ZYX", [A, B, C])
                                              tool z by A, then about the new
                                              y by B, then about the new x by
                                              C (KUKA A, B, C)
    euler_xyz_deg      a b c                  intrinsic X-Y-Z: about x by a,   from_euler("XYZ", [a, b, c])
                                              then new y by b, then new z by c
    fixed_xyz_deg      W P R                  extrinsic x-y-z: about the       from_euler("xyz", [W, P, R])
                                              fixed base x by W, then fixed
                                              y by P, then fixed z by R
                                              (FANUC W, P, R)
    rotvec_deg         x y z                  rotation vector, direction =    from_rotvec(radians(v))
                                              axis, length = angle in degrees
    matrix             m00 m01 m02 m10 ... m22   row-major 3x3 rotation       from_matrix(M)
                                              matrix (r1..r9)

A quaternion that is not of unit length is normalized and reported as a warning;
a rotation matrix that is not orthonormal is also reported and re-orthonormalized.

Problems are collected, not raised one by one: every unmatched file (a capture
file with no log row or no plan row), every unmatched plan pose (no capture file),
every log row naming a file that is not on disk and every pose whose read-back
target pose is far from the planned one is listed by name. Warnings do not stop
the run unless ``strict`` is set, in which case they are errors.
"""
from __future__ import annotations

import csv
import warnings
from collections import defaultdict
from dataclasses import dataclass, field
from decimal import Decimal, InvalidOperation
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from sensorperf.acquisition.plan import MIN_POSE_LOG_DECIMALS, PlannedCapture, read_plan_csv
from sensorperf.geometry.registration import Registration
from sensorperf.geometry.transforms import RigidTransform
from sensorperf.io.manifest import FrameRecord, format_file_name, parse_file_name, validate_record
from sensorperf.parameters import PROCEDURE_ZSTEP

# ---------------------------------------------------------------------------
# Constants (rotation table: from sphcal/cli/make_manifest.py, same names and values)
# ---------------------------------------------------------------------------
QUATERNION_NORM_WARN_TOLERANCE = 1.0e-3
"""A quaternion whose norm differs from 1 by more than this is reported as non-unit
(it is normalized in any case)."""
MATRIX_ORTHONORMALITY_WARN_TOLERANCE = 1.0e-3
"""A rotation matrix whose R R^T differs from the identity by more than this is
reported (it is re-orthonormalized in any case)."""
MIN_QUATERNION_NORM = 1.0e-9
"""A quaternion shorter than this has no direction and is an error."""

ROTATION_NONE = "none"
ROTATION_QUATERNION_WXYZ = "quaternion_wxyz"
ROTATION_QUATERNION_XYZW = "quaternion_xyzw"
ROTATION_EULER_ZYX = "euler_zyx_deg"
ROTATION_EULER_XYZ = "euler_xyz_deg"
ROTATION_FIXED_XYZ = "fixed_xyz_deg"
ROTATION_ROTVEC = "rotvec_deg"
ROTATION_MATRIX = "matrix"

ROTATION_VALUE_COUNTS = {
    ROTATION_NONE: 0, ROTATION_QUATERNION_WXYZ: 4, ROTATION_QUATERNION_XYZW: 4, ROTATION_EULER_ZYX: 3,
    ROTATION_EULER_XYZ: 3, ROTATION_FIXED_XYZ: 3, ROTATION_ROTVEC: 3, ROTATION_MATRIX: 9,
}
"""How many r columns each rotation_type reads."""

EULER_SEQUENCES = {ROTATION_EULER_ZYX: "ZYX", ROTATION_EULER_XYZ: "XYZ", ROTATION_FIXED_XYZ: "xyz"}
"""scipy sequences: upper case is intrinsic (rotating axes), lower case extrinsic (fixed axes)."""

# ---------------------------------------------------------------------------
# Constants (new in this module)
# ---------------------------------------------------------------------------
LOG_VALUE_COLUMNS = tuple(f"r{index}" for index in range(1, ROTATION_VALUE_COUNTS[ROTATION_MATRIX] + 1))
"""The rotation value columns r1..r9 of a pose log."""
PLAN_MATRIX_COLUMNS = tuple(f"r{row}{col}" for row in range(3) for col in range(3))
"""Rotation matrix columns r00..r22 of a poses.csv with flange poses (row-major)."""
PLAN_TRANSLATION_COLUMNS = ("base_x_mm", "base_y_mm", "base_z_mm")
"""Flange position columns of a poses.csv with flange poses."""
LOG_TRANSLATION_COLUMNS = ("x_mm", "y_mm", "z_mm")
"""Flange position columns of a robot pose log."""
IDENTITY_COLUMNS = ("procedure", "target_id", "gap_mm", "station_z_mm", "field", "pose_index")
"""The Section 9 identity of a pose (the columns of a per-pose log row)."""
OPTIONAL_TEXT_COLUMNS = ("timestamp",)
OPTIONAL_FLOAT_COLUMNS = ("sensor_temp_c", "air_temp_c")
"""Optional pose log columns copied to the manifest."""
FRAME_SEPARATOR = "_f"
"""Separator of the pose part and the frame index in a Section 9 file name."""
CAPTURE_PATTERN = "*.mc"
"""Capture files looked for below the captures directory."""
MAX_NAMES_LISTED = 12
"""Names quoted in full in a message about unmatched files or poses (the count is always given)."""
POSE_AGREEMENT_WARN_MM = 1.0
POSE_AGREEMENT_WARN_DEG = 1.0
"""A read-back target pose farther than this (mm, degrees) from the planned pose is reported as a warning:
the robot did not reach the commanded pose, or the registration or the log's pose convention is wrong."""


class PoseLogError(ValueError):
    """The pose log, the plan or the captures have to be fixed; the message lists every problem found."""


@dataclass
class Messages:
    """Problems found while reading; errors stop the run, warnings do not (unless strict)."""

    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


@dataclass
class LogFrame:
    """One frame's worth of pose log: the read-back flange pose and the optional readings."""

    pose_key: tuple
    frame_index: int
    pose: RigidTransform
    text: dict[str, str]
    numbers: dict[str, float | None]
    where: str
    """Where it came from ("pose log line 12"), for messages."""


# ---------------------------------------------------------------------------
# Rotation conversion (verbatim from sphcal/cli/make_manifest.py)
# ---------------------------------------------------------------------------
def rotation_from_values(rotation_type: str, values: list[float], where: str,
                         messages: Messages) -> np.ndarray | None:
    """The 3 x 3 flange-to-base rotation matrix for a rotation_type and its r values
    (conventions in the module docstring). Returns None, after recording an
    error, when the row cannot be converted. ``where`` names the row for messages."""
    if rotation_type == ROTATION_NONE:
        return Rotation.identity().as_matrix()
    if rotation_type in (ROTATION_QUATERNION_WXYZ, ROTATION_QUATERNION_XYZW):
        quaternion = np.asarray(values, dtype=np.float64)
        norm = float(np.linalg.norm(quaternion))
        if norm < MIN_QUATERNION_NORM:
            messages.errors.append(f"{where}: the quaternion is all zeros; enter the robot's quaternion values")
            return None
        if abs(norm - 1.0) > QUATERNION_NORM_WARN_TOLERANCE:
            messages.warnings.append(
                f"{where}: the quaternion has length {norm:.5f}, not 1; it was normalized. If the robot reports "
                "a unit quaternion, check that the four values are in the order given by rotation_type")
        quaternion = quaternion / norm
        if rotation_type == ROTATION_QUATERNION_WXYZ:
            quaternion = np.roll(quaternion, -1)    # scipy wants x, y, z, w
        return Rotation.from_quat(quaternion).as_matrix()
    if rotation_type in EULER_SEQUENCES:
        return Rotation.from_euler(EULER_SEQUENCES[rotation_type], values, degrees=True).as_matrix()
    if rotation_type == ROTATION_ROTVEC:
        return Rotation.from_rotvec(np.radians(values)).as_matrix()
    if rotation_type == ROTATION_MATRIX:
        matrix = np.asarray(values, dtype=np.float64).reshape(3, 3)
        determinant = float(np.linalg.det(matrix))
        if determinant <= 0.0:
            messages.errors.append(f"{where}: the 3x3 matrix is not a proper rotation (determinant "
                                   f"{determinant:.3f}, it must be +1); check that r1..r9 are row-major")
            return None
        if np.abs(matrix @ matrix.T - np.eye(3)).max() > MATRIX_ORTHONORMALITY_WARN_TOLERANCE:
            messages.warnings.append(f"{where}: the matrix is not orthonormal (rows are not perpendicular unit "
                                     "vectors); it was re-orthonormalized. Check the number of digits logged.")
        return Rotation.from_matrix(matrix).as_matrix()
    messages.errors.append(f"{where}: unknown rotation_type {rotation_type!r}; use one of "
                           f"{', '.join(ROTATION_VALUE_COUNTS)}")
    return None


def robot_pose_from_columns(row: dict, where: str, messages: Messages) -> RigidTransform | None:
    """The read-back flange pose of a log row (x_mm, y_mm, z_mm, rotation_type, r1..r9; or the base_* and
    r00..r22 columns of a poses.csv with flange poses). Records errors and returns None when the row cannot be read."""
    errors_before = len(messages.errors)
    from_plan = all(name in row for name in PLAN_TRANSLATION_COLUMNS + PLAN_MATRIX_COLUMNS) \
        and not all(name in row for name in LOG_TRANSLATION_COLUMNS)
    translation_columns = PLAN_TRANSLATION_COLUMNS if from_plan else LOG_TRANSLATION_COLUMNS
    try:
        translation = [_number(row, name) for name in translation_columns]
        if from_plan:
            rotation_type, values = ROTATION_MATRIX, [_number(row, name) for name in PLAN_MATRIX_COLUMNS]
        else:
            rotation_type = (row.get("rotation_type") or "").strip().lower()
            count = ROTATION_VALUE_COUNTS.get(rotation_type)
            values = [_number(row, name) for name in LOG_VALUE_COLUMNS[:count or 0]]
    except ValueError as error:
        messages.errors.append(f"{where}: {error}; enter plain numbers")
        return None
    if any(value is None for value in translation):
        messages.errors.append(f"{where}: the position {', '.join(translation_columns)} must all be filled in")
    if any(value is None for value in values):
        messages.errors.append(f"{where}: rotation_type {rotation_type} needs the values r1..r"
                               f"{ROTATION_VALUE_COUNTS.get(rotation_type, 0)} to be filled in")
    if len(messages.errors) > errors_before:
        return None
    rotation = rotation_from_values(rotation_type, [float(v) for v in values], where, messages)
    if rotation is None:
        return None
    return RigidTransform(rotation, np.asarray(translation, dtype=np.float64))


def _number(row: dict, column: str) -> float | None:
    """A float from a CSV cell; None when the column is absent or the cell empty.
    Raises ValueError for text that is not a number."""
    text = (row.get(column) or "").strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        raise ValueError(f"column {column} is not a number: {text!r}") from None


# ---------------------------------------------------------------------------
# Pose log
# ---------------------------------------------------------------------------
def _identity_key(row: dict, where: str, messages: Messages) -> tuple | None:
    """The pose key of a row with the Section 9 identity columns, or None after recording an error."""
    try:
        gap = _number(row, "gap_mm")
        station = _number(row, "station_z_mm")
        field_code, pose_index = _number(row, "field"), _number(row, "pose_index")
    except ValueError as error:
        messages.errors.append(f"{where}: {error}")
        return None
    procedure, target = (row.get("procedure") or "").strip(), (row.get("target_id") or "").strip()
    if not procedure or not target or station is None or field_code is None or pose_index is None:
        messages.errors.append(f"{where}: the identity columns {', '.join(IDENTITY_COLUMNS)} must be filled in "
                               "(gap_mm may be empty)")
        return None
    return (procedure, target, gap, station, int(field_code), int(pose_index))


def read_pose_log(path: str | Path, plan_by_key: dict[tuple, PlannedCapture],
                  messages: Messages) -> list[LogFrame]:
    """Read the pose log (formats in the module docstring) into per-frame entries. A per-pose row is expanded
    to the frames of the pose (its ``frames`` column, else the planned count). Every problem found is appended
    to ``messages`` with the line it came from."""
    try:
        handle = Path(path).open("r", newline="", encoding="utf-8-sig")
    except OSError as error:
        messages.errors.append(f"cannot read the pose log {path}: {error.strerror}")
        return []
    with handle:
        reader = csv.DictReader(handle)
        columns = [name.strip() for name in (reader.fieldnames or []) if name is not None]
        rows = [{(key or "").strip(): value for key, value in row.items()} for row in reader]
    if not rows:
        messages.errors.append(f"the pose log {path} has no data rows")
        return []
    has_position = all(c in columns for c in LOG_TRANSLATION_COLUMNS) or all(c in columns for c in PLAN_TRANSLATION_COLUMNS)
    per_file = "file" in columns
    has_identity = all(c in columns for c in IDENTITY_COLUMNS)
    if not has_position or not (per_file or has_identity):
        messages.errors.append(
            f"the pose log {path} lacks columns. Expected x_mm, y_mm, z_mm, rotation_type, r1..r9 and either a "
            f"'file' column (one row per frame) or the identity columns {', '.join(IDENTITY_COLUMNS)} with 'frames' "
            "(one row per pose) or 'frame_index' (one row per frame)")
        return []
    entries: list[LogFrame] = []
    z_text_by_where: dict[str, str] = {}                        # raw z_mm text per line, for the resolution check
    for line, row in enumerate(rows, start=2):                  # line 1 is the header
        where = f"pose log line {line}"
        z_text_by_where[where] = (row.get(LOG_TRANSLATION_COLUMNS[2]) or row.get(PLAN_TRANSLATION_COLUMNS[2]) or "").strip()
        pose = robot_pose_from_columns(row, where, messages)
        text = {c: (row.get(c) or "").strip() for c in OPTIONAL_TEXT_COLUMNS if c in row}
        numbers: dict[str, float | None] = {}
        for column in OPTIONAL_FLOAT_COLUMNS:
            try:
                numbers[column] = _number(row, column)
            except ValueError as error:
                messages.errors.append(f"{where}: {error}")
        if pose is None:
            continue
        if per_file and (row.get("file") or "").strip():
            name = Path((row.get("file") or "").strip()).name
            if not name.endswith(".mc"):
                name += ".mc"
            try:
                fields_ = parse_file_name(name)
            except ValueError as error:
                messages.errors.append(f"{where}: {error}")
                continue
            key = (fields_["procedure"], fields_["target_id"], fields_["gap_mm"], fields_["station_z_mm"],
                   fields_["field"], fields_["pose_index"])
            entries.append(LogFrame(key, fields_["frame_index"], pose, text, numbers, where))
            continue
        key = _identity_key(row, where, messages)
        if key is None:
            continue
        try:
            frames, frame_index = _number(row, "frames"), _number(row, "frame_index")
        except ValueError as error:
            messages.errors.append(f"{where}: {error}")
            continue
        if frame_index is not None and frames is None:
            entries.append(LogFrame(key, int(frame_index), pose, text, numbers, where))
            continue
        if frames is None:
            planned = plan_by_key.get(key)
            if planned is None:
                messages.errors.append(f"{where}: no 'frames' count and the pose is not in the plan; add the frame count")
                continue
            frames = planned.frames
        for index in range(int(frames)):
            entries.append(LogFrame(key, index, pose, text, numbers, where))
    _check_z_resolution(entries, z_text_by_where, messages)
    return entries


def _decimals(text: str) -> int | None:
    """Decimals written in a number's text ("750.000" -> 3, "750.1" -> 1, "750" -> 0), None if it is not a number."""
    try:
        exponent = Decimal(text).as_tuple().exponent
    except InvalidOperation:
        return None
    return max(-exponent, 0) if isinstance(exponent, int) else None


def _check_z_resolution(entries: list[LogFrame], z_text_by_where: dict[str, str], messages: Messages) -> None:
    """Warn when every z_mm value of the series-Z rows is written with fewer than ``MIN_POSE_LOG_DECIMALS`` decimals:
    the read-back pose is the step truth of series Z, so a log rounded to 0.1 mm makes the small rungs meaningless."""
    counts = [_decimals(z_text_by_where[e.where]) for e in entries if e.pose_key[0] == PROCEDURE_ZSTEP]
    counts = [c for c in counts if c is not None]
    if counts and all(c < MIN_POSE_LOG_DECIMALS for c in counts):
        messages.warnings.append(
            f"pose log resolution: every z_mm value of the series-Z rows has fewer than {MIN_POSE_LOG_DECIMALS} decimals "
            f"(at most {max(counts)}); the read-back pose is the step truth of series Z, so write x_mm, y_mm, z_mm "
            f"with at least {MIN_POSE_LOG_DECIMALS} decimals (0.01 mm) or the small rungs are meaningless")


# ---------------------------------------------------------------------------
# Matching and manifest
# ---------------------------------------------------------------------------
def _pose_name(planned: PlannedCapture) -> str:
    """The Section 9 file name of a pose without the frame index and extension (for messages)."""
    return planned.file_name(0).rsplit(FRAME_SEPARATOR, 1)[0]


def _listed(names: list[str]) -> str:
    """The names for a message: all of them up to MAX_NAMES_LISTED, then a count of the rest."""
    shown = ", ".join(names[:MAX_NAMES_LISTED])
    return shown + (f", ... ({len(names) - MAX_NAMES_LISTED} more)" if len(names) > MAX_NAMES_LISTED else "")


def build_manifest_with_report(pose_log_csv: str | Path, captures_dir: str | Path, plan_csv: str | Path,
                               registration: Registration, sensor_config_id: str = "",
                               strict: bool = False) -> tuple[list[FrameRecord], Messages]:
    """Build the manifest records and report every problem found (see :func:`build_manifest`).

    Returns (records in plan order and frame order, messages). With ``strict`` every warning is moved to the
    errors. The function does not raise for input problems: the caller decides (the CLI exits with code 2)."""
    messages = Messages()
    try:
        plan = read_plan_csv(plan_csv)
    except (OSError, ValueError) as error:
        messages.errors.append(f"cannot read the plan {plan_csv}: {error}")
        return [], messages
    plan_by_key: dict[tuple, PlannedCapture] = {}
    for planned in plan:
        if planned.pose_key() in plan_by_key:
            messages.errors.append(f"the plan lists pose {planned.file_name(0)} twice; pose keys must be unique")
        plan_by_key[planned.pose_key()] = planned
    captures_root = Path(captures_dir)
    if not captures_root.is_dir():
        messages.errors.append(f"--captures {captures_root} is not a directory; give the folder holding the .mc files")
        return [], messages
    on_disk: dict[str, Path] = {}
    for file in sorted(captures_root.rglob(CAPTURE_PATTERN)):
        if file.name in on_disk:
            messages.errors.append(f"the capture file name {file.name} occurs twice ({on_disk[file.name]} and {file}); "
                                   "file names must be unique below the captures folder")
        on_disk[file.name] = file
    if not on_disk:
        messages.errors.append(f"no {CAPTURE_PATTERN} capture files found below {captures_root}; check the --captures path")
    entries = read_pose_log(pose_log_csv, plan_by_key, messages)

    logged: dict[tuple, LogFrame] = {}
    for entry in entries:
        key = (entry.pose_key, entry.frame_index)
        if key in logged:
            messages.errors.append(f"{entry.where}: the frame {_frame_name(entry)} is logged twice (first at "
                                   f"{logged[key].where}); frames must be unique")
            continue
        logged[key] = entry

    records: list[FrameRecord] = []
    logged_not_on_disk: list[str] = []
    logged_not_in_plan: list[str] = []
    used_files: set[str] = set()
    pose_frames: dict[tuple, int] = defaultdict(int)
    for (pose_key, frame_index), entry in logged.items():
        name = _frame_name(entry)
        planned = plan_by_key.get(pose_key)
        if planned is None:
            logged_not_in_plan.append(name)
            continue
        path = on_disk.get(name)
        if path is None:
            logged_not_on_disk.append(name)
            continue
        used_files.add(name)
        pose_frames[pose_key] += 1
        record = _record(planned, frame_index, path, entry, registration, sensor_config_id)
        try:
            validate_record(record)
        except ValueError as error:
            messages.errors.append(f"{entry.where}: {error}")
            continue
        records.append(record)

    if logged_not_in_plan:
        messages.warnings.append(f"{len(logged_not_in_plan)} logged frame(s) are not in the plan, so they were left out "
                                 f"(a different plan, or a typo in the file name): {_listed(logged_not_in_plan)}")
    if logged_not_on_disk:
        messages.warnings.append(f"{len(logged_not_on_disk)} logged frame(s) have no capture file below {captures_root}: "
                                 f"{_listed(logged_not_on_disk)}")
    logged_names = {_frame_name(e) for e in entries}
    unlogged_planned, unlogged_unplanned = [], []
    for name in sorted(n for n in on_disk if n not in used_files and n not in logged_names):
        try:
            parsed = parse_file_name(name)
            key = (parsed["procedure"], parsed["target_id"], parsed["gap_mm"], parsed["station_z_mm"], parsed["field"],
                   parsed["pose_index"])
            (unlogged_planned if key in plan_by_key else unlogged_unplanned).append(name)
        except ValueError:
            unlogged_unplanned.append(name)
    if unlogged_planned:
        messages.warnings.append(f"{len(unlogged_planned)} capture file(s) have no row in the pose log, so they were left "
                                 f"out; add them to the log or remove them: {_listed(unlogged_planned)}")
    if unlogged_unplanned:
        messages.warnings.append(f"{len(unlogged_unplanned)} capture file(s) are neither in the pose log nor in the plan "
                                 "(the name does not follow the Section 9 rule or names no planned pose), so they were "
                                 f"left out: {_listed(unlogged_unplanned)}")
    missing_poses = [_pose_name(p) for p in plan if p.pose_key() not in pose_frames]
    if missing_poses:
        messages.warnings.append(f"{len(missing_poses)} planned pose(s) have no captured and logged frame: "
                                 f"{_listed(missing_poses)}")
    for key, count in pose_frames.items():
        planned = plan_by_key[key]
        if count != planned.frames:
            messages.warnings.append(f"pose {_pose_name(planned)}: {count} frame(s) captured, "
                                     f"{planned.frames} planned")
    _check_agreement(records, plan_by_key, messages)
    if strict:
        messages.errors.extend(messages.warnings)
        messages.warnings = []
    order = {p.pose_key(): p.order for p in plan}
    records.sort(key=lambda r: (order.get(r.pose_key(), 0), r.frame_index))
    return records, messages


def _frame_name(entry: LogFrame) -> str:
    """The Section 9 file name of a logged frame."""
    procedure, target, gap, station, field_code, pose_index = entry.pose_key
    return format_file_name(procedure, target, gap, station, field_code, pose_index, entry.frame_index)


def _record(planned: PlannedCapture, frame_index: int, path: Path, entry: LogFrame, registration: Registration,
            sensor_config_id: str) -> FrameRecord:
    """The manifest record of one frame: plan fields, the read-back pose and the target pose through the registration."""
    return FrameRecord(
        path=path, procedure=planned.procedure, target_id=planned.target_id, gap_mm=planned.gap_mm,
        station_z_mm=planned.station_z_mm, field=planned.field, pose_index=planned.pose_index,
        frame_index=frame_index, robot_pose=entry.pose, target_pose_camera=registration.target_to_camera(entry.pose),
        seed=planned.seed, offset_h_mm=planned.offset_h_mm, offset_v_mm=planned.offset_v_mm,
        timestamp=entry.text.get("timestamp", ""),
        sensor_temp_c=entry.numbers.get("sensor_temp_c"), air_temp_c=entry.numbers.get("air_temp_c"),
        sensor_config_id=sensor_config_id, subseries=planned.subseries,
        tilt_axis=planned.tilt_axis, tilt_deg=planned.tilt_deg, step_mm=planned.step_mm, visit=planned.visit,
        level_index=planned.level_index)


def _check_agreement(records: list[FrameRecord], plan_by_key: dict[tuple, PlannedCapture], messages: Messages) -> None:
    """Warn for every pose whose read-back target pose is far from the planned pose (first frame of the pose)."""
    seen: set[tuple] = set()
    far: list[str] = []
    for record in records:
        key = record.pose_key()
        if key in seen:
            continue
        seen.add(key)
        planned = plan_by_key[key]
        distance, angle = record.target_pose_camera.difference_from(planned.target_to_camera)
        if distance > POSE_AGREEMENT_WARN_MM or angle > POSE_AGREEMENT_WARN_DEG:
            far.append(f"{_pose_name(planned)} ({distance:.2f} mm, {angle:.2f} deg)")
    if far:
        messages.warnings.append(
            f"{len(far)} pose(s) were read back farther than {POSE_AGREEMENT_WARN_MM:g} mm or {POSE_AGREEMENT_WARN_DEG:g} "
            f"deg from the planned target pose; check the registration, the pose convention (rotation_type) and the "
            f"robot's accuracy: {_listed(far)}")


def build_manifest(pose_log_csv: str | Path, captures_dir: str | Path, plan_csv: str | Path,
                   registration: Registration, sensor_config_id: str = "", strict: bool = False) -> list[FrameRecord]:
    """Match the pose log to the capture files and the plan and return one FrameRecord per frame (Section 9).

    ``captures_dir`` is the session root (or any folder; its sub-folders are searched); the files are matched by
    name and never opened. The target pose of every record is ``registration.target_to_camera(read-back flange
    pose)``. Raises :class:`PoseLogError` listing every error (and, with ``strict``, every warning); otherwise
    warnings are issued with ``warnings.warn``. Use :func:`build_manifest_with_report` to get the messages instead."""
    records, messages = build_manifest_with_report(pose_log_csv, captures_dir, plan_csv, registration,
                                                   sensor_config_id, strict)
    if messages.errors:
        raise PoseLogError("\n".join(messages.errors))
    for message in messages.warnings:
        warnings.warn(message, stacklevel=2)
    return records
