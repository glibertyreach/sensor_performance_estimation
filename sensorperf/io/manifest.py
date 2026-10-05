"""
The capture manifest of the characterization procedure (document, Section 9):
one row per captured frame, saying which procedure and target it belongs to,
where the robot had put the target (read-back pose), where that puts the target
in the camera frame (from registration), and the environmental readings.

File name rule (Section 9)
--------------------------
``<proc>_<target>_G<gap>_Z<zzzz>_F<field>_P<pose>_f<frame>.mc``, for example
``C_T5_G15_Z0800_F0_P017_f03.mc``: procedure C (area), target T5 (the
cutout plate), gap 15 mm, station Z = 800 mm, field position
0 (center), pose 17, frame 3. A target without a back plate (T2) writes
``G0``. :func:`format_file_name` and :func:`parse_file_name` are the two
directions of this rule; the manifest is the authority when both exist.

Frames and conventions
----------------------
- Camera frame: the left IR camera, Z along its optical axis, H (= x) along
  image columns, V (= y) along image rows; millimeters.
- Target frame: origin at the target's reference point on the front face, x
  along the target's H direction, y along V, z pointing AWAY from the sensor
  (into the target), so a fronto-parallel target has the identity rotation and
  its back plate lies at z = +G. See sensorperf.geometry.targets.
- ``robot_pose``: the read-back flange pose, flange -> robot base.
- ``target_pose_camera``: target -> camera, computed from the registration and
  the read-back robot pose (sensorperf.geometry.registration); this is the
  ground-truth pose every analysis uses.
- Poses are stored in CSV as six values: x, y, z in mm and a rotation vector
  (axis times angle) in degrees.

The columns of Section 9 are all present (``MANIFEST_COLUMNS``); the columns
after the document's list (sub-series, tilt, step, visit) carry the details of
the A tilt sub-series, the B-Z ladder, ramp and staircase, and the C and D variants,
which the document names in its steps but not in the column list.
"""
from __future__ import annotations

import csv
import re
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from sensorperf.geometry.transforms import RigidTransform
from sensorperf.parameters import FIELD_POSITION_CODES, PROCEDURES

# ---------------------------------------------------------------------------
# Sub-series labels (the "subseries" column)
# ---------------------------------------------------------------------------
SUBSERIES_MAIN = "main"
"""The main stations of a series."""
SUBSERIES_TILT = "tilt"
"""A tilt sub-series pose (Section 5, Step 5)."""
SUBSERIES_REMOUNT = "remount"
"""The repeat-mount check (Section 5, Step 6)."""
SUBSERIES_FILTERS_OFF = "filters_off"
"""A repeat with the sensor's own filters off (Section 5, Step 7)."""
SUBSERIES_SENTINEL = "sentinel"
"""A drift sentinel capture."""
SUBSERIES_WARMUP = "warmup"
"""Warm-up monitoring captures (Section 4, Step 3)."""
SUBSERIES_SETTLE_SERVO = "settle_servo"
"""Settle check with servos on (Section 4, Step 4)."""
SUBSERIES_SETTLE_BRAKES = "settle_brakes"
"""Settle check with the brakes engaged (Section 4, Step 4)."""
SUBSERIES_MOUNT_CHECK = "mount_check"
"""Once-per-mount plane check (Section 4, Step 8)."""
SUBSERIES_NOMINAL = "nominal"
"""The nominal (zero-offset) edge pose of a B-HV station (Section 6.1, Step 2)."""
SUBSERIES_JITTER = "jitter"
"""A random-offset pose (B-HV Step 3, C Step 2, D)."""
SUBSERIES_LADDER = "ladder"
"""A B-Z step-ladder visit (Section 6.2, Step 2)."""
SUBSERIES_RAMP = "ramp"
"""The B-Z ramp pose of a station: T2 tilted about H so that the depth across the visible plate spans a few quanta
(Section 6.2)."""
SUBSERIES_STAIRCASE = "staircase"
"""A step of the OPTIONAL B-Z second pass, the fine staircase (Section 6.2)."""
SUBSERIES_FIELD = "field"
"""The C field sub-series (Section 7, Step 3)."""
SUBSERIES_OPEN = "open"
"""The C open-background variant (Section 7, Step 4)."""
SUBSERIES_PILOT = "pilot"
"""A D pilot frame (the post check, Section 8, Step 1)."""
SUBSERIES_EXTENDED = "extended"
"""The D extended 0 percent trials at the farthest stations (Section 8)."""

VISIT_A = "A"
VISIT_B = "B"
"""Visit labels of the B-Z ladder's A, B, A, B alternation."""

TILT_AXIS_NONE = ""
TILT_AXIS_H = "H"
TILT_AXIS_V = "V"
"""Tilt axis labels: the plate is rotated about the H axis or about the V axis."""

MANIFEST_FILE_NAME = "manifest.csv"
"""Name of the manifest in a session folder (Section 9)."""

POSE_COMPONENTS = ("x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg")
"""Suffixes of the six pose columns: translation in mm, rotation vector in degrees."""
ROBOT_POSE_COLUMNS = tuple(f"robot_{c}" for c in POSE_COMPONENTS)
"""Read-back robot flange pose, flange -> base."""
TARGET_POSE_COLUMNS = tuple(f"target_{c}" for c in POSE_COMPONENTS)
"""Target pose in the camera frame, target -> camera."""

MANIFEST_COLUMNS = (
    ("file", "procedure", "target_id", "gap_mm", "station_z_mm", "field", "pose_index", "frame_index",
     "seed", "offset_h_mm", "offset_v_mm")
    + ROBOT_POSE_COLUMNS + TARGET_POSE_COLUMNS
    + ("timestamp", "sensor_temp_c", "air_temp_c", "sensor_config_id",
       "subseries", "tilt_axis", "tilt_deg", "step_mm", "visit", "level_index"))
"""All manifest columns in order. Any further column is kept as string metadata."""

FIELD_FRACTION_ACHIEVED_KEY = "field_fraction_achieved"
"""Plan-notes key and manifest metadata column holding the achieved fraction of the requested field offset of a pose
(1 when the requested off-axis position fits the field; smaller when the planner pulled the pose inward). Poses that
are not placed at a field position do not carry it."""

OPTIONAL_FLOAT_COLUMNS = ("gap_mm", "sensor_temp_c", "air_temp_c", "step_mm", "tilt_deg")
"""Float columns that may be empty."""

FILE_NAME_PATTERN = re.compile(
    r"^(?P<proc>[A-Z])_(?P<target>T[0-9][A-Za-z]*)_G(?P<gap>[0-9]+(?:\.[0-9]+)?)_Z(?P<z>[0-9]{4})"
    r"_F(?P<field>[0-9])_P(?P<pose>[0-9]+)_f(?P<frame>[0-9]+)\.mc$")
"""The Section 9 file-name rule as a regular expression."""

FILE_NAME_FORMAT = "{proc}_{target}_G{gap}_Z{z:04d}_F{field}_P{pose:03d}_f{frame:02d}.mc"
"""The Section 9 file-name rule as a format string (see format_file_name)."""

POSE_INDEX_DIGITS = 3
FRAME_INDEX_DIGITS = 2
"""Digit counts of the pose and frame fields in file names (more digits are accepted on read)."""


# ---------------------------------------------------------------------------
# File-name rule
# ---------------------------------------------------------------------------
def _format_gap(gap_mm: float | None) -> str:
    """Gap field of a file name: an integer when whole, else a short decimal; 0 when none."""
    if gap_mm is None:
        return "0"
    if float(gap_mm) == int(gap_mm):
        return str(int(gap_mm))
    return f"{gap_mm:g}"


def format_file_name(procedure: str, target_id: str, gap_mm: float | None, station_z_mm: float, field: int,
                     pose_index: int, frame_index: int) -> str:
    """The capture file name of Section 9 for these fields."""
    return FILE_NAME_FORMAT.format(proc=procedure, target=target_id, gap=_format_gap(gap_mm),
                                   z=int(round(station_z_mm)), field=int(field), pose=int(pose_index),
                                   frame=int(frame_index))


def parse_file_name(name: str | Path) -> dict[str, Any]:
    """Fields of a Section 9 file name: procedure, target_id, gap_mm (None for
    G0), station_z_mm, field, pose_index, frame_index. Raises ValueError for a
    name that does not follow the rule."""
    match = FILE_NAME_PATTERN.match(Path(name).name)
    if match is None:
        raise ValueError(f"{Path(name).name!r} does not follow the file-name rule "
                         "<proc>_<target>_G<gap>_Z<zzzz>_F<field>_P<pose>_f<frame>.mc")
    gap = float(match.group("gap"))
    return {
        "procedure": match.group("proc"),
        "target_id": match.group("target"),
        "gap_mm": None if gap == 0.0 else gap,
        "station_z_mm": float(match.group("z")),
        "field": int(match.group("field")),
        "pose_index": int(match.group("pose")),
        "frame_index": int(match.group("frame")),
    }


# ---------------------------------------------------------------------------
# Records
# ---------------------------------------------------------------------------
@dataclass
class FrameRecord:
    """One captured frame and everything the manifest says about it."""

    path: Path
    procedure: str
    target_id: str
    gap_mm: float | None
    station_z_mm: float
    field: int
    pose_index: int
    frame_index: int
    robot_pose: RigidTransform
    target_pose_camera: RigidTransform
    seed: int | None = None
    offset_h_mm: float = 0.0
    offset_v_mm: float = 0.0
    timestamp: str = ""
    sensor_temp_c: float | None = None
    air_temp_c: float | None = None
    sensor_config_id: str = ""
    subseries: str = SUBSERIES_MAIN
    tilt_axis: str = TILT_AXIS_NONE
    tilt_deg: float = 0.0
    step_mm: float | None = None
    visit: str = ""
    level_index: int | None = None
    metadata: dict[str, str] = field(default_factory=dict)

    def pose_key(self) -> tuple:
        """The key shared by all frames of one commanded pose."""
        return (self.procedure, self.target_id, self.gap_mm, self.station_z_mm, self.field, self.pose_index)

    def configuration_key(self) -> tuple:
        """The key shared by all poses of one configuration (target, gap, station, field, sub-series)."""
        return (self.procedure, self.target_id, self.gap_mm, self.station_z_mm, self.field, self.subseries)

    def file_name(self) -> str:
        """The Section 9 file name these fields imply (the stored path may differ)."""
        return format_file_name(self.procedure, self.target_id, self.gap_mm, self.station_z_mm, self.field,
                                self.pose_index, self.frame_index)


def pose_to_six(transform: RigidTransform) -> list[float]:
    """x, y, z (mm) and rotation vector (degrees) of a transform."""
    return [float(v) for v in transform.translation] + [float(v) for v in transform.rotation_vector_degrees()]


def six_to_pose(values: Iterable[float]) -> RigidTransform:
    """Inverse of pose_to_six."""
    v = [float(x) for x in values]
    if len(v) != len(POSE_COMPONENTS):
        raise ValueError(f"a pose needs {len(POSE_COMPONENTS)} values, got {len(v)}")
    return RigidTransform.from_rotation_vector_degrees(v[3:6], v[0:3])


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------
def validate_record(record: FrameRecord, index: int | None = None) -> None:
    """Raise ValueError (naming the record) for an invalid procedure, field code,
    negative indices or a non-positive station depth."""
    where = f"record {index}" if index is not None else f"record {record.path.name}"
    if record.procedure not in PROCEDURES:
        raise ValueError(f"{where}: unknown procedure {record.procedure!r}; expected one of {PROCEDURES}")
    if record.field not in FIELD_POSITION_CODES:
        raise ValueError(f"{where}: field position {record.field} is not one of {FIELD_POSITION_CODES}")
    if record.pose_index < 0 or record.frame_index < 0:
        raise ValueError(f"{where}: pose and frame indices must be non-negative")
    if not record.station_z_mm > 0.0:
        raise ValueError(f"{where}: station_z_mm must be positive, got {record.station_z_mm}")
    if record.gap_mm is not None and not record.gap_mm > 0.0:
        raise ValueError(f"{where}: gap_mm must be positive or empty, got {record.gap_mm}")


# ---------------------------------------------------------------------------
# CSV I/O
# ---------------------------------------------------------------------------
def _text(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return repr(value)
    return str(value)


def _optional_float(text: str | None, column: str, index: int) -> float | None:
    if text is None or text.strip() == "":
        return None
    try:
        return float(text)
    except ValueError as error:
        raise ValueError(f"record {index}: column {column} is not a number: {text!r}") from error


def _required_float(text: str | None, column: str, index: int) -> float:
    value = _optional_float(text, column, index)
    if value is None:
        raise ValueError(f"record {index}: column {column} is required")
    return value


def _optional_int(text: str | None, column: str, index: int) -> int | None:
    if text is None or text.strip() == "":
        return None
    try:
        return int(text)
    except ValueError as error:
        raise ValueError(f"record {index}: column {column} is not an integer: {text!r}") from error


def record_to_row(record: FrameRecord, manifest_dir: Path) -> list[str]:
    """The CSV row of a record (file path relative to the manifest when below it)."""
    try:
        file_entry = record.path.resolve().relative_to(manifest_dir.resolve()).as_posix()
    except ValueError:
        file_entry = record.path.resolve().as_posix()
    values: dict[str, Any] = {
        "file": file_entry, "procedure": record.procedure, "target_id": record.target_id,
        "gap_mm": record.gap_mm, "station_z_mm": record.station_z_mm, "field": record.field,
        "pose_index": record.pose_index, "frame_index": record.frame_index, "seed": record.seed,
        "offset_h_mm": record.offset_h_mm, "offset_v_mm": record.offset_v_mm,
        "timestamp": record.timestamp,
        "sensor_temp_c": record.sensor_temp_c, "air_temp_c": record.air_temp_c,
        "sensor_config_id": record.sensor_config_id, "subseries": record.subseries,
        "tilt_axis": record.tilt_axis, "tilt_deg": record.tilt_deg, "step_mm": record.step_mm,
        "visit": record.visit, "level_index": record.level_index,
    }
    for column, value in zip(ROBOT_POSE_COLUMNS, pose_to_six(record.robot_pose)):
        values[column] = value
    for column, value in zip(TARGET_POSE_COLUMNS, pose_to_six(record.target_pose_camera)):
        values[column] = value
    return [_text(values[column]) for column in MANIFEST_COLUMNS]


def write_manifest_csv(path: str | Path, records: Iterable[FrameRecord]) -> Path:
    """Write records to a CSV manifest (columns MANIFEST_COLUMNS plus any metadata keys)."""
    manifest_path = Path(path)
    records = list(records)
    metadata_columns = sorted({key for record in records for key in record.metadata})
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    with manifest_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(list(MANIFEST_COLUMNS) + metadata_columns)
        for record in records:
            writer.writerow(record_to_row(record, manifest_path.parent)
                            + [record.metadata.get(key, "") for key in metadata_columns])
    return manifest_path


def load_manifest(path: str | Path) -> list[FrameRecord]:
    """Read a CSV manifest into records; relative file paths are relative to the
    manifest's directory. Raises ValueError with the record index for malformed rows."""
    manifest_path = Path(path)
    records: list[FrameRecord] = []
    with manifest_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        columns = reader.fieldnames or []
        missing = [c for c in MANIFEST_COLUMNS if c not in columns]
        if missing:
            raise ValueError(f"{manifest_path}: missing manifest columns {missing}")
        for index, row in enumerate(reader):
            file_entry = (row.get("file") or "").strip()
            if not file_entry:
                raise ValueError(f"record {index}: missing 'file'")
            candidate = Path(file_entry)
            file_path = candidate if candidate.is_absolute() else manifest_path.parent / candidate
            record = FrameRecord(
                path=file_path,
                procedure=(row.get("procedure") or "").strip(),
                target_id=(row.get("target_id") or "").strip(),
                gap_mm=_optional_float(row.get("gap_mm"), "gap_mm", index),
                station_z_mm=_required_float(row.get("station_z_mm"), "station_z_mm", index),
                field=int(_required_float(row.get("field"), "field", index)),
                pose_index=int(_required_float(row.get("pose_index"), "pose_index", index)),
                frame_index=int(_required_float(row.get("frame_index"), "frame_index", index)),
                robot_pose=six_to_pose(_required_float(row.get(c), c, index) for c in ROBOT_POSE_COLUMNS),
                target_pose_camera=six_to_pose(_required_float(row.get(c), c, index) for c in TARGET_POSE_COLUMNS),
                seed=_optional_int(row.get("seed"), "seed", index),
                offset_h_mm=_optional_float(row.get("offset_h_mm"), "offset_h_mm", index) or 0.0,
                offset_v_mm=_optional_float(row.get("offset_v_mm"), "offset_v_mm", index) or 0.0,
                timestamp=(row.get("timestamp") or "").strip(),
                sensor_temp_c=_optional_float(row.get("sensor_temp_c"), "sensor_temp_c", index),
                air_temp_c=_optional_float(row.get("air_temp_c"), "air_temp_c", index),
                sensor_config_id=(row.get("sensor_config_id") or "").strip(),
                subseries=(row.get("subseries") or SUBSERIES_MAIN).strip() or SUBSERIES_MAIN,
                tilt_axis=(row.get("tilt_axis") or "").strip(),
                tilt_deg=_optional_float(row.get("tilt_deg"), "tilt_deg", index) or 0.0,
                step_mm=_optional_float(row.get("step_mm"), "step_mm", index),
                visit=(row.get("visit") or "").strip(),
                level_index=_optional_int(row.get("level_index"), "level_index", index),
                metadata={k: v for k, v in row.items()
                          if k not in MANIFEST_COLUMNS and k is not None and v not in (None, "")},
            )
            validate_record(record, index)
            records.append(record)
    return records


# ---------------------------------------------------------------------------
# Selection and grouping
# ---------------------------------------------------------------------------
def select(records: Iterable[FrameRecord], **criteria: Any) -> list[FrameRecord]:
    """Records whose attributes equal the given values, e.g.
    select(records, procedure="A", subseries="main"). A tuple or list value
    matches any of its members."""
    chosen = []
    for record in records:
        keep = True
        for name, wanted in criteria.items():
            value = getattr(record, name)
            if isinstance(wanted, (tuple, list, set, frozenset)):
                keep = value in wanted
            else:
                keep = value == wanted
            if not keep:
                break
        if keep:
            chosen.append(record)
    return chosen


def group_by_pose(records: Iterable[FrameRecord]) -> "OrderedDict[tuple, list[FrameRecord]]":
    """Frames grouped by pose key, in order of first appearance, each group sorted by frame index."""
    groups: "OrderedDict[tuple, list[FrameRecord]]" = OrderedDict()
    for record in records:
        groups.setdefault(record.pose_key(), []).append(record)
    for key in groups:
        groups[key].sort(key=lambda r: r.frame_index)
    return groups


def group_by_configuration(records: Iterable[FrameRecord]) -> "OrderedDict[tuple, list[FrameRecord]]":
    """Frames grouped by configuration key (procedure, target, gap, station, field, sub-series)."""
    groups: "OrderedDict[tuple, list[FrameRecord]]" = OrderedDict()
    for record in records:
        groups.setdefault(record.configuration_key(), []).append(record)
    return groups


def pose_order(records: Iterable[FrameRecord]) -> list[tuple]:
    """Pose keys in acquisition order, judged by timestamp when present, else by appearance."""
    first_seen: dict[tuple, tuple] = {}
    for position, record in enumerate(records):
        key = record.pose_key()
        stamp = (record.timestamp or "", position)
        if key not in first_seen or stamp < first_seen[key]:
            first_seen[key] = stamp
    return [key for key, _ in sorted(first_seen.items(), key=lambda item: item[1])]


def offsets_px(records: Iterable[FrameRecord], focal_x_px: float) -> np.ndarray:
    """(N, 2) commanded lateral offsets in pixels at each record's station depth
    (H, V), using p(Z) = Z / f_x."""
    rows = [(r.offset_h_mm * focal_x_px / r.station_z_mm, r.offset_v_mm * focal_x_px / r.station_z_mm)
            for r in records]
    return np.asarray(rows, dtype=np.float64).reshape(-1, 2)
