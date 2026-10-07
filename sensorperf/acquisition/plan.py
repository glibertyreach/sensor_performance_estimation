"""
Station and pose planning for Part I of the procedure (Sections 4 to 9).

A plan is a list of :class:`PlannedCapture`: every commanded pose of every
series, with its Section 9 identity (procedure, target, gap, station, field,
pose index), its sub-series label, the frame count, the logged random seed and
lateral offset, the tilt or Z-step details, and the ground-truth target pose in
the camera frame that the robot must realize. The synthetic session writer
renders a plan; the robot program executes it; the manifest builder matches the
captured files back to it.

The planning functions, one per series, apply the rules the procedure states
step by step (each docstring cites its section):

    plan_registration        Section 4, Step 6   (R, target T2, plane correspondence)
    plan_noise_series        Section 5           (A, target T2)
    plan_edge_series         Section 6.1         (B, targets T3a and T3b)
    plan_zstep_series        Section 6.2         (Z, target T2; the document's B-Z)
    plan_area_series         Section 7           (C, targets T4 and T5; every station)
    plan_detection_series    Section 8           (D, targets T4 and T5; every station)
    insert_sentinels         Section 5, Step 3   (S, drift sentinels on the mounted target, on the budget clock)
    plan_full_session        all of the above in the order of the procedure
    capture_budget           Section 9           (poses, frames and robot hours per series)
    write_plan               poses.csv, plan_summary.txt, plan.png

Conventions (see docs/design/code_design.md, Section 4)
    - The target pose is target -> camera, millimeters. A fronto-parallel target
      at (H, V, Z) has the identity rotation; ``targets.tilted_pose`` rotates it
      about its H or V axis.
    - Field positions: code 0 is the center, codes 1 to 4 are (-,-), (+,-),
      (+,+), (-,+) times FIELD_OFFSET_FRACTION times the half field at the
      station depth (``geometry.half_field_mm``).
    - Random draws: every random quantity comes from ``np.random.default_rng(seed)``
      with a seed drawn from the master generator and stored in the plan. The
      seed of a pose is the seed of the draw that fixed its lateral offset; a pose
      without an offset carries the seed of the shuffle that placed it in the
      order (poses of one shuffle share it). Shuffled series also store the
      shuffle seed in ``notes["order_seed"]``.
    - Pose indices are unique within (procedure, target, gap, station, field),
      so every pose has its own Section 9 file names.

Field-of-view fit (Section 5, Step 1: "Check that the plate fully covers the
analysis region of interest at every station. At Z_MIN off-axis this may limit
the field offset.")
    A pose at a field position is checked with :func:`pose_fits_field` and, when
    the target would not fit, pulled inward along its field direction (the same
    fraction of both the H and V offsets) until it fits; the fraction kept is
    stored in ``notes["field_placement"]`` and listed in plan_summary.txt, and the achieved fraction of the
    requested offset is stored for every pose placed at a field position in ``notes["field_fraction_achieved"]``
    (1 when the request fits), from where the manifest carries it to Analysis A.
    Targets with features (the raised square, the window, the arrays) must have
    every feature outline inside the image by the edge margin. A plate (T2)
    is judged per image axis: it must lie inside the image by the edge margin or,
    when it is larger than the field at that depth, cover the whole image across
    that axis, so that no plate edge falls into the border band of the analysis
    region of interest. The edge margin is BOUNDARY_BAND_HALF_WIDTH_PX (the ROI
    shrink of Analysis A, and the only margin the fit uses), plus half the phase-jitter span for jittered series.

Drift sentinels (Section 5, Step 3, as revised): a sentinel is captured on the front plane of the target that is
mounted at that point of the plan, so none needs a re-mount; see :func:`insert_sentinels`.
Tilt feasibility (Section 5, Step 5): a tilt of the T2 plate is planned only where its near edge stays at or beyond
Z_MIN; see :func:`tilt_is_feasible`.
"""
from __future__ import annotations

import csv
import json
import math
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from sensorperf.geometry.camera import PinholeCamera
from sensorperf.geometry.registration import Registration
from sensorperf.geometry.targets import (
    FEATURE_POST, TARGET_KIND_PLATE, TargetSet, TwoPlaneTarget, fronto_parallel_pose,
    make_standard_target_set, tilted_pose,
)
from sensorperf.geometry.transforms import RigidTransform
from sensorperf.io.manifest import (
    APPROACH_DIRECTION_KEY, APPROACH_FIXED_STAND, APPROACH_STANDARD, DRIFT_RUN_POSE_INDEX_BASE, FIELD_FRACTION_ACHIEVED_KEY, FILTERS_OFF_POSE_INDEX_BASE, FIXED_STAND_KEY, LATERAL_SWEEP_POSE_INDEX_BASE, OPEN_BACKGROUND_POSE_INDEX_BASE, SENTINEL_MOUNT_REFERENCE_KEY,
    OPTIONAL_SUBSERIES, STAIRCASE_POSE_INDEX_BASE, SUBSERIES_DRIFT_RUN, SUBSERIES_EXTENDED, SUBSERIES_FIELD, SUBSERIES_FILTERS_OFF, SUBSERIES_JITTER, SUBSERIES_LATERAL_SWEEP, SUBSERIES_LADDER, SUBSERIES_MAIN,
    SUBSERIES_NOMINAL, SUBSERIES_OPEN, SUBSERIES_RAMP, SUBSERIES_REMOUNT, SUBSERIES_SENTINEL, SUBSERIES_STAIRCASE, SUBSERIES_TILT,
    TARGET_POSE_COLUMNS, TILT_AXIS_H, TILT_AXIS_V, VISIT_A, VISIT_B, format_file_name, format_flag, pose_to_six,
    six_to_pose,
)
from sensorperf.parameters import (
    CharacterizationParameters, FIELD_POSITION_CENTER, FIELD_POSITION_CODES, FIELD_POSITION_SIGNS,
    PROCEDURE_AREA, PROCEDURE_DETECTION, PROCEDURE_EDGES, PROCEDURE_NOISE,
    PROCEDURE_REGISTRATION, PROCEDURE_SENTINEL, PROCEDURE_ZSTEP, SensorGeometry, TARGET_CUTOUTS,
    TARGET_DISKS, TARGET_NOISE_PLATE, TARGET_RAISED_SQUARE, TARGET_SQUARE_WINDOW, TIER_A_DISPARITY_QUANTUM_PX,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
# The pose-index bases of the optional sets outside the budget (FILTERS_OFF_POSE_INDEX_BASE, STAIRCASE_POSE_INDEX_BASE,
# LATERAL_SWEEP_POSE_INDEX_BASE, DRIFT_RUN_POSE_INDEX_BASE and OPEN_BACKGROUND_POSE_INDEX_BASE) and the file-name widths are
# defined together in ``sensorperf.io.manifest``.
SEED_UPPER_BOUND = 2 ** 31 - 1
"""Exclusive upper bound of the seeds drawn from the master generator (a 31-bit
integer, so a seed survives a CSV round trip and any 32-bit consumer)."""
FIELD_PULL_TOLERANCE = 1.0e-3
"""Resolution of the fraction of the requested field offset kept when a pose is pulled
inward (bisection stops when the bracket is narrower than this)."""
TILT_NEAR_EDGE_TOLERANCE_MM = 1.0e-9
"""A tilted plate whose near edge is at Z_MIN or beyond it by no less than minus this many millimeters counts as feasible
(numerical guard of the tilt feasibility rule, :func:`tilt_is_feasible`)."""
FIT_VIOLATION_TOLERANCE_PX = 1.0e-6
"""A fit violation at or below this many pixels counts as fitting (numerical guard)."""
RAMP_FIT_WORSENING_TOLERANCE_PX = 1.0
"""A ramp pose (a tilt of a fraction of a degree to a few degrees) that fits the field of view worse than the untilted plate
by more than this many pixels raises a warning; the sub-pixel worsening that a small tilt always causes at a station where
the untilted plate already touches the edge margin (the principal point is off the image center) is not worth one."""
FIELD_PULL_REPORT_TOLERANCE = 1.0e-9
"""A kept fraction below 1 by more than this counts as an adjustment worth reporting."""
FEATURE_OUTLINE_SAMPLES = 64
"""Points sampled on each feature outline for the field-of-view fit test."""
SECONDS_PER_HOUR = 3600.0
"""Seconds in an hour (budget table)."""
SECONDS_PER_MINUTE = 60.0
"""Seconds in a minute (sentinel interval)."""
TILT_AXIS_BOTH = "HV"
"""``tilt_axis`` label of a registration pose tilted about both H and V (the two angles are in ``notes``)."""
REGISTRATION_FIELD_DIMENSIONS = 2
"""Dimensions of the registration pose design that are field fractions (H and V); the rest are tilts."""
REGISTRATION_SAMPLER_DIMENSIONS = 2 * REGISTRATION_FIELD_DIMENSIONS
"""Dimensions of the registration pose design: field fraction H and V, tilt about H and V."""
BUDGET_TABLE_WIDTH = 49
"""Character width of the Section 9 budget table."""
PERCENT = 100.0
"""Percent per unit fraction (summary text)."""
PLOT_GRID_ALPHA = 0.3
"""Opacity of the grid lines of plan.png."""

DOCUMENT_ESTIMATE_POSES = 7194
DOCUMENT_ESTIMATE_FRAMES = 42520
DOCUMENT_ESTIMATE_HOURS = 7.18
"""The capture-budget estimate printed in Section 9 of the procedure document (poses, frames, robot hours) for the
redesigned plan: the totals of ``plan_full_session`` with the default parameters, the indicative geometry (10
frames/s) and no optional variants (no filters-off repeat, no open-background variant, no staircase; 7,194 poses, 42,520
frames, 7.18 h). The B-Z series has 369 poses: the step ladder (6 rungs x 10 cycles x 2 visits at the three reduced
stations, 360 poses, 3,600 frames) and the ramp (one pose at each of the nine stations, 450 frames). The A series has
47 main poses (nine ladder stations at five field positions plus the two legacy depths at the center), 16 tilt poses (the 800 and 1600 mm stations; every tilt at 400 mm is infeasible) and the re-mount check, and the
sentinels (11, on the mounted target) are part of the totals. plan_summary.txt compares the plan it summarizes with these
numbers, so a change of the parameters shows up as a ratio away from 1. The optional sets (the filters-off repeat, the B-Z
staircase, the B-HV lateral sweep, the drift run and the open-background variant of C) and the drift sentinels captured
during them are never part of these totals (see ``capture_budget``)."""

# OPTIONAL_SUBSERIES (imported from io.manifest): the filters-off repeat, the optional B-Z staircase, the optional B-HV
# lateral sweep, the optional separate drift run and the optional open-background variant of C, all outside the main
# (Section 9) budget. The captures of procedure
# letter S among them (the drift run, and the drift sentinels captured during one of the other sets) are kept out of the
# sentinel row of the main budget by their sub-series.
OPTIONAL_POSE_INDEX_BASES = {SUBSERIES_FILTERS_OFF: FILTERS_OFF_POSE_INDEX_BASE,
                             SUBSERIES_STAIRCASE: STAIRCASE_POSE_INDEX_BASE,
                             SUBSERIES_LATERAL_SWEEP: LATERAL_SWEEP_POSE_INDEX_BASE,
                             SUBSERIES_DRIFT_RUN: DRIFT_RUN_POSE_INDEX_BASE,
                             SUBSERIES_OPEN: OPEN_BACKGROUND_POSE_INDEX_BASE}
"""The pose-index base of each optional sub-series (all of them outside the Section 9 budget; ranges in ``io.manifest``).
The drift sentinels captured during an optional set count their pose indices from the base of that set."""

PLAN_CSV_NAME = "poses.csv"
PLAN_SUMMARY_NAME = "plan_summary.txt"
PLAN_FIGURE_NAME = "plan.png"
"""File names written by :func:`write_plan`."""

SERIES_ORDER = (PROCEDURE_REGISTRATION, PROCEDURE_NOISE, PROCEDURE_EDGES, PROCEDURE_ZSTEP, PROCEDURE_AREA,
                PROCEDURE_DETECTION, PROCEDURE_SENTINEL)
"""Procedure letters in the order of the procedure (and of the budget table)."""
SERIES_LABELS = {
    PROCEDURE_REGISTRATION: "Registration (R)", PROCEDURE_NOISE: "A noise", PROCEDURE_EDGES: "B-HV edges",
    PROCEDURE_ZSTEP: "B-Z depth steps", PROCEDURE_AREA: "C area", PROCEDURE_DETECTION: "D detection",
    PROCEDURE_SENTINEL: "Sentinels",
}
"""Series names of the Section 9 budget table, by procedure letter."""
PLANNED_SERIES = (PROCEDURE_REGISTRATION, PROCEDURE_NOISE, PROCEDURE_EDGES, PROCEDURE_ZSTEP, PROCEDURE_AREA,
                  PROCEDURE_DETECTION)
"""Series selectable on the command line (sentinels are inserted automatically)."""

ROTATION_MATRIX_COLUMNS = tuple(f"r{row}{col}" for row in range(3) for col in range(3))
"""Rotation matrix columns r00..r22 (row-major) of the flange pose in poses.csv."""
PLAN_IDENTITY_COLUMNS = ("order", "procedure", "target_id", "gap_mm", "station_z_mm", "field", "pose_index",
                         "frames", "subseries", "seed", "offset_h_mm", "offset_v_mm", "tilt_axis", "tilt_deg",
                         "step_mm", "visit", "level_index")
"""Identity and sub-series columns of poses.csv (Section 9 identity first)."""
FLANGE_POSE_COLUMNS = (("base_x_mm", "base_y_mm", "base_z_mm", "base_rx_deg", "base_ry_deg", "base_rz_deg")
                       + ROTATION_MATRIX_COLUMNS + ("quat_w", "quat_x", "quat_y", "quat_z"))
"""Flange pose to command in the robot base frame (only with a registration): position in mm and
rotation vector in degrees (six values), the rotation matrix r00..r22 row-major, and the unit
quaternion (w >= 0), as plan_poses.py of the calibration repository wrote them."""
MIN_POSE_LOG_DECIMALS = 2
"""Fewest decimals (0.01 mm) the series-Z rows of a robot pose log must show in x_mm, y_mm, z_mm: the read-back pose is
the step truth of series Z, and the smallest rung and staircase step are 0.1 mm, the robot's floor, so a log rounded to
0.1 mm would hide them (``pose_log.build_manifest`` warns)."""
APPROACH_FROM_BELOW = "from below"
"""``notes["approach"]`` of every series Z pose: the robot arrives at the pose moving toward larger Z (from the side of
smaller Z, nearer the sensor), after backing off by ``z_step_approach_overshoot_mm``, so backlash cancels in A / B."""
PLAN_NOTES_COLUMN = "notes"
"""Column holding the pose's notes as a JSON object."""
PLAN_CSV_COLUMNS = PLAN_IDENTITY_COLUMNS + TARGET_POSE_COLUMNS + (PLAN_NOTES_COLUMN,)
"""Columns of poses.csv without a registration (with one, FLANGE_POSE_COLUMNS follow)."""

PLOT_DPI = 130
"""Resolution of plan.png."""
PLOT_FIGURE_SIZE_IN = (13.0, 6.5)
"""Figure size of plan.png in inches (width, height)."""
PLOT_MARKER_SIZE = 9.0
"""Scatter marker area in points squared."""
PLOT_MARKER_ALPHA = 0.55
"""Marker opacity (many poses overlap, notably the D trials)."""
PLOT_FRUSTUM_COLOR = "#444444"
PLOT_FRUSTUM_LINE_WIDTH = 1.0
"""Frustum line color and width."""
PLOT_SERIES_STYLE = {
    PROCEDURE_REGISTRATION: ("#56B4E9", "D"), PROCEDURE_NOISE: ("#0072B2", "o"), PROCEDURE_EDGES: ("#009E73", "s"),
    PROCEDURE_ZSTEP: ("#D55E00", "^"), PROCEDURE_AREA: ("#CC79A7", "v"), PROCEDURE_DETECTION: ("#E69F00", "."),
    PROCEDURE_SENTINEL: ("#000000", "x"),
}
"""Colorblind-safe (Okabe-Ito) color and marker per procedure letter."""


# ---------------------------------------------------------------------------
# The record of one planned pose
# ---------------------------------------------------------------------------
@dataclass
class PlannedCapture:
    """One commanded pose of the plan (all frames of the pose share it)."""

    procedure: str
    target_id: str
    gap_mm: float | None
    station_z_mm: float
    field: int
    pose_index: int
    frames: int
    subseries: str
    target_to_camera: RigidTransform
    """The wanted ground-truth target pose (target -> camera), offsets and tilt applied."""
    seed: int | None = None
    """Seed of the random draw that produced this pose's offset or its place in the order."""
    offset_h_mm: float = 0.0
    offset_v_mm: float = 0.0
    tilt_axis: str = ""
    tilt_deg: float = 0.0
    step_mm: float | None = None
    """Commanded Z step (B-Z ladder) or staircase position relative to Z0, mm."""
    visit: str = ""
    """A or B for the B-Z ladder alternation."""
    level_index: int | None = None
    """Diameter-ladder level (D) when the pose targets one level; usually None (all levels per frame)."""
    order: int = 0
    """Acquisition order after randomization (0-based, over the whole plan)."""
    notes: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Record the approach direction of EVERY pose (specification, Part I). A pose that names its own approach (the
        lateral sweep) keeps it; a capture of a plate on a fixed stand has none to record (``APPROACH_FIXED_STAND``); every
        other pose is approached by the standard rule (``APPROACH_STANDARD``: from below in Z, from -H and -V laterally)."""
        if APPROACH_DIRECTION_KEY not in self.notes:
            self.notes = {**self.notes, APPROACH_DIRECTION_KEY: APPROACH_FIXED_STAND if self.notes.get(FIXED_STAND_KEY)
                          else APPROACH_STANDARD}

    def pose_key(self) -> tuple:
        """The key shared by all frames of this pose (same as FrameRecord.pose_key)."""
        return (self.procedure, self.target_id, self.gap_mm, self.station_z_mm, self.field, self.pose_index)

    def file_name(self, frame_index: int) -> str:
        """The Section 9 file name of one frame of this pose."""
        return format_file_name(self.procedure, self.target_id, self.gap_mm, self.station_z_mm, self.field,
                                self.pose_index, frame_index)

    def manifest_metadata(self) -> dict[str, str]:
        """The plan quantities the manifest carries as extra columns (string metadata of the FrameRecord): the achieved
        fraction of the requested field offset, for a pose placed at a field position, and the mount-reference flag of a
        drift sentinel (``sentinel_mount_reference``: true for the first sentinel after its target was mounted, false for
        a later one), and the fixed-stand flag (``fixed_stand``: true for a capture of the optional drift run, whose plate
        stands still on a fixed stand, see ``io.manifest``), and the approach direction (``approach_direction``) of EVERY
        pose: ``APPROACH_STANDARD`` ("-Z,-H,-V") for the standard rule, "-H", "+H", "-V" or "+V" for a lateral-sweep pose,
        ``APPROACH_FIXED_STAND`` for the drift run. A pose without the quantity leaves the cell empty."""
        metadata: dict[str, str] = {}
        fraction = self.notes.get(FIELD_FRACTION_ACHIEVED_KEY)
        if fraction is not None:
            metadata[FIELD_FRACTION_ACHIEVED_KEY] = repr(float(fraction))
        reference = self.notes.get(SENTINEL_REFERENCE_KEY)
        if self.procedure == PROCEDURE_SENTINEL and reference is not None:
            metadata[SENTINEL_MOUNT_REFERENCE_KEY] = format_flag(bool(reference))
        if self.notes.get(FIXED_STAND_KEY):
            metadata[FIXED_STAND_KEY] = format_flag(True)
        direction = self.notes.get(APPROACH_DIRECTION_KEY)
        if direction:
            metadata[APPROACH_DIRECTION_KEY] = str(direction)
        return metadata


# ---------------------------------------------------------------------------
# Diagnostics collected while planning
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class CReuse:
    """The D poses that ``--reuse-c-first-frames`` leaves out of the plan because the first frame of a C pose of the same
    configuration and station serves as the D trial instead (Section 8, Reuse). The Section 9 budget is computed
    without reuse, so :func:`capture_budget` adds these poses back to the D row."""

    poses: int
    """Number of D poses taken from C (one per reused C pose)."""
    frames: int
    """Frames those D poses would have had (FRAMES_PER_DETECTION_TRIAL each)."""
    per_configuration: int
    """The most C poses reused by one (target, gap, station) configuration (30 with the defaults)."""
    configurations: int
    """Number of (target, gap, station) configurations that reuse C poses."""


@dataclass
class PlanDiagnostics:
    """Warnings and information lines collected while planning, for plan_summary.txt.

    A caller that wants them passes one instance to the planning functions (or
    to :func:`plan_full_session`) and hands it to :func:`write_plan`."""

    warnings: list[str] = field(default_factory=list)
    """Things the technician should read before running the plan."""
    notes: list[str] = field(default_factory=list)
    """Informational lines (assumptions, counts)."""
    skipped: list[str] = field(default_factory=list)
    """Poses left out of the plan, each with its reason (the infeasible tilts of series A), listed in plan_summary.txt."""
    master_seed: int | None = None
    """Seed of the master generator, when the caller knows it (logged in the summary)."""
    c_reuse: CReuse | None = None
    """Set by the detection planner when it reused the first frames of C poses (``reuse_c_first_frames=True``)."""

    def warn(self, message: str) -> None:
        """Record a warning once (identical messages are not repeated)."""
        if message not in self.warnings:
            self.warnings.append(message)

    def note(self, message: str) -> None:
        """Record an information line once."""
        if message not in self.notes:
            self.notes.append(message)

    def skip(self, message: str) -> None:
        """Record once that poses were left out of the plan, with the reason."""
        if message not in self.skipped:
            self.skipped.append(message)


def _warn(diagnostics: PlanDiagnostics | None, message: str) -> None:
    """Record a warning when a diagnostics collector was given."""
    if diagnostics is not None:
        diagnostics.warn(message)


# ---------------------------------------------------------------------------
# Helpers: seeds, pose indices, camera, targets
# ---------------------------------------------------------------------------
def derive_seed(rng: np.random.Generator) -> int:
    """A seed for one random draw, taken from the master generator (31-bit integer)."""
    return int(rng.integers(0, SEED_UPPER_BOUND))


def jitter_offset_mm(seed: int, params: CharacterizationParameters, geometry: SensorGeometry,
                     station_z_mm: float) -> tuple[float, float]:
    """The random lateral offset (H, V) in mm of a seed: uniform over +/- PHASE_JITTER_SPAN_PX / 2
    in both directions, converted to millimeters at the station depth (Sections 6.1, 7, 8)."""
    half_span_mm = params.phase_jitter_span_mm(geometry, station_z_mm) / 2.0
    draw = np.random.default_rng(seed).uniform(-half_span_mm, half_span_mm, size=2)
    return float(draw[0]), float(draw[1])


class _PoseCounter:
    """Hands out pose indices that are unique within (procedure, target, gap, station, field)."""

    def __init__(self, start: int = 0) -> None:
        """``start`` is the first index handed out for every key."""
        self._start = start
        self._next: dict[tuple, int] = {}

    def next(self, key: tuple) -> int:
        """The next free pose index for the key."""
        index = self._next.get(key, self._start)
        self._next[key] = index + 1
        return index


def camera_of(geometry: SensorGeometry) -> PinholeCamera:
    """The pinhole camera of the sensor geometry (raises MissingSensorValue for unfilled values)."""
    return PinholeCamera(int(geometry.require("image_width_px")), int(geometry.require("image_height_px")),
                         float(geometry.require("sensor_fx_px")), float(geometry.require("sensor_fy_px")),
                         float(geometry.require("sensor_cx_px")), float(geometry.require("sensor_cy_px")))


def _targets(params: CharacterizationParameters, geometry: SensorGeometry, targets: TargetSet | None) -> TargetSet:
    """The given target set or the standard one of Section 3.2 for these parameters."""
    return targets if targets is not None else make_standard_target_set(params, geometry)


def _new_capture(counter: _PoseCounter, procedure: str, target_id: str, gap_mm: float | None, station_z_mm: float,
                 field_code: int, frames: int, subseries: str, pose: RigidTransform, **details: Any) -> PlannedCapture:
    """A PlannedCapture with the next free pose index. The station is rounded to whole
    millimeters because the file-name rule (Section 9) writes it that way; ``details`` are the
    optional PlannedCapture fields (seed, offsets, tilt, step, visit, notes)."""
    station = float(int(round(station_z_mm)))
    index = counter.next((procedure, target_id, gap_mm, station, field_code))
    offset = details.pop("offset", (0.0, 0.0))
    return PlannedCapture(procedure=procedure, target_id=target_id, gap_mm=gap_mm, station_z_mm=station,
                          field=field_code, pose_index=index, frames=frames, subseries=subseries,
                          target_to_camera=pose, offset_h_mm=float(offset[0]), offset_v_mm=float(offset[1]),
                          **details)


def _renumber(plan: list[PlannedCapture]) -> list[PlannedCapture]:
    """Set ``order`` to 0..N-1 along the list and return it."""
    for position, capture in enumerate(plan):
        capture.order = position
    return plan


def two_axis_tilt_rotation(tilt_h_deg: float, tilt_v_deg: float) -> np.ndarray:
    """Rotation of a target tilted about its H axis by tilt_h_deg and then about its V axis by
    tilt_v_deg (both through the reference point), as a 3 x 3 matrix. With one angle zero it is
    exactly what ``targets.tilted_pose`` gives for the other."""
    about_h = tilted_pose(0.0, 0.0, 0.0, TILT_AXIS_H, tilt_h_deg).rotation
    about_v = tilted_pose(0.0, 0.0, 0.0, TILT_AXIS_V, tilt_v_deg).rotation
    return about_v @ about_h


# ---------------------------------------------------------------------------
# Field-of-view fit (Section 5, Step 1)
# ---------------------------------------------------------------------------
def _fit_margin_px(params: CharacterizationParameters, jittered: bool) -> float:
    """Edge margin of the fit test: the ROI shrink BOUNDARY_BAND_HALF_WIDTH_PX, plus half the
    phase-jitter span when the series adds random offsets (so a jittered pose still fits)."""
    return params.boundary_band_half_width_px + (params.phase_jitter_span_px / 2.0 if jittered else 0.0)


def _fit_points_target_frame(target: TwoPlaneTarget) -> tuple[np.ndarray, bool]:
    """(N, 2) target-frame points whose projections the fit test looks at, and whether the
    target may instead cover the image across an axis (plates only). A target with features is
    judged by its feature outlines; a plate by the corners of its front extent."""
    if target.kind == TARGET_KIND_PLATE or not target.features:
        corners = np.array([(sx * target.half_width_mm, sy * target.half_height_mm)
                            for sx in (-1.0, 1.0) for sy in (-1.0, 1.0)])
        return corners, True
    outlines = [f.outline_points(FEATURE_OUTLINE_SAMPLES) for f in target.features if f.kind != FEATURE_POST]
    return np.vstack(outlines), False


def fit_violation_px(camera: PinholeCamera, target: TwoPlaneTarget, pose: RigidTransform, margin_px: float) -> float:
    """How far, in pixels, the target at this pose misses the field-of-view fit (0 when it fits).

    Targets with features: every feature outline point must project at least ``margin_px`` inside the
    image; the violation is the largest distance by which a point lies in the border band or outside. Plates: per
    image axis the projected extent must lie inside the image by the margin or span the whole image minus the
    margin (the plate is larger than the field there); the violation of an axis is that of the better of the two
    alternatives, and the plate's violation is the larger over the two axes. For a tilted plate the extent is
    that of the projected corners, which is slightly generous for the covering alternative. A point behind the
    camera gives infinity."""
    points_xy, may_cover = _fit_points_target_frame(target)
    points_camera = pose.apply_points(np.column_stack([points_xy, np.zeros(len(points_xy))]))
    u, v, in_front = camera.project(points_camera)
    if not np.all(in_front):
        return math.inf
    worst = 0.0
    for coordinate, size in ((u, camera.width), (v, camera.height)):
        low, high = float(np.min(coordinate)), float(np.max(coordinate))
        last = size - 1.0
        contained = max(margin_px - low, high - (last - margin_px), 0.0)
        covering = max(low - margin_px, (last - margin_px) - high, 0.0) if may_cover else math.inf
        worst = max(worst, min(contained, covering))
    return worst


def pose_fits_field(camera: PinholeCamera, target: TwoPlaneTarget, pose: RigidTransform, margin_px: float) -> bool:
    """Whether the target at this pose fits the field (module docstring, "Field-of-view fit"): the violation
    of :func:`fit_violation_px` is zero."""
    return fit_violation_px(camera, target, pose, margin_px) <= FIT_VIOLATION_TOLERANCE_PX


@dataclass(frozen=True)
class FieldPlacement:
    """Where a target was put for a requested field direction (after any pull-in)."""

    h_mm: float
    v_mm: float
    requested_h_mm: float
    requested_v_mm: float
    fraction_kept: float
    """1 when the requested position fits; smaller when the pose was pulled inward; 0 when only the center is acceptable."""
    violation_px: float
    """Fit violation of the final position (0 when it fits; see :func:`fit_violation_px`)."""

    @property
    def adjusted(self) -> bool:
        """True when the position differs from the request."""
        return self.fraction_kept < 1.0 - FIELD_PULL_REPORT_TOLERANCE

    @property
    def fits(self) -> bool:
        """False when the target does not fit the field even at the final position (the center is then as good as it gets)."""
        return self.violation_px <= FIT_VIOLATION_TOLERANCE_PX

    def as_notes(self) -> dict[str, Any]:
        """The record stored in ``PlannedCapture.notes['field_placement']``."""
        return {"requested_h_mm": self.requested_h_mm, "requested_v_mm": self.requested_v_mm, "h_mm": self.h_mm,
                "v_mm": self.v_mm, "fraction_kept": self.fraction_kept, "violation_px": self.violation_px,
                "fits": self.fits}


def place_in_field(params: CharacterizationParameters, geometry: SensorGeometry, target: TwoPlaneTarget,
                   station_z_mm: float, direction: tuple[float, float], margin_px: float,
                   rotation: np.ndarray | None = None) -> FieldPlacement:
    """Position of a target for a field direction (Section 5, Step 1).

    ``direction`` is the (H, V) sign pair of the field position, or fractions in [-1, 1] for the registration poses;
    the requested offset is direction x FIELD_OFFSET_FRACTION x the half field at the station depth. When the
    target does not fit there it is pulled inward along the direction (both offsets scaled by the same factor,
    found by bisection to FIELD_PULL_TOLERANCE) to the largest offset that fits. When the target does not fit even
    at the center (a plate larger than the field, for instance), "fits" means "no worse than at the center", so the
    off-axis stations still differ from the center station; the final violation is reported."""
    half_h, half_v = geometry.half_field_mm(station_z_mm)
    camera = camera_of(geometry)
    rotation = np.eye(3) if rotation is None else rotation

    def position(scale: float) -> tuple[float, float]:
        """The (H, V) offset in mm for a fraction of the requested offset."""
        # "+ 0.0" turns a negative zero (a zero offset times a negative sign) into a plain zero.
        return (direction[0] * scale * params.field_offset_fraction * half_h + 0.0,
                direction[1] * scale * params.field_offset_fraction * half_v + 0.0)

    def violation(scale: float) -> float:
        """The fit violation in pixels of the pose at a fraction of the requested offset."""
        h, v = position(scale)
        return fit_violation_px(camera, target, RigidTransform(rotation, [h, v, station_z_mm]), margin_px)

    allowed = violation(0.0) + FIT_VIOLATION_TOLERANCE_PX            # the center is the reference
    requested = position(1.0)
    if violation(1.0) <= allowed:
        kept = 1.0
    else:
        low, high = 0.0, 1.0                  # acceptable at low, not at high
        while high - low > FIELD_PULL_TOLERANCE:
            middle = (low + high) / 2.0
            if violation(middle) <= allowed:
                low = middle
            else:
                high = middle
        kept = low
    h, v = position(kept)
    return FieldPlacement(h_mm=h, v_mm=v, requested_h_mm=requested[0], requested_v_mm=requested[1],
                          fraction_kept=kept, violation_px=violation(kept))


def _placement_notes(placement: FieldPlacement) -> dict[str, Any]:
    """Notes of a pose placed at a field position: the position and the achieved fraction of the requested offset
    (``FIELD_FRACTION_ACHIEVED_KEY``: 1 when the request fits, smaller when the pose was pulled inward) always, the
    adjustment record when one was made."""
    notes: dict[str, Any] = {"field_h_mm": placement.h_mm, "field_v_mm": placement.v_mm,
                             FIELD_FRACTION_ACHIEVED_KEY: placement.fraction_kept}
    if placement.adjusted or not placement.fits:
        notes["field_placement"] = placement.as_notes()
    return notes


def _warn_not_fitting(diagnostics: PlanDiagnostics | None, series: str, target: TwoPlaneTarget, station_z_mm: float,
                      placement: FieldPlacement) -> None:
    """Warn that a target does not fit the field at its final position (which is then as good as the center)."""
    if not placement.fits:
        gap = "no back plate" if target.gap_mm is None else f"G = {target.gap_mm:g} mm"
        _warn(diagnostics, f"{series}: {target.target_id} ({gap}) does not fit the field of view at Z = "
                           f"{station_z_mm:g} mm even when centered (short by {placement.violation_px:.0f} px at the "
                           "edge margin); off-axis stations are placed no worse than the center, and plate edges "
                           "or features may be cut off")


# ---------------------------------------------------------------------------
# Section 4, Step 6: registration poses
# ---------------------------------------------------------------------------
def plan_registration(params: CharacterizationParameters, geometry: SensorGeometry, rng: np.random.Generator,
                      targets: TargetSet | None = None,
                      diagnostics: PlanDiagnostics | None = None) -> list[PlannedCapture]:
    """Section 4, Step 6: "Command REGISTRATION_POSES poses that span Z_MIN to Z_MAX, cover the
    field of view, and tilt within REGISTRATION_TILT_RANGE_DEG about H and V. At each pose
    capture REGISTRATION_FRAMES frames." Target: the noise plate T2 (redesign note, Section 3: registration is the
    plane-only hand-eye solve, ``geometry.registration.solve_from_planes``, which needs no pattern on the plate).

    The depths are REGISTRATION_POSES values evenly spaced from Z_MIN to Z_MAX, assigned to the
    poses in random order; the lateral field fractions and the two tilts come from a scrambled
    Halton sequence (low discrepancy, so the field and the tilt range are covered evenly without
    the clumps of a pure random draw). A pose that does not fit the field is pulled inward as in
    Section 5, Step 1. Poses are ordered by depth to keep robot travel short. ``seed`` is that of
    the design draw (shared by all registration poses)."""
    from scipy.stats import qmc

    plate = _targets(params, geometry, targets).get(TARGET_NOISE_PLATE)
    count = params.registration_poses
    seed = derive_seed(rng)
    generator = np.random.default_rng(seed)
    unit = qmc.Halton(d=REGISTRATION_SAMPLER_DIMENSIONS, scramble=True, seed=generator).random(count)
    depths = np.linspace(params.z_min_mm, params.z_max_mm, count) if count > 1 else np.array([params.z_min_mm])
    generator.shuffle(depths)
    fractions = 2.0 * unit[:, :REGISTRATION_FIELD_DIMENSIONS] - 1.0                                   # field direction in [-1, 1]
    tilts = (2.0 * unit[:, REGISTRATION_FIELD_DIMENSIONS:] - 1.0) * params.registration_tilt_range_deg  # about H, about V (degrees)
    margin = _fit_margin_px(params, jittered=False)
    entries = []
    for i in range(count):
        rotation = two_axis_tilt_rotation(float(tilts[i, 0]), float(tilts[i, 1]))
        placement = place_in_field(params, geometry, plate, float(depths[i]), (float(fractions[i, 0]),
                                   float(fractions[i, 1])), margin, rotation)
        entries.append((float(depths[i]), placement, rotation, float(tilts[i, 0]), float(tilts[i, 1])))
    entries.sort(key=lambda entry: entry[0])
    counter = _PoseCounter()
    plan = []
    for depth, placement, rotation, tilt_h, tilt_v in entries:
        pose = RigidTransform(rotation, [placement.h_mm, placement.v_mm, depth])
        notes = _placement_notes(placement)
        notes.update({"depth_mm": depth, "tilt_h_deg": tilt_h, "tilt_v_deg": tilt_v})
        angle = float(np.degrees(np.linalg.norm(Rotation.from_matrix(rotation).as_rotvec())))
        plan.append(_new_capture(counter, PROCEDURE_REGISTRATION, TARGET_NOISE_PLATE, None, depth,
                                 FIELD_POSITION_CENTER, params.frames_per_registration_pose, SUBSERIES_MAIN, pose,
                                 seed=seed, tilt_axis=TILT_AXIS_BOTH, tilt_deg=angle, notes=notes))
        _warn_not_fitting(diagnostics, "R", plate, depth, placement)
    return _renumber(plan)


# ---------------------------------------------------------------------------
# Section 5: noise series
# ---------------------------------------------------------------------------
def tilt_near_edge_mm(plate: TwoPlaneTarget, station_z_mm: float, tilt_axis: str, tilt_deg: float) -> float:
    """Depth of the near edge of the plate when it is tilted by ``tilt_deg`` about ``tilt_axis`` through its center at
    ``station_z_mm``: Z - h sin(|tilt|), where h is the half extent of the plate across the tilt axis (the edge that swings
    toward the sensor: half the width for a tilt about V, half the height for a tilt about H; both are 200 mm for the
    400 x 400 mm plate)."""
    half_extent = plate.half_width_mm if tilt_axis == TILT_AXIS_V else plate.half_height_mm
    return station_z_mm - half_extent * math.sin(math.radians(abs(tilt_deg)))


def tilt_is_feasible(params: CharacterizationParameters, plate: TwoPlaneTarget, station_z_mm: float, tilt_axis: str,
                     tilt_deg: float) -> bool:
    """Whether a tilted plate keeps its near edge at or beyond Z_MIN (Section 5, Step 5, tilt feasibility): Z - h sin(tilt)
    >= Z_MIN, so that no part of the plate comes closer to the sensor than the nearest depth the sensor is specified to
    read. With the 400 x 400 mm plate this rules out every tilt at Z = 400 mm and none at 800 and 1600 mm."""
    return tilt_near_edge_mm(plate, station_z_mm, tilt_axis, tilt_deg) >= params.z_min_mm - TILT_NEAR_EDGE_TOLERANCE_MM


def _feasible_tilt_angles(params: CharacterizationParameters, plate: TwoPlaneTarget, station_z_mm: float,
                          tilt_axis: str, diagnostics: PlanDiagnostics | None) -> list[float]:
    """The angles of TILT_ANGLES_DEG of one tilt sweep (station and axis) that are planned. An infeasible angle is
    skipped. When no angle other than zero is feasible the whole sweep is skipped: the zero angle alone is no sweep (the
    fronto-parallel pose is the main station already). Every skipped angle is listed in ``diagnostics`` with the reason."""
    angles = [float(a) for a in params.tilt_angles_deg]
    feasible = [a for a in angles if tilt_is_feasible(params, plate, station_z_mm, tilt_axis, a)]
    if not any(a != 0.0 for a in feasible):
        feasible = []                                                   # no real tilt is allowed: no sweep at all
    skipped = [a for a in angles if a not in feasible]
    if skipped and diagnostics is not None:
        near_edges = ", ".join(f"{a:g} deg -> {tilt_near_edge_mm(plate, station_z_mm, tilt_axis, a):.0f} mm"
                               for a in skipped if a != 0.0)
        reason = (f"the plate's near edge, Z - h sin(tilt) with h = half the plate extent across the tilt axis = "
                  f"{plate.half_width_mm if tilt_axis == TILT_AXIS_V else plate.half_height_mm:g} mm, would come closer than "
                  f"Z_MIN = {params.z_min_mm:g} mm ({near_edges})")
        if not feasible:
            reason += "; with no tilt left, the zero-angle pose alone is not a sweep either"
        diagnostics.skip(f"A tilt about {tilt_axis} at Z = {station_z_mm:g} mm: skipped "
                         f"{', '.join(f'{a:g}' for a in skipped)} deg; {reason}")
    return feasible


def plan_noise_series(params: CharacterizationParameters, geometry: SensorGeometry, rng: np.random.Generator,
                      filters_off: bool = False, with_sentinels: bool = True, targets: TargetSet | None = None,
                      diagnostics: PlanDiagnostics | None = None) -> list[PlannedCapture]:
    """Section 5 (Series A), target T2: main stations, tilt sub-series and repeat-mount check.

    Step 1: the station list is every station of the geometric ladder (``params.z_stations_mm()``, nine
    stations from Z_MIN to Z_MAX) at the five field positions (center, then the four off-axis ones), plus the
    LEGACY_METRIC_DEPTHS_MM (700 and 1000 mm, ``params.legacy_extra_stations_mm()``) as extra stations at the center
    field position ONLY, so the legacy metrics are computed at the depths of the existing data without adding off-axis
    poses: 9 x 5 + 2 = 47 main poses. All are fronto-parallel, FRAMES_PER_NOISE_STATION frames each; off-axis positions
    that do not fit the field are pulled inward (see module docstring) and the achieved fraction of the requested
    offset is stored in ``notes["field_fraction_achieved"]``.
    Step 2: the station list is shuffled with a logged seed. Step 4 (move, settle, capture) is the
    robot's job; the budget counts the settle time. Step 5: at the center and each Z in
    reduced station (``params.z_reduced_stations_mm()``), T2 is tilted about V and then about H through every angle in
    TILT_ANGLES_DEG (FRAMES_PER_TILT_POSE frames each; the zero angle is captured in both
    sweeps, as the procedure lists it). A tilt is planned only where the plate's near edge stays at or beyond Z_MIN
    (:func:`tilt_is_feasible`: Z - h sin(tilt) >= Z_MIN, h the half extent of the plate across the tilt axis); infeasible
    tilts are skipped and listed in plan_summary.txt with the reason. With the 400 x 400 mm plate every tilt at 400 mm is
    infeasible, which leaves the 800 and 1600 mm stations. Step 6: the repeat-mount check repeats the center station at
    Z_REFERENCE_MM after the tilt sub-series (subseries "remount"). Step 3: the drift sentinels
    are inserted by :func:`insert_sentinels` when ``with_sentinels`` is true.

    Step 7 (``filters_off=True``): returns the filters-off repeat of Steps 1-5 instead (all of
    its poses carry the sub-series "filters_off", with pose indices from
    FILTERS_OFF_POSE_INDEX_BASE; tilt poses are the ones with a tilt axis), without a re-mount
    check, so that a full session appends it to the filters-on series."""
    plate = _targets(params, geometry, targets).get(TARGET_NOISE_PLATE)
    margin = _fit_margin_px(params, jittered=False)
    counter = _PoseCounter(FILTERS_OFF_POSE_INDEX_BASE if filters_off else 0)
    main_label = SUBSERIES_FILTERS_OFF if filters_off else SUBSERIES_MAIN
    # Step 1: the station list with its field placements.
    stations = [(z, code) for z in params.z_stations_mm() for code in FIELD_POSITION_CODES]
    stations += [(z, FIELD_POSITION_CENTER) for z in params.legacy_extra_stations_mm()]      # legacy depths: center only
    # Step 2: shuffle with a logged seed so slow drift cannot masquerade as a Z dependence.
    order_seed = derive_seed(rng)
    permutation = np.random.default_rng(order_seed).permutation(len(stations))
    plan: list[PlannedCapture] = []
    for station_index in permutation:
        z, code = stations[int(station_index)]
        signs = FIELD_POSITION_SIGNS.get(code, (0.0, 0.0))
        placement = place_in_field(params, geometry, plate, z, signs, margin)
        notes = _placement_notes(placement)
        notes["order_seed"] = order_seed
        plan.append(_new_capture(counter, PROCEDURE_NOISE, TARGET_NOISE_PLATE, None, z, code,
                                 params.frames_per_noise_station, main_label,
                                 fronto_parallel_pose(placement.h_mm, placement.v_mm, z), seed=order_seed,
                                 notes=notes))
        _warn_not_fitting(diagnostics, "A", plate, z, placement)
    # Step 5: tilt sub-series at the center, about V and then about H.
    camera = camera_of(geometry)
    for z in params.z_reduced_stations_mm():
        for axis in (TILT_AXIS_V, TILT_AXIS_H):
            for angle in _feasible_tilt_angles(params, plate, z, axis, diagnostics):
                pose = tilted_pose(0.0, 0.0, z, axis, angle)
                notes = {"near_edge_mm": tilt_near_edge_mm(plate, z, axis, angle)}
                plan.append(_new_capture(counter, PROCEDURE_NOISE, TARGET_NOISE_PLATE, None, z, FIELD_POSITION_CENTER,
                                         params.frames_per_tilt_pose, SUBSERIES_FILTERS_OFF if filters_off
                                         else SUBSERIES_TILT, pose, tilt_axis=axis, tilt_deg=angle, notes=notes))
                if fit_violation_px(camera, plate, pose, margin) > fit_violation_px(
                        camera, plate, fronto_parallel_pose(0.0, 0.0, z), margin) + FIT_VIOLATION_TOLERANCE_PX:
                    _warn(diagnostics, f"A: the T2 plate tilted {angle:g} deg about {axis} at Z = {z:g} mm fits the "
                                       "field of view worse than the untilted plate; check that it covers the "
                                       "analysis region")
    # Step 6: repeat-mount check (not part of the filters-off repeat, which covers Steps 1-5).
    if not filters_off:
        z = params.z_reference_mm
        plan.append(_new_capture(counter, PROCEDURE_NOISE, TARGET_NOISE_PLATE, None, z, FIELD_POSITION_CENTER,
                                 params.frames_per_noise_station, SUBSERIES_REMOUNT,
                                 fronto_parallel_pose(0.0, 0.0, z)))
    _renumber(plan)
    if with_sentinels:
        plan = insert_sentinels(plan, params, geometry, _seconds_per_frame(geometry), params.move_and_settle_time_s,
                                diagnostics=diagnostics, targets=targets)
    return plan


# ---------------------------------------------------------------------------
# Section 6.1: edge series
# ---------------------------------------------------------------------------
def plan_edge_series(params: CharacterizationParameters, geometry: SensorGeometry, rng: np.random.Generator,
                     targets: TargetSet | None = None,
                     diagnostics: PlanDiagnostics | None = None,
                     filters_off: bool = False, lateral_sweep: bool = False) -> list[PlannedCapture]:
    """Section 6.1 (Series B-HV): T3a (raised square) then T3b (square window), each with G =
    GAP_SMALL_MM and then GAP_LARGE_MM. The slant of the square is part of the target definition.

    At each shape station (``params.z_shape_stations_mm()``), centered and fronto-parallel: Step 2, FRAMES_PER_EDGE_POSE
    frames at the nominal pose (subseries "nominal"); Step 3, PHASE_JITTER_POSES_EDGE further poses,
    each with its own logged random lateral offset uniform over +/- PHASE_JITTER_SPAN_PX / 2 in both
    H and V (subseries "jitter"), FRAMES_PER_EDGE_POSE frames each. Steps 4 and 5 are the loop over the
    two gaps and the two targets; stations run in ascending Z.

    Optional second pass, the lateral sweep (``lateral_sweep=True``; default off; outside the Section 9 budget, as the
    filters-off repeat and the staircase): for the edge target already mounted (T3a, the raised square, with
    GAP_SMALL_MM) at the reference station Z_REFERENCE_MM, a systematic sweep of the lateral position, first in H and then
    in V, in steps of LATERAL_SWEEP_STEP_PX over LATERAL_SWEEP_SPAN_PX: the offsets from the nominal position are
    k x step for k = 1 ... span / step (20 per axis, 40 poses, ``params.lateral_sweep_positions_px``; the origin is the
    nominal B pose already captured and is not repeated). Each step is converted to millimeters at the reference station with the pixel pitch p(Z) = Z / f_x, so the step is 0.1 px x p(Z_REFERENCE_MM)
    (0.116 mm at the indicative geometry). Each pose has FRAMES_PER_EDGE_POSE frames and the sub-series "lateral_sweep";
    the pose indices start at LATERAL_SWEEP_POSE_INDEX_BASE (a four-digit range of their own, so that they cannot be
    confused with the main poses of the same configuration), the sweep is placed right after the stations of T3a with the small gap, while that target is still mounted, and the
    sweep axis, position index and offset in pixels are in ``notes``. The approach direction ALTERNATES on purpose, so
    that lateral hysteresis shows (Section 6.1, Step 6): the odd-numbered poses (1st, 3rd, ...) of an axis are approached
    from the negative side (``notes["approach_direction"]`` is "-H" or "-V", the robot arrives moving toward positive
    offsets) and the even-numbered poses from the positive side ("+H" or "+V"); this is unlike the from-below rule of
    series Z. The sweep is not repeated in the filters-off pass.

    Section 4, Step 4.2 (``filters_off=True``): returns the filters-off repeat of the whole series instead,
    with the same poses (new logged jitter offsets) and the sub-series "filters_off" on every one, and pose
    indices from FILTERS_OFF_POSE_INDEX_BASE, as the A repeat of :func:`plan_noise_series`."""
    target_set = _targets(params, geometry, targets)
    counter = _PoseCounter(FILTERS_OFF_POSE_INDEX_BASE if filters_off else 0)
    nominal_label = SUBSERIES_FILTERS_OFF if filters_off else SUBSERIES_NOMINAL
    jitter_label = SUBSERIES_FILTERS_OFF if filters_off else SUBSERIES_JITTER
    sweep_counter = _PoseCounter(LATERAL_SWEEP_POSE_INDEX_BASE)       # the optional lateral sweep's own index range
    camera = camera_of(geometry)
    plan: list[PlannedCapture] = []
    for target_id in (TARGET_RAISED_SQUARE, TARGET_SQUARE_WINDOW):
        for gap in (params.gap_small_mm, params.gap_large_mm):
            target = target_set.get(target_id).with_gap(gap)
            for z in params.z_shape_stations_mm():
                nominal = fronto_parallel_pose(0.0, 0.0, z)
                if not pose_fits_field(camera, target, nominal, _fit_margin_px(params, jittered=True)):
                    _warn(diagnostics, f"B: {target_id} (G = {gap:g} mm) does not fit the field of view at Z = "
                                       f"{z:g} mm with the phase-jitter margin; features may be cut off")
                plan.append(_new_capture(counter, PROCEDURE_EDGES, target_id, gap, z, FIELD_POSITION_CENTER,
                                         params.frames_per_edge_pose, nominal_label, nominal))
                for _ in range(params.phase_jitter_poses_edge):
                    plan.append(_jitter_capture(counter, params, geometry, rng, PROCEDURE_EDGES, target_id, gap, z,
                                                FIELD_POSITION_CENTER, params.frames_per_edge_pose,
                                                jitter_label, 0.0, 0.0))
            # Optional second pass: the lateral sweep, right after the stations of T3a with the small gap, while that
            # target is still mounted (Section 6.1, Step 6: "where the target is already mounted").
            if (lateral_sweep and not filters_off and target_id == TARGET_RAISED_SQUARE
                    and gap == params.gap_small_mm):
                plan += _plan_lateral_sweep(sweep_counter, params, geometry, diagnostics)
    return _renumber(plan)


LATERAL_SWEEP_AXES = ("H", "V")
"""The axes of the lateral sweep, in the order they are captured: H first, then V."""
LATERAL_SWEEP_APPROACH_PERIOD = 2
"""The approach direction of the lateral sweep alternates with this period: the 1st, 3rd, ... pose of an axis from the
negative side, the 2nd, 4th, ... from the positive side."""


def _plan_lateral_sweep(counter: _PoseCounter, params: CharacterizationParameters, geometry: SensorGeometry,
                        diagnostics: PlanDiagnostics | None) -> list[PlannedCapture]:
    """The poses of the optional lateral sweep (Section 6.1, Step 6; see :func:`plan_edge_series`): T3a with
    GAP_SMALL_MM at Z_REFERENCE_MM, LATERAL_SWEEP_SPAN_PX swept in steps of LATERAL_SWEEP_STEP_PX in H and then in V."""
    z_ref = params.z_reference_mm
    pitch_mm = geometry.pixel_footprint_mm(z_ref)                    # p(Z) = Z / f_x
    positions_px = params.lateral_sweep_positions_px()
    if diagnostics is not None:
        diagnostics.note(f"B-HV lateral sweep (optional) of {TARGET_RAISED_SQUARE} (G = {params.gap_small_mm:g} mm) at "
                         f"Z = {z_ref:g} mm: {len(positions_px)} poses per axis in H and then in V at offsets "
                         f"k x {params.lateral_sweep_step_px:g} px = k x {params.lateral_sweep_step_px * pitch_mm:.4f} mm, "
                         f"k = 1 to {len(positions_px)} (span {params.lateral_sweep_span_px:g} px; the nominal pose is "
                         f"already captured), {params.frames_per_edge_pose} frames each, outside the main budget")
    plan: list[PlannedCapture] = []
    for axis in LATERAL_SWEEP_AXES:
        for index, position_px in enumerate(positions_px):
            offset_mm = position_px * pitch_mm
            offset = (offset_mm, 0.0) if axis == "H" else (0.0, offset_mm)
            pose = fronto_parallel_pose(offset[0], offset[1], z_ref)
            # Alternating approach (Section 6.1, Step 6): odd-numbered poses (1st, 3rd, ...) from the negative side,
            # even-numbered from the positive side of the axis being swept.
            from_negative = index % LATERAL_SWEEP_APPROACH_PERIOD == 0
            direction = ("-" if from_negative else "+") + axis
            plan.append(_new_capture(
                counter, PROCEDURE_EDGES, TARGET_RAISED_SQUARE, params.gap_small_mm, z_ref, FIELD_POSITION_CENTER,
                params.frames_per_edge_pose, SUBSERIES_LATERAL_SWEEP, pose, offset=offset,
                notes={"lateral_sweep_axis": axis, "lateral_sweep_index": index, "lateral_sweep_offset_px": position_px,
                       "lateral_sweep_step_px": params.lateral_sweep_step_px, "pixel_pitch_mm": pitch_mm,
                       APPROACH_DIRECTION_KEY: direction}))
    return plan


def _jitter_capture(counter: _PoseCounter, params: CharacterizationParameters, geometry: SensorGeometry,
                    rng: np.random.Generator, procedure: str, target_id: str, gap_mm: float | None, station_z_mm: float,
                    field_code: int, frames: int, subseries: str, base_h_mm: float, base_v_mm: float,
                    notes: dict[str, Any] | None = None) -> PlannedCapture:
    """A fronto-parallel pose at (base_h, base_v, Z) plus a logged random lateral offset (own seed)."""
    seed = derive_seed(rng)
    offset = jitter_offset_mm(seed, params, geometry, station_z_mm)
    pose = fronto_parallel_pose(base_h_mm + offset[0], base_v_mm + offset[1], station_z_mm)
    return _new_capture(counter, procedure, target_id, gap_mm, station_z_mm, field_code, frames, subseries, pose,
                        seed=seed, offset=offset, notes=dict(notes or {}))


# ---------------------------------------------------------------------------
# Section 6.2: Z-step series
# ---------------------------------------------------------------------------
def default_expected_quantum_mm(geometry: SensorGeometry,
                                disparity_quantum_px: float = TIER_A_DISPARITY_QUANTUM_PX) -> Callable[[float], float]:
    """The expected depth quantum dZ_q(Z) = q Z^2 / k (Section 2) with the disparity quantum q (default: the Tier-A
    value; the planner passes ``params.tier_a_disparity_quantum_px``), as a function of the station depth in mm."""
    return lambda depth_mm: geometry.depth_quantum_mm(disparity_quantum_px, depth_mm)


def z_step_rungs_mm(params: CharacterizationParameters, expected_quantum_mm: float) -> list[float]:
    """The commanded rungs of the step ladder at one station, mm, ascending (Section 6.2, Step 2): each multiple in
    ``params.z_step_ladder_quanta`` of the expected depth quantum there, raised to at least
    ``params.robot_min_resolvable_move_mm`` (the smallest Z move the robot is trusted to execute). Two rungs that the
    floor makes equal are one rung (the ladder repeats a rung only once)."""
    raw = [max(float(q) * float(expected_quantum_mm), params.robot_min_resolvable_move_mm)
           for q in params.z_step_ladder_quanta]
    return sorted(set(raw))


def ramp_visible_height_mm(geometry: SensorGeometry, plate: TwoPlaneTarget, station_z_mm: float) -> float:
    """Height of the part of the plate the sensor sees at the station, mm: the smaller of the plate height and the field
    height at that Z (the plate fills the image at 400 mm and is smaller than the image from about 550 mm on)."""
    field_height = 2.0 * geometry.half_field_mm(station_z_mm)[1]
    return min(2.0 * plate.half_height_mm, field_height)


def ramp_tilt_deg(params: CharacterizationParameters, geometry: SensorGeometry, plate: TwoPlaneTarget,
                  station_z_mm: float, expected_quantum_mm: float) -> float:
    """Tilt about H, degrees, that makes the true depth across the visible height of the plate span
    ``params.ramp_quanta`` expected quanta: sin(tilt) = RAMP_QUANTA dZ_q / visible height (Section 6.2, Step 4). The tilt
    axis is parallel to the baseline (H, image columns), so every image row lies at one true depth. Raises ValueError
    when the wanted span exceeds the visible height (no tilt can give it)."""
    span_mm = params.ramp_quanta * expected_quantum_mm
    visible_mm = ramp_visible_height_mm(geometry, plate, station_z_mm)
    if span_mm >= visible_mm:
        raise ValueError(f"the ramp span {span_mm:.1f} mm ({params.ramp_quanta:g} quanta of {expected_quantum_mm:.2f} mm) "
                         f"is not less than the visible plate height {visible_mm:.1f} mm at Z = {station_z_mm:g} mm")
    return math.degrees(math.asin(span_mm / visible_mm))


def ramp_pose(params: CharacterizationParameters, geometry: SensorGeometry, plate: TwoPlaneTarget,
              station_z_mm: float, expected_quantum_mm: float, tilt_deg: float) -> tuple[RigidTransform, dict[str, Any]]:
    """The target pose of the ramp at a station and the notes that describe it: the plate centered, tilted about H by
    ``tilt_deg`` (:func:`ramp_tilt_deg`). The tilt-feasibility rule (:func:`tilt_is_feasible`) applies: when the near edge
    would come closer than Z_MIN, the plate center is moved farther by exactly the shortfall so that the near edge sits at
    Z_MIN (``ramp_center_shift_mm``, 0 when no move was needed); the station label stays Z0. Shared by the planner and by
    the demonstration plan of the simulator."""
    shift = 0.0
    if not tilt_is_feasible(params, plate, station_z_mm, TILT_AXIS_H, tilt_deg):
        shift = params.z_min_mm - tilt_near_edge_mm(plate, station_z_mm, TILT_AXIS_H, tilt_deg)
    center_z = station_z_mm + shift
    notes = {"expected_quantum_mm": expected_quantum_mm, "ramp_span_mm": params.ramp_quanta * expected_quantum_mm,
             "ramp_visible_height_mm": ramp_visible_height_mm(geometry, plate, station_z_mm),
             "ramp_center_shift_mm": shift,
             "near_edge_mm": tilt_near_edge_mm(plate, center_z, TILT_AXIS_H, tilt_deg),
             "far_edge_mm": center_z + plate.half_height_mm * math.sin(math.radians(tilt_deg))}
    return tilted_pose(0.0, 0.0, center_z, TILT_AXIS_H, tilt_deg), notes


def staircase_step_mm(params: CharacterizationParameters, expected_quantum_mm: float) -> float:
    """Step of the optional staircase, mm: one ``z_staircase_subdivision``-th of the expected quantum, never below
    ``params.robot_min_resolvable_move_mm``."""
    return max(expected_quantum_mm / params.z_staircase_subdivision, params.robot_min_resolvable_move_mm)


def _quantum_function(params: CharacterizationParameters, geometry: SensorGeometry,
                      expected_quantum_mm: Callable[[float], float] | float | None) -> Callable[[float], float]:
    """The expected depth quantum as a function of the station depth: the given function, the given constant (mm) or,
    by default, the one of the Tier-A disparity quantum ``params.tier_a_disparity_quantum_px``
    (:func:`default_expected_quantum_mm`), which a parameters.json override can replace by a measured value."""
    if expected_quantum_mm is None:
        return default_expected_quantum_mm(geometry, params.tier_a_disparity_quantum_px)
    if callable(expected_quantum_mm):
        return expected_quantum_mm
    constant = float(expected_quantum_mm)
    return lambda depth_mm: constant


def plan_zstep_series(params: CharacterizationParameters, geometry: SensorGeometry, rng: np.random.Generator,
                      expected_quantum_mm: Callable[[float], float] | float | None = None,
                      targets: TargetSet | None = None,
                      diagnostics: PlanDiagnostics | None = None,
                      filters_off: bool = False, staircase: bool = False) -> list[PlannedCapture]:
    """Section 6.2 (Series B-Z, procedure letter Z), target T2 fronto-parallel (ladder, staircase) or tilted about H (ramp).

    The expected depth quantum at a station is dZ_q(Z0) = q Z0^2 / k from ``expected_quantum_mm`` (a function of Z, or one
    number in mm; default: the Tier-A disparity quantum, ``params.tier_a_disparity_quantum_px`` (default
    TIER_A_DISPARITY_QUANTUM_PX), until Analysis A or the ramp has measured one and it is passed in the parameters.json).

    Step 2, step ladder, at the reduced stations (``params.z_reduced_stations_mm()``), centered: the rungs of a station are
    the multiples Z_STEP_LADDER_QUANTA of dZ_q(Z0), each raised to at least ROBOT_MIN_RESOLVABLE_MOVE_MM
    (:func:`z_step_rungs_mm`; at the indicative geometry 0.1 to 3.1 mm at 400 mm, 0.39 to 12.4 mm at 800 mm, 1.55 to 50 mm at
    1600 mm, listed per station in plan_summary.txt). For each rung delta the target alternates between Z0 and Z0 + delta
    for Z_STEP_REPEATS cycles (A, B, A, B, ...). Each visit is its own pose with FRAMES_PER_ZSTEP_POSE frames, sub-series
    "ladder", visit "A" or "B" and step_mm = delta for both visits; the commanded displacement is 0 for A and delta for B
    (``notes["displacement_mm"]``, and the target pose carries it in its Z). The ground truth of the step is the read-back
    robot pose carried into the camera frame (the analysis takes the difference of the registered front-plane depth of
    the two visits). Every visit of series Z is approached from below (``notes["approach"]``): the robot backs off by
    ``params.z_step_approach_overshoot_mm`` toward smaller Z and then moves up onto the pose, so backlash does not enter
    the A / B difference. Step 4: the Z0 visits of the ladder serve as the no-step reference, so no extra poses are planned.

    Ramp, at EVERY station of the ladder (``params.z_stations_mm()``), sub-series "ramp": one pose of T2 tilted about H (the
    tilt axis parallel to the baseline, so each image row lies at one true depth) by the angle that makes the true depth
    across the plate's visible height span RAMP_QUANTA expected quanta (:func:`ramp_tilt_deg`), FRAMES_PER_RAMP_POSE frames.
    The tilt is stored in ``tilt_deg`` and printed per station in plan_summary.txt. The tilt-feasibility rule of the A tilt
    sub-series applies (:func:`tilt_is_feasible`: the plate's near edge, Z - h sin(tilt), stays at or beyond Z_MIN). The
    ramp tilts are small, but the plate is 400 mm high, so at Z0 = Z_MIN the near edge would be a millimeter closer than
    Z_MIN; there (and wherever the rule would fail) the plate center is moved farther by exactly the shortfall so that the
    near edge sits at Z_MIN (``notes["ramp_center_shift_mm"]``, noted in plan_summary.txt); the station stays Z0. A station
    whose ramp is geometrically impossible (span not less than the visible height) is skipped and listed with the reason.

    Optional second pass, the staircase (``staircase=True``; default off; outside the Section 9 budget, as the filters-off
    repeat), at the reduced stations: from Z0 to Z0 + Z_STAIRCASE_QUANTA x dZ_q in steps of max(dZ_q /
    Z_STAIRCASE_SUBDIVISION, ROBOT_MIN_RESOLVABLE_MOVE_MM) (:func:`staircase_step_mm`; both ends included, the last step
    the one nearest the span), Z_STAIRCASE_FRAMES frames per step, sub-series "staircase", step_mm = the displacement from Z0.

    Section 4, Step 4.2 (``filters_off=True``): returns the filters-off repeat of the whole series instead (the ladder, the
    ramp and, when asked for, the staircase; visit and step_mm are kept), with the sub-series "filters_off" on every pose and
    pose indices from FILTERS_OFF_POSE_INDEX_BASE, as the A repeat of :func:`plan_noise_series`. The staircase of the
    filters-on pass has its own pose indices from STAIRCASE_POSE_INDEX_BASE.

    No random draws occur in this series; ``rng`` is accepted so all planners share one signature."""
    del rng                                # no randomness needed here
    plate = _targets(params, geometry, targets).get(TARGET_NOISE_PLATE)
    margin = _fit_margin_px(params, jittered=False)
    camera = camera_of(geometry)
    ladder_label = SUBSERIES_FILTERS_OFF if filters_off else SUBSERIES_LADDER
    ramp_label = SUBSERIES_FILTERS_OFF if filters_off else SUBSERIES_RAMP
    staircase_label = SUBSERIES_FILTERS_OFF if filters_off else SUBSERIES_STAIRCASE
    quantum_at = _quantum_function(params, geometry, expected_quantum_mm)
    approach = {"approach": APPROACH_FROM_BELOW, "approach_overshoot_mm": params.z_step_approach_overshoot_mm}
    counter = _PoseCounter(FILTERS_OFF_POSE_INDEX_BASE if filters_off else 0)
    plan: list[PlannedCapture] = []
    # Step 2: the step ladder at the reduced stations.
    for z0 in params.z_reduced_stations_mm():
        quantum = float(quantum_at(z0))
        rungs = z_step_rungs_mm(params, quantum)
        if diagnostics is not None:
            diagnostics.note(f"B-Z ladder at Z0 = {z0:g} mm: expected depth quantum {quantum:.3f} mm "
                             f"(Tier-A q = {params.tier_a_disparity_quantum_px:g} px unless a measured value was given), rungs "
                             + ", ".join(f"{r:.3g}" for r in rungs) + " mm")
            if z0 + rungs[-1] > params.z_max_mm:
                diagnostics.note(f"B-Z ladder at Z0 = {z0:g} mm: the largest rung ({rungs[-1]:.3g} mm) moves the plate to "
                                 f"{z0 + rungs[-1]:.0f} mm, beyond Z_MAX = {params.z_max_mm:g} mm")
        for delta in rungs:
            for _ in range(params.z_step_repeats):
                for visit, displacement in ((VISIT_A, 0.0), (VISIT_B, float(delta))):
                    plan.append(_new_capture(
                        counter, PROCEDURE_ZSTEP, TARGET_NOISE_PLATE, None, z0, FIELD_POSITION_CENTER,
                        params.frames_per_zstep_pose, ladder_label,
                        fronto_parallel_pose(0.0, 0.0, z0 + displacement), step_mm=float(delta), visit=visit,
                        notes={"displacement_mm": displacement, "expected_quantum_mm": quantum, **approach}))
    # Ramp: one tilted pose at every station of the ladder.
    for z0 in params.z_stations_mm():
        quantum = float(quantum_at(z0))
        try:
            tilt = ramp_tilt_deg(params, geometry, plate, z0, quantum)
        except ValueError as error:
            if diagnostics is not None:
                diagnostics.skip(f"B-Z ramp at Z0 = {z0:g} mm: skipped; {error}")
            continue
        pose, notes = ramp_pose(params, geometry, plate, z0, quantum, tilt)
        shift = notes["ramp_center_shift_mm"]
        if shift > 0.0 and diagnostics is not None:
            diagnostics.note(
                f"B-Z ramp at Z0 = {z0:g} mm: the plate's near edge would be at "
                f"{tilt_near_edge_mm(plate, z0, TILT_AXIS_H, tilt):.2f} mm, closer than Z_MIN = {params.z_min_mm:g} mm "
                f"(tilt {tilt:.2f} deg, half plate height {plate.half_height_mm:g} mm); the plate center is moved "
                f"{shift:.2f} mm farther so that the near edge sits at Z_MIN")
        notes.update(approach)
        plan.append(_new_capture(counter, PROCEDURE_ZSTEP, TARGET_NOISE_PLATE, None, z0, FIELD_POSITION_CENTER,
                                 params.frames_per_ramp_pose, ramp_label, pose, tilt_axis=TILT_AXIS_H, tilt_deg=tilt,
                                 notes=notes))
        if fit_violation_px(camera, plate, pose, margin) > fit_violation_px(
                camera, plate, fronto_parallel_pose(0.0, 0.0, z0), margin) + RAMP_FIT_WORSENING_TOLERANCE_PX:
            _warn(diagnostics, f"B-Z: the T2 plate tilted {tilt:.2f} deg about H at Z = {z0:g} mm (ramp) fits the field "
                               "of view worse than the untilted plate; check that it covers the analysis region")
    # Optional second pass: the staircase. Outside the budget, it has its own four-digit pose-index range (the filters-off
    # repeat keeps its own range for all of its poses, the staircase included).
    staircase_counter = counter if filters_off else _PoseCounter(STAIRCASE_POSE_INDEX_BASE)
    if staircase:
        for z0 in params.z_reduced_stations_mm():
            quantum = float(quantum_at(z0))
            step = staircase_step_mm(params, quantum)
            steps = int(round(params.z_staircase_quanta * quantum / step))
            if diagnostics is not None:
                raised = " (raised to ROBOT_MIN_RESOLVABLE_MOVE_MM)" if step > quantum / params.z_staircase_subdivision else ""
                diagnostics.note(f"B-Z staircase (optional) at Z0 = {z0:g} mm: expected depth quantum {quantum:.3f} mm, "
                                 f"{steps + 1} steps of {step:.4f} mm{raised}, {quantum / step:.1f} steps per quantum")
            for k in range(steps + 1):
                displacement = k * step
                plan.append(_new_capture(
                    staircase_counter, PROCEDURE_ZSTEP, TARGET_NOISE_PLATE, None, z0, FIELD_POSITION_CENTER,
                    params.z_staircase_frames, staircase_label, fronto_parallel_pose(0.0, 0.0, z0 + displacement),
                    step_mm=displacement, notes={"displacement_mm": displacement, "staircase_step": k,
                                                 "expected_quantum_mm": quantum, **approach}))
    return _renumber(plan)


# ---------------------------------------------------------------------------
# Section 7: area series
# ---------------------------------------------------------------------------
AREA_TARGET_ORDER = (TARGET_DISKS, TARGET_CUTOUTS)
"""The two feature plates of Sections 7 and 8, one mounting each (T4 disks, T5 cutouts)."""
CUTOUT_TARGETS = (TARGET_CUTOUTS,)
"""The cutout plate (T5), the one of the open-background variant."""


def plan_area_series(params: CharacterizationParameters, geometry: SensorGeometry, rng: np.random.Generator,
                     open_background: bool = False, targets: TargetSet | None = None,
                     diagnostics: PlanDiagnostics | None = None) -> list[PlannedCapture]:
    """Section 7 (Series C): the two feature plates, one mounting each, in the order T4, T5.

    Step 1: the configuration list is {T4, T5} x {GAP_SMALL_MM, GAP_LARGE_MM} x every station of the ladder
    (``params.z_stations_mm()``, all nine), centered and fronto-parallel; the three features of a plate are all on
    every pose, so each (feature, station) pair has PHASE_JITTER_POSES_AREA poses. The order within each mounting is randomized with a
    logged seed (``notes["order_seed"]``), so targets are re-mounted as rarely as possible.
    Step 2: each configuration gets PHASE_JITTER_POSES_AREA poses, each with its own logged random
    lateral offset uniform over +/- PHASE_JITTER_SPAN_PX / 2 in H and V and FRAMES_PER_AREA_POSE frames
    (sub-series "jitter"). Step 3, field sub-series: at Z = Z_REFERENCE_MM and G = GAP_SMALL_MM, each array
    at the four off-axis field positions, FIELD_SUBSERIES_POSES_AREA poses each (sub-series "field"; a
    position that does not fit is pulled inward, Section 5 Step 1; the position order is shuffled).
    Step 4, open-background variant (``open_background=True``, cutout arrays only): at Z = Z_REFERENCE_MM,
    back plate removed (gap None), Step 2 repeated (sub-series "open"). The variant is an optional set outside the Section 9
    budget, like the staircase: its poses have pose indices from OPEN_BACKGROUND_POSE_INDEX_BASE (a four-digit range of their
    own), and :func:`insert_sentinels` gives it a sentinel clock of its own. Step 5 (drift sentinels) is
    :func:`insert_sentinels`."""
    target_set = _targets(params, geometry, targets)
    counter = _PoseCounter()
    open_counter = _PoseCounter(OPEN_BACKGROUND_POSE_INDEX_BASE)      # the optional variant's own index range
    camera = camera_of(geometry)
    plan: list[PlannedCapture] = []
    z_ref = params.z_reference_mm
    for target_id in AREA_TARGET_ORDER:
        configurations = [(gap, z) for gap in (params.gap_small_mm, params.gap_large_mm)
                          for z in params.z_stations_mm()]
        order_seed = derive_seed(rng)
        shuffled = np.random.default_rng(order_seed).permutation(len(configurations))
        for position in shuffled:
            gap, z = configurations[int(position)]
            target = target_set.get(target_id).with_gap(gap)
            nominal = fronto_parallel_pose(0.0, 0.0, z)
            if not pose_fits_field(camera, target, nominal, _fit_margin_px(params, jittered=True)):
                _warn(diagnostics, f"C: {target_id} (G = {gap:g} mm) does not fit the field of view at Z = {z:g} mm "
                                   "with the phase-jitter margin; features may be cut off")
            for _ in range(params.phase_jitter_poses_area):
                plan.append(_jitter_capture(counter, params, geometry, rng, PROCEDURE_AREA, target_id, gap, z,
                                            FIELD_POSITION_CENTER, params.frames_per_area_pose, SUBSERIES_JITTER,
                                            0.0, 0.0, notes={"order_seed": order_seed}))
        # Step 3: the field sub-series of this mounting (G small, Z reference, the four off-axis positions).
        field_seed = derive_seed(rng)
        field_codes = [code for code in FIELD_POSITION_CODES if code != FIELD_POSITION_CENTER]
        target = target_set.get(target_id).with_gap(params.gap_small_mm)
        for position in np.random.default_rng(field_seed).permutation(len(field_codes)):
            code = field_codes[int(position)]
            placement = place_in_field(params, geometry, target, z_ref, FIELD_POSITION_SIGNS[code],
                                       _fit_margin_px(params, jittered=True))
            _warn_not_fitting(diagnostics, "C field", target, z_ref, placement)
            notes = _placement_notes(placement)
            notes["order_seed"] = field_seed
            for _ in range(params.field_subseries_poses_area):
                plan.append(_jitter_capture(counter, params, geometry, rng, PROCEDURE_AREA, target_id,
                                            params.gap_small_mm, z_ref, code, params.frames_per_area_pose,
                                            SUBSERIES_FIELD, placement.h_mm, placement.v_mm, notes=notes))
        # Step 4: open-background variant for the cutout arrays (back plate removed).
        if open_background and target_id in CUTOUT_TARGETS:
            for _ in range(params.phase_jitter_poses_area):
                plan.append(_jitter_capture(open_counter, params, geometry, rng, PROCEDURE_AREA, target_id, None, z_ref,
                                            FIELD_POSITION_CENTER, params.frames_per_area_pose, SUBSERIES_OPEN,
                                            0.0, 0.0))
    return _renumber(plan)


# ---------------------------------------------------------------------------
# Section 8: detection series
# ---------------------------------------------------------------------------
def plan_detection_series(params: CharacterizationParameters, geometry: SensorGeometry, rng: np.random.Generator,
                          extended: bool = True, targets: TargetSet | None = None,
                          diagnostics: PlanDiagnostics | None = None,
                          shuffle_within_mounting: bool = True, reuse_c_first_frames: bool = False,
                          c_plan: Sequence[PlannedCapture] | None = None) -> list[PlannedCapture]:
    """Section 8 (Series D): the two feature plates T4 and T5, one mounting each.

    Main trials: for each configuration {T4, T5} x {GAP_SMALL_MM, GAP_LARGE_MM} x every station of the ladder
    (``params.z_stations_mm()``), DETECTION_TRIALS_PER_LEVEL poses, each with a new random lateral offset (own logged
    seed, uniform over +/- PHASE_JITTER_SPAN_PX / 2) and FRAMES_PER_DETECTION_TRIAL frames (sub-series "jitter"). All
    features are on the plate, so every pose serves all of them and ``level_index`` stays None: the detection
    "levels" are the (feature, station) pairs, FEATURE_COUNT x 9 = 27 values of D_px spaced by the station ratio with
    the overlaps between neighboring features. The pose order is shuffled with a logged seed (``notes["order_seed"]``);
    the shuffle is applied within each mounting (``shuffle_within_mounting=True``), because shuffling across mountings
    would swap the plate thousands of times; with False the whole series is shuffled as the procedure's wording reads
    literally.
    Extended trials for the low point, D_5 (``extended=True``): at the DETECTION_LOW_STATION_COUNT farthest stations
    (``params.detection_low_stations_mm()``: 1131, 1345 and 1600 mm), where the smallest feature lies near and below the
    expected threshold, the pose count is raised to DETECTION_LOW_TRIALS for every configuration (the additional poses
    carry sub-series "extended"); 300 trials measure the probability DETECTION_LOW_PROBABILITY (5 percent) to about
    +/- 2.5 percent. The 0 percent point is not measured; Analysis D predicts it from the fitted curve.

    Reuse (``reuse_c_first_frames=True``; default off): the first frame of each C pose of the same configuration (target,
    gap, station, centered field, sub-series "jitter") counts as a D trial (Section 8, Reuse: 30 of the 60 per
    configuration and station), so the D main series plans only the remaining poses, the number of matching C poses fewer
    per configuration (at most DETECTION_TRIALS_PER_LEVEL, so the extended poses of the far stations are never
    reduced). The matching C poses are those of ``c_plan`` (the C part of the same plan); without it the C series
    is assumed to be planned as :func:`plan_area_series` plans it, PHASE_JITTER_POSES_AREA poses per configuration. Only
    the first frame of a C pose is independent of the D poses, and the C pose carries FRAMES_PER_AREA_POSE frames, so
    the analysis uses the first one. The Section 9 budget counts the D poses without reuse: ``diagnostics.c_reuse``
    records what was left out and :func:`capture_budget` adds it back, and each planned D pose notes in
    ``notes[REUSED_CLOCK_SHARE_KEY]`` how many reused poses it stands for, so that the drift sentinels (which follow the
    budget clock) are the same as without reuse.

    The D pilot keeps only the post check (post-only sites of the disk plate are not detected, see
    ``acquisition.check.pilot_post_check``); the former level selection from a pilot D_50 is gone, since the levels are
    fixed by the feature ladder and the station ladder."""
    target_set = _targets(params, geometry, targets)
    camera = camera_of(geometry)
    counter = _PoseCounter()
    plan: list[PlannedCapture] = []
    low_stations = {int(round(z)) for z in params.detection_low_stations_mm()}
    series_order_seed = derive_seed(rng)
    reusable = c_first_frame_counts(params, c_plan) if reuse_c_first_frames else Counter()
    reused_poses = reused_frames = 0
    reused_configurations = reused_most = 0

    def shuffled(entries: list[PlannedCapture], order_seed: int) -> list[PlannedCapture]:
        """The entries in the order of a seeded permutation, each noting the seed."""
        for entry in entries:
            entry.notes["order_seed"] = order_seed
        return [entries[int(i)] for i in np.random.default_rng(order_seed).permutation(len(entries))]

    for target_id in AREA_TARGET_ORDER:
        block: list[PlannedCapture] = []
        for gap in (params.gap_small_mm, params.gap_large_mm):
            target = target_set.get(target_id).with_gap(gap)
            for z in params.z_stations_mm():
                total = params.detection_low_trials if (extended and int(round(z)) in low_stations) \
                    else params.detection_trials_per_level
                # Reuse: the C poses of this configuration replace that many of the main D poses.
                reused = min(reusable[(target_id, gap, int(round(z)))], params.detection_trials_per_level)
                if reused:
                    reused_poses += reused
                    reused_frames += reused * params.frames_per_detection_trial
                    reused_configurations += 1
                    reused_most = max(reused_most, reused)
                if not pose_fits_field(camera, target, fronto_parallel_pose(0.0, 0.0, z),
                                       _fit_margin_px(params, jittered=True)):
                    _warn(diagnostics, f"D: {target_id} (G = {gap:g} mm) does not fit the field of view at Z = {z:g} "
                                       "mm with the phase-jitter margin; features may be cut off")
                # Each planned pose also stands for its share of the reused ones on the budget clock (sentinels).
                clock_notes = {REUSED_CLOCK_SHARE_KEY: reused / (total - reused)} if 0 < reused < total else {}
                for trial in range(reused, total):
                    label = SUBSERIES_JITTER if trial < params.detection_trials_per_level else SUBSERIES_EXTENDED
                    block.append(_jitter_capture(counter, params, geometry, rng, PROCEDURE_DETECTION, target_id, gap, z,
                                                 FIELD_POSITION_CENTER, params.frames_per_detection_trial, label,
                                                 0.0, 0.0, notes=clock_notes))
        if shuffle_within_mounting:
            plan.extend(shuffled(block, derive_seed(rng)))
        else:
            plan.extend(block)
    if not shuffle_within_mounting:
        plan = shuffled(plan, series_order_seed)
    if reuse_c_first_frames:
        if diagnostics is not None:
            diagnostics.c_reuse = CReuse(reused_poses, reused_frames, reused_most, reused_configurations)
        if not reused_poses:
            _warn(diagnostics, "D: reuse of the C first frames was asked for, but no C pose matches a D configuration "
                               "(plan the C series too, or check the stations); all D poses are planned")
    return _renumber(plan)


def c_first_frame_counts(params: CharacterizationParameters,
                         c_plan: Sequence[PlannedCapture] | None = None) -> Counter:
    """The number of C poses whose first frame may serve as a D trial, per (target_id, gap_mm, station_z_mm).

    A C pose matches a D configuration when its target, gap and station are the same and it is centered
    (field 0) with the sub-series "jitter": the field sub-series is off-axis and the open-background poses have no
    back plate, so neither is the D configuration (Section 8, Reuse). With ``c_plan`` the poses are counted in it;
    without, every configuration is assumed to have PHASE_JITTER_POSES_AREA of them, as :func:`plan_area_series`
    plans."""
    counts: Counter = Counter()
    if c_plan is None:
        for target_id in AREA_TARGET_ORDER:
            for gap in (params.gap_small_mm, params.gap_large_mm):
                for z in params.z_stations_mm():
                    counts[(target_id, gap, int(round(z)))] = params.phase_jitter_poses_area
        return counts
    for capture in c_plan:
        if (capture.procedure == PROCEDURE_AREA and capture.subseries == SUBSERIES_JITTER
                and capture.field == FIELD_POSITION_CENTER):
            counts[(capture.target_id, capture.gap_mm, int(round(capture.station_z_mm)))] += 1
    return counts


# ---------------------------------------------------------------------------
# Section 5, Step 3: drift sentinels
# ---------------------------------------------------------------------------
def _seconds_per_frame(geometry: SensorGeometry) -> float:
    """Seconds per frame from the sensor frame rate (a † value; raises MissingSensorValue if unknown)."""
    return 1.0 / float(geometry.require("frame_rate_hz"))


def capture_duration_s_for_frames(frames: int, seconds_per_frame: float, move_settle_s: float) -> float:
    """Estimated clock time of a pose with this many frames: move plus settle, then the frames (seconds)."""
    return move_settle_s + frames * seconds_per_frame


def capture_duration_s(capture: PlannedCapture, seconds_per_frame: float, move_settle_s: float) -> float:
    """Estimated clock time of one planned pose: the move plus settle time and its frames (Section 9)."""
    return capture_duration_s_for_frames(capture.frames, seconds_per_frame, move_settle_s)


REUSED_CLOCK_SHARE_KEY = "budget_clock_reused_share"
"""``notes`` key of a D pose planned with ``reuse_c_first_frames``: the number of reused D poses that this pose stands for on
the budget clock (reused poses of its configuration divided by its planned poses). The drift sentinels follow the budget
clock, which assumes no reuse (Section 9), so :func:`insert_sentinels` counts each such pose as 1 + this many poses of
time; the sentinels, and with them the Section 9 totals, are then the same with and without the reuse."""
SENTINEL_NOTE_KEY = "sentinel_note"
SENTINEL_TARGET_KEY = "sentinel_target"
SENTINEL_GAP_KEY = "sentinel_gap_mm"
SENTINEL_REFERENCE_KEY = "mount_reference"
"""Keys of the ``notes`` of a sentinel pose: an explanatory note, the mounted target and its gap (also the pose row's
``target_id`` and ``gap_mm``), and whether this is the first sentinel after the target was mounted (the reference of
that target's drift, see ``analysis.noise``). The last one is also carried into the manifest, as its column
``sentinel_mount_reference`` (:meth:`PlannedCapture.manifest_metadata`)."""


def _mounted_target_id(plan: Sequence[PlannedCapture], position: int) -> str | None:
    """The target mounted at ``plan[position]`` as seen by a sentinel: the target of the nearest earlier capture that is
    not a sentinel, or of the nearest later one when nothing was captured before (a sentinel at the start of a plan)."""
    for other in list(reversed(plan[:position])) + list(plan[position + 1:]):
        if other.procedure != PROCEDURE_SENTINEL:
            return other.target_id
    return None


def count_sentinel_remounts(plan: Sequence[PlannedCapture]) -> int:
    """Number of sentinels whose target differs from the target mounted at that point of the plan (see
    :func:`_mounted_target_id`), i.e. sentinels that would need a re-mount. Zero for every plan made by
    :func:`insert_sentinels`: a sentinel is captured on the mounted target. The captures of the optional drift run are no
    sentinels of a mount (their plate stands on its own stand) and are not counted."""
    return sum(1 for position, capture in enumerate(plan)
               if capture.procedure == PROCEDURE_SENTINEL and capture.subseries != SUBSERIES_DRIFT_RUN
               and _mounted_target_id(plan, position) not in (None, capture.target_id))


def plan_drift_run(params: CharacterizationParameters) -> list[PlannedCapture]:
    """The OPTIONAL separate drift run of Section 4, Step 3, outside the Section 9 budget: T2 on a fixed stand at
    Z_REFERENCE_MM, fronto-parallel and centered, the robot idle, one capture of SENTINEL_FRAMES frames every
    DRIFT_RUN_CAPTURE_INTERVAL_MIN minutes for DRIFT_RUN_DURATION_MIN minutes (the first capture at time zero, so
    duration / interval + 1 captures). Procedure S (the sentinels folder), sub-series ``drift_run``; the pose index counts
    the captures from DRIFT_RUN_POSE_INDEX_BASE, so no capture shares a pose key with an in-session sentinel. The order
    runs 0..N-1 on its own; :func:`plan_full_session` (``drift_run=True``) appends the captures after the session's poses.

    The robot is idle and the plate stands on a fixed stand, so the poses have no read-back robot pose: each carries
    ``notes["fixed_stand"] = True`` (``io.manifest.FIXED_STAND_KEY``) and the NOMINAL target pose (T2 centered and
    fronto-parallel at Z_REFERENCE_MM). ``make_manifest`` copies the robot pose and the target pose of the manifest from
    this nominal pose and needs only the capture time and the sensor temperature from the pose log."""
    captures = int(math.floor(params.drift_run_duration_min / params.drift_run_capture_interval_min + 1e-9)) + 1
    pose = fronto_parallel_pose(0.0, 0.0, params.z_reference_mm)
    counter = _PoseCounter(DRIFT_RUN_POSE_INDEX_BASE)
    note = {SENTINEL_NOTE_KEY: f"drift run on the fixed stand: T2 at Z = {params.z_reference_mm:g} mm, robot idle",
            FIXED_STAND_KEY: True}
    return _renumber([_new_capture(counter, PROCEDURE_SENTINEL, TARGET_NOISE_PLATE, None, params.z_reference_mm,
                                   FIELD_POSITION_CENTER, params.sentinel_frames, SUBSERIES_DRIFT_RUN, pose,
                                   notes=dict(note))
                      for _ in range(captures)])


MAIN_CLOCK = "main"
"""Name of the clock of the main plan in :func:`insert_sentinels` (the clock of an optional set is named by its sub-series)."""


def _clock_of(capture: PlannedCapture) -> str:
    """The sentinel clock a capture runs on: that of its optional set (the sub-series label) when it belongs to one, else
    the clock of the main plan. The main clock never counts the time of an optional set, because the set is outside the
    Section 9 budget (and, for the filters-off repeat, the sensor is in another configuration)."""
    return capture.subseries if capture.subseries in OPTIONAL_SUBSERIES else MAIN_CLOCK


def insert_sentinels(plan: list[PlannedCapture], params: CharacterizationParameters, geometry: SensorGeometry,
                     seconds_per_frame: float, move_settle_s: float,
                     diagnostics: PlanDiagnostics | None = None,
                     targets: TargetSet | None = None) -> list[PlannedCapture]:
    """Section 5, Step 3, as revised in the specification review: drift sentinels on the MOUNTED target.

    A sentinel is captured on the front plane of the target that is mounted at that point of the plan (the target of
    the surrounding series, with the gap as mounted), centered at Z_REFERENCE_MM, SENTINEL_FRAMES frames each
    (procedure S, sub-series "sentinel"). The pose row carries the mounted ``target_id`` and ``gap_mm`` and a note
    (``notes["sentinel_note"]``); no target is re-mounted for a sentinel. The first sentinel after each mount is that
    target's reference (``notes["mount_reference"]``): the analysis (``analysis.noise``) computes each target's drift
    relative to it. A mount is a change of ``target_id`` from one capture to the next; a change of gap is not (the
    front plane stays where it is).

    The function walks the plan with a clock that adds, per capture, the move plus settle time and the frames at
    ``seconds_per_frame`` (the Section 9 budget model) and places a sentinel
      * before the first capture that is not a registration pose, on the target of that capture (T2 when series A
        comes first, as the procedure defines it);
      * at every series boundary, that is after the last pose of each series, on the target still mounted there (T2
        after A and after B-Z, so T2 sentinels bracket those series; T3b, T5 and T5 after B-HV, C and D);
      * after any capture that is followed by one that would start when at least DRIFT_SENTINEL_INTERVAL_MIN of clock
        time has passed since the previous sentinel began, on the mounted target;
      * after the last capture (the end of the last series).

    The optional sets (``OPTIONAL_SUBSERIES``: the filters-off repeat, the staircase, the lateral sweep, the open-background
    variant of C; the drift run has no sentinels) are outside the main budget, so the sentinels captured during one belong to that set and never to the main plan. Each clock runs on its
    own: the MAIN clock walks the captures of the main plan only (the sentinels it places are exactly those of a plan without
    the optional sets, so the Section 9 totals do not depend on which optional sets are planned), and each optional set
    has a clock of its own that walks the captures of that set only (the set's captures may lie in several places of the
    plan, e.g. the filters-off repeat of A, B-HV and B-Z), with the same rules (a sentinel at the end of each of its series
    and whenever the interval is up; no opening sentinel, the main plan has just placed one). A sentinel of an optional set
    is placed right after the capture it follows, takes the sub-series label of the set and counts its pose indices from the
    set's base (``OPTIONAL_POSE_INDEX_BASES``), so the set's budget table and pose-index range include it. It carries no
    mount-reference flag: the analysis of the session's drift (``analysis.noise``) leaves the sentinels of the optional sets
    out, because they are not captured in the main configuration.

    Registration poses (Section 4) come before the first sentinel and are not counted. Existing sentinels in ``plan``
    are dropped first, so the function can be applied to a plan again after it was changed. Order is renumbered
    0..N-1; the input list is not modified.

    ``diagnostics`` receives a note with the sentinel count and the number of re-mounts they need (zero by
    construction; a warning if it were not), a note with the sentinels of each optional set, and a warning for a mounted
    target that does not fit the field of view at Z_REFERENCE_MM. ``targets`` is the target set of that fit check
    (default: the standard set)."""
    interval_s = params.drift_sentinel_interval_min * SECONDS_PER_MINUTE
    pose = fronto_parallel_pose(0.0, 0.0, params.z_reference_mm)
    sentinel_duration = capture_duration_s_for_frames(params.sentinel_frames, seconds_per_frame, move_settle_s)
    # One pose-index counter per clock: the main sentinels count from zero, those of an optional set from the set's base.
    sentinel_counters = {MAIN_CLOCK: _PoseCounter()}
    sentinel_counters.update({label: _PoseCounter(base) for label, base in OPTIONAL_POSE_INDEX_BASES.items()})
    captures = [c for c in plan if c.procedure != PROCEDURE_SENTINEL]
    # The capture that follows each capture ON THE SAME CLOCK (None for the last of its clock): a sentinel that is due
    # before that next capture is placed right after this one.
    next_on_clock: dict[int, PlannedCapture | None] = {}
    latest_of_clock: dict[str, int] = {}
    for position, capture in enumerate(captures):
        clock = _clock_of(capture)
        if clock in latest_of_clock:
            next_on_clock[latest_of_clock[clock]] = capture
        latest_of_clock[clock] = position
        next_on_clock[position] = None
    result: list[PlannedCapture] = []
    mounted_target: str | None = None          # target of the latest capture placed (registration poses included)
    mount_serial = 0                           # counts the mounts so far
    referenced_serial = 0                      # the mount that already has its reference sentinel

    def mount(capture: PlannedCapture) -> None:
        """Note that ``capture`` is taken with its target mounted (a change of target is a new mount)."""
        nonlocal mounted_target, mount_serial
        if capture.target_id != mounted_target:
            mounted_target, mount_serial = capture.target_id, mount_serial + 1

    def sentinel(on: PlannedCapture, clock: str) -> PlannedCapture:
        """A new sentinel pose of the clock on the target (and gap) of the capture ``on``, with the next free pose index."""
        nonlocal referenced_serial
        gap_text = "no back plate" if on.gap_mm is None else f"G = {on.gap_mm:g} mm"
        notes = {SENTINEL_TARGET_KEY: on.target_id, SENTINEL_GAP_KEY: on.gap_mm}
        note = (f"drift sentinel on the mounted target {on.target_id} ({gap_text}): front plane at "
                f"Z = {params.z_reference_mm:g} mm, centered")
        if clock == MAIN_CLOCK:
            reference = referenced_serial != mount_serial
            referenced_serial = mount_serial
            notes[SENTINEL_REFERENCE_KEY] = reference
            note += "; reference of this mount" if reference else ""
        else:
            note += f"; captured during the {clock} set, counted with it (outside the main budget)"
        notes[SENTINEL_NOTE_KEY] = note
        return _new_capture(sentinel_counters[clock], PROCEDURE_SENTINEL, on.target_id, on.gap_mm, params.z_reference_mm,
                            FIELD_POSITION_CENTER, params.sentinel_frames,
                            SUBSERIES_SENTINEL if clock == MAIN_CLOCK else clock, pose, notes=notes)

    since_last: dict[str, float] = {}          # per clock: clock time since the last sentinel of it began
    for position, capture in enumerate(captures):
        if capture.procedure == PROCEDURE_REGISTRATION:
            mount(capture)
            result.append(capture)
            continue
        clock = _clock_of(capture)
        if clock == MAIN_CLOCK and MAIN_CLOCK not in since_last:
            mount(capture)                                       # the first pose of the plan: sentinel on its target
            result.append(sentinel(capture, clock))
            since_last[clock] = sentinel_duration
        since_last.setdefault(clock, 0.0)                        # an optional set starts without an opening sentinel
        mount(capture)
        result.append(capture)
        since_last[clock] += capture_duration_s(capture, seconds_per_frame, move_settle_s) \
            * (1.0 + capture.notes.get(REUSED_CLOCK_SHARE_KEY, 0.0))
        following = next_on_clock[position]
        if following is None or following.procedure != capture.procedure or since_last[clock] >= interval_s:
            # The end of the clock's last series, a series boundary, or the interval is up: the target is still mounted.
            result.append(sentinel(capture, clock))
            since_last[clock] = sentinel_duration
    _renumber(result)
    if diagnostics is not None and result:
        sentinels = [c for c in result if c.procedure == PROCEDURE_SENTINEL]
        main_sentinels = [c for c in sentinels if c.subseries == SUBSERIES_SENTINEL]
        per_target = ", ".join(f"{t}: {n}" for t, n in sorted(Counter(c.target_id for c in main_sentinels).items()))
        remounts = count_sentinel_remounts(result)
        diagnostics.note(f"{len(main_sentinels)} drift sentinels ({per_target}), each on the front plane of the target "
                         f"mounted at that point of the plan, centered at Z = {params.z_reference_mm:g} mm, "
                         f"{params.sentinel_frames} frames, at the series boundaries and every "
                         f"{params.drift_sentinel_interval_min:g} min of estimated clock; {remounts} sentinel re-mounts")
        for label in OPTIONAL_POSE_INDEX_BASES:
            in_set = [c for c in sentinels if c.subseries == label]
            if in_set:
                diagnostics.note(f"{len(in_set)} drift sentinels captured during the {label} set: counted with that set, "
                                 "outside the main budget")
        if remounts:
            diagnostics.warn(f"{remounts} of {len(sentinels)} drift sentinels are not on the target mounted at that "
                             "point of the plan; each needs a re-mount")
        target_set = _targets(params, geometry, targets)
        camera = camera_of(geometry)
        for target_id, gap in sorted({(c.target_id, c.gap_mm) for c in sentinels}, key=str):
            if not pose_fits_field(camera, target_set.get(target_id).with_gap(gap), pose,
                                   _fit_margin_px(params, jittered=False)):
                diagnostics.warn(f"the sentinel target {target_id} ({'no back plate' if gap is None else f'G = {gap:g} mm'}) "
                                 f"does not fit the field of view at the sentinel station Z = "
                                 f"{params.z_reference_mm:g} mm")
    return result


# ---------------------------------------------------------------------------
# The whole session
# ---------------------------------------------------------------------------
def plan_full_session(params: CharacterizationParameters, geometry: SensorGeometry, rng: np.random.Generator,
                      registration: Registration | None = None, expected_quantum_mm: Callable[[float], float] | float | None = None,
                      filters_off: bool = False, open_background: bool = False, extended: bool = True,
                      targets: TargetSet | None = None, series: Iterable[str] | None = None,
                      diagnostics: PlanDiagnostics | None = None, staircase: bool = False,
                      reuse_c_first_frames: bool = False, lateral_sweep: bool = False,
                      drift_run: bool = False) -> list[PlannedCapture]:
    """All series in the order of the procedure (R, A, B-HV, B-Z, C, D) with drift sentinels inserted once over
    the whole plan and ``order`` renumbered 0..N-1.

    ``series`` selects a subset by procedure letter (any of R, A, B, Z, C, D; default all). ``filters_off``
    appends the filters-off repeat (Section 4, Step 4.2: series A, B-HV and B-Z, each right after its filters-on
    series; sub-series "filters_off", outside the main budget) to each of those series. ``staircase`` adds the optional
    second pass of series B-Z (the staircase of Section 6.2, outside the main budget; the filters-off repeat of B-Z then
    includes it). ``lateral_sweep`` adds the optional second pass of series B-HV (the lateral sweep of Section 6.1 at the
    reference station, outside the main budget). ``drift_run`` appends the captures of the optional separate drift run
    (:func:`plan_drift_run`, Section 4, Step 3) after everything else, whatever ``series`` selects: they are outside the main
    budget, get no drift sentinels (nothing is mounted by the robot), and have the fixed-stand flag in their notes.
    ``reuse_c_first_frames`` lets the D main series count the first frame
    of each matching C pose as a D trial and plan only the remaining D poses (:func:`plan_detection_series`; the
    budget is still computed without the reuse). ``registration`` is only checked here: a
    registration whose residual was not accepted triggers a warning (the commanded flange poses depend on it; they
    are computed by :func:`write_plan`). The other arguments are those of the series planners."""
    chosen = set(PLANNED_SERIES) if series is None else {s.upper() for s in series}
    unknown = chosen - set(PLANNED_SERIES)
    if unknown:
        raise ValueError(f"unknown series {sorted(unknown)}; choose from {PLANNED_SERIES}")
    target_set = _targets(params, geometry, targets)
    if registration is not None and registration.accepted is False:
        _warn(diagnostics, f"the registration residual {registration.residual_rms_mm:.3f} mm was not accepted "
                           f"(limit {params.registration_residual_accept_mm:g} mm); the commanded flange poses will be "
                           "off by about that much")
    parts: list[PlannedCapture] = []
    if PROCEDURE_REGISTRATION in chosen:
        parts += plan_registration(params, geometry, rng, target_set, diagnostics)
    if PROCEDURE_NOISE in chosen:
        parts += plan_noise_series(params, geometry, rng, False, False, target_set, diagnostics)
        if filters_off:
            parts += plan_noise_series(params, geometry, rng, True, False, target_set, diagnostics)
    if PROCEDURE_EDGES in chosen:
        parts += plan_edge_series(params, geometry, rng, target_set, diagnostics, lateral_sweep=lateral_sweep)
        if filters_off:
            parts += plan_edge_series(params, geometry, rng, target_set, diagnostics, filters_off=True)
    if PROCEDURE_ZSTEP in chosen:
        parts += plan_zstep_series(params, geometry, rng, expected_quantum_mm, target_set, diagnostics,
                                   staircase=staircase)
        if filters_off:
            parts += plan_zstep_series(params, geometry, rng, expected_quantum_mm, target_set, diagnostics,
                                       filters_off=True, staircase=staircase)
    area_plan: list[PlannedCapture] | None = None
    if PROCEDURE_AREA in chosen:
        area_plan = plan_area_series(params, geometry, rng, open_background, target_set, diagnostics)
        parts += area_plan
    if PROCEDURE_DETECTION in chosen:
        parts += plan_detection_series(params, geometry, rng, extended, target_set, diagnostics,
                                       reuse_c_first_frames=reuse_c_first_frames, c_plan=area_plan)
    plan = insert_sentinels(parts, params, geometry, _seconds_per_frame(geometry), params.move_and_settle_time_s,
                            diagnostics=diagnostics, targets=target_set)
    if drift_run:
        # Appended after the sentinels are placed: the run is its own measurement, not part of the session's clock.
        run = plan_drift_run(params)
        plan = _renumber(plan + run)
        if diagnostics is not None:
            diagnostics.note(f"optional drift run (--drift-run): {len(run)} captures of {params.sentinel_frames} frames "
                             f"appended outside the main budget; the robot is idle and T2 stands on a fixed stand at "
                             f"Z = {params.z_reference_mm:g} mm (nominal pose, fixed_stand=true)")
    return plan


# ---------------------------------------------------------------------------
# Section 9: capture budget
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class BudgetRow:
    """One row of the Section 9 table."""

    procedure: str
    series: str
    poses: int
    frames: int
    robot_hours: float


def capture_budget(plan: Sequence[PlannedCapture], frame_rate_hz: float, move_settle_s: float,
                   c_reuse: CReuse | None = None) -> list[BudgetRow]:
    """The Section 9 table: poses, frames and robot hours per series, in the order of the procedure.
    "The estimate assumes 10 frames/s and 3 s per move plus settle": hours = (poses x move_settle_s + frames /
    frame_rate_hz) / 3600. Series without poses are omitted. The optional sets (``OPTIONAL_SUBSERIES``: the filters-off
    repeat, the B-Z staircase, the B-HV lateral sweep, the drift run and the C open-background variant) are outside the main
    budget and are not counted here, nor are the drift sentinels captured during them; see :func:`filters_off_budget`,
    :func:`staircase_budget`, :func:`lateral_sweep_budget` and :func:`open_background_budget`.

    The budget is computed WITHOUT the reuse of C first frames (Section 9: "the budget assumes no reuse"): when the plan
    was made with ``reuse_c_first_frames`` and the caller passes ``c_reuse`` (``diagnostics.c_reuse``), the D poses and
    frames left out of the plan are added back to the D row."""
    rows = _budget_rows([c for c in plan if c.subseries not in OPTIONAL_SUBSERIES], frame_rate_hz, move_settle_s)
    if c_reuse is None or not c_reuse.poses:
        return rows
    adjusted = []
    for row in rows:
        if row.procedure == PROCEDURE_DETECTION:
            poses, frames = row.poses + c_reuse.poses, row.frames + c_reuse.frames
            row = BudgetRow(row.procedure, row.series, poses, frames,
                            (poses * move_settle_s + frames / frame_rate_hz) / SECONDS_PER_HOUR)
        adjusted.append(row)
    return adjusted


def filters_off_budget(plan: Sequence[PlannedCapture], frame_rate_hz: float, move_settle_s: float) -> list[BudgetRow]:
    """The same table for the filters-off repeat alone (Section 4, Step 4.2: series A, B-HV and B-Z), which is
    outside the main budget of the document's estimate. Empty when the plan has no filters-off poses."""
    return _budget_rows([c for c in plan if c.subseries == SUBSERIES_FILTERS_OFF], frame_rate_hz, move_settle_s)


def staircase_budget(plan: Sequence[PlannedCapture], frame_rate_hz: float, move_settle_s: float) -> list[BudgetRow]:
    """The same table for the optional B-Z staircase alone (Section 6.2, second pass), which is outside the main budget.
    Empty when the plan has no staircase poses."""
    return _budget_rows([c for c in plan if c.subseries == SUBSERIES_STAIRCASE], frame_rate_hz, move_settle_s)


def lateral_sweep_budget(plan: Sequence[PlannedCapture], frame_rate_hz: float, move_settle_s: float) -> list[BudgetRow]:
    """The same table for the optional B-HV lateral sweep alone (Section 6.1, second pass), which is outside the main
    budget. Empty when the plan has no lateral-sweep poses."""
    return _budget_rows([c for c in plan if c.subseries == SUBSERIES_LATERAL_SWEEP], frame_rate_hz, move_settle_s)


def open_background_budget(plan: Sequence[PlannedCapture], frame_rate_hz: float, move_settle_s: float) -> list[BudgetRow]:
    """The same table for the optional open-background variant of series C alone (Section 7, Step 4), which is outside the
    main budget. Its Sentinels row holds the drift sentinels captured during the variant. Empty when the plan has no
    open-background poses."""
    return _budget_rows([c for c in plan if c.subseries == SUBSERIES_OPEN], frame_rate_hz, move_settle_s)


def _budget_rows(members_of_plan: Sequence[PlannedCapture], frame_rate_hz: float,
                 move_settle_s: float) -> list[BudgetRow]:
    """Budget rows (poses, frames, robot hours) per series for the given captures."""
    rows = []
    for procedure in SERIES_ORDER:
        members = [c for c in members_of_plan if c.procedure == procedure]
        if not members:
            continue
        poses = len(members)
        frames = sum(c.frames for c in members)
        hours = (poses * move_settle_s + frames / frame_rate_hz) / SECONDS_PER_HOUR
        rows.append(BudgetRow(procedure, SERIES_LABELS[procedure], poses, frames, hours))
    return rows


def budget_total(rows: Sequence[BudgetRow]) -> BudgetRow:
    """The total row of a budget table."""
    return BudgetRow("", "Total", sum(r.poses for r in rows), sum(r.frames for r in rows),
                     sum(r.robot_hours for r in rows))


def format_budget_table(rows: Sequence[BudgetRow]) -> str:
    """The Section 9 table (Series, Poses, Frames, Robot time h) as text, with its total row."""
    lines = [f"{'Series':<18}{'Poses':>8}{'Frames':>9}{'Robot time h':>14}", "-" * BUDGET_TABLE_WIDTH]
    for row in list(rows) + [budget_total(rows)]:
        if row.series == "Total":
            lines.append("-" * BUDGET_TABLE_WIDTH)
        lines.append(f"{row.series:<18}{row.poses:>8,d}{row.frames:>9,d}{row.robot_hours:>14.2f}")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# poses.csv
# ---------------------------------------------------------------------------
def _cell(value: Any) -> str:
    """CSV text of a value: empty for None, repr for floats (exact round trip)."""
    if value is None:
        return ""
    if isinstance(value, (float, np.floating)):
        return repr(float(value))
    return str(value)


def flange_pose_values(registration: Registration, target_to_camera: RigidTransform) -> list[float]:
    """The FLANGE_POSE_COLUMNS values of the flange pose that puts the target at ``target_to_camera``."""
    flange = registration.flange_to_base_for(target_to_camera)
    quaternion_xyzw = Rotation.from_matrix(flange.rotation).as_quat()
    quaternion_wxyz = np.roll(quaternion_xyzw, 1)
    if quaternion_wxyz[0] < 0.0:
        quaternion_wxyz = -quaternion_wxyz                      # the sign is free; w >= 0 by convention
    return pose_to_six(flange) + [float(x) for x in flange.rotation.reshape(-1)] + [float(x) for x in quaternion_wxyz]


def plan_rows(plan: Sequence[PlannedCapture], registration: Registration | None) -> tuple[list[str], list[list[str]]]:
    """(column names, rows of text) of poses.csv for a plan, with the flange pose columns when a registration is given."""
    columns = list(PLAN_CSV_COLUMNS) + (list(FLANGE_POSE_COLUMNS) if registration is not None else [])
    rows = []
    for c in plan:
        values: list[Any] = [c.order, c.procedure, c.target_id, c.gap_mm, c.station_z_mm, c.field, c.pose_index,
                             c.frames, c.subseries, c.seed, c.offset_h_mm, c.offset_v_mm, c.tilt_axis, c.tilt_deg,
                             c.step_mm, c.visit, c.level_index]
        values += pose_to_six(c.target_to_camera)
        values.append(json.dumps(c.notes, sort_keys=True))
        if registration is not None:
            values += flange_pose_values(registration, c.target_to_camera)
        rows.append([_cell(v) for v in values])
    return columns, rows


def read_plan_csv(path: str | Path) -> list[PlannedCapture]:
    """Read a poses.csv written by :func:`write_plan` back into PlannedCapture records (flange columns are ignored).
    Raises ValueError, naming the column, for a missing column or a malformed number."""
    plan = []
    with Path(path).open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        missing = [c for c in PLAN_CSV_COLUMNS if c not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"{path}: missing plan columns {missing}")
        for line, row in enumerate(reader, start=2):
            def number(name: str, required: bool = False) -> float | None:
                """A float from a cell of this row; None when empty (an error when ``required``)."""
                text = (row.get(name) or "").strip()
                if not text:
                    if required:
                        raise ValueError(f"{path} line {line}: column {name} is required")
                    return None
                try:
                    return float(text)
                except ValueError as error:
                    raise ValueError(f"{path} line {line}: column {name} is not a number: {text!r}") from error

            def integer(name: str) -> int | None:
                """An integer from a cell of this row; None when empty."""
                value = number(name)
                return None if value is None else int(value)

            gap = number("gap_mm")
            plan.append(PlannedCapture(
                procedure=row["procedure"].strip(), target_id=row["target_id"].strip(), gap_mm=gap,
                station_z_mm=float(number("station_z_mm", True)), field=int(number("field", True)),
                pose_index=int(number("pose_index", True)), frames=int(number("frames", True)),
                subseries=row["subseries"].strip() or SUBSERIES_MAIN,
                target_to_camera=six_to_pose(float(number(c, True)) for c in TARGET_POSE_COLUMNS),
                seed=integer("seed"), offset_h_mm=number("offset_h_mm") or 0.0, offset_v_mm=number("offset_v_mm") or 0.0,
                tilt_axis=row["tilt_axis"].strip(), tilt_deg=number("tilt_deg") or 0.0, step_mm=number("step_mm"),
                visit=row["visit"].strip(), level_index=integer("level_index"), order=int(number("order", True)),
                notes=json.loads(row[PLAN_NOTES_COLUMN] or "{}")))
    return plan


# ---------------------------------------------------------------------------
# plan_summary.txt
# ---------------------------------------------------------------------------
def _station_table(plan: Sequence[PlannedCapture]) -> list[str]:
    """Poses per station depth and series (the series other than registration, whose depths are continuous)."""
    series = [p for p in SERIES_ORDER if p != PROCEDURE_REGISTRATION and any(c.procedure == p for c in plan)]
    stations = sorted({c.station_z_mm for c in plan if c.procedure in series})
    header = f"  {'Z mm':>6}" + "".join(f"{p:>8}" for p in series) + f"{'total':>8}"
    lines = [header]
    for z in stations:
        counts = [sum(1 for c in plan if c.procedure == p and c.station_z_mm == z) for p in series]
        lines.append(f"  {z:>6.0f}" + "".join(f"{n:>8d}" for n in counts) + f"{sum(counts):>8d}")
    return lines


def _subseries_lines(plan: Sequence[PlannedCapture]) -> list[str]:
    """Poses and frames per (series, sub-series)."""
    lines = []
    for procedure in SERIES_ORDER:
        labels = []
        for c in plan:
            if c.procedure == procedure and c.subseries not in labels:
                labels.append(c.subseries)
        for label in labels:
            members = [c for c in plan if c.procedure == procedure and c.subseries == label]
            lines.append(f"  {SERIES_LABELS[procedure]:<18} {label:<12}{len(members):>8d} poses{sum(c.frames for c in members):>9d} frames")
    return lines




def _pose_index_range_lines(plan: Sequence[PlannedCapture]) -> list[str]:
    """The pose-index range in use by each optional set the plan contains (empty when there is none): the main poses
    have three-digit indices (P000 to P999), each optional set its own four-digit range (``io.manifest``)."""
    lines = []
    for label, base in OPTIONAL_POSE_INDEX_BASES.items():
        indices = [c.pose_index for c in plan if c.subseries == label]
        if indices:
            lines.append(f"  {label:<14} pose indices P{min(indices):04d} to P{max(indices):04d} (four digits; "
                         f"the range starts at P{base:04d})")
    if lines:
        lines.insert(0, "Pose indices of the optional sets outside the budget (the main poses use three digits, P000 up to "
                        f"P{FILTERS_OFF_POSE_INDEX_BASE - 1:03d}):")
        lines.insert(0, "")
    return lines


def _set_sentinel_line(plan: Sequence[PlannedCapture], label: str) -> str:
    """The line under the budget table of an optional set that states how many drift sentinels were captured during the set
    (they are in the set's Sentinels row and never in the main budget)."""
    sentinels = [c for c in plan if c.procedure == PROCEDURE_SENTINEL and c.subseries == label]
    return (f"  Drift sentinels captured during this set (counted here, not in the main budget): {len(sentinels)}, "
            f"{sum(c.frames for c in sentinels)} frames.")


def _drift_run_lines(plan: Sequence[PlannedCapture], params: CharacterizationParameters) -> list[str]:
    """The block of plan_summary.txt for the optional separate drift run (empty when the plan has none): the number of
    captures and frames, the pose-index range, the first and last file names and the statement that the robot is idle and
    the plate stands on a fixed stand at the reference station. The run is outside the main budget and costs no robot time."""
    run = [c for c in plan if c.subseries == SUBSERIES_DRIFT_RUN]
    if not run:
        return []
    frames = sum(c.frames for c in run)
    first, last = run[0], run[-1]
    return ["", "Optional separate drift run (--drift-run, Section 4, Step 3; outside the main budget, not in the totals "
                "above or in the comparison with the document's estimate):",
            f"  {len(run)} captures of {first.frames} frames = {frames} frames, one capture every "
            f"{params.drift_run_capture_interval_min:g} min for {params.drift_run_duration_min:g} min (the first at time zero)",
            f"  pose indices P{min(c.pose_index for c in run):04d} to P{max(c.pose_index for c in run):04d}, files "
            f"{first.file_name(0)} ... {last.file_name(last.frames - 1)} (procedure S, in the sentinels folder)",
            f"  The robot is idle and the plate ({first.target_id}) is on a fixed stand at the reference station, Z = "
            f"{first.station_z_mm:g} mm, centered and fronto-parallel. The rows of poses.csv carry the nominal pose with "
            f"notes {FIXED_STAND_KEY}=true: make_manifest copies the robot pose columns from it, so the pose log needs "
            "only the capture time (timestamp) and the sensor temperature (sensor_temp_c) for these captures."]


def _adjustment_lines(plan: Sequence[PlannedCapture]) -> list[str]:
    """One line per distinct field placement that was pulled inward or could not fit (module docstring)."""
    seen: dict[tuple, dict] = {}
    for c in plan:
        record = c.notes.get("field_placement")
        if record is None:
            continue
        key = (c.procedure, c.target_id, c.gap_mm, c.station_z_mm, c.field, c.subseries)
        seen.setdefault(key, record)
    lines = []
    for (procedure, target_id, gap, z, code, subseries), r in seen.items():
        where = f"{procedure} {target_id}" + ("" if gap is None else f" G={gap:g}") + f" Z={z:g} field {code}" \
            + ("" if subseries in (SUBSERIES_MAIN, SUBSERIES_FIELD) else f" ({subseries})")
        status = f"kept {PERCENT * r['fraction_kept']:.0f}%"
        if not r["fits"]:
            status += f", still short by {r['violation_px']:.0f} px (as at the center)"
        lines.append(f"  {where}: requested ({r['requested_h_mm']:.1f}, {r['requested_v_mm']:.1f}) mm -> "
                     f"({r['h_mm']:.1f}, {r['v_mm']:.1f}) mm, {status}")
    return lines


def _zstep_lines(plan: Sequence[PlannedCapture], params: CharacterizationParameters) -> list[str]:
    """The B-Z lines of plan_summary.txt, from the plan: per station the expected depth quantum, the step-ladder rungs in
    millimeters and the ramp tilt with its depth span (Section 6.2). Empty when the plan has no ladder or ramp pose."""
    ladder = [c for c in plan if c.procedure == PROCEDURE_ZSTEP and c.subseries == SUBSERIES_LADDER]
    ramp = [c for c in plan if c.procedure == PROCEDURE_ZSTEP and c.subseries == SUBSERIES_RAMP]
    lines: list[str] = []
    if ladder:
        lines += ["", f"B-Z step ladder rungs per station (mm; {', '.join(f'{q:g}' for q in params.z_step_ladder_quanta)} "
                      f"times the expected depth quantum dZ_q = q Z^2 / k, each at least ROBOT_MIN_RESOLVABLE_MOVE_MM = "
                      f"{params.robot_min_resolvable_move_mm:g} mm):"]
        for z in sorted({c.station_z_mm for c in ladder}):
            rungs = sorted({c.step_mm for c in ladder if c.station_z_mm == z and c.step_mm is not None})
            quantum = next(c.notes["expected_quantum_mm"] for c in ladder if c.station_z_mm == z)
            lines.append(f"  Z0 = {z:>5.0f} mm: dZ_q = {quantum:.3f} mm; rungs " + ", ".join(f"{r:.3g}" for r in rungs)
                         + f" ({params.robot_repeatability_mm:g} mm repeatability is "
                         + ", ".join(f"{PERCENT * params.robot_repeatability_mm / r:.0f}%" for r in rungs) + " of the rungs)")
    if ramp:
        lines += ["", f"B-Z ramp per station (T2 tilted about H, {params.frames_per_ramp_pose} frames; the true depth "
                      f"across the visible plate height spans {params.ramp_quanta:g} expected quanta):"]
        for c in sorted(ramp, key=lambda c: c.station_z_mm):
            shift = c.notes.get("ramp_center_shift_mm", 0.0)
            lines.append(f"  Z0 = {c.station_z_mm:>5.0f} mm: tilt {c.tilt_deg:.3f} deg, dZ_q = "
                         f"{c.notes['expected_quantum_mm']:.3f} mm, span {c.notes['ramp_span_mm']:.2f} mm over "
                         f"{c.notes['ramp_visible_height_mm']:.0f} mm visible height, near edge {c.notes['near_edge_mm']:.1f} mm"
                         + (f" (plate center moved {shift:.2f} mm farther to keep the near edge at Z_MIN)" if shift > 0 else ""))
    return lines


def plan_summary_text(plan: Sequence[PlannedCapture], params: CharacterizationParameters | None = None,
                      geometry: SensorGeometry | None = None, diagnostics: PlanDiagnostics | None = None,
                      registration: Registration | None = None) -> str:
    """The text of plan_summary.txt: counts per series and station, the Section 9 budget table with its comparison to
    the document's estimate, the field-offset adjustments and the warnings."""
    params = CharacterizationParameters() if params is None else params
    lines = ["Capture plan summary", "====================", ""]
    if diagnostics is not None and diagnostics.master_seed is not None:
        lines.append(f"Master random seed: {diagnostics.master_seed} (every pose also carries its own seed in poses.csv)")
    lines.append("Registration: " + ("flange poses to command are in poses.csv (base_x_mm ...)." if registration is not None
                                     else "none given; poses.csv holds the target poses in the camera frame only."))
    stations = params.z_stations_mm()
    lines += ["", f"Station ladder ({len(stations)} stations, ratio {params.z_station_ratio:.4f}, Z_MIN {params.z_min_mm:g} "
                  f"to Z_MAX {params.z_max_mm:g} mm): " + ", ".join(f"{z:g}" for z in stations),
              "  B-HV shape stations: " + ", ".join(f"{z:g}" for z in params.z_shape_stations_mm()),
              "  B-Z and tilt reduced stations: " + ", ".join(f"{z:g}" for z in params.z_reduced_stations_mm()),
              "  A legacy-metric extra stations (center field only): "
              + ", ".join(f"{z:g}" for z in params.legacy_extra_stations_mm()),
              "  D low-point (D_5) stations (extended trials): "
              + ", ".join(f"{z:g}" for z in params.detection_low_stations_mm()),
              "  Feature diameters (mm): " + ", ".join(f"{d:.1f}" for d in params.feature_diameters_mm(geometry))
              if geometry is not None and geometry.sensor_fx_px is not None else "  Feature diameters: geometry unknown"]
    lines += ["", f"Poses planned: {len(plan)}; frames: {sum(c.frames for c in plan)}", "",
              "Poses per station depth and series (R, the registration poses, span Z_MIN to Z_MAX and are not listed by station):"]
    lines += _station_table(plan)
    lines += ["", "Poses and frames per sub-series:"] + _subseries_lines(plan)
    lines += _pose_index_range_lines(plan)
    lines.append("")
    rate = None if geometry is None else geometry.frame_rate_hz
    if rate is None:
        lines.append("Capture budget (Section 9): not computed, the sensor frame rate is not known (a † value of Section 2).")
    else:
        c_reuse = None if diagnostics is None else diagnostics.c_reuse
        rows = capture_budget(plan, rate, params.move_and_settle_time_s, c_reuse)
        total = budget_total(rows)
        lines.append(f"Capture budget (Section 9; {rate:g} frames/s, {params.move_and_settle_time_s:g} s per move plus settle):")
        lines.append(format_budget_table(rows))
        if c_reuse is not None and c_reuse.poses:
            lines.append(f"  (computed without the reuse of C first frames: the {c_reuse.poses:,} reused D poses are "
                         "counted in the D row, as Section 9 assumes no reuse)")
        lines.append("")
        lines.append(f"Document estimate: {DOCUMENT_ESTIMATE_POSES:,} poses, {DOCUMENT_ESTIMATE_FRAMES:,} frames, "
                     f"{DOCUMENT_ESTIMATE_HOURS:g} h. This plan: {total.poses:,} poses ({total.poses / DOCUMENT_ESTIMATE_POSES:.2f} x), "
                     f"{total.frames:,} frames ({total.frames / DOCUMENT_ESTIMATE_FRAMES:.2f} x), "
                     f"{total.robot_hours:.1f} h ({total.robot_hours / DOCUMENT_ESTIMATE_HOURS:.2f} x).")
        # The optional filters-off repeat is listed on its own: it is outside the main budget and the comparison above.
        # The drift sentinels captured during an optional set are counted with that set (its Sentinels row and the line
        # under its table), never in the totals above.
        off_rows = filters_off_budget(plan, rate, params.move_and_settle_time_s)
        if off_rows:
            lines += ["", "Outside the main budget (filters-off repeat, Section 4, Step 4.2; not in the totals above "
                          "or in the comparison with the document's estimate):", format_budget_table(off_rows),
                      _set_sentinel_line(plan, SUBSERIES_FILTERS_OFF)]
        stair_rows = staircase_budget(plan, rate, params.move_and_settle_time_s)
        if stair_rows:
            lines += ["", "Outside the main budget (optional B-Z staircase, Section 6.2, second pass; not in the totals "
                          "above or in the comparison with the document's estimate):", format_budget_table(stair_rows),
                      _set_sentinel_line(plan, SUBSERIES_STAIRCASE)]
        sweep_rows = lateral_sweep_budget(plan, rate, params.move_and_settle_time_s)
        if sweep_rows:
            lines += ["", "Outside the main budget (optional B-HV lateral sweep, Section 6.1, second pass; not in the "
                          "totals above or in the comparison with the document's estimate):",
                      format_budget_table(sweep_rows), _set_sentinel_line(plan, SUBSERIES_LATERAL_SWEEP)]
        open_rows = open_background_budget(plan, rate, params.move_and_settle_time_s)
        if open_rows:
            lines += ["", "Outside the main budget (optional open-background variant of C, Section 7, Step 4; not in the "
                          "totals above or in the comparison with the document's estimate):",
                      format_budget_table(open_rows), _set_sentinel_line(plan, SUBSERIES_OPEN)]
    if diagnostics is not None and diagnostics.c_reuse is not None:
        reuse = diagnostics.c_reuse
        d_planned = sum(1 for c in plan if c.procedure == PROCEDURE_DETECTION)
        lines += ["", "D reuse of C first frames (--reuse-c-first-frames, Section 8, Reuse): "
                      f"{reuse.poses:,} D poses were taken from C (the first frame of each C pose of the same target, gap "
                      f"and station counts as a D trial: {reuse.per_configuration} per configuration and station at "
                      f"{reuse.configurations} configurations); {d_planned:,} D poses are planned instead of "
                      f"{d_planned + reuse.poses:,}. Only the first frame of a C pose may be used (Section 8, Independence "
                      "rule)."]
    lines += _drift_run_lines(plan, params)
    # The poses of the sweep itself (the sentinels captured during it share its sub-series label but are not sweep poses).
    sweep = [c for c in plan if c.subseries == SUBSERIES_LATERAL_SWEEP and c.procedure == PROCEDURE_EDGES]
    if plan:
        lines += ["", "Approach direction of every pose (specification, Part I): every pose of every series is approached "
                      "along the same direction, from below in Z (nearer the sensor, moving toward larger Z) and from -H and "
                      "-V laterally, so that backlash enters no comparison. poses.csv records it in the notes of every pose "
                      f"and the manifest in the column {APPROACH_DIRECTION_KEY}: '{APPROACH_STANDARD}' for the standard rule."
                      + (" The one exception is the optional lateral sweep (see below), which alternates on purpose."
                         if sweep else " The one exception, the optional lateral sweep, is not in this plan.")
                      + (f" The captures of the optional drift run stand on a fixed stand and carry '{APPROACH_FIXED_STAND}'."
                         if any(c.subseries == SUBSERIES_DRIFT_RUN for c in plan) else "")]
    if sweep:
        lines += ["", f"B-HV lateral sweep approach (Section 6.1, Step 6): {len(sweep)} poses, "
                      f"{sum(1 for c in sweep if c.notes['lateral_sweep_axis'] == 'H')} in H and "
                      f"{sum(1 for c in sweep if c.notes['lateral_sweep_axis'] == 'V')} in V, at "
                      f"Z = {sweep[0].station_z_mm:g} mm. The approach ALTERNATES on purpose so that lateral hysteresis "
                      "shows: odd-numbered poses are approached from the negative side (-H, -V) and even-numbered poses "
                      "from the positive side (+H, +V); the side is in the notes of poses.csv as "
                      f"{APPROACH_DIRECTION_KEY}. Z and the other lateral axis follow the standard rule."]
    if any(c.procedure == PROCEDURE_ZSTEP for c in plan):
        lines += ["", f"Series Z approach: every visit {APPROACH_FROM_BELOW} (back off {params.z_step_approach_overshoot_mm:g} mm "
                      "toward smaller Z, then move up onto the pose), so that backlash does not enter the A / B difference.",
                  f"Series Z pose log: write x_mm, y_mm, z_mm with at least {MIN_POSE_LOG_DECIMALS} decimals; the "
                  "read-back pose is the step truth."]
    lines += _zstep_lines(plan, params)
    adjustments = _adjustment_lines(plan)
    lines += ["", f"Field-offset adjustments (Section 5, Step 1: the target must fit the field; margin = "
                  f"BOUNDARY_BAND_HALF_WIDTH_PX = {params.boundary_band_half_width_px:g} px inside the image, plus half "
                  f"the {params.phase_jitter_span_px:g} px phase-jitter span for the jittered series; the fraction kept "
                  f"of the requested offset is in the notes of poses.csv as {FIELD_FRACTION_ACHIEVED_KEY}): "
                  f"{len(adjustments)}"]
    lines += adjustments if adjustments else ["  none"]
    skipped = [] if diagnostics is None else diagnostics.skipped
    lines += ["", f"Skipped poses (left out of the plan): {len(skipped)}"]
    lines += [f"  {message}" for message in skipped] if skipped else ["  none"]
    if diagnostics is not None and diagnostics.notes:
        lines += ["", "Notes:"] + [f"  {n}" for n in diagnostics.notes]
    warnings = [] if diagnostics is None else diagnostics.warnings
    lines += ["", f"Warnings: {len(warnings)}"]
    lines += [f"  WARNING: {w}" for w in warnings] if warnings else ["  none"]
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# plan.png
# ---------------------------------------------------------------------------
def write_plan_figure(path: str | Path, plan: Sequence[PlannedCapture], params: CharacterizationParameters,
                      geometry: SensorGeometry | None) -> Path:
    """Side view (H versus Z) and front view (H versus V, V down as in the image) of the planned target centers per
    series, with the frustum edges when the geometry is known. Raises ModuleNotFoundError if matplotlib is missing."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, (side, front) = plt.subplots(1, 2, figsize=PLOT_FIGURE_SIZE_IN)
    line = dict(color=PLOT_FRUSTUM_COLOR, linewidth=PLOT_FRUSTUM_LINE_WIDTH)
    if geometry is not None and geometry.image_width_px is not None and geometry.sensor_fx_px is not None:
        half_far = geometry.half_field_mm(params.z_max_mm)
        half_near = geometry.half_field_mm(params.z_min_mm)
        for sign in (-1.0, 1.0):
            side.plot([0.0, sign * half_far[0]], [0.0, params.z_max_mm], **line)
        side.plot([-half_near[0], half_near[0]], [params.z_min_mm] * 2, linestyle=":", **line)
        side.plot([-half_far[0], half_far[0]], [params.z_max_mm] * 2, linestyle=":", **line)
        for half in (half_near, half_far):
            front.plot([-half[0], half[0], half[0], -half[0], -half[0]], [-half[1], -half[1], half[1], half[1], -half[1]],
                       **line)
    for procedure in SERIES_ORDER:
        members = [c for c in plan if c.procedure == procedure]
        if not members:
            continue
        color, marker = PLOT_SERIES_STYLE[procedure]
        centers = np.array([c.target_to_camera.translation for c in members])
        for axes, columns in ((side, (0, 2)), (front, (0, 1))):
            axes.scatter(centers[:, columns[0]], centers[:, columns[1]], s=PLOT_MARKER_SIZE, marker=marker, c=color,
                         alpha=PLOT_MARKER_ALPHA, label=f"{SERIES_LABELS[procedure]} ({len(members)})")
    side.set(xlabel="camera H (mm)", ylabel="camera Z, depth (mm)", title="Side view (H versus Z)")
    front.set(xlabel="camera H (mm)", ylabel="camera V (mm, down)", title="Front view (H versus V)")
    front.invert_yaxis()
    for axes in (side, front):
        axes.set_aspect("equal", adjustable="datalim")
        axes.grid(True, alpha=PLOT_GRID_ALPHA)
    side.legend(loc="lower right", fontsize="small")
    figure.tight_layout()
    figure.savefig(path, dpi=PLOT_DPI)
    plt.close(figure)
    return Path(path)


# ---------------------------------------------------------------------------
# write_plan
# ---------------------------------------------------------------------------
def write_plan(path_dir: str | Path, plan: Sequence[PlannedCapture], registration: Registration | None = None,
               params: CharacterizationParameters | None = None, geometry: SensorGeometry | None = None,
               diagnostics: PlanDiagnostics | None = None) -> list[Path]:
    """Write poses.csv (one row per PlannedCapture; with a registration also the flange pose to command in the robot
    base frame), plan_summary.txt and plan.png into ``path_dir`` (created if needed) and return the paths written.

    poses.csv columns: the Section 9 identity, frames, sub-series, seed, offsets, tilt, step, visit, level, order, the
    target pose in the camera frame (target_x_mm .. target_rz_deg: x, y, z in mm and the rotation vector in degrees),
    the notes as JSON and, with a registration, base_x_mm .. base_rz_deg (flange -> base), r00..r22 and quat_w..quat_z.
    If matplotlib is missing, plan.png is skipped with a warning on stderr (and in the summary).
    The pose log the robot writes for series Z must carry x_mm, y_mm, z_mm with at least MIN_POSE_LOG_DECIMALS decimals (0.01 mm): the
    read-back pose is the step truth, and plan_summary.txt says so (``build_manifest`` warns otherwise)."""
    directory = Path(path_dir)
    directory.mkdir(parents=True, exist_ok=True)
    params = CharacterizationParameters() if params is None else params
    diagnostics = PlanDiagnostics() if diagnostics is None else diagnostics
    written: list[Path] = []
    columns, rows = plan_rows(plan, registration)
    csv_path = directory / PLAN_CSV_NAME
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(columns)
        writer.writerows(rows)
    written.append(csv_path)
    figure_path = directory / PLAN_FIGURE_NAME
    try:
        write_plan_figure(figure_path, plan, params, geometry)
        written.append(figure_path)
    except ModuleNotFoundError as error:
        message = (f"{PLAN_FIGURE_NAME} not written because the plotting package is missing ({error}); install "
                   "matplotlib (pip install matplotlib) to get the picture")
        diagnostics.warn(message)
        print(f"WARNING: {message}", file=sys.stderr)
    summary_path = directory / PLAN_SUMMARY_NAME
    summary_path.write_text(plan_summary_text(plan, params, geometry, diagnostics, registration), encoding="utf-8")
    written.append(summary_path)
    return written
