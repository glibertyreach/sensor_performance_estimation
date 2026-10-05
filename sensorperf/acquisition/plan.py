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
    insert_sentinels         Section 5, Step 3   (S, drift sentinels on the budget clock)
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
    stored in ``notes["field_placement"]`` and listed in plan_summary.txt.
    Targets with features (the raised square, the window, the arrays) must have
    every feature outline inside the image by the edge margin. A plate (T2)
    is judged per image axis: it must lie inside the image by the edge margin or,
    when it is larger than the field at that depth, cover the whole image across
    that axis, so that no plate edge falls into the border band of the analysis
    region of interest. The edge margin is BOUNDARY_BAND_HALF_WIDTH_PX (the ROI
    shrink of Analysis A), plus half the phase-jitter span for jittered series.
"""
from __future__ import annotations

import csv
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from sensorperf.geometry.camera import PinholeCamera
from sensorperf.geometry.registration import Registration
from sensorperf.geometry.targets import (
    FEATURE_POST, TARGET_KIND_PLATE, TargetSet, TwoPlaneTarget, fronto_parallel_pose, make_noise_plate,
    make_standard_target_set, tilted_pose,
)
from sensorperf.geometry.transforms import RigidTransform
from sensorperf.io.manifest import (
    SUBSERIES_EXTENDED, SUBSERIES_FIELD, SUBSERIES_FILTERS_OFF, SUBSERIES_JITTER, SUBSERIES_LADDER, SUBSERIES_MAIN,
    SUBSERIES_NOMINAL, SUBSERIES_OPEN, SUBSERIES_REMOUNT, SUBSERIES_SENTINEL, SUBSERIES_STAIRCASE, SUBSERIES_TILT,
    TARGET_POSE_COLUMNS, TILT_AXIS_H, TILT_AXIS_V, VISIT_A, VISIT_B, format_file_name, pose_to_six,
    six_to_pose,
)
from sensorperf.parameters import (
    CharacterizationParameters, FIELD_POSITION_CENTER, FIELD_POSITION_CODES, FIELD_POSITION_SIGNS,
    PROCEDURE_AREA, PROCEDURE_DETECTION, PROCEDURE_EDGES, PROCEDURE_NOISE,
    PROCEDURE_REGISTRATION, PROCEDURE_SENTINEL, PROCEDURE_ZSTEP, SensorGeometry, TARGET_CUTOUTS,
    TARGET_DISKS, TARGET_NOISE_PLATE, TARGET_RAISED_SQUARE, TARGET_SQUARE_WINDOW,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
SEED_UPPER_BOUND = 2 ** 31 - 1
"""Exclusive upper bound of the seeds drawn from the master generator (a 31-bit
integer, so a seed survives a CSV round trip and any 32-bit consumer)."""
FILTERS_OFF_POSE_INDEX_BASE = 100
"""First pose index of the filters-off repeat of series A (Section 5, Step 7). The
file-name rule has no field for the filter state, so the repeat takes pose
indices above those of the filters-on series at the same station."""
TIER_A_DISPARITY_QUANTUM_PX = 0.125
"""Disparity quantum q assumed for the expected depth quantum dZ_q = q Z^2 / k of the
B-Z staircase until Analysis A measures one (Section 6.2, Step 3: "using the Tier-A q").
The value (1/8 px) is the indicative one of the synthetic sensor model and is NOT a
datasheet value."""
FIELD_PULL_TOLERANCE = 1.0e-3
"""Resolution of the fraction of the requested field offset kept when a pose is pulled
inward (bisection stops when the bracket is narrower than this)."""
FIT_VIOLATION_TOLERANCE_PX = 1.0e-6
"""A fit violation at or below this many pixels counts as fitting (numerical guard)."""
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

DOCUMENT_ESTIMATE_POSES = 7292
DOCUMENT_ESTIMATE_FRAMES = 44140
DOCUMENT_ESTIMATE_HOURS = 7.3
"""The capture-budget estimate printed in Section 9 of the procedure document (poses, frames, robot hours) for the
redesigned plan: the totals of ``plan_full_session`` with the default parameters, the indicative geometry (10
frames/s) and no optional variants (no filters-off repeat, no open-background variant; 7,292 poses, 44,140 frames,
7.30 h). plan_summary.txt compares the plan it summarizes with these numbers, so a change of the parameters shows
up as a ratio away from 1."""

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
the step truth of series Z, and the smallest rungs are tens of micrometers (``pose_log.build_manifest`` warns)."""
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

    def pose_key(self) -> tuple:
        """The key shared by all frames of this pose (same as FrameRecord.pose_key)."""
        return (self.procedure, self.target_id, self.gap_mm, self.station_z_mm, self.field, self.pose_index)

    def file_name(self, frame_index: int) -> str:
        """The Section 9 file name of one frame of this pose."""
        return format_file_name(self.procedure, self.target_id, self.gap_mm, self.station_z_mm, self.field,
                                self.pose_index, frame_index)


# ---------------------------------------------------------------------------
# Diagnostics collected while planning
# ---------------------------------------------------------------------------
@dataclass
class PlanDiagnostics:
    """Warnings and information lines collected while planning, for plan_summary.txt.

    A caller that wants them passes one instance to the planning functions (or
    to :func:`plan_full_session`) and hands it to :func:`write_plan`."""

    warnings: list[str] = field(default_factory=list)
    """Things the technician should read before running the plan."""
    notes: list[str] = field(default_factory=list)
    """Informational lines (assumptions, counts)."""
    master_seed: int | None = None
    """Seed of the master generator, when the caller knows it (logged in the summary)."""

    def warn(self, message: str) -> None:
        """Record a warning once (identical messages are not repeated)."""
        if message not in self.warnings:
            self.warnings.append(message)

    def note(self, message: str) -> None:
        """Record an information line once."""
        if message not in self.notes:
            self.notes.append(message)


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
    """Notes of a pose placed at a field position: the position always, the adjustment record when one was made."""
    notes: dict[str, Any] = {"field_h_mm": placement.h_mm, "field_v_mm": placement.v_mm}
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
def plan_noise_series(params: CharacterizationParameters, geometry: SensorGeometry, rng: np.random.Generator,
                      filters_off: bool = False, with_sentinels: bool = True, targets: TargetSet | None = None,
                      diagnostics: PlanDiagnostics | None = None) -> list[PlannedCapture]:
    """Section 5 (Series A), target T2: main stations, tilt sub-series and repeat-mount check.

    Step 1: the station list is every station of the geometric ladder (``params.z_stations_mm()``, nine
    stations from Z_MIN to Z_MAX) plus the LEGACY_METRIC_DEPTHS_MM (700 and 1000 mm) as extra stations, so the
    legacy metrics are computed at the depths of the existing data (``params.noise_stations_mm()``), at the five
    field positions (center, then the four off-axis ones), all fronto-parallel, FRAMES_PER_NOISE_STATION
    frames each; positions that do not fit the field are pulled inward (see module docstring).
    Step 2: the station list is shuffled with a logged seed. Step 4 (move, settle, capture) is the
    robot's job; the budget counts the settle time. Step 5: at the center and each Z in
    reduced station (``params.z_reduced_stations_mm()``), T2 is tilted about V and then about H through every angle in
    TILT_ANGLES_DEG (FRAMES_PER_TILT_POSE frames each; the zero angle is captured in both
    sweeps, as the procedure lists it). Step 6: the repeat-mount check repeats the center station at
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
    stations = [(z, code) for z in params.noise_stations_mm() for code in FIELD_POSITION_CODES]
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
    for z in params.z_reduced_stations_mm():
        for axis in (TILT_AXIS_V, TILT_AXIS_H):
            for angle in params.tilt_angles_deg:
                pose = tilted_pose(0.0, 0.0, z, axis, angle)
                capture = _new_capture(counter, PROCEDURE_NOISE, TARGET_NOISE_PLATE, None, z, FIELD_POSITION_CENTER,
                                       params.frames_per_tilt_pose, SUBSERIES_FILTERS_OFF if filters_off
                                       else SUBSERIES_TILT, pose, tilt_axis=axis, tilt_deg=float(angle))
                plan.append(capture)
                camera = camera_of(geometry)
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
                                diagnostics=diagnostics)
    return plan


# ---------------------------------------------------------------------------
# Section 6.1: edge series
# ---------------------------------------------------------------------------
def plan_edge_series(params: CharacterizationParameters, geometry: SensorGeometry, rng: np.random.Generator,
                     targets: TargetSet | None = None,
                     diagnostics: PlanDiagnostics | None = None) -> list[PlannedCapture]:
    """Section 6.1 (Series B-HV): T3a (raised square) then T3b (square window), each with G =
    GAP_SMALL_MM and then GAP_LARGE_MM. The slant of the square is part of the target definition.

    At each shape station (``params.z_shape_stations_mm()``), centered and fronto-parallel: Step 2, FRAMES_PER_EDGE_POSE
    frames at the nominal pose (subseries "nominal"); Step 3, PHASE_JITTER_POSES_EDGE further poses,
    each with its own logged random lateral offset uniform over +/- PHASE_JITTER_SPAN_PX / 2 in both
    H and V (subseries "jitter"), FRAMES_PER_EDGE_POSE frames each. Steps 4 and 5 are the loop over the
    two gaps and the two targets; stations run in ascending Z."""
    target_set = _targets(params, geometry, targets)
    counter = _PoseCounter()
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
                                         params.frames_per_edge_pose, SUBSERIES_NOMINAL, nominal))
                for _ in range(params.phase_jitter_poses_edge):
                    plan.append(_jitter_capture(counter, params, geometry, rng, PROCEDURE_EDGES, target_id, gap, z,
                                                FIELD_POSITION_CENTER, params.frames_per_edge_pose,
                                                SUBSERIES_JITTER, 0.0, 0.0))
    return _renumber(plan)


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
    """The expected depth quantum dZ_q(Z) = q Z^2 / k (Section 2) with the Tier-A disparity
    quantum, as a function of the station depth in mm."""
    return lambda depth_mm: geometry.depth_quantum_mm(disparity_quantum_px, depth_mm)


def plan_zstep_series(params: CharacterizationParameters, geometry: SensorGeometry, rng: np.random.Generator,
                      expected_quantum_mm: Callable[[float], float] | float | None = None,
                      targets: TargetSet | None = None,
                      diagnostics: PlanDiagnostics | None = None) -> list[PlannedCapture]:
    """Section 6.2 (Series B-Z, procedure letter Z), target T2 centered and fronto-parallel at each Z0
    in the reduced stations (``params.z_reduced_stations_mm()``, Step 1).

    Step 2, step ladder: for each delta in Z_STEP_LADDER_MM the target alternates between Z0 and
    Z0 + delta for Z_STEP_REPEATS cycles (A, B, A, B, ...). Each visit is its own pose with
    FRAMES_PER_ZSTEP_POSE frames, sub-series "ladder", visit "A" or "B" and step_mm = delta for both
    visits; the commanded displacement is 0 for A and delta for B (``notes["displacement_mm"]``, and the
    target pose carries it in its Z). The ground truth of the
    step is the read-back robot pose carried into the camera frame (the analysis takes the difference of the registered
    front-plane depth of the two visits).
    Every visit of series Z is approached from below (``notes["approach"]``): the robot backs off by
    ``params.z_step_approach_overshoot_mm`` toward smaller Z and then moves up onto the pose, so backlash does not
    enter the A / B difference.
    Step 3, fine staircase: from Z0 to Z0 + Z_STAIRCASE_QUANTA x dZ_q in steps of dZ_q /
    Z_STAIRCASE_SUBDIVISION (both ends included), Z_STAIRCASE_FRAMES frames per step, sub-series
    "staircase", step_mm = the displacement from Z0. dZ_q comes from ``expected_quantum_mm`` (a function
    of Z, or one number in mm; default: the Tier-A disparity quantum, TIER_A_DISPARITY_QUANTUM_PX).
    Step 4: the Z0 visits of the ladder serve as the no-step reference, so no extra poses are planned.

    No random draws occur in this series; ``rng`` is accepted so all planners share one signature."""
    del rng, targets                       # no randomness and no target geometry needed here
    if expected_quantum_mm is None:
        quantum_at = default_expected_quantum_mm(geometry)
    elif callable(expected_quantum_mm):
        quantum_at = expected_quantum_mm
    else:
        constant = float(expected_quantum_mm)
        quantum_at = lambda depth_mm: constant      # noqa: E731 (a constant quantum at every depth)
    counter = _PoseCounter()
    plan: list[PlannedCapture] = []
    for z0 in params.z_reduced_stations_mm():
        for delta in params.z_step_ladder_mm:
            for _ in range(params.z_step_repeats):
                for visit, displacement in ((VISIT_A, 0.0), (VISIT_B, float(delta))):
                    plan.append(_new_capture(
                        counter, PROCEDURE_ZSTEP, TARGET_NOISE_PLATE, None, z0, FIELD_POSITION_CENTER,
                        params.frames_per_zstep_pose, SUBSERIES_LADDER,
                        fronto_parallel_pose(0.0, 0.0, z0 + displacement), step_mm=float(delta), visit=visit,
                        notes={"displacement_mm": displacement, "approach": APPROACH_FROM_BELOW,
                               "approach_overshoot_mm": params.z_step_approach_overshoot_mm}))
        quantum = float(quantum_at(z0))
        step = quantum / params.z_staircase_subdivision
        steps = int(round(params.z_staircase_quanta * params.z_staircase_subdivision))
        if diagnostics is not None:
            diagnostics.note(f"B-Z staircase at Z0 = {z0:g} mm: expected depth quantum {quantum:.3f} mm "
                             f"(Tier-A q = {TIER_A_DISPARITY_QUANTUM_PX:g} px unless a measured value was given), "
                             f"{steps + 1} steps of {step:.4f} mm")
        for k in range(steps + 1):
            displacement = k * step
            plan.append(_new_capture(
                counter, PROCEDURE_ZSTEP, TARGET_NOISE_PLATE, None, z0, FIELD_POSITION_CENTER,
                params.z_staircase_frames, SUBSERIES_STAIRCASE, fronto_parallel_pose(0.0, 0.0, z0 + displacement),
                step_mm=displacement, notes={"displacement_mm": displacement, "staircase_step": k,
                                             "approach": APPROACH_FROM_BELOW,
                                             "approach_overshoot_mm": params.z_step_approach_overshoot_mm}))
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
    back plate removed (gap None), Step 2 repeated (sub-series "open"). Step 5 (drift sentinels) is
    :func:`insert_sentinels`."""
    target_set = _targets(params, geometry, targets)
    counter = _PoseCounter()
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
                plan.append(_jitter_capture(counter, params, geometry, rng, PROCEDURE_AREA, target_id, None, z_ref,
                                            FIELD_POSITION_CENTER, params.frames_per_area_pose, SUBSERIES_OPEN,
                                            0.0, 0.0))
    return _renumber(plan)


# ---------------------------------------------------------------------------
# Section 8: detection series
# ---------------------------------------------------------------------------
def plan_detection_series(params: CharacterizationParameters, geometry: SensorGeometry, rng: np.random.Generator,
                          extended: bool = True, targets: TargetSet | None = None,
                          diagnostics: PlanDiagnostics | None = None,
                          shuffle_within_mounting: bool = True) -> list[PlannedCapture]:
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
    Extended trials for the 0 percent point (``extended=True``): at the DETECTION_ZERO_STATION_COUNT farthest stations
    (``params.detection_zero_stations_mm()``: 1131, 1345 and 1600 mm), where the smallest feature lies below the
    expected threshold, the pose count is raised to DETECTION_ZERO_TRIALS for every configuration (the additional poses
    carry sub-series "extended"). Reuse: the first frame of each C pose at a matching configuration may count as a D
    trial; the plan does not reduce the D poses for it, so the technician or the analysis may drop them.

    The D pilot keeps only the post check (post-only sites of the disk plate are not detected, see
    ``acquisition.check.pilot_post_check``); the former level selection from a pilot D_50 is gone, since the levels are
    fixed by the feature ladder and the station ladder."""
    target_set = _targets(params, geometry, targets)
    camera = camera_of(geometry)
    counter = _PoseCounter()
    plan: list[PlannedCapture] = []
    zero_stations = {int(round(z)) for z in params.detection_zero_stations_mm()}
    series_order_seed = derive_seed(rng)

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
                total = params.detection_zero_trials if (extended and int(round(z)) in zero_stations) \
                    else params.detection_trials_per_level
                if not pose_fits_field(camera, target, fronto_parallel_pose(0.0, 0.0, z),
                                       _fit_margin_px(params, jittered=True)):
                    _warn(diagnostics, f"D: {target_id} (G = {gap:g} mm) does not fit the field of view at Z = {z:g} "
                                       "mm with the phase-jitter margin; features may be cut off")
                for trial in range(total):
                    label = SUBSERIES_JITTER if trial < params.detection_trials_per_level else SUBSERIES_EXTENDED
                    block.append(_jitter_capture(counter, params, geometry, rng, PROCEDURE_DETECTION, target_id, gap, z,
                                                 FIELD_POSITION_CENTER, params.frames_per_detection_trial, label,
                                                 0.0, 0.0))
        if shuffle_within_mounting:
            plan.extend(shuffled(block, derive_seed(rng)))
        else:
            plan.extend(block)
    if not shuffle_within_mounting:
        plan = shuffled(plan, series_order_seed)
    return _renumber(plan)


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


def insert_sentinels(plan: list[PlannedCapture], params: CharacterizationParameters, geometry: SensorGeometry,
                     seconds_per_frame: float, move_settle_s: float,
                     sentinel_target_id: str = TARGET_NOISE_PLATE,
                     diagnostics: PlanDiagnostics | None = None) -> list[PlannedCapture]:
    """Section 5, Step 3: "Before the first station, capture a drift sentinel: center, Z = Z_REFERENCE_MM (800 mm), 30 frames.
    Repeat the sentinel every DRIFT_SENTINEL_INTERVAL_MIN and after the last station." (Section 7, Step 5 and
    Section 8 ask for the same cadence in their series.)

    Walks the plan with a clock that adds, per capture, the move plus settle time and the frames at
    ``seconds_per_frame`` (the Section 9 budget model). A sentinel (procedure S, target T2, gap None, field 0, Z =
    Z_REFERENCE_MM, SENTINEL_FRAMES frames, sub-series "sentinel") is placed before the first non-registration
    capture, before any capture that starts when at least DRIFT_SENTINEL_INTERVAL_MIN of clock time has passed since
    the previous sentinel began, and after the last capture. Registration poses (Section 4) come before the first
    sentinel and are not counted. Existing sentinels in ``plan`` are dropped first, so the function can be
    applied to a plan again after it was changed. Order is renumbered 0..N-1; the input list is not modified.

    The sentinel is the T2 noise plate as the procedure defines it. In series that mount another target (B, C,
    D) a sentinel therefore needs a re-mount of T2; the number of such sentinels is noted in ``diagnostics``.
    ``geometry`` is used to check that the plate fits the field at the sentinel station."""
    interval_s = params.drift_sentinel_interval_min * SECONDS_PER_MINUTE
    pose = fronto_parallel_pose(0.0, 0.0, params.z_reference_mm)
    sentinel_counter = _PoseCounter()

    def sentinel() -> PlannedCapture:
        """A new sentinel pose with the next free pose index."""
        return _new_capture(sentinel_counter, PROCEDURE_SENTINEL, sentinel_target_id, None, params.z_reference_mm,
                            FIELD_POSITION_CENTER, params.sentinel_frames, SUBSERIES_SENTINEL, pose)

    sentinel_duration = capture_duration_s_for_frames(params.sentinel_frames, seconds_per_frame, move_settle_s)
    result: list[PlannedCapture] = []
    since_last: float | None = None            # clock time since the last sentinel began; None before the first
    for capture in (c for c in plan if c.procedure != PROCEDURE_SENTINEL):
        if capture.procedure == PROCEDURE_REGISTRATION:
            result.append(capture)
            continue
        if since_last is None or since_last >= interval_s:
            result.append(sentinel())
            since_last = sentinel_duration
        result.append(capture)
        since_last += capture_duration_s(capture, seconds_per_frame, move_settle_s)
    if since_last is not None:
        result.append(sentinel())
    _renumber(result)
    if diagnostics is not None:
        count = sum(c.procedure == PROCEDURE_SENTINEL for c in result)
        diagnostics.note(f"{count} drift sentinels (every {params.drift_sentinel_interval_min:g} min of estimated "
                         f"clock, {params.sentinel_frames} frames, {sentinel_target_id} at Z = "
                         f"{params.z_reference_mm:g} mm center)")
        # A sentinel whose neighbors (the captures just before and after it) both mount another target needs a re-mount.
        remounts = 0
        for position, capture in enumerate(result):
            if capture.procedure != PROCEDURE_SENTINEL:
                continue
            neighbors = [result[i].target_id for i in (position - 1, position + 1)
                         if 0 <= i < len(result) and result[i].procedure != PROCEDURE_SENTINEL]
            if neighbors and all(t != sentinel_target_id for t in neighbors):
                remounts += 1
        if remounts:
            diagnostics.warn(f"{remounts} of {count} drift sentinels fall between captures of another target; each "
                             f"needs {sentinel_target_id} mounted (a re-mount), as the procedure defines the sentinel "
                             "on the noise plate")
        if not pose_fits_field(camera_of(geometry), make_noise_plate(params), pose,
                               _fit_margin_px(params, jittered=False)):
            diagnostics.warn("the sentinel plate does not fit the field of view at the sentinel station")
    return result


# ---------------------------------------------------------------------------
# The whole session
# ---------------------------------------------------------------------------
def plan_full_session(params: CharacterizationParameters, geometry: SensorGeometry, rng: np.random.Generator,
                      registration: Registration | None = None, expected_quantum_mm: Callable[[float], float] | float | None = None,
                      filters_off: bool = False, open_background: bool = False, extended: bool = True,
                      targets: TargetSet | None = None, series: Iterable[str] | None = None,
                      diagnostics: PlanDiagnostics | None = None) -> list[PlannedCapture]:
    """All series in the order of the procedure (R, A, B-HV, B-Z, C, D) with drift sentinels inserted once over
    the whole plan and ``order`` renumbered 0..N-1.

    ``series`` selects a subset by procedure letter (any of R, A, B, Z, C, D; default all). ``filters_off``
    appends the filters-off repeat of Series A (Section 5, Step 7) to A. ``registration`` is only checked here: a
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
        parts += plan_edge_series(params, geometry, rng, target_set, diagnostics)
    if PROCEDURE_ZSTEP in chosen:
        parts += plan_zstep_series(params, geometry, rng, expected_quantum_mm, target_set, diagnostics)
    if PROCEDURE_AREA in chosen:
        parts += plan_area_series(params, geometry, rng, open_background, target_set, diagnostics)
    if PROCEDURE_DETECTION in chosen:
        parts += plan_detection_series(params, geometry, rng, extended, target_set, diagnostics)
    return insert_sentinels(parts, params, geometry, _seconds_per_frame(geometry), params.move_and_settle_time_s,
                            diagnostics=diagnostics)


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


def capture_budget(plan: Sequence[PlannedCapture], frame_rate_hz: float, move_settle_s: float) -> list[BudgetRow]:
    """The Section 9 table: poses, frames and robot hours per series, in the order of the procedure.
    "The estimate assumes 10 frames/s and 3 s per move plus settle": hours = (poses x move_settle_s + frames /
    frame_rate_hz) / 3600. Series without poses are omitted."""
    rows = []
    for procedure in SERIES_ORDER:
        members = [c for c in plan if c.procedure == procedure]
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
              "  A legacy-metric extra stations: " + ", ".join(f"{z:g}" for z in params.noise_stations_mm()
                                                               if z not in stations),
              "  D zero-detection stations (extra trials): "
              + ", ".join(f"{z:g}" for z in params.detection_zero_stations_mm()),
              "  Feature diameters (mm): " + ", ".join(f"{d:.1f}" for d in params.feature_diameters_mm(geometry))
              if geometry is not None and geometry.sensor_fx_px is not None else "  Feature diameters: geometry unknown"]
    lines += ["", f"Poses planned: {len(plan)}; frames: {sum(c.frames for c in plan)}", "",
              "Poses per station depth and series (R, the registration poses, span Z_MIN to Z_MAX and are not listed by station):"]
    lines += _station_table(plan)
    lines += ["", "Poses and frames per sub-series:"] + _subseries_lines(plan)
    lines.append("")
    rate = None if geometry is None else geometry.frame_rate_hz
    if rate is None:
        lines.append("Capture budget (Section 9): not computed, the sensor frame rate is not known (a † value of Section 2).")
    else:
        rows = capture_budget(plan, rate, params.move_and_settle_time_s)
        total = budget_total(rows)
        lines.append(f"Capture budget (Section 9; {rate:g} frames/s, {params.move_and_settle_time_s:g} s per move plus settle):")
        lines.append(format_budget_table(rows))
        lines.append("")
        lines.append(f"Document estimate: {DOCUMENT_ESTIMATE_POSES:,} poses, {DOCUMENT_ESTIMATE_FRAMES:,} frames, "
                     f"{DOCUMENT_ESTIMATE_HOURS:g} h. This plan: {total.poses:,} poses ({total.poses / DOCUMENT_ESTIMATE_POSES:.2f} x), "
                     f"{total.frames:,} frames ({total.frames / DOCUMENT_ESTIMATE_FRAMES:.2f} x), "
                     f"{total.robot_hours:.1f} h ({total.robot_hours / DOCUMENT_ESTIMATE_HOURS:.2f} x).")
    if any(c.procedure == PROCEDURE_ZSTEP for c in plan):
        lines += ["", f"Series Z approach: every visit {APPROACH_FROM_BELOW} (back off {params.z_step_approach_overshoot_mm:g} mm "
                      "toward smaller Z, then move up onto the pose), so that backlash does not enter the A / B difference.",
                  f"Series Z pose log: write x_mm, y_mm, z_mm with at least {MIN_POSE_LOG_DECIMALS} decimals; the "
                  "read-back pose is the step truth."]
    adjustments = _adjustment_lines(plan)
    lines += ["", f"Field-offset adjustments (Section 5, Step 1: the target must fit the field): {len(adjustments)}"]
    lines += adjustments if adjustments else ["  none"]
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
