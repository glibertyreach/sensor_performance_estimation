"""
Analysis B-Z: effective resolution in depth (procedure document, Section 11.2), from the Z-step series
(procedure "Z"): the noise plate T2 at a station Z0, a step ladder (the plate alternates between Z0, visit A, and
Z0 + delta, visit B, for several delta) and a fine staircase.

Steps (per station Z0)
  1  The ladder visits are put in acquisition order (timestamp, then pose index). For each consecutive A -> B visit pair
     of one step size, Delta = mean(Z at B) - mean(Z at A) for patches of ``zstep_patch_sizes_px`` (1 px, 5 x 5,
     20 x 20), tiled over the region of interest (non-overlapping tiles fully inside the ROI of both visits; the
     patch means of the frame-mean depth images are box filters, ``scipy.ndimage.uniform_filter``, sampled at the tile
     centers). The true step is the difference of the registered front-plane depth of the two visits along the
     optical axis: the z component of the manifest's ``target_pose_camera`` (the read-back robot pose carried into the
     camera frame through the registration) at B minus at A. For a fronto-parallel plate, which series Z uses, the
     front plane is perpendicular to the optical axis and the z component of the target pose is its depth. The
     commanded ``step_mm`` is only the label of the rung; it is never the truth.
  2  Regression of the mean Delta on the true step over the ladder: slope (gain), intercept, largest deviation. Rungs
     whose true step is below ``robot_repeatability_mm`` (the step truth is then no better than the robot itself) get
     ``truth_reliable`` = False: they are excluded from this regression, kept in the detection curve of Step 4, and the
     note says how many rungs were flagged. The registration enters only through the DIFFERENCE of two registered
     poses: its translation cancels and a rotation error acts through the cosine of the angle error (negligible), so
     the step truth is as good as the robot's relative motion accuracy.
     The robot's own read-back scatter is reported from the A -> A pairs (``robot_readback_repeatability_mm``, per
     station) with the mean read-back minus commanded step of the A -> B pairs; a station whose scatter exceeds
     ``robot_repeatability_mm`` is flagged.
  3  The null distribution: Delta of consecutive A -> A visits (same Z0, no commanded step) per patch size;
     tau is its (1 - DETECTION_FALSE_ALARM_TARGET) quantile. Delta and -Delta are both used, so that tau does not
     depend on the sign of the commanded step.
  4  A step counts as detected if Delta > tau in the commanded direction. The detection fraction per step size, pooled
     over pairs and tiles, is fitted with the psychometric model of Section 13 (``stats.psychometric``, logistic, on
     ln delta, guess rate = the measured A -> A false-alarm fraction, lapse rate at most DETECTION_LAPSE_RATE_MAX);
     delta_50 is the step at 50 percent corrected detection; when it falls below the smallest rung with a reliable
     truth it is reported as an upper bound (``delta_50_is_bound``, delta_50_mm = that rung): below the robot's
     repeatability the ladder cannot measure it. Its confidence interval is a bootstrap over ABAB cycles
     (tau and the guess rate are held fixed). The scaling of delta_50 with patch size is compared with the
     1 / sqrt(area) = 1 / side expected for uncorrelated noise.
  5  The staircase (sub-series "staircase", ordered by the commanded displacement): sensed Z of single pixels and of the
     20 x 20 patch against the read-back Z of each step (the z component of the target pose, whose uncertainty is the
     robot repeatability; a note says when the fine step is smaller than that); the measured quantum from plateau widths of single pixels, compared with
     q Z^2 / k using q of Analysis A (``previous["A"]``) when it is available.

The quantum of Step 5. Plateau widths need a staircase much finer than the quantum, with many steps. When too few
complete plateaus exist (the demonstration plan has three staircase steps) the quantum is the one of the depth levels
of the single-pixel readings pooled over all staircase frames (and a few Z0 frames of the ladder as the zero point), by the
phase-resultant method of Analysis A, Step 8 (``noise.estimate_quantum_phase_resultant``); ``quantum_method`` says which.

Conventions: docs/design/code_design.md Section 4. Millimeters, pixels; arrays are (H, W); no-read is NaN.
"""
from __future__ import annotations

import math
import warnings
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
from scipy import ndimage

from sensorperf.analysis.common import (
    central_patch_mask, new_figure, pose_geometry, region_of_interest, save_figure, write_csv_rows, write_json,
)
from sensorperf.analysis.noise import estimate_quantum_phase_resultant
from sensorperf.io.capture_set import load_stack
from sensorperf.io.manifest import SUBSERIES_LADDER, SUBSERIES_STAIRCASE, VISIT_A, VISIT_B, FrameRecord, group_by_pose, select
from sensorperf.io.session import Session
from sensorperf.parameters import PROCEDURE_ZSTEP
from sensorperf.stats.intervals import bootstrap_statistic
from sensorperf.stats.psychometric import PsychometricFitParameters, fit_psychometric

SUMMARY_CSV_NAME = "Z_resolution_summary.csv"
DETAILS_JSON_NAME = "Z_resolution_details.json"
FIGURE_STEMS = {"detection": "Z_detection_curves", "sensed": "Z_sensed_vs_true", "staircase": "Z_staircase"}
SUMMARY_COLUMNS = (
    "station_z_mm", "patch_px", "gain", "intercept_mm", "max_deviation_mm", "tau_mm", "delta_50_mm",
    "delta_50_lower_mm", "delta_50_upper_mm", "quantum_mm", "quantum_predicted_mm", "cycles", "quantum_method",
    "false_alarm_fraction", "tiles_per_pair", "truth_source", "truth_reliable", "delta_50_is_bound",
    "robot_readback_repeatability_mm", "readback_minus_commanded_mm", "robot_scatter_exceeds_spec", "note")
"""Columns of Z_resolution_summary.csv: the Section 11.2 list, then the quantum method, the measured false-alarm
fraction, the tiles per pair, where the true step came from, whether every ladder rung of the station has a reliable
truth (False when any rung is flagged), whether delta_50 is only an upper bound, the robot's read-back scatter from the A -> A
pairs and the mean read-back minus commanded step of the A -> B pairs, whether that scatter exceeds
``robot_repeatability_mm``, and a note."""
RUNGS_CSV_NAME = "Z_resolution_rungs.csv"
RUNG_COLUMNS = (
    "station_z_mm", "patch_px", "step_mm", "true_step_mm", "mean_delta_mm", "detections", "trials",
    "detection_fraction", "readback_minus_commanded_mm", "truth_source", "truth_reliable")
"""Columns of Z_resolution_rungs.csv, one row per station, patch size and ladder rung: the commanded step (the label),
the mean true step, the mean sensed step, the detection counts, the read-back minus commanded step, and ``truth_reliable`` (False for a rung whose true step
is below ``robot_repeatability_mm``: reported, but not part of the gain regression)."""

OKABE_ITO_BLACK = "#000000"
OKABE_ITO_ORANGE = "#E69F00"
OKABE_ITO_SKY_BLUE = "#56B4E9"
OKABE_ITO_BLUISH_GREEN = "#009E73"
OKABE_ITO_BLUE = "#0072B2"
OKABE_ITO_VERMILLION = "#D55E00"
OKABE_ITO_REDDISH_PURPLE = "#CC79A7"
OKABE_ITO_CYCLE = (OKABE_ITO_BLUE, OKABE_ITO_VERMILLION, OKABE_ITO_BLUISH_GREEN, OKABE_ITO_ORANGE,
                   OKABE_ITO_REDDISH_PURPLE, OKABE_ITO_SKY_BLUE, OKABE_ITO_BLACK)
"""Colors used in turn for the series of a figure."""

# ---------------------------------------------------------------------------
# Named numbers that are not procedure parameters
# ---------------------------------------------------------------------------
PAIRS_PER_CYCLE = 2
"""An ABAB cycle holds two A -> B visit pairs; the bootstrap resamples cycles."""
FULL_COVERAGE_TOLERANCE = 1.0e-9
"""A box-filtered window counts as fully inside a mask when its mean is at least 1 minus this."""
MIN_DISTINCT_STEPS_FOR_FIT = 2
"""Fewest distinct step sizes for the gain regression and the psychometric fit."""
MIN_COMPLETE_PLATEAUS = 20
"""Fewest complete plateaus (bounded by a jump on each side) for the plateau-width estimate of the quantum."""
PLATEAU_TOLERANCE_FRACTION = 0.5
"""A change of the single-pixel sensed Z between consecutive staircase steps counts as a jump when it exceeds this
fraction of the expected quantum; smaller changes keep the pixel on its plateau."""
PLATEAU_MIN_STEPS_PER_QUANTUM = 3.0
"""The plateau-width estimate is used only if the staircase has at least this many steps per expected quantum (the
document asks for 10): a plateau cannot be measured with steps as wide as itself."""
PLATEAU_MIN_WIDTH_STEPS = 2.0
"""The plateau-width estimate is used only if the median plateau spans at least this many staircase steps. When the
single-pixel temporal noise is comparable to the quantum, nearly every step registers as a jump and the "plateaus"
are one step wide: the staircase then cannot resolve the quantizer, and the level-based estimate is used instead."""
BASELINE_MAX_POSES = 8
"""At most this many A visits of the ladder serve as the Z0 point of the staircase when the staircase has none."""
PLOTTED_PIXELS = 5
"""Single-pixel curves drawn in the staircase figure (taken along the diagonal of the central patch)."""
SMOOTH_PATCH_FRACTION = 0.25
"""The patch-mean staircase counts as smooth (the quantizer is dithered) when its RMS deviation from a straight line is
below this fraction of the quantum."""
PANEL_WIDTH_IN, PANEL_HEIGHT_IN = 3.8, 3.0
"""Size of one panel of the grid figures, inches."""
BOOTSTRAP_SEED = 20261005
"""Seed of the bootstrap generator (deterministic output)."""
FIT_CURVE_POINTS = 100
"""Points of the smooth fitted curves."""
MIN_FIT_POINTS_FOR_SCALING = 2
"""Fewest patch sizes with a delta_50 for the scaling exponent."""
TRUTH_SOURCE = "read-back robot pose through the registration"
"""Where the true Z step comes from: the manifest's ``target_pose_camera`` (read-back flange pose through the
registration), reported in the ``truth_source`` column."""
TRUTH_COMPARISON_TOLERANCE_MM = 1.0e-9
"""Numerical guard of the comparisons of a true step with the robot repeatability and of a commanded step with zero:
a step that equals the limit up to rounding of the registration product counts as reaching it, mm."""
PAIR_DIFFERENCE_SIGMA_FACTOR = math.sqrt(2.0)
"""The standard deviation of the difference of two independent visits is this factor times the scatter of one visit."""
MIN_PAIRS_FOR_SCATTER = 2
"""Fewest A -> A read-back differences for a standard deviation."""
BOUND_NOTE_ROBOT = "below the robot's repeatability; bound, not a measurement"
"""Note of a delta_50 that falls below the smallest rung whose truth is reliable (smaller rungs were flagged)."""
BOUND_NOTE_LADDER = "below the smallest tested step; bound, not a measurement"
"""Note of a delta_50 that falls below the smallest rung of a ladder in which no rung was flagged."""
OPTICAL_AXIS = 2
"""Index of the optical axis (camera z) in a translation vector."""


# ---------------------------------------------------------------------------
# Options and results
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class DepthResolutionOptions:
    """Choices of Analysis B-Z that are not procedure parameters."""

    plateau_tolerance_fraction: float = PLATEAU_TOLERANCE_FRACTION
    """Fraction of the expected quantum a single-pixel change must exceed to count as a jump."""
    min_complete_plateaus: int = MIN_COMPLETE_PLATEAUS
    """Fewest complete plateaus for the plateau-width quantum."""
    baseline_max_poses: int = BASELINE_MAX_POSES
    """Most ladder A visits used as the Z0 point of the staircase."""
    plotted_pixels: int = PLOTTED_PIXELS
    """Single-pixel curves in the staircase figure."""
    bootstrap_seed: int = BOOTSTRAP_SEED
    """Seed of the bootstrap generator."""
    figure_width_in: float = 7.0
    """Width of the single-panel figures, inches."""
    figure_height_in: float = 4.5
    """Height of the single-panel figures, inches."""


@dataclass
class PatchResult:
    """Steps 1 to 4 for one station and patch size."""

    station_z_mm: float
    patch_px: int
    gain: float = float("nan")
    intercept_mm: float = float("nan")
    max_deviation_mm: float = float("nan")
    tau_mm: float = float("nan")
    false_alarm_fraction: float = float("nan")
    delta_50_mm: float = float("nan")
    delta_50_lower_mm: float = float("nan")
    delta_50_upper_mm: float = float("nan")
    delta_50_is_bound: bool = False
    """True when the fitted delta_50 fell below the smallest reliable rung: ``delta_50_mm`` is then that rung, an upper
    bound, not a measurement."""
    delta_50_fit_mm: float = float("nan")
    """The fitted delta_50 before it was replaced by a bound (equal to ``delta_50_mm`` otherwise)."""
    robot_readback_repeatability_mm: float = float("nan")
    """Scatter of one visit's read-back Z at the station: the standard deviation of the A -> A read-back differences of
    the ladder divided by sqrt(2), mm."""
    readback_minus_commanded_mm: float = float("nan")
    """Mean over the A -> B pairs of the read-back step minus the commanded step, mm."""
    robot_scatter_exceeds_spec: bool = False
    """True when ``robot_readback_repeatability_mm`` exceeds ``robot_repeatability_mm``."""
    cycles: int = 0
    tiles_per_pair: float = float("nan")
    steps_mm: list[float] = field(default_factory=list)
    """Mean true step per ladder step size (the levels of the psychometric fit)."""
    mean_delta_mm: list[float] = field(default_factory=list)
    detections: list[int] = field(default_factory=list)
    trials: list[int] = field(default_factory=list)
    pair_true_mm: list[float] = field(default_factory=list)
    pair_delta_mm: list[float] = field(default_factory=list)
    pair_reliable: list[bool] = field(default_factory=list)
    """For each pair of ``pair_true_mm``: False when the rung's true step is below the robot repeatability."""
    rung_labels_mm: list[float] = field(default_factory=list)
    """Commanded step (``step_mm``, the label of the rung) of each level of ``steps_mm``."""
    rung_readback_offset_mm: list[float] = field(default_factory=list)
    """Mean read-back step minus the commanded step of each level of ``steps_mm``, mm."""
    rung_reliable: list[bool] = field(default_factory=list)
    """``truth_reliable`` of each level of ``steps_mm``: False when the mean true step of the rung is below
    ``robot_repeatability_mm``."""
    flagged_rungs: int = 0
    """Number of rungs with ``truth_reliable`` False."""
    fit_alpha: float = float("nan")
    fit_beta: float = float("nan")
    fit_lapse: float = float("nan")
    fit_converged: bool = False
    bootstrap_failures: int = 0
    note: str = ""


@dataclass
class StaircaseResult:
    """Step 5 for one station."""

    station_z_mm: float
    true_z_mm: list[float] = field(default_factory=list)
    """Read-back Z of each staircase step: the z component of the target pose in the camera frame (read-back robot pose
    through the registration)."""
    patch_mean_mm: list[float] = field(default_factory=list)
    """Sensed Z of the 20 x 20 patch at each staircase point."""
    pixel_curves_mm: list[list[float]] = field(default_factory=list)
    """Sensed Z of a few single pixels at each staircase point."""
    quantum_mm: float = float("nan")
    quantum_method: str = "unavailable"
    quantum_plateau_mm: float = float("nan")
    quantum_levels_mm: float = float("nan")
    quantum_jump_height_mm: float = float("nan")
    plateaus: int = 0
    plateau_width_median_mm: float = float("nan")
    expected_quantum_mm: float = float("nan")
    predicted_quantum_mm: float = float("nan")
    patch_rms_about_line_mm: float = float("nan")
    patch_is_smooth: bool | None = None
    truth_source: str = TRUTH_SOURCE
    note: str = ""


@dataclass
class DepthResolutionResult:
    """Everything Analysis B-Z found."""

    patches: list[PatchResult]
    staircases: list[StaircaseResult]
    scaling: dict[float, dict[str, Any]]
    notes: list[str]
    options: DepthResolutionOptions

    def patch(self, station_z_mm: float, patch_px: int) -> PatchResult | None:
        for p in self.patches:
            if p.station_z_mm == station_z_mm and p.patch_px == patch_px:
                return p
        return None

    def forward_model_terms(self) -> dict[str, Any]:
        """The measured depth quantum per station and delta_50 of the 1-px patch per station (mm), keyed by the station
        depth in mm. An upper bound (``delta_50_is_bound``) is not a measurement and is left out."""
        quanta = {f"{s.station_z_mm:g}": s.quantum_mm for s in self.staircases if math.isfinite(s.quantum_mm)}
        delta = {f"{p.station_z_mm:g}": p.delta_50_mm for p in self.patches
                 if p.patch_px == 1 and math.isfinite(p.delta_50_mm) and not p.delta_50_is_bound}
        return {"depth_quantum_mm": quantum_or_none(quanta), "delta_50_1px_mm": quantum_or_none(delta)}


def quantum_or_none(values: dict[str, float]) -> dict[str, float] | None:
    """The dict, or None if it is empty."""
    return values or None


# ---------------------------------------------------------------------------
# Visits
# ---------------------------------------------------------------------------
@dataclass
class _Visit:
    """One pose of the ladder with what the pair statistics need."""

    record: FrameRecord
    mean_depth: np.ndarray
    roi: np.ndarray
    frames: int
    patch_means: dict[int, np.ndarray]
    """Patch mean of the mean depth image per patch size (NaN where the window is not fully read)."""

    @property
    def visit(self) -> str:
        return self.record.visit


def _time_key(record: FrameRecord) -> tuple:
    try:
        stamp = datetime.fromisoformat(record.timestamp)
    except ValueError:
        stamp = datetime.min
    return stamp, record.pose_index


def patch_mean_image(image: np.ndarray, size: int) -> np.ndarray:
    """Mean of the size x size window centered (for even sizes: at the pixel size // 2 from its top-left corner) on each
    pixel, NaN wherever the window is not completely read. Box filters of the zero-filled image and of the read mask."""
    if size == 1:
        return image.copy()
    finite = np.isfinite(image)
    total = ndimage.uniform_filter(np.where(finite, image, 0.0), size=size, mode="constant", cval=0.0)
    coverage = ndimage.uniform_filter(finite.astype(float), size=size, mode="constant", cval=0.0)
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(coverage >= 1.0 - FULL_COVERAGE_TOLERANCE, total / coverage, np.nan)


def tile_centers(roi: np.ndarray, size: int) -> tuple[np.ndarray, np.ndarray]:
    """(rows, columns) of the centers of non-overlapping size x size tiles fully inside ``roi``. The lattice starts at the
    top-left corner of the ROI bounding box; a tile whose window (top-left row r, column c) is not entirely inside the ROI
    is dropped. The center index of a window is its top-left corner plus size // 2, which is where ``patch_mean_image``
    puts the window's mean."""
    if size == 1:
        return np.nonzero(roi)
    inside = ndimage.uniform_filter(roi.astype(float), size=size, mode="constant", cval=0.0) >= 1.0 - FULL_COVERAGE_TOLERANCE
    rows = np.flatnonzero(roi.any(axis=1))
    cols = np.flatnonzero(roi.any(axis=0))
    half = size // 2
    centers_r, centers_c = [], []
    for r in range(rows[0], rows[-1] + 1 - size + 1, size):
        for c in range(cols[0], cols[-1] + 1 - size + 1, size):
            if inside[r + half, c + half]:
                centers_r.append(r + half)
                centers_c.append(c + half)
    return np.array(centers_r, dtype=int), np.array(centers_c, dtype=int)


def _load_visits(session: Session, records: list[FrameRecord], sizes: tuple[int, ...]) -> list[_Visit]:
    """The ladder poses of one station in acquisition order, with mean depth, ROI and patch means."""
    visits = []
    for _, group in group_by_pose(records).items():
        stack = load_stack(group)
        geometry = pose_geometry(session, stack.record(), stack.camera)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)          # pixels never read give an empty mean (NaN)
            mean = stack.mean_depth()
        visits.append(_Visit(record=stack.record(), mean_depth=mean, roi=region_of_interest(geometry, session.params),
                             frames=stack.frame_count, patch_means={p: patch_mean_image(mean, p) for p in sizes}))
    return sorted(visits, key=lambda v: _time_key(v.record))


def registered_depth_mm(record: FrameRecord) -> float:
    """Depth of the registered front plane of the target along the optical axis, mm: the z component of the manifest's
    ``target_pose_camera`` (read-back robot pose through the registration). The plate of series Z is fronto-parallel, so
    its front plane is perpendicular to the optical axis and the target origin on it has the depth of the whole plane;
    a tilted plate would need the plane's depth at the optical axis instead."""
    return float(record.target_pose_camera.translation[OPTICAL_AXIS])


def _true_step(a: _Visit, b: _Visit) -> float:
    """True step of an A -> B visit pair, mm: the registered front-plane depth at B minus at A. The commanded
    ``step_mm`` is the label of the rung and does not enter."""
    return registered_depth_mm(b.record) - registered_depth_mm(a.record)


def _pair_delta(a: _Visit, b: _Visit, size: int) -> np.ndarray:
    """Delta of every tile of the ROI common to both visits: patch mean at B minus at A."""
    rows, cols = tile_centers(a.roi & b.roi, size)
    delta = b.patch_means[size][rows, cols] - a.patch_means[size][rows, cols]
    return delta[np.isfinite(delta)]


# ---------------------------------------------------------------------------
# Steps 1 to 4 for one station
# ---------------------------------------------------------------------------
def _fit_curve(levels: np.ndarray, detections: np.ndarray, trials: np.ndarray, guess: float, lapse_max: float):
    """The logistic psychometric fit, or None if the data cannot support it."""
    keep = (trials > 0) & np.isfinite(levels)
    if keep.sum() < MIN_DISTINCT_STEPS_FOR_FIT or not 0.0 <= guess < 1.0 - lapse_max:
        return None
    return fit_psychometric(levels[keep], detections[keep], trials[keep], guess,
                            PsychometricFitParameters(lapse_rate_max=lapse_max))


def _analyze_station(session: Session, station: float, visits: list[_Visit], options: DepthResolutionOptions,
                     rng: np.random.Generator, notes: list[str]) -> list[PatchResult]:
    params = session.params
    sizes = tuple(params.zstep_patch_sizes_px)
    step_sizes = sorted({v.record.step_mm for v in visits if v.record.step_mm is not None})
    # Step 1: visit pairs per step size, in acquisition order.
    ab_pairs: dict[float, list[tuple[_Visit, _Visit]]] = {s: [] for s in step_sizes}
    aa_pairs: list[tuple[_Visit, _Visit]] = []
    for step in step_sizes:
        sequence = [v for v in visits if v.record.step_mm == step]
        for first, second in zip(sequence, sequence[1:]):
            if first.visit == VISIT_A and second.visit == VISIT_B:
                ab_pairs[step].append((first, second))
        a_visits = [v for v in sequence if v.visit == VISIT_A]
        aa_pairs.extend(zip(a_visits, a_visits[1:]))
    results = []
    # The robot's own scatter from the read-back poses of consecutive A -> A visits (the plate is commanded to the same
    # Z0): the standard deviation of their registered-depth differences, divided by sqrt(2) for a single visit.
    a_differences = [registered_depth_mm(b.record) - registered_depth_mm(a.record) for a, b in aa_pairs]
    scatter = (float(np.std(a_differences, ddof=1)) / PAIR_DIFFERENCE_SIGMA_FACTOR
               if len(a_differences) >= MIN_PAIRS_FOR_SCATTER else float("nan"))
    exceeds = bool(math.isfinite(scatter) and scatter > params.robot_repeatability_mm)
    scatter_note = ""
    if exceeds:
        scatter_note = (f"the read-back scatter of the robot ({scatter:.3g} mm) exceeds its repeatability of "
                        f"{params.robot_repeatability_mm:g} mm: the step truth is poorer than specified")
        notes.append(f"station {station:g} mm: {scatter_note}")
    for size in sizes:
        out = PatchResult(station_z_mm=station, patch_px=size, robot_readback_repeatability_mm=scatter,
                          robot_scatter_exceeds_spec=exceeds)
        # Step 3: the null distribution and tau (symmetrized so that tau does not depend on the direction).
        null = [_pair_delta(a, b, size) for a, b in aa_pairs]
        null_all = np.concatenate(null) if null else np.array([])
        if null_all.size == 0:
            out.note = "no A -> A visit pairs: no null distribution, no threshold"
            results.append(out)
            continue
        symmetric = np.concatenate([null_all, -null_all])
        out.tau_mm = float(np.quantile(symmetric, 1.0 - params.detection_false_alarm_target))
        out.false_alarm_fraction = float(np.mean(symmetric > out.tau_mm))
        # Steps 1, 2, 4: per step size the Delta of every A -> B pair. The arrays are aligned with step_sizes; a
        # step size without a usable pair has NaN level and zero trials.
        n_steps = len(step_sizes)
        levels = np.full(n_steps, np.nan)
        mean_delta = np.full(n_steps, np.nan)
        detections = np.zeros(n_steps)
        trials = np.zeros(n_steps)
        reliable = np.ones(n_steps, dtype=bool)
        offsets = np.full(n_steps, np.nan)
        pair_steps: list[int] = []
        per_cycle: dict[int, dict[int, tuple[int, int]]] = {}
        tile_counts = []
        for index, step in enumerate(step_sizes):
            true_values, deltas = [], []
            for pair_number, (a, b) in enumerate(ab_pairs[step]):
                true_step = _true_step(a, b)
                delta = _pair_delta(a, b, size)
                if delta.size == 0:
                    continue
                tile_counts.append(delta.size)
                direction = 1.0 if true_step >= 0 else -1.0
                detected = int(np.sum(direction * delta > out.tau_mm))
                detections[index] += detected
                trials[index] += delta.size
                true_values.append(true_step)
                deltas.append(float(np.mean(delta)))
                cycle = per_cycle.setdefault(pair_number // PAIRS_PER_CYCLE, {})
                previous = cycle.get(index, (0, 0))
                cycle[index] = (previous[0] + detected, previous[1] + delta.size)
            if true_values:
                levels[index] = float(np.mean(np.abs(true_values)))
                mean_delta[index] = float(np.mean(deltas))
                offsets[index] = float(np.mean(true_values)) - float(step)
                out.pair_true_mm.extend(true_values)
                out.pair_delta_mm.extend(deltas)
                pair_steps.extend([index] * len(true_values))
                # truth_reliable: the true step reaches the robot's repeatability (Section 3.1, ISO 9283).
                reliable[index] = levels[index] >= params.robot_repeatability_mm - TRUTH_COMPARISON_TOLERANCE_MM
        have = trials > 0
        out.steps_mm, out.mean_delta_mm = levels[have].tolist(), mean_delta[have].tolist()
        out.rung_labels_mm = [float(step_sizes[i]) for i in np.flatnonzero(have)]
        out.rung_reliable = [bool(r) for r in reliable[have]]
        out.pair_reliable = [bool(reliable[i]) for i in pair_steps]
        out.rung_readback_offset_mm = offsets[have].tolist()
        if pair_steps:
            out.readback_minus_commanded_mm = float(np.mean(
                np.array(out.pair_true_mm) - np.array([step_sizes[i] for i in pair_steps])))
        out.flagged_rungs = int(np.sum(~reliable[have]))
        flagged_note = ""
        if out.flagged_rungs:
            flagged_labels = ", ".join(f"{step:g}" for step, ok in zip(out.rung_labels_mm, out.rung_reliable) if not ok)
            flagged_note = (f"{out.flagged_rungs} of {int(have.sum())} ladder rungs flagged truth_reliable = False "
                            f"(commanded {flagged_labels} mm, true step below the robot repeatability of "
                            f"{params.robot_repeatability_mm:g} mm): excluded from the gain regression, kept in the "
                            "detection curve")
        out.detections, out.trials = detections[have].astype(int).tolist(), trials[have].astype(int).tolist()
        out.cycles = len(per_cycle)
        out.tiles_per_pair = float(np.mean(tile_counts)) if tile_counts else float("nan")
        # Step 2: gain, intercept and the largest deviation from the line, over the rungs with a reliable truth.
        keep_pairs = np.array(out.pair_reliable, dtype=bool)
        fit_true = np.array(out.pair_true_mm)[keep_pairs]
        fit_delta = np.array(out.pair_delta_mm)[keep_pairs]
        fit_levels = have & reliable
        if len(set(np.round(fit_true, 9))) >= MIN_DISTINCT_STEPS_FOR_FIT:
            slope, intercept = np.polyfit(fit_true, fit_delta, 1)
            out.gain, out.intercept_mm = float(slope), float(intercept)
            out.max_deviation_mm = float(np.max(np.abs(mean_delta[fit_levels] - (intercept + slope * levels[fit_levels]))))
        elif out.flagged_rungs:
            flagged_note += "; too few reliable rungs for the gain regression"
        # Step 4: the psychometric fit and its bootstrap over cycles.
        lapse = params.detection_lapse_rate_max
        fit = _fit_curve(levels, detections, trials, out.false_alarm_fraction, lapse)
        if fit is None:
            out.note = "too few step sizes (or a false-alarm rate too high) for the psychometric fit"
        else:
            out.fit_alpha, out.fit_beta, out.fit_lapse, out.fit_converged = fit.alpha, fit.beta, fit.lapse_rate, fit.converged
            out.delta_50_mm = out.delta_50_fit_mm = fit.threshold(0.5)
            usable = have & reliable
            if usable.any() and out.delta_50_mm < levels[usable].min():
                # Below the smallest rung whose truth is reliable the ladder cannot measure delta_50: an upper bound.
                out.delta_50_is_bound = True
                out.delta_50_mm = float(levels[usable].min())
                bound_note = BOUND_NOTE_ROBOT if out.flagged_rungs else BOUND_NOTE_LADDER
                out.note = "; ".join(x for x in (out.note, bound_note) if x)
            elif not (levels[have].min() <= out.delta_50_mm <= levels[have].max()):
                out.note = "delta_50 lies outside the tested steps (extrapolated)"
            cache: dict[tuple, float] = {}
            groups = [(cycle, per_cycle[cycle]) for cycle in sorted(per_cycle)]

            def statistic(chosen: list[tuple[int, dict[int, tuple[int, int]]]]) -> float:
                # Resampled cycles give resampled counts per step size; identical draws share one fit (cache).
                key = tuple(sorted(c for c, _ in chosen))
                if key not in cache:
                    successes, attempts = np.zeros(n_steps), np.zeros(n_steps)
                    for _, counts in chosen:
                        for i, (s_count, t_count) in counts.items():
                            successes[i] += s_count
                            attempts[i] += t_count
                    sampled = _fit_curve(levels, successes, attempts, out.false_alarm_fraction, lapse)
                    cache[key] = float("nan") if sampled is None else sampled.threshold(0.5)
                return cache[key]
            if groups and not out.delta_50_is_bound:          # a bound is not a measurement: no interval
                boot = bootstrap_statistic(groups, statistic, params.bootstrap_resamples, params.confidence_level, rng)
                out.delta_50_lower_mm, out.delta_50_upper_mm = float(boot.lower), float(boot.upper)
                out.bootstrap_failures = boot.failures
        out.note = "; ".join(x for x in (out.note, flagged_note, scatter_note) if x)
        results.append(out)
        if out.flagged_rungs and size == sizes[0]:
            notes.append(f"station {station:g} mm: {flagged_note}")
    return results


# ---------------------------------------------------------------------------
# Step 5: the staircase
# ---------------------------------------------------------------------------
def _analyze_staircase(session: Session, station: float, stair_records: list[FrameRecord],
                       ladder_a_records: list[FrameRecord], previous: dict, options: DepthResolutionOptions,
                       lsb_mm: float) -> StaircaseResult:
    params = session.params
    result = StaircaseResult(station_z_mm=station)
    poses = sorted(group_by_pose(stair_records).values(), key=lambda g: g[0].step_mm)
    sensed, true_z = [], []
    roi = None
    for group in poses:
        stack = load_stack(group)
        geometry = pose_geometry(session, stack.record(), stack.camera)
        roi = region_of_interest(geometry, params) if roi is None else roi & region_of_interest(geometry, params)
        record = stack.record()
        true_z.append(registered_depth_mm(record))
        with np.errstate(invalid="ignore"), warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            sensed.append((stack.depth, np.nanmedian(stack.depth, axis=0), stack.camera))
    camera = sensed[0][2]
    # The Z0 point: the staircase starts at Z0, so the ladder's A visits (same plate position) stand in when the
    # staircase has no zero-step pose (judged by the commanded step label; the read-back Z carries the robot's scatter).
    if not any(abs(group[0].step_mm) <= TRUTH_COMPARISON_TOLERANCE_MM for group in poses):
        baseline = []
        for group in sorted(group_by_pose(ladder_a_records).values(), key=lambda g: _time_key(g[0]))[:options.baseline_max_poses]:
            baseline.append(load_stack(group).depth)
        if baseline:
            frames = np.concatenate(baseline, axis=0)
            with np.errstate(invalid="ignore"), warnings.catch_warnings():
                warnings.simplefilter("ignore", RuntimeWarning)
                sensed.insert(0, (frames, np.nanmedian(frames, axis=0), camera))
            true_z.insert(0, float(np.mean([registered_depth_mm(r) for r in ladder_a_records])))
    result.truth_source = TRUTH_SOURCE
    order = np.argsort(true_z)
    true_z = [true_z[i] for i in order]
    sensed = [sensed[i] for i in order]
    result.true_z_mm = true_z
    medians = np.stack([s[1] for s in sensed])                   # (steps, H, W): per-pixel sensed Z
    patch = central_patch_mask(camera, params.quantization_patch_px)
    result.patch_mean_mm = [float(np.nanmean(m[patch])) for m in medians]
    diagonal = np.argwhere(patch)
    picks = diagonal[np.linspace(0, len(diagonal) - 1, options.plotted_pixels).astype(int)]
    result.pixel_curves_mm = [[float(m[r, c]) for m in medians] for r, c in picks]
    # Expected quantum: from Analysis A when available, else from the data's own depth levels.
    pooled = np.concatenate([s[0][:, roi].ravel() for s in sensed])
    levels_mm, _, is_lsb = estimate_quantum_phase_resultant(pooled, lsb_mm)
    result.quantum_levels_mm = levels_mm
    terms = previous["A"].forward_model_terms() if "A" in previous else {}
    q_px = terms.get("q_px")
    if q_px is not None:
        result.predicted_quantum_mm = float(session.geometry.depth_quantum_mm(q_px, station))
        result.expected_quantum_mm = result.predicted_quantum_mm
    else:
        result.note = "q Z^2 / k from Analysis A: unavailable"
        result.expected_quantum_mm = levels_mm
    fine_step = result.expected_quantum_mm / params.z_staircase_subdivision
    if math.isfinite(fine_step) and fine_step < params.robot_repeatability_mm:
        result.note = (result.note + "; " if result.note else "") + (
            f"the staircase steps ({fine_step:.3g} mm) are below the robot's repeatability "
            f"({params.robot_repeatability_mm:g} mm): the read-back Z of each step, and so the plateau widths, carry "
            "that uncertainty")
    # Plateau widths of single pixels. A jump is a change larger than a fraction of the expected quantum.
    if math.isfinite(result.expected_quantum_mm) and len(true_z) >= 3:
        usable = roi & np.all(np.isfinite(medians), axis=0)
        values = medians[:, usable]                              # (steps, pixels)
        change = np.abs(np.diff(values, axis=0))
        jumps = change > options.plateau_tolerance_fraction * result.expected_quantum_mm
        x = np.array(true_z)
        midpoints = (x[:-1] + x[1:]) / 2.0
        widths = []
        for column in range(values.shape[1]):
            positions = midpoints[jumps[:, column]]
            widths.extend(np.diff(positions))
        result.plateaus = len(widths)
        if widths:
            result.plateau_width_median_mm = float(np.median(widths))
        if jumps.any():
            result.quantum_jump_height_mm = float(np.median(change[jumps]))
        spacing = float(np.median(np.diff(x)))
        fine_enough = spacing * PLATEAU_MIN_STEPS_PER_QUANTUM <= result.expected_quantum_mm
        resolved = result.plateaus and result.plateau_width_median_mm >= PLATEAU_MIN_WIDTH_STEPS * spacing
        if result.plateaus >= options.min_complete_plateaus and fine_enough and resolved:
            result.quantum_plateau_mm = result.plateau_width_median_mm
        elif result.plateaus and fine_enough and not resolved:
            result.note = (result.note + "; " if result.note else "") + (
                f"single-pixel plateaus are not resolved (median width {result.plateau_width_median_mm:.3g} mm, "
                f"about one staircase step of {spacing:.3g} mm): the temporal noise is comparable to the quantum, "
                "so the quantum comes from the pooled depth levels")
        elif result.plateaus:
            result.note = (result.note + "; " if result.note else "") + (
                f"staircase step {spacing:.3g} mm is too coarse for plateau widths (expected quantum "
                f"{result.expected_quantum_mm:.3g} mm, at least {PLATEAU_MIN_STEPS_PER_QUANTUM:g} steps per quantum needed)")
    if math.isfinite(result.quantum_plateau_mm):
        result.quantum_mm, result.quantum_method = result.quantum_plateau_mm, "plateau widths"
    elif math.isfinite(levels_mm):
        result.quantum_mm = levels_mm
        result.quantum_method = ("depth levels of the pooled single-pixel readings (output LSB is the quantizer)"
                                 if is_lsb else "depth levels of the pooled single-pixel readings (phase resultant)")
    # Is the patch mean smooth (quantizer dithered)?
    if len(true_z) >= 3 and math.isfinite(result.quantum_mm):
        line = np.polyfit(true_z, result.patch_mean_mm, 1)
        result.patch_rms_about_line_mm = float(np.sqrt(np.mean((np.array(result.patch_mean_mm)
                                                                  - np.polyval(line, true_z)) ** 2)))
        result.patch_is_smooth = bool(result.patch_rms_about_line_mm < SMOOTH_PATCH_FRACTION * result.quantum_mm)
    return result


# ---------------------------------------------------------------------------
# The analysis
# ---------------------------------------------------------------------------
def run_depth_resolution(session: Session, out_dir: str | Path, previous: dict | None = None,
                         options: DepthResolutionOptions | None = None) -> DepthResolutionResult | None:
    """Analysis B-Z, Section 11.2 Steps 1 to 5 (module docstring). Returns None when the manifest has no procedure Z
    frames. ``previous`` may hold Analysis A's result under "A" (its q for the predicted quantum). ``out_dir`` is where
    write_outputs will write; nothing is written here."""
    options = DepthResolutionOptions() if options is None else options
    previous = previous or {}
    records = select(session.records, procedure=PROCEDURE_ZSTEP)
    if not records:
        return None
    rng = np.random.default_rng(options.bootstrap_seed)
    lsb = float(session.geometry.require("depth_lsb_mm"))
    sizes = tuple(session.params.zstep_patch_sizes_px)
    patches: list[PatchResult] = []
    staircases: list[StaircaseResult] = []
    notes: list[str] = []
    for station in sorted({r.station_z_mm for r in records}):
        here = [r for r in records if r.station_z_mm == station]
        ladder = select(here, subseries=SUBSERIES_LADDER)
        stairs = select(here, subseries=SUBSERIES_STAIRCASE)
        if ladder:
            visits = _load_visits(session, ladder, sizes)
            patches.extend(_analyze_station(session, station, visits, options, rng, notes))
        else:
            notes.append(f"station {station:g} mm has no ladder poses")
        if stairs:
            staircases.append(_analyze_staircase(session, station, stairs, [r for r in ladder if r.visit == VISIT_A],
                                                 previous, options, lsb))
        else:
            notes.append(f"station {station:g} mm has no staircase poses")
    scaling: dict[float, dict[str, Any]] = {}
    for station in sorted({p.station_z_mm for p in patches}):
        good = [p for p in patches if p.station_z_mm == station and math.isfinite(p.delta_50_mm) and p.delta_50_mm > 0
                and not p.delta_50_is_bound]
        if len(good) >= MIN_FIT_POINTS_FOR_SCALING:
            exponent = float(np.polyfit(np.log([p.patch_px for p in good]), np.log([p.delta_50_mm for p in good]), 1)[0])
            scaling[station] = {
                "exponent_of_delta_50_vs_patch_side": exponent, "uncorrelated_noise_expectation": -1.0,
                "ratio_to_single_pixel": {str(p.patch_px): p.delta_50_mm / good[0].delta_50_mm for p in good},
                "reading": ("delta_50 shrinks more slowly than 1 / side: the noise is correlated over the patch scale"
                            if exponent > -1.0 + 0.1 else "delta_50 shrinks as 1 / side or faster")}
    return DepthResolutionResult(patches=patches, staircases=staircases, scaling=scaling, notes=notes, options=options)


# ---------------------------------------------------------------------------
# Outputs
# ---------------------------------------------------------------------------
def _grid_figure(panels: int, columns: int):
    rows = max(1, -(-panels // columns))
    columns = min(columns, max(panels, 1))
    figure, axis = new_figure(PANEL_WIDTH_IN * columns, PANEL_HEIGHT_IN * rows)
    figure.delaxes(axis)
    figure.set_layout_engine("constrained")        # keeps titles, labels and colorbars from overlapping
    axes = figure.subplots(rows, columns, squeeze=False).ravel()
    for extra in axes[panels:]:
        extra.set_visible(False)
    return figure, axes[:panels]


def _figure_detection(result: DepthResolutionResult, out_dir: Path) -> list[Path]:
    stations = sorted({p.station_z_mm for p in result.patches})
    figure, axes = _grid_figure(max(len(stations), 1), 2)
    for axis, station in zip(axes, stations):
        for index, p in enumerate((q for q in result.patches if q.station_z_mm == station)):
            if not p.steps_mm or not p.trials:
                continue
            color = OKABE_ITO_CYCLE[index % len(OKABE_ITO_CYCLE)]
            fraction = np.array(p.detections) / np.maximum(np.array(p.trials), 1)
            ok = np.array(p.rung_reliable, dtype=bool)
            axis.plot(np.array(p.steps_mm)[ok], fraction[ok], "o", color=color, label=f"{p.patch_px} px")
            if not ok.all():                  # flagged rungs (truth below the robot repeatability): open markers
                axis.plot(np.array(p.steps_mm)[~ok], fraction[~ok], "o", color=color, markerfacecolor="none")
            if math.isfinite(p.fit_alpha):
                grid = np.geomspace(min(p.steps_mm) * 0.5, max(p.steps_mm) * 1.5, FIT_CURVE_POINTS)
                corrected = 1.0 / (1.0 + np.exp(-(np.log(grid) - p.fit_alpha) / p.fit_beta))
                axis.plot(grid, p.false_alarm_fraction + (1.0 - p.false_alarm_fraction - p.fit_lapse) * corrected,
                          color=color)
            if math.isfinite(p.delta_50_mm):
                axis.axvline(p.delta_50_mm, color=color, linestyle=":", linewidth=0.8)
        axis.set_xscale("log")
        axis.set_xlabel("true step (mm)")
        axis.set_ylabel("detection fraction")
        axis.set_title(f"Z0 = {station:g} mm", fontsize="small")
        axis.set_ylim(-0.02, 1.02)
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize="x-small", title="patch")
    return save_figure(figure, out_dir / FIGURE_STEMS["detection"])


def _figure_sensed(result: DepthResolutionResult, out_dir: Path) -> list[Path]:
    stations = sorted({p.station_z_mm for p in result.patches})
    figure, axes = _grid_figure(max(len(stations), 1), 2)
    for axis, station in zip(axes, stations):
        top = 0.0
        for index, p in enumerate((q for q in result.patches if q.station_z_mm == station)):
            if not p.pair_true_mm:
                continue
            color = OKABE_ITO_CYCLE[index % len(OKABE_ITO_CYCLE)]
            ok = np.array(p.pair_reliable, dtype=bool)
            true, delta = np.array(p.pair_true_mm), np.array(p.pair_delta_mm)
            axis.plot(true[ok], delta[ok], "o", color=color, markersize=3, alpha=0.7,
                      label=f"{p.patch_px} px, gain {p.gain:.3f}")
            if not ok.all():                  # flagged rungs are not in the gain regression: open markers
                axis.plot(true[~ok], delta[~ok], "o", color=color, markersize=3, markerfacecolor="none")
            top = max(top, max(p.pair_true_mm))
        axis.plot([0.0, top], [0.0, top], color=OKABE_ITO_BLACK, linewidth=0.7, linestyle="--")
        axis.set_xlabel("true step, read-back pose difference (mm)")
        axis.set_ylabel("sensed step, mean Delta (mm)")
        axis.set_title(f"Z0 = {station:g} mm", fontsize="small")
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize="x-small")
    return save_figure(figure, out_dir / FIGURE_STEMS["sensed"])


def _figure_staircase(result: DepthResolutionResult, out_dir: Path) -> list[Path] | None:
    if not result.staircases:
        return None
    figure, axes = _grid_figure(len(result.staircases), 2)
    for axis, stair in zip(axes, result.staircases):
        x = np.array(stair.true_z_mm) - stair.station_z_mm
        for index, curve in enumerate(stair.pixel_curves_mm):
            axis.plot(x, np.array(curve) - stair.station_z_mm, marker=".", linewidth=0.7, alpha=0.7,
                      color=OKABE_ITO_SKY_BLUE, label="single pixels" if index == 0 else None)
        axis.plot(x, np.array(stair.patch_mean_mm) - stair.station_z_mm, marker="o", color=OKABE_ITO_VERMILLION,
                  label="20 x 20 patch mean")
        axis.plot(x, x, color=OKABE_ITO_BLACK, linestyle="--", linewidth=0.7, label="read-back Z")
        axis.set_xlabel("read-back Z - Z0 (mm)")
        axis.set_ylabel("sensed Z - Z0 (mm)")
        title = f"Z0 = {stair.station_z_mm:g} mm"
        if math.isfinite(stair.quantum_mm):
            title += f", quantum {stair.quantum_mm:.3g} mm"
        axis.set_title(title, fontsize="small")
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize="x-small")
    return save_figure(figure, out_dir / FIGURE_STEMS["staircase"])


def write_outputs(result: DepthResolutionResult, out_dir: str | Path) -> list[Path]:
    """Z_resolution_summary.csv (one row per station and patch size), Z_resolution_rungs.csv (one row per station, patch
    size and ladder rung, with ``truth_reliable``), Z_resolution_details.json and the figures
    (detection fraction with the fits, sensed against true step, staircase)."""
    out_dir = Path(out_dir)
    stairs = {s.station_z_mm: s for s in result.staircases}
    rows = []
    for p in result.patches:
        stair = stairs.get(p.station_z_mm)
        rows.append({
            "station_z_mm": p.station_z_mm, "patch_px": p.patch_px, "gain": p.gain, "intercept_mm": p.intercept_mm,
            "max_deviation_mm": p.max_deviation_mm, "tau_mm": p.tau_mm, "delta_50_mm": p.delta_50_mm,
            "delta_50_lower_mm": p.delta_50_lower_mm, "delta_50_upper_mm": p.delta_50_upper_mm,
            "quantum_mm": float("nan") if stair is None else stair.quantum_mm,
            "quantum_predicted_mm": float("nan") if stair is None else stair.predicted_quantum_mm,
            "cycles": p.cycles, "quantum_method": "" if stair is None else stair.quantum_method,
            "false_alarm_fraction": p.false_alarm_fraction, "tiles_per_pair": p.tiles_per_pair,
            "truth_source": TRUTH_SOURCE, "truth_reliable": p.flagged_rungs == 0,
            "delta_50_is_bound": p.delta_50_is_bound,
            "robot_readback_repeatability_mm": p.robot_readback_repeatability_mm,
            "readback_minus_commanded_mm": p.readback_minus_commanded_mm,
            "robot_scatter_exceeds_spec": p.robot_scatter_exceeds_spec,
            "note": "; ".join(x for x in (p.note, "" if stair is None else stair.note) if x)})
    rung_rows = [
        {"station_z_mm": p.station_z_mm, "patch_px": p.patch_px, "step_mm": label, "true_step_mm": true,
         "mean_delta_mm": delta, "detections": hits, "trials": trials, "detection_fraction": hits / trials,
         "readback_minus_commanded_mm": offset, "truth_source": TRUTH_SOURCE, "truth_reliable": ok}
        for p in result.patches
        for label, true, delta, hits, trials, offset, ok in zip(
            p.rung_labels_mm, p.steps_mm, p.mean_delta_mm, p.detections, p.trials, p.rung_readback_offset_mm,
            p.rung_reliable)]
    written = [write_csv_rows(out_dir / SUMMARY_CSV_NAME, rows, SUMMARY_COLUMNS),
               write_csv_rows(out_dir / RUNGS_CSV_NAME, rung_rows, RUNG_COLUMNS)]
    details = {
        "patches": [p.__dict__ for p in result.patches],
        "staircases": [s.__dict__ for s in result.staircases],
        "patch_size_scaling": {f"{k:g}": v for k, v in result.scaling.items()},
        "forward_model_terms": result.forward_model_terms(), "notes": result.notes}
    written.append(write_json(out_dir / DETAILS_JSON_NAME, details))
    written += _figure_detection(result, out_dir)
    written += _figure_sensed(result, out_dir)
    stair_paths = _figure_staircase(result, out_dir)
    if stair_paths:
        written += stair_paths
    return written
