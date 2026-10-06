"""
Analysis B-Z: effective resolution in depth (procedure document, Section 11.2), from the Z-step series
(procedure "Z"): the noise plate T2 at a station Z0, a step ladder (the plate alternates between Z0, visit A, and
Z0 + delta, visit B, for several delta; the rungs are multiples of the expected depth quantum at the station, so every
station has its own rung set), a ramp (one pose of T2 tilted about H at each station of the ladder) and, when it was
captured, the optional second pass, a fine staircase.

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
     whose true step is below TWICE the robot repeatability (``params.truth_reliable_rung_floor_mm`` =
     ``TRUTH_RELIABLE_RUNG_TO_REPEATABILITY_RATIO`` x ``robot_repeatability_mm``, Section 11.2, Step 10; a rule of its
     own, separate from the ladder floor ``robot_min_resolvable_move_mm`` of Section 6.2, Step 2, with which it coincides at
     the defaults, 0.1 mm) get ``truth_reliable`` = False: the step truth is then too close to the robot's own scatter. They are
     excluded from this regression, kept in the detection curve of Step 4, and the note says how many rungs were flagged. The registration enters only through the DIFFERENCE of two registered
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
  5  The ramp (sub-series "ramp", one pose per station of the ladder; Section 11.2, Ramp). The plate is tilted about
     H, the axis parallel to the baseline, so every image row lies at one true depth and the rows step through the
     quanta. Per ramp pose: the fixed-pattern map of Analysis A at the same station (``previous["A"]``, the
     fronto-parallel center-field main pose) is subtracted from the frame-mean depth where the map is finite (without
     it, or where it is NaN, nothing is subtracted and the note says so); the depth is averaged along each image row
     within the region of interest (rows with fewer than ``RAMP_MIN_ROW_FILL_FRACTION`` of the widest row are dropped);
     the true depth of each row is the mean over the same pixels of the registered front-plane depth (the read-back pose of
     the tilted plate through the registration, ``PoseGeometry.z_front_gt``). The row average is plotted against the
     true depth. Its plateau widths are the measured quantum dZ_q(Z): a jump is a change over a short window of rows
     (``RAMP_JUMP_WINDOW_FRACTION`` of a quantum) larger than ``plateau_tolerance_fraction`` of the expected quantum, the
     plateau widths are the spacings of the jumps (:func:`plateau_widths`, the detection of the staircase), and the
     estimate is used only if the ramp has at least ``PLATEAU_MIN_STEPS_PER_QUANTUM`` rows per expected quantum, at least
     ``RAMP_MIN_COMPLETE_PLATEAUS`` plateaus, and a median plateau of at least ``RAMP_PLATEAU_MIN_WIDTH_ROWS`` rows (the
     guard ``PLATEAU_MIN_WIDTH_STEPS`` of the staircase, counted in rows). It is compared with q Z^2 / k from Analysis A
     (the expected quantum; without A, the quantum of the depth levels of the pooled readings) and the implied disparity
     quantum q = dZ_q k / Z^2 is reported. Single pixels (the temporal median of a few columns) are drawn the same way.
     DITHERING: a staircase changes over a window of half a quantum either by about nothing (on a plateau) or by about a
     quantum (across a step), a smooth ramp by half a quantum everywhere. A curve is stepped when fewer than
     ``ramp_max_intermediate_fraction`` (0.5) of its window changes lie within ``ramp_intermediate_band`` (0.25 to 0.75)
     of the quantum, and smooth otherwise; both are parameters chosen by argument, not from data (Section 11.2, Step 14). A smooth row average over stepped single pixels (the pixels step at different rows, from their
     fixed pattern or noise) means that the sensor's interpolation or the noise dithers the quantizer; the plateaus are
     then not resolved in the average and the quantum comes from the pooled depth levels (``quantum_method`` says which:
     "plateaus" or "depth levels").
  6  The staircase, when it was captured (sub-series "staircase", the optional second pass, ordered by the commanded
     displacement): sensed Z of single pixels and of the
     20 x 20 patch against the read-back Z of each step (the z component of the target pose, whose uncertainty is the
     robot repeatability; a note says when the fine step is smaller than that); the measured quantum from plateau widths of single pixels, compared with
     q Z^2 / k using q of Analysis A (``previous["A"]``) when it is available. The ramp quantum and the staircase quantum
     of a station are reported side by side.

The quantum of Steps 5 and 6. Plateau widths need a ramp or a staircase much finer than the quantum, with many rows or
steps. When too few complete plateaus exist (the demonstration plan has three staircase steps; a dithered ramp has none in
its row average) the quantum is the one of the depth levels of the single-pixel readings pooled over all frames of the
pose (the staircase adds a few Z0 frames of the ladder as the zero point), by the phase-resultant method of Analysis A,
Step 8 (``noise.estimate_quantum_phase_resultant``); ``quantum_method`` says which ("plateaus" or "depth levels").

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
from sensorperf.io.manifest import (
    SUBSERIES_LADDER, SUBSERIES_MAIN, SUBSERIES_RAMP, SUBSERIES_STAIRCASE, VISIT_A, VISIT_B, FrameRecord, group_by_pose,
    select,
)
from sensorperf.io.session import Session
from sensorperf.parameters import FIELD_POSITION_CENTER, PROCEDURE_ZSTEP, CharacterizationParameters
from sensorperf.stats.intervals import bootstrap_statistic
from sensorperf.stats.psychometric import PsychometricFitParameters, fit_psychometric

SUMMARY_CSV_NAME = "Z_resolution_summary.csv"
DETAILS_JSON_NAME = "Z_resolution_details.json"
FIGURE_STEMS = {"detection": "Z_detection_curves", "sensed": "Z_sensed_vs_true", "ramp": "Z_ramp",
                "quantum": "Z_quantum_vs_z", "staircase": "Z_staircase"}
OPTIONAL_FIGURES = ("staircase",)
"""Figures that exist only when their data were captured (the staircase is the optional second pass)."""
SUMMARY_COLUMNS = (
    "station_z_mm", "patch_px", "gain", "intercept_mm", "max_deviation_mm", "tau_mm", "delta_50_mm",
    "delta_50_lower_mm", "delta_50_upper_mm", "cycles", "false_alarm_fraction", "tiles_per_pair", "truth_source",
    "truth_reliable", "delta_50_is_bound", "smallest_rung_mm", "robot_repeatability_to_smallest_rung",
    "robot_readback_repeatability_mm", "readback_minus_commanded_mm", "robot_scatter_exceeds_spec",
    "ramp_tilt_deg", "ramp_fixed_pattern_subtracted", "ramp_plateaus", "ramp_quantum_mm", "ramp_quantum_predicted_mm",
    "ramp_q_px", "ramp_quantum_method", "ramp_dithered",
    "staircase_quantum_mm", "staircase_quantum_predicted_mm", "staircase_quantum_method", "note")
"""Columns of Z_resolution_summary.csv: the Section 11.2 list (the ladder statistics of the station and patch size), the
measured false-alarm fraction, the tiles per pair, where the true step came from, whether every ladder rung of the
station has a reliable truth (False when any rung is flagged), whether delta_50 is only an upper bound, the smallest
rung of the station (true step) and the ratio of ``robot_repeatability_mm`` to it (the truth uncertainty of the smallest
rung; every rung's ratio is in ``Z_resolution_rungs.csv``), the robot's read-back scatter from the A -> A pairs and the
mean read-back minus commanded step of the A -> B pairs, whether that scatter exceeds ``robot_repeatability_mm``; then the
ramp of the station (its tilt, whether Analysis A's fixed-pattern map was subtracted, the number of complete plateaus,
the measured quantum, the prediction q Z^2 / k from Analysis A, the implied disparity quantum q, the method and the
dithering flag) and, when the optional staircase was captured, the staircase's quantum, its prediction and method, side
by side with the ramp's (NaN or empty for a station without that pass); and a note."""
RUNGS_CSV_NAME = "Z_resolution_rungs.csv"
RUNG_COLUMNS = (
    "station_z_mm", "patch_px", "step_mm", "true_step_mm", "mean_delta_mm", "detections", "trials",
    "detection_fraction", "readback_minus_commanded_mm", "robot_repeatability_ratio", "truth_source", "truth_reliable")
"""Columns of Z_resolution_rungs.csv, one row per station, patch size and ladder rung: the commanded step (the label),
the mean true step, the mean sensed step, the detection counts, the read-back minus commanded step, the ratio of
``robot_repeatability_mm`` to the true step (the truth uncertainty of the rung), and ``truth_reliable`` (False for a rung
whose true step is below twice ``robot_repeatability_mm``, Section 11.2, Step 10: reported, but not part of the gain
regression)."""
RAMP_CSV_NAME = "Z_ramp_rows.csv"
RAMP_COLUMNS = ("station_z_mm", "row", "true_depth_mm", "row_average_mm", "row_average_minus_true_mm")
"""Z_ramp_rows.csv: one row per image row of each ramp pose, the true depth of the row, the row average (fixed pattern of A
subtracted where available) and their difference, the sawtooth in which the quantizer shows."""

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
RAMP_MIN_ROW_FILL_FRACTION = 0.5
"""A row of the ramp enters the row average only if its region of interest has at least this fraction of the pixels of the
widest row (the rows at the rounded corners of the plate have few pixels and a noisy mean)."""
RAMP_MIN_ROWS = 6
"""Fewest rows of a ramp pose for the analysis."""
RAMP_MIN_FIXED_PATTERN_COVERAGE = 0.5
"""The fixed-pattern map of A is subtracted only if it is finite on at least this fraction of the ramp's region of
interest; otherwise nothing is subtracted (NaN-safe fallback, noted)."""
RAMP_JUMP_WINDOW_FRACTION = 0.5
"""A jump of the ramp's row average is a change over a window of this fraction of a quantum, in rows: a transition that the
phase scatter of the pixels smears over a fraction of a quantum then still shows its full height in the window, where the
change from one row to the next would not."""
RAMP_MIN_COMPLETE_PLATEAUS = 1
"""Fewest complete plateaus (bounded by a jump on each side) of the ramp for the plateau-width estimate of the quantum: a
ramp of four quanta, less the rows lost at the ends of the plate, holds two or three (the staircase, which sweeps many
more, asks for ``MIN_COMPLETE_PLATEAUS``)."""
RAMP_PLATEAU_MIN_WIDTH_ROWS = PLATEAU_MIN_WIDTH_STEPS
"""The ramp's plateau-width estimate is used only if the median plateau spans at least this many rows: the guard
``PLATEAU_MIN_WIDTH_STEPS`` of the staircase (steps) counted in rows, since the rows are the ramp's steps."""
QUANTUM_METHOD_PLATEAUS = "plateaus"
"""``quantum_method`` of a quantum taken from the widths of the plateaus of a ramp or a staircase."""
QUANTUM_METHOD_DEPTH_LEVELS = "depth levels"
"""``quantum_method`` of a quantum taken from the spacing of the populated depth levels of the pooled single-pixel readings
(the phase-resultant method of Analysis A, Step 8). Whether the levels were no finer than the output LSB is in
``quantum_is_lsb``."""
RAMP_ROW_SPREAD_NOTE_FRACTION = 0.1
"""The ramp's rows are not at one true depth (the tilt axis is not parallel to the baseline, or the plate is also
rotated about the optical axis) when the true depth varies along a row by more than this fraction of the quantum; the
note says so."""
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
    """For each pair of ``pair_true_mm``: False when the rung's true step is below twice the robot repeatability."""
    rung_labels_mm: list[float] = field(default_factory=list)
    """Commanded step (``step_mm``, the label of the rung) of each level of ``steps_mm``."""
    rung_readback_offset_mm: list[float] = field(default_factory=list)
    """Mean read-back step minus the commanded step of each level of ``steps_mm``, mm."""
    rung_reliable: list[bool] = field(default_factory=list)
    """``truth_reliable`` of each level of ``steps_mm``: False when the mean true step of the rung is below
    ``params.truth_reliable_rung_floor_mm`` (twice ``robot_repeatability_mm``)."""
    rung_robot_ratio: list[float] = field(default_factory=list)
    """``robot_repeatability_mm`` divided by the mean true step of each level of ``steps_mm``: the truth uncertainty of
    the rung as a fraction of the step."""
    smallest_rung_mm: float = float("nan")
    """The smallest mean true step of the station's ladder (the rung sets differ from station to station), mm."""
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
    """QUANTUM_METHOD_PLATEAUS or QUANTUM_METHOD_DEPTH_LEVELS ("unavailable" when no quantum was found)."""
    quantum_is_lsb: bool = False
    """True when the depth levels were no finer than the output LSB, so the quantum found is the quantizer of the output
    format and not the sensor's disparity quantum (no q is implied then)."""
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
class RampResult:
    """Step 5 for one ramp pose (one station)."""

    station_z_mm: float
    tilt_deg: float = float("nan")
    """Tilt about H of the ramp pose as planned (manifest column ``tilt_deg``)."""
    frames: int = 0
    rows: int = 0
    """Rows of the row average."""
    fixed_pattern_subtracted: bool = False
    """True when the fixed-pattern map of Analysis A at the station was subtracted."""
    true_depth_mm: list[float] = field(default_factory=list)
    """True depth of each row: the mean over the row's pixels of the registered front-plane depth (read-back pose through
    the registration)."""
    row_average_mm: list[float] = field(default_factory=list)
    """Sensed depth averaged along each row within the region of interest (fixed pattern of A subtracted if available)."""
    pixel_columns: list[int] = field(default_factory=list)
    pixel_true_mm: list[list[float]] = field(default_factory=list)
    pixel_curves_mm: list[list[float]] = field(default_factory=list)
    """Temporal median of a few single pixels (the columns of ``pixel_columns``) along the rows, with the same subtraction."""
    jump_positions_mm: list[float] = field(default_factory=list)
    """True depth of the jumps of the row average (the plateau edges)."""
    plateaus: int = 0
    plateau_width_median_mm: float = float("nan")
    quantum_jump_height_mm: float = float("nan")
    rows_per_quantum: float = float("nan")
    quantum_mm: float = float("nan")
    quantum_method: str = "unavailable"
    """QUANTUM_METHOD_PLATEAUS or QUANTUM_METHOD_DEPTH_LEVELS ("unavailable" when no quantum was found)."""
    quantum_is_lsb: bool = False
    """True when the depth levels were no finer than the output LSB, so the quantum found is the quantizer of the output
    format and not the sensor's disparity quantum (no q is implied then)."""
    quantum_plateau_mm: float = float("nan")
    quantum_levels_mm: float = float("nan")
    expected_quantum_mm: float = float("nan")
    predicted_quantum_mm: float = float("nan")
    """q Z^2 / k with Analysis A's q at the mean true depth of the rows (NaN without A's q)."""
    q_px: float = float("nan")
    """The implied disparity quantum q = dZ_q k / Z^2 of the measured quantum, pixels."""
    span_quanta: float = float("nan")
    """The true depth spanned by the rows, in expected quanta (the plan asks for ``ramp_quanta`` over the plate's visible
    height; the region of interest is somewhat shorter)."""
    row_intermediate_fraction: float = float("nan")
    """Fraction of the window changes of the row average that are intermediate (``ramp_intermediate_band``): near 0 for a
    staircase, near 1 for a smooth curve."""
    pixel_intermediate_fraction: float = float("nan")
    """The same for the plotted single pixels (median over the pixels)."""
    row_is_smooth: bool | None = None
    pixels_stepped: bool | None = None
    dithered: bool | None = None
    """True when the row average is smooth over stepped single pixels (module docstring, Step 5); None when it could not
    be judged."""
    row_true_spread_mm: float = float("nan")
    """Largest variation of the true depth along a row, mm (zero when the tilt axis is parallel to the baseline)."""
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
    ramps: list[RampResult] = field(default_factory=list)

    def patch(self, station_z_mm: float, patch_px: int) -> PatchResult | None:
        for p in self.patches:
            if p.station_z_mm == station_z_mm and p.patch_px == patch_px:
                return p
        return None

    def ramp(self, station_z_mm: float) -> RampResult | None:
        """The ramp result of a station (the first one if the station has several ramp poses), or None."""
        return next((r for r in self.ramps if r.station_z_mm == station_z_mm), None)

    def staircase(self, station_z_mm: float) -> StaircaseResult | None:
        """The staircase result of a station, or None (the staircase is the optional second pass)."""
        return next((s for s in self.staircases if s.station_z_mm == station_z_mm), None)

    def forward_model_terms(self) -> dict[str, Any]:
        """The measured depth quantum per station and delta_50 of the 1-px patch per station (mm), keyed by the station
        depth in mm. The quantum is the ramp's; the staircase's stands in at a station without a ramp. An upper bound
        (``delta_50_is_bound``) is not a measurement and is left out."""
        quanta = {f"{s.station_z_mm:g}": s.quantum_mm for s in self.staircases if math.isfinite(s.quantum_mm)}
        quanta.update({f"{r.station_z_mm:g}": r.quantum_mm for r in self.ramps if math.isfinite(r.quantum_mm)})
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


def rung_truth_is_reliable(true_step_mm: float, params: CharacterizationParameters) -> bool:
    """The truth rule of Section 11.2, Step 10: the step truth of a rung is reliable when its true step is at least
    ``params.truth_reliable_rung_floor_mm``, twice the robot repeatability (``TRUTH_RELIABLE_RUNG_TO_REPEATABILITY_RATIO``).
    The rule is separate from the ladder floor ``robot_min_resolvable_move_mm`` (Section 6.2, Step 2), which gives the same
    0.1 mm at the defaults. A true step that equals the limit up to rounding of the
    registration product (``TRUTH_COMPARISON_TOLERANCE_MM``) reaches it."""
    return bool(true_step_mm >= params.truth_reliable_rung_floor_mm - TRUTH_COMPARISON_TOLERANCE_MM)


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
                # truth_reliable (Section 11.2, Step 10): the true step reaches TWICE the robot's repeatability
                # (ISO 9283, Section 3.1), the floor the ladder applies (Section 6.2, Step 2).
                reliable[index] = rung_truth_is_reliable(levels[index], params)
        have = trials > 0
        out.steps_mm, out.mean_delta_mm = levels[have].tolist(), mean_delta[have].tolist()
        out.rung_labels_mm = [float(step_sizes[i]) for i in np.flatnonzero(have)]
        out.rung_reliable = [bool(r) for r in reliable[have]]
        out.rung_robot_ratio = [float(params.robot_repeatability_mm / level) for level in levels[have]]
        out.smallest_rung_mm = float(np.min(levels[have])) if have.any() else float("nan")
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
                            f"(commanded {flagged_labels} mm, true step below twice the robot repeatability of "
                            f"{params.robot_repeatability_mm:g} mm, that is {params.truth_reliable_rung_floor_mm:g} mm): "
                            "excluded from the gain regression, kept in the detection curve")
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
        widths = plateau_widths(jumps, midpoints)
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
        result.quantum_mm, result.quantum_method = result.quantum_plateau_mm, QUANTUM_METHOD_PLATEAUS
    elif math.isfinite(levels_mm):
        result.quantum_mm = levels_mm
        result.quantum_method, result.quantum_is_lsb = QUANTUM_METHOD_DEPTH_LEVELS, bool(is_lsb)
    # Is the patch mean smooth (quantizer dithered)?
    if len(true_z) >= 3 and math.isfinite(result.quantum_mm):
        line = np.polyfit(true_z, result.patch_mean_mm, 1)
        result.patch_rms_about_line_mm = float(np.sqrt(np.mean((np.array(result.patch_mean_mm)
                                                                  - np.polyval(line, true_z)) ** 2)))
        result.patch_is_smooth = bool(result.patch_rms_about_line_mm < SMOOTH_PATCH_FRACTION * result.quantum_mm)
    return result


# ---------------------------------------------------------------------------
# Step 5: the ramp
# ---------------------------------------------------------------------------
def plateau_widths(jumps: np.ndarray, midpoints: np.ndarray) -> list[float]:
    """Plateau widths from jumps: ``jumps`` is a (steps, series) boolean array (a jump between step i and i + 1 of a
    series) and ``midpoints`` the position (true depth) of each step. For every series the widths are the spacings of
    consecutive jumps, that is of the complete plateaus (bounded by a jump on each side); the staircase calls it with one
    series per pixel, the ramp with the row average."""
    widths: list[float] = []
    for column in range(jumps.shape[1]):
        widths.extend(np.diff(midpoints[jumps[:, column]]))
    return widths


def _windowed_jumps(values: np.ndarray, positions: np.ndarray, window: int,
                    threshold: float) -> tuple[np.ndarray, np.ndarray, list[float]]:
    """Jumps of a curve sampled at ascending ``positions``: a jump is a change of more than ``threshold`` over ``window``
    samples. Consecutive samples over the threshold are one transition; its center stands for the jump (the transition of
    a sharp step is ``window`` samples long, that of a smeared one longer), and a transition that touches either end of the
    curve is dropped as incomplete. Returns (mask of shape (m, 1) with one True per jump, the position of each of the m
    windows (their centers), the height of each jump)."""
    change = values[window:] - values[:-window]
    centers = (positions[window:] + positions[:-window]) / 2.0
    over = np.abs(change) > threshold
    mask = np.zeros((change.size, 1), dtype=bool)
    heights: list[float] = []
    start = None
    for i in range(change.size + 1):
        if i < change.size and over[i]:
            start = i if start is None else start
        elif start is not None:
            end = i - 1
            if start > 0 and end < change.size - 1:
                mask[(start + end) // 2, 0] = True
                heights.append(float(np.max(np.abs(change[start:end + 1]))))
            start = None
    return mask, centers, heights


def _intermediate_fraction(values: np.ndarray, window: int, quantum_mm: float,
                           band: tuple[float, float]) -> float:
    """Fraction of the changes of a curve over ``window`` samples whose size lies in ``band`` (the parameter
    ``ramp_intermediate_band``, as fractions of the quantum; NaN for a curve shorter than the window): about 0 for a
    staircase, about 1 for a smooth ramp."""
    if values.size <= window:
        return float("nan")
    change = np.abs(values[window:] - values[:-window]) / quantum_mm
    return float(np.mean((change >= band[0]) & (change <= band[1])))


def _fixed_pattern_map(previous: dict, station: float) -> np.ndarray | None:
    """The fixed-pattern map of Analysis A at the station (the fronto-parallel center-field main pose: frame-mean depth
    minus its fitted plane, mm, NaN outside A's region of interest), or None without A's result or without such a pose."""
    noise_result = previous.get("A")
    if noise_result is None:
        return None
    for row in noise_result.rows:
        diagnostics = noise_result.diagnostics.get(row.pose_key)
        if (row.station_z_mm == station and row.subseries == SUBSERIES_MAIN and row.field == FIELD_POSITION_CENTER
                and row.tilt_deg == 0.0 and diagnostics is not None and diagnostics.fixed_pattern_mm is not None):
            return diagnostics.fixed_pattern_mm.astype(np.float64)
    return None


def _analyze_ramp(session: Session, station: float, records: list[FrameRecord], previous: dict,
                  options: DepthResolutionOptions, lsb_mm: float) -> RampResult:
    """Step 5 for one ramp pose (module docstring)."""
    params = session.params
    stack = load_stack(records)
    record = stack.record()
    result = RampResult(station_z_mm=station, tilt_deg=record.tilt_deg, frames=stack.frame_count)
    notes: list[str] = []
    geometry = pose_geometry(session, record, stack.camera)
    roi = region_of_interest(geometry, params)
    mean = stack.mean_depth()
    used = roi & np.isfinite(mean)
    # The fixed-pattern map of A at this station, subtracted where it is finite (NaN-safe fallback: none, noted).
    fixed_pattern = _fixed_pattern_map(previous, station)
    corrected = mean
    if fixed_pattern is None:
        notes.append("the fixed-pattern map of Analysis A at this station is unavailable: not subtracted")
    elif (used & np.isfinite(fixed_pattern)).sum() < RAMP_MIN_FIXED_PATTERN_COVERAGE * max(int(used.sum()), 1):
        fixed_pattern = None
        notes.append("the fixed-pattern map of Analysis A is NaN over most of the ramp's region of interest: not subtracted")
    else:
        used &= np.isfinite(fixed_pattern)
        corrected = mean - fixed_pattern
        result.fixed_pattern_subtracted = True
    # Rows: those whose region of interest holds enough pixels; the row's true depth is the mean of the registered
    # front-plane depth over the pixels that enter its average.
    counts = used.sum(axis=1)
    rows = np.flatnonzero(counts >= RAMP_MIN_ROW_FILL_FRACTION * max(int(counts.max()), 1)) if counts.any() else np.array([], int)
    if rows.size < RAMP_MIN_ROWS:
        result.note = "; ".join(notes + [f"only {rows.size} usable rows: no ramp analysis"])
        return result
    z_true = geometry.z_front_gt
    true_depth = np.array([z_true[r, used[r]].mean() for r in rows])
    row_average = np.array([corrected[r, used[r]].mean() for r in rows])
    spread = max(float(np.ptp(z_true[r, used[r]])) for r in rows)
    order = np.argsort(true_depth)
    rows, true_depth, row_average = rows[order], true_depth[order], row_average[order]
    result.rows, result.row_true_spread_mm = int(rows.size), spread
    result.true_depth_mm, result.row_average_mm = true_depth.tolist(), row_average.tolist()
    mean_depth = float(np.mean(true_depth))
    # Single pixels: the temporal median of a few columns spread over the widest row, along the rows.
    middle = rows[rows.size // 2]
    columns = np.flatnonzero(used[middle])
    picks = columns[np.linspace(0, columns.size - 1, options.plotted_pixels).astype(int)] if columns.size else columns
    with np.errstate(invalid="ignore"), warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        medians = np.nanmedian(stack.depth[:, :, picks], axis=0)               # (H, picked columns)
    pixel_true, pixel_curves = [], []
    for index, column in enumerate(picks):
        keep = rows[used[rows, column] & np.isfinite(medians[rows, index])]
        value = medians[keep, index] - (fixed_pattern[keep, column] if fixed_pattern is not None else 0.0)
        pixel_true.append(z_true[keep, column].tolist())
        pixel_curves.append(value.tolist())
    result.pixel_columns = [int(c) for c in picks]
    result.pixel_true_mm, result.pixel_curves_mm = pixel_true, pixel_curves
    # The expected quantum: q Z^2 / k from Analysis A when it measured a q, else the depth levels of the pooled readings.
    pooled = stack.depth[:, used]
    levels_mm, _, is_lsb = estimate_quantum_phase_resultant(pooled, lsb_mm)
    result.quantum_levels_mm = levels_mm
    terms = previous["A"].forward_model_terms() if "A" in previous else {}
    q_expected = terms.get("q_px")
    if q_expected is not None:
        result.predicted_quantum_mm = float(session.geometry.depth_quantum_mm(q_expected, mean_depth))
        result.expected_quantum_mm = result.predicted_quantum_mm
    else:
        notes.append("q Z^2 / k from Analysis A: unavailable")
        result.expected_quantum_mm = levels_mm
    expected = result.expected_quantum_mm
    if spread > RAMP_ROW_SPREAD_NOTE_FRACTION * expected if math.isfinite(expected) else False:
        notes.append(f"the true depth varies by {spread:.3g} mm along a row: the tilt axis is not parallel to the baseline")
    if math.isfinite(expected):
        result.span_quanta = float(np.ptp(true_depth) / expected)
        spacing = float(np.median(np.diff(true_depth)))
        result.rows_per_quantum = expected / spacing
        window = max(1, int(round(RAMP_JUMP_WINDOW_FRACTION * result.rows_per_quantum)))
        # Dithering: a smooth row average over stepped single pixels.
        band, max_fraction = params.ramp_intermediate_band, params.ramp_max_intermediate_fraction
        result.row_intermediate_fraction = _intermediate_fraction(row_average, window, expected, band)
        result.row_is_smooth = bool(result.row_intermediate_fraction >= max_fraction)
        pixel_fractions = [_intermediate_fraction(np.array(y), window, expected, band) for y in pixel_curves]
        pixel_fractions = [f for f in pixel_fractions if math.isfinite(f)]
        if pixel_fractions:
            result.pixel_intermediate_fraction = float(np.median(pixel_fractions))
            result.pixels_stepped = bool(result.pixel_intermediate_fraction < max_fraction)
            result.dithered = bool(result.row_is_smooth and result.pixels_stepped)
        # Plateau widths of the row average (the staircase's detection, jumps over a window of rows).
        if rows.size > window + 1:
            jumps, centers, heights = _windowed_jumps(row_average, true_depth, window,
                                                      options.plateau_tolerance_fraction * expected)
            widths = plateau_widths(jumps, centers)
            result.jump_positions_mm = centers[jumps[:, 0]].tolist()
            result.plateaus = len(widths)
            if widths:
                result.plateau_width_median_mm = float(np.median(widths))
            if heights:
                result.quantum_jump_height_mm = float(np.median(heights))
            fine_enough = result.rows_per_quantum >= PLATEAU_MIN_STEPS_PER_QUANTUM
            resolved = bool(widths) and result.plateau_width_median_mm >= RAMP_PLATEAU_MIN_WIDTH_ROWS * spacing
            if result.row_is_smooth:
                pass                              # a smooth average has no plateaus: its "jumps" are noise (noted below)
            elif result.plateaus >= RAMP_MIN_COMPLETE_PLATEAUS and fine_enough and resolved:
                result.quantum_plateau_mm = result.plateau_width_median_mm
            elif not fine_enough:
                notes.append(f"{result.rows_per_quantum:.2g} rows per expected quantum: too few for plateau widths "
                             f"(at least {PLATEAU_MIN_STEPS_PER_QUANTUM:g} needed)")
            else:
                notes.append(f"{result.plateaus} complete plateaus in the row average (at least "
                             f"{RAMP_MIN_COMPLETE_PLATEAUS}, each of {RAMP_PLATEAU_MIN_WIDTH_ROWS:g} rows or more, "
                             "needed): plateaus not resolved")
        else:
            notes.append("too few rows for the plateau widths")
    if result.dithered:
        notes.append("the row average is smooth over stepped single pixels: the quantizer is dithered "
                     "(noise or interpolation), so its plateaus are not resolved and the quantum comes from the pooled "
                     "depth levels")
    elif result.row_is_smooth:
        notes.append("the row average is smooth and so are the single pixels: no quantization steps are seen")
    if math.isfinite(result.quantum_plateau_mm):
        result.quantum_mm, result.quantum_method = result.quantum_plateau_mm, QUANTUM_METHOD_PLATEAUS
    elif math.isfinite(levels_mm):
        result.quantum_mm = levels_mm
        result.quantum_method, result.quantum_is_lsb = QUANTUM_METHOD_DEPTH_LEVELS, bool(is_lsb)
    if math.isfinite(result.quantum_mm) and not result.quantum_is_lsb:
        result.q_px = float(session.geometry.disparity_quantum_px(result.quantum_mm, mean_depth))
    elif result.quantum_is_lsb and math.isfinite(expected) and q_expected is not None:
        notes.append(f"the pooled depth levels are no finer than the output LSB ({lsb_mm:g} mm), against an expected "
                     f"quantum of {expected:.3g} mm: the quantum is not resolved by this method (no implied q)")
    result.note = "; ".join(notes)
    return result


# ---------------------------------------------------------------------------
# The analysis
# ---------------------------------------------------------------------------
def run_depth_resolution(session: Session, out_dir: str | Path, previous: dict | None = None,
                         options: DepthResolutionOptions | None = None) -> DepthResolutionResult | None:
    """Analysis B-Z, Section 11.2 Steps 1 to 6 (module docstring). Returns None when the manifest has no procedure Z
    frames. ``previous`` may hold Analysis A's result under "A" (its q for the predicted quantum and its fixed-pattern
    maps for the ramp). ``out_dir`` is where write_outputs will write; nothing is written here."""
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
    ramps: list[RampResult] = []
    for station in sorted({r.station_z_mm for r in records}):
        here = [r for r in records if r.station_z_mm == station]
        ladder = select(here, subseries=SUBSERIES_LADDER)
        ramp = select(here, subseries=SUBSERIES_RAMP)
        stairs = select(here, subseries=SUBSERIES_STAIRCASE)
        if ladder:
            visits = _load_visits(session, ladder, sizes)
            patches.extend(_analyze_station(session, station, visits, options, rng, notes))
        elif not ramp:
            notes.append(f"station {station:g} mm has no ladder or ramp poses")
        for group in group_by_pose(ramp).values():              # the ladder has the reduced stations only, the ramp every one
            ramps.append(_analyze_ramp(session, station, group, previous, options, lsb))
        if stairs:                                              # the optional second pass
            staircases.append(_analyze_staircase(session, station, stairs, [r for r in ladder if r.visit == VISIT_A],
                                                 previous, options, lsb))
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
    return DepthResolutionResult(patches=patches, staircases=staircases, scaling=scaling, notes=notes, options=options,
                                 ramps=ramps)


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


def _figure_ramp(result: DepthResolutionResult, out_dir: Path) -> list[Path] | None:
    """The ramp figure: per station the row average (and a few single pixels) against the true depth, with the plateau edges
    and the identity line."""
    if not result.ramps:
        return None
    figure, axes = _grid_figure(len(result.ramps), 3)
    for axis, ramp in zip(axes, result.ramps):
        x = np.array(ramp.true_depth_mm) - ramp.station_z_mm
        for index, (pixel_x, curve) in enumerate(zip(ramp.pixel_true_mm, ramp.pixel_curves_mm)):
            axis.plot(np.array(pixel_x) - ramp.station_z_mm, np.array(curve) - ramp.station_z_mm, linewidth=0.7,
                      alpha=0.7, color=OKABE_ITO_SKY_BLUE, label="single pixels (temporal median)" if index == 0 else None)
        axis.plot(x, np.array(ramp.row_average_mm) - ramp.station_z_mm, color=OKABE_ITO_VERMILLION, linewidth=1.4,
                  label="row average")
        axis.plot(x, x, color=OKABE_ITO_BLACK, linestyle="--", linewidth=0.7, label="true depth")
        for position in (ramp.jump_positions_mm if math.isfinite(ramp.quantum_plateau_mm) else []):   # edges of real plateaus
            axis.axvline(position - ramp.station_z_mm, color=OKABE_ITO_BLUISH_GREEN, linestyle=":", linewidth=0.7)
        axis.set_xlabel("true depth - Z0 (mm)")
        axis.set_ylabel("sensed depth - Z0 (mm)")
        title = f"Z0 = {ramp.station_z_mm:g} mm, tilt {ramp.tilt_deg:.2f} deg"
        if math.isfinite(ramp.quantum_mm):
            title += f"\nquantum {ramp.quantum_mm:.3g} mm"
            if math.isfinite(ramp.predicted_quantum_mm):
                title += f" (q Z^2 / k {ramp.predicted_quantum_mm:.3g})"
        if ramp.dithered:
            title += ", dithered"
        axis.set_title(title, fontsize="small")
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize="x-small")
    return save_figure(figure, out_dir / FIGURE_STEMS["ramp"])


def _figure_quantum(result: DepthResolutionResult, out_dir: Path) -> list[Path] | None:
    """The measured depth quantum against Z (ramp, and staircase where it was captured) with q Z^2 / k of Analysis A."""
    ramps = [r for r in result.ramps if math.isfinite(r.q_px)]               # not the output LSB standing in for a quantum
    stairs = [s for s in result.staircases if math.isfinite(s.quantum_mm)]
    if not ramps and not stairs:
        return None
    figure, axis = new_figure(result.options.figure_width_in, result.options.figure_height_in)
    if ramps:
        axis.plot([r.station_z_mm for r in ramps], [r.quantum_mm for r in ramps], "o", color=OKABE_ITO_VERMILLION,
                  label="ramp")
        dithered = [r for r in ramps if r.dithered]
        if dithered:
            axis.plot([r.station_z_mm for r in dithered], [r.quantum_mm for r in dithered], "o", markersize=9,
                      markerfacecolor="none", color=OKABE_ITO_BLACK, label="ramp, dithered")
    if stairs:
        axis.plot([s.station_z_mm for s in stairs], [s.quantum_mm for s in stairs], "s", color=OKABE_ITO_BLUE,
                  label="staircase (optional)")
    predicted = sorted({(r.station_z_mm, r.predicted_quantum_mm) for r in result.ramps
                        if math.isfinite(r.predicted_quantum_mm)}
                       | {(s.station_z_mm, s.predicted_quantum_mm) for s in result.staircases
                          if math.isfinite(s.predicted_quantum_mm)})
    if predicted:
        axis.plot(*zip(*predicted), "-", color=OKABE_ITO_BLACK, linewidth=0.8, label="q Z^2 / k (Analysis A)")
    axis.set_xscale("log")
    axis.set_yscale("log")
    axis.set_xlabel("station Z0 (mm)")
    axis.set_ylabel("depth quantum (mm)")
    axis.grid(True, alpha=0.3, which="both")
    axis.legend(fontsize="small")
    return save_figure(figure, out_dir / FIGURE_STEMS["quantum"])


def _ramp_columns(ramp: RampResult | None) -> dict[str, Any]:
    """The ramp columns of a summary row (NaN and empty for a station without a ramp)."""
    if ramp is None:
        return {"ramp_tilt_deg": float("nan"), "ramp_fixed_pattern_subtracted": "", "ramp_plateaus": "",
                "ramp_quantum_mm": float("nan"), "ramp_quantum_predicted_mm": float("nan"), "ramp_q_px": float("nan"),
                "ramp_quantum_method": "", "ramp_dithered": ""}
    return {"ramp_tilt_deg": ramp.tilt_deg, "ramp_fixed_pattern_subtracted": ramp.fixed_pattern_subtracted,
            "ramp_plateaus": ramp.plateaus, "ramp_quantum_mm": ramp.quantum_mm,
            "ramp_quantum_predicted_mm": ramp.predicted_quantum_mm, "ramp_q_px": ramp.q_px,
            "ramp_quantum_method": ramp.quantum_method, "ramp_dithered": "" if ramp.dithered is None else ramp.dithered}


def _staircase_columns(stair: StaircaseResult | None) -> dict[str, Any]:
    """The staircase columns of a summary row (NaN and empty for a station without the optional staircase)."""
    return {"staircase_quantum_mm": float("nan") if stair is None else stair.quantum_mm,
            "staircase_quantum_predicted_mm": float("nan") if stair is None else stair.predicted_quantum_mm,
            "staircase_quantum_method": "" if stair is None else stair.quantum_method}


def write_outputs(result: DepthResolutionResult, out_dir: str | Path) -> list[Path]:
    """Z_resolution_summary.csv (one row per station and patch size; a station with a ramp but no ladder, the stations
    outside the reduced set, has one row with an empty ``patch_px`` that carries its ramp), Z_resolution_rungs.csv (one row
    per station, patch size and ladder rung, with ``truth_reliable`` and the robot-repeatability ratio), Z_ramp_rows.csv
    (the row averages of the ramps), Z_resolution_details.json and the figures (detection fraction with the fits, sensed
    against true step, ramp, quantum against Z, and the staircase when it was captured)."""
    out_dir = Path(out_dir)
    nan = float("nan")
    rows = []
    for p in result.patches:
        stair, ramp = result.staircase(p.station_z_mm), result.ramp(p.station_z_mm)
        rows.append({
            "station_z_mm": p.station_z_mm, "patch_px": p.patch_px, "gain": p.gain, "intercept_mm": p.intercept_mm,
            "max_deviation_mm": p.max_deviation_mm, "tau_mm": p.tau_mm, "delta_50_mm": p.delta_50_mm,
            "delta_50_lower_mm": p.delta_50_lower_mm, "delta_50_upper_mm": p.delta_50_upper_mm,
            "cycles": p.cycles, "false_alarm_fraction": p.false_alarm_fraction, "tiles_per_pair": p.tiles_per_pair,
            "truth_source": TRUTH_SOURCE, "truth_reliable": p.flagged_rungs == 0,
            "delta_50_is_bound": p.delta_50_is_bound, "smallest_rung_mm": p.smallest_rung_mm,
            "robot_repeatability_to_smallest_rung": (nan if not p.rung_robot_ratio else max(p.rung_robot_ratio)),
            "robot_readback_repeatability_mm": p.robot_readback_repeatability_mm,
            "readback_minus_commanded_mm": p.readback_minus_commanded_mm,
            "robot_scatter_exceeds_spec": p.robot_scatter_exceeds_spec,
            **_ramp_columns(ramp), **_staircase_columns(stair),
            "note": "; ".join(x for x in (p.note, "" if ramp is None else f"ramp: {ramp.note}" if ramp.note else "",
                                          "" if stair is None else stair.note) if x)})
    ladder_stations = {p.station_z_mm for p in result.patches}
    for ramp in result.ramps:                                   # ramp-only stations
        if ramp.station_z_mm not in ladder_stations:
            rows.append({"station_z_mm": ramp.station_z_mm, "patch_px": "", **_ramp_columns(ramp),
                         **_staircase_columns(result.staircase(ramp.station_z_mm)),
                         "note": "ramp only (no step ladder at this station)" + (f"; {ramp.note}" if ramp.note else "")})
    rows.sort(key=lambda row: (row["station_z_mm"], row["patch_px"] if row["patch_px"] != "" else 0))
    rung_rows = [
        {"station_z_mm": p.station_z_mm, "patch_px": p.patch_px, "step_mm": label, "true_step_mm": true,
         "mean_delta_mm": delta, "detections": hits, "trials": trials, "detection_fraction": hits / trials,
         "readback_minus_commanded_mm": offset, "robot_repeatability_ratio": ratio, "truth_source": TRUTH_SOURCE,
         "truth_reliable": ok}
        for p in result.patches
        for label, true, delta, hits, trials, offset, ratio, ok in zip(
            p.rung_labels_mm, p.steps_mm, p.mean_delta_mm, p.detections, p.trials, p.rung_readback_offset_mm,
            p.rung_robot_ratio, p.rung_reliable)]
    ramp_rows = [
        {"station_z_mm": r.station_z_mm, "row": index, "true_depth_mm": true, "row_average_mm": average,
         "row_average_minus_true_mm": average - true}
        for r in result.ramps for index, (true, average) in enumerate(zip(r.true_depth_mm, r.row_average_mm))]
    written = [write_csv_rows(out_dir / SUMMARY_CSV_NAME, rows, SUMMARY_COLUMNS),
               write_csv_rows(out_dir / RUNGS_CSV_NAME, rung_rows, RUNG_COLUMNS),
               write_csv_rows(out_dir / RAMP_CSV_NAME, ramp_rows, RAMP_COLUMNS)]
    details = {
        "patches": [p.__dict__ for p in result.patches],
        "ramps": [r.__dict__ for r in result.ramps],
        "staircases": [s.__dict__ for s in result.staircases],
        "patch_size_scaling": {f"{k:g}": v for k, v in result.scaling.items()},
        "forward_model_terms": result.forward_model_terms(), "notes": result.notes}
    written.append(write_json(out_dir / DETAILS_JSON_NAME, details))
    written += _figure_detection(result, out_dir)
    written += _figure_sensed(result, out_dir)
    for figure in (_figure_ramp(result, out_dir), _figure_quantum(result, out_dir), _figure_staircase(result, out_dir)):
        if figure:                                              # absent when its data were not captured
            written += figure
    return written
