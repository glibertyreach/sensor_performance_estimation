"""
Analysis D: minimum detectable size at 50, 10 and 5 percent, with a PREDICTED 0 percent point (procedure document,
Section 13), Steps 1 to 11, in the Z-sweep design (redesign note, Sections 2 and 5).

D_50, D_10 and D_5 are measured points; the lowest measured point is D_5 (DETECTION_LOW_PROBABILITY = 5 percent, which
the 300 extended trials resolve to about +/- 2.5 percent). The 0 percent level D_0 is NOT measured: a smooth curve never
reaches zero and no finite number of trials proves a probability is zero. D_0 is the fitted psychometric curve
extrapolated below D_5 to the level DETECTION_ZERO_PREDICTION_LEVEL (1 percent), reported as ``d0_predicted`` with the
boolean column ``d0_is_prediction`` (always True) and a note column that says so; it is NaN, with the note saying why,
where there is no fit or the fit did not converge. Every figure draws the predicted point with a hollow marker and a
dashed extrapolated segment and labels it "predicted".

What it computes
    For every configuration (target kind disk or cutout, gap, station, field position) the detection probability
    of each feature of the plate (FEATURE_COUNT features, one blank site per feature), from single-frame trials: one
    trial is the FIRST frame of a pose (the frames of one pose are never separate trials, Section 8 Step 5). A frame
    yields one trial per feature and one per blank site. The first frame of each C pose of a matching configuration
    (same target, gap, station and field) is added as an extra trial and flagged as reused (Section 8, Step 6; the
    CSV counts new and reused trials). The detection "levels" are the (feature, station) pairs: 3 features x 9
    stations = 27 values of D_px = D f_x / Z, spaced by the station ratio with the overlaps between neighbors.
    From the trials: false-alarm-calibrated thresholds and the false-alarm rate gamma of every station (from the
    blank sites), and, POOLED over the (feature, station) pairs of a (kind, gap, field, rule) group, the
    psychometric fit on ln D_px, D_50 and D_10 (fitted), D_5 (measured) and the predicted D_0 in D_px and converted to
    mm at each station, the overlap
    (scaling) test between neighboring features, and a logistic regression with the depth noise as a covariate.

Conventions (docs/design/code_design.md, Section 4)
    Camera frame = left IR camera; depth is camera z in mm, NaN for no-reads; pixels (u, v) = (column, row);
    D_px = D f_x / Z; theta = D / Z in mrad. Detection probabilities are dimensionless.

How the steps are implemented
    2  Detection rule. Window radius D_px/2 + DETECTION_WINDOW_MARGIN_PX around the projected feature center.
       Deviation from the reference planes (the front and back planes fitted to the single frame away from the
       edges, falling back to the registered planes, as common.reference_planes does): disk Delta = Z_back - Z,
       cutout Delta = Z - Z_front. A candidate is a valid read with Delta > tau. The feature is detected when at
       least DETECTION_MIN_CONNECTED_PX candidates are 8-connected. Primary rule: no-reads are not candidates.
       "No-read-inclusive" rule (cutouts): no-reads inside the window (on a pixel whose ray hits the plate) are
       candidates for every tau. Both rules are always evaluated. A blank site is large enough (the largest search
       window) to hold the window of any feature, so its window is centered on the blank site with the radius of
       the feature it serves (the same D_px as that feature at that station).
       IMPLEMENTATION: instead of re-labeling the window for every trial threshold, each window is reduced ONCE to
       its detection statistic S, the largest tau at which the rule still detects (the "k-th connected level"
       of the window: the greatest value u such that the pixels with Delta >= u contain a connected region of at
       least k pixels; k = 2 is the best adjacent pair, max over pairs of min(Delta_i, Delta_j)). The window is
       detected at threshold tau exactly when S > tau (verified against the labeling in the tests).
    3  Threshold. tau is set PER WINDOW SIZE (per feature) from the blank sites of that size over all trials of the
       configuration (one station): tau is the (1 - DETECTION_FALSE_ALARM_TARGET) quantile of the blank windows' S
       ("higher" interpolation, so the false-alarm fraction does not exceed the target). That makes the false-alarm
       fraction of the RULE (window level, after the connected-pixel criterion), not of single pixels, equal the
       target. The measured false-alarm rate gamma is the fraction of all blank-site trials of the station that the
       rule then declares detected (with fewer than 1 / target trials per level the quantile is the maximum and gamma
       is 0 of n; the Clopper-Pearson interval is reported either way): gamma per station. The reported tau_mm is the
       median over features (the per-feature values are in the details).
    1  Independence: lag-1 autocorrelation of each feature's detected/missed sequence in acquisition order
       against +/- INDEPENDENCE_SIGMA_MULTIPLE / sqrt(n), and the phi coefficient between the outcomes of adjacent
       features in the same frame against the same band. With many features some exceed 2 / sqrt(n) by chance, so
       ``independence_ok`` is False only when MORE than max(1, ceil(p x tested)) features (or pairs) are outside,
       p = 2 (1 - Phi(multiplier)); the flagged ones are listed in the details. Features whose sequences do not
       vary (always or never detected) carry no information and are not tested.
    4  Pooled psychometric fit on ln D_px (stats.psychometric.fit_all_curves): the detections of all (feature,
       station) pairs of a (kind, gap, field, rule) group against D_px, with gamma fixed at the group's blank-site
       false-alarm rate (the blank detections over the blank trials of all its stations; gamma per station is
       reported with every configuration and used for the per-point corrections of the overlap test). Pairs whose
       D_px agree within MERGE_D_PX_TOLERANCE (the six-station shift that makes neighboring features coincide) are
       merged. D_50 and D_10 come from the best curve by deviance, in D_px, and are converted to mm at each station,
       D_mm = D_px Z / f_x. A threshold is reported only when the data bracket it (the corrected proportions reach
       it and start below it); otherwise it is NaN and the status column says why.
    5  Model range of D_10 over the logistic, normal and Weibull fits; model-free isotonic crossings (0.1, 0.5) of
       the corrected proportions.
    5b D_5, fitted: the best fitted curve (by deviance, as D_10) inverted at DETECTION_LOW_PROBABILITY, reported when the
       corrected proportions bracket it (d5_status), with the range over the three shapes (d5_model_min / _max) and the
       bootstrap interval of Step 9 (d5_lower / d5_upper).
    6  D_5 empirical (measured), reported beside the fitted D_5, as a D_px BRACKET: per merged level the one-sided
       Clopper-Pearson upper bound psi_U at CONFIDENCE_LEVEL, corrected P*_U = (psi_U - gamma) / (1 - gamma); D_5 is the largest D_px such that P*_U <=
       DETECTION_LOW_PROBABILITY at that level and at every smaller one (the smallest level whose corrected
       probability is demonstrably at or below 5 percent, and the lowest measured point); NaN when even the smallest
       level fails (not demonstrated). The bracket is [D_5, next level up], converted to mm at each station
       (columns d5_empirical and d5_empirical_next).
    7  D_0 PREDICTED (a prediction, never a measurement): the best fitted curve (by deviance) inverted at the corrected
       probability DETECTION_ZERO_PREDICTION_LEVEL, i.e. the fit extrapolated below the lowest measured point D_5
       (:func:`predict_d0`); the range of the same inversion over the three fitted shapes is the model range
       (d0_predicted_model_min / _max). ``d0_is_prediction`` is always True and ``d0_predicted_note`` explains the
       extrapolation; with no fit, or when the best fit did not converge, D_0 is NaN and the note says so. The threshold
       model (stats.psychometric.fit_threshold_model, a Weibull with a hard floor) on the pooled levels gives a second
       prediction, ``d0_model``, with its profile-likelihood interval; "not estimable" when every level detects nothing
       (its ValueError is caught and recorded).
    8  Geometric limit (cutouts): the diameter at which A_geo of Analysis C, Step 7 reaches zero, by bisection on a
       probe cutout at the plate center (area.geometric_limit_diameter_mm), cameras only and with the projector, per
       station (mm, and px at that station).
    9  Bootstrap over poses (trials), stratified by station: each resample draws poses with replacement within each
       station, recomputes the per-feature tau from the resampled blank statistics, the pooled gamma and the pooled
       counts, and re-fits ONE curve family (the best family of the original fit; refitting all three in every
       resample would triple the run time) to give D_50, D_10, D_5 and the predicted D_0 in D_px (d5_lower / d5_upper and
       d0_predicted_lower / d0_predicted_upper; the interval of the prediction carries the same flag as the value). The
       model D_0 interval is the profile-likelihood interval of Step 7, not a bootstrap. The number of resamples is BOOTSTRAP_RESAMPLES, reduced
       to DetectionOptions.reduced_bootstrap_resamples when the stations of the group have fewer than
       DetectionOptions.full_bootstrap_min_poses poses each on average (stated in the details).
    10 Every minimum in mm, px and mrad (D_px and theta are constant over stations for a pooled minimum, D_mm scales
       with Z); the minimum diameter in mm against Z in a figure.
    11 D_detect_summary.csv (per configuration), D_pooled_summary.csv (per group), D_overlap_test.csv,
       D_detect_details.json, figures (pooled psychometric curves with binomial error bars and the minimums marked,
       the predicted D_0 as a hollow marker on a dashed extrapolation; the minimum diameter against Z, the predicted D_0
       with a hollow marker and a dashed line).
    12 Overlap (scaling) test (analysis.overlap): per group and pair of neighboring features, the mean difference of
       their corrected detection curves over the D_px range they share and whether zero lies inside its bootstrap
       interval; disagreement is attributed to sigma_tot(Z).
    13 Noise covariate: logistic regression (stats.logistic) of the pooled detection counts on ln D_px and ln
       sigma_tot(Z), sigma_tot per station from Analysis A (``previous["A"]``, log-log interpolated between its
       stations); the coefficient of ln sigma_tot with its standard error and the likelihood-ratio p-value of
       dropping it are reported. Skipped, with a note, when A did not run or the design cannot separate the two
       covariates.

Helpers other modules import from here: ``collect_trials`` / ``ConfigTrials`` (Analysis E, Step 6, recomputes the two
rules per diameter), ``window_statistic``, ``detected_at``.
"""
from __future__ import annotations

import math
import time
import warnings
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from scipy import ndimage
from scipy.stats import norm

from sensorperf.analysis.area import (
    KIND_COLORS, KIND_MARKERS, OKABE_ITO_BLACK, OKABE_ITO_BLUE, OKABE_ITO_ORANGE, OKABE_ITO_VERMILLION,
    STATION_COLORS, AreaOptions, geometric_limit_diameter_mm, log_axis,
)
from sensorperf.analysis.common import (
    PoseGeometry, connected_components, new_figure, pose_geometry, reference_planes, save_figure, write_csv_rows,
    write_json,
)
from sensorperf.analysis.overlap import (
    OverlapOptions, TransferCurve, binomial_standard_error, overlap_tests, sigma_tot_at, sigma_tot_by_station,
)
from sensorperf.geometry.targets import (
    FEATURE_BLANK, FEATURE_CUTOUT, FEATURE_DISK, SURFACE_NONE, Feature, StereoGeometry,
)
from sensorperf.io.capture_set import load_stack
from sensorperf.io.manifest import SUBSERIES_JITTER, SUBSERIES_MAIN, FrameRecord, group_by_pose, pose_order, select
from sensorperf.io.session import Session
from sensorperf.parameters import CharacterizationParameters, PROCEDURE_AREA, PROCEDURE_DETECTION
from sensorperf.stats.intervals import bootstrap_statistic, clopper_pearson, clopper_pearson_upper
from sensorperf.stats.logistic import fit_logistic, likelihood_ratio_pvalue
from sensorperf.stats.psychometric import (
    PsychometricFit, PsychometricFitParameters, best_fit, corrected_rate, fit_all_curves, fit_psychometric,
    fit_threshold_model, isotonic_threshold,
)

# ---------------------------------------------------------------------------
# Names and constants
# ---------------------------------------------------------------------------
RULE_PRIMARY = "primary"
RULE_INCLUSIVE = "no_read_inclusive"
RULES = (RULE_PRIMARY, RULE_INCLUSIVE)
"""The two detection rules of Step 2 (the second applies to cutouts only)."""
SUMMARY_FILE_NAME = "D_detect_summary.csv"
POOLED_FILE_NAME = "D_pooled_summary.csv"
OVERLAP_FILE_NAME = "D_overlap_test.csv"
DETAILS_FILE_NAME = "D_detect_details.json"
MERGE_D_PX_TOLERANCE = 5.0e-3
"""(feature, station) pairs whose D_px agree within this relative tolerance are the same level of the pooled curve:
neighboring features coincide in D_px after six stations (FEATURE_LADDER_RATIO = STATION_RATIO^6), up to the 1 mm
rounding of the stations (about 0.1 percent)."""
MIN_DISTINCT_LEVELS_FOR_FIT = 3
"""A pooled psychometric fit needs at least this many distinct D_px levels with trials."""
LOG_COLUMNS_REGRESSION = 2
"""Covariates of the noise regression: ln D_px and ln sigma_tot."""
MIN_REGRESSION_SPREAD = 1.0e-6
"""The centered covariate matrix of the noise regression has rank below LOG_COLUMNS_REGRESSION when its singular values
fall below this (ln D_px and ln sigma_tot then vary together, or one of them is constant)."""
OVERLAP_COLUMNS = ("kind", "gap_mm", "field", "rule", "feature_small", "feature_large", "status", "shared_low_px",
                   "shared_high_px", "points_small", "points_large", "mean_difference", "interval_lower",
                   "interval_upper", "agrees", "resamples", "sigma_tot_small_mm", "sigma_tot_large_mm", "note")
"""Columns of D_overlap_test.csv."""
C_REUSE_SUBSERIES = (SUBSERIES_MAIN, SUBSERIES_JITTER)
"""C sub-series whose first frames may be reused as D trials (Section 8, Step 6)."""
SOURCE_D = "D"
SOURCE_C = "C"
"""Trial sources: a D pose, or the first frame of a C pose (flagged as reused)."""
NO_THRESHOLD = math.nan
"""A threshold that is not estimable."""
MIN_PAIRS_FOR_PHI = 5
"""Neighbor correlation needs at least this many poses with both outcomes to be tested."""
MIN_TRIALS_FOR_LAG = 5
"""Autocorrelation needs at least this many trials of a feature."""
ALLOWED_FLAGS_FLOOR = 1
"""Independence: at least this many flagged tests are tolerated (multiple testing)."""
SAMPLE_VARIANCE_GUARD = 1.0e-12
"""A binary sequence whose sum of squares is below this has no variance (never or always detected)."""
BOOTSTRAP_STAT_NAMES = ("d50_px", "d10_px", "d5_px", "d0_predicted_px", "gamma", "tau_mm")
"""The quantities the bootstrap resamples, in order."""
PSYCHOMETRIC_CURVE_POINTS = 200
"""Points of the fitted curves drawn in the figures."""
P_STAR_D50 = 0.5
P_STAR_D10 = 0.1
"""The corrected detection probabilities that define D_50 and D_10 (Section 13, Steps 4 and 5). D_5 uses
DETECTION_LOW_PROBABILITY and the predicted D_0 DETECTION_ZERO_PREDICTION_LEVEL, both procedure parameters."""
BOOTSTRAP_SLOT_D50, BOOTSTRAP_SLOT_D10, BOOTSTRAP_SLOT_D5, BOOTSTRAP_SLOT_D0 = 0, 1, 2, 3
"""Positions of D_50, D_10, D_5 and the predicted D_0 in the bootstrap statistic (BOOTSTRAP_STAT_NAMES)."""
NOT_RESAMPLED = 0.0
"""Bootstrap slot of a quantity that the original data do not give (so it is not resampled; its interval is NaN)."""
D0_NOTE_PREDICTION = (
    "PREDICTION, not a measurement: extrapolation of the best fitted psychometric curve ({curve}) below the lowest "
    "measured point (D_5, corrected probability {low:g}) to the corrected probability {level:g}")
"""Note of a predicted D_0 (``d0_predicted_note``); formatted with the curve name, DETECTION_LOW_PROBABILITY and
DETECTION_ZERO_PREDICTION_LEVEL."""
D0_NOTE_NO_FIT = ("predicted D_0 not available: no psychometric fit (too few trials); D_0 is a prediction from a fitted "
                  "curve below the lowest measured point and there is no curve")
"""Note of a predicted D_0 where the group has too few trials for a fit."""
D0_NOTE_NOT_CONVERGED = ("predicted D_0 not available: the best psychometric fit ({curve}) did not converge, so its "
                         "extrapolation below the lowest measured point would not be a prediction worth reporting")
"""Note of a predicted D_0 where the best fit did not converge."""
D0_NOTE_NOT_FINITE = "predicted D_0 not available: the fitted curve ({curve}) does not reach the prediction level"
"""Note of a predicted D_0 whose curve inversion is not a finite size."""
RULE_PLOT_SHIFT = 1.03
"""Figures: the no-read-inclusive points are drawn at this multiple of D_px so that coincident points stay visible."""
HIGHER = "higher"
"""numpy quantile method: the smallest observed value at or above the quantile (conservative false-alarm control)."""


# ---------------------------------------------------------------------------
# Options and result
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class DetectionOptions:
    """Settings of Analysis D that are not procedure parameters."""

    bootstrap_resamples: int | None = None
    """Number of bootstrap resamples; None means BOOTSTRAP_RESAMPLES for a group with at least
    ``full_bootstrap_min_poses`` poses per station and ``reduced_bootstrap_resamples`` otherwise."""
    reduced_bootstrap_resamples: int = 200
    """Resamples used when a group has fewer than ``full_bootstrap_min_poses`` poses per station."""
    full_bootstrap_min_poses: int = 30
    """Poses (trials) per station, on average over the stations of the group, needed for the full BOOTSTRAP_RESAMPLES."""
    min_trials_per_level: int = 20
    """A (feature, station) pair enters the pooled curve fit only with at least this many trials; fewer at every pair
    and the fit is not attempted ("too few trials")."""
    min_pairs_for_fit: int = 4
    """The pooled fit needs at least this many pairs with min_trials_per_level trials (and
    MIN_DISTINCT_LEVELS_FOR_FIT distinct D_px)."""
    overlap_options: OverlapOptions = OverlapOptions()
    """Settings of the overlap (scaling) test."""
    bootstrap_seed: int = 20261005
    """Seed of the bootstrap generator."""
    include_reused: bool = True
    """Add the first frame of each C pose of a matching configuration as an extra trial (Section 8, Step 6)."""
    area_options: AreaOptions = AreaOptions()
    """Settings of the geometric-limit computation (Step 8)."""
    max_geometric_scale: float = 1.0
    """Upper end of the geometric-limit bracket as a multiple of the largest cutout diameter."""


@dataclass
class LevelSpec:
    """One feature of a plate (a detection level at each station): the feature, the blank site that serves it (same
    level index), and their diameters (the blank site is as large as the feature's search window at Z_MAX, so
    its window at any station, cut to the feature's size, lies inside it)."""

    target_id: str
    level_index: int | None
    site_id: str
    blank_id: str | None
    diameter_mm: float
    blank_diameter_mm: float | None


@dataclass
class ConfigTrials:
    """The trials of one configuration: per pose and level the detection statistic S of the feature window and of
    the blank window under each rule (NaN where the pose does not carry that level or the window left the image)."""

    kind: str
    gap_mm: float | None
    station_z_mm: float
    field: int
    levels: list[LevelSpec]
    pose_keys: list[tuple]
    """Pose keys in acquisition order."""
    sources: list[str]
    """SOURCE_D or SOURCE_C (reused) per pose."""
    s_feature: dict[str, np.ndarray]
    s_blank: dict[str, np.ndarray]
    """(poses, levels) arrays per rule."""

    @property
    def target_ids(self) -> list[str]:
        return sorted({level.target_id for level in self.levels})

    def level_diameters_mm(self) -> np.ndarray:
        return np.array([level.diameter_mm for level in self.levels])

    def rules(self) -> tuple[str, ...]:
        """The rules evaluated for this kind: both for cutouts, the primary one for disks."""
        return RULES if self.kind == FEATURE_CUTOUT else (RULE_PRIMARY,)


# ---------------------------------------------------------------------------
# Step 2: the detection statistic of a window
# ---------------------------------------------------------------------------
def deviation_image(kind: str, depth: np.ndarray, z_front: np.ndarray, z_back: np.ndarray | None) -> np.ndarray:
    """Step 2: Delta in mm: Z_back - Z for a disk (toward the sensor), Z - Z_front for a cutout (away from the
    sensor). NaN where the depth is NaN."""
    if kind == FEATURE_DISK:
        if z_back is None:
            raise ValueError("the disk rule needs a back plane")
        return z_back - depth
    return depth - z_front


def _largest_component_at_least(values: np.ndarray, level: float, minimum: int) -> bool:
    """Whether the pixels with ``values >= level`` contain an 8-connected region of at least ``minimum`` pixels."""
    mask = values >= level
    if np.count_nonzero(mask) < minimum:
        return False
    labels, count = connected_components(mask)
    if count == 0:
        return False
    return int(np.bincount(labels.ravel())[1:].max()) >= minimum


def window_statistic(delta: np.ndarray, window: np.ndarray, minimum_connected: int,
                     no_read_candidates: np.ndarray | None = None) -> float:
    """The detection statistic S of a window: the largest tau at which the rule detects (module docstring).

    ``delta`` is the deviation image (NaN where there is no read), ``window`` the boolean window mask, and
    ``no_read_candidates`` (the no-read-inclusive rule) a boolean mask of window pixels whose no-read counts as a
    candidate at every tau (their value is +infinity). The window is detected at threshold tau when S > tau.
    Returns -inf when the window never holds ``minimum_connected`` connected candidates at any tau (for example
    fewer valid pixels than that)."""
    rows, cols = np.nonzero(window)
    if rows.size == 0:
        return -math.inf
    r0, r1, c0, c1 = rows.min(), rows.max() + 1, cols.min(), cols.max() + 1
    values = np.where(window[r0:r1, c0:c1], delta[r0:r1, c0:c1], -math.inf)
    values = np.where(np.isnan(values), -math.inf, values)
    if no_read_candidates is not None:
        values = np.where(no_read_candidates[r0:r1, c0:c1] & window[r0:r1, c0:c1] & ~np.isfinite(delta[r0:r1, c0:c1]),
                          math.inf, values)
    if minimum_connected <= 1:
        return float(values.max())
    if minimum_connected == 2:
        # Adjacent pairs in the four forward directions cover all 8-connected pairs: the best pair's smaller value.
        best = -math.inf
        for a, b in ((values[:, :-1], values[:, 1:]), (values[:-1, :], values[1:, :]),
                     (values[:-1, :-1], values[1:, 1:]), (values[:-1, 1:], values[1:, :-1])):
            if a.size:
                best = max(best, float(np.minimum(a, b).max()))
        return best
    # General k: the largest value u whose superlevel set {values >= u} still holds a big enough region (bisection
    # over the sorted distinct values; the predicate is monotone in u).
    candidates = np.unique(values[values > -math.inf])
    if candidates.size < minimum_connected or not _largest_component_at_least(values, candidates[0], minimum_connected):
        return -math.inf
    low, high = 0, candidates.size - 1                   # candidates[low] satisfies the predicate
    while low < high:
        middle = (low + high + 1) // 2
        if _largest_component_at_least(values, candidates[middle], minimum_connected):
            low = middle
        else:
            high = middle - 1
    return float(candidates[low])


def detected_at(delta: np.ndarray, window: np.ndarray, tau: float, minimum_connected: int,
                no_read_candidates: np.ndarray | None = None) -> bool:
    """Step 2 by direct labeling: candidates are window pixels with Delta > tau (and, for the no-read-inclusive
    rule, the no-read pixels in ``no_read_candidates``); detected when a connected region (8-connectivity,
    common.connected_components) has at least ``minimum_connected`` pixels. Equivalent to
    ``window_statistic(...) > tau``; kept for the tests and for single windows."""
    with np.errstate(invalid="ignore"):
        candidates = window & (delta > tau)
    if no_read_candidates is not None:
        candidates = candidates | (window & no_read_candidates & ~np.isfinite(delta))
    labels, count = connected_components(candidates)
    if count == 0:
        return False
    return int(np.bincount(labels.ravel())[1:].max()) >= minimum_connected


# ---------------------------------------------------------------------------
# Collecting the trials of a session
# ---------------------------------------------------------------------------
def _level_specs(target_features: Sequence[Feature], target_id: str, kind: str) -> list[LevelSpec]:
    """The levels of a plate: each disk or cutout paired with the blank site of the same level index."""
    blanks = {f.level_index: f for f in target_features if f.kind == FEATURE_BLANK and f.level_index is not None}
    specs = []
    for f in target_features:
        if f.kind != kind:
            continue
        blank = blanks.get(f.level_index)
        specs.append(LevelSpec(target_id=target_id, level_index=f.level_index, site_id=f.site_id,
                               blank_id=None if blank is None else blank.site_id, diameter_mm=f.diameter_mm,
                               blank_diameter_mm=None if blank is None else blank.diameter_mm))
    return specs


def frame_statistics(session: Session, record: FrameRecord, levels: Sequence[LevelSpec],
                     kind: str) -> dict[tuple[str, str], float]:
    """Steps 2 for one frame: ``{(site_id, rule): S}`` for every feature's own window and its blank site's window (cut to
    the feature's size) that lies inside the image. Reference planes are fitted to the frame itself away from the edges (the registered planes where too
    few pixels). The no-read-inclusive statistic is computed for cutouts only."""
    params = session.params
    stack = load_stack([record])
    geometry = pose_geometry(session, record, stack.camera)
    depth = stack.depth[0]
    planes = reference_planes(depth, geometry, params)
    delta = deviation_image(kind, depth, planes.z_front, planes.z_back)
    plate_pixels = geometry.hit.surface != SURFACE_NONE        # no-reads off the plate are not "holes"
    target = geometry.target
    margin = params.detection_window_margin_px
    minimum = params.detection_min_connected_px
    output: dict[tuple[str, str], float] = {}
    for level in levels:
        if level.target_id != record.target_id:
            continue
        for site_id in (level.site_id, level.blank_id):
            if site_id is None:
                continue
            # The window of a feature has radius D_px/2 + margin of its OWN diameter. A blank site is as large as the
            # window of the feature it serves at Z_MAX, so it is cut to that feature's size (the same rule as
            # targets.window_diameter_mm): same D_px, same window, and it never extends beyond the blank site.
            feature = replace(target.feature(site_id), diameter_mm=level.diameter_mm)
            if not _window_inside_image(geometry, feature, margin):
                continue
            window = geometry.feature_window(feature, margin)
            output[(site_id, RULE_PRIMARY)] = window_statistic(delta, window, minimum)
            if kind == FEATURE_CUTOUT:
                output[(site_id, RULE_INCLUSIVE)] = window_statistic(delta, window, minimum,
                                                                     no_read_candidates=plate_pixels)
    return output


def _window_inside_image(geometry: PoseGeometry, feature: Feature, margin_px: float) -> bool:
    """True when the feature window (radius D_px/2 + margin) lies entirely inside the image."""
    u0, v0 = geometry.feature_center_px(feature)
    radius = feature.diameter_mm / geometry.pixel_footprint_mm / 2.0 + margin_px
    camera = geometry.camera
    return (u0 - radius >= -0.5 and u0 + radius <= camera.width - 0.5
            and v0 - radius >= -0.5 and v0 + radius <= camera.height - 0.5)


def _kind_of_target(session: Session, target_id: str) -> str | None:
    """FEATURE_DISK or FEATURE_CUTOUT for an array target, None for any other target."""
    target = session.targets.targets.get(target_id)
    if target is None:
        return None
    kinds = {f.kind for f in target.features}
    if FEATURE_DISK in kinds:
        return FEATURE_DISK
    if FEATURE_CUTOUT in kinds:
        return FEATURE_CUTOUT
    return None


def first_frames(records: Sequence[FrameRecord]) -> list[FrameRecord]:
    """One record per pose: its first frame (a trial is a single frame; Section 8, Step 5)."""
    return [group[0] for group in group_by_pose(records).values()]


def collect_trials(session: Session, records_with_source: Sequence[tuple[FrameRecord, str]],
                   progress=None) -> list[ConfigTrials]:
    """Group single-frame records (record, source) into configurations (target kind, gap, station, field) and compute
    the window statistics of every level of every plate in each (Steps 2 and 3). Poses are in acquisition order."""
    groups: dict[tuple, list[tuple[FrameRecord, str]]] = {}
    for record, source in records_with_source:
        kind = _kind_of_target(session, record.target_id)
        if kind is None or record.gap_mm is None:
            continue
        groups.setdefault((kind, record.gap_mm, record.station_z_mm, record.field), []).append((record, source))
    configs: list[ConfigTrials] = []
    done = 0
    total = len(records_with_source)
    for (kind, gap, station, field_code), members in groups.items():
        order = {key: index for index, key in enumerate(pose_order([r for r, _ in members]))}
        members = sorted(members, key=lambda item: order[item[0].pose_key()])
        levels: list[LevelSpec] = []
        for target_id in dict.fromkeys(r.target_id for r, _ in members):          # plates in order of appearance
            target = session.targets.get(target_id, gap)
            levels += _level_specs(target.features, target_id, kind)
        levels.sort(key=lambda spec: (spec.diameter_mm, spec.target_id))
        rules = RULES if kind == FEATURE_CUTOUT else (RULE_PRIMARY,)
        s_feature = {rule: np.full((len(members), len(levels)), np.nan) for rule in rules}
        s_blank = {rule: np.full((len(members), len(levels)), np.nan) for rule in rules}
        for row, (record, _) in enumerate(members):
            stats = frame_statistics(session, record, levels, kind)
            for column, level in enumerate(levels):
                for rule in rules:
                    if (level.site_id, rule) in stats and level.blank_id and (level.blank_id, rule) in stats:
                        s_feature[rule][row, column] = stats[(level.site_id, rule)]
                        s_blank[rule][row, column] = stats[(level.blank_id, rule)]
            done += 1
            if progress is not None:
                progress(done, total)
        configs.append(ConfigTrials(kind=kind, gap_mm=gap, station_z_mm=station, field=field_code, levels=levels,
                                    pose_keys=[r.pose_key() for r, _ in members], sources=[s for _, s in members],
                                    s_feature=s_feature, s_blank=s_blank))
    return configs


# ---------------------------------------------------------------------------
# Step 3: thresholds and outcomes from the statistics
# ---------------------------------------------------------------------------
def calibrate_tau(s_blank: np.ndarray, false_alarm_target: float) -> np.ndarray:
    """Per-level threshold tau (mm): the (1 - false_alarm_target) quantile ("higher") of the blank statistics of that
    level over the poses. Levels without blank trials get the tau of the nearest level that has one (NaN if none)."""
    taus = np.full(s_blank.shape[1], np.nan)
    for column in range(s_blank.shape[1]):
        values = s_blank[:, column]
        values = values[~np.isnan(values)]
        if values.size:
            taus[column] = np.quantile(values, 1.0 - false_alarm_target, method=HIGHER)
    valid = np.nonzero(~np.isnan(taus))[0]
    for column in np.nonzero(np.isnan(taus))[0]:
        if valid.size:
            taus[column] = taus[valid[np.argmin(np.abs(valid - column))]]
    return taus


def outcomes(s_feature: np.ndarray, s_blank: np.ndarray, taus: np.ndarray) -> tuple[np.ndarray, np.ndarray,
                                                                                    np.ndarray, np.ndarray]:
    """Detection outcomes under per-level thresholds: (feature_detected, blank_detected, feature_valid, blank_valid),
    boolean (poses, levels) arrays; a trial is valid where the statistic exists (not NaN)."""
    feature_valid = ~np.isnan(s_feature)
    blank_valid = ~np.isnan(s_blank)
    with np.errstate(invalid="ignore"):
        feature_detected = feature_valid & (s_feature > taus[None, :])
        blank_detected = blank_valid & (s_blank > taus[None, :])
    return feature_detected, blank_detected, feature_valid, blank_valid


def counts_from_outcomes(feature_detected, blank_detected, feature_valid, blank_valid) -> tuple[np.ndarray,
                                                                                                np.ndarray, float,
                                                                                                int, int]:
    """(successes per level, trials per level, gamma, blank detections, blank trials)."""
    successes = feature_detected.sum(axis=0).astype(float)
    trials = feature_valid.sum(axis=0).astype(float)
    blank_hits = int(blank_detected.sum())
    blank_trials = int(blank_valid.sum())
    gamma = blank_hits / blank_trials if blank_trials else 0.0
    return successes, trials, gamma, blank_hits, blank_trials


# ---------------------------------------------------------------------------
# Curve helpers (Steps 4, 5, 6)
# ---------------------------------------------------------------------------
def _quiet():
    """Silence numpy overflow warnings of the Weibull shape in far tails (a fit that overflows there is still
    valid; the likelihood clips probabilities)."""
    return np.errstate(over="ignore", invalid="ignore", divide="ignore")


def _fit_parameters(params, curve: str | None = None,
                    warm_start: tuple[float, float, float] | None = None) -> PsychometricFitParameters:
    """Fit settings from the procedure parameters; ``warm_start`` (alpha, beta, lapse) of the original fit makes a
    bootstrap refit a single optimizer run."""
    if curve is None:
        return PsychometricFitParameters(lapse_rate_max=params.detection_lapse_rate_max)
    return PsychometricFitParameters(lapse_rate_max=params.detection_lapse_rate_max, curve=curve,
                                     warm_start=warm_start)


def bracketed(corrected: np.ndarray, p_star: float) -> bool:
    """Whether the corrected proportions start below p_star and reach it somewhere: the threshold is then
    interpolated, not extrapolated."""
    return bool(corrected.size and np.min(corrected) < p_star <= np.max(corrected))


def threshold_status(corrected: np.ndarray, p_star: float) -> str:
    """'ok', or why the threshold at p_star is not reported."""
    if not corrected.size:
        return "no trials"
    if np.max(corrected) < p_star:
        return "never reached in the tested range"
    if np.min(corrected) >= p_star:
        return "already exceeded at the smallest level"
    return "ok"


def _fit_thresholds(levels, successes, trials, gamma, params, curve: str,
                    warm_start: tuple[float, float, float] | None = None) -> tuple[float, float, float, float]:
    """(D_50, D_10, D_5, predicted D_0) in the units of ``levels`` from one fitted curve family; NaN for a threshold the
    corrected proportions do not bracket (D_50, D_10, D_5), and for the predicted D_0 when the fit did not converge. The
    predicted D_0 is the curve inverted at DETECTION_ZERO_PREDICTION_LEVEL, an extrapolation, so it needs no bracket."""
    keep = trials > 0
    corrected = np.asarray(corrected_rate(successes[keep] / trials[keep], gamma))
    fit = fit_psychometric(levels[keep], successes[keep], trials[keep], gamma, _fit_parameters(params, curve, warm_start))
    reaches_half, reaches_tenth = bracketed(corrected, P_STAR_D50), bracketed(corrected, P_STAR_D10)
    reaches_low = bracketed(corrected, params.detection_low_probability)
    d0 = fit.threshold(params.detection_zero_prediction_level) if fit.converged else NO_THRESHOLD
    return (fit.threshold(P_STAR_D50) if reaches_half else NO_THRESHOLD,
            fit.threshold(P_STAR_D10) if reaches_tenth else NO_THRESHOLD,
            fit.threshold(params.detection_low_probability) if reaches_low else NO_THRESHOLD, d0)


def empirical_d5(levels: np.ndarray, successes: np.ndarray, trials: np.ndarray, gamma: float, confidence: float,
                 bound: float) -> dict[str, Any]:
    """Step 6: the EMPIRICAL low point, reported beside the fitted D_5 (Section 13: the measured corrected rates of the
    levels nearest to D_5, so that the reader sees the point is measured). Per level (ascending), the one-sided
    Clopper-Pearson upper bound psi_U of the raw detection rate at ``confidence``, corrected for false alarms to P*_U = (psi_U - gamma) / (1 - gamma), and
    whether P*_U is within ``bound`` (DETECTION_LOW_PROBABILITY, 5 percent). D_5 is the largest level such that it and
    every smaller level are within the bound (the smallest levels whose corrected probability is demonstrably at or
    below 5 percent), NaN when even the smallest level is not. Returns ``levels`` (the per-level table), ``d5_empirical`` and
    ``d5_empirical_next`` (the next level up, so that it is a bracket; NaN when every level passes). The levels keep the units
    of ``levels`` (D_px in the pooled analysis)."""
    order = np.argsort(levels, kind="stable")
    rows = []
    passing = True
    d5 = NO_THRESHOLD
    next_level = NO_THRESHOLD
    for index in order:
        n, s = int(trials[index]), int(successes[index])
        if n < 1:
            continue
        upper = clopper_pearson_upper(s, n, confidence)
        corrected_upper = float(corrected_rate(upper, gamma))
        ok = corrected_upper <= bound
        rows.append({"d_px": float(levels[index]), "trials": n, "detections": s, "psi_upper": upper,
                     "p_star_upper": corrected_upper, "within_bound": bool(ok)})
        if passing and ok:
            d5 = float(levels[index])
        elif passing:
            passing = False
            next_level = float(levels[index])
    if passing and rows:                                 # every level passes: the 5 percent point is above the range
        next_level = NO_THRESHOLD
    return {"levels": rows, "d5_empirical": d5, "d5_empirical_next": next_level}


def predict_d0(fits: Mapping[str, PsychometricFit] | None, best: PsychometricFit | None, level: float,
               lowest_measured_probability: float) -> dict[str, Any]:
    """Step 7: the PREDICTED D_0, the best fitted curve inverted at the corrected probability ``level``
    (DETECTION_ZERO_PREDICTION_LEVEL). It lies below the lowest measured point (D_5, at
    ``lowest_measured_probability``), so it is an extrapolation of the fitted curve, never a measurement.

    Returns ``d0`` (D_px; NaN where ``best`` is None, did not converge, or its inversion is not a positive finite size),
    ``model_min`` / ``model_max`` (the range of the same inversion over the converged fitted shapes, NaN without
    one), ``is_prediction`` (always True) and ``note`` (the extrapolation, or why there is no value)."""
    result: dict[str, Any] = {"d0": NO_THRESHOLD, "model_min": NO_THRESHOLD, "model_max": NO_THRESHOLD,
                              "is_prediction": True, "level": level, "note": D0_NOTE_NO_FIT}
    if best is None:
        return result
    if not best.converged:
        result["note"] = D0_NOTE_NOT_CONVERGED.format(curve=best.curve)
        return result
    with _quiet():
        value = best.threshold(level)
        shapes = [f.threshold(level) for f in (fits or {}).values() if f.converged]
    if not (math.isfinite(value) and value > 0.0):
        result["note"] = D0_NOTE_NOT_FINITE.format(curve=best.curve)
        return result
    finite = [v for v in shapes if math.isfinite(v) and v > 0.0]
    result.update(d0=float(value), model_min=float(min(finite)) if finite else NO_THRESHOLD,
                  model_max=float(max(finite)) if finite else NO_THRESHOLD,
                  note=D0_NOTE_PREDICTION.format(curve=best.curve, low=lowest_measured_probability, level=level))
    return result


# ---------------------------------------------------------------------------
# Step 1: independence
# ---------------------------------------------------------------------------
def lag1_autocorrelation(sequence: np.ndarray) -> float:
    """Lag-1 autocorrelation of a 0/1 sequence about its mean; NaN when it does not vary."""
    x = np.asarray(sequence, dtype=float)
    centered = x - x.mean()
    denominator = float(np.sum(centered ** 2))
    if denominator < SAMPLE_VARIANCE_GUARD or x.size < 2:
        return math.nan
    return float(np.sum(centered[:-1] * centered[1:]) / denominator)


def phi_coefficient(a: np.ndarray, b: np.ndarray) -> float:
    """Phi coefficient (Pearson correlation of two 0/1 sequences); NaN when either does not vary."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    if a.std() < SAMPLE_VARIANCE_GUARD or b.std() < SAMPLE_VARIANCE_GUARD:
        return math.nan
    return float(np.corrcoef(a, b)[0, 1])


def independence_check(detected: np.ndarray, valid: np.ndarray, levels: Sequence[LevelSpec],
                       sigma_multiple: float) -> dict[str, Any]:
    """Step 1 on the outcome table (poses in acquisition order x levels): per level the lag-1 autocorrelation against
    +/- sigma_multiple / sqrt(n), per adjacent pair of levels of one plate the phi coefficient against the same
    band; ``ok`` per the multiple-testing rule of the module docstring."""
    nominal = 2.0 * (1.0 - float(norm.cdf(sigma_multiple)))
    lag_rows, phi_rows = [], []
    for column, level in enumerate(levels):
        rows = np.nonzero(valid[:, column])[0]
        if rows.size < MIN_TRIALS_FOR_LAG:
            continue
        r1 = lag1_autocorrelation(detected[rows, column])
        if math.isnan(r1):
            continue
        bound = sigma_multiple / math.sqrt(rows.size)
        lag_rows.append({"target_id": level.target_id, "site_id": level.site_id, "d_mm": level.diameter_mm,
                         "n": int(rows.size), "lag1": r1, "bound": bound, "flagged": bool(abs(r1) > bound)})
    for column in range(len(levels) - 1):
        a, b = levels[column], levels[column + 1]
        if a.target_id != b.target_id:
            continue
        both = np.nonzero(valid[:, column] & valid[:, column + 1])[0]
        if both.size < MIN_PAIRS_FOR_PHI:
            continue
        phi = phi_coefficient(detected[both, column], detected[both, column + 1])
        if math.isnan(phi):
            continue
        bound = sigma_multiple / math.sqrt(both.size)
        phi_rows.append({"site_a": a.site_id, "site_b": b.site_id, "n": int(both.size), "phi": phi, "bound": bound,
                         "flagged": bool(abs(phi) > bound)})

    def tolerated(tests: list) -> bool:
        allowed = max(ALLOWED_FLAGS_FLOOR, int(math.ceil(nominal * len(tests))))
        return sum(1 for t in tests if t["flagged"]) <= allowed

    return {"lag1": lag_rows, "phi": phi_rows, "nominal_flag_rate": nominal,
            "lag_ok": tolerated(lag_rows), "phi_ok": tolerated(phi_rows),
            "ok": tolerated(lag_rows) and tolerated(phi_rows),
            "tested": len(lag_rows) + len(phi_rows)}


# ---------------------------------------------------------------------------
# Result
# ---------------------------------------------------------------------------
@dataclass
class DetectionResult:
    """Everything Analysis D produced."""

    rows: list[dict[str, Any]]
    """One dict per configuration (kind, gap, station, field) and rule (the SUMMARY_COLUMNS keys); the thresholds are
    those of the configuration's pooled group converted to mm at its station."""
    details: list[dict[str, Any]]
    """Per configuration and rule: the feature table, independence, gamma, the minimums."""
    pooled_rows: list[dict[str, Any]]
    """One dict per (kind, gap, field, rule) group pooled over its stations (the POOLED_COLUMNS keys)."""
    pooled_details: list[dict[str, Any]]
    """Per group: the (feature, station) pairs, the fits, the D_0 tables, the bootstrap and the noise regression."""
    overlap_rows: list[dict[str, Any]]
    """The overlap (scaling) test of every pair of neighboring features of every group (OVERLAP_COLUMNS keys)."""
    configs: list[ConfigTrials]
    notes: list[str] = field(default_factory=list)
    z_reference_mm: float = field(default_factory=lambda: CharacterizationParameters().z_reference_mm)
    gap_small_mm: float = field(default_factory=lambda: CharacterizationParameters().gap_small_mm)

    def forward_model_terms(self) -> dict[str, Any]:
        """Terms for forward_model_parameters.json: ``d50_px`` and ``d10_px`` of the cutouts under the primary rule,
        from the pooled fit of the on-axis group of the small gap (D_px is the governing variable, so one value
        serves every station). Values that could not be estimated are omitted."""
        rows = [r for r in self.pooled_rows if r["kind"] == FEATURE_CUTOUT and r["rule"] == RULE_PRIMARY
                and r["field"] == 0]
        if not rows:
            return {}
        row = min(rows, key=lambda r: abs((r["gap_mm"] or 0.0) - self.gap_small_mm))
        return {key: float(row[key]) for key in ("d50_px", "d10_px") if math.isfinite(row[key])}


# ---------------------------------------------------------------------------
# Columns
# ---------------------------------------------------------------------------
def _minimum_columns() -> list[str]:
    """Names of the threshold columns that are reported in mm, px and mrad."""
    return ["d50", "d50_lower", "d50_upper", "d10", "d10_lower", "d10_upper", "d10_model_min", "d10_model_max",
            "d50_isotonic", "d10_isotonic", "d5", "d5_lower", "d5_upper", "d5_model_min", "d5_model_max",
            "d5_empirical", "d5_empirical_next", "d0_predicted", "d0_predicted_lower", "d0_predicted_upper",
            "d0_predicted_model_min", "d0_predicted_model_max", "d0_model", "d0_model_lower", "d0_model_upper", "d0_geometric",
            "d0_geometric_cameras"]


POOLED_MINIMUMS = tuple(name for name in _minimum_columns() if not name.startswith("d0_geometric"))
"""The minimums that come from the pooled fit (the geometric limits are per station and computed per configuration)."""


def _summary_columns() -> list[str]:
    columns = ["target_id", "kind", "gap_mm", "station_z_mm", "field", "rule", "trials_per_level",
               "trials_per_level_max", "trials_new", "trials_reused", "gamma", "gamma_lower", "gamma_upper",
               "gamma_blank_trials", "tau_mm", "best_curve", "d50_status", "d10_status", "d5_status",
               "d0_model_status", "d0_is_prediction", "d0_predicted_note"]
    for name in _minimum_columns():
        columns += [f"{name}_mm", f"{name}_px", f"{name}_mrad"]
    columns += ["independence_ok", "bootstrap_resamples", "bootstrap_failures", "fit_note"]
    return columns


SUMMARY_COLUMNS = tuple(_summary_columns())
"""Columns of D_detect_summary.csv, one row per configuration: the specification's list (target, gap, Z, gamma of the
station and its interval, tau, best curve, D_50, D_10 with their intervals, the D_10 model range, the isotonic values,
D_5 (fitted, with its bootstrap interval and model range) beside the empirical bracket [d5_empirical,
d5_empirical_next], the PREDICTED D_0 (``d0_predicted``, its bootstrap interval, its model range, ``d0_is_prediction`` always
True and ``d0_predicted_note``), the floor-model D_0 with its interval (also a prediction), the geometric D_0,
independence flag), each minimum in mm, px and mrad, plus status and bookkeeping columns. The minimums are those of the pooled group (D_px, constant over the
stations) converted to mm at the configuration's station."""

POOLED_COLUMNS = (
    ("kind", "gap_mm", "field", "rule", "stations", "pairs", "trials", "gamma", "gamma_lower", "gamma_upper",
     "gamma_blank_trials", "best_curve", "alpha_ln_px", "beta", "lapse_rate", "deviance", "d50_status", "d10_status",
     "d5_status", "d0_model_status", "d0_is_prediction", "d0_predicted_note")
    + tuple(f"{name}_px" for name in POOLED_MINIMUMS)
    + ("bootstrap_resamples", "bootstrap_failures", "regression_status", "regression_observations",
       "regression_b0", "regression_b_ln_dpx", "regression_se_ln_dpx", "regression_b_ln_sigma",
       "regression_se_ln_sigma", "regression_p_noise", "regression_separated", "fit_note"))
"""Columns of D_pooled_summary.csv, one row per (kind, gap, field, rule) group pooled over its stations: the pooled
psychometric fit on ln D_px (D_50, D_10 and D_5 in D_px, with the empirical D_5 bracket [d5_empirical, d5_empirical_next], and the
predicted D_0 flagged by ``d0_is_prediction`` and ``d0_predicted_note``), the group's
false-alarm rate, and the noise-covariate regression (logit p = b0 + b_ln_dpx ln D_px + b_ln_sigma ln sigma_tot(Z))."""


# ---------------------------------------------------------------------------
# The analysis
# ---------------------------------------------------------------------------
@dataclass
class _ConfigRule:
    """The per-configuration, per-rule intermediate (Steps 1, 3 and 8) that the pooled analysis consumes."""

    config: ConfigTrials
    rule: str
    taus: np.ndarray
    """Per-feature threshold tau (mm) of this station."""
    successes: np.ndarray
    trials: np.ndarray
    """Detections and valid trials per feature at this station."""
    gamma: float
    """False-alarm rate of this station (blank-site detections / blank-site trials)."""
    blank_hits: int
    blank_trials: int
    row: dict[str, Any]
    detail: dict[str, Any]
    geometric_mm: tuple[float, float]
    """(D_0 geometric with the projector, cameras only) in mm at this station; NaN for disks."""


def run_detection(session: Session, out_dir: Path, previous: Mapping[str, Any] | None,
                  options: DetectionOptions | None = None) -> DetectionResult | None:
    """Analysis D (Section 13, Steps 1 to 13 above) on the procedure-"D" frames of the session, plus the first frame of
    each C pose of a matching configuration as extra trials (flagged as reused). Returns None when the session has no
    D frames. ``previous`` maps the letters of analyses already run to their results: A's sigma_tot per station is
    the noise covariate. Nothing is written here; see ``write_outputs``."""
    options = DetectionOptions() if options is None else options
    d_records = select(session.records, procedure=PROCEDURE_DETECTION)
    if not d_records:
        return None
    params = session.params
    notes: list[str] = []
    sources: list[tuple[FrameRecord, str]] = [(r, SOURCE_D) for r in first_frames(d_records)]
    wanted = {(r.target_id, r.gap_mm, r.station_z_mm, r.field) for r, _ in sources}
    if options.include_reused:
        c_records = [r for r in select(session.records, procedure=PROCEDURE_AREA)
                     if r.subseries in C_REUSE_SUBSERIES
                     and (r.target_id, r.gap_mm, r.station_z_mm, r.field) in wanted]
        sources += [(r, SOURCE_C) for r in first_frames(c_records)]
    configs = collect_trials(session, sources)
    stereo = StereoGeometry.from_sensor_geometry(session.geometry)
    sigma_table = sigma_tot_by_station(previous)
    if not sigma_table:
        notes.append("Analysis A did not run (or has no main stations): the noise covariate sigma_tot(Z) is not "
                     "available, so the logistic regression on ln sigma_tot is skipped")
    analyzed = [_analyze_config_rule(session, config, rule, stereo, options)
                for config in configs for rule in config.rules()]
    if not analyzed:
        notes.append("no array configuration with a back plate was found among the D frames")
    rows: list[dict[str, Any]] = []
    details: list[dict[str, Any]] = []
    pooled_rows: list[dict[str, Any]] = []
    pooled_details: list[dict[str, Any]] = []
    overlap_rows: list[dict[str, Any]] = []
    groups: dict[tuple, list[_ConfigRule]] = {}
    for member in analyzed:
        key = (member.config.kind, member.config.gap_mm, member.config.field, member.rule)
        groups.setdefault(key, []).append(member)
    for members in groups.values():
        members.sort(key=lambda m: m.config.station_z_mm)
        pooled_row, pooled_detail, overlaps = _analyze_group(session, members, options, sigma_table)
        pooled_rows.append(pooled_row)
        pooled_details.append(pooled_detail)
        overlap_rows += overlaps
        for member in members:
            _fill_config_minimums(session, member, pooled_row)
    for member in analyzed:
        rows.append(member.row)
        details.append(member.detail)
    return DetectionResult(rows=rows, details=details, pooled_rows=pooled_rows, pooled_details=pooled_details,
                           overlap_rows=overlap_rows, configs=configs, notes=notes,
                           z_reference_mm=params.z_reference_mm, gap_small_mm=params.gap_small_mm)


def _units(session: Session, value_mm: float, station_z_mm: float) -> tuple[float, float, float]:
    """(mm, px, mrad) of a size at the station: D_px = D f_x / Z and theta = D / Z in mrad (SensorGeometry helpers)."""
    if value_mm is None or not math.isfinite(value_mm):
        return math.nan, math.nan, math.nan
    geometry = session.geometry
    return (float(value_mm), geometry.diameter_in_pixels(value_mm, station_z_mm),
            geometry.subtended_angle_mrad(value_mm, station_z_mm))


def _bootstrap_count(poses: int, stations: int, options: DetectionOptions, params) -> int:
    """Bootstrap resamples for a group with this many poses over this many stations: the full BOOTSTRAP_RESAMPLES when
    the stations have at least ``full_bootstrap_min_poses`` poses on average, else the reduced count."""
    if options.bootstrap_resamples is not None:
        return options.bootstrap_resamples
    if poses >= options.full_bootstrap_min_poses * max(stations, 1):
        return params.bootstrap_resamples
    return options.reduced_bootstrap_resamples


def _analyze_config_rule(session: Session, config: ConfigTrials, rule: str, stereo: StereoGeometry,
                         options: DetectionOptions) -> _ConfigRule:
    """Steps 1, 3 and 8 for one configuration (one station) and rule: tau per feature, the outcomes, gamma of the
    station from its blank sites, the independence check, the feature table and the geometric limit. The thresholds
    of the summary row are filled in afterward from the pooled group (:func:`_fill_config_minimums`)."""
    params = session.params
    levels_mm = config.level_diameters_mm()
    s_feature, s_blank = config.s_feature[rule], config.s_blank[rule]
    taus = calibrate_tau(s_blank, params.detection_false_alarm_target)                    # Step 3
    feature_detected, blank_detected, feature_valid, blank_valid = outcomes(s_feature, s_blank, taus)
    successes, trials, gamma, blank_hits, blank_trials = counts_from_outcomes(
        feature_detected, blank_detected, feature_valid, blank_valid)
    gamma_lower, gamma_upper = (clopper_pearson(blank_hits, blank_trials, params.confidence_level)
                                if blank_trials else (math.nan, math.nan))
    keep = trials > 0
    independence = independence_check(feature_detected, feature_valid, config.levels,
                                      params.independence_sigma_multiple)                   # Step 1
    row: dict[str, Any] = {
        "target_id": "+".join(config.target_ids), "kind": config.kind, "gap_mm": config.gap_mm,
        "station_z_mm": config.station_z_mm, "field": config.field, "rule": rule,
        "trials_per_level": int(trials[keep].min()) if keep.any() else 0,
        "trials_per_level_max": int(trials.max()) if trials.size else 0,
        "trials_new": sum(1 for s in config.sources if s == SOURCE_D),
        "trials_reused": sum(1 for s in config.sources if s == SOURCE_C),
        "gamma": gamma, "gamma_lower": gamma_lower, "gamma_upper": gamma_upper, "gamma_blank_trials": blank_trials,
        "tau_mm": float(np.nanmedian(taus[np.isfinite(taus)])) if np.isfinite(taus).any() else math.nan,
        "independence_ok": bool(independence["ok"]),
    }
    detail: dict[str, Any] = {
        "target_ids": config.target_ids, "kind": config.kind, "gap_mm": config.gap_mm,
        "station_z_mm": config.station_z_mm, "field": config.field, "rule": rule, "gamma": gamma,
        "gamma_interval": [gamma_lower, gamma_upper], "blank_detections": blank_hits, "blank_trials": blank_trials,
        "independence": independence, "px_per_mm": session.geometry.diameter_in_pixels(1.0, config.station_z_mm),
    }
    raw = np.where(trials > 0, successes / np.maximum(trials, 1.0), math.nan)
    corrected = np.asarray(corrected_rate(np.nan_to_num(raw), gamma))
    level_table = []
    for column, level in enumerate(config.levels):
        n, s = int(trials[column]), int(successes[column])
        lower, upper = clopper_pearson(s, n, params.confidence_level) if n else (math.nan, math.nan)
        level_table.append({
            "target_id": level.target_id, "site_id": level.site_id, "level_index": level.level_index,
            "d_mm": level.diameter_mm, "d_px": session.geometry.diameter_in_pixels(level.diameter_mm,
                                                                                   config.station_z_mm),
            "trials": n, "detections": s, "proportion": float(raw[column]), "ci_lower": lower, "ci_upper": upper,
            "corrected": float(corrected[column]), "tau_mm": float(taus[column])})
    detail["levels"] = level_table

    # Step 8: geometric limit for cutouts (per station, in mm).
    d0_geo = d0_geo_cameras = math.nan
    if config.kind == FEATURE_CUTOUT:
        plate = session.targets.get(config.target_ids[0], config.gap_mm)
        top = options.max_geometric_scale * float(levels_mm.max())
        d0_geo = geometric_limit_diameter_mm(plate, config.station_z_mm, stereo, True, top, options.area_options)
        d0_geo_cameras = geometric_limit_diameter_mm(plate, config.station_z_mm, stereo, False, top,
                                                     options.area_options)
    return _ConfigRule(config=config, rule=rule, taus=taus, successes=successes, trials=trials, gamma=gamma,
                       blank_hits=blank_hits, blank_trials=blank_trials, row=row, detail=detail,
                       geometric_mm=(d0_geo, d0_geo_cameras))


# ---------------------------------------------------------------------------
# Pooling over the (feature, station) pairs of a group (Steps 4 to 7, 9, 12, 13)
# ---------------------------------------------------------------------------
def _cluster_levels(d_px: np.ndarray) -> np.ndarray:
    """Cluster id (0, 1, ... in increasing D_px) of every entry: entries whose D_px lie within MERGE_D_PX_TOLERANCE
    (relative) of the previous entry in sorted order are one level of the pooled curve."""
    order = np.argsort(d_px, kind="stable")
    ids = np.zeros(d_px.size, dtype=int)
    cluster, previous = -1, math.nan
    for index in order:
        if not d_px[index] <= previous * (1.0 + MERGE_D_PX_TOLERANCE):          # also true for the first (NaN)
            cluster += 1
        ids[index] = cluster
        previous = d_px[index]
    return ids


def _noise_regression(pairs: list[dict[str, Any]], sigma_table: Mapping[float, float]) -> dict[str, Any]:
    """Step 13: logit p = b0 + b1 ln D_px + b2 ln sigma_tot(Z) on the pooled (feature, station) counts, against the
    reduced model without the noise term (likelihood-ratio test of b2). ``status`` says why nothing was fitted."""
    result: dict[str, Any] = {"status": "ok"}
    sigma = [sigma_tot_at(sigma_table, p["station_z_mm"]) for p in pairs]
    if not sigma_table or any(v is None or v <= 0.0 for v in sigma):
        result["status"] = "no sigma_tot(Z) from Analysis A for every station"
        return result
    x = np.column_stack([np.log([p["d_px"] for p in pairs]), np.log(sigma)])
    s = np.array([p["successes"] for p in pairs], dtype=float)
    n = np.array([p["trials"] for p in pairs], dtype=float)
    if np.linalg.matrix_rank(x - x.mean(axis=0), tol=MIN_REGRESSION_SPREAD) < LOG_COLUMNS_REGRESSION:
        result["status"] = "ln D_px and ln sigma_tot are collinear (one feature or one station): not separable"
        return result
    full = fit_logistic(x, s, n)
    reduced = fit_logistic(x[:, :1], s, n)
    errors = full.standard_errors
    result.update(observations=full.observations, b0=float(full.coefficients[0]),
                  b_ln_dpx=float(full.coefficients[1]), se_ln_dpx=float(errors[1]),
                  b_ln_sigma=float(full.coefficients[2]), se_ln_sigma=float(errors[2]),
                  p_noise=likelihood_ratio_pvalue(full, reduced), deviance_full=full.deviance,
                  deviance_reduced=reduced.deviance, converged=full.converged, separated=full.separated,
                  reduced_b_ln_dpx=float(reduced.coefficients[1]))
    if full.separated:
        result["status"] = "the data separate detections from misses (a step): the slopes are not meaningful"
    return result


def _analyze_group(session: Session, members: list[_ConfigRule], options: DetectionOptions,
                   sigma_table: Mapping[float, float]) -> tuple[dict[str, Any], dict[str, Any], list[dict[str, Any]]]:
    """Steps 4 to 7, 9, 12 and 13 for one (kind, gap, field, rule) group: the stations (``members``, sorted by
    Z) pooled into one psychometric curve on ln D_px. Returns the pooled summary row, the detail record and the
    overlap-test rows."""
    params = session.params
    fx = float(session.geometry.require("sensor_fx_px"))
    first = members[0]
    kind, gap, field_code, rule = first.config.kind, first.config.gap_mm, first.config.field, first.rule
    # The (feature, station) pairs with trials: the detection levels.
    pairs: list[dict[str, Any]] = []
    for member_index, member in enumerate(members):
        for column, level in enumerate(member.config.levels):
            n = int(member.trials[column])
            if n == 0:
                continue
            station = member.config.station_z_mm
            pairs.append({"member": member_index, "column": column, "station_z_mm": station,
                          "target_id": level.target_id, "site_id": level.site_id, "level_index": level.level_index,
                          "d_mm": level.diameter_mm, "d_px": level.diameter_mm * fx / station,
                          "successes": int(member.successes[column]), "trials": n, "gamma": member.gamma})
    blank_hits = sum(m.blank_hits for m in members)
    blank_trials = sum(m.blank_trials for m in members)
    gamma = blank_hits / blank_trials if blank_trials else 0.0
    gamma_lower, gamma_upper = (clopper_pearson(blank_hits, blank_trials, params.confidence_level)
                                if blank_trials else (math.nan, math.nan))
    total_poses = sum(len(m.config.pose_keys) for m in members)
    row: dict[str, Any] = {
        "kind": kind, "gap_mm": gap, "field": field_code, "rule": rule,
        "stations": ",".join(f"{m.config.station_z_mm:g}" for m in members), "pairs": len(pairs),
        "trials": sum(p["trials"] for p in pairs), "gamma": gamma, "gamma_lower": gamma_lower,
        "gamma_upper": gamma_upper, "gamma_blank_trials": blank_trials, "best_curve": "", "alpha_ln_px": math.nan,
        "beta": math.nan, "lapse_rate": math.nan, "deviance": math.nan, "d50_status": "too few trials",
        "d10_status": "too few trials", "d5_status": "too few trials", "d0_model_status": "too few trials", "d0_is_prediction": True,
        "d0_predicted_note": D0_NOTE_NO_FIT, "bootstrap_resamples": 0,
        "bootstrap_failures": 0, "regression_status": "not attempted", "fit_note": ""}
    for name in POOLED_MINIMUMS:
        row[f"{name}_px"] = math.nan
    detail: dict[str, Any] = {
        "kind": kind, "gap_mm": gap, "field": field_code, "rule": rule, "gamma": gamma,
        "gamma_interval": [gamma_lower, gamma_upper], "blank_detections": blank_hits, "blank_trials": blank_trials,
        "pairs": [{k: v for k, v in p.items() if k not in ("member", "column")} for p in pairs],
        "mm_per_px_at_station": {f"{m.config.station_z_mm:g}": m.config.station_z_mm / fx for m in members}}

    # Pooled levels: pairs whose D_px coincide are one level (module docstring, Step 4).
    d_px = np.array([p["d_px"] for p in pairs])
    successes = np.array([p["successes"] for p in pairs], dtype=float)
    trials = np.array([p["trials"] for p in pairs], dtype=float)
    well_sampled = int(np.sum(trials >= options.min_trials_per_level))
    clusters = _cluster_levels(d_px) if pairs else np.zeros(0, dtype=int)
    n_levels = int(clusters.max()) + 1 if pairs else 0
    level_px = np.array([math.exp(float(np.average(np.log(d_px[clusters == c]), weights=trials[clusters == c])))
                         for c in range(n_levels)])
    merged_successes = np.bincount(clusters, weights=successes, minlength=n_levels)
    merged_trials = np.bincount(clusters, weights=trials, minlength=n_levels)
    enough = well_sampled >= options.min_pairs_for_fit and n_levels >= MIN_DISTINCT_LEVELS_FOR_FIT

    model = None
    best_name = ""
    d5 = {"d5_empirical": math.nan, "d5_empirical_next": math.nan, "levels": []}
    if enough:
        raw = merged_successes / merged_trials
        corrected = np.asarray(corrected_rate(raw, gamma))
        # Steps 4 and 5.
        with _quiet():
            fits = fit_all_curves(level_px, merged_successes, merged_trials, gamma, _fit_parameters(params))
        best = best_fit(fits)
        best_name = best.curve
        row.update(best_curve=best_name, alpha_ln_px=best.alpha, beta=best.beta, lapse_rate=best.lapse_rate,
                   deviance=best.deviance, d50_status=threshold_status(corrected, P_STAR_D50),
                   d10_status=threshold_status(corrected, P_STAR_D10),
                   d5_status=threshold_status(corrected, params.detection_low_probability))
        if row["d50_status"] == "ok":
            row["d50_px"] = best.threshold(P_STAR_D50)
        if row["d10_status"] == "ok":
            row["d10_px"] = best.threshold(P_STAR_D10)
            tails = [f.threshold(P_STAR_D10) for f in fits.values()]
            row["d10_model_min_px"], row["d10_model_max_px"] = float(min(tails)), float(max(tails))
        if row["d5_status"] == "ok":                      # Step 5b: D_5 from the fitted curves
            row["d5_px"] = best.threshold(params.detection_low_probability)
            lows = [f.threshold(params.detection_low_probability) for f in fits.values()]
            row["d5_model_min_px"], row["d5_model_max_px"] = float(min(lows)), float(max(lows))
        row["d50_isotonic_px"] = isotonic_threshold(level_px, corrected, P_STAR_D50, merged_trials)
        row["d10_isotonic_px"] = isotonic_threshold(level_px, corrected, P_STAR_D10, merged_trials)
        detail["fits"] = {name: {"alpha_ln_px": f.alpha, "beta": f.beta, "lapse_rate": f.lapse_rate,
                                 "deviance": f.deviance, "converged": f.converged, "d50_px": f.threshold(P_STAR_D50),
                                 "d10_px": f.threshold(P_STAR_D10),
                                 "d5_px": f.threshold(params.detection_low_probability),
                                 "d0_predicted_px": f.threshold(params.detection_zero_prediction_level)}
                               for name, f in fits.items()}
        detail["curve_px"] = _curve_points(level_px, best)
        detail["levels_px"] = [{"d_px": float(x), "trials": int(n), "detections": int(k), "proportion": float(p),
                                "corrected": float(c)}
                               for x, n, k, p, c in zip(level_px, merged_trials, merged_successes, raw, corrected)]
        # Step 6: the empirical low point (corrected bound) as a D_px bracket, beside the fitted D_5.
        d5 = empirical_d5(level_px, merged_successes, merged_trials, gamma, params.confidence_level,
                          params.detection_low_probability)
        row["d5_empirical_px"], row["d5_empirical_next_px"] = d5["d5_empirical"], d5["d5_empirical_next"]
        # Step 7: the predicted D_0 (the best curve extrapolated below D_5) and the floor model.
        prediction = predict_d0(fits, best, params.detection_zero_prediction_level, params.detection_low_probability)
        row.update(d0_predicted_px=prediction["d0"], d0_predicted_model_min_px=prediction["model_min"],
                   d0_predicted_model_max_px=prediction["model_max"], d0_is_prediction=True,
                   d0_predicted_note=prediction["note"])
        detail["d0_predicted"] = _prediction_detail(prediction, best, level_px)
        try:
            with _quiet():
                model = fit_threshold_model(level_px, corrected, merged_trials, params.confidence_level)
            row["d0_model_status"] = "ok"
            row["d0_model_px"], row["d0_model_lower_px"], row["d0_model_upper_px"] = (
                model.d0, model.d0_lower, model.d0_upper)
            detail["threshold_model"] = {"d0_px": model.d0, "scale_px": model.scale, "shape": model.shape,
                                         "d0_lower_px": model.d0_lower, "d0_upper_px": model.d0_upper,
                                         "deviance": model.deviance, "converged": model.converged,
                                         "profile_d0_px": model.profile_d0,
                                         "profile_statistic": model.profile_statistic}
        except ValueError as error:                      # every effective count is zero
            row["d0_model_status"] = "not estimable"
            detail["threshold_model"] = {"not_estimable": str(error)}
        _bootstrap_group(row, detail, members, level_px, clusters, pairs, best, params, options, total_poses)
    else:
        row["fit_note"] = (f"too few trials: {well_sampled} (feature, station) pairs with at least "
                           f"{options.min_trials_per_level} trials over {n_levels} distinct D_px "
                           f"(need {options.min_pairs_for_fit} and {MIN_DISTINCT_LEVELS_FOR_FIT}); "
                           "curve fits not attempted")
    detail["d5_empirical"] = d5
    if not enough:
        detail["d0_predicted"] = {"d_px": math.nan, "is_prediction": True, "note": D0_NOTE_NO_FIT}

    # Step 13: the noise covariate.
    if pairs and enough:
        regression = _noise_regression(pairs, sigma_table)
    else:
        regression = {"status": "not attempted (too few trials)"}
    detail["noise_regression"] = regression
    row["regression_status"] = regression["status"]
    for key, name in (("observations", "regression_observations"), ("b0", "regression_b0"),
                      ("b_ln_dpx", "regression_b_ln_dpx"), ("se_ln_dpx", "regression_se_ln_dpx"),
                      ("b_ln_sigma", "regression_b_ln_sigma"), ("se_ln_sigma", "regression_se_ln_sigma"),
                      ("p_noise", "regression_p_noise"), ("separated", "regression_separated")):
        row[name] = regression.get(key)

    # Step 12: the overlap (scaling) test between neighboring features.
    overlaps: list[dict[str, Any]] = []
    curves: dict[tuple, dict[str, list]] = {}
    for p in pairs:
        gamma_station = p["gamma"]
        value = float(corrected_rate(p["successes"] / p["trials"], gamma_station))
        error = float(binomial_standard_error(p["successes"], p["trials"])) / (1.0 - gamma_station)
        entry = curves.setdefault((p["target_id"], p["site_id"]), {"d": [], "v": [], "e": [], "z": []})
        entry["d"].append(p["d_px"])
        entry["v"].append(value)
        entry["e"].append(error)
        entry["z"].append(p["station_z_mm"])
    transfer = [TransferCurve(label=f"{target}:{site}", d_px=np.array(c["d"]), value=np.array(c["v"]),
                              std_error=np.array(c["e"]), station_z_mm=np.array(c["z"]))
                for (target, site), c in curves.items()]
    for result in overlap_tests(transfer, params.confidence_level, params.bootstrap_resamples,
                                options.overlap_options, sigma_table):
        overlaps.append({"kind": kind, "gap_mm": gap, "field": field_code, "rule": rule, **result.as_row()})
    detail["overlap_tests"] = overlaps
    return row, detail, overlaps


def _prediction_detail(prediction: dict[str, Any], best: PsychometricFit, level_px: np.ndarray) -> dict[str, Any]:
    """The record of the predicted D_0 for the details file and the figures: the prediction, its corrected-probability
    level, the raw probability the best curve has there, and the dashed extrapolated segment of the curve from the
    predicted D_0 up to the smallest measured level (empty when the prediction is not below it)."""
    detail: dict[str, Any] = {"d_px": prediction["d0"], "model_min_px": prediction["model_min"],
                              "model_max_px": prediction["model_max"], "is_prediction": True,
                              "corrected_probability_level": prediction["level"], "note": prediction["note"],
                              "psi": math.nan, "extrapolation_px": [], "extrapolation_psi": []}
    lowest = float(level_px.min())
    if math.isfinite(prediction["d0"]):
        detail["psi"] = float(best.probability(np.array([prediction["d0"]]))[0])
        if prediction["d0"] < lowest:
            grid = np.geomspace(prediction["d0"], lowest, PSYCHOMETRIC_CURVE_POINTS)
            detail["extrapolation_px"], detail["extrapolation_psi"] = grid, best.probability(grid)
    return detail


def _bootstrap_group(row: dict[str, Any], detail: dict[str, Any], members: list[_ConfigRule], level_px: np.ndarray,
                     clusters: np.ndarray, pairs: list[dict[str, Any]], best: PsychometricFit, params,
                     options: DetectionOptions, total_poses: int) -> None:
    """Step 9: the stratified bootstrap over poses of the pooled D_50, D_10, D_5 and predicted D_0 (in D_px), filling the interval columns
    of ``row`` and the bootstrap record of ``detail``."""
    lookup = {(p["member"], p["column"]): int(clusters[index]) for index, p in enumerate(pairs)}
    groups = [(m, i) for m, member in enumerate(members) for i in range(len(member.config.pose_keys))]
    strata = [m for m, _ in groups]
    resamples = _bootstrap_count(total_poses, len(members), options, params)
    statistic = _make_bootstrap_statistic(
        members, level_px, lookup, params, best,
        wanted={BOOTSTRAP_SLOT_D50: row["d50_status"] == "ok", BOOTSTRAP_SLOT_D10: row["d10_status"] == "ok",
                BOOTSTRAP_SLOT_D5: row["d5_status"] == "ok",
                BOOTSTRAP_SLOT_D0: math.isfinite(row["d0_predicted_px"])})
    rng = np.random.default_rng(options.bootstrap_seed)
    started = time.time()
    try:
        boot = bootstrap_statistic(groups, statistic, resamples, params.confidence_level, rng, strata=strata)
    except ValueError as error:
        row["fit_note"] = f"bootstrap failed: {error}"
        return
    lower, upper = np.asarray(boot.lower, dtype=float), np.asarray(boot.upper, dtype=float)
    if row["d50_status"] == "ok":
        row["d50_lower_px"], row["d50_upper_px"] = float(lower[BOOTSTRAP_SLOT_D50]), float(upper[BOOTSTRAP_SLOT_D50])
    if row["d10_status"] == "ok":
        row["d10_lower_px"], row["d10_upper_px"] = float(lower[BOOTSTRAP_SLOT_D10]), float(upper[BOOTSTRAP_SLOT_D10])
    if row["d5_status"] == "ok":
        row["d5_lower_px"], row["d5_upper_px"] = float(lower[BOOTSTRAP_SLOT_D5]), float(upper[BOOTSTRAP_SLOT_D5])
    if math.isfinite(row["d0_predicted_px"]):             # an interval of a PREDICTION (extrapolated curve)
        row["d0_predicted_lower_px"] = float(lower[BOOTSTRAP_SLOT_D0])
        row["d0_predicted_upper_px"] = float(upper[BOOTSTRAP_SLOT_D0])
    row["bootstrap_resamples"], row["bootstrap_failures"] = resamples, boot.failures
    detail["bootstrap"] = {
        "resamples": resamples, "failures": boot.failures, "seconds": time.time() - started,
        "reduced": options.bootstrap_resamples is None and resamples != params.bootstrap_resamples,
        "poses": total_poses, "stratified_by": "station", "names": list(BOOTSTRAP_STAT_NAMES),
        "lower": [float(v) for v in lower], "upper": [float(v) for v in upper],
        "estimate": [float(v) for v in np.asarray(boot.estimate)]}


def _fill_config_minimums(session: Session, member: _ConfigRule, pooled_row: dict[str, Any]) -> None:
    """Fill the threshold columns of a configuration's summary row from its pooled group: the pooled D_px values
    converted to mm at the station (D_mm = D_px Z / f_x), plus the per-station geometric limit, and the statuses."""
    row, detail = member.row, member.detail
    station = member.config.station_z_mm
    fx = float(session.geometry.require("sensor_fx_px"))
    row.update(best_curve=pooled_row["best_curve"], d50_status=pooled_row["d50_status"],
               d10_status=pooled_row["d10_status"], d5_status=pooled_row["d5_status"], d0_model_status=pooled_row["d0_model_status"],
               d0_is_prediction=True, d0_predicted_note=pooled_row["d0_predicted_note"],
               bootstrap_resamples=pooled_row["bootstrap_resamples"],
               bootstrap_failures=pooled_row["bootstrap_failures"], fit_note=pooled_row["fit_note"])
    values_mm = {name: (pooled_row[f"{name}_px"] * station / fx if math.isfinite(pooled_row[f"{name}_px"])
                        else math.nan) for name in POOLED_MINIMUMS}
    values_mm["d0_geometric"], values_mm["d0_geometric_cameras"] = member.geometric_mm
    for name, value in values_mm.items():
        mm, px, mrad = _units(session, value, station)
        row[f"{name}_mm"], row[f"{name}_px"], row[f"{name}_mrad"] = mm, px, mrad
    detail["minimums"] = {name: {"mm": row[f"{name}_mm"], "px": row[f"{name}_px"], "mrad": row[f"{name}_mrad"]}
                          for name in values_mm}
    detail["pooled_group"] = {"kind": pooled_row["kind"], "gap_mm": pooled_row["gap_mm"],
                              "field": pooled_row["field"], "rule": pooled_row["rule"]}


def _curve_points(levels_px: np.ndarray, fit: PsychometricFit) -> dict[str, Any]:
    """The fitted raw-probability curve sampled over the tested range (for the figures and the details)."""
    grid = np.geomspace(float(levels_px.min()), float(levels_px.max()), PSYCHOMETRIC_CURVE_POINTS)
    return {"d_px": grid, "psi": fit.probability(grid), "curve": fit.curve}


def _make_bootstrap_statistic(members: list[_ConfigRule], level_px: np.ndarray, lookup: dict[tuple[int, int], int],
                              params, best: PsychometricFit, wanted: Mapping[int, bool]):
    """The bootstrap statistic of Step 9: given the drawn (station, pose) groups, recompute per station the per-feature
    tau from the drawn blank statistics and the detections, pool them into the levels of the curve, take gamma from
    the pooled blank sites, refit the (best) curve family and return (D_50, D_10, D_5, predicted D_0, gamma, median
    tau) (BOOTSTRAP_STAT_NAMES), the D values in D_px. A resample whose proportions do not bracket a threshold that
    the original data bracket gives NaN (counted as a failure by the bootstrap), as does a refit that did not
    converge for the predicted D_0. A quantity the original data do not give (``wanted`` is False for its slot) is not
    resampled (its slot is NOT_RESAMPLED and its interval is reported as NaN by the caller). The refit starts from the original optimum ``best`` (one optimizer
    run per resample)."""
    target = params.detection_false_alarm_target
    rule = members[0].rule
    warm_start = (best.alpha, best.beta, best.lapse_rate)

    def statistic(drawn: Sequence[tuple[int, int]]) -> np.ndarray:
        per_member: list[list[int]] = [[] for _ in members]
        for member_index, pose_index in drawn:
            per_member[member_index].append(pose_index)
        successes = np.zeros(level_px.size)
        trials = np.zeros(level_px.size)
        hits = blanks = 0
        finite_taus: list[float] = []
        for member_index, indices in enumerate(per_member):
            if not indices:
                continue
            config = members[member_index].config
            index = np.asarray(indices, dtype=int)
            sf, sb = config.s_feature[rule][index], config.s_blank[rule][index]
            taus = calibrate_tau(sb, target)
            fd, bd, fv, bv = outcomes(sf, sb, taus)
            level_successes, level_trials, _, h, b = counts_from_outcomes(fd, bd, fv, bv)
            hits += h
            blanks += b
            finite_taus += [float(t) for t in taus[np.isfinite(taus)]]
            for column in range(len(config.levels)):
                cluster = lookup.get((member_index, column))
                if cluster is not None:
                    successes[cluster] += level_successes[column]
                    trials[cluster] += level_trials[column]
        gamma = hits / blanks if blanks else 0.0
        keep = trials > 0
        with _quiet():
            values = _fit_thresholds(level_px[keep], successes[keep], trials[keep], gamma, params, best.curve,
                                     warm_start)
        resampled = [value if wanted[slot] else NOT_RESAMPLED for slot, value in enumerate(values)]
        return np.array(resampled + [gamma, float(np.median(finite_taus)) if finite_taus else math.nan])

    return statistic


# ---------------------------------------------------------------------------
# Outputs (Step 11)
# ---------------------------------------------------------------------------
def write_outputs(result: DetectionResult, out_dir: Path) -> list[Path]:
    """Write D_detect_summary.csv, D_pooled_summary.csv, D_overlap_test.csv, D_detect_details.json and the figures
    into ``out_dir``; returns the paths."""
    out_dir = Path(out_dir)
    written = [write_csv_rows(out_dir / SUMMARY_FILE_NAME, result.rows, SUMMARY_COLUMNS),
               write_csv_rows(out_dir / POOLED_FILE_NAME, result.pooled_rows, POOLED_COLUMNS),
               write_csv_rows(out_dir / OVERLAP_FILE_NAME, result.overlap_rows, OVERLAP_COLUMNS)]
    document = {"notes": result.notes, "configurations": result.details, "pooled": result.pooled_details}
    written.append(write_json(out_dir / DETAILS_FILE_NAME, document))
    written += _figures_psychometric(result, out_dir)
    written += _figure_minimum_vs_z(result, out_dir)
    return written


def _figures_psychometric(result: DetectionResult, out_dir: Path) -> list[Path]:
    """Step 11: per (kind, gap, field) the pooled detection fractions of all (feature, station) pairs with
    Clopper-Pearson error bars against D_px (one color per feature, one marker per station is too many: the stations
    are the points of a feature's curve), the fitted curve of each rule, and the minimums marked (D_50 solid, D_10
    dashed, the fitted D_5 dotted, the floor-model D_0 dash-dot). The predicted D_0 is drawn as a prediction, not as a
    measurement: a hollow marker at the fitted curve's value there, on a dashed extrapolation of the curve below the
    smallest measured level, labeled "predicted"."""
    written: list[Path] = []
    keys = sorted({(d["kind"], d["gap_mm"], d["field"]) for d in result.pooled_details},
                  key=lambda k: (k[0], k[1] or 0.0, k[2]))
    rule_colors = {RULE_PRIMARY: OKABE_ITO_BLUE, RULE_INCLUSIVE: OKABE_ITO_VERMILLION}
    confidence = CharacterizationParameters().confidence_level
    for kind, gap, field_code in keys:
        members = [(d, r) for d, r in zip(result.pooled_details, result.pooled_rows)
                   if (d["kind"], d["gap_mm"], d["field"]) == (kind, gap, field_code)]
        figure, axis = new_figure(6.8, 4.6)
        for detail, row in members:
            rule = detail["rule"]
            color = rule_colors[rule]
            shift = RULE_PLOT_SHIFT if rule == RULE_INCLUSIVE else 1.0     # keeps coincident points visible
            for level in sorted({p["level_index"] for p in detail["pairs"]}):
                table = [p for p in detail["pairs"] if p["level_index"] == level]
                x = np.array([p["d_px"] for p in table])
                k = np.array([p["successes"] for p in table])
                n = np.array([p["trials"] for p in table])
                p_hat = k / n
                bounds = np.array([clopper_pearson(int(a), int(b), confidence) for a, b in zip(k, n)])
                marker = FEATURE_MARKERS[level % len(FEATURE_MARKERS)]
                axis.errorbar(x * shift, p_hat, yerr=[np.maximum(p_hat - bounds[:, 0], 0.0),
                                                      np.maximum(bounds[:, 1] - p_hat, 0.0)],
                              fmt=marker, color=color, capsize=2, markersize=4,
                              label=f"{rule.replace('_', ' ')} rule, feature {level}")
            curve = detail.get("curve_px")
            if curve is not None:
                axis.plot(curve["d_px"], curve["psi"], color=color, linewidth=1.2)
            for name, style in (("d50", "-"), ("d10", "--"), ("d5", ":"), ("d0_model", "-.")):
                value = row[f"{name}_px"]
                if math.isfinite(value) and value > 0.0:
                    axis.axvline(value, color=color, linestyle=style, linewidth=0.9)
            # The predicted D_0: dashed extrapolation of the fitted curve and a hollow marker, never a solid point.
            predicted = detail.get("d0_predicted", {})
            if len(predicted.get("extrapolation_px", [])):
                axis.plot(predicted["extrapolation_px"], predicted["extrapolation_psi"], color=color,
                          linestyle=PREDICTION_LINE_STYLE, linewidth=1.2)
            if math.isfinite(predicted.get("d_px", math.nan)) and math.isfinite(predicted.get("psi", math.nan)):
                axis.plot([predicted["d_px"] * shift], [predicted["psi"]], marker=PREDICTION_MARKER, markerfacecolor="none",
                          markeredgecolor=color, markersize=PREDICTION_MARKER_SIZE, linestyle="none",
                          label=f"{rule.replace('_', ' ')} rule, D_0 predicted (extrapolation)")
        axis.axhline(0.0, color=OKABE_ITO_BLACK, linewidth=0.4)
        log_axis(axis)
        axis.set_ylim(-0.05, 1.05)
        axis.set_xlabel("feature diameter D_px = D f_x / Z (px), all stations pooled")
        axis.set_ylabel("detection fraction (error bars: 95% Clopper-Pearson)")
        axis.set_title(f"{kind}s, G = {gap:g} mm" + (f", field {field_code}" if field_code else "")
                       + "\nsolid D_50, dashed D_10, dotted D_5, dash-dot D_0 floor model; hollow marker "
                       "and dashed curve: D_0 predicted (extrapolation)", fontsize=8)
        axis.legend(fontsize=6, ncol=2)
        axis.grid(True, linewidth=0.3, which="both")
        written += save_figure(figure, out_dir / f"D_psychometric_{kind}_G{gap:g}_F{field_code}")
    return written


PREDICTION_MARKER = "o"
"""Marker of a predicted D_0 in the figures: always drawn hollow, so that it cannot be read as a measured point."""
PREDICTION_MARKER_SIZE = 8.0
"""Size (points) of the hollow marker of a predicted D_0."""
PREDICTION_LINE_STYLE = "--"
"""Line style of the extrapolated segment of the fitted curve (below the lowest measured point) and of the predicted D_0
line in the minimum-versus-Z figure."""
FEATURE_MARKERS = ("o", "s", "^", "D", "v")
"""Markers cycled over the features of a plate in the pooled psychometric figure."""


def _figure_minimum_vs_z(result: DetectionResult, out_dir: Path) -> list[Path]:
    """Step 10: the minimum diameter in mm (D_50, D_10, the fitted D_5, the lower end of its empirical bracket and the
    PREDICTED D_0) against Z, one panel per kind and rule, one line style per gap, with the feature diameters of the plate
    as faint horizontal lines and, for cutouts, the geometric limit. The predicted D_0 has a hollow marker and a dashed
    line and is labeled "predicted". A pooled minimum is constant in D_px, so in mm it rises in proportion to Z."""
    panels = sorted({(r["kind"], r["rule"]) for r in result.rows})
    if not panels:
        return []
    figure, axes = new_figure(5.0 * len(panels), 4.2)
    figure.clf()
    axes = figure.subplots(1, len(panels), squeeze=False, sharey=True)[0]
    styles = (("d50", "o", OKABE_ITO_BLUE, "50%"), ("d10", "s", OKABE_ITO_ORANGE, "10%"),
              ("d5", "^", OKABE_ITO_VERMILLION, "5%"),
              ("d5_empirical", "x", OKABE_ITO_VERMILLION, "5% (empirical bracket)"),
              ("d0_predicted", "v", OKABE_ITO_BLACK, "0% (predicted)"))
    for axis, (kind, rule) in zip(axes, panels):
        rows = [r for r in result.rows if r["kind"] == kind and r["rule"] == rule and r["field"] == 0]
        gaps = sorted({r["gap_mm"] for r in rows})
        for gap in gaps:
            gap_rows = sorted((r for r in rows if r["gap_mm"] == gap), key=lambda r: r["station_z_mm"])
            linestyle = "-" if gap == gaps[0] else ":"
            for name, marker, color, label in styles:
                points = [(r["station_z_mm"], r[f"{name}_mm"]) for r in gap_rows if math.isfinite(r[f"{name}_mm"])]
                if points and name == "d0_predicted":
                    # A prediction: hollow marker, dashed line.
                    axis.plot(*zip(*points), marker=marker, color=color, linestyle=PREDICTION_LINE_STYLE,
                              markerfacecolor="none", label=f"{label}, G = {gap:g} mm")
                elif points:
                    axis.plot(*zip(*points), marker=marker, color=color,
                              linestyle=":" if name == "d5_empirical" else linestyle,
                              label=f"{label}, G = {gap:g} mm")
        geometric = [(r["station_z_mm"], r["d0_geometric_mm"]) for r in sorted(
            rows, key=lambda r: r["station_z_mm"]) if math.isfinite(r["d0_geometric_mm"]) and r["gap_mm"] == gaps[0]]
        if geometric and rule == RULE_PRIMARY:
            axis.plot(*zip(*geometric), color=OKABE_ITO_BLACK, linestyle=(0, (6, 3)), linewidth=0.8,
                      label="geometric limit")
        for config in result.configs:
            if config.kind == kind:
                for diameter in config.level_diameters_mm():
                    axis.axhline(diameter, color=OKABE_ITO_BLACK, linewidth=0.3, alpha=0.4)
                break
        log_axis(axis, "x")
        log_axis(axis, "y")
        axis.set_xlabel("station Z (mm)")
        axis.set_title(f"{kind}s, {rule.replace('_', ' ')} rule (faint lines: feature diameters)", fontsize=8)
        axis.grid(True, linewidth=0.3, which="both")
        axis.legend(fontsize=6)
    axes[0].set_ylabel("minimum detectable diameter (mm)")
    figure.tight_layout()
    return save_figure(figure, out_dir / "D_minimum_vs_z")
