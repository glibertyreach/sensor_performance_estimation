"""
Analysis D: minimum detectable size at 50, 10 and 0 percent (procedure document, Section 13), Steps 1 to 11.

What it computes
    For every configuration (target kind disk or cutout, gap, station, field position) the detection probability
    of each diameter level of the plate(s), from single-frame trials: one trial is the FIRST frame of a pose (the
    frames of one pose are never separate trials, Section 8 Step 5). A frame yields one trial per feature and one
    per blank site. The first frame of each C pose of a matching configuration (same target, gap, station and
    field) is added as an extra trial and flagged as reused (Section 8, Step 6; the CSV counts new and reused
    trials). From the trials: false-alarm-calibrated thresholds, the psychometric fit, D_50, D_10, D_0 (three
    ways) and their units (mm, px, mrad).

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
       candidates for every tau. Both rules are always evaluated.
       IMPLEMENTATION: instead of re-labeling the window for every trial threshold, each window is reduced ONCE to
       its detection statistic S, the largest tau at which the rule still detects (the "k-th connected level"
       of the window: the greatest value u such that the pixels with Delta >= u contain a connected region of at
       least k pixels; k = 2 is the best adjacent pair, max over pairs of min(Delta_i, Delta_j)). The window is
       detected at threshold tau exactly when S > tau (verified against the labeling in the tests).
    3  Threshold. tau is set PER WINDOW SIZE (per level) from the blank sites of that level over all trials of the
       configuration: tau is the (1 - DETECTION_FALSE_ALARM_TARGET) quantile of the blank windows' S ("higher"
       interpolation, so the false-alarm fraction does not exceed the target). That makes the false-alarm fraction
       of the RULE (window level, after the connected-pixel criterion), not of single pixels, equal the target.
       The measured false-alarm rate gamma is the fraction of all blank-site trials of the configuration that the
       rule then declares detected (with fewer than 1 / target trials per level the quantile is the maximum and
       gamma is 0 of n; the Clopper-Pearson interval is reported either way). The reported tau_mm is the median
       over levels (the per-level values are in the details).
    1  Independence: lag-1 autocorrelation of each feature's detected/missed sequence in acquisition order
       against +/- INDEPENDENCE_SIGMA_MULTIPLE / sqrt(n), and the phi coefficient between the outcomes of adjacent
       levels in the same frame against the same band. With many features some exceed 2 / sqrt(n) by chance, so
       ``independence_ok`` is False only when MORE than max(1, ceil(p x tested)) features (or pairs) are outside,
       p = 2 (1 - Phi(multiplier)); the flagged ones are listed in the details. Features whose sequences do not
       vary (always or never detected) carry no information and are not tested.
    4  Psychometric fit with gamma fixed (stats.psychometric.fit_all_curves): the raw detections per level against
       the level's diameter in mm. D_50 and D_10 come from the best curve by deviance. A threshold is reported only
       when the data bracket it (the corrected proportions reach it and start below it); otherwise it is NaN and
       the status column says why.
    5  Model range of D_10 over the logistic, normal and Weibull fits; model-free isotonic crossings (0.1, 0.5) of
       the corrected proportions.
    6  D_0 empirical: per level the one-sided Clopper-Pearson upper bound psi_U at CONFIDENCE_LEVEL, corrected
       P*_U = (psi_U - gamma) / (1 - gamma); D_0,emp is the largest level such that P*_U <=
       DETECTION_ZERO_PROBABILITY_BOUND at that level and at every smaller one; NaN when even the smallest level
       fails (not demonstrated). The bracket is [D_0,emp, next level up].
    7  D_0 threshold model (stats.psychometric.fit_threshold_model) with its profile-likelihood interval; "not
       estimable" when every level detects nothing (its ValueError is caught and recorded).
    8  Geometric limit (cutouts): the diameter at which A_geo of Analysis C, Step 7 reaches zero, by bisection on a
       probe cutout at the plate center (area.geometric_limit_diameter_mm), cameras only and with the projector.
    9  Bootstrap over poses (trials): each resample draws poses with replacement from the per-pose outcome tables,
       recomputes the per-level tau from the resampled blank statistics, gamma and the counts, and re-fits ONE curve
       family (the best family of the original fit; refitting all three in every resample would triple the run
       time) to give D_50 and D_10. The model D_0 interval is the profile-likelihood interval of Step 7, not a
       bootstrap (the threshold-model fit is far slower than the logistic fit). The number of resamples is
       BOOTSTRAP_RESAMPLES, reduced to DetectionOptions.reduced_bootstrap_resamples when the configuration has fewer
       than DetectionOptions.full_bootstrap_min_poses poses (stated in the details).
    10 Every minimum in mm, px and mrad; theta against Z in a figure.
    11 D_detect_summary.csv, D_detect_details.json, figures (psychometric curves with binomial error bars and the
       minimums marked; theta against Z).

Helpers other modules import from here: ``collect_trials`` / ``ConfigTrials`` (Analysis E, Step 6, recomputes the two
rules per diameter), ``window_statistic``, ``detected_at``.
"""
from __future__ import annotations

import math
import time
import warnings
from dataclasses import dataclass, field
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
from sensorperf.geometry.targets import (
    FEATURE_BLANK, FEATURE_CUTOUT, FEATURE_DISK, SURFACE_NONE, Feature, StereoGeometry,
)
from sensorperf.io.capture_set import load_stack
from sensorperf.io.manifest import (
    SUBSERIES_CONTINUOUS, SUBSERIES_JITTER, SUBSERIES_MAIN, FrameRecord, group_by_pose, pose_order, select,
)
from sensorperf.io.session import Session
from sensorperf.parameters import PROCEDURE_AREA, PROCEDURE_DETECTION
from sensorperf.stats.intervals import bootstrap_statistic, clopper_pearson, clopper_pearson_upper
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
DETAILS_FILE_NAME = "D_detect_details.json"
D_SUBSERIES_EXCLUDED = (SUBSERIES_CONTINUOUS,)
"""D sub-series that are not detection-curve trials (the continuous-angle variant varies Z, not the level)."""
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
BOOTSTRAP_STAT_NAMES = ("d50_mm", "d10_mm", "gamma", "tau_mm")
"""The quantities the bootstrap resamples, in order."""
PSYCHOMETRIC_CURVE_POINTS = 200
"""Points of the fitted curves drawn in the figures."""
P_STAR_D50 = 0.5
P_STAR_D10 = 0.1
"""The corrected detection probabilities that define D_50 and D_10 (Section 13, Steps 4 and 5)."""
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
    """Number of bootstrap resamples; None means BOOTSTRAP_RESAMPLES for a configuration with at least
    ``full_bootstrap_min_poses`` poses and ``reduced_bootstrap_resamples`` otherwise."""
    reduced_bootstrap_resamples: int = 200
    """Resamples used when a configuration has fewer than ``full_bootstrap_min_poses`` poses."""
    full_bootstrap_min_poses: int = 30
    """Poses (trials) a configuration needs for the full BOOTSTRAP_RESAMPLES."""
    min_trials_per_level: int = 20
    """Fewer trials than this at the best-sampled level and the curve fit is not attempted ("too few trials")."""
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
    """One diameter level of a plate: its feature, the blank site of the same window size, and their diameters."""

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
    """Steps 2 for one frame: ``{(site_id, rule): S}`` for every level's feature and blank window that lies inside the
    image. Reference planes are fitted to the frame itself away from the edges (the registered planes where too
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
            feature = target.feature(site_id)
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


def _fit_parameters(params, curve: str | None = None) -> PsychometricFitParameters:
    if curve is None:
        return PsychometricFitParameters(lapse_rate_max=params.detection_lapse_rate_max)
    return PsychometricFitParameters(lapse_rate_max=params.detection_lapse_rate_max, curve=curve)


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


def _fit_thresholds(levels, successes, trials, gamma, params, curve: str) -> tuple[float, float]:
    """(D_50, D_10) in mm from one fitted curve family; NaN for a threshold the corrected proportions do not bracket."""
    keep = trials > 0
    corrected = np.asarray(corrected_rate(successes[keep] / trials[keep], gamma))
    reaches_half, reaches_tenth = bracketed(corrected, P_STAR_D50), bracketed(corrected, P_STAR_D10)
    if not (reaches_half or reaches_tenth):
        return NO_THRESHOLD, NO_THRESHOLD
    fit = fit_psychometric(levels[keep], successes[keep], trials[keep], gamma, _fit_parameters(params, curve))
    return (fit.threshold(P_STAR_D50) if reaches_half else NO_THRESHOLD,
            fit.threshold(P_STAR_D10) if reaches_tenth else NO_THRESHOLD)


def empirical_d0(levels: np.ndarray, successes: np.ndarray, trials: np.ndarray, gamma: float, confidence: float,
                 bound: float) -> dict[str, Any]:
    """Step 6: the empirical zero-detection size. Returns the per-level one-sided upper bounds psi_U and corrected
    P*_U and D_0,emp (NaN when even the smallest level fails), the next level up, and the 'passing' flags."""
    order = np.argsort(levels, kind="stable")
    rows = []
    passing = True
    d0 = NO_THRESHOLD
    next_level = NO_THRESHOLD
    for position, index in enumerate(order):
        n, s = int(trials[index]), int(successes[index])
        if n < 1:
            continue
        upper = clopper_pearson_upper(s, n, confidence)
        corrected_upper = float(corrected_rate(upper, gamma))
        ok = corrected_upper <= bound
        rows.append({"d_mm": float(levels[index]), "trials": n, "detections": s, "psi_upper": upper,
                     "p_star_upper": corrected_upper, "within_bound": bool(ok)})
        if passing and ok:
            d0 = float(levels[index])
        elif passing:
            passing = False
            next_level = float(levels[index])
    if passing and rows:                                 # every level passes: the 0 percent point is above the range
        next_level = NO_THRESHOLD
    return {"levels": rows, "d0_emp_mm": d0, "d0_emp_next_mm": next_level}


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
    """One dict per configuration and rule (the SUMMARY_COLUMNS keys)."""
    details: list[dict[str, Any]]
    """Per configuration and rule: the level table, fits, bound table, independence, bootstrap summary."""
    configs: list[ConfigTrials]
    notes: list[str] = field(default_factory=list)
    z_reference_mm: float = 750.0
    gap_small_mm: float = 15.0

    def forward_model_terms(self) -> dict[str, Any]:
        """Terms for forward_model_parameters.json: ``d50_px`` and ``d10_px`` of the cutouts under the primary rule
        at the station nearest the mid station Z_REFERENCE_MM (the small gap, on-axis, when several exist). Values
        that could not be estimated are omitted."""
        rows = [r for r in self.rows if r["kind"] == FEATURE_CUTOUT and r["rule"] == RULE_PRIMARY and r["field"] == 0]
        if not rows:
            return {}
        station = min({r["station_z_mm"] for r in rows}, key=lambda z: abs(z - self.z_reference_mm))
        candidates = [r for r in rows if r["station_z_mm"] == station]
        row = min(candidates, key=lambda r: abs((r["gap_mm"] or 0.0) - self.gap_small_mm))
        terms = {}
        for key in ("d50_px", "d10_px"):
            if math.isfinite(row[key]):
                terms[key] = float(row[key])
        return terms


# ---------------------------------------------------------------------------
# The analysis
# ---------------------------------------------------------------------------
def _minimum_columns() -> list[str]:
    """Names of the threshold columns that are reported in mm, px and mrad."""
    return ["d50", "d50_lower", "d50_upper", "d10", "d10_lower", "d10_upper", "d10_model_min", "d10_model_max",
            "d50_isotonic", "d10_isotonic", "d0_emp", "d0_emp_next", "d0_model", "d0_model_lower", "d0_model_upper",
            "d0_geometric", "d0_geometric_cameras"]


def _summary_columns() -> list[str]:
    columns = ["target_id", "kind", "gap_mm", "station_z_mm", "field", "rule", "trials_per_level",
               "trials_per_level_max", "trials_new", "trials_reused", "gamma", "gamma_lower", "gamma_upper",
               "gamma_blank_trials", "tau_mm", "best_curve", "d50_status", "d10_status", "d0_model_status"]
    for name in _minimum_columns():
        columns += [f"{name}_mm", f"{name}_px", f"{name}_mrad"]
    columns += ["independence_ok", "bootstrap_resamples", "bootstrap_failures", "fit_note"]
    return columns


SUMMARY_COLUMNS = tuple(_summary_columns())
"""Columns of D_detect_summary.csv: the specification's list (target, gap, Z, gamma and its interval, tau, best curve,
D_50, D_10 with their intervals, the D_10 model range, the isotonic values, the D_0,emp bracket, model D_0 with its
interval, the geometric D_0, independence flag), each minimum in mm, px and mrad, plus status and bookkeeping columns."""


def run_detection(session: Session, out_dir: Path, previous: Mapping[str, Any] | None,
                  options: DetectionOptions | None = None) -> DetectionResult | None:
    """Analysis D (Section 13, Steps 1 to 10) on the procedure-"D" frames of the session, plus the first frame of
    each C pose of a matching configuration as extra trials (flagged as reused). Returns None when the session has no
    D frames. Nothing is written here; see ``write_outputs``."""
    options = DetectionOptions() if options is None else options
    d_records = [r for r in select(session.records, procedure=PROCEDURE_DETECTION)
                 if r.subseries not in D_SUBSERIES_EXCLUDED]
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
    rows: list[dict[str, Any]] = []
    details: list[dict[str, Any]] = []
    for config in configs:
        for rule in config.rules():
            row, detail = _analyze_config_rule(session, config, rule, stereo, options)
            rows.append(row)
            details.append(detail)
    if not rows:
        notes.append("no array configuration with a back plate was found among the D frames")
    return DetectionResult(rows=rows, details=details, configs=configs, notes=notes,
                           z_reference_mm=params.z_reference_mm, gap_small_mm=params.gap_small_mm)


def _units(session: Session, value_mm: float, station_z_mm: float) -> tuple[float, float, float]:
    """(mm, px, mrad) of a size at the station: D_px = D f_x / Z and theta = D / Z in mrad (SensorGeometry helpers)."""
    if value_mm is None or not math.isfinite(value_mm):
        return math.nan, math.nan, math.nan
    geometry = session.geometry
    return (float(value_mm), geometry.diameter_in_pixels(value_mm, station_z_mm),
            geometry.subtended_angle_mrad(value_mm, station_z_mm))


def _bootstrap_count(config: ConfigTrials, options: DetectionOptions, params) -> int:
    if options.bootstrap_resamples is not None:
        return options.bootstrap_resamples
    if len(config.pose_keys) >= options.full_bootstrap_min_poses:
        return params.bootstrap_resamples
    return options.reduced_bootstrap_resamples


def _analyze_config_rule(session: Session, config: ConfigTrials, rule: str, stereo: StereoGeometry,
                         options: DetectionOptions) -> tuple[dict[str, Any], dict[str, Any]]:
    """Steps 1 and 3 to 10 for one configuration and rule: the summary row and the detail record."""
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

    # Step 8: geometric limit for cutouts.
    d0_geo = d0_geo_cameras = math.nan
    if config.kind == FEATURE_CUTOUT:
        plate = session.targets.get(config.target_ids[0], config.gap_mm)
        top = options.max_geometric_scale * float(levels_mm.max())
        d0_geo = geometric_limit_diameter_mm(plate, config.station_z_mm, stereo, True, top, options.area_options)
        d0_geo_cameras = geometric_limit_diameter_mm(plate, config.station_z_mm, stereo, False, top,
                                                     options.area_options)

    enough = trials.max() >= options.min_trials_per_level if trials.size else False
    d50 = d10 = d50_lo = d50_hi = d10_lo = d10_hi = d10_min = d10_max = iso50 = iso10 = math.nan
    d0 = {"d0_emp_mm": math.nan, "d0_emp_next_mm": math.nan, "levels": []}
    model = None
    best_name = ""
    row.update(d50_status="too few trials", d10_status="too few trials", d0_model_status="too few trials",
               best_curve="", bootstrap_resamples=0, bootstrap_failures=0, fit_note="")
    if keep.sum() and enough:
        # Steps 4 and 5.
        with _quiet():
            fits = fit_all_curves(levels_mm[keep], successes[keep], trials[keep], gamma, _fit_parameters(params))
        best = best_fit(fits)
        best_name = best.curve
        row["best_curve"] = best_name
        row["d50_status"] = threshold_status(corrected[keep], P_STAR_D50)
        row["d10_status"] = threshold_status(corrected[keep], P_STAR_D10)
        if row["d50_status"] == "ok":
            d50 = best.threshold(P_STAR_D50)
        if row["d10_status"] == "ok":
            d10 = best.threshold(P_STAR_D10)
            tails = [f.threshold(P_STAR_D10) for f in fits.values()]
            d10_min, d10_max = float(min(tails)), float(max(tails))
        proportions = corrected[keep]
        iso50 = isotonic_threshold(levels_mm[keep], proportions, P_STAR_D50, trials[keep])
        iso10 = isotonic_threshold(levels_mm[keep], proportions, P_STAR_D10, trials[keep])
        detail["fits"] = {name: {"alpha": f.alpha, "beta": f.beta, "lapse_rate": f.lapse_rate,
                                 "deviance": f.deviance, "converged": f.converged, "d50_mm": f.threshold(P_STAR_D50),
                                 "d10_mm": f.threshold(P_STAR_D10)} for name, f in fits.items()}
        detail["curve_mm"] = _curve_points(levels_mm[keep], best)
        # Step 6.
        d0 = empirical_d0(levels_mm, successes, trials, gamma, params.confidence_level,
                          params.detection_zero_probability_bound)
        # Step 7.
        try:
            with _quiet():
                model = fit_threshold_model(levels_mm[keep], proportions, trials[keep], params.confidence_level)
            row["d0_model_status"] = "ok"
            detail["threshold_model"] = {"d0_mm": model.d0, "scale_mm": model.scale, "shape": model.shape,
                                         "d0_lower_mm": model.d0_lower, "d0_upper_mm": model.d0_upper,
                                         "deviance": model.deviance, "converged": model.converged,
                                         "profile_d0_mm": model.profile_d0, "profile_statistic": model.profile_statistic}
        except ValueError as error:                      # every effective count is zero
            row["d0_model_status"] = "not estimable"
            detail["threshold_model"] = {"not_estimable": str(error)}
        # Step 9: bootstrap over poses.
        resamples = _bootstrap_count(config, options, params)
        statistic = _make_bootstrap_statistic(s_feature, s_blank, levels_mm, params, best_name,
                                              row["d50_status"] == "ok", row["d10_status"] == "ok")
        rng = np.random.default_rng(options.bootstrap_seed)
        started = time.time()
        try:
            boot = bootstrap_statistic(list(range(s_feature.shape[0])), statistic, resamples,
                                       params.confidence_level, rng)
            d50_lo, d10_lo, _, _ = (float(v) for v in boot.lower)
            d50_hi, d10_hi, _, _ = (float(v) for v in boot.upper)
            if row["d50_status"] != "ok":
                d50_lo = d50_hi = math.nan
            if row["d10_status"] != "ok":
                d10_lo = d10_hi = math.nan
            row["bootstrap_resamples"], row["bootstrap_failures"] = resamples, boot.failures
            detail["bootstrap"] = {
                "resamples": resamples, "failures": boot.failures, "seconds": time.time() - started,
                "reduced": options.bootstrap_resamples is None and resamples != params.bootstrap_resamples,
                "poses": s_feature.shape[0], "names": list(BOOTSTRAP_STAT_NAMES),
                "lower": [float(v) for v in boot.lower], "upper": [float(v) for v in boot.upper],
                "estimate": [float(v) for v in np.asarray(boot.estimate)]}
        except ValueError as error:
            row["fit_note"] = f"bootstrap failed: {error}"
    elif not enough:
        row["fit_note"] = (f"too few trials: at most {int(trials.max()) if trials.size else 0} per level "
                           f"(minimum {options.min_trials_per_level}); curve fits not attempted")
    detail["d0_empirical"] = d0
    if model is not None:
        row_d0_model, row_d0_lower, row_d0_upper = model.d0, model.d0_lower, model.d0_upper
    else:
        row_d0_model = row_d0_lower = row_d0_upper = math.nan

    values = {"d50": d50, "d50_lower": d50_lo, "d50_upper": d50_hi, "d10": d10, "d10_lower": d10_lo,
              "d10_upper": d10_hi, "d10_model_min": d10_min, "d10_model_max": d10_max, "d50_isotonic": iso50,
              "d10_isotonic": iso10, "d0_emp": d0["d0_emp_mm"], "d0_emp_next": d0["d0_emp_next_mm"],
              "d0_model": row_d0_model, "d0_model_lower": row_d0_lower, "d0_model_upper": row_d0_upper,
              "d0_geometric": d0_geo, "d0_geometric_cameras": d0_geo_cameras}
    for name, value in values.items():
        mm, px, mrad = _units(session, value, config.station_z_mm)
        row[f"{name}_mm"], row[f"{name}_px"], row[f"{name}_mrad"] = mm, px, mrad
    detail["minimums"] = {name: {"mm": row[f"{name}_mm"], "px": row[f"{name}_px"], "mrad": row[f"{name}_mrad"]}
                          for name in values}
    return row, detail


def _curve_points(levels_mm: np.ndarray, fit: PsychometricFit) -> dict[str, Any]:
    """The fitted raw-probability curve sampled over the tested range (for the figures and the details)."""
    grid = np.geomspace(float(levels_mm.min()), float(levels_mm.max()), PSYCHOMETRIC_CURVE_POINTS)
    return {"d_mm": grid, "psi": fit.probability(grid), "curve": fit.curve}


def _make_bootstrap_statistic(s_feature: np.ndarray, s_blank: np.ndarray, levels_mm: np.ndarray, params,
                              curve: str, want_d50: bool, want_d10: bool):
    """The bootstrap statistic of Step 9: given the drawn pose indices, recompute the per-level tau from the drawn
    blank statistics, the detections, gamma and the counts, refit the (best) curve family and return
    (D_50, D_10, gamma, median tau). A resample whose proportions do not bracket a threshold that the original data
    bracket gives NaN (counted as a failure by the bootstrap). A threshold the original data do not bracket is not
    resampled (its slot is 0.0 and its interval is reported as NaN by the caller)."""
    target = params.detection_false_alarm_target

    def statistic(drawn: Sequence[int]) -> np.ndarray:
        index = np.asarray(drawn, dtype=int)
        sf, sb = s_feature[index], s_blank[index]
        taus = calibrate_tau(sb, target)
        fd, bd, fv, bv = outcomes(sf, sb, taus)
        successes, trials, gamma, _, _ = counts_from_outcomes(fd, bd, fv, bv)
        with _quiet():
            d50, d10 = _fit_thresholds(levels_mm, successes, trials, gamma, params, curve)
        d50 = d50 if want_d50 else 0.0
        d10 = d10 if want_d10 else 0.0
        finite_taus = taus[np.isfinite(taus)]
        return np.array([d50, d10, gamma, float(np.median(finite_taus)) if finite_taus.size else math.nan])

    return statistic


# ---------------------------------------------------------------------------
# Outputs (Step 11)
# ---------------------------------------------------------------------------
def write_outputs(result: DetectionResult, out_dir: Path) -> list[Path]:
    """Write D_detect_summary.csv, D_detect_details.json and the figures into ``out_dir``; returns the paths."""
    out_dir = Path(out_dir)
    written = [write_csv_rows(out_dir / SUMMARY_FILE_NAME, result.rows, SUMMARY_COLUMNS)]
    document = {"notes": result.notes, "configurations": result.details}
    written.append(write_json(out_dir / DETAILS_FILE_NAME, document))
    written += _figures_psychometric(result, out_dir)
    written += _figure_theta(result, out_dir)
    return written


def _figures_psychometric(result: DetectionResult, out_dir: Path) -> list[Path]:
    """Step 11: per configuration the raw detection fractions with Clopper-Pearson error bars against D_px, the fitted
    curve of each rule, and the three minimums marked (D_50 solid, D_10 dashed, D_0,emp dotted)."""
    written: list[Path] = []
    keys = sorted({(d["kind"], d["gap_mm"], d["station_z_mm"], d["field"]) for d in result.details},
                  key=lambda k: (k[0], k[1], k[2], k[3]))
    rule_colors = {RULE_PRIMARY: OKABE_ITO_BLUE, RULE_INCLUSIVE: OKABE_ITO_VERMILLION}
    for kind, gap, station, field_code in keys:
        members = [(d, r) for d, r in zip(result.details, result.rows)
                   if (d["kind"], d["gap_mm"], d["station_z_mm"], d["field"]) == (kind, gap, station, field_code)]
        figure, axis = new_figure(6.5, 4.5)
        for detail, row in members:
            rule = detail["rule"]
            table = [t for t in detail["levels"] if t["trials"] > 0]
            if not table:
                continue
            x = np.array([t["d_px"] for t in table])
            p = np.array([t["proportion"] for t in table])
            lower = np.array([t["ci_lower"] for t in table])
            upper = np.array([t["ci_upper"] for t in table])
            color = rule_colors[rule]
            shift = RULE_PLOT_SHIFT if rule == RULE_INCLUSIVE else 1.0     # keeps coincident points visible
            axis.errorbar(x * shift, p, yerr=[np.maximum(p - lower, 0.0), np.maximum(upper - p, 0.0)],
                          fmt=KIND_MARKERS[kind], color=color, capsize=2, label=f"{rule.replace('_', ' ')} rule")
            curve = detail.get("curve_mm")
            if curve is not None:
                px = np.asarray(curve["d_mm"]) * detail["px_per_mm"]
                axis.plot(px, curve["psi"], color=color, linewidth=1.2)
            for name, style in (("d50", "-"), ("d10", "--"), ("d0_emp", ":"), ("d0_model", "-.")):
                value = row[f"{name}_px"]
                if math.isfinite(value) and value > 0.0:
                    axis.axvline(value, color=color, linestyle=style, linewidth=0.9)
            geometric = row["d0_geometric_px"]
            if rule == RULE_PRIMARY and math.isfinite(geometric) and geometric > 0.0:
                axis.axvline(geometric, color=OKABE_ITO_BLACK, linestyle=(0, (6, 3)), linewidth=0.7)
        axis.axhline(0.0, color=OKABE_ITO_BLACK, linewidth=0.4)
        log_axis(axis)
        axis.set_ylim(-0.05, 1.05)
        axis.set_xlabel("feature diameter D_px = D f_x / Z (px)")
        axis.set_ylabel("detection fraction (error bars: 95% Clopper-Pearson)")
        axis.set_title(f"{kind}s, G = {gap:g} mm, Z = {station:g} mm"
                       + (f", field {field_code}" if field_code else "") + "\nsolid D_50, dashed D_10, dotted D_0 empirical, dash-dot D_0 model, black long dashes geometric limit",
                       fontsize=8)
        axis.legend(fontsize=8)
        axis.grid(True, linewidth=0.3, which="both")
        written += save_figure(figure, out_dir / f"D_psychometric_{kind}_G{gap:g}_Z{station:g}_F{field_code}")
    return written


def _figure_theta(result: DetectionResult, out_dir: Path) -> list[Path]:
    """Step 10: theta (mrad) of the 50 percent, 10 percent and 0 percent (empirical) minimums against Z, one panel per
    kind and rule, one line per gap."""
    panels = sorted({(r["kind"], r["rule"]) for r in result.rows})
    if not panels:
        return []
    figure, axes = new_figure(5.0 * len(panels), 4.2)
    figure.clf()
    axes = figure.subplots(1, len(panels), squeeze=False, sharey=True)[0]
    styles = (("d50", "o", OKABE_ITO_BLUE, "50%"), ("d10", "s", OKABE_ITO_ORANGE, "10%"),
              ("d0_emp", "^", OKABE_ITO_VERMILLION, "0% (empirical)"))
    drawn = False
    for axis, (kind, rule) in zip(axes, panels):
        rows = [r for r in result.rows if r["kind"] == kind and r["rule"] == rule and r["field"] == 0]
        for gap in sorted({r["gap_mm"] for r in rows}):
            gap_rows = sorted((r for r in rows if r["gap_mm"] == gap), key=lambda r: r["station_z_mm"])
            for name, marker, color, label in styles:
                points = [(r["station_z_mm"], r[f"{name}_mrad"]) for r in gap_rows if math.isfinite(r[f"{name}_mrad"])]
                if points:
                    drawn = True
                    axis.plot(*zip(*points), marker=marker, color=color, linestyle="-" if gap == min(
                        r["gap_mm"] for r in rows) else ":", label=f"{label}, G = {gap:g} mm")
        axis.set_xlabel("station Z (mm)")
        axis.set_title(f"{kind}s, {rule.replace('_', ' ')} rule", fontsize=9)
        axis.grid(True, linewidth=0.3)
        if drawn:
            axis.legend(fontsize=7)
    axes[0].set_ylabel("subtended angle theta = D / Z (mrad)")
    figure.tight_layout()
    return save_figure(figure, out_dir / "D_theta_vs_z")
