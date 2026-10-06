"""
Analysis E: boundary detection bias (procedure document, Section 14), Steps 1 to 8.

What it computes
    At a boundary the sensor can report a value where stereo geometry says no reading is possible (a fabricated
    read) or no value where a valid surface was visible (a dropout), and when it does read it can favor the near
    (front) or the far (back) surface. This analysis measures those behaviors from the B edge frames (the raised
    square T3a and the square window T3b) and the C array frames (disks and cutouts); no new captures are needed.

Conventions (docs/design/code_design.md, Section 4)
    Camera frame = left IR camera; depth is camera z in mm, NaN for no-reads; pixels (u, v) = (column, row).
    s is the signed distance (px) of a pixel's front-plane point to the TRUE edge, positive on the FRONT-material
    side (PoseGeometry.signed_distance_px: for a raised square or a disk the inside of the feature is s > 0; for a
    window or a cutout the outside is s > 0). A pixel-frame is counted once, for the edge (or feature) it is nearest.

How the steps are implemented
    1  Visibility V per pixel of the band |s| <= BOUNDARY_BAND_HALF_WIDTH_PX, from the registered pose
       (PoseGeometry.visibility_with_projector and visibility_cameras_only); every statistic is reported for both
       rules (the "visibility_rule" column: projector, cameras).
    2  Outcome per pixel-frame: front read (|Z - Z_front| <= SURFACE_ASSIGNMENT_SIGMA_MULTIPLE sigma_tot), back
       read (the same against Z_back), intermediate (valid, neither), no-read. The reference planes are fitted to
       the pose-mean depth away from the edges (common.reference_planes, registered planes as fallback). sigma_tot:
       from ``previous["A"]`` (its rows at the nearest station) when Analysis A ran and offers them, else the robust
       standard deviation (1.4826 MAD) of Z - Z_front over the front pixels away from the edges of the same
       configuration (for a disk array, whose front faces are too small, Z - Z_back over the back plate away from
       the edges); the details say which source each configuration used. A pixel whose ideal ray meets neither plate
       (outside the target) is not in the band.
    3  Outcome profiles P_front, P_back, P_mid, P_none against s in bins of BOUNDARY_BIN_WIDTH_PX, pooled over the
       phase-jitter poses (and over the frames of a pose).
       Straight edges (T3a, T3b): each of the four edges separately, by the signed distance to the edge's line, for
       pixels in the band whose position along the edge is at least (band + EDGE_CORNER_MARGIN_PX) pixels inside
       the edge's end points (the one-edge-near rule: pixels near a corner, where two edges are within reach, are
       excluded). Orientation: left/right edges are near-vertical, their normal along H, the baseline ("along");
       top/bottom edges are near-horizontal ("across"). Disks and cutouts: the signed distance from the circle (r - D/2), one feature
       per pixel (the nearest).
    4  R_fab = P(read | V = 0); R_drop = P(no-read | V = 1); beta_read = (R_fab - R_drop) / (R_fab + R_drop), NaN
       when both are 0. W_fab = sum over bins of (reads at V = 0 in the bin / pixels in the bin) x bin width, the
       width in px over which reads extend into the geometrically unreadable region; W_drop the same for no-reads
       at V = 1 (the width over which no-reads intrude into the readable region). Both are in px and lie between
       0 and the band width.
    5  pi_near = (N_fb - N_bf) / (N_fb + N_bf), N_fb the front reads at s < 0, N_bf the back reads at s > 0; the mean
       normalized height of the intermediate reads; s_50 from the mean-h profile (the edge spread function of the
       band). Cross-checks against Analysis B's edge offset and Analysis C's edge bias are reported, not enforced:
       foreground fattening means pi_near > 0, s_50 < 0, b > 0 for disks and b < 0 for cutouts.
    6  Feature scale (C arrays): per feature and frame the majority outcome of the pixels inside the true outline
       (the C classes: back read h < 0.5, front read, no-read). For a cutout, back = correct and front = fill-in; for
       a disk, front = correct and back = erased. Fractions over the poses against D_px, with the spread across
       poses. The profiles are against D_px = D f_x / Z, pooled over the features of a plate and over all stations
       (the table keeps the feature identity and the station as columns; the figure draws one curve per feature). The
       two detection rules of Analysis D (detection.collect_trials on the first frame of each C pose)
       are evaluated per feature and station and their difference is reported.
    7  Breakdowns: every group of the CSV is one cell of the breakdown by source (B straight edges or C circular
       features), target (polarity: raised T3a/T4 or window T3b/T5), orientation, Z and G. Confidence intervals
       by bootstrap over poses of the per-pose bin counts (BOOTSTRAP_RESAMPLES). Rows with target_id "all" pool
       every target and gap of a source at a station; the forward-model terms are the pooled B row at the station
       nearest Z_REFERENCE_MM under the projector rule.
    8  E_boundary_bias.csv, E_boundary_details.json, figures (stacked outcome profiles, beta_read and pi_near
       against Z, feature-scale outcome fractions against D_px).
"""
from __future__ import annotations

import math
import warnings
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from sensorperf.analysis.area import (
    KIND_COLORS, KIND_LINESTYLES, KIND_MARKERS, OKABE_ITO_BLACK, OKABE_ITO_BLUE, OKABE_ITO_BLUISH_GREEN,
    OKABE_ITO_ORANGE, OKABE_ITO_SKY_BLUE, OKABE_ITO_VERMILLION, OKABE_ITO_YELLOW, STATION_COLORS, log_axis,
)
from sensorperf.analysis.common import (
    BinnedProfile, MIN_PIXELS_FOR_PLANE, PoseGeometry, away_from_edges, new_figure, pose_geometry, reference_planes,
    save_figure, write_csv_rows, write_json,
)
from sensorperf.analysis.detection import (
    RULE_INCLUSIVE, RULE_PRIMARY, SOURCE_C, calibrate_tau, collect_trials, counts_from_outcomes, first_frames,
    outcomes,
)
from sensorperf.features.planes import normalized_height
from sensorperf.geometry.targets import (
    FEATURE_CUTOUT, FEATURE_DISK, SQUARE_EDGE_BOTTOM, SQUARE_EDGE_LEFT, SQUARE_EDGE_RIGHT, SQUARE_EDGE_TOP,
    SURFACE_NONE, Feature,
)
from sensorperf.io.capture_set import load_stack
from sensorperf.io.manifest import (
    SUBSERIES_JITTER, SUBSERIES_MAIN, SUBSERIES_NOMINAL, FrameRecord, group_by_pose, select,
)
from sensorperf.io.session import Session
from sensorperf.parameters import CharacterizationParameters, FIELD_POSITION_CENTER, PROCEDURE_AREA, PROCEDURE_EDGES
from sensorperf.stats.intervals import bootstrap_statistic

# ---------------------------------------------------------------------------
# Names and constants
# ---------------------------------------------------------------------------
SUMMARY_FILE_NAME = "E_boundary_bias.csv"
DETAILS_FILE_NAME = "E_boundary_details.json"
SUMMARY_COLUMNS = (
    "source", "target_id", "kind", "gap_mm", "station_z_mm", "orientation", "polarity", "visibility_rule",
    "r_fab", "r_drop", "beta_read", "beta_lower", "beta_upper", "w_fab_px", "w_drop_px", "pi_near", "pi_lower",
    "pi_upper", "h_mid_mean", "s50_px", "pixels", "poses")
"""Columns of E_boundary_bias.csv (Step 8): the group columns, then the values."""

SOURCE_B = "B"
SOURCE_C_ARRAYS = "C"
"""The 'source' column: B edge frames or C array frames."""
KIND_STRAIGHT_EDGE = "straight_edge"
"""The 'kind' of a B row (C rows carry 'disk' or 'cutout')."""
ORIENTATION_ALONG = "along"
ORIENTATION_ACROSS = "across"
ORIENTATION_ALL = "all"
ORIENTATION_CIRCULAR = "circular"
"""Edge orientations: along the baseline (near-vertical edges), across it (near-horizontal edges), pooled, and the
orientation column of circular features."""
POLARITY_RAISED = "raised"
POLARITY_WINDOW = "window"
POLARITY_ALL = "all"
"""Polarity: front material inside the outline (T3a, disks) or outside it (T3b, cutouts); 'all' for pooled rows."""
TARGET_ALL = "all"
"""Target id of pooled rows."""
RULE_PROJECTOR = "projector"
RULE_CAMERAS = "cameras"
VISIBILITY_RULES = (RULE_PROJECTOR, RULE_CAMERAS)
"""V with the projector condition and with the cameras only (Step 1)."""
EDGE_ORIENTATION = {SQUARE_EDGE_LEFT: ORIENTATION_ALONG, SQUARE_EDGE_RIGHT: ORIENTATION_ALONG,
                    SQUARE_EDGE_TOP: ORIENTATION_ACROSS, SQUARE_EDGE_BOTTOM: ORIENTATION_ACROSS}
"""Left and right edges are near-vertical (normal along the baseline); top and bottom near-horizontal."""
EDGE_SUBSERIES = (SUBSERIES_NOMINAL, SUBSERIES_JITTER)
"""B sub-series pooled (the nominal pose is one more sub-pixel phase)."""
ARRAY_SUBSERIES = (SUBSERIES_MAIN, SUBSERIES_JITTER)
"""C sub-series used (on-axis main configurations)."""
EDGE_CORNER_MARGIN_PX = 2.0
"""Step 3: edge pixels must lie this many pixels beyond the band inside the edge's end points (the one-edge-near rule)."""
SIGMA_ESTIMATION_POSES = 5
"""Poses per configuration used for the local estimate of sigma_tot when Analysis A is not available."""
SIGMA_FLOOR_MM = 1.0e-3
"""Lower guard of sigma_tot (mm), so a noiseless synthetic plane does not give a zero-width assignment band."""
MAD_TO_SIGMA = 1.4826
"""Median absolute deviation to Gaussian sigma."""
N_RULES = 2
N_V = 2
N_OUTCOMES = 4
OUT_FRONT, OUT_BACK, OUT_MID, OUT_NONE = 0, 1, 2, 3
"""Outcome classes of Step 2 (the index order of the count arrays)."""
OUTCOME_NAMES = ("front", "back", "mid", "none")
FEATURE_OUTCOME_BACK, FEATURE_OUTCOME_FRONT, FEATURE_OUTCOME_NONE = 0, 1, 2
"""Feature-scale majority outcomes (Step 6), index order of the per-pose counts."""
OUTCOME_COLORS = {OUT_FRONT: OKABE_ITO_VERMILLION, OUT_BACK: OKABE_ITO_BLUE, OUT_MID: OKABE_ITO_YELLOW,
                  OUT_NONE: OKABE_ITO_BLACK}
"""Stacked profile colors: front (vermillion), back (blue), intermediate (yellow), no-read (black)."""


# ---------------------------------------------------------------------------
# Options and result
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class BoundaryOptions:
    """Settings of Analysis E that are not procedure parameters."""

    bootstrap_resamples: int | None = None
    """Bootstrap resamples; None means BOOTSTRAP_RESAMPLES."""
    bootstrap_seed: int = 20261005
    """Seed of the bootstrap generator."""
    corner_margin_px: float = EDGE_CORNER_MARGIN_PX
    """Step 3: margin of the one-edge-near rule beyond the band, pixels."""
    sigma_estimation_poses: int = SIGMA_ESTIMATION_POSES
    """Poses used for the local sigma_tot estimate."""
    feature_scale: bool = True
    """Compute Step 6 (the feature-scale outcomes and the rule difference) on the C frames."""
    array_edge_min_d_px: float = 6.0
    """Step 3, circular features: features smaller than this many pixels across are left out of the band statistics
    (R_fab, R_drop, W, pi_near, s_50, profiles). For them the sensor does not see an edge but a whole feature that it
    fills in or erases, which is the subject of Step 6; counting their erased interior as 'back reads at s > 0'
    would turn the edge statistic into a feature-scale statistic. They stay in Step 6."""


@dataclass(frozen=True)
class BinSpec:
    """The signed-distance bins of the band: ``count`` bins of ``width`` px over [-band, band]."""

    band: float
    width: float
    count: int
    centers: np.ndarray

    @classmethod
    def from_params(cls, params) -> "BinSpec":
        band, width = params.boundary_band_half_width_px, params.boundary_bin_width_px
        count = int(round(2.0 * band / width))
        centers = -band + (np.arange(count) + 0.5) * (2.0 * band / count)
        return cls(band=band, width=2.0 * band / count, count=count, centers=centers)

    def vector_size(self) -> int:
        """Length of a count vector: (rule, V, outcome, bin) counts, h sums and counts per bin, and the
        intermediate-read h sum and count."""
        return N_RULES * N_V * N_OUTCOMES * self.count + 2 * self.count + 2

    def index(self, s: np.ndarray) -> np.ndarray:
        """Bin index of signed distances (clipped into the band)."""
        return np.clip(np.floor((s + self.band) / self.width).astype(int), 0, self.count - 1)


class _CountVector:
    """Per-pose accumulator of the counts of Steps 2 to 5, kept as ONE flat float vector so that pooling poses and
    resampling poses (bootstrap) are plain sums. Layout: counts[rule, V, outcome, bin], hsum[bin], hn[bin],
    mid_h_sum, mid_n."""

    def __init__(self, bins: BinSpec) -> None:
        self.bins = bins
        self.vec = np.zeros(bins.vector_size())

    def _views(self):
        nb = self.bins.count
        counts = self.vec[:N_RULES * N_V * N_OUTCOMES * nb].reshape(N_RULES, N_V, N_OUTCOMES, nb)
        offset = N_RULES * N_V * N_OUTCOMES * nb
        return counts, self.vec[offset:offset + nb], self.vec[offset + nb:offset + 2 * nb], self.vec[offset + 2 * nb:]

    def add(self, s: np.ndarray, outcome: np.ndarray, v_projector: np.ndarray, v_cameras: np.ndarray,
            h: np.ndarray) -> None:
        """Add pixel-frames: signed distance, outcome class, V under both rules, and the normalized height (NaN for
        no-reads)."""
        if s.size == 0:
            return
        nb = self.bins.count
        counts, hsum, hn, mid = self._views()
        index = self.bins.index(s)
        for rule, v in ((0, v_projector), (1, v_cameras)):
            combined = (v.astype(int) * N_OUTCOMES + outcome) * nb + index
            counts[rule] += np.bincount(combined, minlength=N_V * N_OUTCOMES * nb).reshape(N_V, N_OUTCOMES, nb)
        finite = np.isfinite(h)
        hsum += np.bincount(index[finite], weights=h[finite], minlength=nb)
        hn += np.bincount(index[finite], minlength=nb)
        middle = (outcome == OUT_MID) & finite
        mid[0] += float(h[middle].sum())
        mid[1] += float(np.count_nonzero(middle))


def count_statistics(vec: np.ndarray, bins: BinSpec, rule_index: int, esf_level: float) -> dict[str, float]:
    """Steps 4 and 5 from a (pooled) count vector for one visibility rule: R_fab, R_drop, beta_read, W_fab, W_drop
    (px), pi_near, the mean h of intermediate reads, s_50 (the mean-h crossing of ``esf_level``) and the pixel
    count."""
    nb = bins.count
    holder = _CountVector(bins)
    holder.vec = np.asarray(vec, dtype=float)
    counts, hsum, hn, mid = holder._views()
    c = counts[rule_index]                                # (V, outcome, bin)
    total_v0, total_v1 = c[0].sum(), c[1].sum()
    reads_v0 = c[0, :OUT_NONE].sum()
    none_v1 = c[1, OUT_NONE]
    r_fab = reads_v0 / total_v0 if total_v0 > 0 else math.nan
    r_drop = none_v1.sum() / total_v1 if total_v1 > 0 else math.nan
    denominator = r_fab + r_drop
    beta = (r_fab - r_drop) / denominator if (math.isfinite(denominator) and denominator > 0) else math.nan
    per_bin_total = c.sum(axis=(0, 1))
    with np.errstate(invalid="ignore", divide="ignore"):
        fab_fraction = np.where(per_bin_total > 0, c[0, :OUT_NONE].sum(axis=0) / per_bin_total, 0.0)
        drop_fraction = np.where(per_bin_total > 0, none_v1 / per_bin_total, 0.0)
    w_fab = float(fab_fraction.sum() * bins.width)
    w_drop = float(drop_fraction.sum() * bins.width)
    negative = bins.centers < 0.0
    n_fb = c[:, OUT_FRONT, negative].sum()
    n_bf = c[:, OUT_BACK, ~negative].sum()
    pi = (n_fb - n_bf) / (n_fb + n_bf) if (n_fb + n_bf) > 0 else math.nan
    h_mid = mid[0] / mid[1] if mid[1] > 0 else math.nan
    with np.errstate(invalid="ignore", divide="ignore"):
        esf = np.where(hn > 0, hsum / hn, np.nan)
    s50 = BinnedProfile(centers=bins.centers, mean=esf, count=hn, std=np.zeros(nb)).interpolate_crossing(esf_level)
    return {"r_fab": float(r_fab), "r_drop": float(r_drop), "beta_read": float(beta), "w_fab_px": w_fab,
            "w_drop_px": w_drop, "pi_near": float(pi), "h_mid_mean": float(h_mid), "s50_px": float(s50),
            "pixels": float(c.sum())}


def outcome_profiles(vec: np.ndarray, bins: BinSpec) -> dict[str, np.ndarray]:
    """Step 3: P_front(s), P_back(s), P_mid(s), P_none(s) from a (pooled) count vector. Bins without pixels are NaN;
    elsewhere the four profiles sum to 1."""
    holder = _CountVector(bins)
    holder.vec = np.asarray(vec, dtype=float)
    counts = holder._views()[0][0]                        # rule 0; the profile does not depend on V
    per_outcome = counts.sum(axis=0)                      # (outcome, bin)
    total = per_outcome.sum(axis=0)
    with np.errstate(invalid="ignore", divide="ignore"):
        fractions = np.where(total > 0, per_outcome / total, np.nan)
    profiles = {name: fractions[k] for k, name in enumerate(OUTCOME_NAMES)}
    profiles["count"] = total
    profiles["s_px"] = bins.centers
    return profiles


# ---------------------------------------------------------------------------
# Step 2: sigma_tot
# ---------------------------------------------------------------------------
def _sigma_from_a(previous: Mapping[str, Any] | None, station_z_mm: float) -> float | None:
    """sigma_tot (mm) at the station nearest ``station_z_mm`` from Analysis A's result: its ``rows`` (StationNoise
    dataclasses of sensorperf.analysis.noise; dict rows are accepted too) with ``station_z_mm`` and ``sigma_tot_mm``,
    on-axis main rows preferred. None when A did not run or has no such rows."""
    result = None if previous is None else previous.get("A")
    if result is None:
        return None

    def value(row: Any, name: str, default: Any = None) -> Any:
        return row.get(name, default) if isinstance(row, Mapping) else getattr(row, name, default)

    for attribute in ("summary_rows", "rows", "summary"):
        rows = getattr(result, attribute, None)
        if not isinstance(rows, (list, tuple)):
            continue
        usable = []
        for row in rows:
            sigma, station = value(row, "sigma_tot_mm"), value(row, "station_z_mm")
            if sigma is None or station is None or not math.isfinite(float(sigma)):
                continue
            if value(row, "field", FIELD_POSITION_CENTER) not in (FIELD_POSITION_CENTER, None):
                continue
            if value(row, "subseries", "main") not in ("main", None, ""):
                continue
            usable.append((abs(float(station) - station_z_mm), float(sigma)))
        if usable:
            return min(usable)[1]
    return None


def _local_sigma(session: Session, records: Sequence[FrameRecord], poses: int) -> float | None:
    """Local sigma_tot estimate (Step 2): robust std (1.4826 MAD) of Z - Z_front over the front pixels away from the
    edges of the first ``poses`` poses (all frames); for a plate whose front pixels are too few (disk arrays),
    Z - Z_back over the back plate away from the edges. None when neither has enough pixels."""
    params = session.params
    residuals: list[np.ndarray] = []
    for key, group in list(group_by_pose(records).items())[:poses]:
        stack = load_stack(group)
        geometry = pose_geometry(session, group[0], stack.camera)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            mean_depth = np.nanmean(stack.depth, axis=0)
        planes = reference_planes(mean_depth, geometry, params)
        front_away, back_away = away_from_edges(geometry, params)
        use_front = np.count_nonzero(front_away) >= MIN_PIXELS_FOR_PLANE
        mask, plane = (front_away, planes.z_front) if use_front else (back_away, planes.z_back)
        if plane is None:
            continue
        for depth in stack.depth:
            selected = mask & np.isfinite(depth) & np.isfinite(plane)
            residuals.append(depth[selected] - plane[selected])
    if not residuals:
        return None
    pooled = np.concatenate(residuals)
    if pooled.size < MIN_PIXELS_FOR_PLANE:
        return None
    return float(MAD_TO_SIGMA * np.median(np.abs(pooled - np.median(pooled))))


# ---------------------------------------------------------------------------
# Per-pose processing (Steps 1 to 3, 6)
# ---------------------------------------------------------------------------
def classify_outcomes(depth: np.ndarray, z_front: np.ndarray, z_back: np.ndarray | None,
                      assignment_mm: float) -> np.ndarray:
    """Step 2: the outcome class per pixel (OUT_* codes): front read within ``assignment_mm`` of the front plane,
    back read within it of the back plane (the nearer plane when both apply), intermediate (valid, neither) and
    no-read (NaN)."""
    valid = np.isfinite(depth)
    with np.errstate(invalid="ignore"):
        d_front = np.abs(depth - z_front)
        d_back = np.abs(depth - z_back) if z_back is not None else np.full(depth.shape, np.inf)
        is_front = valid & (d_front <= assignment_mm) & (d_front <= d_back)
        is_back = valid & (d_back <= assignment_mm) & ~is_front
    outcome = np.full(depth.shape, OUT_NONE, dtype=np.int64)
    outcome[valid] = OUT_MID
    outcome[is_front] = OUT_FRONT
    outcome[is_back] = OUT_BACK
    return outcome


def _edge_masks(geometry: PoseGeometry, feature: Feature, band: float, corner_margin_px: float):
    """Step 3, straight edges: for each edge of a square feature a (selection mask, signed distance image) pair. The
    selection keeps pixels with |s| <= band whose position along the edge lies at least band + corner margin pixels
    inside the end points (so that only one edge is near), on pixels whose ideal ray meets the target."""
    lx, ly = feature.local_coordinates(geometry.front_xy_target[..., 0], geometry.front_xy_target[..., 1])
    reach_mm = feature.diameter_mm / 2.0 - (band + corner_margin_px) * geometry.pixel_footprint_mm
    on_target = geometry.hit.surface != SURFACE_NONE
    out = []
    for edge in (SQUARE_EDGE_LEFT, SQUARE_EDGE_RIGHT, SQUARE_EDGE_TOP, SQUARE_EDGE_BOTTOM):
        s = geometry.signed_distance_px(feature, edge)
        along = ly if edge in (SQUARE_EDGE_LEFT, SQUARE_EDGE_RIGHT) else lx
        selection = on_target & (np.abs(s) <= band) & (np.abs(along) <= reach_mm)
        out.append((edge, selection, s))
    return out


@dataclass
class _PoseResult:
    """What one pose contributes: count vectors per group orientation, the feature-scale majorities."""

    vectors: dict[str, _CountVector]
    """Orientation -> counts (B: along and across; C: circular)."""
    feature_majority: dict[str, np.ndarray] = field(default_factory=dict)
    """site_id -> (3,) frames whose majority outcome was back, front, no-read (Step 6)."""


def _process_pose(session: Session, group_records: list[FrameRecord], sigma_mm: float, bins: BinSpec,
                  options: BoundaryOptions, source: str) -> _PoseResult | None:
    """Steps 1 to 3 (and 6 for arrays) for one pose."""
    params = session.params
    stack = load_stack(group_records)
    geometry = pose_geometry(session, group_records[0], stack.camera)
    target = geometry.target
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        mean_depth = np.nanmean(stack.depth, axis=0)
    planes = reference_planes(mean_depth, geometry, params)
    assignment = params.surface_assignment_sigma_multiple * max(sigma_mm, SIGMA_FLOOR_MM)
    band = bins.band
    v_projector, v_cameras = geometry.visibility_with_projector, geometry.visibility_cameras_only
    result = _PoseResult(vectors={})
    heights = [normalized_height(depth, planes.z_front, planes.z_back) if planes.z_back is not None
               else np.full(depth.shape, np.nan) for depth in stack.depth]
    outcomes_per_frame = [classify_outcomes(depth, planes.z_front, planes.z_back, assignment)
                          for depth in stack.depth]

    if source == SOURCE_B:
        squares = [f for f in target.features if not f.is_circular()]
        for feature in squares:
            for edge, selection, s in _edge_masks(geometry, feature, band, options.corner_margin_px):
                orientation = EDGE_ORIENTATION[edge]
                vector = result.vectors.setdefault(orientation, _CountVector(bins))
                for outcome, h in zip(outcomes_per_frame, heights):
                    vector.add(s[selection], outcome[selection], v_projector[selection], v_cameras[selection],
                               h[selection])
        return result

    # C arrays: each pixel counted for the nearest feature (smallest |s|) when that is within the band.
    features = [f for f in target.features if f.kind in (FEATURE_DISK, FEATURE_CUTOUT)]
    if not features:
        return None
    edge_features = [f for f in features
                     if f.diameter_mm / geometry.pixel_footprint_mm >= options.array_edge_min_d_px]
    if edge_features:
        s_stack = np.stack([geometry.signed_distance_px(f) for f in edge_features])
        nearest = np.argmin(np.abs(s_stack), axis=0)
        s_best = np.take_along_axis(s_stack, nearest[None], axis=0)[0]
        on_target = geometry.hit.surface != SURFACE_NONE
        selection = on_target & np.isfinite(s_best) & (np.abs(s_best) <= band)
        vector = result.vectors.setdefault(ORIENTATION_CIRCULAR, _CountVector(bins))
        for outcome, h in zip(outcomes_per_frame, heights):
            vector.add(s_best[selection], outcome[selection], v_projector[selection], v_cameras[selection],
                       h[selection])

    if options.feature_scale and planes.z_back is not None:
        threshold = params.front_read_height_threshold
        for feature in features:
            region = geometry.inside_outline(feature, 0.0)
            if not region.any():                          # sub-pixel feature: the pixel containing its center
                u0, v0 = geometry.feature_center_px(feature)
                region = np.zeros_like(region)
                row, col = int(round(v0)), int(round(u0))
                if 0 <= row < region.shape[0] and 0 <= col < region.shape[1]:
                    region[row, col] = True
                else:
                    continue
            majority = np.zeros(3)
            for depth, h in zip(stack.depth, heights):
                n_none = int(np.count_nonzero(region & ~np.isfinite(depth)))
                n_front = int(np.count_nonzero(region & np.isfinite(h) & (h >= threshold)))
                n_back = int(np.count_nonzero(region & np.isfinite(h) & (h < threshold)))
                # Ties resolve in the order back, front, no-read (argmax returns the first maximum).
                majority[int(np.argmax([n_back, n_front, n_none]))] += 1.0
            result.feature_majority[feature.site_id] = majority
    return result


# ---------------------------------------------------------------------------
# Result
# ---------------------------------------------------------------------------
@dataclass
class BoundaryBiasResult:
    """Everything Analysis E produced."""

    rows: list[dict[str, Any]]
    """One dict per group and visibility rule (the SUMMARY_COLUMNS keys)."""
    details: dict[str, Any]
    """Profiles, feature-scale curves, cross-checks, sigma sources (E_boundary_details.json)."""
    notes: list[str] = field(default_factory=list)
    z_reference_mm: float = field(default_factory=lambda: CharacterizationParameters().z_reference_mm)

    def forward_model_terms(self) -> dict[str, Any]:
        """Terms for forward_model_parameters.json: ``w_fab_px``, ``w_drop_px``, ``pi_near`` and ``beta_read``,
        pooled over the straight edges (every target and gap) at the station nearest the mid station Z_REFERENCE_MM,
        projector rule; the pooled C row stands in when there are no B frames. Values that are NaN are omitted."""
        for source in (SOURCE_B, SOURCE_C_ARRAYS):
            rows = [r for r in self.rows if r["source"] == source and r["target_id"] == TARGET_ALL
                    and r["visibility_rule"] == RULE_PROJECTOR]
            if not rows:
                continue
            station = min({r["station_z_mm"] for r in rows}, key=lambda z: abs(z - self.z_reference_mm))
            row = next(r for r in rows if r["station_z_mm"] == station)
            return {key: float(row[key]) for key in ("w_fab_px", "w_drop_px", "pi_near", "beta_read")
                    if row[key] is not None and math.isfinite(row[key])}
        return {}


# ---------------------------------------------------------------------------
# The analysis
# ---------------------------------------------------------------------------
def _polarity(session: Session, target_id: str) -> str:
    """Raised when the target's feature is front material (T3a, T4), window otherwise (T3b, T5)."""
    target = session.targets.targets[target_id]
    kinds = [f for f in target.features if f.kind not in ("blank", "post")]
    return POLARITY_RAISED if kinds and kinds[0].is_front_material() else POLARITY_WINDOW


def _kind_of_target(session: Session, target_id: str, source: str) -> str:
    if source == SOURCE_B:
        return KIND_STRAIGHT_EDGE
    target = session.targets.targets[target_id]
    return FEATURE_DISK if any(f.kind == FEATURE_DISK for f in target.features) else FEATURE_CUTOUT


def run_boundary_bias(session: Session, out_dir: Path, previous: Mapping[str, Any] | None,
                      options: BoundaryOptions | None = None) -> BoundaryBiasResult | None:
    """Analysis E (Section 14, Steps 1 to 7) on the B edge frames and the C array frames. Returns None when the
    session has neither. ``previous`` may hold the results of A (sigma_tot), B (edge offset) and C (edge bias).
    Nothing is written here; see ``write_outputs``."""
    options = BoundaryOptions() if options is None else options
    params = session.params
    bins = BinSpec.from_params(params)
    b_records = [r for r in select(session.records, procedure=PROCEDURE_EDGES) if r.subseries in EDGE_SUBSERIES
                 and r.gap_mm is not None]
    c_records = [r for r in select(session.records, procedure=PROCEDURE_AREA) if r.subseries in ARRAY_SUBSERIES
                 and r.field == FIELD_POSITION_CENTER and r.gap_mm is not None]
    if not b_records and not c_records:
        return None
    notes: list[str] = []
    sigma_sources: list[dict[str, Any]] = []
    # pose_data[(source, target, gap, station)] -> list of (pose_key, _PoseResult) in order of appearance
    pose_data: dict[tuple, list[tuple[tuple, _PoseResult]]] = {}
    for source, records in ((SOURCE_B, b_records), (SOURCE_C_ARRAYS, c_records)):
        configurations: dict[tuple, list[FrameRecord]] = {}
        for record in records:
            configurations.setdefault((record.target_id, record.gap_mm, record.station_z_mm), []).append(record)
        for (target_id, gap, station), members in configurations.items():
            sigma = _sigma_from_a(previous, station)
            origin = "analysis A"
            if sigma is None:
                sigma = _local_sigma(session, members, options.sigma_estimation_poses)
                origin = "local robust std of the residual to the reference plane away from the edges"
            if sigma is None:
                sigma, origin = SIGMA_FLOOR_MM, "floor (no usable pixels for an estimate)"
            sigma_sources.append({"source": source, "target_id": target_id, "gap_mm": gap, "station_z_mm": station,
                                  "sigma_tot_mm": sigma, "origin": origin})
            for key, group in group_by_pose(members).items():
                pose = _process_pose(session, group, sigma, bins, options, source)
                if pose is not None:
                    pose_data.setdefault((source, target_id, gap, station), []).append((key, pose))

    rows, profiles, groups_details = _build_rows(session, pose_data, bins, options)
    details: dict[str, Any] = {
        "bins": {"band_px": bins.band, "width_px": bins.width, "centers_px": bins.centers},
        "sigma_tot": sigma_sources, "profiles": profiles, "groups": groups_details,
        "cross_checks": _cross_checks(rows, previous), "notes": notes,
    }
    if options.feature_scale:
        details["feature_scale"] = _feature_scale(session, pose_data, c_records)
    return BoundaryBiasResult(rows=rows, details=details, notes=notes, z_reference_mm=params.z_reference_mm)


def _pose_vector(pose: _PoseResult, bins: BinSpec, orientation: str | None) -> np.ndarray:
    """The pose's count vector of one orientation, or the sum over orientations (orientation None / 'all')."""
    total = np.zeros(bins.vector_size())
    for name, vector in pose.vectors.items():
        if orientation in (None, ORIENTATION_ALL) or name == orientation:
            total += vector.vec
    return total


def _build_rows(session: Session, pose_data: dict[tuple, list[tuple[tuple, _PoseResult]]], bins: BinSpec,
                options: BoundaryOptions) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    """Steps 3 to 5 and 7: the group rows (both visibility rules), the pooled profiles and the group details."""
    params = session.params
    resamples = params.bootstrap_resamples if options.bootstrap_resamples is None else options.bootstrap_resamples
    rng = np.random.default_rng(options.bootstrap_seed)
    # Groups: (source, target, kind, gap, station, orientation, polarity) -> list of per-pose vectors.
    groups: dict[tuple, list[np.ndarray]] = {}
    for (source, target_id, gap, station), poses in pose_data.items():
        kind = _kind_of_target(session, target_id, source)
        polarity = _polarity(session, target_id)
        orientations = {name for _, pose in poses for name in pose.vectors}
        for orientation in sorted(orientations):
            groups[(source, target_id, kind, gap, station, orientation, polarity)] = [
                _pose_vector(pose, bins, orientation) for _, pose in poses]
        if len(orientations) > 1:                         # straight edges: both orientations pooled
            groups[(source, target_id, kind, gap, station, ORIENTATION_ALL, polarity)] = [
                _pose_vector(pose, bins, ORIENTATION_ALL) for _, pose in poses]
    # Pooled over targets and gaps at each station of a source.
    pooled: dict[tuple, list[np.ndarray]] = {}
    for (source, target_id, gap, station), poses in pose_data.items():
        pooled.setdefault((source, station), []).extend(_pose_vector(pose, bins, ORIENTATION_ALL) for _, pose in poses)
    for (source, station), vectors in pooled.items():
        kind = KIND_STRAIGHT_EDGE if source == SOURCE_B else "circular"
        groups[(source, TARGET_ALL, kind, None, station, ORIENTATION_ALL, POLARITY_ALL)] = vectors

    rows: list[dict[str, Any]] = []
    profiles: list[dict[str, Any]] = []
    details: list[dict[str, Any]] = []
    for key in sorted(groups, key=lambda k: tuple("" if v is None else v for v in k)):
        source, target_id, kind, gap, station, orientation, polarity = key
        vectors = groups[key]
        total = np.sum(vectors, axis=0)
        if total.sum() == 0:
            continue
        boot = _bootstrap_group(vectors, bins, params.esf_half_height, resamples, params.confidence_level, rng)
        for rule_index, rule in enumerate(VISIBILITY_RULES):
            stats = count_statistics(total, bins, rule_index, params.esf_half_height)
            lower, upper = boot[rule_index]
            rows.append({
                "source": source, "target_id": target_id, "kind": kind, "gap_mm": gap, "station_z_mm": station,
                "orientation": orientation, "polarity": polarity, "visibility_rule": rule,
                "r_fab": stats["r_fab"], "r_drop": stats["r_drop"], "beta_read": stats["beta_read"],
                "beta_lower": lower[0], "beta_upper": upper[0], "w_fab_px": stats["w_fab_px"],
                "w_drop_px": stats["w_drop_px"], "pi_near": stats["pi_near"], "pi_lower": lower[1],
                "pi_upper": upper[1], "h_mid_mean": stats["h_mid_mean"], "s50_px": stats["s50_px"],
                "pixels": int(stats["pixels"]), "poses": len(vectors)})
        profile = outcome_profiles(total, bins)
        profiles.append({"source": source, "target_id": target_id, "kind": kind, "gap_mm": gap,
                         "station_z_mm": station, "orientation": orientation, "polarity": polarity, **profile})
        details.append({"source": source, "target_id": target_id, "gap_mm": gap, "station_z_mm": station,
                        "orientation": orientation, "bootstrap_resamples": resamples, "poses": len(vectors)})
    return rows, profiles, details


def _bootstrap_group(vectors: list[np.ndarray], bins: BinSpec, esf_level: float, resamples: int,
                     confidence: float, rng: np.random.Generator) -> list[tuple[tuple[float, float],
                                                                                tuple[float, float]]]:
    """Step 7: percentile intervals of beta_read and pi_near by bootstrap over poses (the per-pose count vectors
    are resampled with replacement and summed; one set of resamples serves both visibility rules, since pi_near does
    not depend on V). Returns, per visibility rule, ((beta_lower, pi_lower), (beta_upper, pi_upper)); NaN when there
    are fewer than two poses or no resample gave a value."""
    nan_pair = (math.nan, math.nan)
    if len(vectors) < 2 or resamples < 1:
        return [(nan_pair, nan_pair)] * N_RULES

    def statistic(drawn):
        total = np.sum(drawn, axis=0)
        projector = count_statistics(total, bins, 0, esf_level)
        cameras = count_statistics(total, bins, 1, esf_level)
        return np.array([projector["beta_read"], projector["pi_near"], cameras["beta_read"]])

    try:
        boot = bootstrap_statistic(vectors, statistic, resamples, confidence, rng)
    except ValueError:
        return [(nan_pair, nan_pair)] * N_RULES
    lower, upper = np.atleast_1d(boot.lower), np.atleast_1d(boot.upper)
    return [((float(lower[0]), float(lower[1])), (float(upper[0]), float(upper[1]))),
            ((float(lower[2]), float(lower[1])), (float(upper[2]), float(upper[1])))]


def _cross_checks(rows: list[dict[str, Any]], previous: Mapping[str, Any] | None) -> dict[str, Any]:
    """Step 5: the sign relations between pi_near, s_50 (own, and Analysis B's edge offset when present) and the
    edge bias b of Analysis C when present. Reported, not enforced."""
    out: dict[str, Any] = {"expected": "pi_near > 0 goes with s_50 < 0, b > 0 for disks and b < 0 for cutouts"}
    pooled = [r for r in rows if r["target_id"] == TARGET_ALL and r["visibility_rule"] == RULE_PROJECTOR]
    checks = []
    for row in pooled:
        checks.append({"source": row["source"], "station_z_mm": row["station_z_mm"], "pi_near": row["pi_near"],
                       "s50_px": row["s50_px"],
                       "signs_consistent": bool(math.isfinite(row["pi_near"]) and math.isfinite(row["s50_px"])
                                                and (row["pi_near"] > 0) == (row["s50_px"] < 0))})
    out["own_s50_vs_pi"] = checks
    b_result = None if previous is None else previous.get("B")
    terms = None
    if b_result is not None and hasattr(b_result, "forward_model_terms"):
        try:
            terms = b_result.forward_model_terms()
        except Exception:                                # another analysis' failure must not stop E
            terms = None
    if terms and terms.get("edge_offset_px") is not None:
        offset = terms["edge_offset_px"]
        offset = float(np.mean([v for v in (offset.values() if isinstance(offset, Mapping) else np.atleast_1d(offset))
                                if v is not None and math.isfinite(float(v))] or [math.nan]))
        out["analysis_b_edge_offset_px"] = offset
        out["pi_near_pooled_mean"] = float(np.nanmean([r["pi_near"] for r in pooled])) if pooled else math.nan
        out["b_offset_sign_consistent"] = bool(math.isfinite(offset) and pooled and
                                               ((np.nanmean([r["pi_near"] for r in pooled]) > 0) == (offset < 0)))
    c_result = None if previous is None else previous.get("C")
    if c_result is not None and hasattr(c_result, "edge_bias_terms"):      # C's bias is no forward-model term
        try:
            c_terms = c_result.edge_bias_terms()
        except Exception:
            c_terms = {}
        out["analysis_c_edge_bias"] = {k: v for k, v in c_terms.items() if k.startswith("edge_bias")
                                       and isinstance(v, (int, float))}
    return out


def _feature_scale(session: Session, pose_data: dict[tuple, list[tuple[tuple, _PoseResult]]],
                   c_records: list[FrameRecord]) -> list[dict[str, Any]]:
    """Step 6: per C configuration and feature the fractions of frames whose majority outcome inside the true outline
    was correct, wrong-surface (fill-in for a cutout, erased for a disk) or no-read, with the spread across poses, and
    the detection fractions of the two rules of Analysis D at that diameter and their difference."""
    params = session.params
    fx = float(session.geometry.require("sensor_fx_px"))
    # Detection fractions per (target, gap, station, site) and rule from the first frame of each C pose.
    rule_fractions: dict[tuple, dict[str, float]] = {}
    for config in collect_trials(session, [(r, SOURCE_C) for r in first_frames(c_records)]):
        for rule in config.rules():
            taus = calibrate_tau(config.s_blank[rule], params.detection_false_alarm_target)
            fd, bd, fv, bv = outcomes(config.s_feature[rule], config.s_blank[rule], taus)
            successes, trials, gamma, _, _ = counts_from_outcomes(fd, bd, fv, bv)
            for column, level in enumerate(config.levels):
                if trials[column] > 0:
                    rule_fractions.setdefault((level.target_id, config.gap_mm, config.station_z_mm, level.site_id),
                                              {})[rule] = float(successes[column] / trials[column])
    table: list[dict[str, Any]] = []
    for (source, target_id, gap, station), poses in pose_data.items():
        if source != SOURCE_C_ARRAYS:
            continue
        target = session.targets.get(target_id, gap)
        kind = _kind_of_target(session, target_id, source)
        for feature in target.features:
            if feature.kind not in (FEATURE_DISK, FEATURE_CUTOUT):
                continue
            per_pose = [pose.feature_majority[feature.site_id] for _, pose in poses
                        if feature.site_id in pose.feature_majority]
            if not per_pose:
                continue
            fractions = np.array([p / p.sum() for p in per_pose])        # (poses, [back, front, none])
            mean = fractions.mean(axis=0)
            spread = fractions.std(axis=0, ddof=1) if len(per_pose) > 1 else np.full(3, math.nan)
            correct_index = FEATURE_OUTCOME_BACK if kind == FEATURE_CUTOUT else FEATURE_OUTCOME_FRONT
            wrong_index = FEATURE_OUTCOME_FRONT if kind == FEATURE_CUTOUT else FEATURE_OUTCOME_BACK
            detection = rule_fractions.get((target_id, gap, station, feature.site_id), {})
            primary, inclusive = detection.get(RULE_PRIMARY, math.nan), detection.get(RULE_INCLUSIVE, math.nan)
            table.append({
                "target_id": target_id, "kind": kind, "gap_mm": gap, "station_z_mm": station,
                "site_id": feature.site_id, "level_index": feature.level_index, "diameter_mm": feature.diameter_mm,
                "d_px": feature.diameter_mm * fx / station, "poses": len(per_pose),
                "p_correct": float(mean[correct_index]), "p_wrong_surface": float(mean[wrong_index]),
                "p_fill_in": float(mean[wrong_index]) if kind == FEATURE_CUTOUT else math.nan,
                "p_erased": float(mean[wrong_index]) if kind == FEATURE_DISK else math.nan,
                "p_no_read": float(mean[FEATURE_OUTCOME_NONE]),
                "p_correct_std": float(spread[correct_index]), "p_wrong_surface_std": float(spread[wrong_index]),
                "p_no_read_std": float(spread[FEATURE_OUTCOME_NONE]),
                "detect_primary": primary, "detect_inclusive": inclusive,
                "rule_difference": inclusive - primary if (math.isfinite(primary) and math.isfinite(inclusive))
                else math.nan})
    table.sort(key=lambda r: (r["target_id"], r["gap_mm"], r["station_z_mm"], r["diameter_mm"]))
    return table


# ---------------------------------------------------------------------------
# Outputs (Step 8)
# ---------------------------------------------------------------------------
def write_outputs(result: BoundaryBiasResult, out_dir: Path) -> list[Path]:
    """Write E_boundary_bias.csv, E_boundary_details.json and the figures into ``out_dir``; returns the paths."""
    out_dir = Path(out_dir)
    written = [write_csv_rows(out_dir / SUMMARY_FILE_NAME, result.rows, SUMMARY_COLUMNS),
               write_json(out_dir / DETAILS_FILE_NAME, result.details)]
    written += _figure_profiles(result, out_dir)
    written += _figure_vs_z(result, out_dir)
    written += _figure_feature_scale(result, out_dir)
    return written


def _figure_profiles(result: BoundaryBiasResult, out_dir: Path) -> list[Path]:
    """Step 8: stacked outcome profiles against s, one panel per edge type (source, target, orientation), pooled
    over the stations and gaps of that edge type."""
    profiles = result.details.get("profiles", [])
    types: dict[tuple, list[dict[str, Any]]] = {}
    for p in profiles:
        if p["target_id"] == TARGET_ALL or p["orientation"] == ORIENTATION_ALL:
            continue
        types.setdefault((p["source"], p["target_id"], p["orientation"]), []).append(p)
    if not types:
        return []
    columns = min(len(types), 3)
    rows_n = int(math.ceil(len(types) / columns))
    figure, axes = new_figure(4.4 * columns, 3.2 * rows_n)
    figure.clf()
    axes = figure.subplots(rows_n, columns, squeeze=False, sharey=True)
    for axis, ((source, target_id, orientation), members) in zip(axes.ravel(), sorted(types.items())):
        s = np.asarray(members[0]["s_px"])
        counts = {name: sum(np.nan_to_num(np.asarray(m[name])) * np.asarray(m["count"]) for m in members)
                  for name in OUTCOME_NAMES}
        total = sum(np.asarray(m["count"]) for m in members)
        with np.errstate(invalid="ignore", divide="ignore"):
            stack = [np.where(total > 0, counts[name] / total, 0.0) for name in OUTCOME_NAMES]
        axis.stackplot(s, *stack, colors=[OUTCOME_COLORS[k] for k in range(N_OUTCOMES)], labels=OUTCOME_NAMES)
        axis.axvline(0.0, color="white", linewidth=0.8, linestyle="--")
        axis.set_title(f"{source} {target_id}, {orientation}", fontsize=9)
        axis.set_xlim(s.min(), s.max())
        axis.set_ylim(0, 1)
    for axis in axes[-1, :]:
        axis.set_xlabel("signed distance s (px; > 0 on the front side)")
    for axis in axes[:, 0]:
        axis.set_ylabel("fraction of pixel-frames")
    axes.ravel()[0].legend(fontsize=7, loc="center left")
    for extra in axes.ravel()[len(types):]:
        extra.axis("off")
    figure.tight_layout()
    return save_figure(figure, out_dir / "E_outcome_profiles")


def _figure_vs_z(result: BoundaryBiasResult, out_dir: Path) -> list[Path]:
    """Step 8: beta_read and pi_near against Z (projector rule solid, cameras-only dotted), one line per source,
    target and orientation, error bars from the bootstrap."""
    rows = [r for r in result.rows if r["orientation"] != ORIENTATION_ALL or r["target_id"] == TARGET_ALL]
    if not rows:
        return []
    figure, axes = new_figure(10.0, 4.2)
    figure.clf()
    axes = figure.subplots(1, 2)
    keys = sorted({(r["source"], r["target_id"], r["orientation"], r["gap_mm"]) for r in rows},
                  key=lambda k: tuple("" if v is None else str(v) for v in k))
    for index, key in enumerate(keys):
        color = STATION_COLORS[index % len(STATION_COLORS)]
        label = f"{key[0]} {key[1]} {key[2]}" + (f" G={key[3]:g}" if key[3] is not None else "")
        for rule, style in ((RULE_PROJECTOR, "-"), (RULE_CAMERAS, ":")):
            members = sorted((r for r in rows if (r["source"], r["target_id"], r["orientation"], r["gap_mm"]) == key
                              and r["visibility_rule"] == rule), key=lambda r: r["station_z_mm"])
            for axis, value, low, high in ((axes[0], "beta_read", "beta_lower", "beta_upper"),
                                           (axes[1], "pi_near", "pi_lower", "pi_upper")):
                if rule == RULE_CAMERAS and value == "pi_near":
                    continue                              # pi_near does not depend on the visibility rule
                points = [(r["station_z_mm"], r[value], r[low], r[high]) for r in members
                          if math.isfinite(r[value])]
                if points:
                    z, y, lo, hi = (np.array(v, dtype=float) for v in zip(*points))
                    yerr = [np.where(np.isfinite(lo), np.maximum(y - lo, 0.0), 0.0),
                            np.where(np.isfinite(hi), np.maximum(hi - y, 0.0), 0.0)]
                    axis.errorbar(z, y, yerr=yerr, color=color, linestyle=style, marker="o", capsize=2,
                                  label=label if rule == RULE_PROJECTOR else None)
    for axis, name in zip(axes, ("beta_read (+1: always fills in, -1: always drops out)",
                                 "pi_near (+1: favors the near surface)")):
        axis.axhline(0.0, color=OKABE_ITO_BLACK, linewidth=0.6)
        axis.set_xlabel("station Z (mm)")
        axis.set_ylabel(name, fontsize=8)
        axis.grid(True, linewidth=0.3)
    axes[0].legend(fontsize=6)
    figure.tight_layout()
    return save_figure(figure, out_dir / "E_beta_pi_vs_z")


def _figure_feature_scale(result: BoundaryBiasResult, out_dir: Path) -> list[Path]:
    """Step 8: P(correct), P(fill-in or erased) and P(no-read) against D_px for the cutouts and the disks, pooled over
    all stations of the smaller gap: the colors are the outcomes, the markers and line styles are the features of
    the plate, and the points of a feature are its stations (so a feature's curve spans two octaves of D_px and
    neighboring features overlap)."""
    table = result.details.get("feature_scale", [])
    if not table:
        return []
    kinds = [k for k in (FEATURE_CUTOUT, FEATURE_DISK) if any(r["kind"] == k for r in table)]
    figure, axes = new_figure(5.2 * len(kinds), 4.2)
    figure.clf()
    axes = figure.subplots(1, len(kinds), squeeze=False, sharey=True)[0]
    styles = (("p_correct", OKABE_ITO_BLUE, "correct"), ("p_wrong_surface", OKABE_ITO_VERMILLION, None),
              ("p_no_read", OKABE_ITO_BLACK, "no-read"))
    markers = ("o", "s", "^", "D", "v")
    linestyles = ("-", "--", ":", "-.")
    smaller_gap = min(r["gap_mm"] for r in table)
    for axis, kind in zip(axes, kinds):
        wrong = "fill-in (front read)" if kind == FEATURE_CUTOUT else "erased (back read)"
        members = [r for r in table if r["kind"] == kind and r["gap_mm"] == smaller_gap]
        for position, level in enumerate(sorted({r["level_index"] for r in members})):
            points = sorted((r for r in members if r["level_index"] == level), key=lambda r: r["d_px"])
            for key, color, label in styles:
                axis.plot([r["d_px"] for r in points], [r[key] for r in points], color=color,
                          marker=markers[position % len(markers)], linestyle=linestyles[position % len(linestyles)],
                          markersize=4, label=f"{label or wrong}, feature {level}")
        log_axis(axis)
        axis.set_ylim(-0.05, 1.05)
        axis.set_xlabel("feature diameter D_px (px), all stations pooled")
        axis.set_title(f"{kind}s, G = {smaller_gap:g} mm", fontsize=9)
        axis.grid(True, linewidth=0.3, which="both")
        axis.legend(fontsize=6, ncol=2)
    axes[0].set_ylabel("fraction of frames (majority outcome inside the outline)")
    figure.tight_layout()
    return save_figure(figure, out_dir / "E_feature_scale_vs_dpx")
