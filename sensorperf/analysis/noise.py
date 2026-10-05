"""
Analysis A: noise versus Z (procedure document, Section 10), on the frames of procedure "A"
(the noise-plate series) and the drift sentinels (procedure "S", Step 11).

What it computes, per commanded pose (one row of ``A_noise_summary.csv`` per pose: the main
stations, the tilt sub-series poses and any other A pose)
    Step 1-2  the region of interest (the registered plate outline shrunk by
              BOUNDARY_BAND_HALF_WIDTH_PX) and the ground-truth depth Z_GT(u, v) of the registered plate
    Step 3    the temporal noise map sigma_t(u, v) (per-pixel standard deviation over frames, ddof = 1)
              with its ROI median and 95th percentile
    Step 4    the fixed-pattern noise sigma_fp (standard deviation of the frame-mean depth about a free
              plane), the bias (mean of Zbar - Z_GT over the ROI) and the angle between the fitted and the
              registered plane normals
    Step 5    the total noise sigma_tot (RMS of Z - Z_GT over ROI and frames) and the closure check
              sigma_tot^2 ~ sigma_t^2 + sigma_fp^2 + bias^2
    Step 6    the fill rate
    Step 7    the normalized spatial autocorrelation of the temporal component and of the fixed-pattern
              residual, and the correlation lengths along H and V (first lag at 1/e)
    Step 8    the depth quantum from the central patch (see below)
over the poses
    Step 9    the noise model sigma_t(Z) = sqrt(sigma_0^2 + (sigma_d Z^2 / k)^2) and the free power law
    Step 10   the tilt sub-series curves (sigma_t, sigma_tot, fill rate against incidence angle)
    Step 11   the sentinel drift of every mounted target (sentinels are captured on the target mounted at that point
              of the plan, grouped by target and mount, each relative to its first sentinel after the mount) and,
              if it matters, the time-interpolated bias correction of A from the T2 sentinels of A's mount
    Step 12   the legacy metrics of testZRepeatabilityBrownBoard.py
    Step 13   the outputs: ``A_noise_summary.csv``, ``A_noise_details.json`` and the figures.

Conventions (docs/design/code_design.md, Section 4): millimeters, pixels, degrees; image arrays are
(H, W); depth is camera z and a no-read is NaN. ``s`` of Section 11 is not used here.

Step 8, the depth quantum. Two estimators are computed and compared.
  * Primary: the phase-resultant (circular statistics) method of the 6DOF repository's
    ``sixdof/noise/repeatability.py::estimate_depth_quantum_mm`` (copied in method, not imported; see
    :func:`estimate_quantum_phase_resultant`). For the true quantum q every depth value is close to a multiple
    of q, so value mod q concentrates on one point of the circle; the mean resultant length is then near 1.
    Divisors of q pass the same test, so the LARGEST candidate that clears the concentration threshold is
    the quantum. Two adaptations: (a) the candidates are a fine geometric grid, not the fixed list of that
    module, because a disparity quantizer gives a quantum q Z^2 / k that is not a round number of mm; (b) the
    threshold is relaxed (named constant) because the spacing of a disparity quantizer varies slowly with
    depth, which smears the phase a little. If no candidate coarser than the output LSB clears the
    threshold, the output LSB is the quantizer.
  * Cross-check (the procedure document's wording): the most common spacing between populated depth-code
    levels (:func:`code_spacing_quantum`).
The disparity quantum is q = delta Z_q k / Z^2 at each Z; it should be the same at every Z.

Step 12, the legacy metrics, follow the 6DOF repository's ``sixdof/noise/repeatability.py`` (which implements
testZRepeatabilityBrownBoard.py): per pixel the standard deviation over the frames in which the pixel was
read (``np.nanstd``, ddof = 0, NOT the ddof = 1 of Step 3) and the range (max - min); a pixel is used only if it
was read in at least LEGACY_MIN_VALID_FRACTION of the frames; the boxes are 2 x legacy_box_half_px square,
rows cy - half .. cy + half - 1 and columns cx - half .. cx + half - 1 (the legacy module clips a box at the
image border; this analysis skips a box that does not lie fully inside the image and says so). That module
summarizes the per-pixel standard deviation by its median and percentiles; the procedure document asks for
means; both are reported.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from scipy import fft as sp_fft
from scipy.optimize import least_squares

from sensorperf.analysis.common import (
    central_patch_mask, depth_codes, new_figure, pose_geometry, region_of_interest, save_figure,
    write_csv_rows, write_json,
)
from sensorperf.features.depth_features import temporal_statistics
from sensorperf.features.planes import fit_plane_robust, plane_depth_image
from sensorperf.io.capture_set import PoseStack, load_stack
from sensorperf.io.manifest import (
    FIELD_FRACTION_ACHIEVED_KEY, FrameRecord, SENTINEL_MOUNT_REFERENCE_KEY, SUBSERIES_MAIN, SUBSERIES_TILT, TILT_AXIS_H,
    TILT_AXIS_V, group_by_pose, parse_flag, select,
)
from sensorperf.io.session import Session
from sensorperf.parameters import FIELD_POSITION_CENTER, PROCEDURE_NOISE, PROCEDURE_SENTINEL, TARGET_NOISE_PLATE

# ---------------------------------------------------------------------------
# Output names
# ---------------------------------------------------------------------------
SUMMARY_CSV_NAME = "A_noise_summary.csv"
DETAILS_JSON_NAME = "A_noise_details.json"
FIGURE_STEMS = {
    "sigma": "A_noise_vs_z", "fill": "A_fill_rate_vs_z", "maps": "A_sigma_t_maps",
    "autocorrelation": "A_autocorrelation", "codes": "A_depth_code_histograms", "tilt": "A_tilt_curves",
    "drift": "A_sentinel_drift",
}
"""Stem (without extension) of each figure; PNG and SVG are written."""
SUMMARY_COLUMNS = (
    "station_z_mm", "field", "field_fraction_achieved", "tilt_axis", "tilt_deg", "subseries", "frames", "sigma_t_median_mm", "sigma_t_p95_mm",
    "sigma_fp_mm", "bias_mm", "bias_corrected_mm", "sigma_tot_mm", "closure_ok", "fill_rate", "corr_len_h_px",
    "corr_len_v_px", "corr_len_fp_h_px", "corr_len_fp_v_px", "depth_quantum_mm", "q_px", "plane_angle_deg",
    "legacy_minmax_mm", "legacy_std_mm", "legacy_std_median_mm", "closure_ratio", "quantum_code_spacing_mm", "note")
"""Columns of A_noise_summary.csv: the Step 13 list, then the extra columns (legacy median, closure ratio, the
code-spacing cross-check of Step 8 and a free-text note). ``field_fraction_achieved`` is the achieved fraction of the
requested field offset of the pose (1 when the plate fit at the requested position, smaller when the planner pulled
it inward), taken from the manifest column of that name; NaN when the manifest does not provide it."""

# ---------------------------------------------------------------------------
# Okabe-Ito colors (colorblind-safe palette) for the figures
# ---------------------------------------------------------------------------
OKABE_ITO_BLACK = "#000000"
OKABE_ITO_ORANGE = "#E69F00"
OKABE_ITO_SKY_BLUE = "#56B4E9"
OKABE_ITO_BLUISH_GREEN = "#009E73"
OKABE_ITO_YELLOW = "#F0E442"
OKABE_ITO_BLUE = "#0072B2"
OKABE_ITO_VERMILLION = "#D55E00"
OKABE_ITO_REDDISH_PURPLE = "#CC79A7"
OKABE_ITO_CYCLE = (OKABE_ITO_BLUE, OKABE_ITO_VERMILLION, OKABE_ITO_BLUISH_GREEN, OKABE_ITO_ORANGE,
                   OKABE_ITO_REDDISH_PURPLE, OKABE_ITO_SKY_BLUE, OKABE_ITO_BLACK)
"""Colors used in turn for the series of a figure."""

# ---------------------------------------------------------------------------
# Named numbers that are not procedure parameters
# ---------------------------------------------------------------------------
MIN_ROI_PIXELS = 100
"""A pose whose region of interest has fewer pixels than this is reported with a note and no statistics."""
MIN_STATION_COUNT_FOR_FIT = 2
"""Fewest stations for the Z fits of Step 9 (two give a power law and a model that is exactly determined)."""
MIN_STATION_COUNT_FOR_MODEL_DOF = 3
"""With fewer stations than this the two-parameter model has no residual degrees of freedom (noted)."""
SIGMA_ZERO_INITIAL_FRACTION = 0.1
"""Initial sigma_0 of the model fit as a fraction of the smallest sigma_t."""
POWER_LAW_NEAR_TWO_TOLERANCE = 0.5
"""The fitted exponent n counts as 'near 2' when |n - 2| is at most this."""
TRIANGULATION_EXPONENT = 2.0
"""Exponent of Z in the triangulation prediction sigma_t ~ sigma_d Z^2 / k (Step 9)."""
AUTOCORRELATION_MAX_LAG_PX = 16
"""Longest lag of the autocorrelation profiles (it is cut at half the ROI bounding box if that is shorter)."""
AUTOCORRELATION_MIN_PAIR_FRACTION = 0.25
"""A lag is used only if at least this fraction of the zero-lag pair count contributes (masked border effects)."""
MIN_FRAMES_FOR_STATISTICS = 2
"""Fewest frames for a per-pixel standard deviation."""
LEGACY_MIN_VALID_FRACTION = 0.8
"""A pixel enters the legacy statistics only if it was read in this fraction of the frames (the 6DOF module's
``min_valid_fraction_per_pixel``)."""
LEGACY_EFFECTIVE_WIDTH_PX = 160
"""Width of the sensor's physically effective depth image in the 6DOF repository (640 / 4); the effective-
resolution block is the image width divided by this."""
LEGACY_PERCENTILES = (5, 25, 50, 75, 95)
"""Percentiles of the per-pixel standard deviation reported by the legacy module."""
QUANTUM_CANDIDATE_RATIO = 1.0025
"""Ratio of successive candidate quanta of the phase-resultant search (0.25 percent steps, fine enough that a
spread of a few quanta of populated levels stays in phase)."""
QUANTUM_MIN_CANDIDATE_LSB = 1.5
"""Smallest candidate quantum, in output LSBs (anything smaller is the LSB itself)."""
QUANTUM_MAX_CANDIDATE_MM = 50.0
"""Largest candidate quantum, mm (also limited by the span of the populated depths)."""
QUANTUM_CONCENTRATION_THRESHOLD = 0.9
"""Mean resultant length a candidate must reach to explain the depths (the 6DOF module uses 0.999 for its
exactly uniform millimeter quantum; the spacing of a disparity quantizer varies with depth, so this is relaxed)."""
QUANTUM_REFINE_LOWER_FRACTION = 0.85
QUANTUM_REFINE_UPPER_FRACTION = 1.03
"""The best candidate (highest resultant length) is taken between these fractions of the coarsest passing
candidate. The coarsest passing candidate sits on the upper shoulder of the resultant peak, because with only a
few populated levels a spacing error of several percent still keeps the phases together; the peak itself lies
below it. The lower fraction stays well above one half, so the sub-multiple q / 2 is never reached."""
QUANTUM_MIN_SAMPLES = 20
"""Fewest depth samples for a quantum estimate."""
QUANTUM_MIN_SPAN_QUANTA = 1.5
"""The populated depths must span at least this many candidate quanta, or the candidate is not testable."""
LEVEL_MIN_RELATIVE_COUNT = 0.01
"""A populated code level counts in the spacing histogram only if it holds this fraction of the most populated one."""
QUANTUM_AGREEMENT_TOLERANCE = 0.25
"""The two quantum estimators agree if they differ by at most this fraction (flagged otherwise)."""
QUANTUM_LSB_TOLERANCE = 1.25
"""A quantum within this factor of the output LSB is 'the LSB itself'."""
QUANTUM_CONSTANCY_TOLERANCE = 0.2
"""q = delta Z_q k / Z^2 counts as constant over Z if its relative spread is at most this."""
SENTINEL_MIN_FRAMES = 2
"""Fewest sentinel frames for a drift line."""
SECONDS_PER_HOUR = 3600.0
"""Seconds in an hour (the drift rate is in mm per hour)."""
MAP_PANEL_LIMIT = 12
"""Most panels of the sigma_t map figure."""
MAP_COLOR_PERCENTILE = 99.0
"""Upper color limit of the sigma_t maps, as a percentile of the map."""
FIT_CURVE_POINTS = 100
"""Points of the smooth model curves in the figures."""
HISTOGRAM_COLUMNS = 3
"""Panels per row in the grid figures."""
PANEL_WIDTH_IN, PANEL_HEIGHT_IN = 3.2, 2.8
"""Size of one panel of the grid figures, inches."""
INCIDENCE_MIN_COSINE = 0.05
"""Floor on cos(tilt) in the incidence exponent fit (same guard as the renderer)."""
MIN_TILT_ROWS_FOR_EXPONENT = 1
"""Fewest tilted poses for the incidence-noise exponent."""
RADIUS_MODEL_MAX_PIXELS = 20000
"""Pixels sampled per station for the legacy sigma(z, r) = a + b z^2 + c r fit."""
SAMPLING_SEED = 20261005
"""Seed of the pixel subsampling of the legacy radius model (deterministic output)."""
PLAUSIBLE_LEGACY_BOX_NOTE = "legacy box outside the image"
"""Note text for a skipped legacy box."""


# ---------------------------------------------------------------------------
# Options and result objects
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class NoiseOptions:
    """Choices of Analysis A that are not procedure parameters."""

    min_valid_fraction: float = 0.5
    """A pixel enters the temporal statistics and the mean frame only if it was read in at least this fraction of
    the frames of its pose."""
    sentinel_reference: str = "registered"
    """What the sentinel line (Step 11) is drawn through: 'registered' uses the per-frame ROI mean of Z - Z_GT, which
    cancels the robot's pose repeatability with the registered (read-back) pose; 'raw' uses the ROI mean of Z."""
    figure_width_in: float = 7.0
    """Width of the single-panel figures, inches."""
    figure_height_in: float = 4.5
    """Height of the single-panel figures, inches."""
    maximum_lag_px: int = AUTOCORRELATION_MAX_LAG_PX
    """Longest lag of the autocorrelation profiles, pixels."""


@dataclass
class StationNoise:
    """One pose of the A series: the Step 13 row and the quantities behind it."""

    pose_key: tuple
    station_z_mm: float
    field: int
    tilt_axis: str
    tilt_deg: float
    subseries: str
    frames: int
    field_fraction_achieved: float = float("nan")
    """Achieved fraction of the requested field offset (manifest column ``field_fraction_achieved``); NaN if absent."""
    roi_pixels: int = 0
    sigma_t_median_mm: float = float("nan")
    sigma_t_p95_mm: float = float("nan")
    sigma_t_rms_mm: float = float("nan")
    """Root of the ROI mean of the per-pixel variance, used in the closure check."""
    sigma_fp_mm: float = float("nan")
    bias_mm: float = float("nan")
    bias_corrected_mm: float = float("nan")
    bias_correction_mm: float = 0.0
    sigma_tot_mm: float = float("nan")
    closure_ratio: float = float("nan")
    closure_ok: bool | None = None
    fill_rate: float = float("nan")
    corr_len_h_px: float = float("nan")
    corr_len_v_px: float = float("nan")
    corr_len_fp_h_px: float = float("nan")
    corr_len_fp_v_px: float = float("nan")
    depth_quantum_mm: float = float("nan")
    quantum_code_spacing_mm: float = float("nan")
    quantum_is_lsb: bool | None = None
    q_px: float | None = None
    plane_angle_deg: float = float("nan")
    legacy_minmax_mm: float = float("nan")
    legacy_std_mm: float = float("nan")
    legacy_std_median_mm: float = float("nan")
    mean_time_hours: float = float("nan")
    note: str = ""

    def csv_row(self) -> dict[str, Any]:
        """The row of A_noise_summary.csv."""
        row = {name: getattr(self, name, None) for name in SUMMARY_COLUMNS}
        row["q_px"] = self.q_px
        return row


@dataclass
class PoseDiagnostics:
    """The arrays behind a row, kept for the figures (not written to the CSV)."""

    sigma_t_map: np.ndarray
    roi: np.ndarray
    acf_h: np.ndarray
    acf_v: np.ndarray
    acf_fp_h: np.ndarray
    acf_fp_v: np.ndarray
    code_levels: np.ndarray
    code_counts: np.ndarray
    lsb_mm: float
    legacy_boxes: list[dict[str, Any]] = field(default_factory=list)
    fixed_pattern_mm: np.ndarray | None = None
    """The fixed-pattern map of the pose (Step 4): the frame-mean depth minus the free plane fitted to it, mm, NaN outside
    the region of interest. Kept (float32) only for the fronto-parallel center-field main poses, the ones Analysis B-Z
    subtracts from its ramp poses (Section 11.2, Ramp); None for every other pose."""


@dataclass
class NoiseModelFit:
    """Step 9: the model fit and the power law over the center-field stations."""

    stations_mm: list[float]
    sigma_t_mm: list[float]
    sigma_0_mm: float
    sigma_d_px: float
    k_mm_px: float
    model_residual_rms_mm: float
    power_law_a: float
    power_law_n: float
    n_near_two: bool
    degrees_of_freedom: int
    note: str = ""

    def predict(self, z_mm) -> np.ndarray:
        """sigma_t(Z) = sqrt(sigma_0^2 + (sigma_d Z^2 / k)^2) in mm."""
        z = np.asarray(z_mm, dtype=np.float64)
        return np.sqrt(self.sigma_0_mm ** 2 + (self.sigma_d_px * z ** 2 / self.k_mm_px) ** 2)

    def power_law(self, z_mm) -> np.ndarray:
        """sigma_t = a Z^n in mm."""
        return self.power_law_a * np.asarray(z_mm, dtype=np.float64) ** self.power_law_n


@dataclass
class TargetDrift:
    """Step 11: the drift of one mounted target, from the sentinels captured on it during one mount, relative to the first
    sentinel after the mount (the reference of that mount)."""

    target_id: str
    mount: int
    """Mount number (0-based) in acquisition order; a target mounted again later has a new number."""
    gap_mm: float | None
    """Gap of the first sentinel of the mount (the front plane does not depend on it)."""
    sentinel_poses: int
    pose_hours: list[float]
    """Time of each sentinel pose, hours since the first A or sentinel frame."""
    pose_offset_mm: list[float]
    """Mean of Z - Z_GT (or of Z, per ``NoiseOptions.sentinel_reference``) of each sentinel pose minus that of the first one."""
    rate_mm_per_hour: float
    """Slope of the sentinel line over the mount; NaN with a single sentinel pose."""
    span_hours: float
    """Time from the first to the last sentinel pose of the mount."""
    drift_over_span_mm: float
    """|rate| x span: the drift of the target between its first and last sentinel of the mount."""
    used_for_a_correction: bool
    """True for the T2 sentinels of the mount of series A (the ones the A drift correction uses)."""


@dataclass
class DriftResult:
    """Step 11: the sentinel mean against time (and sensor temperature), the drift rate and the correction."""

    reference: str
    elapsed_hours: list[float]
    mean_z_mm: list[float]
    temperature_c: list[float | None]
    sentinel_ids: list[int]
    rate_mm_per_hour: float
    rate_raw_mm_per_hour: float
    rate_registered_mm_per_hour: float
    temperature_slope_mm_per_c: float | None
    session_hours: float
    drift_over_session_mm: float
    threshold_mm: float
    sentinel_station_mm: float
    correction_applied: bool
    pose_hours: list[float]
    pose_offset_mm: list[float]
    targets: list[TargetDrift] = field(default_factory=list)
    """The drift of every mounted target (T2 of A, T2 of B-Z, T3a, T3b, T4, T5 ...), each relative to its own first sentinel."""
    note: str = ""


@dataclass
class NoiseResult:
    """Everything Analysis A found."""

    rows: list[StationNoise]
    diagnostics: dict[tuple, PoseDiagnostics]
    model: NoiseModelFit | None
    drift: DriftResult | None
    quantization: dict[str, Any]
    legacy: dict[str, Any]
    tilt: list[dict[str, Any]]
    incidence_exponent: float | None
    radius_model: dict[str, Any] | None
    notes: list[str]
    k_mm_px: float
    lsb_mm: float
    options: NoiseOptions
    correlation_threshold: float = 1.0 / math.e
    """The autocorrelation threshold used (params.autocorrelation_threshold), for the figure."""

    def main_rows(self) -> list[StationNoise]:
        """The fronto-parallel center-field main stations, in order of Z."""
        rows = [r for r in self.rows if r.subseries == SUBSERIES_MAIN and r.field == 0 and r.tilt_deg == 0.0
                and math.isfinite(r.sigma_t_median_mm)]
        return sorted(rows, key=lambda r: r.station_z_mm)

    def forward_model_terms(self) -> dict[str, Any]:
        """The Tier-A simulator terms Analysis A measured (see sensorperf.analysis.forward_model). The incidence
        exponent is deliberately not included: it is only estimated roughly (see :func:`incidence_exponent`) and is
        written to A_noise_details.json instead."""
        main = self.main_rows()
        q_values = [r.q_px for r in main if r.q_px is not None]
        lengths_h = [r.corr_len_h_px for r in main if math.isfinite(r.corr_len_h_px)]
        lengths_v = [r.corr_len_v_px for r in main if math.isfinite(r.corr_len_v_px)]
        model = self.model
        sigma_d = None if model is None else model.sigma_d_px
        return {
            "sigma_d_px": sigma_d,
            "sigma_0_mm": None if model is None else model.sigma_0_mm,
            "q_px": float(np.median(q_values)) if q_values else None,
            "k_mm_px": self.k_mm_px,
            "corr_len_h_px": float(np.median(lengths_h)) if lengths_h else None,
            "corr_len_v_px": float(np.median(lengths_v)) if lengths_v else None,
            "power_law_a": None if model is None else model.power_law_a,
            "power_law_n": None if model is None else model.power_law_n,
            "noise_coefficient_per_mm": None if sigma_d is None else sigma_d / self.k_mm_px,
        }


# ---------------------------------------------------------------------------
# Step 7: autocorrelation
# ---------------------------------------------------------------------------
def masked_autocorrelation(fields: np.ndarray, mask: np.ndarray, max_lag: int) -> tuple[np.ndarray, np.ndarray]:
    """Normalized autocorrelation profiles along H (columns) and V (rows) of zero-mean fields inside a mask.

    ``fields`` is (F, H, W) or (H, W); NaN entries are excluded like pixels outside ``mask``. Each field is
    cropped to the bounding box of the mask, its mean inside the mask is subtracted, and the unnormalized
    correlation sum and the number of contributing pixel pairs are computed with the FFT (zero padded to twice
    the box, so no wrap-around). Both are summed over the fields; the autocorrelation is their ratio at each lag,
    divided by its zero-lag value. Returns (profile along H, profile along V), each of length max_lag + 1 with
    lag 0 first; lags with too few pairs are NaN."""
    fields = np.asarray(fields, dtype=np.float64)
    if fields.ndim == 2:
        fields = fields[None]
    rows = np.flatnonzero(mask.any(axis=1))
    cols = np.flatnonzero(mask.any(axis=0))
    r0, r1, c0, c1 = rows[0], rows[-1] + 1, cols[0], cols[-1] + 1
    box_mask = mask[r0:r1, c0:c1]
    shape = (2 * (r1 - r0), 2 * (c1 - c0))
    total_sum = np.zeros(shape)
    total_pairs = np.zeros(shape)
    for frame in fields:
        values = frame[r0:r1, c0:c1]
        usable = box_mask & np.isfinite(values)
        if usable.sum() < MIN_FRAMES_FOR_STATISTICS:
            continue
        centered = np.where(usable, values - values[usable].mean(), 0.0)
        weight = usable.astype(np.float64)
        total_sum += sp_fft.irfft2(np.abs(sp_fft.rfft2(centered, s=shape)) ** 2, s=shape)
        total_pairs += sp_fft.irfft2(np.abs(sp_fft.rfft2(weight, s=shape)) ** 2, s=shape)
    zero_pairs = total_pairs[0, 0]
    profiles = []
    for axis, extent in ((1, c1 - c0), (0, r1 - r0)):
        lags = min(max_lag, extent // 2)
        line_sum = total_sum[0, :lags + 1] if axis == 1 else total_sum[:lags + 1, 0]
        line_pairs = total_pairs[0, :lags + 1] if axis == 1 else total_pairs[:lags + 1, 0]
        with np.errstate(invalid="ignore", divide="ignore"):
            normalized = np.where(line_pairs >= AUTOCORRELATION_MIN_PAIR_FRACTION * zero_pairs,
                                  line_sum / np.maximum(line_pairs, 1.0), np.nan)
            normalized = normalized / normalized[0]
        full = np.full(max_lag + 1, np.nan)
        full[:lags + 1] = normalized
        profiles.append(full)
    return profiles[0], profiles[1]


def correlation_length_px(profile: np.ndarray, threshold: float) -> float:
    """First lag (pixels, linearly interpolated) at which a normalized autocorrelation profile (lag 0 first) falls
    to ``threshold``; NaN if it does not within the profile (Step 7)."""
    for lag in range(1, len(profile)):
        if not np.isfinite(profile[lag]):
            break
        if profile[lag] <= threshold:
            previous = profile[lag - 1]
            if previous == profile[lag]:
                return float(lag)
            return float(lag - 1 + (previous - threshold) / (previous - profile[lag]))
    return float("nan")


# ---------------------------------------------------------------------------
# Step 8: quantization
# ---------------------------------------------------------------------------
def estimate_quantum_phase_resultant(values_mm: np.ndarray, lsb_mm: float) -> tuple[float, float, bool]:
    """(quantum in mm, its mean resultant length, is_lsb) from depth values by the phase-resultant method.

    Method adapted from the 6DOF repository (sixdof/noise/repeatability.py, ``estimate_depth_quantum_mm``, by
    the same author): for a candidate quantum q the values are wrapped onto a circle of circumference q; the
    mean resultant length R(q) = |mean(exp(2 pi i value / q))| is near 1 when the values are multiples of q.
    Divisors of the true quantum have the same property, so the coarsest candidate with R above the threshold is
    taken, then refined to the best candidate between QUANTUM_REFINE_LOWER_FRACTION and QUANTUM_REFINE_UPPER_FRACTION of it. Candidates are a fine geometric
    grid between QUANTUM_MIN_CANDIDATE_LSB output LSBs and the span of the populated depths. If none passes, the
    depths are explained by the output LSB alone: the result is (lsb, 1, True). NaN when there are too few
    samples."""
    values = np.asarray(values_mm, dtype=np.float64).ravel()
    values = values[np.isfinite(values)]
    if values.size < QUANTUM_MIN_SAMPLES:
        return float("nan"), float("nan"), False
    # Unique depths with their counts (the output LSB makes the number of distinct values small).
    unique, counts = np.unique(np.round(values / lsb_mm * 1000.0) * lsb_mm / 1000.0, return_counts=True)
    span = float(unique[-1] - unique[0])
    low = QUANTUM_MIN_CANDIDATE_LSB * lsb_mm
    high = min(QUANTUM_MAX_CANDIDATE_MM, span / QUANTUM_MIN_SPAN_QUANTA)
    if high <= low:
        return lsb_mm, 1.0, True
    steps = int(math.ceil(math.log(high / low) / math.log(QUANTUM_CANDIDATE_RATIO))) + 1
    candidates = low * QUANTUM_CANDIDATE_RATIO ** np.arange(steps)
    weights = counts / counts.sum()
    angle = 2.0 * np.pi * unique[:, None] / candidates[None, :]
    resultant = np.hypot(weights @ np.cos(angle), weights @ np.sin(angle))
    passing = np.flatnonzero(resultant >= QUANTUM_CONCENTRATION_THRESHOLD)
    if passing.size == 0:
        return lsb_mm, 1.0, True
    coarsest = candidates[passing[-1]]
    near = np.flatnonzero((candidates >= QUANTUM_REFINE_LOWER_FRACTION * coarsest)
                          & (candidates <= QUANTUM_REFINE_UPPER_FRACTION * coarsest))
    best = near[int(np.argmax(resultant[near]))]
    return float(candidates[best]), float(resultant[best]), False


def code_spacing_quantum(codes: np.ndarray, lsb_mm: float) -> tuple[float, int, np.ndarray, np.ndarray]:
    """(quantum mm, modal spacing in codes, populated code levels, their counts): the procedure document's
    estimate, the most common spacing between populated depth-code levels. Levels holding less than
    LEVEL_MIN_RELATIVE_COUNT of the most populated one are ignored. The quantum is the mean of the spacings within
    one code of the mode (the output LSB rounds the true spacing to whole codes). NaN with fewer than two levels."""
    valid = codes[codes >= 0]
    if valid.size == 0:
        return float("nan"), 0, np.array([]), np.array([])
    levels, counts = np.unique(valid, return_counts=True)
    keep = counts >= LEVEL_MIN_RELATIVE_COUNT * counts.max()
    levels, counts = levels[keep], counts[keep]
    if levels.size < 2:
        return float("nan"), 0, levels, counts
    spacing = np.diff(levels)
    values, occurrences = np.unique(spacing, return_counts=True)
    mode = int(values[int(np.argmax(occurrences))])
    close = spacing[np.abs(spacing - mode) <= 1] if mode > 2 else spacing[spacing == mode]
    return float(np.mean(close)) * lsb_mm, mode, levels, counts


# ---------------------------------------------------------------------------
# Step 12: legacy metrics
# ---------------------------------------------------------------------------
def legacy_box_metrics(depth: np.ndarray, centers_px, half_px: int, effective_block_px: int) -> list[dict[str, Any]]:
    """Per legacy box the metrics of testZRepeatabilityBrownBoard.py as implemented by the 6DOF module (see the
    module docstring): mean and median over the box pixels of the per-pixel standard deviation over the frames
    (ddof = 0, frames in which the pixel was read), mean over the pixels of (max - min over frames), the
    percentiles of the standard deviation, and the pixel count. ``depth`` is (F, H, W) with NaN no-reads. A box
    outside the image, or one with no usable pixel, gets ``status`` saying so and NaN metrics."""
    import warnings
    frames, height, width = depth.shape
    out = []
    for center_u, center_v in centers_px:
        entry: dict[str, Any] = {"center_px": [int(center_u), int(center_v)], "status": "ok"}
        row0, row1, col0, col1 = center_v - half_px, center_v + half_px, center_u - half_px, center_u + half_px
        if row0 < 0 or col0 < 0 or row1 > height or col1 > width:
            entry["status"] = f"skipped: {PLAUSIBLE_LEGACY_BOX_NOTE} (image is {width} x {height} px)"
        else:
            patch = depth[:, row0:row1, col0:col1]
            valid_fraction = np.isfinite(patch).mean(axis=0)
            usable = valid_fraction >= LEGACY_MIN_VALID_FRACTION
            if not usable.any():
                entry["status"] = f"skipped: no pixel read in {LEGACY_MIN_VALID_FRACTION:.0%} of the frames"
            else:
                with np.errstate(invalid="ignore"), warnings.catch_warnings():
                    warnings.simplefilter("ignore", RuntimeWarning)     # all-NaN pixels are masked out by 'usable'
                    std = np.nanstd(patch, axis=0)[usable]
                    span = (np.nanmax(patch, axis=0) - np.nanmin(patch, axis=0))[usable]
                entry.update({
                    "pixels": int(usable.sum()), "std_mean_mm": float(std.mean()), "std_median_mm": float(np.median(std)),
                    "std_percentiles_mm": {str(p): float(np.percentile(std, p)) for p in LEGACY_PERCENTILES},
                    "minmax_mean_mm": float(span.mean())})
                if effective_block_px > 1:
                    block = effective_block_px
                    h2, w2 = patch.shape[1] // block * block, patch.shape[2] // block * block
                    with np.errstate(invalid="ignore"), warnings.catch_warnings():
                        warnings.simplefilter("ignore", RuntimeWarning)
                        blocks = np.nanmean(patch[:, :h2, :w2].reshape(frames, h2 // block, block, w2 // block, block),
                                            axis=(2, 4))
                        entry["effective_block_px"] = block
                        entry["effective_std_median_mm"] = float(np.nanmedian(np.nanstd(blocks, axis=0)))
        out.append(entry)
    return out


# ---------------------------------------------------------------------------
# One pose
# ---------------------------------------------------------------------------
def _angle_between_deg(a: np.ndarray, b: np.ndarray) -> float:
    """Angle in degrees between two plane normals (the sign of a normal does not matter)."""
    cosine = abs(float(np.dot(a, b)) / (np.linalg.norm(a) * np.linalg.norm(b)))
    return float(np.degrees(np.arccos(min(cosine, 1.0))))


def _parse_time(text: str) -> datetime | None:
    try:
        return datetime.fromisoformat(text)
    except ValueError:
        return None


def _metadata_float(record: FrameRecord, key: str) -> float:
    """The manifest metadata value ``key`` of the record as a float; NaN when absent or not a number."""
    try:
        return float(record.metadata[key])
    except (KeyError, ValueError):
        return float("nan")


def analyze_pose(session: Session, stack: PoseStack, options: NoiseOptions) -> tuple[StationNoise, PoseDiagnostics | None]:
    """Steps 1 to 8 and 12 for one pose; returns its row and the diagnostics arrays (None if the ROI is empty)."""
    params = session.params
    record = stack.record()
    row = StationNoise(pose_key=record.pose_key(), station_z_mm=record.station_z_mm, field=record.field,
                       tilt_axis=record.tilt_axis, tilt_deg=record.tilt_deg, subseries=record.subseries,
                       frames=stack.frame_count, field_fraction_achieved=_metadata_float(record, FIELD_FRACTION_ACHIEVED_KEY))
    camera = stack.camera
    # Steps 1-2: registered geometry, region of interest and ground truth.
    geometry = pose_geometry(session, record, camera)
    roi = region_of_interest(geometry, params)
    row.roi_pixels = int(roi.sum())
    if row.roi_pixels < MIN_ROI_PIXELS or stack.frame_count < MIN_FRAMES_FOR_STATISTICS:
        row.note = f"region of interest has {row.roi_pixels} pixels and the pose {stack.frame_count} frames: no statistics"
        return row, None
    z_gt = geometry.z_front_gt
    depth = stack.depth
    # Step 3: temporal noise map (ddof = 1) and its ROI summary.
    stats = temporal_statistics(depth, stack.valid, options.min_valid_fraction)
    sigma_map = np.sqrt(stats.variance)
    sigma_roi = sigma_map[roi & np.isfinite(sigma_map)]
    row.sigma_t_median_mm = float(np.median(sigma_roi))
    row.sigma_t_p95_mm = float(np.percentile(sigma_roi, 95.0))
    row.sigma_t_rms_mm = float(np.sqrt(np.mean(sigma_roi ** 2)))
    # Step 4: mean frame, free plane, fixed-pattern noise, bias, plane angle.
    mean_depth = stats.mean_depth
    usable = roi & np.isfinite(mean_depth)
    u, v = camera.pixel_grid()
    points = camera.back_project(u, v, mean_depth)
    fit = fit_plane_robust(points[usable])
    plane_z = plane_depth_image(camera, fit.point, fit.normal)
    residual = np.where(usable, mean_depth - plane_z, np.nan)
    row.sigma_fp_mm = float(np.nanstd(residual[usable]))
    row.bias_mm = float(np.mean((mean_depth - z_gt)[usable]))
    row.bias_corrected_mm = row.bias_mm
    registered_normal = geometry.target.front_plane_camera(geometry.pose_camera)[1]
    row.plane_angle_deg = _angle_between_deg(fit.normal, registered_normal)
    # Step 5: total noise and the closure check.
    deviation = np.where(roi[None] & stack.valid, depth - z_gt[None], np.nan)
    row.sigma_tot_mm = float(np.sqrt(np.nanmean(deviation ** 2)))
    expected = row.sigma_t_rms_mm ** 2 + row.sigma_fp_mm ** 2 + row.bias_mm ** 2
    row.closure_ratio = float(row.sigma_tot_mm ** 2 / expected) if expected > 0 else float("nan")
    row.closure_ok = bool(abs(row.closure_ratio - 1.0) <= params.noise_closure_tolerance)
    # Step 6: fill rate, averaged over the frames.
    row.fill_rate = float(stack.valid[:, roi].mean())
    # Step 7: spatial correlation of the temporal component and of the fixed-pattern residual.
    temporal_component = np.where(roi[None] & stack.valid, depth - mean_depth[None], np.nan)
    acf_h, acf_v = masked_autocorrelation(temporal_component, roi, options.maximum_lag_px)
    acf_fp_h, acf_fp_v = masked_autocorrelation(residual, usable, options.maximum_lag_px)
    threshold = params.autocorrelation_threshold
    row.corr_len_h_px = correlation_length_px(acf_h, threshold)
    row.corr_len_v_px = correlation_length_px(acf_v, threshold)
    row.corr_len_fp_h_px = correlation_length_px(acf_fp_h, threshold)
    row.corr_len_fp_v_px = correlation_length_px(acf_fp_v, threshold)
    notes = []
    if not (params.phase_jitter_span_px > max([x for x in (row.corr_len_h_px, row.corr_len_v_px) if math.isfinite(x)],
                                              default=0.0)):
        notes.append("PHASE_JITTER_SPAN_PX is not larger than the correlation length")
    # Step 8: depth quantum of the central patch (fronto-parallel poses only: on a tilt the patch spans many levels).
    lsb = float(session.geometry.require("depth_lsb_mm"))
    patch = central_patch_mask(camera, params.quantization_patch_px)
    patch_depth = depth[:, patch]
    codes = depth_codes(patch_depth[np.isfinite(patch_depth)], lsb)
    spacing_mm, _, levels, counts = code_spacing_quantum(codes, lsb)
    if record.tilt_deg == 0.0:
        phase_mm, _, is_lsb = estimate_quantum_phase_resultant(patch_depth, lsb)
        row.depth_quantum_mm = phase_mm
        row.quantum_code_spacing_mm = spacing_mm
        row.quantum_is_lsb = is_lsb
        if not is_lsb and math.isfinite(phase_mm):
            row.q_px = float(session.geometry.disparity_quantum_px(phase_mm, record.station_z_mm))
        if (math.isfinite(phase_mm) and math.isfinite(spacing_mm)
                and abs(phase_mm - spacing_mm) > QUANTUM_AGREEMENT_TOLERANCE * phase_mm):
            notes.append("quantum estimators disagree (phase resultant vs code spacing)")
    # Step 12: legacy boxes (all poses; the stations to report are chosen later).
    block = max(1, int(round(camera.width / LEGACY_EFFECTIVE_WIDTH_PX)))
    boxes = legacy_box_metrics(depth, params.legacy_box_centers_px, params.legacy_box_half_px, block)
    row.note = "; ".join(notes)
    is_center_main = (record.subseries == SUBSERIES_MAIN and record.field == FIELD_POSITION_CENTER
                      and record.tilt_deg == 0.0)
    diagnostics = PoseDiagnostics(sigma_t_map=sigma_map, roi=roi, acf_h=acf_h, acf_v=acf_v, acf_fp_h=acf_fp_h,
                                  acf_fp_v=acf_fp_v, code_levels=levels, code_counts=counts, lsb_mm=lsb,
                                  legacy_boxes=boxes,
                                  fixed_pattern_mm=residual.astype(np.float32) if is_center_main else None)
    stamps = [_parse_time(r.timestamp) for r in stack.records]
    stamps = [s for s in stamps if s is not None]
    if stamps:
        row.mean_time_hours = float(np.mean([s.timestamp() for s in stamps])) / SECONDS_PER_HOUR
    return row, diagnostics


# ---------------------------------------------------------------------------
# Step 9: model fit
# ---------------------------------------------------------------------------
def fit_noise_model(stations_mm: np.ndarray, sigma_t_mm: np.ndarray, k_mm_px: float) -> NoiseModelFit | None:
    """Weighted least squares fit of sigma_t(Z) = sqrt(sigma_0^2 + (sigma_d Z^2 / k)^2) (weights 1 / sigma_t, so the
    residuals are relative) and the free power law sigma_t = a Z^n (regression of ln sigma_t on ln Z). sigma_0 and
    sigma_d are constrained to be non-negative. None with fewer than MIN_STATION_COUNT_FOR_FIT stations."""
    z = np.asarray(stations_mm, dtype=np.float64)
    sigma = np.asarray(sigma_t_mm, dtype=np.float64)
    if z.size < MIN_STATION_COUNT_FOR_FIT or np.ptp(z) == 0.0:
        return None
    weights = 1.0 / sigma

    def residuals(theta: np.ndarray) -> np.ndarray:
        sigma_0, sigma_d = theta
        return weights * (np.sqrt(sigma_0 ** 2 + (sigma_d * z ** 2 / k_mm_px) ** 2) - sigma)

    top = int(np.argmax(z))
    start = np.array([SIGMA_ZERO_INITIAL_FRACTION * sigma.min(), sigma[top] * k_mm_px / z[top] ** 2])
    solution = least_squares(residuals, start, bounds=([0.0, 0.0], [np.inf, np.inf]))
    sigma_0, sigma_d = (float(x) for x in solution.x)
    slope, intercept = np.polyfit(np.log(z), np.log(sigma), 1)
    rms = float(np.sqrt(np.mean((np.sqrt(sigma_0 ** 2 + (sigma_d * z ** 2 / k_mm_px) ** 2) - sigma) ** 2)))
    note = "" if z.size >= MIN_STATION_COUNT_FOR_MODEL_DOF else "two stations: the model is exactly determined"
    return NoiseModelFit(stations_mm=z.tolist(), sigma_t_mm=sigma.tolist(), sigma_0_mm=sigma_0, sigma_d_px=sigma_d,
                         k_mm_px=k_mm_px, model_residual_rms_mm=rms, power_law_a=float(math.exp(intercept)),
                         power_law_n=float(slope), n_near_two=bool(abs(slope - TRIANGULATION_EXPONENT)
                                                                      <= POWER_LAW_NEAR_TWO_TOLERANCE),
                         degrees_of_freedom=int(z.size - 2), note=note)


# ---------------------------------------------------------------------------
# Step 11: sentinels
# ---------------------------------------------------------------------------
def mount_epochs(records: Sequence[FrameRecord]) -> dict[tuple, int]:
    """The mount number of every pose of the manifest, by pose key. A mount is a change of ``target_id`` from one
    capture that is not a sentinel to the next (registration poses count: they use T2); a sentinel belongs to the mount
    it is captured in, because a sentinel is captured on the target mounted at that point of the plan
    (``acquisition.plan.insert_sentinels``). The records must be in acquisition order, which both manifest writers
    (``simulate.session`` and ``acquisition.pose_log``) guarantee. Mount 0 is the first.

    When the manifest has the column ``sentinel_mount_reference`` (the planner's flag of the first sentinel after each
    mount), it adds what the target ids alone cannot show: a sentinel flagged as a reference in a mount that already has
    its reference sentinel starts a new mount (the same target mounted again). Without the column, or in a manifest
    where every flagged sentinel is the first of its mount, the epochs are those of the target changes alone."""
    epochs: dict[tuple, int] = {}
    mounted: str | None = None
    epoch = 0
    referenced_epoch: int | None = None          # the mount whose reference sentinel has been seen
    for record in records:
        if record.procedure != PROCEDURE_SENTINEL:
            if mounted is not None and record.target_id != mounted:
                epoch += 1
            mounted = record.target_id
        elif parse_flag(record.metadata.get(SENTINEL_MOUNT_REFERENCE_KEY)):
            if referenced_epoch == epoch and record.pose_key() not in epochs:
                epoch += 1                       # a second reference in one mount: the target was mounted again
            referenced_epoch = epoch
        epochs.setdefault(record.pose_key(), epoch)
    return epochs


def a_mount_epochs(records: Sequence[FrameRecord], epochs: dict[tuple, int]) -> set[int]:
    """The mounts in which series A was captured (the T2 mount of the plan; the drift correction of A is taken from the
    T2 sentinels of these mounts)."""
    return {epochs[r.pose_key()] for r in records if r.procedure == PROCEDURE_NOISE}


def _frame_means(session: Session, records: list[FrameRecord], options: NoiseOptions,
                 epochs: dict[tuple, int]) -> list[dict[str, Any]]:
    """Per frame of the sentinel poses: the mounted target, its gap and mount number, time (s since epoch),
    temperature, ROI mean of Z - Z_GT and of Z."""
    out = []
    for pose_id, (key, group) in enumerate(group_by_pose(records).items()):
        stack = load_stack(group)
        geometry = pose_geometry(session, stack.record(), stack.camera)
        roi = region_of_interest(geometry, session.params)
        for frame_index, frame_record in enumerate(stack.records):
            frame = stack.depth[frame_index]
            ok = roi & np.isfinite(frame)
            if not ok.any():
                continue
            stamp = _parse_time(frame_record.timestamp)
            out.append({"pose_id": pose_id, "target_id": frame_record.target_id, "gap_mm": frame_record.gap_mm,
                        "epoch": epochs.get(key, 0),
                        "reference": parse_flag(frame_record.metadata.get(SENTINEL_MOUNT_REFERENCE_KEY)),
                        "time_s": None if stamp is None else stamp.timestamp(),
                        "temperature_c": frame_record.sensor_temp_c,
                        "registered_mm": float(np.mean((frame - geometry.z_front_gt)[ok])),
                        "raw_mm": float(np.mean(frame[ok])), "station_z_mm": frame_record.station_z_mm})
    return out


def _pose_means(hours: np.ndarray, values: np.ndarray, pose_ids: np.ndarray,
                reference_pose_id: int | None = None) -> tuple[list[float], list[float]]:
    """(time in hours, mean value) of every sentinel pose, in order of time, with the values relative to the reference
    pose: the one the manifest flags as the reference of the mount (``reference_pose_id``) or, when none is flagged, the
    first pose in time, the first sentinel after mounting."""
    ids = sorted(set(pose_ids.tolist()))
    pose_hours = [float(np.mean(hours[pose_ids == p])) for p in ids]
    pose_values = [float(np.mean(values[pose_ids == p])) for p in ids]
    order = np.argsort(pose_hours)
    reference_value = pose_values[order[0]] if reference_pose_id not in ids else pose_values[ids.index(reference_pose_id)]
    return [pose_hours[i] for i in order], [pose_values[i] - reference_value for i in order]


def _line_rate(hours: np.ndarray, values: np.ndarray, pose_ids: np.ndarray) -> float:
    """Slope of the least-squares line of values against hours (mm per hour). NaN with a single sentinel pose (only the
    frame noise would set the slope: the reference of a mount alone has no drift) or without a spread in time."""
    if len(set(pose_ids.tolist())) < 2 or not np.ptp(hours) > 0:
        return float("nan")
    return float(np.polyfit(hours, values, 1)[0])


def target_drifts(frames: list[dict[str, Any]], options: NoiseOptions, time_origin_s: float,
                  correction_epochs: set[int]) -> list[TargetDrift]:
    """The drift of every mounted target (Step 11): the sentinel frames are grouped by (target, mount), and each group's
    drift is the sentinel line relative to the reference of that mount, so that a re-mount of the target, whose
    repeatability is ADAPTER_REMOUNT_REPEATABILITY_MM, does not enter. The reference is the sentinel that the manifest
    column ``sentinel_mount_reference`` flags when the manifest has it, and the first sentinel after the mount otherwise.
    A group with one sentinel pose has only its reference (offset 0, no rate). ``correction_epochs`` are the mounts of
    series A; the T2 groups among them are the ones the A drift correction uses."""
    groups: dict[tuple, list[dict[str, Any]]] = {}
    for f in frames:
        groups.setdefault((f["epoch"], f["target_id"]), []).append(f)
    result = []
    for (epoch, target_id), members in sorted(groups.items(), key=lambda item: min(f["time_s"] for f in item[1])):
        hours = np.array([(f["time_s"] - time_origin_s) / SECONDS_PER_HOUR for f in members])
        key = "registered_mm" if options.sentinel_reference == "registered" else "raw_mm"
        values = np.array([f[key] for f in members])
        # The reference pose is the sentinel the manifest flags as such; without the flag it is the first one in time.
        flagged = [f["pose_id"] for f in members if f.get("reference")]
        pose_hours, pose_offsets = _pose_means(hours, values, np.array([f["pose_id"] for f in members]),
                                               min(flagged) if flagged else None)
        rate = _line_rate(hours, values, np.array([f["pose_id"] for f in members]))
        span_hours = float(np.ptp(pose_hours)) if len(pose_hours) > 1 else 0.0
        result.append(TargetDrift(
            target_id=target_id, mount=epoch, gap_mm=members[0]["gap_mm"], sentinel_poses=len(pose_hours),
            pose_hours=pose_hours, pose_offset_mm=pose_offsets, rate_mm_per_hour=rate, span_hours=span_hours,
            drift_over_span_mm=abs(rate) * span_hours if math.isfinite(rate) else float("nan"),
            used_for_a_correction=(target_id == TARGET_NOISE_PLATE and epoch in correction_epochs)))
    return result


def analyze_sentinels(session: Session, rows: list[StationNoise], model: NoiseModelFit | None,
                      session_hours: float, options: NoiseOptions, time_origin_s: float,
                      epochs: dict[tuple, int] | None = None) -> DriftResult | None:
    """Step 11: the sentinel lines per mounted target, the drift of A against the allowance, and the bias correction.

    The sentinels are captured on the target mounted at that point of the plan, so they are grouped by (target, mount)
    and each group's drift is computed relative to its first sentinel after mounting (:func:`target_drifts`, all of them
    returned in ``DriftResult.targets``). The A drift correction uses the T2 sentinels of the mount(s) in which series
    A was captured: their line gives the rate, the drift over ``session_hours`` is compared with the allowance
    WARMUP_DRIFT_FRACTION_OF_SIGMA x sigma_t at the sentinel Z, and, when it exceeds it, the offsets of the sentinel poses
    (relative to the first one) interpolated in time are subtracted from the bias of every A pose."""
    params = session.params
    records = select(session.records, procedure=PROCEDURE_SENTINEL)
    if not records:
        return None
    epochs = mount_epochs(session.records) if epochs is None else epochs
    correction_epochs = a_mount_epochs(session.records, epochs)
    all_frames = [f for f in _frame_means(session, records, options, epochs) if f["time_s"] is not None]
    per_target = target_drifts(all_frames, options, time_origin_s, correction_epochs)
    frames = [f for f in all_frames if f["target_id"] == TARGET_NOISE_PLATE and f["epoch"] in correction_epochs]
    if len(frames) < SENTINEL_MIN_FRAMES:
        return None
    hours = np.array([(f["time_s"] - time_origin_s) / SECONDS_PER_HOUR for f in frames])
    registered = np.array([f["registered_mm"] for f in frames])
    raw = np.array([f["raw_mm"] for f in frames])
    chosen = registered if options.sentinel_reference == "registered" else raw
    frame_poses = np.array([f["pose_id"] for f in frames])
    rate_registered = _line_rate(hours, registered, frame_poses)
    rate_raw = _line_rate(hours, raw, frame_poses)
    rate = rate_registered if options.sentinel_reference == "registered" else rate_raw
    temperatures = [f["temperature_c"] for f in frames]
    slope_temperature = None
    known = np.array([t is not None for t in temperatures])
    if known.sum() >= SENTINEL_MIN_FRAMES and np.ptp(np.array([t for t in temperatures if t is not None])) > 0:
        slope_temperature = float(np.polyfit(np.array([t for t in temperatures if t is not None]), chosen[known], 1)[0])
    sentinel_z = float(frames[0]["station_z_mm"])
    sigma_here = (float(model.predict(sentinel_z)) if model is not None else
                  float(np.median([r.sigma_t_median_mm for r in rows if math.isfinite(r.sigma_t_median_mm)] or [np.nan])))
    drift_over_session = abs(rate) * session_hours if math.isfinite(rate) else float("nan")
    threshold = params.warmup_drift_fraction_of_sigma * sigma_here
    apply = bool(math.isfinite(drift_over_session) and drift_over_session > threshold)
    # Per-sentinel-pose offsets relative to the first sentinel after the mount, for the time interpolation.
    pose_hours, pose_offsets = _pose_means(hours, chosen, frame_poses)
    if apply:
        for row in rows:
            if math.isfinite(row.mean_time_hours):
                t = row.mean_time_hours - time_origin_s / SECONDS_PER_HOUR
                row.bias_correction_mm = float(np.interp(t, pose_hours, pose_offsets))
                row.bias_corrected_mm = row.bias_mm - row.bias_correction_mm
    return DriftResult(
        reference=options.sentinel_reference, elapsed_hours=hours.tolist(), mean_z_mm=chosen.tolist(),
        temperature_c=temperatures, sentinel_ids=[f["pose_id"] for f in frames], rate_mm_per_hour=rate,
        rate_raw_mm_per_hour=rate_raw, rate_registered_mm_per_hour=rate_registered,
        temperature_slope_mm_per_c=slope_temperature, session_hours=session_hours,
        drift_over_session_mm=drift_over_session, threshold_mm=threshold, sentinel_station_mm=sentinel_z,
        correction_applied=apply, pose_hours=pose_hours, pose_offset_mm=pose_offsets, targets=per_target,
        note=("sensor temperature not logged: no drift against temperature" if slope_temperature is None else ""))


# ---------------------------------------------------------------------------
# Step 10 and the incidence exponent
# ---------------------------------------------------------------------------
def tilt_curves(rows: list[StationNoise]) -> list[dict[str, Any]]:
    """Step 10: for every (tilt axis, Z) of the tilt sub-series the sigma_t, sigma_tot and fill rate against the
    incidence angle (the tilt), with the fronto-parallel main station at that Z as the 0 degree point."""
    curves = []
    tilt_rows = [r for r in rows if r.subseries == SUBSERIES_TILT and math.isfinite(r.sigma_t_median_mm)]
    for axis in (TILT_AXIS_H, TILT_AXIS_V):
        for z in sorted({r.station_z_mm for r in tilt_rows if r.tilt_axis == axis}):
            points = [r for r in tilt_rows if r.tilt_axis == axis and r.station_z_mm == z]
            base = [r for r in rows if r.subseries == SUBSERIES_MAIN and r.field == 0 and r.tilt_deg == 0.0
                    and r.station_z_mm == z and math.isfinite(r.sigma_t_median_mm)]
            points = sorted(base[:1] + points, key=lambda r: r.tilt_deg)
            curves.append({"tilt_axis": axis, "station_z_mm": z, "tilt_deg": [r.tilt_deg for r in points],
                           "sigma_t_median_mm": [r.sigma_t_median_mm for r in points],
                           "sigma_tot_mm": [r.sigma_tot_mm for r in points], "fill_rate": [r.fill_rate for r in points]})
    return curves


def incidence_exponent(curves: list[dict[str, Any]]) -> float | None:
    """Exponent m of sigma_t(theta) = sigma_t(0) cos(theta)^(-m): the least-squares slope through the origin of
    ln(sigma_t(theta) / sigma_t(0)) against -ln cos(theta), over the tilted points of the curves. The ROI-mean
    incidence angle differs a little from the tilt (field angle), so this is an estimate of the renderer's
    parameter, not an exact inversion. None without a tilted pose."""
    numerator = denominator = 0.0
    count = 0
    for curve in curves:
        angles = np.radians(curve["tilt_deg"])
        sigma = np.array(curve["sigma_t_median_mm"])
        base = [s for a, s in zip(curve["tilt_deg"], sigma) if a == 0.0]
        if not base:
            continue
        for angle, s in zip(angles, sigma):
            if angle == 0.0:
                continue
            x = -math.log(max(math.cos(angle), INCIDENCE_MIN_COSINE))
            numerator += x * math.log(s / base[0])
            denominator += x * x
            count += 1
    return numerator / denominator if count >= MIN_TILT_ROWS_FOR_EXPONENT and denominator > 0 else None


def fit_radius_model(session: Session, main_rows: list[StationNoise], diagnostics: dict[tuple, PoseDiagnostics],
                     camera_by_key: dict[tuple, Any]) -> dict[str, Any] | None:
    """The 6DOF repository's sigma(z, r) = a + b z^2 + c r model (r = pixel radius from the principal point) by
    linear least squares over a deterministic subsample of the per-pixel sigma_t of the main stations. For
    comparison with that repository's fits only; Step 9's model is the one handed to the forward model."""
    rng = np.random.default_rng(SAMPLING_SEED)
    z_all, r_all, s_all = [], [], []
    for row in main_rows:
        diag, camera = diagnostics.get(row.pose_key), camera_by_key.get(row.pose_key)
        if diag is None or camera is None:
            continue
        u, v = camera.pixel_grid()
        radius = np.hypot(u - camera.principal_x_px, v - camera.principal_y_px)
        selected = np.flatnonzero((diag.roi & np.isfinite(diag.sigma_t_map)).ravel())
        if selected.size > RADIUS_MODEL_MAX_PIXELS:
            selected = rng.choice(selected, RADIUS_MODEL_MAX_PIXELS, replace=False)
        z_all.append(np.full(selected.size, row.station_z_mm))
        r_all.append(radius.ravel()[selected])
        s_all.append(diag.sigma_t_map.ravel()[selected])
    if not z_all:
        return None
    z, r, s = np.concatenate(z_all), np.concatenate(r_all), np.concatenate(s_all)
    design = np.column_stack([np.ones_like(z), z ** 2, r])
    (a, b, c), *_ = np.linalg.lstsq(design, s, rcond=None)
    return {"a_mm": float(a), "b_per_mm": float(b), "c_mm_per_px": float(c),
            "model": "sigma(z, r) = a + b z^2 + c r (sixdof/noise/repeatability.py)"}


# ---------------------------------------------------------------------------
# The analysis
# ---------------------------------------------------------------------------
def run_noise(session: Session, out_dir: str | Path, previous: dict | None = None,
              options: NoiseOptions | None = None) -> NoiseResult | None:
    """Analysis A, Steps 1 to 13 (module docstring). Returns None when the manifest has no procedure A frames.
    ``out_dir`` is where write_outputs will write (nothing is written here); ``previous`` holds earlier results
    and is not used by Analysis A."""
    options = NoiseOptions() if options is None else options
    a_records = select(session.records, procedure=PROCEDURE_NOISE)
    if not a_records:
        return None
    params = session.params
    k = session.geometry.disparity_constant_mm_px()
    lsb = float(session.geometry.require("depth_lsb_mm"))
    rows: list[StationNoise] = []
    diagnostics: dict[tuple, PoseDiagnostics] = {}
    cameras: dict[tuple, Any] = {}
    for key, group in group_by_pose(a_records).items():
        stack = load_stack(group)
        row, diag = analyze_pose(session, stack, options)
        rows.append(row)
        cameras[key] = stack.camera
        if diag is not None:
            diagnostics[key] = diag
    notes: list[str] = []
    # Step 12: attach the legacy metrics to the stations nearest the requested depths.
    centers = [r for r in rows if r.subseries == SUBSERIES_MAIN and r.field == 0 and r.tilt_deg == 0.0
               and r.pose_key in diagnostics]
    legacy: dict[str, Any] = {"depths_mm": list(params.legacy_metric_depths_mm), "stations": []}
    for wanted in params.legacy_metric_depths_mm:
        if not centers:
            break
        nearest = min(centers, key=lambda r: abs(r.station_z_mm - wanted))
        boxes = diagnostics[nearest.pose_key].legacy_boxes
        good = [b for b in boxes if b["status"] == "ok"]
        if good:
            nearest.legacy_minmax_mm = float(np.mean([b["minmax_mean_mm"] for b in good]))
            nearest.legacy_std_mm = float(np.mean([b["std_mean_mm"] for b in good]))
            nearest.legacy_std_median_mm = float(np.median([b["std_median_mm"] for b in good]))
        else:
            notes.append(f"legacy metrics at {wanted:g} mm (station {nearest.station_z_mm:g} mm): no box usable")
        for b in boxes:
            if b["status"] != "ok":
                notes.append(f"legacy box {b['center_px']} at station {nearest.station_z_mm:g} mm {b['status']}")
        legacy["stations"].append({"requested_mm": wanted, "station_z_mm": nearest.station_z_mm, "boxes": boxes})
    # Step 9: the noise model over the center-field main stations.
    main = sorted([r for r in rows if r.subseries == SUBSERIES_MAIN and r.field == 0 and r.tilt_deg == 0.0
                   and math.isfinite(r.sigma_t_median_mm)], key=lambda r: r.station_z_mm)
    model = fit_noise_model(np.array([r.station_z_mm for r in main]), np.array([r.sigma_t_median_mm for r in main]), k)
    if model is None:
        notes.append("fewer than two main stations: no noise model (sigma_d, sigma_0) and no power law")
    # Quantization summary (Step 8).
    quanta = [(r.station_z_mm, r.depth_quantum_mm, r.quantum_is_lsb, r.q_px) for r in main]
    all_lsb = bool(quanta) and all(item[2] for item in quanta)
    q_values = [q for *_, q in quanta if q is not None]
    quantization = {
        "output_lsb_mm": lsb,
        "per_station": [{"station_z_mm": z, "depth_quantum_mm": d, "is_lsb": lsb_flag, "q_px": q}
                        for z, d, lsb_flag, q in quanta],
        "output_lsb_is_the_quantizer": all_lsb,
        "q_constant": (bool(np.std(q_values) / np.mean(q_values) <= QUANTUM_CONSTANCY_TOLERANCE)
                       if len(q_values) >= 2 else None),
        "note": ("output LSB is the quantizer: disparity quantization cannot be seen here; use the B-Z staircase"
                 if all_lsb else
                 "quantum from the phase-resultant method; the code-spacing histogram is the cross-check "
                 "(quantum_code_spacing_mm)")}
    # Time origin and span: all A frames and the T2 sentinels of the mount of A (those the A correction uses).
    epochs = mount_epochs(session.records)
    a_epochs = a_mount_epochs(session.records, epochs)
    a_sentinels = [r for r in select(session.records, procedure=PROCEDURE_SENTINEL, target_id=TARGET_NOISE_PLATE)
                   if epochs[r.pose_key()] in a_epochs]
    stamps = [s for s in (_parse_time(r.timestamp) for r in a_records + a_sentinels) if s is not None]
    origin_s = min(s.timestamp() for s in stamps) if stamps else 0.0
    session_hours = (max(s.timestamp() for s in stamps) - origin_s) / SECONDS_PER_HOUR if stamps else 0.0
    drift = analyze_sentinels(session, rows, model, session_hours, options, origin_s, epochs) if stamps else None
    if drift is None:
        notes.append("no usable T2 sentinel frames in the mount of series A: no drift line, no bias correction")
    curves = tilt_curves(rows)
    return NoiseResult(
        rows=rows, diagnostics=diagnostics, model=model, drift=drift, quantization=quantization, legacy=legacy,
        tilt=curves, incidence_exponent=incidence_exponent(curves),
        radius_model=fit_radius_model(session, main, diagnostics, cameras), notes=notes, k_mm_px=k, lsb_mm=lsb,
        options=options, correlation_threshold=params.autocorrelation_threshold)


# ---------------------------------------------------------------------------
# Outputs
# ---------------------------------------------------------------------------
def _grid_figure(panels: int, columns: int = HISTOGRAM_COLUMNS):
    """A figure with a grid of axes for ``panels`` panels (Agg backend through common.new_figure)."""
    rows = max(1, -(-panels // columns))
    columns = min(columns, max(panels, 1))
    figure, axis = new_figure(PANEL_WIDTH_IN * columns, PANEL_HEIGHT_IN * rows)
    figure.delaxes(axis)
    figure.set_layout_engine("constrained")        # keeps titles, labels and colorbars from overlapping
    axes = figure.subplots(rows, columns, squeeze=False)
    for extra in axes.ravel()[panels:]:
        extra.set_visible(False)
    return figure, axes.ravel()[:panels]


def _figure_sigma(result: NoiseResult, out_dir: Path) -> list[Path]:
    """The three sigma curves against Z with the model and the power law (Step 13)."""
    figure, axis = new_figure(result.options.figure_width_in, result.options.figure_height_in)
    main = result.main_rows()
    z = np.array([r.station_z_mm for r in main])
    for label, values, color, marker in (
            ("sigma_t (median)", [r.sigma_t_median_mm for r in main], OKABE_ITO_BLUE, "o"),
            ("sigma_fp", [r.sigma_fp_mm for r in main], OKABE_ITO_BLUISH_GREEN, "s"),
            ("sigma_tot", [r.sigma_tot_mm for r in main], OKABE_ITO_VERMILLION, "^")):
        axis.plot(z, values, marker=marker, color=color, linestyle="none", label=label)
    if result.model is not None and z.size:
        grid = np.linspace(z.min(), z.max(), FIT_CURVE_POINTS)
        model = result.model
        axis.plot(grid, model.predict(grid), color=OKABE_ITO_BLUE,
                  label=f"model: sigma_d = {model.sigma_d_px:.3g} px, sigma_0 = {model.sigma_0_mm:.3g} mm")
        axis.plot(grid, model.power_law(grid), color=OKABE_ITO_ORANGE, linestyle="--",
                  label=f"power law: n = {model.power_law_n:.2f}")
    axis.set_xscale("log")
    axis.set_yscale("log")
    axis.set_xlabel("station depth Z (mm)")
    axis.set_ylabel("noise (mm)")
    axis.set_title("Analysis A: noise versus Z")
    axis.grid(True, which="both", alpha=0.3)
    axis.legend(fontsize="small")
    return save_figure(figure, out_dir / FIGURE_STEMS["sigma"])


def _figure_fill(result: NoiseResult, out_dir: Path) -> list[Path]:
    figure, axis = new_figure(result.options.figure_width_in, result.options.figure_height_in)
    main = result.main_rows()
    axis.plot([r.station_z_mm for r in main], [r.fill_rate for r in main], marker="o", color=OKABE_ITO_BLUE)
    axis.set_xlabel("station depth Z (mm)")
    axis.set_ylabel("fill rate (fraction of ROI pixels read)")
    axis.set_ylim(0.0, 1.05)
    axis.set_title("Analysis A: fill rate versus Z")
    axis.grid(True, alpha=0.3)
    return save_figure(figure, out_dir / FIGURE_STEMS["fill"])


def _figure_maps(result: NoiseResult, out_dir: Path) -> list[Path]:
    rows = [r for r in result.rows if r.pose_key in result.diagnostics and r.tilt_deg == 0.0][:MAP_PANEL_LIMIT]
    figure, axes = _grid_figure(max(len(rows), 1))
    image = None
    for axis, row in zip(axes, rows):
        diag = result.diagnostics[row.pose_key]
        shown = np.where(diag.roi, diag.sigma_t_map, np.nan)
        top = np.nanpercentile(shown, MAP_COLOR_PERCENTILE)
        image = axis.imshow(shown, cmap="viridis", vmin=0.0, vmax=top)
        axis.set_title(f"Z = {row.station_z_mm:g} mm, field {row.field}", fontsize="small")
        axis.set_xlabel("u (px)")
        axis.set_ylabel("v (px)")
    if image is not None:
        figure.colorbar(image, ax=list(axes), label="sigma_t (mm)", shrink=0.8)
    return save_figure(figure, out_dir / FIGURE_STEMS["maps"])


def _figure_autocorrelation(result: NoiseResult, out_dir: Path) -> list[Path]:
    figure, axes = _grid_figure(2, columns=2)
    main = result.main_rows()
    for panel, (axis, name) in enumerate(zip(axes, ("H (columns)", "V (rows)"))):
        for index, row in enumerate(main):
            diag = result.diagnostics[row.pose_key]
            color = OKABE_ITO_CYCLE[index % len(OKABE_ITO_CYCLE)]
            temporal = diag.acf_h if panel == 0 else diag.acf_v
            fixed = diag.acf_fp_h if panel == 0 else diag.acf_fp_v
            axis.plot(np.arange(len(temporal)), temporal, color=color, marker="o", markersize=3,
                      label=f"temporal, Z = {row.station_z_mm:g}")
            axis.plot(np.arange(len(fixed)), fixed, color=color, linestyle="--", label="fixed pattern" if index == 0 else None)
        axis.axhline(result.correlation_threshold, color=OKABE_ITO_BLACK, linewidth=0.8, linestyle=":")
        axis.set_xlabel(f"lag along {name} (px)")
        axis.set_ylabel("normalized autocorrelation")
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize="x-small")
    return save_figure(figure, out_dir / FIGURE_STEMS["autocorrelation"])


def _figure_codes(result: NoiseResult, out_dir: Path) -> list[Path]:
    main = result.main_rows()
    figure, axes = _grid_figure(max(len(main), 1))
    for axis, row in zip(axes, main):
        diag = result.diagnostics[row.pose_key]
        axis.vlines(diag.code_levels * diag.lsb_mm, 0, diag.code_counts, color=OKABE_ITO_BLUE)
        title = f"Z = {row.station_z_mm:g} mm"
        if math.isfinite(row.depth_quantum_mm):
            title += f", quantum {row.depth_quantum_mm:.3g} mm"
        axis.set_title(title, fontsize="small")
        axis.set_xlabel("depth (mm)")
        axis.set_ylabel("count")
    return save_figure(figure, out_dir / FIGURE_STEMS["codes"])


def _figure_tilt(result: NoiseResult, out_dir: Path) -> list[Path] | None:
    if not result.tilt:
        return None
    figure, axes = _grid_figure(3, columns=3)
    for index, curve in enumerate(result.tilt):
        color = OKABE_ITO_CYCLE[index % len(OKABE_ITO_CYCLE)]
        label = f"tilt about {curve['tilt_axis']}, Z = {curve['station_z_mm']:g} mm"
        for axis, key in zip(axes, ("sigma_t_median_mm", "sigma_tot_mm", "fill_rate")):
            axis.plot(curve["tilt_deg"], curve[key], marker="o", color=color, label=label)
    for axis, name in zip(axes, ("sigma_t (mm)", "sigma_tot (mm)", "fill rate")):
        axis.set_xlabel("incidence angle (tilt, degrees)")
        axis.set_ylabel(name)
        axis.grid(True, alpha=0.3)
    axes[0].legend(fontsize="x-small")
    return save_figure(figure, out_dir / FIGURE_STEMS["tilt"])


def _figure_drift(result: NoiseResult, out_dir: Path) -> list[Path] | None:
    drift = result.drift
    if drift is None:
        return None
    has_temperature = any(t is not None for t in drift.temperature_c)
    figure, axes = _grid_figure(3 if has_temperature else 2, columns=2)
    hours = np.array(drift.elapsed_hours)
    values = np.array(drift.mean_z_mm)
    axes[0].plot(hours, values, "o", color=OKABE_ITO_BLUE, markersize=3)
    if np.ptp(hours) > 0:
        line = np.polyfit(hours, values, 1)
        axes[0].plot(hours, np.polyval(line, hours), color=OKABE_ITO_VERMILLION,
                     label=f"{drift.rate_mm_per_hour:.3g} mm/h")
        axes[0].legend(fontsize="small")
    axes[0].set_xlabel("time since the first frame (h)")
    axes[0].set_ylabel("T2 sentinel mean Z - Z_GT (mm)" if drift.reference == "registered"
                       else "T2 sentinel mean Z (mm)")
    axes[0].set_title("T2 sentinels of A (the correction)", fontsize="small")
    axes[0].grid(True, alpha=0.3)
    # Every mounted target, each relative to its own first sentinel after the mount.
    for index, target in enumerate(drift.targets):
        axes[1].plot(target.pose_hours, target.pose_offset_mm, "o-", color=OKABE_ITO_CYCLE[index % len(OKABE_ITO_CYCLE)],
                     markersize=3, label=f"{target.target_id} (mount {target.mount})")
    axes[1].set_xlabel("time since the first frame (h)")
    axes[1].set_ylabel("sentinel offset from the first of the mount (mm)")
    axes[1].set_title("drift per mounted target", fontsize="small")
    axes[1].grid(True, alpha=0.3)
    if drift.targets:
        axes[1].legend(fontsize="x-small")
    if has_temperature:
        known = [(t, v) for t, v in zip(drift.temperature_c, values) if t is not None]
        axes[2].plot([k[0] for k in known], [k[1] for k in known], "o", color=OKABE_ITO_BLUISH_GREEN, markersize=3)
        axes[2].set_xlabel("sensor temperature (degrees C)")
        axes[2].set_ylabel("sentinel mean (mm)")
        axes[2].grid(True, alpha=0.3)
    return save_figure(figure, out_dir / FIGURE_STEMS["drift"])


def write_outputs(result: NoiseResult, out_dir: str | Path) -> list[Path]:
    """Step 13: A_noise_summary.csv (one row per pose), A_noise_details.json and the figures (PNG and SVG).
    Returns the paths written."""
    out_dir = Path(out_dir)
    written = [write_csv_rows(out_dir / SUMMARY_CSV_NAME, [r.csv_row() for r in result.rows], SUMMARY_COLUMNS)]
    model, drift = result.model, result.drift
    details = {
        "model_fit": None if model is None else {
            "form": "sigma_t(Z) = sqrt(sigma_0^2 + (sigma_d Z^2 / k)^2)", "sigma_0_mm": model.sigma_0_mm,
            "sigma_d_px": model.sigma_d_px, "k_mm_px": model.k_mm_px, "stations_mm": model.stations_mm,
            "sigma_t_mm": model.sigma_t_mm, "residual_rms_mm": model.model_residual_rms_mm,
            "power_law": {"a": model.power_law_a, "n": model.power_law_n, "n_near_two": model.n_near_two},
            "degrees_of_freedom": model.degrees_of_freedom, "note": model.note,
            "noise_coefficient_per_mm": model.sigma_d_px / model.k_mm_px},
        "drift": None if drift is None else {
            "reference": drift.reference, "rate_mm_per_hour": drift.rate_mm_per_hour,
            "rate_raw_mm_per_hour": drift.rate_raw_mm_per_hour,
            "rate_registered_mm_per_hour": drift.rate_registered_mm_per_hour,
            "temperature_slope_mm_per_c": drift.temperature_slope_mm_per_c, "session_hours": drift.session_hours,
            "drift_over_session_mm": drift.drift_over_session_mm, "threshold_mm": drift.threshold_mm,
            "sentinel_station_mm": drift.sentinel_station_mm, "correction_applied": drift.correction_applied,
            "sentinel_pose_hours": drift.pose_hours, "sentinel_pose_offset_mm": drift.pose_offset_mm,
            "targets": [{"target_id": t.target_id, "mount": t.mount, "gap_mm": t.gap_mm,
                         "sentinel_poses": t.sentinel_poses, "pose_hours": t.pose_hours,
                         "pose_offset_from_first_sentinel_mm": t.pose_offset_mm, "rate_mm_per_hour": t.rate_mm_per_hour,
                         "span_hours": t.span_hours, "drift_over_span_mm": t.drift_over_span_mm,
                         "used_for_a_correction": t.used_for_a_correction} for t in drift.targets],
            "correction_per_station_mm": {f"{r.station_z_mm:g}/{r.subseries}/{r.field}/{r.tilt_axis}{r.tilt_deg:g}":
                                          r.bias_correction_mm for r in result.rows}, "note": drift.note},
        "legacy": result.legacy,
        "quantization": result.quantization,
        "tilt_curves": result.tilt,
        "noise_incidence_exponent": result.incidence_exponent,
        "radius_model": result.radius_model,
        "autocorrelation_profiles": {
            f"{r.station_z_mm:g}/{r.subseries}/{r.field}/{r.tilt_axis}{r.tilt_deg:g}": {
                "temporal_h": result.diagnostics[r.pose_key].acf_h, "temporal_v": result.diagnostics[r.pose_key].acf_v,
                "fixed_pattern_h": result.diagnostics[r.pose_key].acf_fp_h,
                "fixed_pattern_v": result.diagnostics[r.pose_key].acf_fp_v}
            for r in result.rows if r.pose_key in result.diagnostics},
        "stations": [{k: v for k, v in r.__dict__.items() if k != "pose_key"} for r in result.rows],
        "forward_model_terms": result.forward_model_terms(),
        "notes": result.notes,
    }
    written.append(write_json(out_dir / DETAILS_JSON_NAME, details))
    for builder in (_figure_sigma, _figure_fill, _figure_maps, _figure_autocorrelation, _figure_codes, _figure_tilt,
                    _figure_drift):
        paths = builder(result, out_dir)
        if paths:
            written.extend(paths)
    return written
