"""
Analysis B-HV: effective lateral resolution in H and V (procedure document, Section 11.1), from the edge series
(procedure "B"): the raised square T3a and the square window T3b (opposite edge polarity), at each station and
gap, nominal pose plus the phase-jitter poses.

Per configuration (target, gap, station) and per square edge (left and right measure H, top and bottom measure V;
the square carries the slant, so every edge is slanted against the pixel grid):

  Step 1-2  reference planes Z_front, Z_back fitted to each pose's mean frame on pixels farther than
            BOUNDARY_BAND_HALF_WIDTH_PX from the edges (``common.reference_planes``, registered planes where too
            few pixels), and the normalized height h = (Z_back - Z) / (Z_back - Z_front) of every pixel of every
            frame (1 on the front surface, 0 on the back plate)
  Step 3    the signed distance s (px) of each pixel to the TRUE edge line, projected from the registered pose
            (``PoseGeometry.signed_distance_px``); positive on the front-material side
  Step 4    the robot-stepped ESF: all frames of the nominal and the jitter poses are pooled; only pixels near
            ONE edge are used (the distance to the other three edges must exceed a clearance, see
            :class:`LateralOptions`); h is binned by s at BOUNDARY_BIN_WIDTH_PX over +/- BOUNDARY_BAND_HALF_WIDTH_PX;
            ESF(s) = mean h of the read pixels in the bin; the number of no-reads and the total count per bin are
            kept (Analysis E uses them)
  Step 5    the slanted-edge ESF of the nominal pose's mean frame: per row (near-vertical edges) or column
            (near-horizontal edges) the half-height crossing of h, a line fitted to the crossings, h binned by
            the distance to that line; the alignment offset between the fitted line and the true edge and
            whether the two ESFs agree within ESF_AGREEMENT_BINS bins
  Step 6    the 10-90 percent rise distance (px, and mm via p(Z)), the LSF = dESF/ds (smoothed with a Hann
            window), the MTF = |FFT(LSF)| normalized at zero frequency and MTF50 in cycles per pixel
  Step 7    the linearity test: small-gap against large-gap ESF of the same edge, Z and polarity
  Step 8    the edge offset s_50 = s(h = 0.5): positive means the measured edge lies on the front-material side of
            the true edge; a matcher that fattens the front surface gives a NEGATIVE s_50
  Step 9    the edge position transfer (Section 11.1, "Edge position transfer"): every phase-jitter pose of the edge
            (the poses the pooled ESF pools: the nominal pose and the jitter poses of the same edge, Z, gap and
            polarity) gets its OWN s_50 from its own binned ESF (the same binning and half-height crossing as Steps 4
            and 8; a pose with too few valid bins is skipped and counted) and its read-back lateral offset across the
            edge in px: the shift of the true edge line, projected from the read-back pose carried through the
            registration, relative to the nominal pose, divided by p(Z). The regression, its sign convention and the
            periodic terms are described at :func:`fit_edge_transfer`. Where the optional lateral sweep poses exist
            (sub-series "lateral_sweep", the approach direction in the manifest metadata ``approach_direction`` or,
            when the manifest does not carry it, derived from the sweep rule of the plan) they are used ONLY here:
            the approach hysteresis (mean s_50 of the poses approached from the negative side minus that of the poses
            approached from the positive side) and a refit of the periodic terms on the sweep poses alone at the
            sweep's 0.1 px spacing. They are never pooled into the ESF (the sweep covers only +/- 1 px and would bias it).
  Step 10   confidence intervals by bootstrapping over poses (BOOTSTRAP_RESAMPLES, CONFIDENCE_LEVEL). To keep
            the run time low the resampling unit is the pose's binned sums of h and counts, not the raw pixels.

The ESF is made monotone (weighted pool-adjacent-violators) before a level crossing is looked up, so that bin noise
in the tails cannot produce an early crossing of 0.1 or 0.9; the raw ESF is what is reported and differentiated.

Conventions: docs/design/code_design.md Section 4 (millimeters, pixels; arrays are (H, W); no-read is NaN).
"""
from __future__ import annotations

import math
import warnings
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from sensorperf.acquisition.plan import APPROACH_DIRECTION_KEY, LATERAL_SWEEP_APPROACH_PERIOD
from sensorperf.analysis.common import (
    PoseGeometry, hann_smooth, new_figure, pose_geometry, reference_planes, save_figure, write_csv_rows, write_json,
)
from sensorperf.features.planes import normalized_height
from sensorperf.geometry.targets import (
    FEATURE_SQUARE_RAISED, FEATURE_SQUARE_WINDOW, SQUARE_EDGE_BOTTOM, SQUARE_EDGE_LEFT, SQUARE_EDGE_RIGHT,
    SQUARE_EDGE_TOP, SQUARE_EDGES, SURFACE_NONE, Feature,
)
from sensorperf.io.capture_set import load_stack
from sensorperf.io.manifest import SUBSERIES_LATERAL_SWEEP, SUBSERIES_NOMINAL, FrameRecord, group_by_pose, select
from sensorperf.io.session import Session
from sensorperf.parameters import PROCEDURE_EDGES, CharacterizationParameters
from sensorperf.stats.intervals import bootstrap_statistic
from sensorperf.stats.psychometric import pool_adjacent_violators

# ---------------------------------------------------------------------------
# Output names
# ---------------------------------------------------------------------------
SUMMARY_CSV_NAME = "B_resolution_summary.csv"
DETAILS_JSON_NAME = "B_resolution_details.json"
FIGURE_STEMS = {"esf": "B_esf_curves", "lsf_mtf": "B_lsf_mtf", "rise": "B_rise_vs_z", "s50": "B_s50_vs_z",
                "transfer": "B_transfer"}
SUMMARY_COLUMNS = (
    "target_id", "polarity", "gap_mm", "station_z_mm", "edge", "orientation", "rise_px", "rise_mm", "rise_lower_px",
    "rise_upper_px", "mtf50_cycles_per_px", "s50_px", "s50_lower_px", "s50_upper_px", "slanted_rise_px",
    "esf_agreement_ok", "step_height_dependent", "poses", "frames", "slanted_s50_px", "esf_difference_shift_px",
    "lateral_gain", "lateral_gain_se", "pixel_lock_amplitude_px", "dot_pitch_amplitude_px", "dot_pitch_px",
    "transfer_poses", "hysteresis_px", "sweep_pixel_lock_amplitude_px", "sweep_dot_pitch_amplitude_px", "note")
"""Columns of B_resolution_summary.csv: the Section 11.1 list, then the slanted-edge s_50, the equivalent lateral
error of the ESF difference (the two numbers behind esf_agreement_ok), the edge position transfer terms (Step 9:
``lateral_gain`` and its standard error, the amplitudes of the 1 px pixel-locking term and of the projector dot pitch
term, the dot pitch used, the poses that entered the regression, the approach hysteresis and the two amplitudes refitted
on the lateral-sweep poses; NaN where the pose count, Analysis A or the sweep does not allow them) and a note."""

ORIENTATION_H, ORIENTATION_V = "H", "V"
EDGE_ORIENTATION = {SQUARE_EDGE_LEFT: ORIENTATION_H, SQUARE_EDGE_RIGHT: ORIENTATION_H,
                    SQUARE_EDGE_TOP: ORIENTATION_V, SQUARE_EDGE_BOTTOM: ORIENTATION_V}
"""Left and right (near-vertical) edges measure H resolution, top and bottom (near-horizontal) edges V."""
POLARITY_FRONT_INSIDE = "front_inside"
POLARITY_FRONT_OUTSIDE = "front_outside"
"""T3a (raised square: front material inside the square) and T3b (square window: front material outside)."""

# Okabe-Ito palette
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
LINE_STYLES = ("-", "--", ":", "-.")
"""Line styles used in turn for the second series dimension of a figure."""

# ---------------------------------------------------------------------------
# Named numbers that are not procedure parameters
# ---------------------------------------------------------------------------
OTHER_EDGE_CLEARANCE_PX = 6.0
"""Default clearance of the robot-stepped ESF from the other three edges, pixels. Section 11.1, Step 4 keeps the
pixels whose distance to the other three edges exceeds the square's half side minus a margin; for a pixel near one
edge that distance is the along-edge position measured from the neighboring edge, so the rule is the same as a
minimum distance from the other edges. This is that minimum distance: wider than the half width of the imitation
matcher window (3.5 px) plus the sub-pixel jitter, so no pixel of the ESF sees a corner."""
LSF_HANN_WINDOW_BINS = 7
"""Odd length, in bins, of the Hann window that smooths the LSF (1.75 px at the 0.25 px bin width)."""
MTF_ZERO_PAD_FACTOR = 8
"""The LSF is zero padded to this many times its length before the FFT, so MTF50 is not limited by the 1 / (2 band)
frequency spacing of the unpadded transform."""
MTF_HALF_LEVEL = 0.5
"""MTF level of MTF50."""
MIN_BIN_COUNT = 10
"""A bin enters the comparison of two ESFs only if both have at least this many read pixels in it."""
MIN_SLANTED_BIN_COUNT = 3
"""Fewest pixels per bin of the slanted-edge ESF in its comparison with the robot-stepped ESF. The slanted ESF comes from
one pose's mean frame, so its bins hold few pixels (about the number of rows times the bin width over the band)."""
MIN_BINS_FOR_COMPARISON = 5
"""Fewest common bins for an ESF comparison."""
SLANT_TRIM_SIGMA = 3.0
"""Crossings farther than this many robust sigmas from the fitted line are dropped from the slanted-edge fit."""
SLANT_TRIM_ROUNDS = 2
"""Rounds of fit-and-trim of the slanted-edge line."""
MIN_CROSSINGS_FOR_LINE = 5
"""Fewest row (column) crossings for a slanted-edge line."""
MEDIAN_ABSOLUTE_TO_SIGMA = 1.4826
"""Factor converting a median absolute deviation into a Gaussian sigma."""
SLANT_BAND_FACTOR = 2.0
"""The slanted-edge pixels are taken within this multiple of the band around the TRUE edge, then binned by distance to
the fitted line within one band, so a fitted line offset by a pixel still has its full +/- band of data."""
TRANSFER_MIN_READS_PER_BIN = 3
"""A bin of a SINGLE pose's ESF is valid for the edge position transfer when it holds at least this many read pixels
(a pose's bins hold few pixels, like those of the slanted-edge ESF, ``MIN_SLANTED_BIN_COUNT``)."""
TRANSFER_MIN_VALID_BIN_FRACTION = 0.5
"""A pose enters the edge position transfer only if at least this fraction of the bins of its ESF is valid
(``TRANSFER_MIN_READS_PER_BIN``): the crossing of h = 0.5 is then not an extrapolation over a gap; otherwise the pose is
skipped and counted."""
TRANSFER_MIN_POSES = 3
"""Fewest poses for the line of the edge position transfer (two parameters and one degree of freedom for its standard
error)."""
TRANSFER_MIN_RESIDUAL_DOF = 2
"""A periodic term (two more parameters, a sine and a cosine) is fitted only if at least this many degrees of freedom
remain after it, so that a handful of poses cannot be fitted exactly by terms that then carry no information."""
TRANSFER_MAX_CONDITION_NUMBER = 1.0e3
"""A periodic term is skipped when adding it makes the design matrix, with its columns scaled to unit length, worse
conditioned than this: its period is then indistinguishable from the others over the span of offsets (a dot pitch of
one pixel, or a span much shorter than the period)."""
PIXEL_LOCK_PERIOD_PX = 1.0
"""Period of the pixel-locking term of the edge position transfer, pixels: the sensed edge position repeats when the
true edge moves by one pixel (Section 11.1, "Edge position transfer")."""
SWEEP_STEP_MATCH_TOLERANCE = 0.1
"""A sweep pose's offset is taken as the k-th multiple of the sweep step when it lies within this fraction of a step of it."""
SWEEP_APPROACH_SIGN = 0.5
"""The approach regressor of the sweep refit is +this for a pose approached from the negative side and minus this for one
approached from the positive side, so that its coefficient is the hysteresis (the difference of the two sides)."""
APPROACH_FROM_NEGATIVE_SIGN = "-"
"""First character of an ``approach_direction`` of a pose approached from the negative side."""
BOOTSTRAP_SEED = 20261005
"""Seed of the bootstrap generator (deterministic output)."""
MIN_FRAMES = 1
"""Fewest frames of a pose."""
FIGURE_PANEL_WIDTH_IN, FIGURE_PANEL_HEIGHT_IN = 3.6, 2.8
"""Size of one panel of the grid figures, inches."""
MM_PER_PX_LABEL = "rise distance (mm)"
GAP_MATCH_TOLERANCE_MM = 1.0e-6
"""Gaps closer than this are the same gap."""


# ---------------------------------------------------------------------------
# Options and results
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class LateralOptions:
    """Choices of Analysis B-HV that are not procedure parameters."""

    other_edge_clearance_px: float = OTHER_EDGE_CLEARANCE_PX
    """Minimum distance of an ESF pixel from the other three edges, pixels (OTHER_EDGE_CLEARANCE_PX says why)."""
    lsf_window_bins: int = LSF_HANN_WINDOW_BINS
    """Odd Hann window length of the LSF smoothing, bins."""
    mtf_zero_pad_factor: int = MTF_ZERO_PAD_FACTOR
    """Zero-padding factor of the MTF transform."""
    min_bin_count: int = MIN_BIN_COUNT
    """Fewest read pixels per bin in an ESF comparison."""
    min_slanted_bin_count: int = MIN_SLANTED_BIN_COUNT
    """Fewest pixels per bin of the slanted-edge ESF in the agreement test."""
    transfer_min_reads_per_bin: int = TRANSFER_MIN_READS_PER_BIN
    """Fewest read pixels of a valid bin of a single pose's ESF (edge position transfer)."""
    transfer_min_valid_bin_fraction: float = TRANSFER_MIN_VALID_BIN_FRACTION
    """Smallest fraction of valid bins of a pose that enters the edge position transfer."""
    transfer_min_poses: int = TRANSFER_MIN_POSES
    """Fewest poses for the line of the edge position transfer."""
    transfer_min_residual_dof: int = TRANSFER_MIN_RESIDUAL_DOF
    """Degrees of freedom that must remain after a periodic term for it to be fitted."""
    transfer_max_condition_number: float = TRANSFER_MAX_CONDITION_NUMBER
    """Largest condition number of the (column-scaled) design matrix with which a periodic term is still fitted."""
    bootstrap_seed: int = BOOTSTRAP_SEED
    """Seed of the bootstrap generator."""
    figure_width_in: float = 7.0
    """Width of the single-panel figures, inches."""
    figure_height_in: float = 4.5
    """Height of the single-panel figures, inches."""


@dataclass
class PoseBins:
    """Binned sums of one pose and one edge (the resampling unit of Step 9)."""

    sum_h: np.ndarray
    n_read: np.ndarray
    n_noread: np.ndarray


@dataclass(frozen=True)
class TransferPoint:
    """One pose of one edge in the edge position transfer (Step 9)."""

    offset_px: float
    """Read-back lateral offset of the pose across the edge, px: the displacement of the true edge line in the sensor frame
    relative to the nominal pose, positive toward the front-material side (the direction of positive s)."""
    s50_px: float
    """The pose's own edge crossing s(h = 0.5), px, in the frame of the true edge line projected from the pose's read-back
    pose (as every s of Section 11.1)."""
    approached_from_negative: bool | None = None
    """For a lateral-sweep pose: True when the robot arrived from the negative side, False from the positive side; None for
    a nominal or jitter pose (and for a sweep pose whose approach cannot be told)."""


@dataclass
class TransferFit:
    """The regression and the periodic terms of the edge position transfer of one set of poses (:func:`fit_edge_transfer`)."""

    poses: int
    """Poses that entered the fit."""
    gain: float = float("nan")
    """Lateral gain: the slope of the sensed edge position on the true edge position (expected 1)."""
    gain_se: float = float("nan")
    """Standard error of the gain from the residual scatter of the fit (NaN without a degree of freedom)."""
    intercept_px: float = float("nan")
    """Edge bias: s_50 at zero offset, that is, at the nominal pose (the edge bias of Step 8)."""
    pixel_lock_amplitude_px: float = float("nan")
    """Amplitude of the 1 px periodic term, px (sine and cosine fitted by least squares)."""
    pixel_lock_coefficients: tuple[float, float] = (float("nan"), float("nan"))
    """(sine, cosine) coefficients of the 1 px term, px."""
    dot_pitch_amplitude_px: float = float("nan")
    """Amplitude of the projector dot pitch term, px (NaN when no pitch was given or the term was not fitted)."""
    dot_pitch_coefficients: tuple[float, float] = (float("nan"), float("nan"))
    """(sine, cosine) coefficients of the dot pitch term, px."""
    dot_pitch_px: float = float("nan")
    """The dot pitch period of the fitted term, px (NaN when it was not fitted)."""
    hysteresis_coefficient_px: float = float("nan")
    """Coefficient of the approach regressor of a sweep refit (equals the hysteresis when the sides are balanced)."""
    residual_rms_px: float = float("nan")
    """RMS of what the full fit leaves, px."""
    note: str = ""


@dataclass
class EdgeTransfer:
    """Step 9 for one edge: the per-pose points, the fit, and the sweep results."""

    points: list[TransferPoint] = field(default_factory=list)
    skipped_poses: int = 0
    """Poses left out for too few valid bins."""
    fit: TransferFit | None = None
    sweep_points: list[TransferPoint] = field(default_factory=list)
    sweep_skipped_poses: int = 0
    sweep_fit: TransferFit | None = None
    hysteresis_px: float = float("nan")
    """Mean s_50 of the sweep poses approached from the negative side minus that of those approached from the positive
    side (px; NaN without a sweep or without poses on both sides). The sign is that of s, positive toward the front side."""
    dot_pitch_px: float = float("nan")
    """The dot pitch (the correlation length of Analysis A) offered to the fits, px; NaN without it."""
    notes: list[str] = field(default_factory=list)


@dataclass
class SlantedEsf:
    """Step 5 for one edge: the slanted-edge ESF of the nominal pose."""

    centers_px: np.ndarray
    esf: np.ndarray
    count: np.ndarray
    offset_px: float
    """Signed distance (px, positive on the front side) of the fitted line from the true edge line."""
    s50_px: float
    """s(h = 0.5) of the binned slanted ESF in the true-edge frame (the offset plus the half-height bin crossing)."""
    rise_px: float
    crossings: int
    line_residual_rms_px: float
    ok: bool
    note: str = ""


@dataclass
class EdgeResult:
    """One edge of one configuration: ESF, metrics, intervals and flags (a row of the CSV)."""

    target_id: str
    polarity: str
    gap_mm: float
    station_z_mm: float
    edge: str
    orientation: str
    poses: int
    frames: int
    pixel_footprint_mm: float
    bin_centers_px: np.ndarray
    esf: np.ndarray
    """Robot-stepped ESF: mean h per bin (NaN where no read)."""
    esf_monotone: np.ndarray
    count_read: np.ndarray
    count_noread: np.ndarray
    count_total: np.ndarray
    lsf: np.ndarray
    mtf_frequency: np.ndarray
    mtf: np.ndarray
    rise_px: float = float("nan")
    rise_lower_px: float = float("nan")
    rise_upper_px: float = float("nan")
    mtf50_cycles_per_px: float = float("nan")
    s50_px: float = float("nan")
    s50_lower_px: float = float("nan")
    s50_upper_px: float = float("nan")
    slanted: SlantedEsf | None = None
    transfer: EdgeTransfer | None = None
    """The edge position transfer of Step 9 (None when the edge has no poses)."""
    slanted_s50_difference_px: float = float("nan")
    esf_rms_difference_h: float = float("nan")
    esf_difference_shift_px: float = float("nan")
    esf_agreement_ok: bool | None = None
    step_height_dependent: bool | None = None
    max_delta_h: float = float("nan")
    bootstrap_failures: int = 0
    pose_bins: list[PoseBins] = field(default_factory=list, repr=False)
    note: str = ""

    @property
    def rise_mm(self) -> float:
        """Rise distance in mm: rise_px times p(Z)."""
        return self.rise_px * self.pixel_footprint_mm

    def csv_row(self) -> dict[str, Any]:
        nan = float("nan")
        transfer = self.transfer
        fit = None if transfer is None else transfer.fit
        sweep_fit = None if transfer is None else transfer.sweep_fit
        return {
            "target_id": self.target_id, "polarity": self.polarity, "gap_mm": self.gap_mm,
            "station_z_mm": self.station_z_mm, "edge": self.edge, "orientation": self.orientation,
            "rise_px": self.rise_px, "rise_mm": self.rise_mm, "rise_lower_px": self.rise_lower_px,
            "rise_upper_px": self.rise_upper_px, "mtf50_cycles_per_px": self.mtf50_cycles_per_px,
            "s50_px": self.s50_px, "s50_lower_px": self.s50_lower_px, "s50_upper_px": self.s50_upper_px,
            "slanted_rise_px": float("nan") if self.slanted is None else self.slanted.rise_px,
            "esf_agreement_ok": self.esf_agreement_ok, "step_height_dependent": self.step_height_dependent,
            "poses": self.poses, "frames": self.frames,
            "slanted_s50_px": float("nan") if self.slanted is None else self.slanted.s50_px,
            "esf_difference_shift_px": self.esf_difference_shift_px,
            "lateral_gain": nan if fit is None else fit.gain, "lateral_gain_se": nan if fit is None else fit.gain_se,
            "pixel_lock_amplitude_px": nan if fit is None else fit.pixel_lock_amplitude_px,
            "dot_pitch_amplitude_px": nan if fit is None else fit.dot_pitch_amplitude_px,
            "dot_pitch_px": nan if fit is None else fit.dot_pitch_px,
            "transfer_poses": 0 if transfer is None else len(transfer.points),
            "hysteresis_px": nan if transfer is None else transfer.hysteresis_px,
            "sweep_pixel_lock_amplitude_px": nan if sweep_fit is None else sweep_fit.pixel_lock_amplitude_px,
            "sweep_dot_pitch_amplitude_px": nan if sweep_fit is None else sweep_fit.dot_pitch_amplitude_px,
            "note": self.note}


@dataclass
class LateralResolutionResult:
    """Everything Analysis B-HV found."""

    edges: list[EdgeResult]
    bin_width_px: float
    band_px: float
    reference_station_mm: float
    notes: list[str]
    options: LateralOptions

    def at_station(self, station_z_mm: float, orientation: str | None = None) -> list[EdgeResult]:
        """The edge results of a station, optionally of one orientation."""
        return [e for e in self.edges if e.station_z_mm == station_z_mm
                and (orientation is None or e.orientation == orientation)]

    def forward_model_terms(self) -> dict[str, Any]:
        """Mean rise distance (px) along H and V at the mid station (the station nearest Z_REFERENCE_MM) and the mean
        edge offset s_50 over all edges, for the boundary terms of the Tier-A simulator."""
        def mean_of(values: list[float]) -> float | None:
            finite = [v for v in values if math.isfinite(v)]
            return float(np.mean(finite)) if finite else None
        return {
            "rise_h_px": mean_of([e.rise_px for e in self.at_station(self.reference_station_mm, ORIENTATION_H)]),
            "rise_v_px": mean_of([e.rise_px for e in self.at_station(self.reference_station_mm, ORIENTATION_V)]),
            "edge_offset_px": mean_of([e.s50_px for e in self.edges]),
        }


# ---------------------------------------------------------------------------
# ESF numerics (Steps 6 to 8)
# ---------------------------------------------------------------------------
def esf_from_sums(sum_h: np.ndarray, count: np.ndarray) -> np.ndarray:
    """Mean h per bin from the binned sums; NaN in empty bins."""
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(count > 0, sum_h / np.maximum(count, 1.0), np.nan)


def monotone_esf(esf: np.ndarray, count: np.ndarray) -> np.ndarray:
    """The ESF made non-decreasing in s by weighted pool-adjacent-violators (weights = read counts), with empty bins
    filled by linear interpolation. Used only to look up level crossings."""
    ok = np.isfinite(esf) & (count > 0)
    if ok.sum() < 2:
        return np.full(esf.shape, np.nan)
    fitted = pool_adjacent_violators(esf[ok], count[ok])
    index = np.arange(esf.size)
    return np.interp(index, index[ok], fitted)


def level_crossing(centers: np.ndarray, monotone: np.ndarray, level: float) -> float:
    """s at which a non-decreasing profile first reaches ``level`` (linear interpolation); NaN if it does not span it."""
    if not np.all(np.isfinite(monotone)) or monotone[0] > level or monotone[-1] < level:
        return float("nan")
    upper = int(np.searchsorted(monotone, level, side="left"))
    if upper == 0:
        return float(centers[0])
    low_value, high_value = monotone[upper - 1], monotone[upper]
    if high_value == low_value:
        return float(centers[upper])
    return float(centers[upper - 1] + (level - low_value) * (centers[upper] - centers[upper - 1])
                 / (high_value - low_value))


def rise_and_s50(centers: np.ndarray, sum_h: np.ndarray, count: np.ndarray,
                 params: CharacterizationParameters) -> tuple[float, float]:
    """(10-90 percent rise distance, s_50) in px of the ESF from binned sums (Step 6 and Step 8)."""
    monotone = monotone_esf(esf_from_sums(sum_h, count), count)
    if not np.all(np.isfinite(monotone)):
        return float("nan"), float("nan")
    s_low = level_crossing(centers, monotone, params.esf_rise_low)
    s_high = level_crossing(centers, monotone, params.esf_rise_high)
    s_half = level_crossing(centers, monotone, params.esf_half_height)
    return s_high - s_low, s_half


def lsf_and_mtf(centers: np.ndarray, esf: np.ndarray, bin_width_px: float,
                options: LateralOptions) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(LSF, MTF frequencies in cycles/px, MTF) of an ESF (Step 6): LSF = dESF/ds of the ESF with empty bins
    interpolated, smoothed with a Hann window; MTF = |FFT(LSF)| of the zero-padded LSF, normalized to 1 at zero
    frequency."""
    ok = np.isfinite(esf)
    if ok.sum() < options.lsf_window_bins:
        nan = np.full(esf.shape, np.nan)
        return nan, np.array([np.nan]), np.array([np.nan])
    filled = np.interp(centers, centers[ok], esf[ok])
    lsf = hann_smooth(np.gradient(filled, bin_width_px), options.lsf_window_bins)
    padded = options.mtf_zero_pad_factor * lsf.size
    spectrum = np.abs(np.fft.rfft(lsf, n=padded))
    frequency = np.fft.rfftfreq(padded, d=bin_width_px)
    return lsf, frequency, spectrum / spectrum[0] if spectrum[0] > 0 else spectrum


def mtf50(frequency: np.ndarray, mtf: np.ndarray) -> float:
    """First frequency (cycles/px, linear interpolation) at which the MTF falls to MTF_HALF_LEVEL; NaN if it never does."""
    below = np.flatnonzero(mtf <= MTF_HALF_LEVEL)
    if below.size == 0 or not np.all(np.isfinite(mtf[:below[0] + 1])):
        return float("nan")
    i = int(below[0])
    if i == 0:
        return float(frequency[0])
    return float(frequency[i - 1] + (mtf[i - 1] - MTF_HALF_LEVEL) * (frequency[i] - frequency[i - 1])
                 / (mtf[i - 1] - mtf[i]))


# ---------------------------------------------------------------------------
# Edge position transfer (Step 9)
# ---------------------------------------------------------------------------
def pose_s50(bins: PoseBins, centers: np.ndarray, params: CharacterizationParameters,
             options: LateralOptions) -> float | None:
    """The edge crossing s_50 of ONE pose from its own binned ESF (Step 8 applied to a single pose: the same monotone ESF
    and half-height crossing as the pooled one), or None when the pose has too few valid bins.

    A bin is valid with at least ``options.transfer_min_reads_per_bin`` read pixels; the pose needs at least
    ``options.transfer_min_valid_bin_fraction`` of its bins valid, and the ESF of the valid bins must reach h = 0.5.
    Invalid bins are left out of the ESF (their counts are set to zero), not interpolated from a few noisy pixels."""
    valid = bins.n_read >= options.transfer_min_reads_per_bin
    if valid.sum() < options.transfer_min_valid_bin_fraction * valid.size:
        return None
    _, s_half = rise_and_s50(centers, np.where(valid, bins.sum_h, 0.0), np.where(valid, bins.n_read, 0.0), params)
    return s_half if math.isfinite(s_half) else None


def _scaled_condition_number(design: np.ndarray) -> float:
    """Condition number of a design matrix whose columns are scaled to unit length (so the units of the columns, pixels of
    offset against dimensionless sines, do not decide it)."""
    norms = np.linalg.norm(design, axis=0)
    return float(np.linalg.cond(design / np.where(norms > 0.0, norms, 1.0)))


def fit_edge_transfer(offset_px: np.ndarray, s50_px: np.ndarray, dot_pitch_px: float | None,
                      options: LateralOptions, approach_sign: np.ndarray | None = None) -> TransferFit:
    """The regression of the edge position transfer (Section 11.1, "Edge position transfer").

    ``s50_px`` is each pose's s_50 in the frame of ITS OWN true edge line (the read-back pose through the registration), and
    ``offset_px`` the displacement of that line relative to the nominal pose along the s axis. A sensor that reports edge
    motion with gain g puts the edge, in a frame fixed to the sensor, at ``offset + s_50 = g * offset + bias``: the
    sensed position against the true position has slope g (expected 1) and intercept bias. In the frame of the true edge
    that is ``s_50 = bias + (g - 1) * offset``, so the fit regresses s_50 on the offset and reports the gain as ONE PLUS the
    slope; a sensor that follows the edge exactly has zero slope here and gain 1. The intercept is s_50 at the nominal pose,
    the edge bias of Step 8, and the standard error of the gain is that of the slope.

    The residual is fitted with periodic terms in the offset, a sine and a cosine at each period by least squares:
    ``PIXEL_LOCK_PERIOD_PX`` (pixel locking) and, when ``dot_pitch_px`` is given, the projector dot pitch (the correlation
    length of Analysis A). Their amplitudes are sqrt(sine^2 + cosine^2). Line and periodic terms are fitted TOGETHER, not
    one after the other: over a short span of offsets (the sweep covers 2 px) a sequential fit would leak part of a
    periodic term into the slope. A term is skipped, with a note, when too few degrees of freedom would remain
    (``options.transfer_min_residual_dof``) or when it is not separable from the terms already in (condition number above
    ``options.transfer_max_condition_number``, which is what a dot pitch of exactly 1 px gives).

    ``approach_sign`` (sweep poses only) adds a regressor, +SWEEP_APPROACH_SIGN for a pose approached from the negative side and
    minus it from the positive side, so that approach hysteresis cannot leak into the periodic terms of the refit; its
    coefficient is returned as ``hysteresis_coefficient_px``."""
    offset = np.asarray(offset_px, dtype=np.float64)
    s50 = np.asarray(s50_px, dtype=np.float64)
    poses = int(offset.size)
    fit = TransferFit(poses=poses)
    if poses < options.transfer_min_poses:
        fit.note = f"only {poses} poses (at least {options.transfer_min_poses} needed)"
        return fit
    columns = [np.ones(poses), offset]
    has_approach = approach_sign is not None and bool(np.ptp(approach_sign) > 0.0)
    if has_approach:
        columns.append(np.asarray(approach_sign, dtype=np.float64))
    notes: list[str] = []
    periods = {"pixel_lock": PIXEL_LOCK_PERIOD_PX}
    if dot_pitch_px is not None and math.isfinite(dot_pitch_px) and dot_pitch_px > 0.0:
        periods["dot_pitch"] = float(dot_pitch_px)
    fitted: list[str] = []
    for name, period in periods.items():
        phase = 2.0 * math.pi * offset / period
        candidate = columns + [np.sin(phase), np.cos(phase)]
        if poses - len(candidate) < options.transfer_min_residual_dof:
            notes.append(f"{name} term skipped: {poses} poses leave fewer than {options.transfer_min_residual_dof} "
                         "degrees of freedom")
        elif _scaled_condition_number(np.column_stack(candidate)) > options.transfer_max_condition_number:
            notes.append(f"{name} term skipped: its period of {period:g} px is not separable from the other terms "
                         "over the span of offsets")
        else:
            columns = candidate
            fitted.append(name)
    design = np.column_stack(columns)
    coefficients, *_ = np.linalg.lstsq(design, s50, rcond=None)
    residual = s50 - design @ coefficients
    dof = poses - design.shape[1]
    fit.intercept_px, fit.gain = float(coefficients[0]), 1.0 + float(coefficients[1])
    if dof >= 1:
        variance = float(residual @ residual) / dof
        covariance = variance * np.linalg.pinv(design.T @ design)
        fit.gain_se = float(math.sqrt(max(covariance[1, 1], 0.0)))
        fit.residual_rms_px = float(math.sqrt(float(residual @ residual) / poses))
    index = 2
    if has_approach:
        fit.hysteresis_coefficient_px = float(coefficients[index])
        index += 1
    for name in fitted:
        sine, cosine = float(coefficients[index]), float(coefficients[index + 1])
        amplitude = math.hypot(sine, cosine)
        if name == "pixel_lock":
            fit.pixel_lock_amplitude_px, fit.pixel_lock_coefficients = amplitude, (sine, cosine)
        else:
            fit.dot_pitch_amplitude_px, fit.dot_pitch_coefficients = amplitude, (sine, cosine)
            fit.dot_pitch_px = periods[name]
        index += 2
    fit.note = "; ".join(notes)
    return fit


def approach_hysteresis(s50_px: np.ndarray, approached_from_negative: list[bool | None]) -> float:
    """Approach hysteresis (px): the mean s_50 of the poses approached from the negative side minus the mean s_50 of those
    approached from the positive side; NaN unless both sides have a pose. Poses whose approach is unknown are left out."""
    s50 = np.asarray(s50_px, dtype=np.float64)
    from_negative = np.array([a is True for a in approached_from_negative], dtype=bool)
    from_positive = np.array([a is False for a in approached_from_negative], dtype=bool)
    if not from_negative.any() or not from_positive.any():
        return float("nan")
    return float(np.mean(s50[from_negative]) - np.mean(s50[from_positive]))


def analyze_edge_transfer(points: list[TransferPoint], sweep_points: list[TransferPoint], dot_pitch_px: float | None,
                          options: LateralOptions, skipped: int = 0, sweep_skipped: int = 0) -> EdgeTransfer:
    """Step 9 for one edge from its per-pose points: the regression and periodic terms over the nominal and jitter poses,
    and, where sweep poses exist, the approach hysteresis and the refit of the periodic terms on the sweep poses alone."""
    transfer = EdgeTransfer(points=list(points), skipped_poses=skipped, sweep_points=list(sweep_points),
                            sweep_skipped_poses=sweep_skipped,
                            dot_pitch_px=float("nan") if dot_pitch_px is None else float(dot_pitch_px))
    if dot_pitch_px is None:
        transfer.notes.append("dot pitch term skipped: no Analysis A correlation length")
    if skipped:
        transfer.notes.append(f"{skipped} poses skipped for too few valid bins")
    transfer.fit = fit_edge_transfer(np.array([p.offset_px for p in points]), np.array([p.s50_px for p in points]),
                                     dot_pitch_px, options)
    if transfer.fit.note:
        transfer.notes.append(transfer.fit.note)
    if sweep_points:
        sweep_offset = np.array([p.offset_px for p in sweep_points])
        sweep_s50 = np.array([p.s50_px for p in sweep_points])
        sides = [p.approached_from_negative for p in sweep_points]
        transfer.hysteresis_px = approach_hysteresis(sweep_s50, sides)
        sign = np.array([0.0 if side is None else (SWEEP_APPROACH_SIGN if side else -SWEEP_APPROACH_SIGN)
                         for side in sides])
        transfer.sweep_fit = fit_edge_transfer(sweep_offset, sweep_s50, dot_pitch_px, options, approach_sign=sign)
        if sweep_skipped:
            transfer.notes.append(f"{sweep_skipped} sweep poses skipped for too few valid bins")
        if transfer.sweep_fit.note:
            transfer.notes.append("sweep: " + transfer.sweep_fit.note)
    return transfer


def noise_correlation_length_px(previous: dict | None, station_z_mm: float, orientation: str) -> float | None:
    """The correlation length (px) of Analysis A (Section 10, Step 7) at the station nearest to ``station_z_mm`` (by the
    ratio of depths), along H (``corr_len_h_px``) for H edges and along V (``corr_len_v_px``) for V edges, from the
    fronto-parallel center-field main rows of ``previous["A"]``; None when A did not run or has no finite length. The
    projector dot pitch of the edge position transfer is taken from it."""
    result = None if not previous else previous.get("A")
    main_rows = getattr(result, "main_rows", None)
    if not callable(main_rows):
        return None
    attribute = "corr_len_h_px" if orientation == ORIENTATION_H else "corr_len_v_px"
    candidates = [(abs(math.log(row.station_z_mm / station_z_mm)), float(getattr(row, attribute)))
                  for row in main_rows() if math.isfinite(getattr(row, attribute, float("nan")))
                  and getattr(row, attribute) > 0.0 and row.station_z_mm > 0.0]
    return min(candidates)[1] if candidates else None


def sweep_axis_and_approach(record: FrameRecord, pixel_footprint_mm: float,
                            sweep_step_px: float) -> tuple[str, bool | None]:
    """(axis, approached from the negative side) of a lateral-sweep pose.

    The manifest metadata ``approach_direction`` ("-H", "+H", "-V" or "+V", the plan's note) is used where the manifest
    carries it. The manifest written by the planner at present does not, so otherwise the sweep rule of the plan is applied
    to the logged offset: the axis is the one the pose is offset along, and the k-th pose of an axis, at k x the sweep step,
    is approached from the negative side for odd k (the 1st, 3rd, ...) and from the positive side for even k
    (``LATERAL_SWEEP_APPROACH_PERIOD``). None for the approach when the offset is not a multiple of the step."""
    direction = record.metadata.get(APPROACH_DIRECTION_KEY, "")
    if len(direction) == 2 and direction[1] in (ORIENTATION_H, ORIENTATION_V) and direction[0] in ("-", "+"):
        return direction[1], direction[0] == APPROACH_FROM_NEGATIVE_SIGN
    axis = ORIENTATION_H if abs(record.offset_h_mm) >= abs(record.offset_v_mm) else ORIENTATION_V
    offset_mm = record.offset_h_mm if axis == ORIENTATION_H else record.offset_v_mm
    k = abs(offset_mm) / (sweep_step_px * pixel_footprint_mm)
    k_rounded = int(round(k))
    if k_rounded < 1 or abs(k - k_rounded) > SWEEP_STEP_MATCH_TOLERANCE:
        return axis, None
    return axis, (k_rounded - 1) % LATERAL_SWEEP_APPROACH_PERIOD == 0


# ---------------------------------------------------------------------------
# One pose
# ---------------------------------------------------------------------------
def _square_of(geometry: PoseGeometry) -> Feature:
    squares = geometry.target.features_of_kind(FEATURE_SQUARE_RAISED, FEATURE_SQUARE_WINDOW)
    if not squares:
        raise ValueError(f"target {geometry.target.target_id} has no square feature; Analysis B-HV needs T3a or T3b")
    return squares[0]


def inward_distances_px(geometry: PoseGeometry, square: Feature) -> dict[str, np.ndarray]:
    """Per edge the (H, W) signed distance (px) to that edge's line, positive toward the square's INTERIOR whatever
    the polarity (``Feature.signed_distance_mm`` is positive on the front-material side, which is the exterior for
    a window)."""
    sign = 1.0 if square.is_front_material() else -1.0
    return {edge: sign * geometry.signed_distance_px(square, edge) for edge in SQUARE_EDGES}


def edge_selection(geometry: PoseGeometry, inward: dict[str, np.ndarray], edge: str, clearance_px: float,
                   reach_px: float) -> np.ndarray:
    """(H, W) mask of the pixels used for one edge: within ``reach_px`` of its line, at least ``clearance_px`` from
    the other three edges (on the inner side of their lines, as inward distance), and on a ray that meets a plate."""
    mask = np.abs(inward[edge]) <= reach_px
    for other in SQUARE_EDGES:
        if other != edge:
            mask &= inward[other] >= clearance_px
    return mask & (geometry.hit.surface != SURFACE_NONE)


def bin_pose_edge(s: np.ndarray, selected: np.ndarray, height: np.ndarray, edges_px: np.ndarray) -> PoseBins:
    """Step 4 for one pose and one edge: sum of h, count of reads and count of no-reads per s bin. ``height`` is
    (F, H, W) with NaN where the pixel was not read or h is undefined; ``selected`` marks the pixels used."""
    n_bins = len(edges_px) - 1
    bin_width = edges_px[1] - edges_px[0]
    s_selected = s[selected]
    in_range = (s_selected >= edges_px[0]) & (s_selected < edges_px[-1])
    index = np.clip(((s_selected - edges_px[0]) / bin_width).astype(int), 0, n_bins - 1)
    h_selected = height[:, selected][:, in_range]
    index = index[in_range]
    frames = h_selected.shape[0]
    read = np.isfinite(h_selected)
    flat_index = np.broadcast_to(index, h_selected.shape)
    sum_h = np.bincount(flat_index[read], weights=h_selected[read], minlength=n_bins)
    n_read = np.bincount(flat_index[read], minlength=n_bins).astype(float)
    n_total = np.bincount(index, minlength=n_bins).astype(float) * frames
    return PoseBins(sum_h=sum_h, n_read=n_read, n_noread=n_total - n_read)


def slanted_edge_esf(h_mean: np.ndarray, s: np.ndarray, selected: np.ndarray, edges_px: np.ndarray,
                     params: CharacterizationParameters, near_vertical: bool) -> SlantedEsf:
    """Step 5: the slanted-edge ESF on the nominal pose's mean frame.

    For a near-vertical edge each ROW is scanned along the columns, for a near-horizontal edge each COLUMN along the
    rows (the arrays are transposed so that the edge is near-vertical in what follows). In each line the pixels
    of the edge's selection are ordered; the crossing of h = ESF_HALF_HEIGHT nearest to the true edge (s = 0) is
    found by linear interpolation, provided the line starts on the back side (h below) and ends on the front side
    (h above), i.e. the edge really lies inside the window. A straight line u = a + b r is fitted to the crossings with
    trimming of outliers; every selected pixel is then binned by its signed distance d to that line (positive on the
    front side) and the mean h per bin is the slanted-edge ESF. The alignment offset is the signed distance of the
    fitted line from the true edge line, so d + offset puts the ESF in the true-edge frame of the robot-stepped one."""
    if not near_vertical:
        h_mean, s, selected = h_mean.T, s.T, selected.T
    half = params.esf_half_height
    n_bins = len(edges_px) - 1
    centers = (edges_px[:-1] + edges_px[1:]) / 2.0
    nan_bins = np.full(n_bins, np.nan)
    failed = SlantedEsf(centers_px=centers, esf=nan_bins, count=np.zeros(n_bins), offset_px=float("nan"),
                        s50_px=float("nan"), rise_px=float("nan"), crossings=0, line_residual_rms_px=float("nan"),
                        ok=False)
    rows_used, crossings, s_cross = [], [], []
    front_sign = 0.0
    for r in range(h_mean.shape[0]):
        columns = np.flatnonzero(selected[r] & np.isfinite(h_mean[r]) & np.isfinite(s[r]))
        if columns.size < 4 or np.any(np.diff(columns) != 1):
            continue
        s_line, h_line = s[r, columns], h_mean[r, columns]
        slope = np.polyfit(columns, s_line, 1)[0]
        if slope == 0.0:
            continue
        oriented = slice(None) if slope > 0 else slice(None, None, -1)        # front side last
        columns_o, s_o, h_o = columns[oriented], s_line[oriented], h_line[oriented]
        if not (h_o[0] < half < h_o[-1]):
            continue
        # Crossings of the half height; keep the one nearest to the true edge (s = 0).
        best = None
        for i in range(len(h_o) - 1):
            if (h_o[i] - half) * (h_o[i + 1] - half) <= 0.0 and h_o[i] != h_o[i + 1]:
                fraction = (half - h_o[i]) / (h_o[i + 1] - h_o[i])
                s_here = s_o[i] + fraction * (s_o[i + 1] - s_o[i])
                if best is None or abs(s_here) < abs(best[1]):
                    best = (columns_o[i] + fraction * (columns_o[i + 1] - columns_o[i]), s_here)
        if best is None:
            continue
        rows_used.append(r)
        crossings.append(best[0])
        s_cross.append(best[1])
        front_sign = 1.0 if slope > 0 else -1.0
    if len(rows_used) < MIN_CROSSINGS_FOR_LINE:
        failed.note = f"only {len(rows_used)} crossings of the half height"
        failed.crossings = len(rows_used)
        return failed
    rows_arr, cross_arr, s_arr = np.array(rows_used, float), np.array(crossings), np.array(s_cross)
    keep = np.ones(rows_arr.size, bool)
    for _ in range(SLANT_TRIM_ROUNDS):
        slope_b, intercept_a = np.polyfit(rows_arr[keep], cross_arr[keep], 1)
        residual = cross_arr - (intercept_a + slope_b * rows_arr)
        scale = max(MEDIAN_ABSOLUTE_TO_SIGMA * np.median(np.abs(residual[keep])), np.finfo(float).eps)
        new_keep = np.abs(residual) <= SLANT_TRIM_SIGMA * scale
        if new_keep.sum() < MIN_CROSSINGS_FOR_LINE or np.array_equal(new_keep, keep):
            break
        keep = new_keep
    slope_b, intercept_a = np.polyfit(rows_arr[keep], cross_arr[keep], 1)
    residual_rms = float(np.sqrt(np.mean((cross_arr[keep] - (intercept_a + slope_b * rows_arr[keep])) ** 2)))
    # Signed distance of the fitted line from the true edge: s along each used row at the fitted crossing. The
    # crossing s values themselves are on the line to within the fit residual, so their trimmed mean is the offset.
    offset = float(np.mean(s_arr[keep]))
    # Bin all selected pixels by the distance d to the fitted line (positive on the front side), in the true-edge
    # frame (d + offset), then the ESF is the mean h per bin.
    u, r_grid = np.meshgrid(np.arange(h_mean.shape[1], dtype=float), np.arange(h_mean.shape[0], dtype=float))
    d = front_sign * (u - (intercept_a + slope_b * r_grid)) / math.sqrt(1.0 + slope_b ** 2) + offset
    use = selected & np.isfinite(h_mean) & (d >= edges_px[0]) & (d < edges_px[-1])
    index = np.clip(((d[use] - edges_px[0]) / (edges_px[1] - edges_px[0])).astype(int), 0, n_bins - 1)
    count = np.bincount(index, minlength=n_bins).astype(float)
    total = np.bincount(index, weights=h_mean[use], minlength=n_bins)
    esf = esf_from_sums(total, count)
    rise, s_half = rise_and_s50(centers, total, count, params)
    return SlantedEsf(centers_px=centers, esf=esf, count=count, offset_px=offset, s50_px=s_half, rise_px=rise,
                      crossings=int(keep.sum()), line_residual_rms_px=residual_rms, ok=True)


# ---------------------------------------------------------------------------
# The analysis
# ---------------------------------------------------------------------------
def _polarity(kind: str) -> str:
    return POLARITY_FRONT_INSIDE if kind == "raised_square" else POLARITY_FRONT_OUTSIDE


def _compare_esfs(esf_a: np.ndarray, count_a: np.ndarray, esf_b: np.ndarray, count_b: np.ndarray,
                  min_count: int, min_count_b: int | None = None) -> tuple[float, float]:
    """(RMS difference, max |difference|) of two ESFs over the bins where both have at least min_count reads;
    NaN if fewer than MIN_BINS_FOR_COMPARISON bins are common."""
    common = (count_a >= min_count) & (count_b >= (min_count if min_count_b is None else min_count_b)) & np.isfinite(esf_a) & np.isfinite(esf_b)
    if common.sum() < MIN_BINS_FOR_COMPARISON:
        return float("nan"), float("nan")
    difference = esf_a[common] - esf_b[common]
    return float(np.sqrt(np.mean(difference ** 2))), float(np.max(np.abs(difference)))


@dataclass
class _PoseEdgeData:
    """What one pose contributes for one edge: its binned ESF sums and the mean s of the reference pixels (Step 9)."""

    bins: PoseBins
    reference_s_px: float
    """Mean signed distance, px, of the edge's reference pixels (the pixels the nominal pose selected for this edge) to
    the true edge line of THIS pose. The shift of this number from the nominal pose to this pose is minus the pose's
    lateral offset across the edge (the true edge moves one way, the pixels see it the other way)."""


def _process_pose(session: Session, group: list[FrameRecord], options: LateralOptions, edges_px: np.ndarray,
                  reference_masks: dict[str, np.ndarray]):
    """Steps 1 to 5 for one pose: reference planes, normalized height of every frame, the signed distance of every pixel
    to each true edge, the pose's binned sums per edge, the reference-pixel mean s per edge (for the lateral offset of
    Step 9) and, for the nominal pose, the slanted-edge ESF. ``reference_masks`` is filled from the first pose processed
    (the nominal pose: the caller orders it first) and read for every later one, so that the offsets of all poses are
    measured at the same pixels."""
    params = session.params
    band = params.boundary_band_half_width_px
    stack = load_stack(group)
    geometry = pose_geometry(session, stack.record(), stack.camera)
    if geometry.z_back_gt is None:
        return None
    square = _square_of(geometry)
    # Steps 1-2: reference planes from the pose's mean frame, normalized height of every frame.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)          # pixels never read give an empty mean (NaN)
        mean_depth = stack.mean_depth()
    planes = reference_planes(mean_depth, geometry, params)
    height = normalized_height(stack.depth, planes.z_front[None], planes.z_back[None])
    inward = inward_distances_px(geometry, square)
    is_nominal = stack.record().subseries == SUBSERIES_NOMINAL
    h_mean = normalized_height(mean_depth, planes.z_front, planes.z_back) if is_nominal else None
    edges: dict[str, _PoseEdgeData] = {}
    slanted: dict[str, SlantedEsf] = {}
    for edge in SQUARE_EDGES:
        # Step 3: signed distance to the true edge (positive on the front-material side).
        s = geometry.signed_distance_px(square, edge)
        # Step 4: pixels near this edge only; per-pose binned sums.
        selected = edge_selection(geometry, inward, edge, options.other_edge_clearance_px, band)
        reference = reference_masks.setdefault(edge, selected) & np.isfinite(s)
        edges[edge] = _PoseEdgeData(
            bins=bin_pose_edge(s, selected, height, edges_px),
            reference_s_px=float(np.mean(s[reference])) if reference.any() else float("nan"))
        if is_nominal:
            wide = edge_selection(geometry, inward, edge, options.other_edge_clearance_px, SLANT_BAND_FACTOR * band)
            slanted[edge] = slanted_edge_esf(h_mean, s, wide, edges_px, params, EDGE_ORIENTATION[edge] == ORIENTATION_H)
    return geometry, stack, edges, slanted


def _analyze_configuration(session: Session, records: list, sweep_records: list, options: LateralOptions,
                           rng: np.random.Generator, notes: list[str],
                           previous: dict | None = None) -> list[EdgeResult]:
    """Steps 1 to 6 and 8 to 10 for the four edges of one configuration (target, gap, station); Step 9 also uses the
    ``sweep_records`` (lateral-sweep poses of the same configuration) and Analysis A's result in ``previous``."""
    params = session.params
    band, bin_width = params.boundary_band_half_width_px, params.boundary_bin_width_px
    edges_px = np.arange(-band, band + bin_width / 2.0, bin_width)
    centers = (edges_px[:-1] + edges_px[1:]) / 2.0
    first = records[0]
    per_edge_bins: dict[str, list[PoseBins]] = {e: [] for e in SQUARE_EDGES}
    slanted: dict[str, SlantedEsf | None] = {e: None for e in SQUARE_EDGES}
    transfer_points: dict[str, list[TransferPoint]] = {e: [] for e in SQUARE_EDGES}
    sweep_points: dict[str, list[TransferPoint]] = {e: [] for e in SQUARE_EDGES}
    skipped = {e: 0 for e in SQUARE_EDGES}
    sweep_skipped = {e: 0 for e in SQUARE_EDGES}
    frames_total = 0
    polarity = ""
    footprint = float("nan")
    reference_masks: dict[str, np.ndarray] = {}
    reference_s: dict[str, float] = {}
    # The nominal pose goes first: its pixels and its read-back pose are the reference of every lateral offset (Step 9).
    # sorted() is stable, so the other poses keep the order of the manifest.
    groups = sorted(group_by_pose(list(records) + list(sweep_records)).items(),
                    key=lambda item: item[1][0].subseries != SUBSERIES_NOMINAL)
    for key, group in groups:
        processed = _process_pose(session, group, options, edges_px, reference_masks)
        if processed is None:
            notes.append(f"pose {key}: target has no back plate; skipped")
            continue
        geometry, stack, pose_edges, pose_slanted = processed
        is_sweep = stack.record().subseries == SUBSERIES_LATERAL_SWEEP
        if is_sweep:
            sweep_axis, from_negative = sweep_axis_and_approach(stack.record(), geometry.pixel_footprint_mm,
                                                                params.lateral_sweep_step_px)
        else:
            polarity = _polarity(geometry.target.kind)
            footprint = geometry.pixel_footprint_mm
            frames_total += stack.frame_count
        for edge, data in pose_edges.items():
            if not reference_s:
                reference_s = {e: d.reference_s_px for e, d in pose_edges.items()}
            if not is_sweep:
                per_edge_bins[edge].append(data.bins)
                if edge in pose_slanted:
                    slanted[edge] = pose_slanted[edge]
            elif sweep_axis != EDGE_ORIENTATION[edge]:
                continue                            # a sweep along H moves no edge of V (and the other way round)
            # Step 9: the pose's own s_50 and its read-back lateral offset across the edge (nominal pose = 0).
            s50 = pose_s50(data.bins, centers, params, options)
            offset = reference_s[edge] - data.reference_s_px
            if s50 is None or not math.isfinite(offset):
                (sweep_skipped if is_sweep else skipped)[edge] += 1
            elif is_sweep:
                sweep_points[edge].append(TransferPoint(offset, s50, from_negative))
            else:
                transfer_points[edge].append(TransferPoint(offset, s50))
    results = []
    for edge in SQUARE_EDGES:
        bins = per_edge_bins[edge]
        if not bins:
            continue
        sum_h = sum(b.sum_h for b in bins)
        n_read = sum(b.n_read for b in bins)
        n_noread = sum(b.n_noread for b in bins)
        esf = esf_from_sums(sum_h, n_read)
        result = EdgeResult(
            target_id=first.target_id, polarity=polarity, gap_mm=float(first.gap_mm), station_z_mm=first.station_z_mm,
            edge=edge, orientation=EDGE_ORIENTATION[edge], poses=len(bins), frames=frames_total,
            pixel_footprint_mm=footprint, bin_centers_px=centers, esf=esf, esf_monotone=monotone_esf(esf, n_read),
            count_read=n_read, count_noread=n_noread, count_total=n_read + n_noread, lsf=np.array([]),
            mtf_frequency=np.array([]), mtf=np.array([]), pose_bins=bins)
        # Step 6, Step 8: rise distance, LSF, MTF, MTF50, s_50.
        result.rise_px, result.s50_px = rise_and_s50(centers, sum_h, n_read, params)
        result.lsf, result.mtf_frequency, result.mtf = lsf_and_mtf(centers, esf, bin_width, options)
        result.mtf50_cycles_per_px = mtf50(result.mtf_frequency, result.mtf)
        # Step 9: the edge position transfer.
        dot_pitch = noise_correlation_length_px(previous, first.station_z_mm, EDGE_ORIENTATION[edge])
        result.transfer = analyze_edge_transfer(transfer_points[edge], sweep_points[edge], dot_pitch, options,
                                                skipped[edge], sweep_skipped[edge])
        # Step 10: bootstrap over poses of the binned sums.
        def statistic(groups: list[PoseBins]) -> np.ndarray:
            return np.array(rise_and_s50(centers, sum(g.sum_h for g in groups), sum(g.n_read for g in groups), params))
        try:
            boot = bootstrap_statistic(bins, statistic, params.bootstrap_resamples, params.confidence_level, rng)
            result.rise_lower_px, result.s50_lower_px = (float(x) for x in boot.lower)
            result.rise_upper_px, result.s50_upper_px = (float(x) for x in boot.upper)
            result.bootstrap_failures = boot.failures
        except ValueError as error:
            result.note = f"bootstrap failed: {error}"
        # Step 5: compare with the slanted-edge ESF.
        slant = slanted[edge]
        result.slanted = slant
        if slant is not None and slant.ok:
            rms, _ = _compare_esfs(esf, n_read, slant.esf, slant.count, options.min_bin_count,
                                   options.min_slanted_bin_count)
            result.esf_rms_difference_h = rms
            result.slanted_s50_difference_px = slant.s50_px - result.s50_px
            slope = np.nanmax(np.abs(np.gradient(result.esf_monotone, bin_width)))
            result.esf_difference_shift_px = rms / slope if slope > 0 and math.isfinite(rms) else float("nan")
            tolerance = params.esf_agreement_bins * bin_width
            result.esf_agreement_ok = bool(abs(result.slanted_s50_difference_px) <= tolerance
                                           and result.esf_difference_shift_px <= tolerance)
        elif slant is not None:
            result.note = (result.note + "; " if result.note else "") + f"slanted-edge ESF failed: {slant.note}"
        if result.transfer.notes:
            result.note = (result.note + "; " if result.note else "") + "; ".join(result.transfer.notes)
        results.append(result)
    return results


def run_lateral_resolution(session: Session, out_dir: str | Path, previous: dict | None = None,
                           options: LateralOptions | None = None) -> LateralResolutionResult | None:
    """Analysis B-HV, Section 11.1 Steps 1 to 10 (module docstring). Returns None when the manifest has no procedure
    B frames. ``out_dir`` is where write_outputs will write (nothing is written here); ``previous`` may hold Analysis A's
    result (``previous["A"]``), whose correlation length is the projector dot pitch of the edge position transfer
    (Step 9; the dot pitch term is skipped, with a note, without it)."""
    options = LateralOptions() if options is None else options
    # The optional lateral sweep (Section 6.1, Step 6) covers only +/- 1 px and would bias the pooled edge spread
    # function of the reference station: it is kept out of the ESF and read separately, in the edge position transfer.
    edge_records = select(session.records, procedure=PROCEDURE_EDGES)
    records = [r for r in edge_records if r.subseries != SUBSERIES_LATERAL_SWEEP]
    sweep_records = [r for r in edge_records if r.subseries == SUBSERIES_LATERAL_SWEEP]
    if not records:
        return None
    params = session.params
    rng = np.random.default_rng(options.bootstrap_seed)
    configurations: dict[tuple, list] = {}
    for record in records:
        configurations.setdefault((record.target_id, record.gap_mm, record.station_z_mm), []).append(record)
    sweeps: dict[tuple, list] = {}
    for record in sweep_records:
        sweeps.setdefault((record.target_id, record.gap_mm, record.station_z_mm), []).append(record)
    notes: list[str] = []
    for key in sweeps:
        if key not in configurations:
            notes.append(f"lateral-sweep poses of {key[0]} (G = {key[1]}, Z = {key[2]:g} mm) have no nominal or jitter "
                         "poses of the same configuration: not analyzed")
    edges: list[EdgeResult] = []
    for key in sorted(configurations, key=lambda k: (k[0], k[1] if k[1] is not None else 0.0, k[2])):
        edges.extend(_analyze_configuration(session, configurations[key], sweeps.get(key, []), options, rng, notes,
                                            previous))
    if not any(e.transfer is not None and e.transfer.sweep_points for e in edges):
        notes.append("no lateral-sweep poses: approach hysteresis and the sweep refit of the periodic terms are not available")
    # Step 7: small-gap against large-gap ESF of the same target, station and edge.
    by_key: dict[tuple, dict[float, EdgeResult]] = {}
    for e in edges:
        by_key.setdefault((e.target_id, e.station_z_mm, e.edge), {})[e.gap_mm] = e
    for gaps in by_key.values():
        small = next((g for g in gaps if abs(g - params.gap_small_mm) < GAP_MATCH_TOLERANCE_MM), None)
        large = next((g for g in gaps if abs(g - params.gap_large_mm) < GAP_MATCH_TOLERANCE_MM), None)
        if small is None or large is None:
            continue
        a, b = gaps[small], gaps[large]
        _, worst = _compare_esfs(a.esf, a.count_read, b.esf, b.count_read, options.min_bin_count)
        flag = bool(worst > params.esf_linearity_tolerance_h) if math.isfinite(worst) else None
        for e in (a, b):
            e.max_delta_h, e.step_height_dependent = worst, flag
    stations = sorted({e.station_z_mm for e in edges})
    reference = min(stations, key=lambda z: abs(z - params.z_reference_mm)) if stations else params.z_reference_mm
    if not any(e.step_height_dependent is not None for e in edges):
        notes.append("the linearity test (Step 7) needs both gaps for a target, station and edge: not available")
    return LateralResolutionResult(edges=edges, bin_width_px=params.boundary_bin_width_px,
                                   band_px=params.boundary_band_half_width_px, reference_station_mm=reference,
                                   notes=notes, options=options)


# ---------------------------------------------------------------------------
# Outputs
# ---------------------------------------------------------------------------
def _grid_figure(rows: int, columns: int):
    """A figure with a rows x columns grid of axes (Agg backend through common.new_figure)."""
    figure, axis = new_figure(FIGURE_PANEL_WIDTH_IN * columns, FIGURE_PANEL_HEIGHT_IN * rows)
    figure.delaxes(axis)
    figure.set_layout_engine("constrained")        # keeps titles, labels and colorbars from overlapping
    return figure, figure.subplots(rows, columns, squeeze=False)


def _series_key(e: EdgeResult) -> tuple:
    return (e.polarity, e.gap_mm)


def _series_style(result: LateralResolutionResult) -> dict[tuple, tuple[str, str]]:
    """(color, line style) of every (polarity, gap) series: color by polarity, line style by gap."""
    polarities = sorted({e.polarity for e in result.edges})
    gaps = sorted({e.gap_mm for e in result.edges})
    return {(p, g): (OKABE_ITO_CYCLE[polarities.index(p) % len(OKABE_ITO_CYCLE)],
                     LINE_STYLES[gaps.index(g) % len(LINE_STYLES)]) for p in polarities for g in gaps}


def _pooled_esf(edges: list[EdgeResult]) -> tuple[np.ndarray, np.ndarray]:
    """Count-weighted mean ESF of several edges and the total read count per bin."""
    total = sum(np.nan_to_num(e.esf) * e.count_read for e in edges)
    count = sum(e.count_read for e in edges)
    return esf_from_sums(total, count), count


def _figure_esf(result: LateralResolutionResult, out_dir: Path) -> list[Path]:
    stations = sorted({e.station_z_mm for e in result.edges})
    style = _series_style(result)
    figure, axes = _grid_figure(len(stations), 2)
    for row, z in enumerate(stations):
        for column, orientation in enumerate((ORIENTATION_H, ORIENTATION_V)):
            axis = axes[row][column]
            for (polarity, gap), (color, line) in style.items():
                chosen = [e for e in result.at_station(z, orientation) if _series_key(e) == (polarity, gap)]
                if not chosen:
                    continue
                esf, _ = _pooled_esf(chosen)
                axis.plot(chosen[0].bin_centers_px, esf, color=color, linestyle=line, label=f"{polarity}, G = {gap:g} mm")
            axis.axhline(0.5, color=OKABE_ITO_BLACK, linewidth=0.5)
            axis.axvline(0.0, color=OKABE_ITO_BLACK, linewidth=0.5)
            axis.set_title(f"Z = {z:g} mm, {orientation} edges", fontsize="small")
            axis.set_xlabel("signed distance s (px, + = front side)")
            axis.set_ylabel("normalized height h")
            axis.grid(True, alpha=0.3)
    axes[0][0].legend(fontsize="x-small")
    return save_figure(figure, out_dir / FIGURE_STEMS["esf"])


def _figure_lsf_mtf(result: LateralResolutionResult, out_dir: Path) -> list[Path]:
    style = _series_style(result)
    stations = sorted({e.station_z_mm for e in result.edges})
    figure, axes = _grid_figure(1, 2)
    for e in result.edges:
        color, line = style[_series_key(e)]
        if e.orientation == ORIENTATION_V:
            line = "-."
        axes[0][0].plot(e.bin_centers_px, e.lsf, color=color, linestyle=line, alpha=0.6, linewidth=0.9)
        axes[0][1].plot(e.mtf_frequency, e.mtf, color=color, linestyle=line, alpha=0.6, linewidth=0.9)
    axes[0][0].set_xlabel("signed distance s (px)")
    axes[0][0].set_ylabel("LSF (1/px)")
    axes[0][1].set_xlabel("spatial frequency (cycles/px)")
    axes[0][1].set_ylabel("MTF")
    axes[0][1].set_xlim(0.0, 0.5)
    axes[0][1].axhline(MTF_HALF_LEVEL, color=OKABE_ITO_BLACK, linewidth=0.5)
    axes[0][0].set_title(f"stations {', '.join(f'{z:g}' for z in stations)} mm; dash-dot = V edges", fontsize="small")
    for axis in axes[0]:
        axis.grid(True, alpha=0.3)
    return save_figure(figure, out_dir / FIGURE_STEMS["lsf_mtf"])


def _figure_versus_z(result: LateralResolutionResult, out_dir: Path, stem: str, keys: tuple[str, str, str],
                     labels: tuple[str, ...], title: str, with_mm: bool) -> list[Path]:
    """Rise distance (or s_50) against Z per orientation, polarity and gap, with the bootstrap interval."""
    style = _series_style(result)
    columns = 2 if with_mm else 1
    figure, axes = _grid_figure(1, columns)
    for e in result.edges:
        if e.edge not in (SQUARE_EDGE_LEFT, SQUARE_EDGE_TOP):
            continue                                            # one marker per orientation and configuration
        color, line = style[_series_key(e)]
        value, lower, upper = (getattr(e, k) for k in keys)
        marker = "o" if e.orientation == ORIENTATION_H else "s"
        yerr = None
        if math.isfinite(lower) and math.isfinite(upper):
            yerr = [[max(value - lower, 0.0)], [max(upper - value, 0.0)]]
        axes[0][0].errorbar([e.station_z_mm], [value], yerr=yerr, color=color, marker=marker, linestyle="none",
                            capsize=2)
        if with_mm:
            axes[0][1].plot([e.station_z_mm], [e.rise_mm], color=color, marker=marker, linestyle="none")
    axes[0][0].set_xlabel("station depth Z (mm)")
    axes[0][0].set_ylabel(labels[0])
    axes[0][0].set_title(title, fontsize="small")
    if with_mm:
        axes[0][1].set_xlabel("station depth Z (mm)")
        axes[0][1].set_ylabel(MM_PER_PX_LABEL)
    for axis in axes[0]:
        axis.grid(True, alpha=0.3)
    handles = [axes[0][0].plot([], [], color=c, linestyle=l, label=f"{p}, G = {g:g} mm")[0]
               for (p, g), (c, l) in style.items()]
    axes[0][0].legend(handles=handles, fontsize="x-small")
    return save_figure(figure, out_dir / FIGURE_STEMS[stem])


def _transfer_details(transfer: EdgeTransfer) -> dict[str, Any]:
    """The edge position transfer of one edge for the details JSON: every pose's point, the skipped counts, both fits (with
    the intercept, the edge bias of Step 8, and the sine and cosine coefficients) and the hysteresis."""
    def fit_dict(fit: TransferFit | None) -> dict[str, Any] | None:
        return None if fit is None else {
            "poses": fit.poses, "gain": fit.gain, "gain_se": fit.gain_se, "intercept_px": fit.intercept_px,
            "pixel_lock_amplitude_px": fit.pixel_lock_amplitude_px,
            "pixel_lock_sine_cosine_px": list(fit.pixel_lock_coefficients),
            "dot_pitch_px": fit.dot_pitch_px, "dot_pitch_amplitude_px": fit.dot_pitch_amplitude_px,
            "dot_pitch_sine_cosine_px": list(fit.dot_pitch_coefficients),
            "hysteresis_coefficient_px": fit.hysteresis_coefficient_px, "residual_rms_px": fit.residual_rms_px,
            "note": fit.note}
    def point_dict(point: TransferPoint) -> dict[str, Any]:
        return {"offset_px": point.offset_px, "s50_px": point.s50_px,
                "approached_from_negative": point.approached_from_negative}
    return {
        "sign_convention": "offset is the displacement of the true edge relative to the nominal pose along s (+ = front "
                           "side); lateral_gain = 1 + slope of s_50 on the offset; hysteresis_px = mean s_50 of the sweep "
                           "poses approached from the negative side minus that from the positive side",
        "dot_pitch_offered_px": transfer.dot_pitch_px, "skipped_poses": transfer.skipped_poses,
        "sweep_skipped_poses": transfer.sweep_skipped_poses, "hysteresis_px": transfer.hysteresis_px,
        "fit": fit_dict(transfer.fit), "sweep_fit": fit_dict(transfer.sweep_fit),
        "points": [point_dict(p) for p in transfer.points],
        "sweep_points": [point_dict(p) for p in transfer.sweep_points], "notes": transfer.notes}


TRANSFER_CURVE_POINTS = 100
"""Points of the smooth fitted pixel-locking curve of the transfer figure."""
TRANSFER_MARKER_SIZE = 6.0
"""Marker size (points squared) of the per-pose points of the transfer figure."""
TRANSFER_SWEEP_MARKER = "x"
"""Marker of the lateral-sweep poses of the transfer figure (the nominal and jitter poses use the orientation's marker)."""
TRANSFER_ORIENTATION_MARKERS = {ORIENTATION_H: "o", ORIENTATION_V: "s"}
"""Marker of the poses of an H edge and of a V edge in the transfer figure (as in the rise and s_50 figures)."""


def _figure_transfer(result: LateralResolutionResult, out_dir: Path) -> list[Path]:
    """The edge position transfer (Step 9), one row per station. Left: the sensed edge position, s_50 plus the read-back
    offset (the edge position in a frame fixed to the sensor, relative to the nominal true edge), against the offset of
    each pose, with the fitted line per edge series and the line of gain 1 (dotted); the sweep poses are crosses. Right:
    the residual about the fitted line against the offset modulo one pixel, with the fitted pixel-locking term."""
    with_transfer = [e for e in result.edges if e.transfer is not None and e.transfer.fit is not None
                     and (e.transfer.points or e.transfer.sweep_points)]
    stations = sorted({e.station_z_mm for e in with_transfer})
    style = _series_style(result)
    figure, axes = _grid_figure(max(len(stations), 1), 2)
    phase_grid = np.linspace(0.0, PIXEL_LOCK_PERIOD_PX, TRANSFER_CURVE_POINTS)
    for row, z in enumerate(stations):
        line_axis, residual_axis = axes[row]
        for e in (e for e in with_transfer if e.station_z_mm == z):
            color, _ = style[_series_key(e)]
            marker = TRANSFER_ORIENTATION_MARKERS[e.orientation]
            transfer, fit = e.transfer, e.transfer.fit
            for points, point_marker in ((transfer.points, marker), (transfer.sweep_points, TRANSFER_SWEEP_MARKER)):
                if not points:
                    continue
                offset = np.array([p.offset_px for p in points])
                s50 = np.array([p.s50_px for p in points])
                line_axis.scatter(offset, s50 + offset, s=TRANSFER_MARKER_SIZE, color=color, marker=point_marker,
                                  alpha=0.7)
                if math.isfinite(fit.gain) and points is transfer.points:
                    line_axis.plot(offset[np.argsort(offset)],
                                   (fit.intercept_px + fit.gain * offset)[np.argsort(offset)], color=color,
                                   linewidth=0.8)
                    residual = s50 - (fit.intercept_px + (fit.gain - 1.0) * offset)
                else:
                    line_fit = np.polyfit(offset, s50, 1) if offset.size >= 2 else (0.0, 0.0)
                    residual = s50 - np.polyval(line_fit, offset)
                residual_axis.scatter(np.mod(offset, PIXEL_LOCK_PERIOD_PX), residual, s=TRANSFER_MARKER_SIZE,
                                      color=color, marker=point_marker, alpha=0.7)
            if math.isfinite(fit.pixel_lock_amplitude_px):
                sine, cosine = fit.pixel_lock_coefficients
                phase = 2.0 * math.pi * phase_grid / PIXEL_LOCK_PERIOD_PX
                residual_axis.plot(phase_grid, sine * np.sin(phase) + cosine * np.cos(phase), color=color,
                                   linewidth=0.8)
        span = line_axis.get_xlim()
        line_axis.plot(span, span, color=OKABE_ITO_BLACK, linestyle=":", linewidth=0.8)
        line_axis.set_xlim(span)
        line_axis.set_title(f"Z = {z:g} mm; circle = H, square = V, cross = sweep", fontsize="small")
        line_axis.set_xlabel("read-back offset across the edge (px)")
        line_axis.set_ylabel("sensed edge position, s_50 + offset (px)")
        residual_axis.set_xlabel("offset modulo 1 px")
        residual_axis.set_ylabel("residual about the line (px)")
        residual_axis.set_title("pixel locking", fontsize="small")
        for axis in (line_axis, residual_axis):
            axis.grid(True, alpha=0.3)
    handles = [axes[0][0].plot([], [], color=c, label=f"{p}, G = {g:g} mm")[0] for (p, g), (c, _) in style.items()]
    axes[0][0].legend(handles=handles, fontsize="x-small")
    return save_figure(figure, out_dir / FIGURE_STEMS["transfer"])


def write_outputs(result: LateralResolutionResult, out_dir: str | Path) -> list[Path]:
    """B_resolution_summary.csv (one row per configuration and edge), B_resolution_details.json (the ESF arrays with
    the per-bin no-read counts and the per-pose points of the edge position transfer) and the figures (ESF curves, LSF and
    MTF, rise distance and s_50 against Z, the edge position transfer)."""
    out_dir = Path(out_dir)
    written = [write_csv_rows(out_dir / SUMMARY_CSV_NAME, [e.csv_row() for e in result.edges], SUMMARY_COLUMNS)]
    details = {
        "bin_width_px": result.bin_width_px, "band_px": result.band_px, "reference_station_mm": result.reference_station_mm,
        "sign_convention": "s is positive on the front-material side; s_50 < 0 means the measured edge lies on the "
                           "back side of the true edge (the front surface is fattened)",
        "forward_model_terms": result.forward_model_terms(), "notes": result.notes,
        "configurations": [{
            "target_id": e.target_id, "polarity": e.polarity, "gap_mm": e.gap_mm, "station_z_mm": e.station_z_mm,
            "edge": e.edge, "orientation": e.orientation, "poses": e.poses, "frames": e.frames,
            "pixel_footprint_mm": e.pixel_footprint_mm, "bin_centers_px": e.bin_centers_px, "esf": e.esf,
            "esf_monotone": e.esf_monotone, "count_read": e.count_read, "count_noread": e.count_noread,
            "count_total": e.count_total, "lsf": e.lsf, "mtf_frequency_cycles_per_px": e.mtf_frequency, "mtf": e.mtf,
            "rise_px": e.rise_px, "rise_mm": e.rise_mm, "mtf50_cycles_per_px": e.mtf50_cycles_per_px, "s50_px": e.s50_px,
            "max_delta_h_between_gaps": e.max_delta_h, "step_height_dependent": e.step_height_dependent,
            "esf_rms_difference_h": e.esf_rms_difference_h, "bootstrap_failures": e.bootstrap_failures,
            "edge_position_transfer": None if e.transfer is None else _transfer_details(e.transfer),
            "slanted": None if e.slanted is None else {
                "ok": e.slanted.ok, "note": e.slanted.note, "centers_px": e.slanted.centers_px, "esf": e.slanted.esf,
                "count": e.slanted.count, "alignment_offset_px": e.slanted.offset_px, "s50_px": e.slanted.s50_px,
                "rise_px": e.slanted.rise_px, "crossings": e.slanted.crossings,
                "line_residual_rms_px": e.slanted.line_residual_rms_px,
                "s50_difference_to_robot_stepped_px": e.slanted_s50_difference_px,
                "equivalent_lateral_difference_px": e.esf_difference_shift_px, "agreement_ok": e.esf_agreement_ok},
            "note": e.note} for e in result.edges],
    }
    written.append(write_json(out_dir / DETAILS_JSON_NAME, details))
    written += _figure_esf(result, out_dir)
    written += _figure_lsf_mtf(result, out_dir)
    written += _figure_versus_z(result, out_dir, "rise", ("rise_px", "rise_lower_px", "rise_upper_px"),
                                ("10-90 percent rise distance (px)",), "circle = H edges, square = V edges", True)
    written += _figure_versus_z(result, out_dir, "s50", ("s50_px", "s50_lower_px", "s50_upper_px"),
                                ("edge offset s_50 (px, + = front side)",), "circle = H edges, square = V edges", False)
    written += _figure_transfer(result, out_dir)
    return written
