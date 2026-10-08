"""
Quick-look check of a capture set (Part I of the procedure) and the D pilot
post check (Section 8, Step 1; Section 13, Step 2).

``check_session`` is meant to be run right after capturing, before the long
analyses. For every pose of the session (all frames sharing a pose key) it
streams the frames, averages them, and compares what the sensor saw with what
the registration says should be there:

    frames          number of capture files of the pose.
    valid fraction  the mean fraction of frames in which the pixels that the
                    registered target should occupy (front or back surface, from
                    ``TwoPlaneTarget.intersect_rays`` at the registered pose) were
                    read. A target that is out of view or flickers scores low.
    border contact  whether valid reads come within ``border_margin_px`` of the
                    image border (informational: a plate or back plate larger than
                    the field always touches it) and how many feature outlines of
                    the registered target cross that border band (flagged: a
                    disk, cutout or square that is cut off cannot be analyzed).
    front plane     a robust plane fitted to the temporal-mean points of the
                    pixels that should see the FRONT surface (eroded by
                    ``classification_margin_px`` so that the matcher's edge
                    mixing does not enter): its RMS residual, its signed
                    distance to the registered front plane (positive = nearer the
                    sensor) and the angle between the two normals.
    back plane      the same for the back plate, where the target has one and
                    enough pixels read it.

Conventions: millimeters and degrees at the interface; image arrays are
(H, W); depth is camera z with NaN for a no-read; planes are point + unit
normal toward the camera (``features.planes``). The registered pose is the
manifest's ``target_pose_camera`` of the first frame of the pose.

A pose is flagged when a threshold of :class:`CheckParameters` is exceeded. The
thresholds are for gross errors (wrong target, wrong registration, target
moved), not for the sensor's own noise and bias, which are what the analyses
measure; the defaults are therefore loose (millimeters, not tenths). Because the depth
noise of a stereo sensor grows with the square of the distance, the stated residual and
distance limits are the limits at the reference station and are multiplied by
``(Z / limit_reference_z_mm) ** limit_depth_exponent`` for a registered target distance Z beyond it
(never reduced below the stated values). The angle limit is the larger of ``normal_warn_deg`` and
``normal_sigma_factor`` times the standard error of the fitted plane's tilt (computed from the number of independent
samples, the fitted pixels divided by the correlation area of their residuals); where that exceeds
``normal_limit_cap_deg`` the tilt is reported as not testable from these frames instead of flagged.

``pilot_post_check`` implements the rule of Section 13, Step 2 on the first frame of each C pose of
the reference station and reports how often a bare support post of the disk plate is detected (it
must not be). The former pilot D_50 and D_0 are gone: the D levels are the fixed (feature, station)
pairs (redesign note, Section 2).
"""
from __future__ import annotations

import json
import math
import struct
import zlib
from collections import OrderedDict
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from scipy import ndimage

from sensorperf.features.planes import MIN_PLANE_POINTS, PlaneFit, fit_plane_robust, plane_depth_image
from sensorperf.geometry.camera import PinholeCamera
from sensorperf.geometry.targets import (
    FEATURE_BLANK, FEATURE_POST, SURFACE_BACK, SURFACE_FRONT, SURFACE_NONE,
    TARGET_KIND_DISK_ARRAY, TARGET_KIND_PLATE, TwoPlaneTarget, window_diameter_mm,
)
from sensorperf.io.capture_set import load_frame_depth
from sensorperf.io.manifest import FrameRecord, SUBSERIES_JITTER, format_file_name, group_by_pose
from sensorperf.io.session import Session
from sensorperf.parameters import CharacterizationParameters, PROCEDURE_AREA, SensorGeometry

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
EXIT_OK = 0
EXIT_FLAGGED = 1
EXIT_INPUT_ERROR = 2
"""Exit codes of the command line tool: nothing flagged, something flagged, the session could not be read."""

REPORT_JSON_INDENT = 2
"""Indentation of the JSON report."""
ADVICE = "re-capture the flagged poses, or check the registration and the target mounting"
"""Advice printed when anything is flagged."""
MIN_PLANE_PIXELS_DEFAULT = 50
"""Fewest pixels a plane fit is attempted with (see CheckParameters.min_plane_pixels)."""
STRUCTURE_SIZE = 3
"""Side of the square structuring element of the mask erosion."""
UNREADABLE_ERRORS = (OSError, ValueError, KeyError, EOFError, zlib.error, struct.error)
"""What reading a damaged or empty capture file can raise (the file system, the header, the decompressor, the
Qt data stream); such a pose is flagged instead of stopping the check."""
CORRELATION_MAX_LAG_PX = 8
"""Largest lag (pixels, along each image axis) of the autocorrelation of the fit residuals."""
CORRELATION_MIN_PIXELS = 4 * (2 * CORRELATION_MAX_LAG_PX + 1) ** 2
"""Fewest fitted pixels for which the autocorrelation is estimated from the residuals (about four per lag)."""
CORRELATION_FALLBACK_AREA_PX = (2 * CORRELATION_MAX_LAG_PX + 1) ** 2 / 4.0
"""Correlation area assumed for a plane with fewer fitted pixels than CORRELATION_MIN_PIXELS (a quarter of the lag window)."""
MINOR_IN_PLANE_AXIS = 1
"""Index of the minor principal axis in the plane among the singular values of centered plane points (descending:
major axis, minor axis, normal)."""
FRAME_SEPARATOR = "_f"
"""Separator of the pose part and the frame index in a Section 9 file name."""

FLAG_UNREADABLE = "capture files could not be read"
FLAG_NO_TARGET = "target id is not in targets.json"
FLAG_NOT_IN_VIEW = "registered target does not appear in the field of view"
FLAG_LOW_VALID = "low valid fraction (target flickers or is not read)"
FLAG_FEATURE_CUT = "feature outline crosses the image border (target partly out of view)"
FLAG_FRONT_UNREAD = "front surface should be visible but too few pixels were read for a plane fit"
FLAG_FRONT_RMS = "front plane fit residual too large"
FLAG_FRONT_OFFSET = "front plane is far from the registered front plane"
FLAG_FRONT_TILT = "front plane normal disagrees with the registered normal"
FLAG_BACK_RMS = "back plane fit residual too large"
FLAG_BACK_OFFSET = "back plane is far from the registered back plane"
FLAG_BACK_TILT = "back plane normal disagrees with the registered normal"

LIMIT_SCALE_FLOOR = 1.0
"""The depth scaling of the residual limits never goes below this (the stated limits are the tightest)."""
SURFACE_FRONT_LABEL = "front"
SURFACE_BACK_LABEL = "back"
"""Names of the two surfaces in the notes of a pose."""
NOTE_SEPARATOR = "; "
"""Separator of the flags and notes on a printed pose line."""


# ---------------------------------------------------------------------------
# Parameters and report records
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class CheckParameters:
    """Thresholds of the check; the command-line defaults come from here."""

    min_valid_fraction: float = 0.5
    """Flag a pose in which fewer than this fraction of the expected target pixels were read; pixels read in
    fewer than this fraction of the frames are also left out of the temporal-mean points of the fits."""
    border_margin_px: int = 4
    """Reads this close to the image border count as border contact, and a feature outline this close to it
    (or beyond) counts as cut off."""
    plane_residual_warn_mm: float = 3.0
    """Flag a plane whose fit RMS exceeds this at the reference station (scaled with depth beyond it, see
    ``limit_depth_exponent``). Loose on purpose: a single frame at Z_MAX has a sigma_t of millimeters with the
    indicative sensor; the check is for gross errors such as a wrongly classified surface."""
    pose_residual_warn_mm: float = 3.0
    """Flag a plane whose signed distance to the registered plane exceeds this at the reference station (scaled
    with depth beyond it; the sensor's own depth bias is what the analyses measure, so this catches a wrong
    registration or a target that moved, not the bias)."""
    normal_warn_deg: float = 2.0
    """Flag a plane whose normal differs from the registered one by more than this, or by more than
    ``normal_sigma_factor`` standard errors of the fitted tilt when that is larger (the angle limit is never
    tighter than this value)."""
    limit_reference_z_mm: float = CharacterizationParameters().z_reference_mm
    """Registered target distance at which the residual limits hold as stated (the reference station, Z_REFERENCE_MM;
    the command line takes it from the session's parameters.json unless given)."""
    limit_depth_exponent: float = 2.0
    """Exponent of the depth scaling of the residual limits: the plane-residual and pose-offset limits are multiplied
    by max(1, (Z / limit_reference_z_mm) ** limit_depth_exponent). The default 2 is the depth-noise law of a stereo
    sensor (sigma_Z grows with Z squared); 0 gives fixed limits."""
    normal_sigma_factor: float = 4.0
    """The tilt limit of a plane is at least this many standard errors of its fitted tilt (from the fit RMS, the
    spread of the fitted points and their effective count N / A_corr, A_corr being the correlation area of the
    residuals measured from the fit), so that a plane fitted from few noisy pixels is not judged
    against an angle it cannot resolve."""
    normal_limit_cap_deg: float = 10.0
    """When the tilt limit of a plane (normal_sigma_factor standard errors) exceeds this, the fit cannot resolve a
    tilt worth testing: no tilt flag is raised and the plane is reported as 'tilt not testable'."""
    classification_margin_px: int = 4
    """The expected-front and expected-back pixel masks are eroded by this many pixels before a plane is fitted
    (not at the image border), because reads next to a surface boundary mix the two surfaces."""
    min_plane_pixels: int = MIN_PLANE_PIXELS_DEFAULT
    """A plane is fitted only to at least this many pixels; with fewer the fit is skipped (and flagged for the
    front surface when the registered geometry says that many pixels should be visible)."""


@dataclass
class PlaneComparison:
    """A fitted plane against the registered one."""

    pixels: int = 0
    rms_mm: float = float("nan")
    offset_mm: float = float("nan")
    """Signed distance of the fitted plane to the registered plane at the fit centroid; positive = nearer the sensor."""
    angle_deg: float = float("nan")
    sigma_angle_deg: float = float("nan")
    """Standard error of the fitted plane's tilt (degrees), from the fit RMS, the spread of the fitted points along the
    minor in-plane axis and their effective number; infinite when the points do not span a plane."""
    correlation_area_px: float = float("nan")
    """Correlation area A_corr of the residuals (pixels): the sum of the positive values of their normalized 2-D
    autocorrelation over the lags of +/- CORRELATION_MAX_LAG_PX, at least 1 (1 = independent pixels)."""
    effective_pixels: float = float("nan")
    """N_eff = N / A_corr, the number of independent samples the tilt standard error is based on."""
    correlation_estimated: bool = False
    """True when A_corr was measured from the residuals; False when too few pixels were fitted and the fallback
    CORRELATION_FALLBACK_AREA_PX stands in."""
    residual_limit_mm: float = float("nan")
    """The limit on ``rms_mm`` applied to this plane (the stated limit scaled for the depth of the target)."""
    offset_limit_mm: float = float("nan")
    """The limit on ``|offset_mm|`` applied to this plane (the stated limit scaled for the depth of the target)."""
    normal_limit_deg: float | None = None
    """The limit on ``angle_deg`` applied to this plane; None when the plane was not fitted or its tilt is not testable."""
    normal_untestable: bool = False
    """True when the fit cannot resolve a tilt worth testing (limit above ``normal_limit_cap_deg``); no tilt flag is raised."""


@dataclass
class PoseCheck:
    """Measurements and flags of one pose."""

    pose_key: tuple
    name: str
    """The Section 9 file name of the pose without frame index and extension."""
    target_id: str
    gap_mm: float | None
    frames: int = 0
    valid_fraction: float = float("nan")
    touches_border: bool = False
    features_cut: int = 0
    front: PlaneComparison = field(default_factory=PlaneComparison)
    back: PlaneComparison = field(default_factory=PlaneComparison)
    flags: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    """Information that does not flag the pose, such as a tilt that could not be tested."""
    target_z_mm: float = float("nan")
    """Registered distance of the target (z of its pose in the camera frame), the Z of the limit scaling."""


@dataclass
class CheckReport:
    """The result of :func:`check_session`."""

    poses: list[PoseCheck]
    parameters: CheckParameters
    notes: list[str] = field(default_factory=list)

    @property
    def flagged(self) -> list[PoseCheck]:
        """The poses with at least one flag."""
        return [p for p in self.poses if p.flags]

    def verdict(self) -> str:
        """The final verdict line: how many poses are flagged and why."""
        flagged = self.flagged
        if not flagged:
            return f"VERDICT: all {len(self.poses)} poses passed."
        reasons: dict[str, int] = {}
        for pose in flagged:
            for flag in pose.flags:
                reasons[flag.split(":")[0]] = reasons.get(flag.split(":")[0], 0) + 1
        parts = [f"{count} x {reason}" for reason, count in reasons.items()]
        return f"VERDICT: {len(flagged)} of {len(self.poses)} poses flagged ({'; '.join(parts)}). Advice: {ADVICE}."

    def format_table(self) -> str:
        """Console table of all poses."""
        width = max([len("pose")] + [len(p.name) for p in self.poses])
        header = (f"{'pose':<{width}} {'frames':>6} {'valid':>6} {'border':>6} {'cut':>3} {'f_rms':>7} {'f_dist':>7} "
                  f"{'f_deg':>6} {'b_rms':>7} {'b_dist':>7} {'b_deg':>6}  flags")
        lines = [header, "-" * len(header)]
        for p in self.poses:
            lines.append(
                f"{p.name:<{width}} {p.frames:>6d} {_number(p.valid_fraction, '.2f'):>6} "
                f"{'yes' if p.touches_border else 'no':>6} {p.features_cut:>3d} {_number(p.front.rms_mm, '.3f'):>7} "
                f"{_number(p.front.offset_mm, '.3f'):>7} {_number(p.front.angle_deg, '.2f'):>6} "
                f"{_number(p.back.rms_mm, '.3f'):>7} {_number(p.back.offset_mm, '.3f'):>7} "
                f"{_number(p.back.angle_deg, '.2f'):>6}  {_flags_text(p)}")
        return "\n".join(lines)

    def to_dict(self) -> dict[str, Any]:
        """The JSON report: parameters, one entry per pose, notes, count flagged and the verdict (NaN becomes null)."""
        return json_safe({
            "parameters": asdict(self.parameters),
            "poses": [{"pose": p.name, "target_id": p.target_id, "gap_mm": p.gap_mm, "frames": p.frames,
                       "valid_fraction": p.valid_fraction, "touches_border": p.touches_border,
                       "features_cut": p.features_cut, "target_z_mm": p.target_z_mm, "front": asdict(p.front),
                       "back": asdict(p.back), "flags": p.flags, "notes": p.notes} for p in self.poses],
            "notes": self.notes, "n_poses": len(self.poses), "n_flagged": len(self.flagged),
            "verdict": self.verdict()})

    def write_json(self, path: str | Path) -> Path:
        """Write the JSON report."""
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(self.to_dict(), indent=REPORT_JSON_INDENT), encoding="utf-8")
        return Path(path)


def _flags_text(pose: PoseCheck) -> str:
    """The last column of a pose line: the flags, or ok, followed by the notes of the pose in parentheses."""
    text = NOTE_SEPARATOR.join(pose.flags) if pose.flags else "ok"
    return f"{text} ({NOTE_SEPARATOR.join(pose.notes)})" if pose.notes else text


def _number(value: float, spec: str) -> str:
    """Formatted number, or a dash for NaN."""
    return "-" if not np.isfinite(value) else format(value, spec)


def json_safe(value: Any) -> Any:
    """Convert numpy values to JSON types; NaN and infinity become null."""
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, (np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


# ---------------------------------------------------------------------------
# Per-pose measurements
# ---------------------------------------------------------------------------
def border_touched(valid: np.ndarray, margin_px: int) -> bool:
    """True when any True pixel of the (H, W) mask lies within margin_px of the image border."""
    if margin_px <= 0:
        return False
    inner = valid[margin_px:valid.shape[0] - margin_px, margin_px:valid.shape[1] - margin_px]
    return bool(valid.sum() > inner.sum())


def features_cut_by_border(target: TwoPlaneTarget, camera: PinholeCamera, pose, margin_px: int) -> int:
    """Number of feature outlines of a target (not plates) that cross the image border band at this pose."""
    if target.kind == TARGET_KIND_PLATE:
        return 0
    cut = 0
    for feature in target.features:
        if feature.kind == FEATURE_POST:
            continue
        outline = target.project_outline(camera, pose, feature)
        u, v = outline[:, 0], outline[:, 1]
        if (np.any(~np.isfinite(u)) or u.min() < margin_px or u.max() > camera.width - 1 - margin_px
                or v.min() < margin_px or v.max() > camera.height - 1 - margin_px):
            cut += 1
    return cut


def depth_limit_scale(z_mm: float, params: CheckParameters) -> float:
    """Factor of the residual limits at a registered target distance: max(1, (Z / limit_reference_z_mm) ** exponent).
    The depth noise of a stereo sensor grows with Z squared, so a fixed millimeter limit would flag normal
    behavior at the far stations; the factor is never below 1, so the stated limits are the tightest. A target
    distance that is not a positive finite number (nothing to scale with) leaves the limits as stated."""
    if not (np.isfinite(z_mm) and z_mm > 0.0 and params.limit_reference_z_mm > 0.0):
        return LIMIT_SCALE_FLOOR
    return max(LIMIT_SCALE_FLOOR, (z_mm / params.limit_reference_z_mm) ** params.limit_depth_exponent)


def residual_correlation_area(residual_map: np.ndarray) -> float:
    """Correlation area (pixels) of the residuals of a plane fit.

    ``residual_map`` is an (H, W) array of the residuals in mm, NaN outside the fitted pixels. Its autocorrelation at
    lag (dy, dx) is the mean product of the residuals of the pixel pairs that are both fitted (normalized by the count
    of such pairs), divided by the mean square residual, so that lag zero is 1. The area is the sum of the positive
    values over all lags of +/- CORRELATION_MAX_LAG_PX along each axis (the lag-zero 1 included), at least 1: it is how
    many pixels share one independent sample of the noise (a 4 px block of correlated noise gives about 16 to 25).
    The products are computed with FFTs of the bounding box of the fitted pixels, padded so that no lag wraps around."""
    fitted = np.isfinite(residual_map)
    rows, cols = np.nonzero(fitted)
    box = (slice(rows.min(), rows.max() + 1), slice(cols.min(), cols.max() + 1))
    mask = fitted[box].astype(np.float64)
    values = np.where(fitted[box], residual_map[box], 0.0)
    shape = (mask.shape[0] + CORRELATION_MAX_LAG_PX, mask.shape[1] + CORRELATION_MAX_LAG_PX)

    def autocorrelate(image: np.ndarray) -> np.ndarray:
        spectrum = np.fft.rfft2(image, s=shape)
        return np.fft.irfft2(spectrum * np.conj(spectrum), s=shape)

    products, counts = autocorrelate(values), autocorrelate(mask)
    variance = products[0, 0] / counts[0, 0]
    if not variance > 0.0:
        return 1.0
    lags = np.arange(-CORRELATION_MAX_LAG_PX, CORRELATION_MAX_LAG_PX + 1)
    area = 0.0
    for dy in lags:
        for dx in lags:
            count = counts[dy, dx]                      # negative lags wrap to the far end of the padded array
            if count > 0.5:                             # the counts are integers up to FFT round-off
                area += max(products[dy, dx] / count / variance, 0.0)
    return max(area, 1.0)


def tilt_standard_error_deg(points: np.ndarray, rms_mm: float, correlation_area_px: float = 1.0) -> float:
    """Standard error (degrees) of the tilt of a plane fitted to ``points`` (N, 3, mm) with residual RMS ``rms_mm``.

    The slope of a fitted plane along an in-plane direction has the standard error
    sigma_slope = rms / (s * sqrt(N_eff)), where s is the RMS spread of the points about their centroid along that
    direction and N_eff = N / correlation_area_px the number of independent samples (N for independent pixels; the
    depth noise of a stereo sensor is correlated over its matching window, see :func:`residual_correlation_area`).
    The least favorable direction is the minor principal axis in the plane (the second singular value of the
    centered points, the first being the major axis and the third the normal), so s = s_minor is used.
    The angle is atan(sigma_slope). Infinite when the points are too few or too collinear to span a plane."""
    count = len(points)
    if count < MIN_PLANE_POINTS:
        return float("inf")
    singular_values = np.linalg.svd(points - points.mean(axis=0), compute_uv=False)
    s_minor = singular_values[MINOR_IN_PLANE_AXIS] / math.sqrt(count)      # RMS spread along the minor in-plane axis
    if not s_minor > 0.0:
        return float("inf")
    effective = count / max(correlation_area_px, 1.0)
    return math.degrees(math.atan(rms_mm / (s_minor * math.sqrt(effective))))


def _compare_plane(mask: np.ndarray, mean_depth: np.ndarray, valid_enough: np.ndarray, camera: PinholeCamera,
                   registered: tuple[np.ndarray, np.ndarray], params: CheckParameters,
                   target_z_mm: float = float("nan")) -> PlaneComparison:
    """Fit a plane to the mean points under the (eroded) mask and compare it with the registered plane
    (point, unit normal toward the sensor), and set the limits that apply to it: the residual and offset limits
    scaled for the depth ``target_z_mm`` of the target, and the tilt limit from the precision of the fit.
    Returns pixels = 0 and NaNs when too few pixels are available."""
    eroded = ndimage.binary_erosion(mask, structure=np.ones((STRUCTURE_SIZE, STRUCTURE_SIZE), dtype=bool),
                                    iterations=params.classification_margin_px, border_value=1) \
        if params.classification_margin_px > 0 else mask
    chosen = eroded & valid_enough & np.isfinite(mean_depth)
    comparison = PlaneComparison(pixels=int(chosen.sum()))
    if comparison.pixels < params.min_plane_pixels:
        return comparison
    u, v = camera.pixel_grid()
    points = camera.back_project(u[chosen], v[chosen], mean_depth[chosen])
    fit: PlaneFit = fit_plane_robust(points)
    point, normal = registered
    comparison.rms_mm = fit.rms_mm
    comparison.offset_mm = float(normal @ (fit.point - point))
    comparison.angle_deg = float(np.degrees(np.arccos(np.clip(fit.normal @ normal, -1.0, 1.0))))
    # Limits: the residual limits grow with the depth noise; the tilt limit follows what this fit can resolve.
    scale = depth_limit_scale(target_z_mm, params)
    comparison.residual_limit_mm = params.plane_residual_warn_mm * scale
    comparison.offset_limit_mm = params.pose_residual_warn_mm * scale
    inlier_points = points[fit.inliers]
    comparison.correlation_estimated = len(inlier_points) >= CORRELATION_MIN_PIXELS
    if comparison.correlation_estimated:
        residual_map = np.full(mask.shape, np.nan)
        residual_map[np.nonzero(chosen)[0][fit.inliers], np.nonzero(chosen)[1][fit.inliers]] = fit.distance(inlier_points)
        comparison.correlation_area_px = residual_correlation_area(residual_map)
    else:
        comparison.correlation_area_px = CORRELATION_FALLBACK_AREA_PX
    comparison.effective_pixels = len(inlier_points) / comparison.correlation_area_px
    comparison.sigma_angle_deg = tilt_standard_error_deg(inlier_points, fit.rms_mm, comparison.correlation_area_px)
    normal_limit = max(params.normal_warn_deg, params.normal_sigma_factor * comparison.sigma_angle_deg)
    if normal_limit > params.normal_limit_cap_deg:
        comparison.normal_untestable = True                            # no tilt limit: the tilt is not judged
    else:
        comparison.normal_limit_deg = normal_limit
    return comparison


def _plane_flags(comparison: PlaneComparison, params: CheckParameters, rms_flag: str, offset_flag: str,
                 tilt_flag: str) -> list[str]:
    """The flags a plane comparison earns against the limits stored in it (see :func:`_compare_plane`)."""
    flags = []
    if np.isfinite(comparison.rms_mm) and comparison.rms_mm > comparison.residual_limit_mm:
        flags.append(rms_flag)
    if np.isfinite(comparison.offset_mm) and abs(comparison.offset_mm) > comparison.offset_limit_mm:
        flags.append(offset_flag)
    if (np.isfinite(comparison.angle_deg) and comparison.normal_limit_deg is not None
            and comparison.angle_deg > comparison.normal_limit_deg):
        flags.append(tilt_flag)
    return flags


def _untestable_note(label: str, comparison: PlaneComparison) -> list[str]:
    """The note of a plane whose tilt cannot be tested from the frames of the pose (empty list otherwise)."""
    if not comparison.normal_untestable:
        return []
    return [f"{label} tilt not testable: {comparison.pixels} px, sigma {comparison.sigma_angle_deg:.1f} deg"
            + ("" if comparison.correlation_estimated else " (assumed correlation area)")]


def pose_name(record: FrameRecord) -> str:
    """The Section 9 file name of the pose without the frame index and extension."""
    first_frame = format_file_name(record.procedure, record.target_id, record.gap_mm, record.station_z_mm,
                                   record.field, record.pose_index, 0)
    return first_frame.rsplit(FRAME_SEPARATOR, 1)[0]


def check_pose(session: Session, records: Sequence[FrameRecord], params: CheckParameters) -> PoseCheck:
    """Measure and flag one pose (the records of its frames, in frame order)."""
    first = records[0]
    check = PoseCheck(pose_key=first.pose_key(), name=pose_name(first), target_id=first.target_id,
                      gap_mm=first.gap_mm, frames=len(records))
    template = session.targets.targets.get(first.target_id)
    if template is None:
        check.flags.append(FLAG_NO_TARGET)
        return check
    # The target as mounted: the manifest's gap (None = no back plate, e.g. the open-background variant).
    target = template.with_gap(first.gap_mm)
    pose = first.target_pose_camera
    check.target_z_mm = float(pose.translation[2])         # the registered distance that scales the residual limits
    depth_sum = depth_count = expected = None
    camera: PinholeCamera | None = None
    valid_fractions: list[float] = []
    try:
        for record in records:
            depth, _, header = load_frame_depth(record.path)
            if camera is None:
                camera = PinholeCamera.from_matcloud_header(header, width_px=depth.shape[1], height_px=depth.shape[0])
                hit = target.intersect_rays(pose, np.zeros(3), camera.ray_directions())
                expected = hit.surface
                depth_sum = np.zeros(depth.shape)
                depth_count = np.zeros(depth.shape, dtype=np.int64)
            finite = np.isfinite(depth)
            depth_sum += np.where(finite, depth, 0.0)
            depth_count += finite
            seen = expected != SURFACE_NONE
            if seen.any():
                valid_fractions.append(float(finite[seen].mean()))
    except UNREADABLE_ERRORS as error:
        check.flags.append(f"{FLAG_UNREADABLE}: {error}")
        return check
    check.frames = len(records)
    with np.errstate(invalid="ignore", divide="ignore"):
        mean_depth = np.where(depth_count > 0, depth_sum / np.maximum(depth_count, 1), np.nan)
    valid_enough = depth_count >= params.min_valid_fraction * len(records)
    check.touches_border = border_touched(depth_count > 0, params.border_margin_px)
    check.features_cut = features_cut_by_border(target, camera, pose, params.border_margin_px)
    if not valid_fractions:
        check.flags.append(FLAG_NOT_IN_VIEW)
        return check
    check.valid_fraction = float(np.mean(valid_fractions))
    if check.valid_fraction < params.min_valid_fraction:
        check.flags.append(FLAG_LOW_VALID)
    if check.features_cut:
        check.flags.append(f"{FLAG_FEATURE_CUT} ({check.features_cut})")
    front_mask = expected == SURFACE_FRONT
    check.front = _compare_plane(front_mask, mean_depth, valid_enough, camera, target.front_plane_camera(pose), params,
                                 check.target_z_mm)
    check.flags += _plane_flags(check.front, params, FLAG_FRONT_RMS, FLAG_FRONT_OFFSET, FLAG_FRONT_TILT)
    check.notes += _untestable_note(SURFACE_FRONT_LABEL, check.front)
    if check.front.pixels < params.min_plane_pixels and _eroded_count(front_mask, params) >= params.min_plane_pixels:
        check.flags.append(FLAG_FRONT_UNREAD)
    if target.gap_mm is not None:
        back_mask = expected == SURFACE_BACK
        check.back = _compare_plane(back_mask, mean_depth, valid_enough, camera, target.back_plane_camera(pose), params,
                                    check.target_z_mm)
        check.flags += _plane_flags(check.back, params, FLAG_BACK_RMS, FLAG_BACK_OFFSET, FLAG_BACK_TILT)
        check.notes += _untestable_note(SURFACE_BACK_LABEL, check.back)
    return check


def _eroded_count(mask: np.ndarray, params: CheckParameters) -> int:
    """Pixels of a mask left after the classification erosion (how many the geometry says should be usable)."""
    if params.classification_margin_px <= 0:
        return int(mask.sum())
    return int(ndimage.binary_erosion(mask, structure=np.ones((STRUCTURE_SIZE, STRUCTURE_SIZE), dtype=bool),
                                      iterations=params.classification_margin_px, border_value=1).sum())


def check_session(session: Session, check_params: CheckParameters | None = None) -> CheckReport:
    """The quick-look check of every pose of the session (module docstring): frames, valid fraction, border
    contact, the front-plane fit against the registered front plane (RMS, distance, angle) and the back-plane
    fit where the target has a back plate and enough pixels read it, with the flags of ``check_params`` (the
    residual and distance limits scaled for the depth of each target, the tilt limit from the precision of each fit).
    Frames are streamed one at a time, so memory use does not grow with the frame count."""
    if check_params is None:
        # Without options the reference station of the limits is the session's own (parameters.json, else the default).
        check_params = CheckParameters(limit_reference_z_mm=session.params.z_reference_mm)
    groups = group_by_pose(session.records)
    poses = [check_pose(session, records, check_params) for records in groups.values()]
    notes = []
    if session.registration is None:
        notes.append("no registration.json: the manifest's target poses are used as they are")
    assumed = sum(1 for p in poses for c in (p.front, p.back) if c.pixels and not c.correlation_estimated)
    if assumed:
        notes.append(f"{assumed} plane(s) have fewer than {CORRELATION_MIN_PIXELS} fitted pixels, too few to measure the "
                     f"correlation of their residuals: a correlation area of {CORRELATION_FALLBACK_AREA_PX:g} px is assumed")
    untestable = sum(p.front.normal_untestable + p.back.normal_untestable for p in poses)
    if untestable:
        notes.append(f"the tilt of {untestable} plane(s) is not testable from the frames of its pose (limit above "
                     f"{check_params.normal_limit_cap_deg:g} deg, see the notes of the pose in check.json); the offset "
                     "and residual of those planes are still checked")
    return CheckReport(poses=poses, parameters=check_params, notes=notes)


# ---------------------------------------------------------------------------
# D pilot: the post check (Section 8, Step 1; Section 13, Step 2)
# ---------------------------------------------------------------------------
@dataclass
class PostCheck:
    """The post check of one (disk plate, gap) at the reference station: a bare support post must not be detected."""

    target_id: str
    gap_mm: float | None
    station_z_mm: float
    poses: int = 0
    """C poses (first frames) used."""
    tau_mm: float | None = None
    """Threshold: the (1 - DETECTION_FALSE_ALARM_TARGET) quantile of the deviation over the blank sites."""
    blank_pixels: int = 0
    """Valid reads in the blank-site windows that set the threshold."""
    post_trials: int = 0
    post_detections: int = 0
    post_fraction: float | None = None
    """Fraction of post-only sites in which the detection rule fires (should be about the false-alarm target)."""
    notes: list[str] = field(default_factory=list)


def _window_deviation(depth: np.ndarray, reference_deviation: np.ndarray, u0: float, v0: float, radius_px: float):
    """(deviation values inside the circular window as a cropped 2-D array with NaN outside the circle or without
    a read, True) or (None, False) when the window does not touch the image."""
    height, width = depth.shape
    row_low, row_high = max(int(math.floor(v0 - radius_px)), 0), min(int(math.ceil(v0 + radius_px)) + 1, height)
    col_low, col_high = max(int(math.floor(u0 - radius_px)), 0), min(int(math.ceil(u0 + radius_px)) + 1, width)
    if row_low >= row_high or col_low >= col_high:
        return None, False
    rows, cols = np.mgrid[row_low:row_high, col_low:col_high]
    inside = (cols - u0) ** 2 + (rows - v0) ** 2 <= radius_px ** 2
    crop = np.where(inside, reference_deviation[row_low:row_high, col_low:col_high], np.nan)
    return crop, True


def pilot_post_check(session: Session, params: CharacterizationParameters, geometry: SensorGeometry,
                     station_z_mm: float | None = None, subseries: Sequence[str] = (SUBSERIES_JITTER,)
                     ) -> dict[tuple[str, float | None], PostCheck]:
    """The D pilot of Section 8, Step 1, which keeps only the post check (the redesign note, Section 2: the selection of
    detection levels from a pilot D_50 is gone, the levels are the fixed (feature, station) pairs). Applies the
    detection rule of Section 13, Step 2 to the first frame of every C pose at ``station_z_mm`` (default Z_REFERENCE_MM,
    sub-series ``subseries``, default the main "jitter" poses) of the disk plate, for each (target_id, gap).

    The window of a site is the disk of radius D_px / 2 + DETECTION_WINDOW_MARGIN_PX around its projected center,
    D_px = D f_x / Z. The deviation of a read from the reference plane, taken from the registered geometry, is
    Z_back - Z (positive toward the sensor). The threshold tau is the (1 - DETECTION_FALSE_ALARM_TARGET) quantile of that
    deviation over the valid reads in the blank-site windows of all the poses of the group (the distribution of false
    deviations). A post-only site is detected in a pose when at least DETECTION_MIN_CONNECTED_PX connected
    (scipy.ndimage.label) pixels of its window have valid reads with deviation above tau. The post check passes when the
    fraction of detected post sites is no larger than about the false-alarm target: a bare post must not look like a
    disk. Groups without a back plate (gap None) and plates without posts are skipped."""
    station = params.z_reference_mm if station_z_mm is None else station_z_mm
    groups: "OrderedDict[tuple, dict[tuple, FrameRecord]]" = OrderedDict()
    for record in session.records:
        if record.procedure != PROCEDURE_AREA or record.subseries not in tuple(subseries):
            continue
        if abs(record.station_z_mm - station) > params.station_match_tolerance_mm:
            continue
        poses = groups.setdefault((record.target_id, record.gap_mm), {})
        current = poses.get(record.pose_key())
        if current is None or record.frame_index < current.frame_index:
            poses[record.pose_key()] = record               # keep the first frame of each pose
    results: dict[tuple[str, float | None], PostCheck] = {}
    for (target_id, gap), poses in groups.items():
        template = session.targets.targets.get(target_id)
        if template is None or template.kind != TARGET_KIND_DISK_ARRAY:
            continue
        result = PostCheck(target_id=target_id, gap_mm=gap, station_z_mm=station)
        results[(target_id, gap)] = result
        if gap is None:
            result.notes.append("no back plate: the reference plane of the detection rule is missing, group skipped")
            continue
        _count_posts(session, params, geometry, template.with_gap(gap), list(poses.values()), result)
    return results


def _count_posts(session: Session, params: CharacterizationParameters, geometry: SensorGeometry,
                 target: TwoPlaneTarget, records: list[FrameRecord], result: PostCheck) -> None:
    """Fill a PostCheck from the first frames of the C poses of one (disk plate, gap) (see pilot_post_check)."""
    windows: list[tuple[str, np.ndarray]] = []                          # (feature kind, deviation crop)
    for record in records:
        try:
            depth, _, header = load_frame_depth(record.path)
        except UNREADABLE_ERRORS as error:
            result.notes.append(f"{record.path.name}: {error}")
            continue
        camera = PinholeCamera.from_matcloud_header(header, width_px=depth.shape[1], height_px=depth.shape[0])
        pose = record.target_pose_camera
        back_depth = plane_depth_image(camera, *target.back_plane_camera(pose))
        deviation = back_depth - depth                                  # NaN where there is no read
        result.poses += 1
        for feature in target.features:
            if feature.kind not in (FEATURE_BLANK, FEATURE_POST):
                continue
            center = target.feature_center_camera(pose, feature)
            u0, v0, in_front = camera.project(center)
            if not bool(in_front):
                continue
            # A blank site is cut to the size of the feature it serves, as in the detection analysis.
            radius_px = geometry.diameter_in_pixels(window_diameter_mm(target, feature), float(center[2])) / 2.0 \
                + params.detection_window_margin_px
            crop, touches = _window_deviation(depth, deviation, float(u0), float(v0), radius_px)
            if touches:
                windows.append((feature.kind, crop))
    blank = np.concatenate([crop[np.isfinite(crop)] for kind, crop in windows if kind == FEATURE_BLANK]
                           or [np.empty(0)])
    result.blank_pixels = int(blank.size)
    if blank.size == 0:
        result.notes.append("no valid reads in the blank-site windows, so the threshold tau cannot be set")
        return
    result.tau_mm = float(np.quantile(blank, 1.0 - params.detection_false_alarm_target))
    for kind, crop in windows:
        if kind != FEATURE_POST:
            continue
        labeled, count = ndimage.label(np.isfinite(crop) & (crop > result.tau_mm))
        detected = count > 0 and int(np.bincount(labeled.ravel())[1:].max()) >= params.detection_min_connected_px
        result.post_trials += 1
        result.post_detections += int(detected)
    if result.post_trials:
        result.post_fraction = result.post_detections / result.post_trials


def format_post_check_table(results: dict[tuple[str, float | None], PostCheck]) -> str:
    """Console text of the post check: per (disk plate, gap) the threshold and the post-site detection fraction."""
    if not results:
        return "No C poses of the requested station and sub-series were found on a disk plate."
    lines = []
    for (target_id, gap), r in results.items():
        gap_text = "none" if gap is None else f"{gap:g} mm"
        tau = "n/a" if r.tau_mm is None else f"{r.tau_mm:.3f} mm"
        post = "n/a" if r.post_fraction is None else f"{r.post_fraction:.2f} ({r.post_detections}/{r.post_trials})"
        lines.append(f"{target_id}  G = {gap_text}  Z = {r.station_z_mm:g} mm: {r.poses} poses, tau = {tau} "
                     f"({r.blank_pixels} blank-site reads), post-site detection fraction = {post}")
        lines += [f"    NOTE: {note}" for note in r.notes]
    return "\n".join(lines)
