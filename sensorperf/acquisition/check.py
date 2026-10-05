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
measure; the defaults are therefore loose (millimeters, not tenths).

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

from sensorperf.features.planes import PlaneFit, fit_plane_robust, plane_depth_image
from sensorperf.geometry.camera import PinholeCamera
from sensorperf.geometry.targets import (
    FEATURE_BLANK, FEATURE_POST, SURFACE_BACK, SURFACE_FRONT, SURFACE_NONE,
    TARGET_KIND_DISK_ARRAY, TARGET_KIND_PLATE, TwoPlaneTarget,
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
    """Flag a plane whose fit RMS exceeds this. Loose on purpose: a single frame at Z_MAX has a sigma_t of
    millimeters with the indicative sensor; the check is for gross errors such as a wrongly classified surface."""
    pose_residual_warn_mm: float = 3.0
    """Flag a plane whose signed distance to the registered plane exceeds this (the sensor's own depth bias is
    what the analyses measure, so this catches a wrong registration or a target that moved, not the bias)."""
    normal_warn_deg: float = 2.0
    """Flag a plane whose normal differs from the registered one by more than this."""
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
                f"{_number(p.back.angle_deg, '.2f'):>6}  {'; '.join(p.flags) if p.flags else 'ok'}")
        return "\n".join(lines)

    def to_dict(self) -> dict[str, Any]:
        """The JSON report: parameters, one entry per pose, notes, count flagged and the verdict (NaN becomes null)."""
        return json_safe({
            "parameters": asdict(self.parameters),
            "poses": [{"pose": p.name, "target_id": p.target_id, "gap_mm": p.gap_mm, "frames": p.frames,
                       "valid_fraction": p.valid_fraction, "touches_border": p.touches_border,
                       "features_cut": p.features_cut, "front": asdict(p.front), "back": asdict(p.back),
                       "flags": p.flags} for p in self.poses],
            "notes": self.notes, "n_poses": len(self.poses), "n_flagged": len(self.flagged),
            "verdict": self.verdict()})

    def write_json(self, path: str | Path) -> Path:
        """Write the JSON report."""
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(self.to_dict(), indent=REPORT_JSON_INDENT), encoding="utf-8")
        return Path(path)


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


def _compare_plane(mask: np.ndarray, mean_depth: np.ndarray, valid_enough: np.ndarray, camera: PinholeCamera,
                   registered: tuple[np.ndarray, np.ndarray], params: CheckParameters) -> PlaneComparison:
    """Fit a plane to the mean points under the (eroded) mask and compare it with the registered plane
    (point, unit normal toward the sensor). Returns pixels = 0 and NaNs when too few pixels are available."""
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
    return comparison


def _plane_flags(comparison: PlaneComparison, params: CheckParameters, rms_flag: str, offset_flag: str,
                 tilt_flag: str) -> list[str]:
    """The flags a plane comparison earns against the thresholds."""
    flags = []
    if np.isfinite(comparison.rms_mm) and comparison.rms_mm > params.plane_residual_warn_mm:
        flags.append(rms_flag)
    if np.isfinite(comparison.offset_mm) and abs(comparison.offset_mm) > params.pose_residual_warn_mm:
        flags.append(offset_flag)
    if np.isfinite(comparison.angle_deg) and comparison.angle_deg > params.normal_warn_deg:
        flags.append(tilt_flag)
    return flags


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
    check.front = _compare_plane(front_mask, mean_depth, valid_enough, camera, target.front_plane_camera(pose), params)
    check.flags += _plane_flags(check.front, params, FLAG_FRONT_RMS, FLAG_FRONT_OFFSET, FLAG_FRONT_TILT)
    if check.front.pixels < params.min_plane_pixels and _eroded_count(front_mask, params) >= params.min_plane_pixels:
        check.flags.append(FLAG_FRONT_UNREAD)
    if target.gap_mm is not None:
        back_mask = expected == SURFACE_BACK
        check.back = _compare_plane(back_mask, mean_depth, valid_enough, camera, target.back_plane_camera(pose), params)
        check.flags += _plane_flags(check.back, params, FLAG_BACK_RMS, FLAG_BACK_OFFSET, FLAG_BACK_TILT)
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
    fit where the target has a back plate and enough pixels read it, with the flags of ``check_params``.
    Frames are streamed one at a time, so memory use does not grow with the frame count."""
    check_params = CheckParameters() if check_params is None else check_params
    groups = group_by_pose(session.records)
    poses = [check_pose(session, records, check_params) for records in groups.values()]
    notes = []
    if session.registration is None:
        notes.append("no registration.json: the manifest's target poses are used as they are")
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
            radius_px = geometry.diameter_in_pixels(feature.diameter_mm, float(center[2])) / 2.0 \
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
