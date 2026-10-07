"""
The mount check of the procedure, Step 4.8: is a freshly mounted target where the registration says it is?

Re-mounting a target on the dowel-pinned adapter needs no new registration, provided this check passes. It is made once
per mount, at the mount-check station (``mount_check_depth_mm``, 800 mm), with the target fronto-parallel and centered
(the pose the registration commands), from the depth frames of a short capture (the sub-series ``mount_check``):

    Z      the mounted target's front plane is fitted to the temporal-mean depth of the pixels that should see the front
           surface (the same robust plane fit and pixel classification as the quick-look check, ``acquisition.check``).
           The Z error is the distance, along the camera ray through the target's reference point, between the fitted
           plane and the registered one; it must be within ``registration_residual_accept_mm`` (0.15 mm).
    tilt   the angle between the fitted plane's normal and the registered normal must be within
           ``mount_tilt_tolerance_deg`` (0.05 degrees; a dagger parameter).
    H, V   one feature of the target (the largest one that lies well inside the field of view) is located in the image and
           its center is compared with where the registered pose and the AS-BUILT feature offsets from the datum
           (targets.json with targets_asbuilt.csv applied, the record of the target) put it: both within
           ``frame_check_px`` (0.5 px). Only for a target with features and a back plate (T3a, T3b, T4, T5); T2 has none.

The registered pose. The registration (registration.json) turns a read-back flange pose into the target pose in the camera
frame (:meth:`Registration.target_to_camera`). When the read-back flange pose of the mount-check capture is given, that is
the registered pose. Without it the registered pose is the one the robot was commanded to: the target centered and
fronto-parallel at ``mount_check_depth_mm`` (the robot reaches it within its repeatability, which is far below the
tolerances above), and a note says so.

HOW THE FEATURE IS LOCATED. The package has no routine that locates an edge or outline in the left IR image (the analyses
all work on the depth), so the H and V check uses a DEPTH-DERIVED outline: inside a search region around the feature's
expected outline, a pixel is "front" when its mean depth lies within half the gap of the fitted front plane and "back"
otherwise (deeper or not read). The feature's interior is the front region for a raised feature (disk, raised square) and
the back region for a hole (cutout, window). The measured center is the centroid of the interior pixels; the expected one
is the centroid of the pixels that the registered, as-built geometry puts in the interior, found on the same grid, so that
the pixel discretization and the matcher's symmetric edge blur cancel. A left-IR image in the capture is not used.

Units: millimeters, degrees and pixels. Image arrays are (H, W); depth is camera z with NaN for a no-read.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from scipy import ndimage

from sensorperf.acquisition.check import (
    REPORT_JSON_INDENT, STRUCTURE_SIZE, UNREADABLE_ERRORS, CheckParameters, json_safe,
)
from sensorperf.features.planes import fit_plane_robust, plane_depth_at, plane_depth_image
from sensorperf.geometry.camera import PinholeCamera
from sensorperf.geometry.registration import Registration
from sensorperf.geometry.targets import (
    FEATURE_CUTOUT, FEATURE_DISK, FEATURE_SQUARE_RAISED, FEATURE_SQUARE_WINDOW, SURFACE_FRONT, Feature, TwoPlaneTarget,
    fronto_parallel_pose,
)
from sensorperf.geometry.transforms import RigidTransform
from sensorperf.io.capture_set import load_frame_depth
from sensorperf.parameters import CharacterizationParameters

MOUNT_CHECK_FILE_NAME = "mount_check.json"
"""Name of the report written next to the frames."""
FRAME_SUFFIX = ".mc"
"""Extension of a capture file; a folder given to the check is searched for these."""

CHECK_Z = "Z"
CHECK_TILT = "tilt"
CHECK_H = "H"
CHECK_V = "V"
"""Names of the four checks."""
PASS_TEXT = "PASS"
FAIL_TEXT = "FAIL"
SKIP_TEXT = "SKIP"
"""Outcome of a check as printed. SKIP: the check could not be made (the reason is printed)."""

FEATURE_SEARCH_MARGIN_PX = 6
"""The search region of a feature is its expected interior grown by this many pixels, so that a feature that sits a few
pixels away from its expected place is still found whole. A shift larger than this is clipped and measured as smaller than
it is, but it is still far beyond ``frame_check_px`` and fails."""
FEATURE_BORDER_MARGIN_PX = 2
"""A feature is used only when its search region stays this many pixels inside the image."""
MIN_FEATURE_PIXELS = 20
"""Fewest interior pixels a feature must have in the image to be located; a smaller one gives no centroid worth comparing
with half a pixel."""
FRONT_BAND_FRACTION_OF_GAP = 0.5
"""A pixel is "front" when its depth is within this fraction of the gap of the fitted front plane, "back" when it is
deeper or not read (half the gap is the level midway between the two surfaces)."""
LOCATED_FEATURE_KINDS = (FEATURE_DISK, FEATURE_CUTOUT, FEATURE_SQUARE_RAISED, FEATURE_SQUARE_WINDOW)
"""Feature kinds that can be located (circles and squares with an interior); blank and post sites have none."""


class MountCheckInputError(Exception):
    """An input of the mount check has to be fixed; the message says what to change."""


# ---------------------------------------------------------------------------
# Result records
# ---------------------------------------------------------------------------
@dataclass
class CheckResult:
    """One of the checks: the measured value against its tolerance."""

    name: str
    """CHECK_Z, CHECK_TILT, CHECK_H or CHECK_V."""
    measured: float | None
    tolerance: float
    unit: str
    tolerance_name: str
    """The parameter the tolerance comes from (REGISTRATION_RESIDUAL_ACCEPT_MM ...), or "option" when overridden."""
    passed: bool | None
    """True or False; None when the check could not be made (``detail`` says why)."""
    detail: str = ""
    extra: dict[str, Any] = field(default_factory=dict)
    """Further numbers of the check (the same quantity in millimeters, the fit RMS, pixel counts ...)."""

    @property
    def outcome(self) -> str:
        """PASS, FAIL or SKIP."""
        return SKIP_TEXT if self.passed is None else (PASS_TEXT if self.passed else FAIL_TEXT)

    def format_line(self) -> str:
        """One console line: the outcome, the measured value and the tolerance."""
        if self.measured is None:
            return f"  {self.name:<5}{self.outcome}  {self.detail}"
        line = (f"  {self.name:<5}{self.outcome}  measured {self.measured:+.3f} {self.unit}, tolerance "
                f"{self.tolerance:g} {self.unit} ({self.tolerance_name})")
        return line + (f"; {self.detail}" if self.detail else "")


@dataclass
class MountCheckReport:
    """The result of :func:`check_mount`."""

    target_id: str
    gap_mm: float | None
    files: list[str]
    registered_pose_note: str
    results: list[CheckResult]
    notes: list[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        """True when no check failed (a skipped check does not fail the mount, but the verdict says it was skipped)."""
        return not any(result.passed is False for result in self.results)

    def verdict(self) -> str:
        """The final line."""
        failed = [r.name for r in self.results if r.passed is False]
        skipped = [r.name for r in self.results if r.passed is None]
        text = (f"VERDICT: mount check PASSED for {self.target_id}" if not failed
                else f"VERDICT: mount check FAILED for {self.target_id} ({', '.join(failed)})")
        if skipped:
            text += f"; not checked: {', '.join(skipped)}"
        return text + ("." if not failed else ". Re-seat the target or correct the datum record, then check again.")

    def format_text(self) -> str:
        """The console text: the mounted target, one line per check, the notes and the verdict."""
        gap = "no back plate" if self.gap_mm is None else f"G = {self.gap_mm:g} mm"
        lines = [f"Mount check of {self.target_id} ({gap}) from {len(self.files)} frame(s); {self.registered_pose_note}"]
        lines += [result.format_line() for result in self.results]
        lines += [f"NOTE: {note}" for note in self.notes]
        lines.append(self.verdict())
        return "\n".join(lines)

    def to_dict(self) -> dict[str, Any]:
        """The JSON report (NaN becomes null)."""
        return json_safe({
            "target_id": self.target_id, "gap_mm": self.gap_mm, "frames": len(self.files), "files": self.files,
            "registered_pose": self.registered_pose_note,
            "checks": [{"name": r.name, "outcome": r.outcome, "measured": r.measured, "unit": r.unit,
                        "tolerance": r.tolerance, "tolerance_name": r.tolerance_name, "detail": r.detail, **r.extra}
                       for r in self.results],
            "notes": self.notes, "passed": self.passed, "verdict": self.verdict()})

    def write_json(self, path: str | Path) -> Path:
        """Write the JSON report (mount_check.json)."""
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(self.to_dict(), indent=REPORT_JSON_INDENT), encoding="utf-8")
        return Path(path)


@dataclass(frozen=True)
class MountTolerances:
    """The tolerances of the three comparisons (Step 4.8); the defaults are the parameter table's."""

    z_mm: float
    tilt_deg: float
    frame_px: float
    z_name: str = "REGISTRATION_RESIDUAL_ACCEPT_MM"
    tilt_name: str = "MOUNT_TILT_TOLERANCE_DEG"
    frame_name: str = "FRAME_CHECK_PX"

    @classmethod
    def from_parameters(cls, params: CharacterizationParameters) -> "MountTolerances":
        """The tolerances of a parameter table."""
        return cls(z_mm=params.registration_residual_accept_mm, tilt_deg=params.mount_tilt_tolerance_deg,
                   frame_px=params.frame_check_px)


# ---------------------------------------------------------------------------
# Inputs
# ---------------------------------------------------------------------------
def frame_paths(inputs: Sequence[str | Path]) -> list[Path]:
    """The capture files named by the inputs: a file is taken as it is, a folder gives its ``.mc`` files in name order.
    Raises MountCheckInputError when a path does not exist or nothing is found."""
    paths: list[Path] = []
    for item in inputs:
        path = Path(item)
        if path.is_dir():
            paths += sorted(path.glob(f"*{FRAME_SUFFIX}"))
        elif path.is_file():
            paths.append(path)
        else:
            raise MountCheckInputError(f"{path} does not exist; give capture files or the folder that holds them")
    if not paths:
        raise MountCheckInputError(f"no {FRAME_SUFFIX} capture files found in {', '.join(str(i) for i in inputs)}")
    return paths


def registered_pose(params: CharacterizationParameters, registration: Registration,
                    flange_pose: RigidTransform | None = None) -> tuple[RigidTransform, str]:
    """(target pose in the camera frame, a sentence saying where it comes from). With the read-back flange pose of the
    capture it is ``registration.target_to_camera(flange_pose)``; without, the commanded pose: the target centered and
    fronto-parallel at the mount-check station."""
    if flange_pose is not None:
        return (registration.target_to_camera(flange_pose),
                "registered pose = the registration applied to the given read-back flange pose")
    return (fronto_parallel_pose(0.0, 0.0, params.mount_check_depth_mm),
            f"registered pose = the commanded one (centered, fronto-parallel at Z = {params.mount_check_depth_mm:g} mm)")


def mean_depth_of_frames(paths: Sequence[Path], min_valid_fraction: float) -> tuple[np.ndarray, PinholeCamera]:
    """(mean depth, camera): the mean over the frames of the reads of each pixel, NaN where the pixel was read in fewer than
    ``min_valid_fraction`` of the frames. Frames are streamed one at a time. Raises MountCheckInputError for a file that
    cannot be read or whose size differs from the first."""
    total = count = None
    camera: PinholeCamera | None = None
    for path in paths:
        try:
            depth, _, header = load_frame_depth(path)
        except UNREADABLE_ERRORS as error:
            raise MountCheckInputError(f"cannot read the capture {path}: {error}") from None
        if camera is None:
            camera = PinholeCamera.from_matcloud_header(header, width_px=depth.shape[1], height_px=depth.shape[0])
            total, count = np.zeros(depth.shape), np.zeros(depth.shape, dtype=np.int64)
        elif depth.shape != total.shape:
            raise MountCheckInputError(f"{path} has another image size ({depth.shape}) than the first frame")
        finite = np.isfinite(depth)
        total += np.where(finite, depth, 0.0)
        count += finite
    enough = count >= max(1.0, min_valid_fraction * len(paths))
    with np.errstate(invalid="ignore", divide="ignore"):
        mean = np.where(enough, total / np.maximum(count, 1), np.nan)
    return mean, camera


# ---------------------------------------------------------------------------
# The checks
# ---------------------------------------------------------------------------
def _eroded(mask: np.ndarray, margin_px: int) -> np.ndarray:
    """The mask eroded by the classification margin (not at the image border)."""
    if margin_px <= 0:
        return mask
    return ndimage.binary_erosion(mask, structure=np.ones((STRUCTURE_SIZE, STRUCTURE_SIZE), dtype=bool),
                                  iterations=margin_px, border_value=1)


def check_z_and_tilt(target: TwoPlaneTarget, pose: RigidTransform, camera: PinholeCamera, mean_depth: np.ndarray,
                     tolerances: MountTolerances, check_params: CheckParameters):
    """(Z result, tilt result, the fitted plane or None): the mounted target's front plane fitted to the mean depth of the
    pixels that the registered pose says see the front surface, against the registered front plane."""
    hit = target.intersect_rays(pose, np.zeros(3), camera.ray_directions())
    chosen = _eroded(hit.surface == SURFACE_FRONT, check_params.classification_margin_px) & np.isfinite(mean_depth)
    pixels = int(chosen.sum())
    if pixels < check_params.min_plane_pixels:
        reason = (f"only {pixels} pixels of the expected front surface were read (at least "
                  f"{check_params.min_plane_pixels} are needed): is the target in the field of view?")
        return (CheckResult(CHECK_Z, None, tolerances.z_mm, "mm", tolerances.z_name, None, reason),
                CheckResult(CHECK_TILT, None, tolerances.tilt_deg, "deg", tolerances.tilt_name, None, reason), None)
    u, v = camera.pixel_grid()
    fit = fit_plane_robust(camera.back_project(u[chosen], v[chosen], mean_depth[chosen]))
    registered_point, registered_normal = target.front_plane_camera(pose)
    # Z error: the fitted plane against the registered one along the ray through the target's reference point.
    ref_u, ref_v, _ = camera.project(registered_point)
    fitted_z = float(plane_depth_at(camera, ref_u, ref_v, fit.point, fit.normal))
    z_error = fitted_z - float(registered_point[2])
    tilt = float(np.degrees(np.arccos(np.clip(fit.normal @ registered_normal, -1.0, 1.0))))
    common = {"fit_rms_mm": fit.rms_mm, "fit_pixels": pixels}
    return (CheckResult(CHECK_Z, z_error, tolerances.z_mm, "mm", tolerances.z_name, abs(z_error) <= tolerances.z_mm,
                        "positive = farther than registered", dict(common)),
            CheckResult(CHECK_TILT, tilt, tolerances.tilt_deg, "deg", tolerances.tilt_name,
                        tilt <= tolerances.tilt_deg, "angle between the normals", dict(common)), fit)


def _locatable_features(target: TwoPlaneTarget) -> list[Feature]:
    """The features that have an interior to locate (not blank sites or posts), largest first."""
    usable = [f for f in target.features if f.kind in LOCATED_FEATURE_KINDS]
    return sorted(usable, key=lambda f: -f.diameter_mm)


def feature_search_region(target: TwoPlaneTarget, feature: Feature, pose: RigidTransform, camera: PinholeCamera):
    """(expected interior mask, search region mask) on the pixel grid for one feature at the registered pose with the
    as-built feature values: the interior is where the feature's outline puts it, the region is the interior grown by
    FEATURE_SEARCH_MARGIN_PX without the interiors of the other features."""
    xy = target.pixel_xy_on_front_plane(camera, pose)
    with np.errstate(invalid="ignore"):
        interior = np.isfinite(xy[..., 0]) & feature.inside(xy[..., 0], xy[..., 1])
        others = np.zeros_like(interior)
        for other in target.features:
            if other is not feature:
                others |= np.isfinite(xy[..., 0]) & other.inside(xy[..., 0], xy[..., 1])
    grown = ndimage.binary_dilation(interior, structure=np.ones((STRUCTURE_SIZE, STRUCTURE_SIZE), dtype=bool),
                                    iterations=FEATURE_SEARCH_MARGIN_PX)
    return interior, grown & ~others


def _centroid_px(mask: np.ndarray, u: np.ndarray, v: np.ndarray) -> tuple[float, float]:
    """Centroid (u, v) of a mask in pixel coordinates."""
    return float(u[mask].mean()), float(v[mask].mean())


def check_feature_position(target: TwoPlaneTarget, pose: RigidTransform, camera: PinholeCamera, mean_depth: np.ndarray,
                           fit, tolerances: MountTolerances):
    """(H result, V result): the center of the largest usable feature, located in the depth image as described in the
    module docstring, against the center that the registered pose and the as-built feature offsets give. Both results are
    skipped, with the reason, for a target without features or back plate or when no feature lies well inside the field."""
    def skipped(reason: str):
        return (CheckResult(CHECK_H, None, tolerances.frame_px, "px", tolerances.frame_name, None, reason),
                CheckResult(CHECK_V, None, tolerances.frame_px, "px", tolerances.frame_name, None, reason))

    candidates = _locatable_features(target)
    if not candidates:
        return skipped("the target has no feature to locate (a plain plate)")
    if target.gap_mm is None or fit is None:
        return skipped("a feature is located by its depth contrast, which needs the back plate and the front plane fit")
    u, v = camera.pixel_grid()
    front_depth = plane_depth_image(camera, fit.point, fit.normal)
    front_like = np.isfinite(mean_depth) & (np.abs(mean_depth - front_depth) <= FRONT_BAND_FRACTION_OF_GAP * target.gap_mm)
    border = FEATURE_BORDER_MARGIN_PX
    for feature in candidates:
        interior, region = feature_search_region(target, feature, pose, camera)
        rows, cols = np.nonzero(region)
        if (interior.sum() < MIN_FEATURE_PIXELS or rows.size == 0 or rows.min() < border or cols.min() < border
                or rows.max() > camera.height - 1 - border or cols.max() > camera.width - 1 - border):
            continue
        # The measured interior: front-material pixels of a raised feature, everything else (back plate or no read)
        # of a hole, inside the search region.
        measured = region & (front_like if feature.is_front_material() else ~front_like)
        if measured.sum() < MIN_FEATURE_PIXELS:
            continue
        expected_u, expected_v = _centroid_px(interior & region, u, v)
        measured_u, measured_v = _centroid_px(measured, u, v)
        depth_mm = float(np.nanmedian(mean_depth[interior])) if np.isfinite(mean_depth[interior]).any() else float(pose.translation[2])
        shift_px = (measured_u - expected_u, measured_v - expected_v)
        common = {"site_id": feature.site_id, "feature_kind": feature.kind, "feature_pixels": int(interior.sum()),
                  "located_from": "depth outline"}
        return (CheckResult(CHECK_H, shift_px[0], tolerances.frame_px, "px", tolerances.frame_name,
                            abs(shift_px[0]) <= tolerances.frame_px,
                            f"site {feature.site_id} ({feature.kind}); {shift_px[0] * depth_mm / camera.focal_x_px:+.3f} mm",
                            {**common, "shift_mm": shift_px[0] * depth_mm / camera.focal_x_px}),
                CheckResult(CHECK_V, shift_px[1], tolerances.frame_px, "px", tolerances.frame_name,
                            abs(shift_px[1]) <= tolerances.frame_px,
                            f"site {feature.site_id} ({feature.kind}); {shift_px[1] * depth_mm / camera.focal_y_px:+.3f} mm",
                            {**common, "shift_mm": shift_px[1] * depth_mm / camera.focal_y_px}))
    return skipped("no feature lies well inside the field of view with enough pixels to locate")


def check_mount(paths: Sequence[Path], target: TwoPlaneTarget, pose: RigidTransform, pose_note: str,
                tolerances: MountTolerances, check_params: CheckParameters | None = None) -> MountCheckReport:
    """The mount check of one target from the depth frames of its mount-check capture (module docstring)."""
    check_params = CheckParameters() if check_params is None else check_params
    mean_depth, camera = mean_depth_of_frames(paths, check_params.min_valid_fraction)
    z_result, tilt_result, fit = check_z_and_tilt(target, pose, camera, mean_depth, tolerances, check_params)
    h_result, v_result = check_feature_position(target, pose, camera, mean_depth, fit, tolerances)
    notes = []
    if h_result.passed is not None:
        notes.append("H and V are located from the depth outline of the feature: the package has no routine that locates an "
                     "edge in the left IR image, so the IR image is not used.")
    return MountCheckReport(target_id=target.target_id, gap_mm=target.gap_mm, files=[str(p) for p in paths],
                            registered_pose_note=pose_note, results=[z_result, tilt_result, h_result, v_result], notes=notes)


def report_location(inputs: Sequence[str | Path]) -> Path:
    """Where mount_check.json goes by default: inside the folder that was given, else next to the first frame."""
    first = Path(inputs[0])
    return (first if first.is_dir() else first.parent) / MOUNT_CHECK_FILE_NAME
