"""
What every analysis of Part II shares: the registered geometry of a pose
(which pixel should see which surface, where the true edges are), the
reference planes, the region-of-interest masks, signed distances in pixels,
binning, and the output writers.

The ground truth of every pose is its registered target pose
(``FrameRecord.target_pose_camera``, from the robot read-back pose through the
registration). From it and the target definition, :class:`PoseGeometry` casts
the left camera's pixel rays onto the target once and keeps: the ideal surface
class per pixel (front, back, none), the target-frame (x, y) each ray meets on
the front plane (so any feature's signed distance can be evaluated per pixel),
the ideal front and back plane depth images Z_front_GT(u, v) and Z_back_GT(u, v)
(Section 10, Step 2), the geometric visibility V with and without the projector
condition (Sections 12 and 14), and the pixel footprint p(Z) at the station.

Conventions: see docs/design/code_design.md Section 4. Image arrays are
(H, W); depth is camera z in mm with NaN for no-reads.
"""
from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np
from scipy import ndimage

from sensorperf.features.planes import PlaneFit, PlaneFitParameters, fit_plane_robust, plane_depth_image
from sensorperf.geometry.camera import PinholeCamera
from sensorperf.geometry.targets import (
    Feature, StereoGeometry, SURFACE_BACK, SURFACE_FRONT, SURFACE_NONE, SurfaceHit, TwoPlaneTarget,
    geometric_visibility,
)
from sensorperf.geometry.transforms import RigidTransform
from sensorperf.io.capture_set import PoseStack
from sensorperf.io.manifest import FrameRecord
from sensorperf.io.session import Session
from sensorperf.parameters import CharacterizationParameters, SensorGeometry

FIGURE_FORMATS = ("png", "svg")
"""Every figure is written in these formats (Section 10, Step 13: PNG and SVG)."""
FIGURE_DPI = 150
"""Resolution of PNG figures."""
JSON_INDENT = 2
"""Indentation of JSON outputs."""
MIN_PIXELS_FOR_PLANE = 30
"""Fewer front or back pixels than this and a reference plane is not fitted from the data
(the registered plane is used instead and the result says so)."""
EROSION_STRUCTURE_CONNECTIVITY = 1
"""4-connected structuring element for mask erosion (a disk-like shrink of the ROI)."""


# ---------------------------------------------------------------------------
# Registered geometry of a pose
# ---------------------------------------------------------------------------
@dataclass
class PoseGeometry:
    """The ideal geometry of one pose from the registered target pose."""

    target: TwoPlaneTarget
    pose_camera: RigidTransform
    camera: PinholeCamera
    stereo: StereoGeometry
    hit: SurfaceHit
    """Ideal ray cast from the left camera (surface class, hit point, target-frame xy)."""
    front_xy_target: np.ndarray
    """(H, W, 2) target-frame (x, y) where each ray meets the front PLANE (solid or not)."""
    z_front_gt: np.ndarray
    """(H, W) depth of the registered front plane along each ray (Section 10, Step 2)."""
    z_back_gt: np.ndarray | None
    """(H, W) depth of the registered back plate along each ray, or None without a back plate."""
    visibility_with_projector: np.ndarray
    visibility_cameras_only: np.ndarray
    """V per pixel for the ideal hit point, with and without the projector condition."""
    pixel_footprint_mm: float
    """p(Z) at the station depth."""
    station_z_mm: float

    @property
    def front_mask(self) -> np.ndarray:
        return self.hit.surface == SURFACE_FRONT

    @property
    def back_mask(self) -> np.ndarray:
        return self.hit.surface == SURFACE_BACK

    @property
    def none_mask(self) -> np.ndarray:
        return self.hit.surface == SURFACE_NONE

    def ideal_depth(self) -> np.ndarray:
        """(H, W) ideal camera-z of the surface each ray meets, NaN where none."""
        return self.hit.point_camera[..., 2]

    def signed_distance_px(self, feature: Feature, edge: str | None = None) -> np.ndarray:
        """(H, W) signed distance s of each pixel's front-plane point to the feature's
        edge, in pixels at the station (positive on the front-material side; Sections 11, 14)."""
        s_mm = feature.signed_distance_mm(self.front_xy_target[..., 0], self.front_xy_target[..., 1], edge)
        return s_mm / self.pixel_footprint_mm

    def radial_distance_px(self, feature: Feature) -> np.ndarray:
        """(H, W) r - D/2 in pixels for a circular feature (negative inside the circle)."""
        lx, ly = feature.local_coordinates(self.front_xy_target[..., 0], self.front_xy_target[..., 1])
        return (np.sqrt(lx ** 2 + ly ** 2) - feature.diameter_mm / 2.0) / self.pixel_footprint_mm

    def feature_center_px(self, feature: Feature) -> tuple[float, float]:
        """Image coordinates (u, v) of the feature center on the front plane."""
        center = self.target.feature_center_camera(self.pose_camera, feature)
        u, v, _ = self.camera.project(center)
        return float(u), float(v)

    def feature_window(self, feature: Feature, margin_px: float) -> np.ndarray:
        """(H, W) mask of the circle of radius D_px/2 + margin around the projected center
        (the feature window of Sections 12 and 13)."""
        u0, v0 = self.feature_center_px(feature)
        u, v = self.camera.pixel_grid()
        radius = feature.diameter_mm / self.pixel_footprint_mm / 2.0 + margin_px
        return (u - u0) ** 2 + (v - v0) ** 2 <= radius ** 2

    def inside_outline(self, feature: Feature, grow_px: float = 0.0) -> np.ndarray:
        """(H, W) pixels whose front-plane point lies inside the feature outline grown by grow_px."""
        lx, ly = feature.local_coordinates(self.front_xy_target[..., 0], self.front_xy_target[..., 1])
        half = feature.diameter_mm / 2.0 + grow_px * self.pixel_footprint_mm
        if feature.is_circular():
            return lx ** 2 + ly ** 2 <= half ** 2
        return (np.abs(lx) <= half) & (np.abs(ly) <= half)


def pose_geometry(session: Session, record: FrameRecord, camera: PinholeCamera,
                  include_posts: bool = False) -> PoseGeometry:
    """Cast the registered geometry of a pose. ``camera`` is the capture's own
    pinhole model (from the file header, see io/capture_set); the stereo
    viewpoints come from the session's sensor geometry."""
    target = session.targets.get(record.target_id, record.gap_mm)
    if record.gap_mm is None and target.gap_mm is not None:
        target = target.with_gap(None)                        # open-background variant: no back plate
    stereo = StereoGeometry.from_sensor_geometry(_geometry_with_camera(session.geometry, camera))
    pose = record.target_pose_camera
    rays = camera.ray_directions()
    hit = target.intersect_rays(pose, np.zeros(3), rays, include_posts=include_posts)
    front_xy = target.pixel_xy_on_front_plane(camera, pose)
    point, normal = target.front_plane_camera(pose)
    z_front = plane_depth_image(camera, point, normal)
    z_back = None
    if target.gap_mm is not None:
        back_point, back_normal = target.back_plane_camera(pose)
        z_back = plane_depth_image(camera, back_point, back_normal)
    v_with = geometric_visibility(target, pose, hit.point_camera, stereo, True, include_posts=include_posts)
    v_without = geometric_visibility(target, pose, hit.point_camera, stereo, False, include_posts=include_posts)
    return PoseGeometry(target=target, pose_camera=pose, camera=camera, stereo=stereo, hit=hit,
                        front_xy_target=front_xy, z_front_gt=z_front, z_back_gt=z_back,
                        visibility_with_projector=v_with, visibility_cameras_only=v_without,
                        pixel_footprint_mm=record.station_z_mm / camera.focal_x_px,
                        station_z_mm=record.station_z_mm)


def _geometry_with_camera(geometry: SensorGeometry, camera: PinholeCamera) -> SensorGeometry:
    """The session's sensor geometry with the intrinsics and image size taken from the
    capture header (the header is the authority for the camera that took the frame)."""
    from dataclasses import replace
    return replace(geometry, sensor_fx_px=camera.focal_x_px, sensor_fy_px=camera.focal_y_px,
                   sensor_cx_px=camera.principal_x_px, sensor_cy_px=camera.principal_y_px,
                   image_width_px=camera.width, image_height_px=camera.height)


# ---------------------------------------------------------------------------
# Masks
# ---------------------------------------------------------------------------
def erode_mask(mask: np.ndarray, pixels: float) -> np.ndarray:
    """Shrink a mask by ``pixels`` (rounded up) with a 4-connected structuring element
    (Section 10, Step 1: shrink the plate outline by BOUNDARY_BAND_HALF_WIDTH_PX)."""
    iterations = int(math.ceil(pixels))
    if iterations <= 0:
        return mask.copy()
    structure = ndimage.generate_binary_structure(2, EROSION_STRUCTURE_CONNECTIVITY)
    return ndimage.binary_erosion(mask, structure=structure, iterations=iterations, border_value=0)


def region_of_interest(geometry: PoseGeometry, params: CharacterizationParameters) -> np.ndarray:
    """The front-material mask shrunk by BOUNDARY_BAND_HALF_WIDTH_PX (the ROI of Analysis A)."""
    return erode_mask(geometry.front_mask, params.boundary_band_half_width_px)


def away_from_edges(geometry: PoseGeometry, params: CharacterizationParameters,
                    band_px: float | None = None) -> tuple[np.ndarray, np.ndarray]:
    """(front, back) masks of pixels farther than the band from any front/back
    boundary: the front mask eroded, and the back mask eroded, by band_px
    (default BOUNDARY_BAND_HALF_WIDTH_PX). Used for the reference planes
    (Section 11.1, Step 1; Section 12, Step 2)."""
    band = params.boundary_band_half_width_px if band_px is None else band_px
    return erode_mask(geometry.front_mask, band), erode_mask(geometry.back_mask, band)


def central_patch_mask(camera: PinholeCamera, side_px: int, center_uv: tuple[float, float] | None = None) -> np.ndarray:
    """(H, W) mask of a side_px x side_px patch centered on the principal point (or center_uv)."""
    u0, v0 = (camera.principal_x_px, camera.principal_y_px) if center_uv is None else center_uv
    u, v = camera.pixel_grid()
    half = side_px / 2.0
    return (np.abs(u - u0) < half) & (np.abs(v - v0) < half)


# ---------------------------------------------------------------------------
# Reference planes
# ---------------------------------------------------------------------------
@dataclass
class ReferencePlanes:
    """Front and back reference depth images of a pose and where they came from."""

    z_front: np.ndarray
    z_back: np.ndarray | None
    front_fit: PlaneFit | None
    back_fit: PlaneFit | None
    front_from_data: bool
    back_from_data: bool


def reference_planes(stack_depth: np.ndarray, geometry: PoseGeometry, params: CharacterizationParameters,
                     fit_params: PlaneFitParameters = PlaneFitParameters(),
                     front_mask: np.ndarray | None = None, back_mask: np.ndarray | None = None) -> ReferencePlanes:
    """Fit the front and back planes to the pixels of a (mean) depth image that lie
    farther than the boundary band from any edge (Section 11.1, Step 1; Section 12,
    Step 2). Where too few pixels are available the registered plane stands in.
    ``stack_depth`` is (H, W) camera-z (NaN for no-reads)."""
    default_front, default_back = away_from_edges(geometry, params)
    front_mask = default_front if front_mask is None else front_mask
    back_mask = default_back if back_mask is None else back_mask
    camera = geometry.camera
    points = camera.back_project(*camera.pixel_grid(), stack_depth)
    valid = np.isfinite(stack_depth)

    def fit(mask: np.ndarray) -> PlaneFit | None:
        chosen = mask & valid
        if chosen.sum() < MIN_PIXELS_FOR_PLANE:
            return None
        try:
            return fit_plane_robust(points[chosen], fit_params)
        except ValueError:
            return None

    front_fit = fit(front_mask)
    back_fit = fit(back_mask) if geometry.z_back_gt is not None else None
    z_front = plane_depth_image(camera, front_fit.point, front_fit.normal) if front_fit else geometry.z_front_gt
    if geometry.z_back_gt is None:
        z_back = None
    else:
        z_back = plane_depth_image(camera, back_fit.point, back_fit.normal) if back_fit else geometry.z_back_gt
    return ReferencePlanes(z_front=z_front, z_back=z_back, front_fit=front_fit, back_fit=back_fit,
                           front_from_data=front_fit is not None, back_from_data=back_fit is not None)


# ---------------------------------------------------------------------------
# Binning and small numerics
# ---------------------------------------------------------------------------
@dataclass
class BinnedProfile:
    """Mean of y in bins of x."""

    centers: np.ndarray
    mean: np.ndarray
    count: np.ndarray
    std: np.ndarray

    def interpolate_crossing(self, level: float) -> float:
        """x at which the (monotone-ish) mean first crosses ``level``, by linear interpolation; NaN if never."""
        y = self.mean
        x = self.centers
        ok = np.isfinite(y)
        x, y = x[ok], y[ok]
        for i in range(len(y) - 1):
            if (y[i] - level) * (y[i + 1] - level) <= 0.0 and y[i] != y[i + 1]:
                return float(x[i] + (level - y[i]) * (x[i + 1] - x[i]) / (y[i + 1] - y[i]))
        return float("nan")


def binned_mean(x, y, bin_width: float, x_min: float, x_max: float) -> BinnedProfile:
    """Mean, count and standard deviation of y in bins of x of width bin_width over
    [x_min, x_max]; NaN entries of x or y are ignored; empty bins give NaN."""
    x = np.asarray(x, dtype=np.float64).ravel()
    y = np.asarray(y, dtype=np.float64).ravel()
    ok = np.isfinite(x) & np.isfinite(y) & (x >= x_min) & (x < x_max)
    edges = np.arange(x_min, x_max + bin_width / 2.0, bin_width)
    centers = (edges[:-1] + edges[1:]) / 2.0
    index = np.clip(((x[ok] - x_min) / bin_width).astype(int), 0, len(centers) - 1)
    count = np.bincount(index, minlength=len(centers)).astype(float)
    total = np.bincount(index, weights=y[ok], minlength=len(centers))
    square = np.bincount(index, weights=y[ok] ** 2, minlength=len(centers))
    with np.errstate(invalid="ignore", divide="ignore"):
        mean = np.where(count > 0, total / count, np.nan)
        variance = np.where(count > 1, (square - count * mean ** 2) / (count - 1), np.nan)
    return BinnedProfile(centers=centers, mean=mean, count=count, std=np.sqrt(np.maximum(variance, 0.0)))


def depth_codes(depth: np.ndarray, lsb_mm: float) -> np.ndarray:
    """Integer depth codes round(Z / LSB) (Section 10, Step 8), -1 where NaN."""
    codes = np.full(depth.shape, -1, dtype=np.int64)
    ok = np.isfinite(depth)
    codes[ok] = np.round(depth[ok] / lsb_mm).astype(np.int64)
    return codes


def connected_components(mask: np.ndarray) -> tuple[np.ndarray, int]:
    """8-connected labeling of a boolean mask (labels, count)."""
    structure = np.ones((3, 3), dtype=bool)
    labels, count = ndimage.label(mask, structure=structure)
    return labels, int(count)


def hann_smooth(values: np.ndarray, window: int) -> np.ndarray:
    """Smooth a 1-D array with a normalized Hann window of the given odd length (NaN-aware)."""
    if window < 3:
        return np.asarray(values, dtype=np.float64).copy()
    kernel = np.hanning(window + 2)[1:-1]
    kernel /= kernel.sum()
    v = np.asarray(values, dtype=np.float64)
    ok = np.isfinite(v)
    filled = np.where(ok, v, 0.0)
    numerator = np.convolve(filled, kernel, mode="same")
    denominator = np.convolve(ok.astype(float), kernel, mode="same")
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(denominator > 0, numerator / denominator, np.nan)


# ---------------------------------------------------------------------------
# Output writers
# ---------------------------------------------------------------------------
def write_csv_rows(path: str | Path, rows: Sequence[dict[str, Any]], columns: Sequence[str] | None = None) -> Path:
    """Write dict rows as CSV; columns default to the union of keys in first-seen order."""
    path = Path(path)
    if columns is None:
        seen: dict[str, None] = {}
        for row in rows:
            for key in row:
                seen.setdefault(key, None)
        columns = list(seen)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(columns), extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({k: _csv_value(row.get(k)) for k in columns})
    return path


def _csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, float):
        return "" if math.isnan(value) else repr(value)
    if isinstance(value, (np.floating, np.integer)):
        return _csv_value(value.item())
    if isinstance(value, (list, tuple, np.ndarray)):
        return ";".join(str(_csv_value(v)) for v in value)
    return value


def write_json(path: str | Path, document: Any) -> Path:
    """Write JSON with numpy values converted to plain Python."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_jsonable(document), indent=JSON_INDENT), encoding="utf-8")
    return path


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return [_jsonable(v) for v in value.tolist()]
    if isinstance(value, (np.floating,)):
        return None if np.isnan(value) else float(value)
    if isinstance(value, float):
        return None if math.isnan(value) else value
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.bool_,)):
        return bool(value)
    if isinstance(value, Path):
        return str(value)
    return value


def save_figure(figure, path_stem: str | Path) -> list[Path]:
    """Save a matplotlib figure as PNG and SVG next to each other; returns the paths."""
    import matplotlib.pyplot as plt
    stem = Path(path_stem)
    stem.parent.mkdir(parents=True, exist_ok=True)
    written = []
    for extension in FIGURE_FORMATS:
        out = stem.with_suffix(f".{extension}")
        figure.savefig(out, dpi=FIGURE_DPI, bbox_inches="tight")
        written.append(out)
    plt.close(figure)
    return written


def new_figure(width_in: float = 7.0, height_in: float = 4.5):
    """A matplotlib figure and axes with the Agg backend (no display needed)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt.subplots(figsize=(width_in, height_in))
