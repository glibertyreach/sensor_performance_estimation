"""
Plane fits and plane-based depth references shared by the analyses.

- :func:`fit_plane_robust`: a least-squares plane through 3-D points (camera
  frame, mm) with iterative trimming of outliers, returning the plane as a
  point and a unit normal oriented toward the camera, plus the residual RMS.
- :func:`plane_depth_image`: for every pixel, the camera-z depth at which its
  ray meets a plane. With the registered target pose this is Z_GT(u, v) of
  Section 10, Step 2; with fitted planes it is Z_front and Z_back of Section 11.
- :func:`normalized_height`: h = (Z_back - Z) / (Z_back - Z_front), 1 on the
  front surface and 0 on the back plate (Section 11.1, Step 2).
- :func:`incidence_cosine`: cos of the angle between a plane normal and each
  pixel's ray (the phi of Section 12, Step 4).

Conventions: image arrays are (H, W), pixel (u, v) = (column, row); depth is
camera z in mm; invalid depth is NaN (callers convert the zero sentinel first).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from sensorperf.geometry.camera import PinholeCamera

MEDIAN_ABSOLUTE_TO_SIGMA = 1.4826
"""Factor converting a median absolute deviation into a Gaussian sigma."""
MIN_PLANE_POINTS = 3
"""A plane needs at least three points."""
MIN_ROBUST_SCALE_MM = 1.0e-6
"""Smallest robust scale, so a perfect fit does not reject everything."""
PLANE_PARALLEL_TOLERANCE = 1.0e-12
"""Rays whose dot product with the plane normal is smaller than this are parallel to it."""


@dataclass(frozen=True)
class PlaneFitParameters:
    """Trimming of the robust plane fit."""

    outlier_sigma_multiple: float = 3.0
    """Points farther from the plane than this multiple of the robust scale are dropped."""
    rounds: int = 3
    """Rounds of fit-and-trim."""


@dataclass
class PlaneFit:
    """A fitted plane in the camera frame."""

    point: np.ndarray
    """A point on the plane (the centroid of the inliers), mm."""
    normal: np.ndarray
    """Unit normal oriented toward the camera (normal . point < 0)."""
    rms_mm: float
    """RMS orthogonal distance of the inliers."""
    inliers: np.ndarray
    """Boolean mask over the input points."""

    def distance(self, points) -> np.ndarray:
        """Signed orthogonal distance of points to the plane (positive on the camera side)."""
        return (np.asarray(points, dtype=np.float64) - self.point) @ self.normal


def fit_plane_robust(points, params: PlaneFitParameters = PlaneFitParameters()) -> PlaneFit:
    """Least-squares plane (smallest singular vector of the centered inliers)
    with ``params.rounds`` of trimming at ``outlier_sigma_multiple`` robust
    scales. ``points`` is (N, 3) or (..., 3) with NaN rows ignored. Raises
    ValueError when fewer than MIN_PLANE_POINTS finite points remain."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    finite = np.all(np.isfinite(pts), axis=1)
    inliers = finite.copy()
    if inliers.sum() < MIN_PLANE_POINTS:
        raise ValueError(f"a plane fit needs at least {MIN_PLANE_POINTS} finite points, got {int(inliers.sum())}")
    point = normal = None
    for _ in range(max(params.rounds, 1)):
        chosen = pts[inliers]
        centroid = chosen.mean(axis=0)
        _, _, vt = np.linalg.svd(chosen - centroid, full_matrices=False)
        normal = vt[-1]
        if normal @ centroid > 0.0:
            normal = -normal                       # face the camera (origin)
        point = centroid
        residual = (pts - point) @ normal
        scale = max(MEDIAN_ABSOLUTE_TO_SIGMA * np.median(np.abs(residual[inliers])), MIN_ROBUST_SCALE_MM)
        new_inliers = finite & (np.abs(residual) <= params.outlier_sigma_multiple * scale)
        if new_inliers.sum() < MIN_PLANE_POINTS or np.array_equal(new_inliers, inliers):
            break
        inliers = new_inliers
    residual = (pts - point) @ normal
    rms = float(np.sqrt(np.mean(residual[inliers] ** 2)))
    return PlaneFit(point=point, normal=normal, rms_mm=rms, inliers=inliers.reshape(np.asarray(points).shape[:-1]))


def plane_depth_image(camera: PinholeCamera, plane_point, plane_normal) -> np.ndarray:
    """(H, W) camera-z depth at which each pixel's ray meets the plane; NaN where
    the ray is parallel to the plane or meets it behind the camera."""
    rays = camera.ray_directions()
    normal = np.asarray(plane_normal, dtype=np.float64)
    point = np.asarray(plane_point, dtype=np.float64)
    denominator = rays @ normal
    moving = np.abs(denominator) > PLANE_PARALLEL_TOLERANCE
    t = np.where(moving, (point @ normal) / np.where(moving, denominator, 1.0), np.nan)
    depth = t * rays[..., 2]
    return np.where(moving & (t > 0.0), depth, np.nan)


def plane_depth_at(camera: PinholeCamera, u, v, plane_point, plane_normal) -> np.ndarray:
    """Camera-z depth of the plane along the rays of arbitrary (fractional) pixel coordinates."""
    rays = camera.ray_directions_at(np.asarray(u, dtype=np.float64), np.asarray(v, dtype=np.float64))
    normal = np.asarray(plane_normal, dtype=np.float64)
    point = np.asarray(plane_point, dtype=np.float64)
    denominator = rays @ normal
    moving = np.abs(denominator) > PLANE_PARALLEL_TOLERANCE
    t = np.where(moving, (point @ normal) / np.where(moving, denominator, 1.0), np.nan)
    return np.where(moving & (t > 0.0), t * rays[..., 2], np.nan)


def normalized_height(depth, z_front, z_back) -> np.ndarray:
    """h = (Z_back - Z) / (Z_back - Z_front): 1 on the front surface, 0 on the back
    plate (Section 11.1, Step 2). NaN where the depth is NaN or the two references coincide."""
    z = np.asarray(depth, dtype=np.float64)
    zf = np.asarray(z_front, dtype=np.float64)
    zb = np.asarray(z_back, dtype=np.float64)
    span = zb - zf
    with np.errstate(invalid="ignore", divide="ignore"):
        h = (zb - z) / span
    return np.where(np.isfinite(h) & (span != 0.0), h, np.nan)


def incidence_cosine(camera: PinholeCamera, plane_normal) -> np.ndarray:
    """(H, W) cos(phi): cosine of the angle between the plane normal (toward the
    camera) and each pixel's ray pointing back to the camera."""
    rays = camera.ray_directions()
    normal = np.asarray(plane_normal, dtype=np.float64)
    return np.clip(-(rays @ normal), 0.0, 1.0)


def pixel_area_on_plane_mm2(camera: PinholeCamera, depth_on_plane, plane_normal) -> np.ndarray:
    """a = Z^2 / (f_x f_y cos(phi)): the area on the plane seen by each pixel
    (Section 12, Step 4). NaN where the depth is NaN."""
    z = np.asarray(depth_on_plane, dtype=np.float64)
    cos_phi = incidence_cosine(camera, plane_normal)
    with np.errstate(invalid="ignore", divide="ignore"):
        area = z ** 2 / (camera.focal_x_px * camera.focal_y_px * cos_phi)
    return np.where(np.isfinite(area), area, np.nan)
