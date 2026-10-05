# Reused from the depth_calibration_from_spherical_target repository (sphcal/geometry/camera.py), import path
# adjusted to this package. Keep in step with that repository; fix upstream and re-copy.
"""
Pinhole camera model of the depth sensor, built from a capture file's header.

Conventions (OpenCV): pixel (u, v) = (column, row), u to the right, v downward,
camera z along the optical axis into the scene, millimeters throughout. The
model is distortion-free by design: whatever lens or rectification error the
sensor carries is what the correction map is meant to absorb, so it must not
also be modeled here.

The sixdof repository's camera module was the reference for these conventions;
this is a smaller module with only what the calibration needs.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache

import numpy as np


@dataclass(frozen=True)
class PinholeCamera:
    """An ideal pinhole camera. Attributes are in pixels."""

    width: int
    height: int
    focal_x_px: float
    focal_y_px: float
    principal_x_px: float
    principal_y_px: float

    @classmethod
    def from_matcloud_header(cls, header: dict, width_px: int, height_px: int) -> "PinholeCamera":
        """Build from the header keys fx, fy, cx, cy, which every real capture
        inspected carries. The image size is taken from the decoded array shape
        rather than from the header, which is the reliable source."""
        return cls(int(width_px), int(height_px), float(header["fx"]), float(header["fy"]),
                   float(header["cx"]), float(header["cy"]))

    def pixel_grid(self) -> tuple[np.ndarray, np.ndarray]:
        """(u, v) pixel-center coordinate images, each of shape (height, width)."""
        u, v = np.meshgrid(np.arange(self.width, dtype=np.float64), np.arange(self.height, dtype=np.float64))
        return u, v

    def back_project(self, u: np.ndarray, v: np.ndarray, depth_z: np.ndarray) -> np.ndarray:
        """Camera-frame points (..., 3) for pixel coordinates at camera-z depth."""
        u = np.asarray(u, dtype=np.float64)
        v = np.asarray(v, dtype=np.float64)
        depth_z = np.asarray(depth_z, dtype=np.float64)
        x = (u - self.principal_x_px) / self.focal_x_px * depth_z
        y = (v - self.principal_y_px) / self.focal_y_px * depth_z
        return np.stack([x, y, depth_z], axis=-1)

    def project(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Pixel coordinates (u, v) of camera-frame points (..., 3), and a mask of
        points in front of the camera. Points at or behind the camera get NaN."""
        p = np.asarray(points, dtype=np.float64)
        z = p[..., 2]
        in_front = z > 0.0
        safe_z = np.where(in_front, z, np.nan)
        u = self.focal_x_px * p[..., 0] / safe_z + self.principal_x_px
        v = self.focal_y_px * p[..., 1] / safe_z + self.principal_y_px
        return u, v, in_front

    def ray_directions(self) -> np.ndarray:
        """Unit ray direction of every pixel center, shape (height, width, 3).
        Cached per camera (the camera is immutable); callers must not write into it."""
        return _cached_ray_directions(self)

    def ray_directions_at(self, u: np.ndarray, v: np.ndarray) -> np.ndarray:
        """Unit ray directions for arbitrary (possibly fractional) pixel coordinates."""
        directions = self.back_project(u, v, np.ones_like(np.asarray(u, dtype=np.float64)))
        return directions / np.linalg.norm(directions, axis=-1, keepdims=True)

    def half_angles_degrees(self) -> tuple[float, float]:
        """(horizontal, vertical) half field of view in degrees."""
        return (float(np.degrees(np.arctan((self.width / 2.0) / self.focal_x_px))),
                float(np.degrees(np.arctan((self.height / 2.0) / self.focal_y_px))))


@lru_cache(maxsize=8)
def _cached_ray_directions(camera: PinholeCamera) -> np.ndarray:
    u, v = camera.pixel_grid()
    directions = camera.back_project(u, v, np.ones_like(u))
    directions /= np.linalg.norm(directions, axis=-1, keepdims=True)
    directions.setflags(write=False)
    return directions
