# Reused from the depth_calibration_from_spherical_target repository (sphcal/features/normals.py), import path
# adjusted to this package. Keep in step with that repository; fix upstream and re-copy.
"""
Surface normals from a least-squares plane fit over an N x N window of
camera-frame points (the downstream 5 x 5 estimator, D-8).

Conventions: xyz is (H, W, 3) in the camera frame, millimeters; a pixel is
valid when its mask entry is True, all three coordinates are finite and z > 0.
The plane through the valid points of a window minimizing the sum of squared
orthogonal distances has as its normal the eigenvector of the smallest
eigenvalue of the points' 3 x 3 covariance. The sign is chosen to face the
camera: normal . ray < 0, where the ray is the direction from the camera center
to the window's mean point. Windows are centered on each pixel; a normal is
reported at every pixel whose window is usable, including pixels whose own
point is invalid (callers mask by validity).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import ndimage

DEGENERACY_RELATIVE_TOLERANCE = 1e-6
"""A window is degenerate (the plane is not determined) when its second-smallest
covariance eigenvalue is at most this fraction of the largest, i.e. the valid
points are collinear or coincident. The smallest eigenvalue is not used: it is
zero for a perfect plane, which is the good case."""

CONSTANT_PAD_VALUE = 0.0
"""Value assumed outside the image by the windowed sums; sums are weighted by
the validity mask, so outside pixels count as invalid."""


@dataclass(frozen=True)
class NormalEstimatorParameters:
    """Window of the plane fit."""

    window_px: int = 5
    """Odd side length of the square window (D-8)."""
    min_valid_fraction: float = 0.6
    """Fraction of window pixels that must be valid for a normal to be reported."""


def plane_fit_normals(xyz, valid, params: NormalEstimatorParameters) -> np.ndarray:
    """
    Unit normals (H, W, 3) oriented toward the camera (negative dot product with
    the ray). NaN where the fraction of valid points in the window is below
    params.min_valid_fraction or the covariance is degenerate
    (DEGENERACY_RELATIVE_TOLERANCE).
    """
    if params.window_px < 1 or params.window_px % 2 == 0:
        raise ValueError("window_px must be a positive odd integer")
    points = np.asarray(xyz, dtype=np.float64)
    mask = (np.asarray(valid, dtype=bool) & np.all(np.isfinite(points), axis=-1)
            & (points[..., 2] > 0.0))
    # Shift by the mean valid point so the second moments are small numbers
    # (the covariance is shift invariant); the shift is added back for the ray.
    shift = points[mask].mean(axis=0) if mask.any() else np.zeros(3)
    centered = np.where(mask[..., None], points - shift, 0.0)
    weight = mask.astype(np.float64)

    def box_mean(image: np.ndarray) -> np.ndarray:
        # Mean over the window of the (masked) image, zero padded outside the image.
        return ndimage.uniform_filter(image, size=params.window_px, mode="constant",
                                      cval=CONSTANT_PAD_VALUE)

    # Round to a whole pixel count so that a window exactly at the threshold is
    # not lost to floating-point error in the filter.
    window_pixels = float(params.window_px ** 2)
    valid_fraction = np.round(box_mean(weight) * window_pixels) / window_pixels
    enough = valid_fraction >= params.min_valid_fraction
    safe_fraction = np.where(enough, valid_fraction, 1.0)
    # Means over the valid pixels of the window: E[p] and E[p p^T].
    mean_point = np.stack([box_mean(centered[..., i]) for i in range(3)], axis=-1) / safe_fraction[..., None]
    second_moment = np.empty(mask.shape + (3, 3))
    for i in range(3):
        for j in range(i, 3):
            entry = box_mean(centered[..., i] * centered[..., j]) / safe_fraction
            second_moment[..., i, j] = entry
            second_moment[..., j, i] = entry
    covariance = second_moment - mean_point[..., :, None] * mean_point[..., None, :]

    normals = np.full(mask.shape + (3,), np.nan)
    candidates = enough
    if not candidates.any():
        return normals
    eigenvalues, eigenvectors = np.linalg.eigh(covariance[candidates])             # ascending
    largest = eigenvalues[:, 2]
    determined = (largest > 0.0) & (eigenvalues[:, 1] > DEGENERACY_RELATIVE_TOLERANCE * largest)
    normal = eigenvectors[:, :, 0]                                                 # smallest eigenvalue
    ray = mean_point[candidates] + shift                                           # window mean point
    flip = np.sum(normal * ray, axis=-1) > 0.0                                     # face the camera
    normal = np.where(flip[:, None], -normal, normal)
    normal[~determined] = np.nan
    normals[candidates] = normal
    return normals
