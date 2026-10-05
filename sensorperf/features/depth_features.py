# Reused from the depth_calibration_from_spherical_target repository (sphcal/features/depth_features.py), import path
# adjusted to this package. Keep in step with that repository; fix upstream and re-copy.
"""
Per-pixel features computed from depth images: temporal statistics over the
frames of one pose, the local surface slopes (s_u, s_v) that are inputs of the
correction map, the range along each pixel's ray, and the block-independence
weight.

Conventions
-----------
- Depth z is camera-z in millimeters; invalid depth is z <= 0 (or a False entry
  of the validity mask, or a non-finite value). Image arrays are (H, W);
  pixel (u, v) = (column, row).
- Range rho is the Euclidean distance from the camera center along the pixel's
  ray: rho = z * sqrt(1 + ((u - cx)/fx)^2 + ((v - cy)/fy)^2).
- Slopes follow code_design.md section 4. A plane z = a + (dz/du) du + (dz/dv) dv
  is fitted by least squares over the square window, with du, dv the pixel
  offsets from the window center and only valid pixels contributing. Then
      s_u = (dz/du) * f_x / a,    s_v = (dz/dv) * f_y / a,
  where a is the fitted plane's value at the window center (not the raw depth).
  Sign: s_u > 0 when depth increases toward larger u (to the right), s_v > 0
  when depth increases toward larger v (downward). For a plane tilted by theta
  about the v axis, s_u = tan(theta) at the optical axis; the cosine of the
  incidence angle is 1 / sqrt(1 + s_u^2 + s_v^2).
- Slopes are computed at every pixel whose window has enough valid pixels,
  including pixels whose own depth is invalid; callers mask by validity.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from sensorperf.geometry.camera import PinholeCamera

MIN_SAMPLES_FOR_VARIANCE = 2
"""Smallest number of valid frames for which an unbiased variance exists."""

SLOPE_DETERMINANT_RELATIVE_TOLERANCE = 1e-6
"""A slope window's 3 x 3 normal-equation matrix M is treated as singular when
det(M) <= this * M[0,0] * M[1,1] * M[2,2] (the product of its diagonal, which
bounds the determinant of a positive semidefinite matrix; the ratio is 1 for a
full symmetric window and tends to 0 when the valid pixels are collinear)."""

CONSTANT_PAD_VALUE = 0.0
"""Value assumed outside the image by the windowed sums. Zero is correct
because every sum is weighted by the validity mask (outside = invalid)."""

POWER_ZERO = 0
POWER_ONE = 1
POWER_TWO = 2
"""Exponents of the window-offset monomials du^a dv^b in the normal equations."""


@dataclass(frozen=True)
class SlopeParameters:
    """Window of the local plane fit that defines the slope inputs."""

    window_px: int = 13
    """Odd side length of the square window (at least three effective cells, D-8)."""
    min_valid_fraction: float = 0.6
    """Fraction of window pixels that must be valid for a slope to be reported."""


@dataclass
class TemporalStatistics:
    """Per-pixel statistics over the frames of one pose, each (H, W)."""

    mean_depth: np.ndarray      # mm, NaN where too few frames were valid
    variance: np.ndarray        # mm^2, unbiased, NaN where too few frames were valid
    valid_count: np.ndarray     # int, number of valid frames
    read_fraction: np.ndarray   # valid_count / F


def temporal_statistics(depth_stack, valid_stack, min_valid_fraction) -> TemporalStatistics:
    """
    Mean and unbiased variance of depth over the frames where the pixel is valid.

    depth_stack (F, H, W) camera-z in mm; valid_stack (F, H, W) bool.
    read_fraction = valid_count / F. The mean is NaN where read_fraction is below
    min_valid_fraction (or no frame is valid); the variance is additionally NaN
    where valid_count < MIN_SAMPLES_FOR_VARIANCE.
    """
    depth = np.asarray(depth_stack, dtype=np.float64)
    valid = np.asarray(valid_stack, dtype=bool) & np.isfinite(depth)
    n_frames = depth.shape[0]
    valid_count = valid.sum(axis=0)
    read_fraction = valid_count / float(n_frames)
    enough = (read_fraction >= min_valid_fraction) & (valid_count >= 1)
    safe_count = np.maximum(valid_count, 1)
    masked = np.where(valid, depth, 0.0)
    mean = masked.sum(axis=0) / safe_count
    # Two-pass variance: sum of squared deviations over valid frames only.
    squared_deviation = np.where(valid, (depth - mean[None]) ** 2, 0.0)
    enough_for_variance = enough & (valid_count >= MIN_SAMPLES_FOR_VARIANCE)
    variance = squared_deviation.sum(axis=0) / np.maximum(valid_count - 1, 1)
    return TemporalStatistics(
        mean_depth=np.where(enough, mean, np.nan),
        variance=np.where(enough_for_variance, variance, np.nan),
        valid_count=valid_count,
        read_fraction=read_fraction)


def temporal_mean_points(xyz_stack: np.ndarray, valid_stack: np.ndarray, min_valid_fraction: float) -> np.ndarray:
    """
    Per-pixel mean 3-D point over the frames in which the pixel was valid.

    xyz_stack (F, H, W, 3) camera-frame points in mm; valid_stack (F, H, W) bool.
    The mean is NaN where fewer than min_valid_fraction of the F frames are
    valid. This is the point-cloud counterpart of temporal_statistics, shared
    by the capture check tool and the fit so that both average frames the same
    way.
    """
    count = valid_stack.sum(axis=0)
    with np.errstate(invalid="ignore", divide="ignore"):
        mean = (xyz_stack * valid_stack[..., None]).sum(axis=0) / count[..., None]
    enough = count >= min_valid_fraction * valid_stack.shape[0]
    return np.where(enough[..., None], mean, np.nan)


def _window_sum(array: np.ndarray, half: int, power_u: int, power_v: int) -> np.ndarray:
    """
    Sum over the (2 half + 1)^2 window centered on each pixel of
    array * du^power_u * dv^power_v, with du (dv) the column (row) offset from
    the window center. Zero padding outside the image. Separable: a 1-D
    correlation along each axis with the offset polynomial as the kernel (the
    same box sum as scipy.ndimage.uniform_filter, with the offsets as weights;
    uniform_filter itself cannot weight by offset).
    """
    offsets = np.arange(-half, half + 1, dtype=np.float64)
    along_u = ndimage.correlate1d(array, offsets ** power_u, axis=1, mode="constant",
                                  cval=CONSTANT_PAD_VALUE)
    return ndimage.correlate1d(along_u, offsets ** power_v, axis=0, mode="constant",
                               cval=CONSTANT_PAD_VALUE)


def _check_window(window_px: int) -> int:
    """Validate an odd positive window size and return its half width."""
    if window_px < 1 or window_px % 2 == 0:
        raise ValueError("window_px must be a positive odd integer")
    return window_px // 2


def image_slopes(depth_image, valid, camera: PinholeCamera,
                 params: SlopeParameters) -> tuple[np.ndarray, np.ndarray]:
    """
    Slope inputs (s_u, s_v), each (H, W), dimensionless; see the module
    docstring for the definition and sign. NaN where the fraction of valid pixels
    in the window is below params.min_valid_fraction, where the fitted center
    depth is not positive, or where the 3 x 3 normal-equation matrix is singular
    (SLOPE_DETERMINANT_RELATIVE_TOLERANCE).
    """
    half = _check_window(params.window_px)
    z = np.asarray(depth_image, dtype=np.float64)
    mask = np.asarray(valid, dtype=bool) & np.isfinite(z) & (z > 0.0)
    weight = mask.astype(np.float64)
    z_masked = np.where(mask, z, 0.0)

    def moment(power_u: int, power_v: int) -> np.ndarray:
        return _window_sum(weight, half, power_u, power_v)

    def depth_moment(power_u: int, power_v: int) -> np.ndarray:
        return _window_sum(z_masked, half, power_u, power_v)

    s00, s10, s01 = moment(POWER_ZERO, POWER_ZERO), moment(POWER_ONE, POWER_ZERO), moment(POWER_ZERO, POWER_ONE)
    s20, s11, s02 = moment(POWER_TWO, POWER_ZERO), moment(POWER_ONE, POWER_ONE), moment(POWER_ZERO, POWER_TWO)
    t00, t10, t01 = (depth_moment(POWER_ZERO, POWER_ZERO), depth_moment(POWER_ONE, POWER_ZERO),
                     depth_moment(POWER_ZERO, POWER_ONE))

    # Normal equations M [a, b, c]^T = rhs for z = a + b du + c dv, per pixel.
    normal_matrix = np.stack([np.stack([s00, s10, s01], axis=-1),
                              np.stack([s10, s20, s11], axis=-1),
                              np.stack([s01, s11, s02], axis=-1)], axis=-2)      # (H, W, 3, 3)
    rhs = np.stack([t00, t10, t01], axis=-1)                                       # (H, W, 3)

    window_pixels = float(params.window_px ** 2)
    enough_valid = (s00 / window_pixels) >= params.min_valid_fraction
    determinant = np.linalg.det(normal_matrix)
    well_conditioned = determinant > SLOPE_DETERMINANT_RELATIVE_TOLERANCE * (s00 * s20 * s02)
    solvable = enough_valid & well_conditioned

    s_u = np.full(z.shape, np.nan)
    s_v = np.full(z.shape, np.nan)
    if solvable.any():
        coefficients = np.linalg.solve(normal_matrix[solvable], rhs[solvable][..., None])[..., 0]
        center_depth, dz_du, dz_dv = coefficients[:, 0], coefficients[:, 1], coefficients[:, 2]
        positive = center_depth > 0.0
        safe_depth = np.where(positive, center_depth, np.nan)
        s_u[solvable] = dz_du * camera.focal_x_px / safe_depth
        s_v[solvable] = dz_dv * camera.focal_y_px / safe_depth
    return s_u, s_v


def range_from_depth(depth_image, camera: PinholeCamera) -> np.ndarray:
    """Range (mm) along each pixel's ray: rho = z * sqrt(1 + ((u - cx)/fx)^2 +
    ((v - cy)/fy)^2). Invalid depths are scaled like any other (z <= 0 stays <= 0)."""
    z = np.asarray(depth_image, dtype=np.float64)
    u, v = camera.pixel_grid()
    tan_u = (u - camera.principal_x_px) / camera.focal_x_px
    tan_v = (v - camera.principal_y_px) / camera.focal_y_px
    return z * np.sqrt(1.0 + tan_u ** 2 + tan_v ** 2)


def block_independence_weight(effective_block_px: int) -> float:
    """Weight 1 / block^2 (D-4): one independent measurement per
    effective_block_px x effective_block_px native pixels."""
    if effective_block_px < 1:
        raise ValueError("effective_block_px must be at least 1")
    return 1.0 / float(effective_block_px) ** 2
