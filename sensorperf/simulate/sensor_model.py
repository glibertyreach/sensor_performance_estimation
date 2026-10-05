"""
INDICATIVE synthetic depth renderer of a :class:`~sensorperf.geometry.targets.TwoPlaneTarget`
(design document, Section 5, "simulate/sensor_model.py").

What it is for. The acquisition tools and the analyses of the characterization
procedure (Parts I and II) need sample data that exercises them without the
sensor. This module renders one depth frame of a two-plane target from the left
camera of a stereo sensor and imitates the GROSS features of a VSX3000-like
sensor that the analyses measure: depth noise that grows with Z^2 and with the
incidence angle and is spatially correlated over a few pixels, a static fixed
pattern, a disparity quantizer followed by a depth LSB, a matcher that is
coarser than the pixel grid and prefers the foreground at depth edges, no-reads
in occlusion shadows and slow drift. It is NOT a model of the sensor's internal
matcher; every value below is INDICATIVE, and the analyses must not rely on
any of them (they only make the synthetic data statistically plausible).

Conventions (design document, Section 4). Camera frame = the left IR camera,
Z along its optical axis, H = x along image columns, V = y along image rows,
millimeters. Image arrays are (height, width); pixel (u, v) = (column, row).
Depth is camera z; a no-read is NaN in :attr:`RenderedFrame.depth` and the
all-zero XYZ point of the ``.mc`` convention in :attr:`RenderedFrame.xyz`.

Rendering pipeline of :func:`render_frame` (the seven steps of the contract)

1. Ideal ray cast of every pixel ray against the target: which surface it
   meets (front, back or none) and where.
2. Geometric visibility of each hit (left camera, right camera and, if
   required, projector): readable = hit and visible.
3. Imitation matcher: the depth of a pixel is the weighted mean of the
   readable true depths in a square window of ``matching_window_px`` pixels,
   weight 1 for back-surface pixels and ``front_preference`` for front-surface
   pixels, so that a depth edge fattens toward the front. A pixel is a no-read
   when the readable fraction of its window is below ``min_window_fill``, or
   when its own ray hits nothing.
4. Disparity noise: d = k / Z with k = f_x B, plus Gaussian noise of standard
   deviation sigma_d cos(alpha)^(-m), drawn once per block of
   ``fixed_pattern_block_px`` pixels and interpolated bilinearly back to the
   pixels. This is what correlates the noise spatially over about one block
   (Analysis A measures that length).
5. A static fixed pattern (one value per block, seeded, identical in every
   frame, amplitude in mm scaled by (Z / 1000 mm)^2 and converted to
   disparity), then quantization of d to multiples of ``disparity_quantum_px``,
   Z = k / d, and rounding of Z to multiples of ``output_lsb_mm``.
6. Drift: ``drift_mm_per_hour`` times the elapsed hours is added to Z.
7. xyz = unit pixel ray times Z / ray_z (so that its z component is Z), float32;
   zeros and NaN where there is no read.

Two details that go beyond the contract's wording (both keep it true):

- The bilinear interpolation of block draws is normalized so that the
  interpolated field has unit variance at EVERY pixel (not only at block
  centers). Plain bilinear interpolation of independent draws would lower the
  standard deviation between block centers to about two thirds; with the
  normalization ``disparity_noise_px`` really is the per-pixel temporal sigma_d
  and ``fixed_pattern_amplitude_mm`` the RMS of the fixed pattern at 1 m.
- The visibility test is evaluated only for the pixels that hit the back
  plate: a front-surface point is always visible (``TwoPlaneTarget.visible_from``
  says so), so evaluating the segment test for it is wasted work. The result
  is the same as calling :func:`geometric_visibility` on every hit.

The block-grid helpers (block layout and interpolation matrices) are adapted
from ``sphcal/simulate/synthetic.py`` of the depth_calibration_from_spherical_target
repository (``_block_layout``, ``_interpolation_matrix`` and the ``_BlockGrid``
idea), simplified to plain bilinear interpolation without valid-count weights
and extended with the unit-variance normalization above.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from functools import lru_cache
from pathlib import Path
from typing import Any

import numpy as np
from scipy.ndimage import uniform_filter

from sensorperf.features.planes import incidence_cosine
from sensorperf.geometry.targets import (
    FEATURE_CUTOUT, FEATURE_DISK, SURFACE_BACK, SURFACE_FRONT, SURFACE_NONE, StereoGeometry, TwoPlaneTarget,
    geometric_visibility,
)
from sensorperf.geometry.transforms import RigidTransform
from sensorperf.io.capture_set import XYZ_CHANNEL_NAME
from sensorperf.io.matcloud import write_matcloud
from sensorperf.parameters import SensorGeometry

# ---------------------------------------------------------------------------
# Indicative values (not datasheet values; see the module docstring)
# ---------------------------------------------------------------------------
INDICATIVE_DISPARITY_NOISE_PX = 0.08
"""Indicative disparity noise sigma_d, pixels."""
INDICATIVE_NOISE_INCIDENCE_EXPONENT = 1.3
"""Indicative exponent m of the noise multiplier cos(alpha)^(-m)."""
INDICATIVE_DISPARITY_QUANTUM_PX = 1.0 / 8.0
"""Indicative disparity quantum q, pixels (an eighth of a pixel)."""
INDICATIVE_MATCHING_WINDOW_PX = 7
"""Indicative side of the imitation matcher's square window, pixels (odd)."""
INDICATIVE_FRONT_PREFERENCE = 2.0
"""Indicative weight of front-surface pixels in the window mean (> 1 fattens the foreground)."""
INDICATIVE_MIN_WINDOW_FILL = 0.5
"""Indicative minimum readable fraction of the matching window."""
INDICATIVE_FIXED_PATTERN_AMPLITUDE_MM = 0.5
"""Indicative RMS of the static fixed pattern at 1 m, mm."""
INDICATIVE_FIXED_PATTERN_BLOCK_PX = 4
"""Indicative block size of the fixed pattern and of the noise correlation, pixels."""
INDICATIVE_FIXED_PATTERN_SEED = 20261004
"""Seed of the indicative fixed pattern (the same pattern in every frame)."""
INDICATIVE_REQUIRE_PROJECTOR = True
"""The indicative sensor is an active stereo sensor: a read needs projector light."""
INDICATIVE_POSTS_VISIBLE = False
"""By default the hidden disk posts are not rendered (Section 3.2: they are thinner than the pixel)."""
INDICATIVE_DRIFT_MM_PER_HOUR = 0.0
"""Indicative drift of the depth field; zero keeps sentinels flat unless a test asks otherwise."""
INDICATIVE_MIN_FEATURE_DIAMETER_PX = 10.0
"""Indicative smallest feature diameter, in pixels of the full-size (640 x 480) image, that the imitation matcher
resolves. The redesign note brackets the real sensor's minimum detectable diameter at 10 to 15 px (a path correlation
that needs about 4 pencil detections, a diameter of about 7 px, and matcher support windows of 7 x 7 to 13 x 13 px);
the synthetic sensor takes a value inside 8 to 12 px so that the detection transitions of the synthetic data fall
inside the feature ladder (3 to 96 px). Not a datasheet value."""

# ---------------------------------------------------------------------------
# Numerical guards and file-format constants
# ---------------------------------------------------------------------------
MIN_NOISE_COSINE = 0.05
"""Floor on cos(alpha) in the noise multiplier cos^(-m): at grazing incidence the multiplier
would otherwise grow without bound (cos = 0.05 is 87 degrees)."""
WEIGHT_SUM_TOLERANCE = 1.0e-9
"""A window whose total matching weight is below this has no readable pixel."""
MIN_VALID_DISPARITY_PX = 0.5
"""A disparity at or below this (after noise) is not a valid read: Z = k / d would be absurd."""
FIXED_PATTERN_REFERENCE_DEPTH_MM = 1000.0
"""Depth at which ``fixed_pattern_amplitude_mm`` applies; the amplitude scales as (Z / this)^2."""
SYNTHETIC_CAMERA_NAME = "synthetic"
"""Value of the header key cameraName in synthetic files."""
SYNTHETIC_HEADER_VERSION = 2
"""Value of the header key version in synthetic files (matches real files)."""
PLACEHOLDER_DEPTH_MM = 1000.0
"""Depth substituted at no-read pixels so that the vectorized arithmetic of steps 4 to 6 stays finite
there; the result at those pixels is discarded."""
BLOCK_CACHE_SIZE = 8
"""Number of block grids and fixed patterns kept per process."""


# ---------------------------------------------------------------------------
# The model and the frame
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class SyntheticSensorModel:
    """INDICATIVE behavior of the synthetic sensor (module docstring). All
    values are indicative; none is a confirmed VSX3000 specification."""

    geometry: SensorGeometry
    """Intrinsics, baseline, projector offset and depth LSB of the simulated sensor."""
    disparity_noise_px: float
    """sigma_d: standard deviation of the temporal disparity noise at normal incidence, pixels
    (indicative 0.08). The depth noise is sigma_Z = sigma_d Z^2 / k."""
    noise_incidence_exponent: float
    """m: the noise standard deviation is multiplied by cos(alpha)^(-m) (indicative 1.3)."""
    disparity_quantum_px: float
    """q: disparity quantization step, pixels; 0 disables the quantizer (indicative 1/8)."""
    output_lsb_mm: float
    """Depth output LSB applied after the disparity quantizer, mm; 0 disables it."""
    matching_window_px: int
    """Side of the imitation matcher's square window, pixels, odd (indicative 7)."""
    front_preference: float
    """Weight of front-surface pixels in the window mean relative to back-surface pixels;
    values above 1 fatten the foreground at depth edges (indicative 2.0)."""
    min_window_fill: float
    """Fraction of the window that must be readable; below it the pixel is a no-read (indicative 0.5)."""
    fixed_pattern_amplitude_mm: float
    """RMS of the per-block static depth offset at 1 m, mm; it scales as (Z / 1000 mm)^2 and is
    identical in every frame. 0 disables the pattern."""
    fixed_pattern_block_px: int
    """Block size of the fixed pattern and of the noise draws, pixels; the noise correlation
    length is about this size."""
    fixed_pattern_seed: int
    """Seed of the fixed pattern: the same seed gives the same pattern in every frame and session."""
    require_projector: bool
    """Whether a read needs projector illumination in addition to both cameras seeing the
    point (the question of Section 12, Step 7)."""
    posts_visible: bool
    """Whether the disk support posts are rendered as front material."""
    drift_mm_per_hour: float
    """Linear drift of the whole depth field, mm per hour of elapsed time (for sentinels); 0 disables."""
    min_feature_diameter_px: float = 0.0
    """Smallest disk or cutout diameter, in pixels of the rendered image, that the imitation matcher resolves
    (the synthetic sensor's minimum detectable size; indicative 10 px at 640 x 480, roughly 8 to 12). A feature that
    subtends fewer pixels than this is not rendered: a disk reads as back plate and a cutout as front plate, as if
    the matcher's support window averaged it away. 0 disables the rule (every feature is rendered, and only the
    window blur and the noise limit detection)."""

    def __post_init__(self) -> None:
        if self.matching_window_px < 1 or self.matching_window_px % 2 == 0:
            raise ValueError(f"matching_window_px must be a positive odd integer, got {self.matching_window_px}")
        if self.fixed_pattern_block_px < 1:
            raise ValueError(f"fixed_pattern_block_px must be at least 1, got {self.fixed_pattern_block_px}")
        if not 0.0 <= self.min_window_fill <= 1.0:
            raise ValueError(f"min_window_fill must lie in [0, 1], got {self.min_window_fill}")
        if self.front_preference <= 0.0:
            raise ValueError(f"front_preference must be positive, got {self.front_preference}")
        if min(self.disparity_noise_px, self.disparity_quantum_px, self.output_lsb_mm,
               self.fixed_pattern_amplitude_mm, self.min_feature_diameter_px) < 0.0:
            raise ValueError("noise, quantum, LSB, fixed pattern amplitude and minimum feature size must not be negative")

    @classmethod
    def indicative(cls, geometry: SensorGeometry) -> "SyntheticSensorModel":
        """The indicative model on the given geometry: the module constants above, and the geometry's
        own depth LSB as the output LSB (``MissingSensorValue`` if the geometry has none)."""
        return cls(
            geometry=geometry,
            disparity_noise_px=INDICATIVE_DISPARITY_NOISE_PX,
            noise_incidence_exponent=INDICATIVE_NOISE_INCIDENCE_EXPONENT,
            disparity_quantum_px=INDICATIVE_DISPARITY_QUANTUM_PX,
            output_lsb_mm=float(geometry.require("depth_lsb_mm")),
            matching_window_px=INDICATIVE_MATCHING_WINDOW_PX,
            front_preference=INDICATIVE_FRONT_PREFERENCE,
            min_window_fill=INDICATIVE_MIN_WINDOW_FILL,
            fixed_pattern_amplitude_mm=INDICATIVE_FIXED_PATTERN_AMPLITUDE_MM,
            fixed_pattern_block_px=INDICATIVE_FIXED_PATTERN_BLOCK_PX,
            fixed_pattern_seed=INDICATIVE_FIXED_PATTERN_SEED,
            require_projector=INDICATIVE_REQUIRE_PROJECTOR,
            posts_visible=INDICATIVE_POSTS_VISIBLE,
            drift_mm_per_hour=INDICATIVE_DRIFT_MM_PER_HOUR,
            min_feature_diameter_px=INDICATIVE_MIN_FEATURE_DIAMETER_PX,
        )

    @classmethod
    def indicative_scaled(cls, geometry: SensorGeometry, pixel_divisor: int = 1) -> "SyntheticSensorModel":
        """The indicative model on a geometry whose pixels are ``pixel_divisor`` times larger than the full-size
        (640 x 480) sensor's, with the same field of view (``demo_plan.scaled_geometry``). The disparity noise,
        the disparity quantum and the minimum feature diameter, which are in pixels, are divided by the divisor so
        that the depth noise and the depth quantum in millimeters and the minimum feature size in millimeters are
        those of the full-size sensor (sigma_Z = sigma_d Z^2 / k and delta_Z = q Z^2 / k keep their values)."""
        indicative = cls.indicative(geometry)
        return replace(indicative, disparity_noise_px=indicative.disparity_noise_px / pixel_divisor,
                       disparity_quantum_px=indicative.disparity_quantum_px / pixel_divisor,
                       min_feature_diameter_px=indicative.min_feature_diameter_px / pixel_divisor)


@dataclass
class RenderedFrame:
    """One rendered frame and the truth it was rendered from, all (H, W[, 3])."""

    depth: np.ndarray
    """(H, W) camera z in mm, NaN where the sensor delivered no read."""
    xyz: np.ndarray
    """(H, W, 3) float32 camera-frame points in mm, zeros where not read (the .mc convention)."""
    true_surface: np.ndarray
    """SURFACE_NONE / SURFACE_FRONT / SURFACE_BACK per pixel from the ideal ray cast."""
    true_depth: np.ndarray
    """Ideal camera z of the hit, mm (NaN where SURFACE_NONE)."""
    visibility: np.ndarray
    """Geometric visibility V per pixel (left, right and, if required, projector); False where nothing is hit."""


# ---------------------------------------------------------------------------
# Block grid: one random value per block, interpolated back to the pixels
# (adapted from sphcal/simulate/synthetic.py; see the module docstring)
# ---------------------------------------------------------------------------
def _block_layout(length: int, block: int) -> tuple[int, np.ndarray]:
    """Number of blocks along an axis of ``length`` pixels (the last block may be
    partial) and the pixel coordinate of each block's center."""
    count = -(-length // block)
    starts = np.arange(count) * block
    sizes = np.minimum(block, length - starts)
    return count, starts + (sizes - 1) / 2.0


def _interpolation_matrix(length: int, centers: np.ndarray) -> np.ndarray:
    """(length, n_blocks) matrix of 1-D linear interpolation weights from block
    centers to the pixel coordinates 0..length-1 (clamped at the ends), with every
    row scaled to unit Euclidean norm. For independent unit-variance block values b the
    interpolated value w . b then has unit variance at every pixel (module docstring)."""
    n_blocks = centers.size
    weights = np.zeros((length, n_blocks))
    if n_blocks == 1:
        weights[:, 0] = 1.0
        return weights
    pixels = np.arange(length, dtype=np.float64)
    lower = np.clip(np.searchsorted(centers, pixels, side="right") - 1, 0, n_blocks - 2)
    fraction = np.clip((pixels - centers[lower]) / (centers[lower + 1] - centers[lower]), 0.0, 1.0)
    rows = np.arange(length)
    weights[rows, lower] = 1.0 - fraction
    weights[rows, lower + 1] = fraction
    return weights / np.linalg.norm(weights, axis=1, keepdims=True)


class _BlockGrid:
    """Block structure of one image size: draws one value per block and interpolates
    block values to the pixels with unit-variance bilinear weights."""

    def __init__(self, height: int, width: int, block: int) -> None:
        self.n_rows, row_centers = _block_layout(height, block)
        self.n_cols, col_centers = _block_layout(width, block)
        self.row_weights = _interpolation_matrix(height, row_centers)
        self.col_weights = _interpolation_matrix(width, col_centers)

    def block_shape(self) -> tuple[int, int]:
        """(n_rows, n_cols) of the block lattice."""
        return self.n_rows, self.n_cols

    def interpolate(self, block_values: np.ndarray) -> np.ndarray:
        """(H, W) bilinear interpolation of an (n_rows, n_cols) block lattice to the pixels."""
        return self.row_weights @ block_values @ self.col_weights.T


@lru_cache(maxsize=BLOCK_CACHE_SIZE)
def _block_grid(height: int, width: int, block: int) -> _BlockGrid:
    """The (cached) block grid of an image size; it is immutable in use."""
    return _BlockGrid(height, width, block)


@lru_cache(maxsize=BLOCK_CACHE_SIZE)
def _fixed_pattern_unit(height: int, width: int, block: int, seed: int) -> np.ndarray:
    """(H, W) unit-variance static pattern: one standard normal value per block from a generator
    seeded with ``seed``, interpolated to the pixels. Cached because it is identical in every frame;
    read-only so that no caller can change the shared array."""
    grid = _block_grid(height, width, block)
    pattern = grid.interpolate(np.random.default_rng(seed).standard_normal(grid.block_shape()))
    pattern.setflags(write=False)
    return pattern


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------
def _imitation_matcher(true_depth: np.ndarray, surface: np.ndarray, readable: np.ndarray,
                       model: SyntheticSensorModel) -> tuple[np.ndarray, np.ndarray]:
    """Step 3: (matched depth in mm, read mask) of the imitation matcher.

    The depth is the weighted mean of the readable true depths in the square window, weight 1 for
    back-surface and ``front_preference`` for front-surface pixels. Both sums are box filters of
    masked images (``uniform_filter`` returns window means; the common 1 / window^2 factor cancels
    in the ratio), so there is no Python loop over pixels. Outside the image counts as unreadable
    (``cval`` 0 enters both sums and the fill)."""
    size = model.matching_window_px
    weights = np.where(surface == SURFACE_FRONT, model.front_preference, 1.0) * readable
    weighted_depth = np.where(readable, true_depth, 0.0) * weights
    weight_sum = uniform_filter(weights, size=size, mode="constant", cval=0.0)
    depth_sum = uniform_filter(weighted_depth, size=size, mode="constant", cval=0.0)
    fill = uniform_filter(readable.astype(np.float64), size=size, mode="constant", cval=0.0)
    has_weight = weight_sum > WEIGHT_SUM_TOLERANCE
    matched = np.where(has_weight, depth_sum / np.where(has_weight, weight_sum, 1.0), np.nan)
    # A pixel whose own ray hits nothing is always a no-read, whatever its neighbors say.
    read = (surface != SURFACE_NONE) & has_weight & (fill >= model.min_window_fill)
    return matched, read


def _resolved_target(model: SyntheticSensorModel, target: TwoPlaneTarget,
                     pose_camera: RigidTransform) -> TwoPlaneTarget:
    """The target as the imitation matcher can resolve it: the disks and cutouts whose diameter in pixels,
    D_px = D f_x / Z at the feature center, is below ``model.min_feature_diameter_px`` are left out, so a small
    disk is not seen (the back plate shows) and a small cutout is filled in (the front plate shows). The analyses
    still use the full target as ground truth, which is what makes such a feature a miss. Blank and post sites and the
    squares of the edge targets are never removed."""
    if model.min_feature_diameter_px <= 0.0:
        return target
    kept = []
    for feature in target.features:
        if feature.kind in (FEATURE_DISK, FEATURE_CUTOUT):
            depth_mm = float(target.feature_center_camera(pose_camera, feature)[2])
            if model.geometry.diameter_in_pixels(feature.diameter_mm, depth_mm) < model.min_feature_diameter_px:
                continue
        kept.append(feature)
    return target if len(kept) == len(target.features) else replace(target, features=kept)


def render_frame(model: SyntheticSensorModel, target: TwoPlaneTarget, pose_camera: RigidTransform,
                 rng: np.random.Generator, elapsed_hours: float = 0.0) -> RenderedFrame:
    """Render one frame of ``target`` at ``pose_camera`` (target -> camera, mm) with the seven steps of
    the module docstring. ``rng`` supplies the temporal noise (the fixed pattern has its own seed);
    ``elapsed_hours`` is the time since the session's start for the drift term. INDICATIVE only."""
    stereo = StereoGeometry.from_sensor_geometry(model.geometry)
    camera = stereo.camera
    rays = camera.ray_directions()
    height, width = camera.height, camera.width
    disparity_constant = model.geometry.disparity_constant_mm_px()

    # Step 1: ideal ray cast from the left camera center, of the target as the matcher resolves it.
    target = _resolved_target(model, target, pose_camera)
    hit = target.intersect_rays(pose_camera, np.zeros(3), rays, include_posts=model.posts_visible)
    surface = hit.surface
    true_depth = hit.point_camera[..., 2]

    # Step 2: geometric visibility. Front-surface points are always visible; only back-plate points
    # can be hidden from the right camera or the projector (module docstring, second remark).
    visibility = surface == SURFACE_FRONT
    back = surface == SURFACE_BACK
    if np.any(back):
        visibility[back] = geometric_visibility(target, pose_camera, hit.point_camera[back], stereo,
                                                model.require_projector, include_posts=model.posts_visible)
    readable = (surface != SURFACE_NONE) & visibility

    # Step 3: imitation matcher.
    matched_depth, read = _imitation_matcher(true_depth, surface, readable, model)
    # Work with a safe depth everywhere so that no NaN or zero enters the arithmetic of steps 4 to 6.
    depth = np.where(read, matched_depth, PLACEHOLDER_DEPTH_MM)

    # Step 4: disparity noise, one draw per block, interpolated back, with the incidence multiplier.
    # The draw is made for every frame so that the random stream does not depend on the geometry.
    grid = _block_grid(height, width, model.fixed_pattern_block_px)
    noise_unit = grid.interpolate(rng.standard_normal(grid.block_shape()))
    plane_normal = target.front_plane_camera(pose_camera)[1]
    cosine = np.maximum(incidence_cosine(camera, plane_normal), MIN_NOISE_COSINE)
    sigma_disparity = model.disparity_noise_px * cosine ** (-model.noise_incidence_exponent)
    disparity = disparity_constant / depth + sigma_disparity * noise_unit

    # Step 5: fixed pattern (mm at the pixel's depth, converted to disparity: dd = -k dZ / Z^2),
    # disparity quantizer, back to depth, depth LSB.
    if model.fixed_pattern_amplitude_mm > 0.0:
        pattern_unit = _fixed_pattern_unit(height, width, model.fixed_pattern_block_px, model.fixed_pattern_seed)
        pattern_mm = (model.fixed_pattern_amplitude_mm * pattern_unit
                      * (depth / FIXED_PATTERN_REFERENCE_DEPTH_MM) ** 2)
        disparity = disparity - disparity_constant * pattern_mm / depth ** 2
    if model.disparity_quantum_px > 0.0:
        disparity = np.round(disparity / model.disparity_quantum_px) * model.disparity_quantum_px
    read = read & (disparity > MIN_VALID_DISPARITY_PX)
    final_depth = disparity_constant / np.where(read, disparity, 1.0)
    if model.output_lsb_mm > 0.0:
        final_depth = np.round(final_depth / model.output_lsb_mm) * model.output_lsb_mm

    # Step 6: drift of the whole field.
    final_depth = final_depth + model.drift_mm_per_hour * elapsed_hours
    read = read & (final_depth > 0.0)

    # Step 7: points along the unit rays so that z equals the final depth; zeros where there is no read.
    ray_z = rays[..., 2]
    xyz = np.where(read[..., None], rays * (final_depth / ray_z)[..., None], 0.0).astype(np.float32)
    return RenderedFrame(depth=np.where(read, final_depth, np.nan), xyz=xyz, true_surface=surface,
                         true_depth=true_depth, visibility=visibility)


def write_frame(path: str | Path, frame: RenderedFrame, geometry: SensorGeometry, extra_header: dict[str, Any]) -> None:
    """Write a rendered frame as a ``.mc`` file with the vendored writer: header fx, fy, cx, cy (from
    the geometry), h (image width), v (image height), cameraName "synthetic", version 2 and the
    entries of ``extra_header``; one matrix, "XYZ", the float32 (H, W, 3) points."""
    height, width = frame.xyz.shape[:2]
    header: dict[str, Any] = {
        "fx": float(geometry.require("sensor_fx_px")), "fy": float(geometry.require("sensor_fy_px")),
        "cx": float(geometry.require("sensor_cx_px")), "cy": float(geometry.require("sensor_cy_px")),
        "h": int(width), "v": int(height),
        "cameraName": SYNTHETIC_CAMERA_NAME, "version": SYNTHETIC_HEADER_VERSION,
    }
    header.update(extra_header)
    write_matcloud(path, header, {XYZ_CHANNEL_NAME: frame.xyz})
