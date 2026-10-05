"""
Analysis C: true versus sensed area (procedure document, Section 12), Steps 1 to 12.

What it computes
    For every configuration of the C frames (target, gap, station, field position, sub-series) and for
    every measured feature of the array (the disks of T4, the cutouts of T5) the area the sensor reports
    for the feature, compared with the true area of the as-built feature (Step 6) and, for cutouts, with
    the area that stereo geometry allows the sensor to see (Step 7).

Conventions (docs/design/code_design.md, Section 4)
    Camera frame = left IR camera; pixel (u, v) = (column, row); depth is camera z in mm, NaN for no-reads;
    normalized height h = (Z_back - Z) / (Z_back - Z_front), 1 on the front surface and 0 on the back plate;
    areas in mm^2 on the feature plane, lengths in mm or in pixels of the station (p(Z) = Z / f_x).

How the steps are implemented (the step numbers are cited in the code)
    1  Window: the circle of radius D_px/2 + BOUNDARY_BAND_HALF_WIDTH_PX around the projected center
       (PoseGeometry.feature_window). A feature whose window is not entirely inside the image is skipped
       and counted in the details.
    2  Reference planes (common.reference_planes). Cutouts: the front plane is fitted to the front plate
       beyond all feature windows; the back plane to the back pixels beyond the band (when there are too few,
       as for small holes, the registered back plane stands in, which reference_planes reports). Disks: the
       front reference is the REGISTERED disk plane (the disk faces have too few pixels to fit), the back
       plane is fitted to the back plate beyond the band.
    3  Pixel classes in the window: front read (h >= FRONT_READ_HEIGHT_THRESHOLD), back read, no-read.
    4  Sensed pixels per frame: the front reads (disk) or back reads (cutout) that are 8-connected to the
       component at the projected center pixel (or the nearest labeled pixel within one pixel of it).
       a = Z_plane^2 / (f_x f_y cos phi) on the FEATURE plane. A_sensed, A_noread (no-reads inside the
       outline grown by one pixel) and their sum A_upper.
    5  Sub-pixel area: the frames of a pose are averaged, h is interpolated over the valid pixels of the
       window (linear, nearest where outside the hull), the h = ESF_HALF_HEIGHT iso-contour is extracted with
       contourpy, the closed contour enclosing the projected center (the outermost one) is deprojected onto
       the feature plane (fractional pixel -> ray/plane intersection) and its vector shoelace area is taken.
    6  A_true = Feature.true_area_mm2() (as-built).
    7  A_geo (cutouts): a regular grid of cells (spacing from AreaOptions) over the as-built outline on the back
       plane; a cell counts when its center is visible from the left camera, the right camera and (second
       version) the projector (TwoPlaneTarget.visible_from). A_geo = A_true x (visible cells / cells), so the
       grid cannot make A_geo exceed A_true. Disks: A_geo = A_true.
    8  Transfer curves: ratio_true = A_sensed / A_true and ratio_geo = A_sensed / A_geo against D_px = D f_x / Z, with
       the mean over all frames, the standard deviation of the pose means (phase effect) and the root-mean-square
       within-pose standard deviation (temporal effect). The curves are POOLED over the features of a plate and over
       all stations (the summary rows carry the feature identity in ``site_id`` and ``level_index``; the figures
       draw one curve per feature). SCALING (OVERLAP) TEST (redesign note, Section 5): where two neighboring
       features overlap in D_px, their curves are compared (analysis.overlap): the mean difference of the curves over
       the shared D_px range and whether zero lies inside its bootstrap interval, per (kind, gap) and per ratio
       (A_sensed / A_true, and A_sensed / A_geo of the projector version for cutouts). Agreement supports D_px as the
       governing variable; disagreement is attributed to sigma_tot(Z) (from ``previous["A"]``) and reported.
    9  Edge bias b = (D_s - D) / 2 with D_s = 2 sqrt(A_sensed / pi), in mm and px. b is fitted against Z
       (b = c0 + c1 Z) using the features in the LARGEST THIRD of the plate's ladder whose D_px is at least
       AreaOptions.bias_fit_min_d_px (well above D_50, where b should not depend on D).
    10 Consistency with B: an ideal disk of the as-built diameter (rasterized at AreaOptions.kernel_subsample
       samples per pixel) is blurred with a separable Gaussian kernel and thresholded at 0.5. The kernel comes
       from Analysis B when ``previous["B"]`` offers rise distances (rise_h_px, rise_v_px of its
       forward_model_terms; ASSUMPTION: a Gaussian whose 10-90 percent rise equals the rise distance, because
       the line spread function of B is not handed over). Without B the kernel is taken from the radial edge
       spread function of this analysis' own large features (same Gaussian assumption); the details say which.
       The prediction is made twice: pure blur-and-threshold (the specification) and with the edge offset
       s_50 (B's edge_offset_px, or the own ESF) applied as a growth of the front material, since a symmetric
       blur cannot shift the 0.5 crossing by itself. The sign of b is compared with the sign expected from
       s_50: front material grows by -s_50, so b_disk ~ -s_50 and b_cutout ~ +s_50 (a hole shrinks).
    11 Field sub-series (subseries "field") against the on-axis transfer curves of the same plate and gap;
       open-background cutouts (subseries "open", no back plate): no back plane exists, so only A_noread is
       defined, compared with A_true (fill-in by front-plate values shows as A_noread < A_true).
    12 Outputs: C_area_summary.csv, C_area_details.json, C_overlap_test.csv, figures (transfer curves with the
       overlap ranges shaded, b versus Z, phase spread versus D_px, predicted versus measured).

Shared helpers other modules import from here: the geometric visible area and the geometric limit diameter
(``geometric_visible_area_mm2``, ``geometric_limit_diameter_mm``, used by Analysis D, Step 8) and the Okabe-Ito
palette constants.
"""
from __future__ import annotations

import math
import warnings
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Mapping

import numpy as np
from scipy import ndimage
from scipy.interpolate import griddata
from scipy.special import ndtri

from sensorperf.analysis.common import (
    BinnedProfile, PoseGeometry, away_from_edges, connected_components, new_figure, pose_geometry,
    reference_planes, save_figure, write_csv_rows, write_json,
)
from sensorperf.features.planes import normalized_height, pixel_area_on_plane_mm2, plane_depth_at
from sensorperf.geometry.targets import (
    FEATURE_CUTOUT, FEATURE_DISK, Feature, StereoGeometry, TwoPlaneTarget, fronto_parallel_pose,
)
from sensorperf.geometry.transforms import RigidTransform
from sensorperf.io.capture_set import load_stack
from sensorperf.io.manifest import (
    SUBSERIES_FIELD, SUBSERIES_JITTER, SUBSERIES_MAIN, SUBSERIES_OPEN, FrameRecord, group_by_configuration,
    group_by_pose, select,
)
from sensorperf.analysis.overlap import (
    OverlapOptions, OverlapResult, TransferCurve, overlap_tests, sigma_tot_by_station,
)
from sensorperf.io.session import Session
from sensorperf.parameters import CharacterizationParameters, FIELD_POSITION_CENTER, PROCEDURE_AREA

# ---------------------------------------------------------------------------
# Palette (Okabe-Ito, colorblind safe) shared by the C, D and E figures
# ---------------------------------------------------------------------------
OKABE_ITO_BLACK = "#000000"
OKABE_ITO_ORANGE = "#E69F00"
OKABE_ITO_SKY_BLUE = "#56B4E9"
OKABE_ITO_BLUISH_GREEN = "#009E73"
OKABE_ITO_YELLOW = "#F0E442"
OKABE_ITO_BLUE = "#0072B2"
OKABE_ITO_VERMILLION = "#D55E00"
OKABE_ITO_REDDISH_PURPLE = "#CC79A7"
STATION_COLORS = (OKABE_ITO_BLUE, OKABE_ITO_VERMILLION, OKABE_ITO_BLUISH_GREEN, OKABE_ITO_ORANGE,
                  OKABE_ITO_REDDISH_PURPLE, OKABE_ITO_SKY_BLUE, OKABE_ITO_BLACK)
"""Colors cycled over stations (or other categories) in the figures."""
KIND_COLORS = {FEATURE_DISK: OKABE_ITO_VERMILLION, FEATURE_CUTOUT: OKABE_ITO_BLUE}
KIND_MARKERS = {FEATURE_DISK: "o", FEATURE_CUTOUT: "s"}
KIND_LINESTYLES = {FEATURE_DISK: "-", FEATURE_CUTOUT: "--"}
"""Disks are drawn with circles and solid lines, cutouts with squares and dashed lines (so both kinds
stay distinguishable without color)."""

# ---------------------------------------------------------------------------
# Output names
# ---------------------------------------------------------------------------
SUMMARY_FILE_NAME = "C_area_summary.csv"
DETAILS_FILE_NAME = "C_area_details.json"
OVERLAP_FILE_NAME = "C_overlap_test.csv"
OVERLAP_COLUMNS = ("kind", "gap_mm", "quantity", "feature_small", "feature_large", "status", "shared_low_px",
                   "shared_high_px", "points_small", "points_large", "mean_difference", "interval_lower",
                   "interval_upper", "agrees", "resamples", "sigma_tot_small_mm", "sigma_tot_large_mm", "note")
"""Columns of C_overlap_test.csv: one row per (kind, gap, ratio) and pair of neighboring features."""
OVERLAP_QUANTITIES = (("ratio_true", "A_sensed / A_true", "a_true_mm2"),
                      ("ratio_geo_projector", "A_sensed / A_geo (cameras + projector)", "a_geo_projector_mm2"))
"""(row key of the curve value, its label, row key of the denominator area) of the transfer curves that get the
overlap test."""
SUMMARY_COLUMNS = (
    "target_id", "kind", "gap_mm", "station_z_mm", "field", "subseries", "site_id", "level_index", "diameter_mm",
    "d_px", "a_true_mm2", "a_geo_cameras_mm2", "a_geo_projector_mm2", "a_sensed_mean_mm2",
    "a_sensed_phase_std_mm2", "a_sensed_temporal_std_mm2", "a_noread_mean_mm2", "a_upper_mean_mm2",
    "a_contour_mean_mm2", "ratio_true", "ratio_geo_cameras", "ratio_geo_projector", "b_mm", "b_px", "poses",
    "frames", "ratio_noread_true", "ratio_contour_pixelcount")
"""Columns of C_area_summary.csv: the specification's list (Step 12) followed by two extra columns
(A_noread / A_true, used by the open-background comparison, and the contour-to-pixel-count area ratio of Step 5)."""

MAIN_SUBSERIES = (SUBSERIES_MAIN, SUBSERIES_JITTER)
"""Sub-series labels of the main C configurations (the transfer curves and the b fit use these only)."""

# ---------------------------------------------------------------------------
# Numerical constants
# ---------------------------------------------------------------------------
NEIGHBORHOOD_RADIUS_PX = 1
"""Step 4: when the center pixel itself is not a sensed pixel, the nearest labeled pixel within this many
pixels of the center is used."""
MIN_INTERPOLATION_POINTS = 4
"""Step 5: fewer valid pixels than this in the window and no contour is attempted."""
CONTOUR_CLOSE_TOLERANCE = 1.0e-9
"""Step 5: a contour whose end points differ by less than this (pixels) is closed."""
MIN_CONTOUR_POINTS = 4
"""Step 5: a closed contour needs at least this many points (including the repeated end point)."""
MIN_POSES_FOR_STD = 2
"""Standard deviations over poses or frames need at least this many values."""
MIN_STATIONS_FOR_FIT = 2
"""A linear fit of b against Z needs at least this many distinct stations."""
SQUARE_ROOT_PI_FACTOR = 2.0
"""D_s = 2 sqrt(A / pi): the diameter of the circle of the same area."""
KERNEL_SIGMA_EXTENT = 5.0
"""Step 10: the rasterized disk is padded by this many kernel sigmas on every side."""
KERNEL_THRESHOLD = 0.5
"""Step 10: the blurred ideal disk is thresholded at this value."""
GEO_LIMIT_CELLS_ACROSS = 64
"""Grid cells across the diameter when the geometric limit (D 0, Analysis D Step 8) is bisected."""
SIGN_AGREEMENT_MIN_D_PX = 1.0
"""Step 10 sign check: only features with at least this many pixels of diameter vote on the sign of b."""
GEO_VERSION_TIE_REL_TOL = 1.0e-6
"""Step 7: two A_geo versions whose comparison statistics agree to this relative tolerance are reported as a tie."""
PHASE_SPREAD_MIN_AREA_FRACTION = 0.0
"""Phase-spread plot: features with a mean sensed area at or below this fraction of A_true are omitted."""


def _gaussian_rise_factor(low: float, high: float) -> float:
    """The 10-90 percent (or low-high) rise of a unit-sigma Gaussian edge: Phi^-1(high) - Phi^-1(low)."""
    return float(ndtri(high) - ndtri(low))


# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class AreaOptions:
    """Tunable numerical settings of Analysis C that are not procedure parameters."""

    geo_grid_spacing_mm: float = 0.05
    """Step 7: the finest cell spacing of the A_geo grid, mm (used for small features)."""
    geo_cells_across_max: int = 100
    """Step 7: the grid never has more than this many cells across the diameter (coarser spacing for large
    features keeps the point count bounded)."""
    geo_cells_across_min: int = 40
    """Step 7: the grid always has at least this many cells across the diameter, even for features smaller
    than 40 fine spacings."""
    contour_pad_px: int = 2
    """Step 5: the interpolation region extends this many pixels beyond the feature window."""
    bias_fit_min_d_px: float = 4.0
    """Step 9: a feature enters the b-versus-Z fit only when D_px is at least this many pixels (a named
    multiple of the pixel), in addition to being in the largest third of the plate's ladder."""
    bias_fit_min_ratio: float = 0.25
    """Step 9: ... and only when the sensor resolves it at all, that is when its mean sensed area is at least this
    fraction of A_true (a feature the sensor erases or fills in has no meaningful b; b is then about -D/2)."""
    bias_fit_top_fraction: float = 1.0 / 3.0
    """Step 9: the fraction of the plate's ladder (largest diameters) that is 'well above D_50'."""
    kernel_subsample: int = 8
    """Step 10: samples per pixel of the rasterized ideal disk."""
    esf_min_d_px: float = 4.0
    """Step 10 fallback: the own radial ESF uses features at least this many pixels in diameter."""
    geo_limit_min_diameter_mm: float = 0.01
    """Geometric limit search: lower end of the bisection bracket, mm."""
    geo_limit_bisection_steps: int = 40
    """Geometric limit search: bisection steps (the bracket shrinks by 2^-steps in ln D)."""
    make_geo_areas: bool = True
    """Compute the A_geo columns (set False to skip the ray casts when only pixel areas are wanted)."""


# ---------------------------------------------------------------------------
# Result
# ---------------------------------------------------------------------------
@dataclass
class AreaResult:
    """Everything Analysis C produced: the summary rows, the details and the notes."""

    rows: list[dict[str, Any]]
    """One dict per configuration x feature with the SUMMARY_COLUMNS keys."""
    details: dict[str, Any]
    """Contents of C_area_details.json (b-versus-Z fits, kernel comparison, field and open comparisons)."""
    options: AreaOptions
    notes: list[str] = field(default_factory=list)
    z_reference_mm: float = field(default_factory=lambda: CharacterizationParameters().z_reference_mm)
    """The reference station (Z_REFERENCE_MM) used by forward_model_terms."""

    def forward_model_terms(self) -> dict[str, Any]:
        """Terms for forward_model_parameters.json: ``edge_bias_px`` (the growth of the front material at the
        edge in pixels at the mid station, from the large features: b of the disks and -b of the cutouts,
        averaged over the kinds present, so a positive value means foreground fattening), the signed
        ``edge_bias_disk_px`` and ``edge_bias_cutout_px`` and the fitted b(Z) coefficients per kind
        (``edge_bias_fit_<kind>``: intercept in mm and slope in mm per mm of Z). Missing values are omitted."""
        terms: dict[str, Any] = {}
        fits = self.details.get("bias_fits", [])
        growth: list[float] = []
        for kind in (FEATURE_DISK, FEATURE_CUTOUT):
            candidates = [f for f in fits if f["kind"] == kind and f.get("b_px_at_reference") is not None
                          and math.isfinite(f["b_px_at_reference"])]
            if not candidates:
                continue
            value = float(np.mean([f["b_px_at_reference"] for f in candidates]))
            terms[f"edge_bias_{kind}_px"] = value
            growth.append(value if kind == FEATURE_DISK else -value)
            coefficients = [f for f in candidates if f.get("slope_mm_per_mm") is not None
                            and math.isfinite(f["slope_mm_per_mm"])]
            if coefficients:
                terms[f"edge_bias_fit_{kind}"] = {
                    "intercept_mm": float(np.mean([f["intercept_mm"] for f in coefficients])),
                    "slope_mm_per_mm": float(np.mean([f["slope_mm_per_mm"] for f in coefficients]))}
        if growth:
            terms["edge_bias_px"] = float(np.mean(growth))
        return terms


# ---------------------------------------------------------------------------
# Step 7 helpers (also used by Analysis D, Step 8)
# ---------------------------------------------------------------------------
def _grid_spacing_mm(diameter_mm: float, options: AreaOptions) -> float:
    """Cell spacing of the A_geo grid: at least geo_cells_across_min cells across the diameter, at most
    geo_cells_across_max, and as fine as geo_grid_spacing_mm where that fits between the two."""
    fine = max(options.geo_grid_spacing_mm, diameter_mm / options.geo_cells_across_max)
    return min(fine, diameter_mm / options.geo_cells_across_min)


def visible_fraction(target: TwoPlaneTarget, pose_camera: RigidTransform, feature: Feature,
                     stereo: StereoGeometry, with_projector: bool, spacing_mm: float) -> float:
    """Step 7: the fraction of the cells of a regular grid over the circular outline, taken on the back plane,
    whose centers are visible from the left camera, the right camera and (when ``with_projector``) the projector.
    NaN when the target has no back plate. Cells centered outside the outline are not counted."""
    if target.gap_mm is None:
        return math.nan
    radius = feature.diameter_mm / 2.0
    cells = max(int(math.ceil(2.0 * radius / spacing_mm)), 1)
    spacing = 2.0 * radius / cells
    offsets = (np.arange(cells) + 0.5) * spacing - radius
    gx, gy = np.meshgrid(offsets, offsets)
    inside = gx ** 2 + gy ** 2 <= radius ** 2
    if not inside.any():
        return 0.0
    x = feature.x_mm + gx[inside]
    y = feature.y_mm + gy[inside]
    points_target = np.column_stack([x, y, np.full(x.shape, float(target.gap_mm))])
    points_camera = pose_camera.apply_points(points_target)
    visible = target.inside_back_extent(x, y)
    for viewpoint in stereo.viewpoints(with_projector):
        visible = visible & target.visible_from(pose_camera, points_camera, viewpoint)
    return float(np.count_nonzero(visible)) / float(x.size)


def geometric_visible_area_mm2(target: TwoPlaneTarget, pose_camera: RigidTransform, feature: Feature,
                               stereo: StereoGeometry, with_projector: bool,
                               options: AreaOptions = AreaOptions()) -> float:
    """A_geo of a cutout (Section 12, Step 7): the part of the as-built outline on the back plate that every
    required viewpoint sees through the hole, in mm^2. With ``with_projector`` False only the two cameras are
    required. NaN without a back plate. For a disk face A_geo = A_true (returned directly)."""
    if feature.kind == FEATURE_DISK:
        return feature.true_area_mm2()
    fraction = visible_fraction(target, pose_camera, feature, stereo, with_projector,
                                _grid_spacing_mm(feature.diameter_mm, options))
    return feature.true_area_mm2() * fraction if math.isfinite(fraction) else math.nan


def geometric_limit_diameter_mm(target: TwoPlaneTarget, station_z_mm: float, stereo: StereoGeometry,
                                with_projector: bool, max_diameter_mm: float,
                                options: AreaOptions = AreaOptions()) -> float:
    """The cutout diameter at which A_geo reaches zero (Analysis D, Step 8): below it no part of the back plate
    is seen by every required viewpoint through the hole. A probe cutout at the plate center of a fronto-parallel
    pose at ``station_z_mm`` is used, in the plate's own gap, and the diameter is found by bisection in ln D
    between options.geo_limit_min_diameter_mm and ``max_diameter_mm`` (a grid of GEO_LIMIT_CELLS_ACROSS cells
    across each trial diameter, so the limit is overestimated by about one grid cell, 1.6 percent). Returns NaN
    when the plate has no back plate or the largest diameter is still not visible, and the lower end of the
    bracket when even that is visible."""
    if target.gap_mm is None:
        return math.nan
    pose = fronto_parallel_pose(0.0, 0.0, station_z_mm)

    def nonempty(diameter_mm: float) -> bool:
        probe = Feature(site_id="geo_probe", kind=FEATURE_CUTOUT, x_mm=0.0, y_mm=0.0, diameter_mm=diameter_mm)
        probe_target = replace(target, features=[probe])
        return visible_fraction(probe_target, pose, probe, stereo, with_projector,
                                diameter_mm / GEO_LIMIT_CELLS_ACROSS) > 0.0

    low, high = options.geo_limit_min_diameter_mm, float(max_diameter_mm)
    if nonempty(low):
        return low
    if not nonempty(high):
        return math.nan
    for _ in range(options.geo_limit_bisection_steps):
        middle = math.sqrt(low * high)
        if nonempty(middle):
            high = middle
        else:
            low = middle
    return high


# ---------------------------------------------------------------------------
# Per-frame and per-pose pixel work (Steps 3 to 5)
# ---------------------------------------------------------------------------
def window_inside_image(geometry: PoseGeometry, feature: Feature, margin_px: float) -> bool:
    """True when the whole feature window (radius D_px/2 + margin) lies inside the image."""
    u0, v0 = geometry.feature_center_px(feature)
    radius = feature.diameter_mm / geometry.pixel_footprint_mm / 2.0 + margin_px
    camera = geometry.camera
    return (u0 - radius >= -0.5 and u0 + radius <= camera.width - 0.5
            and v0 - radius >= -0.5 and v0 + radius <= camera.height - 0.5)


def component_at_center(mask: np.ndarray, center_uv: tuple[float, float]) -> np.ndarray:
    """Step 4: the 8-connected component of ``mask`` that contains the pixel at the projected center. When that
    pixel is not in the mask, the nearest labeled pixel within NEIGHBORHOOD_RADIUS_PX pixels of the center is
    used; if there is none the result is empty. Returns a boolean image."""
    labels, count = connected_components(mask)
    if count == 0:
        return np.zeros(mask.shape, dtype=bool)
    u0, v0 = center_uv
    row, col = int(round(v0)), int(round(u0))
    best_label, best_distance = 0, math.inf
    for dr in range(-NEIGHBORHOOD_RADIUS_PX, NEIGHBORHOOD_RADIUS_PX + 1):
        for dc in range(-NEIGHBORHOOD_RADIUS_PX, NEIGHBORHOOD_RADIUS_PX + 1):
            r, c = row + dr, col + dc
            if 0 <= r < mask.shape[0] and 0 <= c < mask.shape[1] and labels[r, c] > 0:
                distance = math.hypot(c - u0, r - v0)
                if distance < best_distance:
                    best_label, best_distance = int(labels[r, c]), distance
    return labels == best_label if best_label > 0 else np.zeros(mask.shape, dtype=bool)


def _point_in_polygon(polygon_uv: np.ndarray, u: float, v: float) -> bool:
    """Even-odd ray casting test of a point against a closed polygon (N, 2)."""
    x, y = polygon_uv[:, 0], polygon_uv[:, 1]
    x2, y2 = np.roll(x, -1), np.roll(y, -1)
    crosses = (y > v) != (y2 > v)
    with np.errstate(divide="ignore", invalid="ignore"):
        x_at = x + (v - y) * (x2 - x) / (y2 - y)
    return bool(np.count_nonzero(crosses & (u < x_at)) % 2 == 1)


def _vector_area(points_xyz: np.ndarray) -> float:
    """Area of a planar 3-D polygon (N, 3): half the norm of the sum of cross products of successive vertices
    (the vector area; it equals the shoelace area for a polygon in a plane parallel to the image)."""
    nxt = np.roll(points_xyz, -1, axis=0)
    return 0.5 * float(np.linalg.norm(np.cross(points_xyz, nxt).sum(axis=0)))


def subpixel_contour_area_mm2(h_image: np.ndarray, geometry: PoseGeometry, feature: Feature, plane_point,
                              plane_normal, params_level: float, options: AreaOptions,
                              margin_px: float) -> float:
    """Step 5: the area enclosed by the h = ``params_level`` iso-contour of an (averaged) normalized-height image
    around the projected feature center, in mm^2 on the feature plane. The height is interpolated over the valid
    pixels of the window region (linear inside their hull, nearest outside); the contour comes from contourpy;
    the polygon is the outermost closed contour that encloses the center, deprojected through each fractional
    pixel's ray onto the plane. NaN when too few valid pixels, no closed contour encloses the center, or the
    interpolation fails."""
    import contourpy
    camera = geometry.camera
    u0, v0 = geometry.feature_center_px(feature)
    radius = feature.diameter_mm / geometry.pixel_footprint_mm / 2.0 + margin_px + options.contour_pad_px
    col0, col1 = max(int(math.floor(u0 - radius)), 0), min(int(math.ceil(u0 + radius)), camera.width - 1)
    row0, row1 = max(int(math.floor(v0 - radius)), 0), min(int(math.ceil(v0 + radius)), camera.height - 1)
    if col1 - col0 < 2 or row1 - row0 < 2:
        return math.nan
    us = np.arange(col0, col1 + 1, dtype=np.float64)
    vs = np.arange(row0, row1 + 1, dtype=np.float64)
    uu, vv = np.meshgrid(us, vs)
    block = h_image[row0:row1 + 1, col0:col1 + 1]
    valid = np.isfinite(block)
    if np.count_nonzero(valid) < MIN_INTERPOLATION_POINTS:
        return math.nan
    points = np.column_stack([uu[valid], vv[valid]])
    values = block[valid]
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            filled = griddata(points, values, (uu, vv), method="linear")
            missing = ~np.isfinite(filled)
            if missing.any():
                filled[missing] = griddata(points, values, (uu[missing], vv[missing]), method="nearest")
    except Exception:                                    # QhullError for degenerate point sets
        return math.nan
    generator = contourpy.contour_generator(x=us, y=vs, z=filled)
    best = math.nan
    for line in generator.lines(params_level):
        line = np.asarray(line)
        if line.shape[0] < MIN_CONTOUR_POINTS or np.max(np.abs(line[0] - line[-1])) > CONTOUR_CLOSE_TOLERANCE:
            continue                                     # open contour: runs off the region
        polygon = line[:-1]
        if not _point_in_polygon(polygon, u0, v0):
            continue
        depth = plane_depth_at(camera, polygon[:, 0], polygon[:, 1], plane_point, plane_normal)
        if not np.all(np.isfinite(depth)):
            continue
        area = _vector_area(camera.back_project(polygon[:, 0], polygon[:, 1], depth))
        if not math.isfinite(best) or area > best:
            best = area                                  # the outermost loop around the center
    return best


@dataclass
class _PoseMeasure:
    """What one pose contributes for one feature (frame arrays and pose-level values)."""

    sensed: np.ndarray
    noread: np.ndarray
    contour: float
    geo_cameras: float
    geo_projector: float


@dataclass
class _EsfAccumulator:
    """Running sums of h in bins of signed distance (Step 10 fallback kernel), one per feature kind."""

    sums: np.ndarray
    counts: np.ndarray


def _plane_of_feature(planes, geometry: PoseGeometry, use_front: bool):
    """(depth image, point, normal) of the plane the feature lives on: the front plane for a disk face (or when
    there is no back plate) and the back plane for a cutout. The fitted plane when there is one, else the
    registered plane."""
    if use_front:
        if planes.front_fit is not None:
            return planes.z_front, planes.front_fit.point, planes.front_fit.normal
        point, normal = geometry.target.front_plane_camera(geometry.pose_camera)
        return planes.z_front, point, normal
    if planes.back_fit is not None:
        return planes.z_back, planes.back_fit.point, planes.back_fit.normal
    point, normal = geometry.target.back_plane_camera(geometry.pose_camera)
    return planes.z_back, point, normal


def _measured_features(target: TwoPlaneTarget) -> list[Feature]:
    """The disks and cutouts of a target (blank and post sites are not measured here)."""
    return [f for f in target.features if f.kind in (FEATURE_DISK, FEATURE_CUTOUT)]


def _process_pose(session: Session, pose_records: list[FrameRecord], options: AreaOptions,
                  esf: dict[str, _EsfAccumulator] | None, skipped: dict[str, int]) -> dict[str, _PoseMeasure]:
    """Steps 1 to 5 and 7 for one pose: per feature the per-frame sensed and no-read areas, the sub-pixel
    contour area of the pose mean, and the A_geo values. Features whose window leaves the image are skipped."""
    params = session.params
    stack = load_stack(pose_records)
    camera = stack.camera
    geometry = pose_geometry(session, pose_records[0], camera)
    target = geometry.target
    measured = _measured_features(target)
    if not measured:
        return {}
    band = params.boundary_band_half_width_px
    has_back = geometry.z_back_gt is not None
    kind = measured[0].kind
    inside = [f for f in measured if window_inside_image(geometry, f, band)]
    for f in measured:
        if f not in inside:
            skipped[f.site_id] = skipped.get(f.site_id, 0) + 1
    if not inside:
        return {}
    with warnings.catch_warnings():                      # pixels never read in any frame are all-NaN slices
        warnings.simplefilter("ignore", RuntimeWarning)
        mean_depth = np.nanmean(stack.depth, axis=0)

    # Step 2: reference planes. Disk: registered front plane (empty front mask makes reference_planes fall back
    # to it). Cutout: the front plate outside every feature window.
    away_front, away_back = away_from_edges(geometry, params)
    windows = {f.site_id: geometry.feature_window(f, band) for f in inside}
    if kind == FEATURE_DISK:
        front_mask = np.zeros_like(away_front)
    else:
        outside_windows = np.ones_like(away_front)
        for f in measured:                               # all windows, also those of skipped features
            outside_windows &= ~geometry.feature_window(f, band)
        front_mask = away_front & outside_windows
    planes = reference_planes(mean_depth, geometry, params, front_mask=front_mask, back_mask=away_back)
    use_front_plane = kind == FEATURE_DISK or not has_back
    z_plane, plane_point, plane_normal = _plane_of_feature(planes, geometry, use_front_plane)
    area_image = np.nan_to_num(pixel_area_on_plane_mm2(camera, z_plane, plane_normal), nan=0.0)

    # Step 3: pixel classes need the back plane; without one (open background) only no-reads are defined.
    threshold = params.front_read_height_threshold
    heights = ([normalized_height(depth, planes.z_front, planes.z_back) for depth in stack.depth]
               if has_back else None)
    mean_height = normalized_height(mean_depth, planes.z_front, planes.z_back) if has_back else None

    measures: dict[str, _PoseMeasure] = {}
    for feature in inside:
        window = windows[feature.site_id]
        center = geometry.feature_center_px(feature)
        grown_outline = geometry.inside_outline(feature, 1.0)
        sensed = np.zeros(stack.frame_count)
        noread = np.zeros(stack.frame_count)
        for index in range(stack.frame_count):
            no_read_pixels = grown_outline & ~np.isfinite(stack.depth[index])
            noread[index] = float(area_image[no_read_pixels].sum())
            if not has_back:
                sensed[index] = math.nan                  # nothing to compare the depth against
                continue
            h = heights[index]
            reads = window & np.isfinite(h)
            wanted = (reads & (h >= threshold)) if kind == FEATURE_DISK else (reads & (h < threshold))
            component = component_at_center(wanted, center)       # Step 4
            sensed[index] = float(area_image[component].sum())
        contour = math.nan
        geo_c = geo_p = math.nan
        if has_back:
            contour = subpixel_contour_area_mm2(mean_height, geometry, feature, plane_point, plane_normal,
                                                params.esf_half_height, options, band)      # Step 5
            if options.make_geo_areas:
                geo_c = geometric_visible_area_mm2(target, geometry.pose_camera, feature, geometry.stereo, False,
                                                   options)                                  # Step 7
                geo_p = geometric_visible_area_mm2(target, geometry.pose_camera, feature, geometry.stereo, True,
                                                   options)
        measures[feature.site_id] = _PoseMeasure(sensed, noread, contour, geo_c, geo_p)

        # Step 10 fallback kernel: radial ESF of the large features of this pose (mean frame).
        if (esf is not None and mean_height is not None
                and feature.diameter_mm / geometry.pixel_footprint_mm >= options.esf_min_d_px):
            s = geometry.signed_distance_px(feature)
            use = window & np.isfinite(mean_height) & (np.abs(s) <= band)
            bins = esf[feature.kind].sums.size
            width = 2.0 * band / bins
            index_bins = np.clip(((s[use] + band) / width).astype(int), 0, bins - 1)
            esf[feature.kind].sums += np.bincount(index_bins, weights=mean_height[use], minlength=bins)
            esf[feature.kind].counts += np.bincount(index_bins, minlength=bins)
    return measures


# ---------------------------------------------------------------------------
# Step 10 helpers
# ---------------------------------------------------------------------------
def predicted_area_px2(diameter_px: float, sigma_h_px: float, sigma_v_px: float, growth_px: float,
                       subsample: int, kind: str) -> float:
    """Step 10: the area, in square pixels, of the region where an ideal disk (or hole) of the given diameter
    stays at or above KERNEL_THRESHOLD after a separable Gaussian blur. The disk of radius R + growth (disk) or
    R - growth (cutout; ``growth`` is the growth of the FRONT material) is rasterized at ``subsample`` samples
    per pixel, padded by KERNEL_SIGMA_EXTENT sigmas, blurred and thresholded. A hole is the complement of a
    front disk, so the same blur-and-threshold applies to its mask."""
    radius = diameter_px / 2.0 + (growth_px if kind == FEATURE_DISK else -growth_px)
    if radius <= 0.0:
        return 0.0
    extent = radius + KERNEL_SIGMA_EXTENT * max(sigma_h_px, sigma_v_px, 0.0) + 1.0
    cells = int(math.ceil(2.0 * extent * subsample))
    coordinates = (np.arange(cells) + 0.5 - cells / 2.0) / subsample
    xx, yy = np.meshgrid(coordinates, coordinates)
    mask = (xx ** 2 + yy ** 2 <= radius ** 2).astype(np.float64)
    blurred = ndimage.gaussian_filter(mask, sigma=(sigma_v_px * subsample, sigma_h_px * subsample), mode="constant")
    return float(np.count_nonzero(blurred >= KERNEL_THRESHOLD)) / float(subsample ** 2)


def _finite_mean(value: Any) -> float | None:
    """The mean of the finite numbers inside ``value`` (a number, or a list or dict of them); None if none."""
    if value is None:
        return None
    if isinstance(value, Mapping):
        value = list(value.values())
    if isinstance(value, (list, tuple, np.ndarray)):
        numbers = [x for x in (_finite_mean(v) for v in value) if x is not None]
        return float(np.mean(numbers)) if numbers else None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _kernel_from_b(previous: Mapping[str, Any] | None, rise_factor: float) -> dict[str, Any] | None:
    """The Gaussian kernel (sigma_h, sigma_v in px) and edge offset from Analysis B's forward-model terms
    (rise_h_px, rise_v_px, edge_offset_px) when present, else None. ASSUMPTION: Gaussian with the 10-90 percent
    rise equal to B's rise distance."""
    result = None if previous is None else previous.get("B")
    getter = getattr(result, "forward_model_terms", None)
    if getter is None:
        return None
    try:
        terms = getter()
    except Exception:                                    # B's own failure must not stop C
        return None
    rise_h, rise_v = _finite_mean(terms.get("rise_h_px")), _finite_mean(terms.get("rise_v_px"))
    if rise_h is None and rise_v is None:
        return None
    rise_h = rise_v if rise_h is None else rise_h
    rise_v = rise_h if rise_v is None else rise_v
    return {"source": "analysis B rise distances (Gaussian assumption)", "sigma_h_px": rise_h / rise_factor,
            "sigma_v_px": rise_v / rise_factor, "edge_offset_px": _finite_mean(terms.get("edge_offset_px"))}


def _kernel_from_own_esf(esf: dict[str, _EsfAccumulator], params, rise_factor: float,
                         band_px: float) -> dict[str, Any] | None:
    """Fallback kernel from the radial ESF of C's own large features (both kinds pooled): the 10-90 percent rise
    and the half-height crossing s_50 of the pooled mean-h profile. None when the profile has no crossings."""
    sums = sum(a.sums for a in esf.values())
    counts = sum(a.counts for a in esf.values())
    if counts.sum() <= 0:
        return None
    bins = counts.size
    width = 2.0 * band_px / bins
    centers = -band_px + (np.arange(bins) + 0.5) * width
    with np.errstate(invalid="ignore", divide="ignore"):
        mean = np.where(counts > 0, sums / counts, np.nan)
    profile = BinnedProfile(centers=centers, mean=mean, count=counts, std=np.zeros(bins))
    low = profile.interpolate_crossing(params.esf_rise_low)
    high = profile.interpolate_crossing(params.esf_rise_high)
    half = profile.interpolate_crossing(params.esf_half_height)
    if not (math.isfinite(low) and math.isfinite(high)) or high <= low:
        return None
    sigma = (high - low) / rise_factor
    return {"source": "own radial ESF of the large features (Gaussian assumption)", "sigma_h_px": sigma,
            "sigma_v_px": sigma, "edge_offset_px": half if math.isfinite(half) else None,
            "rise_10_90_px": high - low}


# ---------------------------------------------------------------------------
# Summary rows (Steps 6, 8, 9)
# ---------------------------------------------------------------------------
def _std(values: np.ndarray) -> float:
    return float(np.std(values, ddof=1)) if values.size >= MIN_POSES_FOR_STD else math.nan


def _nanmean_or_nan(values) -> float:
    """Mean of the finite entries, NaN when there are none."""
    arr = np.asarray(values, dtype=float)
    return float(np.mean(arr[np.isfinite(arr)])) if np.any(np.isfinite(arr)) else math.nan


def _row_for_feature(record: FrameRecord, feature: Feature, measures: list[_PoseMeasure],
                     p_mm: float, fx: float) -> dict[str, Any]:
    """One summary row from the poses' measures of one feature in one configuration."""
    a_true = feature.true_area_mm2()
    all_sensed = np.concatenate([m.sensed for m in measures])
    pose_means = np.array([np.mean(m.sensed) for m in measures])
    within = [np.var(m.sensed, ddof=1) for m in measures if m.sensed.size >= MIN_POSES_FOR_STD]
    all_noread = np.concatenate([m.noread for m in measures])
    sensed_valid = bool(np.all(np.isfinite(all_sensed)))
    a_sensed = float(np.mean(all_sensed)) if sensed_valid else math.nan
    a_noread = float(np.mean(all_noread))
    geo_c = _nanmean_or_nan([m.geo_cameras for m in measures])
    geo_p = _nanmean_or_nan([m.geo_projector for m in measures])
    contour = _nanmean_or_nan([m.contour for m in measures])
    d_s = SQUARE_ROOT_PI_FACTOR * math.sqrt(a_sensed / math.pi) if sensed_valid else math.nan
    b_mm = (d_s - feature.diameter_mm) / 2.0 if sensed_valid else math.nan

    def ratio(numerator: float, denominator: float) -> float:
        return numerator / denominator if (math.isfinite(numerator) and math.isfinite(denominator)
                                           and denominator > 0.0) else math.nan

    return {
        "target_id": record.target_id, "kind": feature.kind, "gap_mm": record.gap_mm,
        "station_z_mm": record.station_z_mm, "field": record.field, "subseries": record.subseries,
        "site_id": feature.site_id, "level_index": feature.level_index, "diameter_mm": feature.diameter_mm,
        "d_px": feature.diameter_mm * fx / record.station_z_mm, "a_true_mm2": a_true,
        "a_geo_cameras_mm2": geo_c, "a_geo_projector_mm2": geo_p, "a_sensed_mean_mm2": a_sensed,
        "a_sensed_phase_std_mm2": _std(pose_means) if sensed_valid else math.nan,
        "a_sensed_temporal_std_mm2": float(math.sqrt(np.mean(within))) if within and sensed_valid else math.nan,
        "a_noread_mean_mm2": a_noread,
        "a_upper_mean_mm2": a_sensed + a_noread if sensed_valid else math.nan,
        "a_contour_mean_mm2": contour,
        "ratio_true": ratio(a_sensed, a_true), "ratio_geo_cameras": ratio(a_sensed, geo_c),
        "ratio_geo_projector": ratio(a_sensed, geo_p),
        "b_mm": b_mm, "b_px": b_mm / p_mm if math.isfinite(b_mm) else math.nan,
        "poses": len(measures), "frames": int(all_sensed.size),
        "ratio_noread_true": ratio(a_noread, a_true), "ratio_contour_pixelcount": ratio(contour, a_sensed),
    }


def _bias_fits(rows: list[dict[str, Any]], options: AreaOptions, geometry_fx: float,
               z_reference_mm: float) -> list[dict[str, Any]]:
    """Step 9: b against Z per (kind, gap) from the main configurations, using the features in the largest third
    of each plate's ladder whose D_px is at least options.bias_fit_min_d_px. Returns one dict per kind and gap
    with the points used, the linear fit b_mm = intercept + slope Z (when at least two stations), the mean b in
    px per station, and b_px at the reference station."""
    main = [r for r in rows if r["subseries"] in MAIN_SUBSERIES and r["field"] == FIELD_POSITION_CENTER
            and r["gap_mm"] is not None and math.isfinite(r["b_mm"])]
    fits: list[dict[str, Any]] = []
    for (kind, gap), group in _group(main, ("kind", "gap_mm")).items():
        # The largest third of each plate's ladder (by diameter, per target), judged on all stations' features.
        eligible: list[dict[str, Any]] = []
        for target_id, target_rows in _group(group, ("target_id",)).items():
            diameters = sorted({r["diameter_mm"] for r in target_rows})
            count = max(int(math.ceil(len(diameters) * options.bias_fit_top_fraction)), 1)
            cut = diameters[-count]
            eligible += [r for r in target_rows if r["diameter_mm"] >= cut and r["d_px"] >= options.bias_fit_min_d_px
                         and r["ratio_true"] >= options.bias_fit_min_ratio]
        if not eligible:
            fits.append({"kind": kind, "gap_mm": gap, "points": 0,
                         "note": "no feature in the largest third reaches the minimum D_px"})
            continue
        z = np.array([r["station_z_mm"] for r in eligible])
        b_mm = np.array([r["b_mm"] for r in eligible])
        b_px = np.array([r["b_px"] for r in eligible])
        stations = sorted(set(z.tolist()))
        entry: dict[str, Any] = {
            "kind": kind, "gap_mm": eligible[0]["gap_mm"], "points": len(eligible), "stations_mm": stations,
            "sites": sorted({(r["target_id"], r["site_id"]) for r in eligible}),
            "selection": (f"largest {options.bias_fit_top_fraction:.3g} of the plate ladder, "
                          f"D_px >= {options.bias_fit_min_d_px:g} and A_sensed/A_true >= "
                          f"{options.bias_fit_min_ratio:g}"),
            "b_px_mean_by_station": {str(s): float(np.mean(b_px[z == s])) for s in stations},
            "b_px_mean": float(np.mean(b_px)), "b_px_std": _std(b_px),
        }
        if len(stations) >= MIN_STATIONS_FOR_FIT:
            slope, intercept = np.polyfit(z, b_mm, 1)
            entry.update(slope_mm_per_mm=float(slope), intercept_mm=float(intercept),
                         b_mm_at_reference=float(intercept + slope * z_reference_mm),
                         b_px_at_reference=float((intercept + slope * z_reference_mm) * geometry_fx / z_reference_mm))
        else:
            entry.update(slope_mm_per_mm=None, intercept_mm=None, b_mm_at_reference=None,
                         b_px_at_reference=float(np.mean(b_px)),
                         note="a single station: b(Z) not fitted, the mean b in px is reported")
        fits.append(entry)
    return fits


def _transfer_curves(rows: list[dict[str, Any]], key: str, denominator_key: str) -> list[TransferCurve]:
    """The curves of one (kind, gap) group of main rows, one per feature (target and site): the value ``key`` against
    D_px over the stations, with the standard error of the mean of the pose means,
    phase_std / sqrt(poses) / denominator (NaN when there is a single pose)."""
    curves = []
    for (target_id, site_id), members in _group(rows, ("target_id", "site_id")).items():
        usable = [r for r in members if math.isfinite(r[key])]
        if not usable:
            continue
        error = []
        for r in usable:
            area = r[denominator_key]
            spread = r["a_sensed_phase_std_mm2"]
            error.append(spread / math.sqrt(r["poses"]) / area
                         if (math.isfinite(spread) and math.isfinite(area) and area > 0.0) else math.nan)
        curves.append(TransferCurve(label=f"{target_id}:{site_id}", d_px=np.array([r["d_px"] for r in usable]),
                                    value=np.array([r[key] for r in usable]), std_error=np.array(error),
                                    station_z_mm=np.array([r["station_z_mm"] for r in usable])))
    return curves


def _overlap_section(rows: list[dict[str, Any]], params, previous: Mapping[str, Any] | None,
                     options: OverlapOptions) -> list[dict[str, Any]]:
    """Redesign note, Section 5: the scaling test between neighboring features, per (kind, gap) of the main on-axis
    rows and per transfer ratio. Returns one dict per tested pair (OVERLAP_COLUMNS keys)."""
    main = [r for r in rows if r["subseries"] in MAIN_SUBSERIES and r["field"] == FIELD_POSITION_CENTER
            and r["gap_mm"] is not None]
    sigma_tot = sigma_tot_by_station(previous)
    out: list[dict[str, Any]] = []
    for (kind, gap), group in _group(main, ("kind", "gap_mm")).items():
        for key, label, denominator in OVERLAP_QUANTITIES:
            if key == "ratio_geo_projector" and kind != FEATURE_CUTOUT:
                continue                                    # A_geo = A_true for a disk: the same curve twice
            results: list[OverlapResult] = overlap_tests(_transfer_curves(group, key, denominator),
                                                         params.confidence_level, params.bootstrap_resamples,
                                                         options, sigma_tot)
            for result in results:
                out.append({"kind": kind, "gap_mm": gap, "quantity": label, **result.as_row()})
    return out


def _group(rows: list[dict[str, Any]], keys: tuple[str, ...]) -> dict[Any, list[dict[str, Any]]]:
    """Rows grouped by the values of ``keys`` (a single key gives a bare value, several give a tuple)."""
    groups: dict[Any, list[dict[str, Any]]] = {}
    for row in rows:
        key = tuple(row[k] for k in keys)
        groups.setdefault(key if len(keys) > 1 else key[0], []).append(row)
    return groups


# ---------------------------------------------------------------------------
# The analysis
# ---------------------------------------------------------------------------
def run_area(session: Session, out_dir: Path, previous: Mapping[str, Any] | None,
             options: AreaOptions | None = None, overlap_options: OverlapOptions = OverlapOptions()
             ) -> AreaResult | None:
    """Analysis C (Section 12, Steps 1 to 11) on the procedure-"C" frames of the session. Returns None when
    there are none. ``previous`` maps the letters of analyses already run to their results; B's rise distances and
    edge offset are used in Step 10 when present. Nothing is written here; see ``write_outputs``."""
    options = AreaOptions() if options is None else options
    records = select(session.records, procedure=PROCEDURE_AREA)
    if not records:
        return None
    params = session.params
    fx = float(session.geometry.require("sensor_fx_px"))
    notes: list[str] = []
    rows: list[dict[str, Any]] = []
    skipped: dict[str, int] = {}
    esf = {kind: _EsfAccumulator(np.zeros(_esf_bins(params)), np.zeros(_esf_bins(params)))
           for kind in (FEATURE_DISK, FEATURE_CUTOUT)}
    open_rows: list[dict[str, Any]] = []

    for config_key, config_records in group_by_configuration(records).items():
        poses = group_by_pose(config_records)
        per_feature: dict[str, list[_PoseMeasure]] = {}
        features: dict[str, Feature] = {}
        for pose_records in poses.values():
            geometry_target = session.targets.get(pose_records[0].target_id, pose_records[0].gap_mm)
            for f in _measured_features(geometry_target):
                features[f.site_id] = f
            measures = _process_pose(session, pose_records, options, esf, skipped)
            for site_id, measure in measures.items():
                per_feature.setdefault(site_id, []).append(measure)
        first = config_records[0]
        p_mm = first.station_z_mm / fx
        for site_id, measure_list in per_feature.items():
            row = _row_for_feature(first, features[site_id], measure_list, p_mm, fx)
            rows.append(row)
            if first.subseries == SUBSERIES_OPEN or first.gap_mm is None:
                open_rows.append(row)

    if skipped:
        notes.append(f"{len(skipped)} feature site(s) had a window outside the image in at least one pose and "
                     f"were skipped for those poses: {sorted(skipped)}")
    z_reference = params.z_reference_mm
    fits = _bias_fits(rows, options, fx, z_reference)
    rise_factor = _gaussian_rise_factor(params.esf_rise_low, params.esf_rise_high)
    kernel = _kernel_from_b(previous, rise_factor) or _kernel_from_own_esf(
        esf, params, rise_factor, params.boundary_band_half_width_px)
    consistency, sign_check = _consistency_with_b(rows, kernel, options, fx, params)
    details: dict[str, Any] = {
        "bias_fits": fits,
        "kernel": kernel,
        "consistency_with_b": consistency,
        "sign_check": sign_check,
        "field_comparison": _field_comparison(rows),
        "open_background": _open_background(open_rows),
        "contour_check": _contour_check(rows),
        "options": options.__dict__,
        "skipped_feature_sites": skipped,
        "geometric_area_note": ("A_geo uses a grid over the as-built outline on the back plane; with the projector "
                                "it needs the left camera, the right camera and the projector, without it the two "
                                "cameras; which version tracks the data better is in 'geo_version_comparison'."),
        "geo_version_comparison": _geo_version_comparison(rows, options),
        "overlap_tests": _overlap_section(rows, params, previous, overlap_options),
        "notes": notes,
    }
    if kernel is None:
        notes.append("no kernel for Step 10: neither Analysis B nor the own ESF gave rise distances")
    return AreaResult(rows=rows, details=details, options=options, notes=notes, z_reference_mm=z_reference)


def _esf_bins(params) -> int:
    """Number of bins of the fallback ESF over the band, at BOUNDARY_BIN_WIDTH_PX."""
    return int(round(2.0 * params.boundary_band_half_width_px / params.boundary_bin_width_px))


def _consistency_with_b(rows: list[dict[str, Any]], kernel: dict[str, Any] | None, options: AreaOptions,
                        fx: float, params) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Step 10: predicted versus measured sensed area per main row, and the sign check of b against s_50."""
    if kernel is None:
        return [], {"available": False}
    s50 = kernel.get("edge_offset_px")
    out: list[dict[str, Any]] = []
    for row in rows:
        if row["subseries"] not in MAIN_SUBSERIES or row["field"] != FIELD_POSITION_CENTER or row["gap_mm"] is None:
            continue
        p_mm = row["station_z_mm"] / fx
        d_px = row["d_px"]
        pure = predicted_area_px2(d_px, kernel["sigma_h_px"], kernel["sigma_v_px"], 0.0, options.kernel_subsample,
                                  row["kind"]) * p_mm ** 2
        shifted = (predicted_area_px2(d_px, kernel["sigma_h_px"], kernel["sigma_v_px"], -s50,
                                      options.kernel_subsample, row["kind"]) * p_mm ** 2
                   if s50 is not None else math.nan)
        measured = row["a_sensed_mean_mm2"]
        out.append({
            "target_id": row["target_id"], "kind": row["kind"], "gap_mm": row["gap_mm"],
            "station_z_mm": row["station_z_mm"], "site_id": row["site_id"], "d_px": d_px,
            "a_true_mm2": row["a_true_mm2"], "a_measured_mm2": measured,
            "a_predicted_blur_mm2": pure, "a_predicted_blur_offset_mm2": shifted,
            "ratio_measured_over_blur": measured / pure if pure > 0 else math.nan,
            "ratio_measured_over_blur_offset": measured / shifted if (math.isfinite(shifted) and shifted > 0)
            else math.nan})
    sign: dict[str, Any] = {"available": True, "s50_px": s50, "kernel_source": kernel["source"]}
    for kind in (FEATURE_DISK, FEATURE_CUTOUT):
        eligible = [r for r in rows if r["kind"] == kind and r["subseries"] in MAIN_SUBSERIES
                    and r["field"] == FIELD_POSITION_CENTER and r["d_px"] >= max(options.bias_fit_min_d_px,
                                                                                  SIGN_AGREEMENT_MIN_D_PX)
                    and math.isfinite(r["b_px"])]
        b_mean = float(np.mean([r["b_px"] for r in eligible])) if eligible else math.nan
        expected = None
        if s50 is not None:
            expected = -s50 if kind == FEATURE_DISK else s50     # front material grows by -s50
        sign[kind] = {"b_px_mean": b_mean, "expected_sign_from_s50": None if expected is None else
                      (0 if expected == 0 else int(math.copysign(1, expected))),
                      "agrees": None if (expected is None or not math.isfinite(b_mean) or expected == 0)
                      else bool(math.copysign(1, b_mean) == math.copysign(1, expected)),
                      "features": len(eligible)}
    return out, sign


def _field_comparison(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Step 11: field sub-series ratios against the on-axis ratios of the same plate, gap, station and site."""
    on_axis = {(r["target_id"], r["gap_mm"], r["station_z_mm"], r["site_id"]): r for r in rows
               if r["subseries"] in MAIN_SUBSERIES and r["field"] == FIELD_POSITION_CENTER}
    out = []
    for row in rows:
        if row["subseries"] != SUBSERIES_FIELD:
            continue
        reference = on_axis.get((row["target_id"], row["gap_mm"], row["station_z_mm"], row["site_id"]))
        if reference is None:
            continue
        out.append({"target_id": row["target_id"], "kind": row["kind"], "gap_mm": row["gap_mm"],
                    "station_z_mm": row["station_z_mm"], "field": row["field"], "site_id": row["site_id"],
                    "d_px": row["d_px"], "ratio_true_field": row["ratio_true"],
                    "ratio_true_on_axis": reference["ratio_true"],
                    "difference": row["ratio_true"] - reference["ratio_true"]
                    if math.isfinite(row["ratio_true"]) and math.isfinite(reference["ratio_true"]) else math.nan,
                    "b_px_field": row["b_px"], "b_px_on_axis": reference["b_px"]})
    return out


def _open_background(open_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Step 11: open-background cutouts: A_noread against A_true (fill-in by front values: ratio < 1)."""
    return [{"target_id": r["target_id"], "station_z_mm": r["station_z_mm"], "site_id": r["site_id"],
             "d_px": r["d_px"], "a_true_mm2": r["a_true_mm2"], "a_noread_mean_mm2": r["a_noread_mean_mm2"],
             "ratio_noread_true": r["ratio_noread_true"]} for r in open_rows]


def _contour_check(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Step 5: the contour area against the pixel-count area, per kind, for the features with a sensed area
    of at least one square pixel's worth of the largest third (summary of ratio_contour_pixelcount)."""
    out: dict[str, Any] = {}
    for kind in (FEATURE_DISK, FEATURE_CUTOUT):
        values = [r["ratio_contour_pixelcount"] for r in rows if r["kind"] == kind
                  and r["subseries"] in MAIN_SUBSERIES and math.isfinite(r["ratio_contour_pixelcount"])]
        out[kind] = {"n": len(values), "median": float(np.median(values)) if values else math.nan,
                     "min": float(np.min(values)) if values else math.nan,
                     "max": float(np.max(values)) if values else math.nan}
    return out


def _geo_version_comparison(rows: list[dict[str, Any]], options: AreaOptions) -> dict[str, Any]:
    """Step 7: which A_geo version the data track better: for the cutouts the sensor resolves (A_sensed / A_true
    at least options.bias_fit_min_ratio, D_px at least options.bias_fit_min_d_px), the mean of |ln(A_sensed /
    A_geo)| for each version (the closer to zero, the better). When the two versions give the same area (for
    example the projector lies between the cameras, inside the overlap of their views) the data cannot tell them
    apart and the comparison says so."""
    out: dict[str, Any] = {}
    for name, key in (("cameras_only", "ratio_geo_cameras"), ("with_projector", "ratio_geo_projector")):
        values = [abs(math.log(r[key])) for r in rows if r["kind"] == FEATURE_CUTOUT and math.isfinite(r[key])
                  and r[key] > 0.0 and r["ratio_true"] >= options.bias_fit_min_ratio
                  and r["d_px"] >= options.bias_fit_min_d_px]
        out[name] = {"n": len(values), "mean_abs_log_ratio": float(np.mean(values)) if values else math.nan}
    both = [out[k]["mean_abs_log_ratio"] for k in out]
    if all(math.isfinite(v) for v in both):
        distinct = not math.isclose(both[0], both[1], rel_tol=GEO_VERSION_TIE_REL_TOL)
        out["better"] = min(out, key=lambda k: out[k]["mean_abs_log_ratio"]) if distinct else "tie"
    return out


# ---------------------------------------------------------------------------
# Outputs (Step 12)
# ---------------------------------------------------------------------------
def write_outputs(result: AreaResult, out_dir: Path) -> list[Path]:
    """Write C_area_summary.csv, C_area_details.json and the figures into ``out_dir``; returns the paths."""
    out_dir = Path(out_dir)
    written = [write_csv_rows(out_dir / SUMMARY_FILE_NAME, result.rows, SUMMARY_COLUMNS),
               write_json(out_dir / DETAILS_FILE_NAME, result.details),
               write_csv_rows(out_dir / OVERLAP_FILE_NAME, result.details.get("overlap_tests", []), OVERLAP_COLUMNS)]
    written += _figure_transfer(result, out_dir, "ratio_true", "A_sensed / A_true", "C_transfer_true")
    written += _figure_transfer(result, out_dir, "ratio_geo_projector", "A_sensed / A_geo (cameras + projector)",
                                "C_transfer_geo")
    written += _figure_bias(result, out_dir)
    written += _figure_phase_spread(result, out_dir)
    written += _figure_predicted(result, out_dir)
    return written


def log_axis(axis, which: str = "x") -> None:
    """Logarithmic axis without crowded minor tick labels (the minor ticks stay, unlabeled)."""
    from matplotlib.ticker import NullFormatter
    if which == "x":
        axis.set_xscale("log")
        axis.xaxis.set_minor_formatter(NullFormatter())
    else:
        axis.set_yscale("log")
        axis.yaxis.set_minor_formatter(NullFormatter())


def _main_rows(result: AreaResult) -> list[dict[str, Any]]:
    return [r for r in result.rows if r["subseries"] in MAIN_SUBSERIES and r["field"] == FIELD_POSITION_CENTER
            and r["gap_mm"] is not None]


def _feature_colors(rows: list[dict[str, Any]]) -> dict[Any, str]:
    """A color per feature identity (level index), cycled over the palette, so a feature keeps its color in every
    panel and figure."""
    levels = sorted({r["level_index"] for r in rows if r["level_index"] is not None})
    return {level: STATION_COLORS[index % len(STATION_COLORS)] for index, level in enumerate(levels)}


def _figure_transfer(result: AreaResult, out_dir: Path, key: str, label: str, stem: str) -> list[Path]:
    """Step 8: transfer curves against D_px, pooled over all stations: one figure per gap with one panel per kind
    (disks left, cutouts right), one curve per feature (color = feature, the points of a feature are its stations),
    error bars = phase spread, and the D_px ranges shared by neighboring features (the overlap test) shaded."""
    written: list[Path] = []
    rows = _main_rows(result)
    overlaps = result.details.get("overlap_tests", [])
    quantity_label = {k: text for k, text, _ in OVERLAP_QUANTITIES}[key]
    for gap in sorted({r["gap_mm"] for r in rows}):
        gap_rows = [r for r in rows if r["gap_mm"] == gap]
        kinds = [k for k in (FEATURE_DISK, FEATURE_CUTOUT) if any(r["kind"] == k for r in gap_rows)]
        figure, axes = new_figure(5.2 * len(kinds), 4.2)
        figure.clf()
        axes = figure.subplots(1, len(kinds), squeeze=False, sharey=True)[0]
        colors = _feature_colors(gap_rows)
        for axis, kind in zip(axes, kinds):
            for level, color in colors.items():
                points = sorted((r for r in gap_rows if r["kind"] == kind and r["level_index"] == level
                                 and math.isfinite(r[key])), key=lambda r: r["d_px"])
                if not points:
                    continue
                x = [r["d_px"] for r in points]
                y = [r[key] for r in points]
                denominator = [r["a_true_mm2"] if key == "ratio_true" else (r["a_geo_projector_mm2"]) for r in points]
                err = [r["a_sensed_phase_std_mm2"] / d if (math.isfinite(r["a_sensed_phase_std_mm2"])
                                                           and math.isfinite(d) and d > 0) else 0.0
                       for r, d in zip(points, denominator)]
                axis.errorbar(x, y, yerr=err, color=color, marker=KIND_MARKERS[kind],
                              linestyle=KIND_LINESTYLES[kind], capsize=2, label=f"feature {level}")
            for entry in overlaps:                          # the shared D_px ranges of the scaling test
                if ((entry["kind"], entry["gap_mm"], entry["quantity"]) == (kind, gap, quantity_label)
                        and math.isfinite(entry["shared_low_px"])):
                    axis.axvspan(entry["shared_low_px"], entry["shared_high_px"], color=OKABE_ITO_BLACK, alpha=0.06)
            axis.axhline(1.0, color=OKABE_ITO_BLACK, linewidth=0.6)
            log_axis(axis)
            axis.set_xlabel("feature diameter D_px = D f_x / Z (px)")
            axis.set_title(f"{kind}s, G = {gap:g} mm (all stations; shaded: shared D_px range of neighbors)",
                           fontsize=8)
            axis.grid(True, linewidth=0.3, which="both")
        axes[0].set_ylabel(label)
        axes[0].legend(fontsize=8)
        figure.tight_layout()
        written += save_figure(figure, out_dir / f"{stem}_G{gap:g}")
    return written


def _figure_bias(result: AreaResult, out_dir: Path) -> list[Path]:
    """Step 9: edge bias b (mm) against Z for the features that enter the fit, with the fitted lines."""
    fits = [f for f in result.details.get("bias_fits", []) if f.get("points")]
    if not fits:
        return []
    figure, axis = new_figure(6.5, 4.2)
    for index, fit in enumerate(fits):
        color = KIND_COLORS[fit["kind"]]
        linestyle = ":" if fit["gap_mm"] != min(f["gap_mm"] for f in fits) else "-"
        rows = [r for r in _main_rows(result) if r["kind"] == fit["kind"] and r["gap_mm"] == fit["gap_mm"]
                and (r["target_id"], r["site_id"]) in {tuple(s) for s in fit["sites"]}]
        axis.scatter([r["station_z_mm"] for r in rows], [r["b_mm"] for r in rows], color=color,
                     marker=KIND_MARKERS[fit["kind"]], s=18, alpha=0.7,
                     label=f"{fit['kind']}, G = {fit['gap_mm']:g} mm")
        if fit.get("slope_mm_per_mm") is not None:
            z = np.linspace(min(fit["stations_mm"]), max(fit["stations_mm"]), 20)
            axis.plot(z, fit["intercept_mm"] + fit["slope_mm_per_mm"] * z, color=color, linestyle=linestyle)
    axis.axhline(0.0, color=OKABE_ITO_BLACK, linewidth=0.6)
    axis.set_xlabel("station Z (mm)")
    axis.set_ylabel("edge bias b = (D_s - D) / 2 (mm)")
    axis.legend(fontsize=8)
    axis.grid(True, linewidth=0.3)
    return save_figure(figure, out_dir / "C_edge_bias_vs_z")


def _figure_phase_spread(result: AreaResult, out_dir: Path) -> list[Path]:
    """Step 8: the relative phase spread (std of pose means / mean sensed area) against D_px."""
    rows = [r for r in _main_rows(result) if math.isfinite(r["a_sensed_phase_std_mm2"])
            and r["a_sensed_mean_mm2"] > PHASE_SPREAD_MIN_AREA_FRACTION * r["a_true_mm2"]
            and r["a_sensed_mean_mm2"] > 0.0]
    if not rows:
        return []
    figure, axis = new_figure(6.5, 4.2)
    colors = _feature_colors(rows)
    for kind in (FEATURE_DISK, FEATURE_CUTOUT):
        for level, color in colors.items():
            points = [r for r in rows if r["kind"] == kind and r["level_index"] == level]
            if points:
                axis.scatter([r["d_px"] for r in points],
                             [r["a_sensed_phase_std_mm2"] / r["a_sensed_mean_mm2"] for r in points],
                             color=color, marker=KIND_MARKERS[kind], s=22, label=f"{kind}s, feature {level}")
    log_axis(axis, "x")
    log_axis(axis, "y")
    axis.set_xlabel("feature diameter D_px (px)")
    axis.set_ylabel("phase spread of sensed area (std / mean)")
    axis.legend(fontsize=7)
    axis.grid(True, linewidth=0.3, which="both")
    return save_figure(figure, out_dir / "C_phase_spread_vs_dpx")


def _figure_predicted(result: AreaResult, out_dir: Path) -> list[Path]:
    """Step 10: measured A_sensed / A_true against the blur-and-threshold prediction (pure and with the edge offset)."""
    comparison = result.details.get("consistency_with_b", [])
    if not comparison:
        return []
    figure, axes = new_figure(10.0, 4.2)
    figure.clf()
    axes = figure.subplots(1, 2, sharey=True)
    for axis, kind in zip(axes, (FEATURE_DISK, FEATURE_CUTOUT)):
        points = sorted((c for c in comparison if c["kind"] == kind), key=lambda c: c["d_px"])
        if not points:
            continue
        x = [c["d_px"] for c in points]
        axis.scatter(x, [c["a_measured_mm2"] / c["a_true_mm2"] for c in points], color=KIND_COLORS[kind],
                     marker=KIND_MARKERS[kind], s=20, label="measured")
        axis.plot(x, [c["a_predicted_blur_mm2"] / c["a_true_mm2"] for c in points], color=OKABE_ITO_BLACK,
                  linestyle="-", label="blur and threshold")
        shifted = [c["a_predicted_blur_offset_mm2"] / c["a_true_mm2"] for c in points]
        if np.any(np.isfinite(shifted)):
            axis.plot(x, shifted, color=OKABE_ITO_ORANGE, linestyle="--", label="blur and threshold + edge offset")
        log_axis(axis)
        axis.set_xlabel("feature diameter D_px (px)")
        axis.set_title(f"{kind}s")
        axis.grid(True, linewidth=0.3)
    axes[0].set_ylabel("area / A_true")
    axes[0].legend(fontsize=8)
    figure.tight_layout()
    return save_figure(figure, out_dir / "C_predicted_vs_measured")
