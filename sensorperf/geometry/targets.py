"""
The two-plane targets of the characterization procedure (document, Section 3):
a front surface with knife-edge features standing a gap G in front of a back
plate of the same finish. One geometry describes the noise plate (T2), the
raised square (T3a), the square window (T3b), the disk arrays (T4) and the
cutout arrays (T5), and the same code answers, for any ray from any viewpoint,
which surface the ray meets and where.

Target frame (used everywhere in this package)
    origin   the target's reference point on the FRONT face (the plate center)
    x        the target's H direction (image columns when fronto-parallel)
    y        the target's V direction (image rows when fronto-parallel)
    z        pointing AWAY from the sensor, into the target
    front face: the plane z = 0; back plate: the plane z = +G
A fronto-parallel target at depth Z centered at (H, V) therefore has the pose
RigidTransform(identity, [H, V, Z]) (target -> camera). This differs from the
earlier calibration repository's board convention (outward normal toward the
sensor); the difference is deliberate so that the gap is a positive z.

Front material
    Which points of the z = 0 plane are solid is decided per target kind:
    plate               the whole front extent
    raised square       the (slanted) square only; the back plate is seen around it
    square window       the front extent minus the square; the back plate is seen through it
    disk array          the disks (and, if asked for, the posts); the back plate elsewhere
    cutout array        the front extent minus the holes; the back plate through the holes
Because every boundary is chamfered from the back (Section 3.3), no ray meets
a side wall: a ray either hits the front material at z = 0 or passes the front
plane and hits the back plate at z = G (or nothing, if the back plate is
absent or missed). That is exactly what :meth:`TwoPlaneTarget.intersect_rays`
computes, and :meth:`TwoPlaneTarget.visible_from` applies the same test to the
segment from a viewpoint (right camera, projector) to a back-plate point, which
gives the geometric visibility of Sections 12 and 14.

Signed distance
    :meth:`Feature.signed_distance_mm` is positive on the front-material side of
    the feature's edge and negative on the back-plate side, in millimeters on
    the front plane. Divided by the pixel footprint p(Z) it is the s of Sections
    11 and 14.

As-built values
    Analyses use the measured feature sizes (Section 3.3, "As-built record").
    :func:`load_targets_asbuilt` reads targets_asbuilt.csv and
    :meth:`TargetSet.apply_asbuilt` overwrites the nominal values with it.
"""
from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from sensorperf.geometry.camera import PinholeCamera
from sensorperf.geometry.transforms import RigidTransform
from sensorperf.parameters import (
    CharacterizationParameters, SensorGeometry, TARGET_CUTOUTS_LARGE, TARGET_CUTOUTS_SMALL, TARGET_DISKS_LARGE,
    TARGET_DISKS_SMALL, TARGET_NOISE_PLATE, TARGET_RAISED_SQUARE, TARGET_REGISTRATION_PLATE, TARGET_SQUARE_WINDOW,
)

# ---------------------------------------------------------------------------
# Kinds
# ---------------------------------------------------------------------------
FEATURE_DISK = "disk"
"""A raised disk on a hidden post: front material inside the circle."""
FEATURE_CUTOUT = "cutout"
"""A hole in the front plate: back plate inside the circle."""
FEATURE_BLANK = "blank"
"""A blank site: no feature; its diameter is the search-window size it serves."""
FEATURE_POST = "post"
"""A post-only control site: the support post without a disk."""
FEATURE_SQUARE_RAISED = "square_raised"
"""The raised square of T3a: front material inside the square."""
FEATURE_SQUARE_WINDOW = "square_window"
"""The square window of T3b: back plate inside the square."""
FEATURE_KINDS = (FEATURE_DISK, FEATURE_CUTOUT, FEATURE_BLANK, FEATURE_POST, FEATURE_SQUARE_RAISED,
                 FEATURE_SQUARE_WINDOW)

TARGET_KIND_PLATE = "plate"
TARGET_KIND_RAISED_SQUARE = "raised_square"
TARGET_KIND_SQUARE_WINDOW = "square_window"
TARGET_KIND_DISK_ARRAY = "disk_array"
TARGET_KIND_CUTOUT_ARRAY = "cutout_array"
TARGET_KINDS = (TARGET_KIND_PLATE, TARGET_KIND_RAISED_SQUARE, TARGET_KIND_SQUARE_WINDOW, TARGET_KIND_DISK_ARRAY,
                TARGET_KIND_CUTOUT_ARRAY)

SURFACE_NONE = 0
"""The ray meets neither plane (open background or outside both plates)."""
SURFACE_FRONT = 1
"""The ray meets the front material at z = 0."""
SURFACE_BACK = 2
"""The ray passes the front plane and meets the back plate at z = G."""

SQUARE_EDGE_LEFT, SQUARE_EDGE_RIGHT, SQUARE_EDGE_TOP, SQUARE_EDGE_BOTTOM = "left", "right", "top", "bottom"
SQUARE_EDGES = (SQUARE_EDGE_LEFT, SQUARE_EDGE_RIGHT, SQUARE_EDGE_TOP, SQUARE_EDGE_BOTTOM)
"""The four edges of a square feature, named in the target frame before the slant
(left/right are the near-vertical edges that measure H resolution, top/bottom the
near-horizontal edges that measure V resolution)."""

PARALLEL_RAY_TOLERANCE = 1.0e-12
"""A ray whose z component in the target frame is smaller than this never meets the planes."""
OUTLINE_POINTS_DEFAULT = 256
"""Points on a projected feature outline."""
ARRAY_ROW_PACKING_FRACTION = 0.9
"""Fraction of the plate width used by a row of features when laying out an array."""
ARRAY_MIN_PLATE_SIZE_MM = 100.0
"""An array plate is never smaller than this (width and height), whatever the ladder."""
BLANK_SITE_WINDOW_FRACTION = 1.0
"""A blank site's diameter equals the diameter of the level it serves (one blank per level)."""
POST_NOMINAL_DIAMETER_MM = 0.5
"""Nominal post diameter used when laying out an array (replaced by the as-built value)."""
POST_SITE_LEVELS = 2
"""Number of post-only control sites per disk array (the two thinnest posts)."""
ASBUILT_COLUMNS = ("target_id", "site_id", "kind", "x_mm", "y_mm", "diameter_mm", "diameter_uncertainty_mm",
                   "land_mm", "bevel_deg", "rotation_deg", "level_index")
"""Columns of targets_asbuilt.csv (Section 3.3)."""
TARGETS_FILE_NAME = "targets.json"
"""Name of the target definition file written by the planner and read by the analyses."""
ASBUILT_FILE_NAME = "targets_asbuilt.csv"
"""Name of the as-built record (Section 3.3, Section 9)."""


# ---------------------------------------------------------------------------
# Features
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Feature:
    """One feature on a target's front face. ``diameter_mm`` is the circle
    diameter for disks, cutouts, blanks and posts and the SIDE for squares."""

    site_id: str
    kind: str
    x_mm: float
    y_mm: float
    diameter_mm: float
    diameter_uncertainty_mm: float = 0.0
    land_mm: float = 0.0
    bevel_deg: float = 45.0
    rotation_deg: float = 0.0
    """In-plane rotation of a square feature (the slant of Section 6.1), degrees, counterclockwise about +z."""
    level_index: int | None = None
    """Rung of the diameter ladder (disks, cutouts, blanks) or None."""

    def is_front_material(self) -> bool:
        """True for kinds that add front material (disk, post, raised square)."""
        return self.kind in (FEATURE_DISK, FEATURE_POST, FEATURE_SQUARE_RAISED)

    def is_circular(self) -> bool:
        return self.kind in (FEATURE_DISK, FEATURE_CUTOUT, FEATURE_BLANK, FEATURE_POST)

    def true_area_mm2(self) -> float:
        """pi D^2 / 4 for circles, side^2 for squares (A_true of Section 12, Step 6)."""
        if self.is_circular():
            return math.pi * self.diameter_mm ** 2 / 4.0
        return self.diameter_mm ** 2

    def local_coordinates(self, x_mm, y_mm) -> tuple[np.ndarray, np.ndarray]:
        """Coordinates relative to the feature center, rotated back by the feature's rotation."""
        dx = np.asarray(x_mm, dtype=np.float64) - self.x_mm
        dy = np.asarray(y_mm, dtype=np.float64) - self.y_mm
        angle = math.radians(self.rotation_deg)
        c, s = math.cos(angle), math.sin(angle)
        return c * dx + s * dy, -s * dx + c * dy

    def inside(self, x_mm, y_mm) -> np.ndarray:
        """True where the point lies inside the feature's outline (circle or square)."""
        lx, ly = self.local_coordinates(x_mm, y_mm)
        if self.is_circular():
            return lx ** 2 + ly ** 2 <= (self.diameter_mm / 2.0) ** 2
        half = self.diameter_mm / 2.0
        return (np.abs(lx) <= half) & (np.abs(ly) <= half)

    def signed_distance_mm(self, x_mm, y_mm, edge: str | None = None) -> np.ndarray:
        """Signed distance to the feature's edge, positive on the FRONT-material
        side (Sections 11 and 14). For a square, ``edge`` selects one of
        SQUARE_EDGES and gives the signed distance to that edge's infinite line
        (positive toward the square's interior for a raised square, toward the
        exterior for a window); None gives the distance to the nearest edge."""
        lx, ly = self.local_coordinates(x_mm, y_mm)
        half = self.diameter_mm / 2.0
        if self.is_circular():
            inward = half - np.sqrt(lx ** 2 + ly ** 2)             # positive inside the circle
        else:
            if edge is None:
                # Exact signed distance to a square: positive inside.
                qx, qy = np.abs(lx) - half, np.abs(ly) - half
                outside = np.sqrt(np.maximum(qx, 0.0) ** 2 + np.maximum(qy, 0.0) ** 2)
                inside = np.minimum(np.maximum(qx, qy), 0.0)
                inward = -(outside + inside)
            elif edge == SQUARE_EDGE_LEFT:
                inward = lx + half
            elif edge == SQUARE_EDGE_RIGHT:
                inward = half - lx
            elif edge == SQUARE_EDGE_TOP:
                inward = ly + half
            elif edge == SQUARE_EDGE_BOTTOM:
                inward = half - ly
            else:
                raise ValueError(f"unknown square edge {edge!r}; expected one of {SQUARE_EDGES}")
        # Inside the outline is front material for disks, posts and the raised square,
        # and back plate for cutouts, blanks (no edge, but the sign convention is kept) and the window.
        return inward if self.is_front_material() else -inward

    def outline_points(self, count: int = OUTLINE_POINTS_DEFAULT) -> np.ndarray:
        """(count, 2) points along the feature's outline in target-frame mm."""
        angle = math.radians(self.rotation_deg)
        c, s = math.cos(angle), math.sin(angle)
        half = self.diameter_mm / 2.0
        if self.is_circular():
            t = np.linspace(0.0, 2.0 * math.pi, count, endpoint=False)
            lx, ly = half * np.cos(t), half * np.sin(t)
        else:
            per_side = max(count // 4, 1)
            u = np.linspace(-half, half, per_side, endpoint=False)
            lx = np.concatenate([u, np.full(per_side, half), -u, np.full(per_side, -half)])
            ly = np.concatenate([np.full(per_side, -half), u, np.full(per_side, half), -u])
        return np.column_stack([self.x_mm + c * lx - s * ly, self.y_mm + s * lx + c * ly])

    def edge_line(self, edge: str) -> tuple[np.ndarray, np.ndarray]:
        """A point on and the unit direction of a square edge's line, target frame (mm)."""
        angle = math.radians(self.rotation_deg)
        c, s = math.cos(angle), math.sin(angle)
        half = self.diameter_mm / 2.0
        local_point = {SQUARE_EDGE_LEFT: (-half, 0.0), SQUARE_EDGE_RIGHT: (half, 0.0),
                       SQUARE_EDGE_TOP: (0.0, -half), SQUARE_EDGE_BOTTOM: (0.0, half)}[edge]
        local_dir = (0.0, 1.0) if edge in (SQUARE_EDGE_LEFT, SQUARE_EDGE_RIGHT) else (1.0, 0.0)
        point = np.array([self.x_mm + c * local_point[0] - s * local_point[1],
                          self.y_mm + s * local_point[0] + c * local_point[1]])
        direction = np.array([c * local_dir[0] - s * local_dir[1], s * local_dir[0] + c * local_dir[1]])
        return point, direction


# ---------------------------------------------------------------------------
# Targets
# ---------------------------------------------------------------------------
@dataclass
class SurfaceHit:
    """Result of intersecting rays with a target, arrays broadcast to the ray shape.

    surface: SURFACE_NONE / SURFACE_FRONT / SURFACE_BACK per ray
    point_camera: (..., 3) hit point in the camera frame (NaN where none)
    xy_target: (..., 2) hit point on its plane in target-frame mm (NaN where none)
    range_mm: (...) distance from the ray origin to the hit (NaN where none)
    """

    surface: np.ndarray
    point_camera: np.ndarray
    xy_target: np.ndarray
    range_mm: np.ndarray


@dataclass
class TwoPlaneTarget:
    """A target: front extent, optional back plate at a gap, and features."""

    target_id: str
    kind: str
    half_width_mm: float
    half_height_mm: float
    gap_mm: float | None = None
    """Front-to-back plate distance; None means no back plate (T1, T2, or the open-background variant)."""
    back_half_width_mm: float | None = None
    back_half_height_mm: float | None = None
    """Back plate half sizes; None means the same as the front plate."""
    features: list[Feature] = field(default_factory=list)
    finish: str = ""
    """Surface finish description (Section 3.2) for the record."""

    def __post_init__(self) -> None:
        if self.kind not in TARGET_KINDS:
            raise ValueError(f"unknown target kind {self.kind!r}; expected one of {TARGET_KINDS}")

    # -- derived sizes ------------------------------------------------------
    def back_extent(self) -> tuple[float, float]:
        return (self.half_width_mm if self.back_half_width_mm is None else self.back_half_width_mm,
                self.half_height_mm if self.back_half_height_mm is None else self.back_half_height_mm)

    def with_gap(self, gap_mm: float | None) -> "TwoPlaneTarget":
        """The same target with another gap (the spacers of Section 3.2) or no back plate."""
        return replace(self, gap_mm=gap_mm)

    def feature(self, site_id: str) -> Feature:
        for f in self.features:
            if f.site_id == site_id:
                return f
        raise KeyError(f"target {self.target_id} has no site {site_id!r}")

    def features_of_kind(self, *kinds: str) -> list[Feature]:
        return [f for f in self.features if f.kind in kinds]

    # -- front material -----------------------------------------------------
    def inside_front_extent(self, x_mm, y_mm) -> np.ndarray:
        return (np.abs(np.asarray(x_mm)) <= self.half_width_mm) & (np.abs(np.asarray(y_mm)) <= self.half_height_mm)

    def inside_back_extent(self, x_mm, y_mm) -> np.ndarray:
        bw, bh = self.back_extent()
        return (np.abs(np.asarray(x_mm)) <= bw) & (np.abs(np.asarray(y_mm)) <= bh)

    def front_material(self, x_mm, y_mm, include_posts: bool = False) -> np.ndarray:
        """True where the z = 0 plane is solid (module docstring, "Front material")."""
        x = np.asarray(x_mm, dtype=np.float64)
        y = np.asarray(y_mm, dtype=np.float64)
        extent = self.inside_front_extent(x, y)
        if self.kind == TARGET_KIND_PLATE:
            return extent
        if self.kind == TARGET_KIND_RAISED_SQUARE:
            solid = np.zeros(x.shape, dtype=bool)
            for f in self.features_of_kind(FEATURE_SQUARE_RAISED):
                solid |= f.inside(x, y)
            return solid
        if self.kind == TARGET_KIND_SQUARE_WINDOW:
            hole = np.zeros(x.shape, dtype=bool)
            for f in self.features_of_kind(FEATURE_SQUARE_WINDOW):
                hole |= f.inside(x, y)
            return extent & ~hole
        if self.kind == TARGET_KIND_DISK_ARRAY:
            solid = np.zeros(x.shape, dtype=bool)
            kinds = (FEATURE_DISK, FEATURE_POST) if include_posts else (FEATURE_DISK,)
            for f in self.features_of_kind(*kinds):
                solid |= f.inside(x, y)
            return solid & extent
        if self.kind == TARGET_KIND_CUTOUT_ARRAY:
            hole = np.zeros(x.shape, dtype=bool)
            for f in self.features_of_kind(FEATURE_CUTOUT):
                hole |= f.inside(x, y)
            return extent & ~hole
        raise AssertionError(self.kind)

    # -- ray casting --------------------------------------------------------
    def intersect_rays(self, pose_camera: RigidTransform, origin_camera, directions_camera,
                       include_posts: bool = False) -> SurfaceHit:
        """Which surface each ray meets, from an origin (camera frame, mm) along
        unit or non-unit directions (camera frame). ``pose_camera`` maps target
        -> camera. Only hits in the ray's forward direction count."""
        to_target = pose_camera.inverse()
        origin = to_target.apply_points(np.asarray(origin_camera, dtype=np.float64).reshape(3))
        dirs = to_target.apply_directions(np.asarray(directions_camera, dtype=np.float64))
        shape = dirs.shape[:-1]
        surface = np.zeros(shape, dtype=np.int8)
        xy = np.full(shape + (2,), np.nan)
        t_hit = np.full(shape, np.nan)
        dz = dirs[..., 2]
        moving = np.abs(dz) > PARALLEL_RAY_TOLERANCE
        safe_dz = np.where(moving, dz, 1.0)
        # Front plane z = 0.
        t_front = np.where(moving, -origin[2] / safe_dz, np.nan)
        front_ok = moving & (t_front > 0.0)
        px = origin[0] + t_front * dirs[..., 0]
        py = origin[1] + t_front * dirs[..., 1]
        hit_front = front_ok & self.front_material(np.where(front_ok, px, 0.0), np.where(front_ok, py, 0.0),
                                                   include_posts=include_posts)
        surface[hit_front] = SURFACE_FRONT
        xy[hit_front, 0] = px[hit_front]
        xy[hit_front, 1] = py[hit_front]
        t_hit[hit_front] = t_front[hit_front]
        # Back plane z = G for the rays that passed the front plane.
        if self.gap_mm is not None:
            t_back = np.where(moving, (self.gap_mm - origin[2]) / safe_dz, np.nan)
            back_ok = moving & ~hit_front & (t_back > 0.0)
            bx = origin[0] + t_back * dirs[..., 0]
            by = origin[1] + t_back * dirs[..., 1]
            hit_back = back_ok & self.inside_back_extent(np.where(back_ok, bx, 0.0), np.where(back_ok, by, 0.0))
            surface[hit_back] = SURFACE_BACK
            xy[hit_back, 0] = bx[hit_back]
            xy[hit_back, 1] = by[hit_back]
            t_hit[hit_back] = t_back[hit_back]
        hit = surface != SURFACE_NONE
        point_target = origin + t_hit[..., None] * dirs
        point_camera = pose_camera.apply_points(np.where(hit[..., None], point_target, 0.0))
        point_camera = np.where(hit[..., None], point_camera, np.nan)
        range_mm = np.where(hit, t_hit * np.linalg.norm(dirs, axis=-1), np.nan)
        return SurfaceHit(surface=surface, point_camera=point_camera, xy_target=xy, range_mm=range_mm)

    def visible_from(self, pose_camera: RigidTransform, points_camera, viewpoint_camera,
                     include_posts: bool = False) -> np.ndarray:
        """True where the straight segment from the viewpoint (camera frame) to each
        point (camera frame, on the front face or the back plate) is not blocked by
        the front material. Points on the front face are always visible; a back-plate
        point is visible when the segment crosses the front plane at a point that is
        not solid (knife-edge targets: no side walls). Points that are NaN give False."""
        to_target = pose_camera.inverse()
        points = to_target.apply_points(np.asarray(points_camera, dtype=np.float64))
        view = to_target.apply_points(np.asarray(viewpoint_camera, dtype=np.float64).reshape(3))
        finite = np.all(np.isfinite(points), axis=-1)
        pz = np.where(finite, points[..., 2], 0.0)
        on_front = finite & (np.abs(pz) <= PARALLEL_RAY_TOLERANCE * max(1.0, abs(self.gap_mm or 1.0)))
        behind = finite & (pz > 0.0)
        dz = pz - view[2]
        moving = np.abs(dz) > PARALLEL_RAY_TOLERANCE
        safe_dz = np.where(moving, dz, 1.0)
        # Fraction of the way from the viewpoint to the point at which z = 0 is crossed.
        fraction = np.where(moving, -view[2] / safe_dz, np.nan)
        crossing_x = view[0] + fraction * (points[..., 0] - view[0])
        crossing_y = view[1] + fraction * (points[..., 1] - view[1])
        crosses = behind & moving & (fraction > 0.0) & (fraction < 1.0)
        blocked = crosses & self.front_material(np.where(crosses, crossing_x, 0.0), np.where(crosses, crossing_y, 0.0),
                                                include_posts=include_posts)
        return on_front | (behind & ~blocked)

    # -- projection helpers --------------------------------------------------
    def feature_center_camera(self, pose_camera: RigidTransform, feature: Feature) -> np.ndarray:
        """The feature center on the front plane, camera frame (mm)."""
        return pose_camera.apply_points(np.array([feature.x_mm, feature.y_mm, 0.0]))

    def project_outline(self, camera: PinholeCamera, pose_camera: RigidTransform, feature: Feature,
                        count: int = OUTLINE_POINTS_DEFAULT) -> np.ndarray:
        """(count, 2) image coordinates (u, v) of the feature's outline on the front plane."""
        xy = feature.outline_points(count)
        points = pose_camera.apply_points(np.column_stack([xy, np.zeros(len(xy))]))
        u, v, _ = camera.project(points)
        return np.column_stack([u, v])

    def front_plane_camera(self, pose_camera: RigidTransform) -> tuple[np.ndarray, np.ndarray]:
        """(point, unit normal) of the front plane in the camera frame; the normal
        points toward the sensor (-z of the target)."""
        return pose_camera.translation.copy(), -pose_camera.rotation[:, 2]

    def back_plane_camera(self, pose_camera: RigidTransform) -> tuple[np.ndarray, np.ndarray]:
        """(point, unit normal) of the back plate in the camera frame."""
        if self.gap_mm is None:
            raise ValueError(f"target {self.target_id} has no back plate")
        point = pose_camera.apply_points(np.array([0.0, 0.0, self.gap_mm]))
        return point, -pose_camera.rotation[:, 2]

    def pixel_xy_on_front_plane(self, camera: PinholeCamera, pose_camera: RigidTransform) -> np.ndarray:
        """(H, W, 2) target-frame (x, y) where each pixel's ray meets the front plane
        z = 0 (whether or not it is solid there); NaN where the ray is parallel."""
        hit = TwoPlaneTarget(self.target_id, TARGET_KIND_PLATE, np.inf, np.inf).intersect_rays(
            pose_camera, np.zeros(3), camera.ray_directions())
        return hit.xy_target

    # -- serialization ------------------------------------------------------
    def to_dict(self) -> dict[str, Any]:
        document = asdict(self)
        return document

    @classmethod
    def from_dict(cls, document: dict[str, Any]) -> "TwoPlaneTarget":
        features = [Feature(**f) for f in document.get("features", [])]
        fields_ = {k: v for k, v in document.items() if k != "features"}
        return cls(features=features, **fields_)


# ---------------------------------------------------------------------------
# Target set, as-built record
# ---------------------------------------------------------------------------
@dataclass
class TargetSet:
    """All targets of a session, by id."""

    targets: dict[str, TwoPlaneTarget] = field(default_factory=dict)

    def add(self, target: TwoPlaneTarget) -> None:
        self.targets[target.target_id] = target

    def get(self, target_id: str, gap_mm: float | None = None) -> TwoPlaneTarget:
        """The target, with the manifest's gap applied (None keeps the definition's gap)."""
        target = self.targets[target_id]
        return target if gap_mm is None else target.with_gap(gap_mm)

    def save(self, path: str | Path) -> Path:
        document = {"targets": [t.to_dict() for t in self.targets.values()]}
        Path(path).write_text(json.dumps(document, indent=2), encoding="utf-8")
        return Path(path)

    @classmethod
    def load(cls, path: str | Path) -> "TargetSet":
        document = json.loads(Path(path).read_text(encoding="utf-8"))
        targets = cls()
        for entry in document["targets"]:
            targets.add(TwoPlaneTarget.from_dict(entry))
        return targets

    def apply_asbuilt(self, rows: Iterable[dict[str, Any]]) -> list[str]:
        """Overwrite feature values with as-built measurements (see
        load_targets_asbuilt). Returns the site ids that were updated; a row
        naming an unknown target or site raises KeyError."""
        updated = []
        for row in rows:
            target = self.targets[row["target_id"]]
            site_id = row["site_id"]
            index = next((i for i, f in enumerate(target.features) if f.site_id == site_id), None)
            if index is None:
                raise KeyError(f"target {target.target_id} has no site {site_id!r} (targets_asbuilt.csv)")
            current = target.features[index]
            values = {k: row[k] for k in ("x_mm", "y_mm", "diameter_mm", "diameter_uncertainty_mm", "land_mm",
                                           "bevel_deg", "rotation_deg") if row.get(k) is not None}
            target.features[index] = replace(current, **values)
            updated.append(site_id)
        return updated

    def write_asbuilt_template(self, path: str | Path) -> Path:
        """Write targets_asbuilt.csv with the nominal values, for the metrologist to overwrite."""
        with Path(path).open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(ASBUILT_COLUMNS)
            for target in self.targets.values():
                for f in target.features:
                    writer.writerow([target.target_id, f.site_id, f.kind, repr(f.x_mm), repr(f.y_mm),
                                     repr(f.diameter_mm), repr(f.diameter_uncertainty_mm), repr(f.land_mm),
                                     repr(f.bevel_deg), repr(f.rotation_deg),
                                     "" if f.level_index is None else f.level_index])
        return Path(path)


def load_targets_asbuilt(path: str | Path) -> list[dict[str, Any]]:
    """Rows of targets_asbuilt.csv (ASBUILT_COLUMNS) with numbers converted; an
    empty numeric cell becomes None (meaning: keep the nominal value)."""
    rows: list[dict[str, Any]] = []
    with Path(path).open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        missing = [c for c in ("target_id", "site_id") if c not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"{path}: missing columns {missing}")
        for index, raw in enumerate(reader):
            row: dict[str, Any] = {"target_id": raw["target_id"].strip(), "site_id": raw["site_id"].strip(),
                                   "kind": (raw.get("kind") or "").strip()}
            for column in ("x_mm", "y_mm", "diameter_mm", "diameter_uncertainty_mm", "land_mm", "bevel_deg",
                           "rotation_deg"):
                text = (raw.get(column) or "").strip()
                try:
                    row[column] = float(text) if text else None
                except ValueError as error:
                    raise ValueError(f"{path} row {index}: {column} is not a number: {text!r}") from error
            text = (raw.get("level_index") or "").strip()
            row["level_index"] = int(text) if text else None
            rows.append(row)
    return rows


# ---------------------------------------------------------------------------
# Standard targets from the parameters (Section 3.2)
# ---------------------------------------------------------------------------
def make_registration_plate(params: CharacterizationParameters, half_width_mm: float = 200.0,
                            half_height_mm: float = 150.0) -> TwoPlaneTarget:
    """T1: a flat plate (about 400 x 300 mm) carrying the fiducial pattern."""
    return TwoPlaneTarget(TARGET_REGISTRATION_PLATE, TARGET_KIND_PLATE, half_width_mm, half_height_mm)


def make_noise_plate(params: CharacterizationParameters) -> TwoPlaneTarget:
    """T2: the uniform matte plate of NOISE_PLATE_SIZE_MM."""
    width, height = params.noise_plate_size_mm
    return TwoPlaneTarget(TARGET_NOISE_PLATE, TARGET_KIND_PLATE, width / 2.0, height / 2.0)


def make_edge_target(params: CharacterizationParameters, kind: str, gap_mm: float,
                     plate_half_size_mm: tuple[float, float] | None = None) -> TwoPlaneTarget:
    """T3a (kind TARGET_KIND_RAISED_SQUARE) or T3b (TARGET_KIND_SQUARE_WINDOW): a
    square of EDGE_SQUARE_SIZE_MM rotated by EDGE_SLANT_DEG, with the back plate
    (T3a) or the front plate (T3b) sized like the noise plate unless given."""
    if plate_half_size_mm is None:
        width, height = params.noise_plate_size_mm
        plate_half_size_mm = (width / 2.0, height / 2.0)
    if kind == TARGET_KIND_RAISED_SQUARE:
        target_id, feature_kind = TARGET_RAISED_SQUARE, FEATURE_SQUARE_RAISED
    elif kind == TARGET_KIND_SQUARE_WINDOW:
        target_id, feature_kind = TARGET_SQUARE_WINDOW, FEATURE_SQUARE_WINDOW
    else:
        raise ValueError("kind must be TARGET_KIND_RAISED_SQUARE or TARGET_KIND_SQUARE_WINDOW")
    square = Feature(site_id="square", kind=feature_kind, x_mm=0.0, y_mm=0.0, diameter_mm=params.edge_square_size_mm,
                     rotation_deg=params.edge_slant_deg)
    return TwoPlaneTarget(target_id, kind, plate_half_size_mm[0], plate_half_size_mm[1], gap_mm=gap_mm,
                          features=[square])


def _layout_rows(diameters_mm: list[float], isolation_mm: float, max_row_width_mm: float) -> list[list[int]]:
    """Greedy row packing of feature indices (largest first) so that neighbors are
    at least isolation_mm apart edge to edge and a row is at most max_row_width_mm wide."""
    order = sorted(range(len(diameters_mm)), key=lambda i: -diameters_mm[i])
    rows: list[list[int]] = []
    for index in order:
        placed = False
        for row in rows:
            width = sum(diameters_mm[i] for i in row) + diameters_mm[index] + isolation_mm * len(row)
            if width <= max_row_width_mm:
                row.append(index)
                placed = True
                break
        if not placed:
            rows.append([index])
    return rows


def make_feature_array(target_id: str, kind: str, diameters_mm: list[float], isolation_mm: float, gap_mm: float,
                       level_indices: list[int] | None = None, post_diameter_mm: float = POST_NOMINAL_DIAMETER_MM,
                       post_sites: int = POST_SITE_LEVELS, min_plate_mm: float = ARRAY_MIN_PLATE_SIZE_MM,
                       ) -> TwoPlaneTarget:
    """A disk (TARGET_KIND_DISK_ARRAY) or cutout (TARGET_KIND_CUTOUT_ARRAY) array
    carrying the given diameters, one blank site per diameter (for the matching
    search window) and, for disks, post-only control sites (Section 3.2). Features
    are packed in rows with at least ``isolation_mm`` between edges; the plate is
    sized to hold them with a half-isolation margin."""
    if kind not in (TARGET_KIND_DISK_ARRAY, TARGET_KIND_CUTOUT_ARRAY):
        raise ValueError("kind must be TARGET_KIND_DISK_ARRAY or TARGET_KIND_CUTOUT_ARRAY")
    feature_kind = FEATURE_DISK if kind == TARGET_KIND_DISK_ARRAY else FEATURE_CUTOUT
    if level_indices is None:
        level_indices = list(range(len(diameters_mm)))
    entries: list[tuple[str, str, float, int | None]] = []
    for diameter, level in zip(diameters_mm, level_indices):
        entries.append((f"{feature_kind}_{level:02d}", feature_kind, float(diameter), level))
        entries.append((f"blank_{level:02d}", FEATURE_BLANK, float(diameter) * BLANK_SITE_WINDOW_FRACTION, level))
    if kind == TARGET_KIND_DISK_ARRAY:
        for post_index in range(post_sites):
            entries.append((f"post_{post_index:02d}", FEATURE_POST, post_diameter_mm, None))
    sizes = [e[2] for e in entries]
    largest = max(sizes)
    total = sum(sizes) + isolation_mm * len(sizes)
    # A roughly square plate: row width about the square root of the packed area, at least the largest feature.
    row_width = max(math.sqrt(total * (largest + isolation_mm)), largest + isolation_mm, min_plate_mm)
    rows = _layout_rows(sizes, isolation_mm, row_width * ARRAY_ROW_PACKING_FRACTION)
    row_heights = [max(sizes[i] for i in row) for row in rows]
    total_height = sum(row_heights) + isolation_mm * (len(rows) + 1)
    total_width = max(sum(sizes[i] for i in row) + isolation_mm * (len(row) + 1) for row in rows)
    half_width = max(total_width, min_plate_mm) / 2.0
    half_height = max(total_height, min_plate_mm) / 2.0
    features: list[Feature] = []
    y = -total_height / 2.0 + isolation_mm
    for row, row_height in zip(rows, row_heights):
        row_width_used = sum(sizes[i] for i in row) + isolation_mm * (len(row) - 1)
        x = -row_width_used / 2.0
        for i in row:
            site_id, fkind, diameter, level = entries[i]
            features.append(Feature(site_id=site_id, kind=fkind, x_mm=x + diameter / 2.0, y_mm=y + row_height / 2.0,
                                    diameter_mm=diameter, level_index=level))
            x += diameter + isolation_mm
        y += row_height + isolation_mm
    return TwoPlaneTarget(target_id, kind, half_width, half_height, gap_mm=gap_mm, features=features)


def make_standard_target_set(params: CharacterizationParameters, geometry: SensorGeometry,
                             gap_mm: float | None = None) -> TargetSet:
    """T1, T2, T3a, T3b, T4-S, T4-L, T5-S and T5-L from the parameters: the
    diameter ladder of Section 3.2 is split at its middle rung into the small
    and large arrays; the feature isolation is FEATURE_ISOLATION_PX at Z_MIN.
    The gap defaults to GAP_SMALL_MM (the manifest's gap overrides it per pose)."""
    gap = params.gap_small_mm if gap_mm is None else gap_mm
    ladder = list(params.diameter_ladder_mm(geometry))
    isolation_mm = params.feature_isolation_px * geometry.pixel_footprint_mm(params.z_min_mm)
    split = len(ladder) // 2
    small, large = ladder[:split], ladder[split:]
    small_levels, large_levels = list(range(split)), list(range(split, len(ladder)))
    targets = TargetSet()
    targets.add(make_registration_plate(params))
    targets.add(make_noise_plate(params))
    targets.add(make_edge_target(params, TARGET_KIND_RAISED_SQUARE, gap))
    targets.add(make_edge_target(params, TARGET_KIND_SQUARE_WINDOW, gap))
    targets.add(make_feature_array(TARGET_DISKS_SMALL, TARGET_KIND_DISK_ARRAY, small, isolation_mm, gap, small_levels))
    targets.add(make_feature_array(TARGET_DISKS_LARGE, TARGET_KIND_DISK_ARRAY, large, isolation_mm, gap, large_levels))
    targets.add(make_feature_array(TARGET_CUTOUTS_SMALL, TARGET_KIND_CUTOUT_ARRAY, small, isolation_mm, gap,
                                   small_levels))
    targets.add(make_feature_array(TARGET_CUTOUTS_LARGE, TARGET_KIND_CUTOUT_ARRAY, large, isolation_mm, gap,
                                   large_levels))
    return targets


# ---------------------------------------------------------------------------
# Stereo geometry and geometric visibility (Sections 12 and 14)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class StereoGeometry:
    """The three viewpoints of the active stereo sensor in the left camera frame."""

    camera: PinholeCamera
    right_center_mm: tuple[float, float, float]
    projector_center_mm: tuple[float, float, float]

    @classmethod
    def from_sensor_geometry(cls, geometry: SensorGeometry) -> "StereoGeometry":
        """Left camera from the intrinsics; right camera at (+B, 0, 0); projector at
        PROJECTOR_OFFSET_MM. Raises MissingSensorValue for unfilled † values."""
        camera = PinholeCamera(int(geometry.require("image_width_px")), int(geometry.require("image_height_px")),
                               float(geometry.require("sensor_fx_px")), float(geometry.require("sensor_fy_px")),
                               float(geometry.require("sensor_cx_px")), float(geometry.require("sensor_cy_px")))
        baseline = float(geometry.require("sensor_baseline_mm"))
        projector = tuple(float(v) for v in geometry.require("projector_offset_mm"))
        return cls(camera, (baseline, 0.0, 0.0), projector)  # type: ignore[arg-type]

    def viewpoints(self, require_projector: bool) -> list[np.ndarray]:
        """Left camera, right camera and (optionally) projector centers, camera frame."""
        points = [np.zeros(3), np.asarray(self.right_center_mm, dtype=np.float64)]
        if require_projector:
            points.append(np.asarray(self.projector_center_mm, dtype=np.float64))
        return points


def geometric_visibility(target: TwoPlaneTarget, pose_camera: RigidTransform, points_camera, stereo: StereoGeometry,
                         require_projector: bool, include_posts: bool = False) -> np.ndarray:
    """V of Section 14, Step 1: True where a stereo read of the point is
    geometrically possible, i.e. the point is visible to the left camera, the
    right camera and, if required, lit by the projector."""
    visible = np.ones(np.asarray(points_camera).shape[:-1], dtype=bool)
    for viewpoint in stereo.viewpoints(require_projector):
        visible &= target.visible_from(pose_camera, points_camera, viewpoint, include_posts=include_posts)
    return visible


def fronto_parallel_pose(h_mm: float, v_mm: float, z_mm: float) -> RigidTransform:
    """Target -> camera pose of a fronto-parallel target whose reference point is at (H, V, Z)."""
    return RigidTransform(np.eye(3), np.array([h_mm, v_mm, z_mm], dtype=np.float64))


def tilted_pose(h_mm: float, v_mm: float, z_mm: float, tilt_axis: str, tilt_deg: float) -> RigidTransform:
    """A fronto-parallel pose rotated about the target's H axis ("H") or V axis ("V")
    through its reference point (the tilt sub-series of Section 5, Step 5)."""
    from scipy.spatial.transform import Rotation
    if tilt_axis == "H":
        axis = np.array([1.0, 0.0, 0.0])
    elif tilt_axis == "V":
        axis = np.array([0.0, 1.0, 0.0])
    else:
        raise ValueError("tilt_axis must be 'H' or 'V'")
    rotation = Rotation.from_rotvec(axis * math.radians(tilt_deg)).as_matrix()
    return RigidTransform(rotation, np.array([h_mm, v_mm, z_mm], dtype=np.float64))
