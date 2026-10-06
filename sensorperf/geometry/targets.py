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
from typing import Any, Iterable, Sequence

import numpy as np

from sensorperf.geometry.camera import PinholeCamera
from sensorperf.geometry.transforms import RigidTransform
from sensorperf.parameters import (
    CharacterizationParameters, SensorGeometry, TARGET_CUTOUTS, TARGET_DISKS, TARGET_NOISE_PLATE,
    TARGET_RAISED_SQUARE, TARGET_SQUARE_WINDOW,
)

# ---------------------------------------------------------------------------
# Kinds
# ---------------------------------------------------------------------------
FEATURE_DISK = "disk"
"""A raised disk on a hidden post: front material inside the circle."""
FEATURE_CUTOUT = "cutout"
"""A hole in the front plate: back plate inside the circle."""
FEATURE_BLANK = "blank"
"""A blank site: no feature; its diameter is the search-window size of the feature it serves (same level index)."""
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
"""Fraction of the plate width used by a row of features when laying out an array whose caller gives no row width limit
(a roughly square plate)."""
ARRAY_MIN_PLATE_SIZE_MM = 100.0
"""An array plate is never smaller than this (width and height), whatever the ladder."""
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
    """Front-to-back plate distance; None means no back plate (T2, or the open-background variant)."""
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
def make_noise_plate(params: CharacterizationParameters) -> TwoPlaneTarget:
    """T2: the uniform matte plate of NOISE_PLATE_SIZE_MM. It is also the registration target: the plane-only
    registration solve needs no pattern on it (redesign note, Section 3)."""
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
    """First-fit-decreasing row packing of site indices: the sites are taken largest first and each goes into the
    first row that still holds it, so the number of rows is as small as the heuristic finds. Sites of one row are
    ``isolation_mm`` apart edge to edge and a row is at most ``max_row_width_mm`` wide (a site wider than that gets a
    row of its own; the caller's fit check then reports it)."""
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
                       blank_diameters_mm: Sequence[float] = (), post_sites: int = 0,
                       post_diameter_mm: float | None = None, min_plate_mm: float = ARRAY_MIN_PLATE_SIZE_MM,
                       max_row_width_mm: float | None = None, plate_margin_mm: float | None = None) -> TwoPlaneTarget:
    """A disk (TARGET_KIND_DISK_ARRAY) or cutout (TARGET_KIND_CUTOUT_ARRAY) plate carrying the given diameters
    (feature levels 0, 1, ... in the order given), one blank site per entry of ``blank_diameters_mm`` and
    ``post_sites`` post-only control sites (Section 3.2) of diameter ``post_diameter_mm``, which the caller derives
    from the rule of Section 3.2 (:meth:`CharacterizationParameters.post_diameter_mm`); it is required when
    ``post_sites`` is positive, since there is no fixed nominal post diameter.

    Blank site i has level index i and serves feature i of the detection analysis: its diameter is the search window of
    that feature at the far station (see :func:`blank_site_diameters_mm`), so the window of feature i fits on blank
    site i at every station (the analysis cuts the blank's window to the feature's size).

    Sites are packed in rows (:func:`_layout_rows`) with at least ``isolation_mm`` between edges, in both directions: a
    row is at most ``max_row_width_mm`` wide (default: ARRAY_ROW_PACKING_FRACTION of a roughly square plate), which lets
    the caller keep the rows inside the field of view at the nearest station, and the rows are ``isolation_mm`` apart
    (row height = its largest site, so the vertical edge-to-edge distance of any two sites in different rows is at
    least ``isolation_mm`` as well). The plate is the bounding box of the sites plus ``plate_margin_mm`` on every side
    (default ``isolation_mm``), and never smaller than ``min_plate_mm``."""
    if kind not in (TARGET_KIND_DISK_ARRAY, TARGET_KIND_CUTOUT_ARRAY):
        raise ValueError("kind must be TARGET_KIND_DISK_ARRAY or TARGET_KIND_CUTOUT_ARRAY")
    if len(blank_diameters_mm) > len(diameters_mm):
        raise ValueError(f"{len(blank_diameters_mm)} blank sites for {len(diameters_mm)} features: blank site i serves "
                         "feature i, so there cannot be more blank sites than features")
    if post_sites > 0 and post_diameter_mm is None:
        raise ValueError("post_diameter_mm is required when post_sites > 0 (derive it from POST_DIAMETER_FRACTION_OF_D0 "
                         "x the expected D_0, CharacterizationParameters.post_diameter_mm)")
    feature_kind = FEATURE_DISK if kind == TARGET_KIND_DISK_ARRAY else FEATURE_CUTOUT
    margin_mm = isolation_mm if plate_margin_mm is None else plate_margin_mm
    entries: list[tuple[str, str, float, int | None]] = []
    for level, diameter in enumerate(diameters_mm):
        entries.append((f"{feature_kind}_{level:02d}", feature_kind, float(diameter), level))
    for level, diameter in enumerate(blank_diameters_mm):
        entries.append((f"blank_{level:02d}", FEATURE_BLANK, float(diameter), level))
    if kind == TARGET_KIND_DISK_ARRAY:
        for post_index in range(post_sites):
            entries.append((f"post_{post_index:02d}", FEATURE_POST, post_diameter_mm, None))
    sizes = [e[2] for e in entries]
    largest = max(sizes)
    if max_row_width_mm is None:
        # A roughly square plate: row width about the square root of the packed area, at least the largest site.
        total = sum(sizes) + isolation_mm * len(sizes)
        row_width = max(math.sqrt(total * (largest + isolation_mm)), largest + isolation_mm, min_plate_mm)
        max_row_width_mm = row_width * ARRAY_ROW_PACKING_FRACTION
    rows = _layout_rows(sizes, isolation_mm, max_row_width_mm)
    row_heights = [max(sizes[i] for i in row) for row in rows]
    row_widths = [sum(sizes[i] for i in row) + isolation_mm * (len(row) - 1) for row in rows]
    sites_height = sum(row_heights) + isolation_mm * (len(rows) - 1)
    half_width = max(max(row_widths) + 2.0 * margin_mm, min_plate_mm) / 2.0
    half_height = max(sites_height + 2.0 * margin_mm, min_plate_mm) / 2.0
    features: list[Feature] = []
    y = -sites_height / 2.0                       # the block of rows is centered on the plate center
    for row, row_height, row_width in zip(rows, row_heights, row_widths):
        x = -row_width / 2.0                      # and so is each row
        for i in row:
            site_id, fkind, diameter, level = entries[i]
            features.append(Feature(site_id=site_id, kind=fkind, x_mm=x + diameter / 2.0, y_mm=y + row_height / 2.0,
                                    diameter_mm=diameter, level_index=level))
            x += diameter + isolation_mm
        y += row_height + isolation_mm
    return TwoPlaneTarget(target_id, kind, half_width, half_height, gap_mm=gap_mm, features=features)


def blank_site_diameters_mm(params: CharacterizationParameters, geometry: SensorGeometry) -> tuple[float, ...]:
    """The diameters of the blank sites: blank site i is sized to the search window of feature i at the far station.
    The window of a feature has radius D_px / 2 + DETECTION_WINDOW_MARGIN_PX pixels, which is largest in millimeters at
    Z_MAX (the pixel footprint is largest there), so blank site i is D_i + 2 DETECTION_WINDOW_MARGIN_PX p(Z_MAX): about
    21, 34 and 70 mm for the 7.0, 19.7 and 55.8 mm features of the indicative geometry. Only BLANK_SITES_PER_PLATE of
    them are built (the first ones, serving the smallest features)."""
    footprint_mm = geometry.pixel_footprint_mm(params.z_max_mm)
    margin_mm = 2.0 * params.detection_window_margin_px * footprint_mm
    return tuple(d + margin_mm for d in params.feature_diameters_mm(geometry))[:params.blank_sites_per_plate]


def window_diameter_mm(target: TwoPlaneTarget, site: Feature) -> float:
    """The diameter whose search window (radius D_px / 2 + DETECTION_WINDOW_MARGIN_PX) applies to a site: the site's
    own diameter, except for a blank site, which is as large as the window of the feature it serves at Z_MAX and is cut
    to that feature's size (the feature of the same level index) so that the same D_px gives the same window."""
    if site.kind == FEATURE_BLANK and site.level_index is not None:
        for f in target.features:
            if f.kind in (FEATURE_DISK, FEATURE_CUTOUT) and f.level_index == site.level_index:
                return f.diameter_mm
    return site.diameter_mm


def check_array_fits_field(params: CharacterizationParameters, geometry: SensorGeometry, target: TwoPlaneTarget) -> None:
    """Raise ValueError when an array plate does not fit the field of view at Z_MIN with room on every side for the
    phase-jitter span (PHASE_JITTER_SPAN_PX) and the boundary band (BOUNDARY_BAND_HALF_WIDTH_PX on each side): the
    plate must be at most ``field - jitter span - 2 band`` wide and high, all in millimeters at Z_MIN."""
    footprint_mm = geometry.pixel_footprint_mm(params.z_min_mm)
    field_mm = tuple(2.0 * h for h in geometry.half_field_mm(params.z_min_mm))
    reserve_mm = (params.phase_jitter_span_px + 2.0 * params.boundary_band_half_width_px) * footprint_mm
    usable_mm = (field_mm[0] - reserve_mm, field_mm[1] - reserve_mm)
    plate_mm = (2.0 * target.half_width_mm, 2.0 * target.half_height_mm)
    if plate_mm[0] > usable_mm[0] + 1.0e-9 or plate_mm[1] > usable_mm[1] + 1.0e-9:
        raise ValueError(
            f"plate {target.target_id} is {plate_mm[0]:.0f} x {plate_mm[1]:.0f} mm but only {usable_mm[0]:.0f} x "
            f"{usable_mm[1]:.0f} mm fit the field of view at Z_MIN = {params.z_min_mm:g} mm "
            f"({field_mm[0]:.0f} x {field_mm[1]:.0f} mm, minus the phase-jitter span and twice the boundary band, "
            f"{reserve_mm:.0f} mm in all); reduce FEATURE_COUNT, FEATURE_LADDER_RATIO, FEATURE_ISOLATION_PX or "
            "DETECTION_WINDOW_MARGIN_PX, or raise Z_MIN_MM")


def make_standard_target_set(params: CharacterizationParameters, geometry: SensorGeometry,
                             gap_mm: float | None = None) -> TargetSet:
    """T2, T3a, T3b, T4 and T5 from the parameters (Section 3.2 and the redesign note, Section 2).

    T4 (disks) and T5 (cutouts) each carry FEATURE_COUNT features of diameters
    ``params.feature_diameters_mm(geometry)``, BLANK_SITES_PER_PLATE blank sites (site i sized to the search window of
    feature i at Z_MAX, :func:`blank_site_diameters_mm`) and, for the disk plate, POST_SITES_PER_PLATE post-only sites (diameter from the rule
    POST_DIAMETER_FRACTION_OF_D0 x EXPECTED_D0_PX x p(Z_MIN), :meth:`CharacterizationParameters.post_diameter_mm`).
    The isolation between neighboring sites is FEATURE_ISOLATION_PX at Z_MAX (edge to edge, in both directions), so
    neighbors stay separated at the far station. The sites are packed into as few rows as fit the usable width at
    Z_MIN (the field width minus the phase-jitter span and twice the boundary band), and the plate carries
    FEATURE_PLATE_MARGIN_PX at Z_MAX of front plate beyond the outermost sites. A plate that still does not fit the
    field at Z_MIN raises ValueError (:func:`check_array_fits_field`). The gap defaults to GAP_SMALL_MM (the manifest's
    gap overrides it per pose)."""
    gap = params.gap_small_mm if gap_mm is None else gap_mm
    diameters = list(params.feature_diameters_mm(geometry))
    p_far = geometry.pixel_footprint_mm(params.z_max_mm)
    isolation_mm = params.feature_isolation_px * p_far
    plate_margin_mm = params.feature_plate_margin_px * p_far
    blanks = blank_site_diameters_mm(params, geometry)
    # Widest row: the field width at Z_MIN less the jitter span and a boundary band on each side (all in mm at Z_MIN),
    # less the plate margin on each side so that the plate itself fits.
    p_near = geometry.pixel_footprint_mm(params.z_min_mm)
    field_width_mm = 2.0 * geometry.half_field_mm(params.z_min_mm)[0]
    usable_width_mm = field_width_mm - (params.phase_jitter_span_px + 2.0 * params.boundary_band_half_width_px) * p_near
    max_row_mm = usable_width_mm - 2.0 * plate_margin_mm
    targets = TargetSet()
    targets.add(make_noise_plate(params))
    targets.add(make_edge_target(params, TARGET_KIND_RAISED_SQUARE, gap))
    targets.add(make_edge_target(params, TARGET_KIND_SQUARE_WINDOW, gap))
    targets.add(make_feature_array(TARGET_DISKS, TARGET_KIND_DISK_ARRAY, diameters, isolation_mm, gap, blanks,
                                   params.post_sites_per_plate, post_diameter_mm=params.post_diameter_mm(geometry),
                                   max_row_width_mm=max_row_mm, plate_margin_mm=plate_margin_mm))
    targets.add(make_feature_array(TARGET_CUTOUTS, TARGET_KIND_CUTOUT_ARRAY, diameters, isolation_mm, gap, blanks,
                                   max_row_width_mm=max_row_mm, plate_margin_mm=plate_margin_mm))
    for target_id in (TARGET_DISKS, TARGET_CUTOUTS):
        check_array_fits_field(params, geometry, targets.get(target_id))
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
    camera_2d_center_mm: tuple[float, float, float] | None = None
    """Center of the 2-D image sensor and its collocated illumination (CAMERA_2D_OFFSET_MM), or None when the sensor
    configuration does not give it."""

    @classmethod
    def from_sensor_geometry(cls, geometry: SensorGeometry) -> "StereoGeometry":
        """Left camera from the intrinsics; right camera at (+B, 0, 0); projector at
        PROJECTOR_OFFSET_MM; the 2-D sensor at CAMERA_2D_OFFSET_MM when the configuration has it (it is optional).
        Raises MissingSensorValue for the unfilled † values the depth sense needs."""
        camera = PinholeCamera(int(geometry.require("image_width_px")), int(geometry.require("image_height_px")),
                               float(geometry.require("sensor_fx_px")), float(geometry.require("sensor_fy_px")),
                               float(geometry.require("sensor_cx_px")), float(geometry.require("sensor_cy_px")))
        baseline = float(geometry.require("sensor_baseline_mm"))
        projector = tuple(float(v) for v in geometry.require("projector_offset_mm"))
        offset_2d = geometry.camera_2d_offset_mm
        camera_2d = None if offset_2d is None else tuple(float(v) for v in offset_2d)
        return cls(camera, (baseline, 0.0, 0.0), projector, camera_2d)  # type: ignore[arg-type]

    def viewpoints(self, require_projector: bool) -> list[np.ndarray]:
        """Left camera, right camera and (optionally) projector centers, camera frame."""
        points = [np.zeros(3), np.asarray(self.right_center_mm, dtype=np.float64)]
        if require_projector:
            points.append(np.asarray(self.projector_center_mm, dtype=np.float64))
        return points

    def viewpoints_2d(self) -> list[np.ndarray]:
        """The single viewpoint of the 2-D image (Section 12, Step 7): the 2-D sensor, whose illumination is collocated,
        so that a point is seen and lit along the same ray. Empty when the 2-D position is not known."""
        if self.camera_2d_center_mm is None:
            return []
        return [np.asarray(self.camera_2d_center_mm, dtype=np.float64)]


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
