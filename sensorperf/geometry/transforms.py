# Reused from the depth_calibration_from_spherical_target repository (sphcal/geometry/transforms.py), import path
# adjusted to this package. Keep in step with that repository; fix upstream and re-copy.
"""
Rigid transforms and the rigid fit between two point sets.

Conventions: a RigidTransform T = (R, t) acts on a point x as R @ x + t and on a
direction n as R @ n. compose(a, b) applies b first, then a. Rotations are 3 x 3
matrices; translations are in millimeters. These match the sixdof repository.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class RigidTransform:
    rotation: np.ndarray = field(default_factory=lambda: np.eye(3))
    translation: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self) -> None:
        self.rotation = np.asarray(self.rotation, dtype=np.float64).reshape(3, 3)
        self.translation = np.asarray(self.translation, dtype=np.float64).reshape(3)

    @classmethod
    def identity(cls) -> "RigidTransform":
        return cls(np.eye(3), np.zeros(3))

    @classmethod
    def from_matrix(cls, matrix_4x4) -> "RigidTransform":
        m = np.asarray(matrix_4x4, dtype=np.float64).reshape(4, 4)
        return cls(m[:3, :3], m[:3, 3])

    @classmethod
    def from_rotation_vector_degrees(cls, rotation_vector_degrees, translation) -> "RigidTransform":
        """Axis-angle vector whose length is the angle in degrees, plus a translation."""
        rv = np.asarray(rotation_vector_degrees, dtype=np.float64)
        return cls(Rotation.from_rotvec(np.radians(rv)).as_matrix(), translation)

    def as_matrix(self) -> np.ndarray:
        m = np.eye(4)
        m[:3, :3] = self.rotation
        m[:3, 3] = self.translation
        return m

    def rotation_vector_degrees(self) -> np.ndarray:
        return np.degrees(Rotation.from_matrix(self.rotation).as_rotvec())

    def compose(self, other: "RigidTransform") -> "RigidTransform":
        """self after other: (self o other)(x) = self(other(x))."""
        return RigidTransform(self.rotation @ other.rotation, self.rotation @ other.translation + self.translation)

    def inverse(self) -> "RigidTransform":
        rt = self.rotation.T
        return RigidTransform(rt, -rt @ self.translation)

    def apply_points(self, points) -> np.ndarray:
        points = np.asarray(points, dtype=np.float64)
        return points @ self.rotation.T + self.translation

    def apply_directions(self, directions) -> np.ndarray:
        directions = np.asarray(directions, dtype=np.float64)
        return directions @ self.rotation.T

    def difference_from(self, other: "RigidTransform") -> tuple[float, float]:
        """(translation error in mm, rotation error in degrees) between two transforms."""
        relative = self.rotation @ other.rotation.T
        rotation_error = float(np.degrees(np.linalg.norm(Rotation.from_matrix(relative).as_rotvec())))
        return float(np.linalg.norm(self.translation - other.translation)), rotation_error


def fit_rigid_transform(source_points, target_points, weights=None) -> RigidTransform:
    """
    The rigid transform T minimizing sum_i w_i |T(source_i) - target_i|^2 (the
    Kabsch / Umeyama solution without scale). Needs at least three
    non-collinear points. Used to solve the sensor-to-positioner transform from
    fitted sphere centers (sensor frame) and commanded centers (positioner frame).
    """
    source = np.asarray(source_points, dtype=np.float64).reshape(-1, 3)
    target = np.asarray(target_points, dtype=np.float64).reshape(-1, 3)
    if source.shape[0] < 3:
        raise ValueError("fit_rigid_transform needs at least three point pairs")
    w = np.ones(source.shape[0]) if weights is None else np.asarray(weights, dtype=np.float64)
    w = w / w.sum()
    source_mean = (w[:, None] * source).sum(axis=0)
    target_mean = (w[:, None] * target).sum(axis=0)
    cross_covariance = ((target - target_mean) * w[:, None]).T @ (source - source_mean)
    u, _, vt = np.linalg.svd(cross_covariance)
    # Reflection guard: force a proper rotation (determinant +1).
    sign_fix = np.diag([1.0, 1.0, np.sign(np.linalg.det(u @ vt))])
    rotation = u @ sign_fix @ vt
    translation = target_mean - rotation @ source_mean
    return RigidTransform(rotation, translation)
