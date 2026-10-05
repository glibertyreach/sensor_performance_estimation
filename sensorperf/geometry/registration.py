"""
Robot-to-sensor registration (document, Section 4): the two transforms that
turn a read-back robot pose into a ground-truth target pose in the camera
frame, their file (registration.json), and the solves that produce them.

Frames
    base      the robot base frame (the robot reports flange poses in it)
    flange    the robot flange (tool) frame
    target    the target frame of sensorperf.geometry.targets (front face,
              z away from the sensor)
    camera    the left IR camera frame

    camera_to_base   T_bc: points in the camera frame -> base frame
    target_to_flange T_ft: points in the target frame -> flange frame (the
                     target's position on the dowel-pinned adapter; the same
                     for every target mounted on that adapter, up to the
                     adapter-to-target offsets recorded per target)

Composition (the chain every analysis relies on)
    target_to_camera = camera_to_base^-1 . flange_to_base . target_to_flange

and its inverse for planning: given the wanted target_to_camera,
    flange_to_base = camera_to_base . target_to_camera . target_to_flange^-1

The hand-eye problem (Step 4.7): with observations i of the flange pose
A_i = flange_to_base_i (robot read-back) and the target pose B_i =
target_to_camera_i (fiducial solve or depth-plane fit), the unknowns X =
target_to_flange and Y = camera_to_base satisfy A_i X = Y B_i. This is the
classic AX = YB form with two unknown transforms. :func:`solve_hand_eye`
solves it from a closed-form start followed by nonlinear least squares, and
reports the RMS pose residual used by the acceptance gate
(REGISTRATION_RESIDUAL_ACCEPT_MM).

When only depth-plane fits are available (Step 4.6, second bullet), each
observation gives the target's front plane in the camera frame (unit normal
toward the camera, signed distance), not a full pose. :func:`solve_from_planes`
fits the same unknowns to plane observations; the in-plane position and the
rotation about the normal of X are unobservable and are fixed by convention,
and a constant depth offset is absorbed into Y (Section 15, "Limitations").
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from sensorperf.geometry.transforms import RigidTransform

REGISTRATION_FILE_NAME = "registration.json"
"""Name of the registration record in a session folder (Section 9)."""

METHOD_FIDUCIAL = "fiducial_hand_eye"
"""Registration from full target poses (IR fiducial solve)."""
METHOD_DEPTH_PLANES = "depth_planes"
"""Registration from depth-plane fits only (constant depth offset not observable)."""

ROTATION_RESIDUAL_LEVER_MM = 100.0
"""Length that converts a rotation residual (radians) into millimeters in the least-squares
cost, so that rotation and translation residuals are weighed on one scale: a rotation error
of e radians moves a point this far from the target's reference point by e times this length."""
MIN_OBSERVATIONS_FULL_POSE = 3
"""Three non-degenerate pose pairs determine the hand-eye problem."""
MIN_OBSERVATIONS_PLANES = 6
"""Plane observations give two rotation constraints and one translation constraint each;
with tilts about two axes, six are a practical minimum."""
LEAST_SQUARES_TOLERANCE = 1.0e-12
"""Convergence tolerances of the nonlinear refinement."""
LEAST_SQUARES_MAX_EVALUATIONS = 2000
"""Evaluation budget of the nonlinear refinement."""


@dataclass
class Registration:
    """registration.json: the two solved transforms and how good they are."""

    camera_to_base: RigidTransform
    target_to_flange: RigidTransform
    residual_rms_mm: float = float("nan")
    """RMS distance between solved and observed target poses (Step 4.7)."""
    rotation_residual_rms_deg: float = float("nan")
    pose_count: int = 0
    method: str = METHOD_FIDUCIAL
    accepted: bool | None = None
    """Whether residual_rms_mm <= REGISTRATION_RESIDUAL_ACCEPT_MM at the time of solving."""
    notes: dict[str, Any] = field(default_factory=dict)

    # -- the composition chain -------------------------------------------------
    def target_to_camera(self, flange_to_base: RigidTransform,
                         target_to_flange: RigidTransform | None = None) -> RigidTransform:
        """Ground-truth target pose in the camera frame for a read-back flange pose."""
        offset = self.target_to_flange if target_to_flange is None else target_to_flange
        return self.camera_to_base.inverse().compose(flange_to_base).compose(offset)

    def flange_to_base_for(self, target_to_camera: RigidTransform,
                           target_to_flange: RigidTransform | None = None) -> RigidTransform:
        """The flange pose to command so that the target lands at target_to_camera (Step 4.8)."""
        offset = self.target_to_flange if target_to_flange is None else target_to_flange
        return self.camera_to_base.compose(target_to_camera).compose(offset.inverse())

    # -- file -------------------------------------------------------------------
    def save(self, path: str | Path) -> Path:
        document = {
            "camera_to_base": self.camera_to_base.as_matrix().reshape(-1).tolist(),
            "target_to_flange": self.target_to_flange.as_matrix().reshape(-1).tolist(),
            "residual_rms_mm": self.residual_rms_mm,
            "rotation_residual_rms_deg": self.rotation_residual_rms_deg,
            "pose_count": self.pose_count,
            "method": self.method,
            "accepted": self.accepted,
            "notes": self.notes,
            "matrix_convention": "row-major 4x4, points in the first frame -> points in the second frame, mm",
        }
        Path(path).write_text(json.dumps(document, indent=2), encoding="utf-8")
        return Path(path)

    @classmethod
    def load(cls, path: str | Path) -> "Registration":
        document = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(
            camera_to_base=RigidTransform.from_matrix(np.asarray(document["camera_to_base"]).reshape(4, 4)),
            target_to_flange=RigidTransform.from_matrix(np.asarray(document["target_to_flange"]).reshape(4, 4)),
            residual_rms_mm=float(document.get("residual_rms_mm", float("nan"))),
            rotation_residual_rms_deg=float(document.get("rotation_residual_rms_deg", float("nan"))),
            pose_count=int(document.get("pose_count", 0)),
            method=str(document.get("method", METHOD_FIDUCIAL)),
            accepted=document.get("accepted"),
            notes=dict(document.get("notes") or {}),
        )


@dataclass
class PlaneObservation:
    """A target front plane observed in the camera frame: unit normal toward the
    camera and the signed distance such that normal . p + distance = 0 for points p
    on the plane (so distance = -normal . point)."""

    normal: np.ndarray
    distance_mm: float


# ---------------------------------------------------------------------------
# Parametrization shared by the two solves
# ---------------------------------------------------------------------------
def _pack(x: RigidTransform, y: RigidTransform) -> np.ndarray:
    """12 parameters: rotation vectors (radians) and translations of X and Y."""
    return np.concatenate([Rotation.from_matrix(x.rotation).as_rotvec(), x.translation,
                           Rotation.from_matrix(y.rotation).as_rotvec(), y.translation])


def _unpack(p: np.ndarray) -> tuple[RigidTransform, RigidTransform]:
    x = RigidTransform(Rotation.from_rotvec(p[0:3]).as_matrix(), p[3:6])
    y = RigidTransform(Rotation.from_rotvec(p[6:9]).as_matrix(), p[9:12])
    return x, y


def _pose_residuals(x: RigidTransform, y: RigidTransform, flange_to_base: Sequence[RigidTransform],
                    target_to_camera: Sequence[RigidTransform]) -> tuple[np.ndarray, np.ndarray]:
    """Per observation: translation residual vector (mm) and rotation residual vector
    (radians) of A_i X against Y B_i."""
    translations, rotations = [], []
    for a, b in zip(flange_to_base, target_to_camera):
        left = a.compose(x)
        right = y.compose(b)
        translations.append(left.translation - right.translation)
        relative = left.rotation @ right.rotation.T
        rotations.append(Rotation.from_matrix(relative).as_rotvec())
    return np.asarray(translations), np.asarray(rotations)


# ---------------------------------------------------------------------------
# Closed-form start for AX = YB (rotation by the AX = XB reduction, translation linear)
# ---------------------------------------------------------------------------
def _rotation_x_closed_form(flange_to_base: Sequence[RigidTransform],
                            target_to_camera: Sequence[RigidTransform]) -> np.ndarray:
    """R_X from pairs of observations: A_i^-1 A_j R_X = R_X B_i^-1 B_j, solved by
    aligning the rotation axes of the relative motions (Park and Martin, 1994):
    alpha_ij = log(R_Ai^T R_Aj), beta_ij = log(R_Bi^T R_Bj), R_X maps beta to alpha."""
    alphas, betas = [], []
    count = len(flange_to_base)
    for i in range(count):
        for j in range(i + 1, count):
            ra = flange_to_base[i].rotation.T @ flange_to_base[j].rotation
            rb = target_to_camera[i].rotation.T @ target_to_camera[j].rotation
            alphas.append(Rotation.from_matrix(ra).as_rotvec())
            betas.append(Rotation.from_matrix(rb).as_rotvec())
    alpha = np.asarray(alphas)
    beta = np.asarray(betas)
    # Kabsch: the rotation taking beta vectors onto alpha vectors.
    covariance = alpha.T @ beta
    u, _, vt = np.linalg.svd(covariance)
    sign_fix = np.diag([1.0, 1.0, np.sign(np.linalg.det(u @ vt))])
    return u @ sign_fix @ vt


def _closed_form_start(flange_to_base: Sequence[RigidTransform],
                       target_to_camera: Sequence[RigidTransform]) -> tuple[RigidTransform, RigidTransform]:
    """Closed-form X and Y: R_X from the relative-motion alignment, R_Y as the
    chordal mean of R_Ai R_X R_Bi^T, then the translations by linear least squares
    from R_Ai t_X + t_Ai = R_Y t_Bi + t_Y."""
    rotation_x = _rotation_x_closed_form(flange_to_base, target_to_camera)
    stacked = sum(a.rotation @ rotation_x @ b.rotation.T for a, b in zip(flange_to_base, target_to_camera))
    u, _, vt = np.linalg.svd(stacked)
    rotation_y = u @ np.diag([1.0, 1.0, np.sign(np.linalg.det(u @ vt))]) @ vt
    rows, rhs = [], []
    for a, b in zip(flange_to_base, target_to_camera):
        rows.append(np.hstack([a.rotation, -np.eye(3)]))
        rhs.append(rotation_y @ b.translation - a.translation)
    solution, *_ = np.linalg.lstsq(np.vstack(rows), np.concatenate(rhs), rcond=None)
    return RigidTransform(rotation_x, solution[0:3]), RigidTransform(rotation_y, solution[3:6])


# ---------------------------------------------------------------------------
# Solves
# ---------------------------------------------------------------------------
def solve_hand_eye(flange_to_base: Sequence[RigidTransform], target_to_camera: Sequence[RigidTransform],
                   accept_rms_mm: float | None = None) -> Registration:
    """Solve A_i X = Y B_i for X = target_to_flange and Y = camera_to_base from
    full pose observations (Step 4.7): closed-form start, then joint nonlinear
    least squares on the translation residual (mm) and the rotation residual
    (radians times ROTATION_RESIDUAL_LEVER_MM). The residual RMS reported is the
    RMS over observations of |t(A_i X) - t(Y B_i)|, i.e. the distance between
    the solved and the observed target reference point."""
    if len(flange_to_base) != len(target_to_camera):
        raise ValueError("flange_to_base and target_to_camera must have the same length")
    if len(flange_to_base) < MIN_OBSERVATIONS_FULL_POSE:
        raise ValueError(f"the hand-eye solve needs at least {MIN_OBSERVATIONS_FULL_POSE} observations")
    x0, y0 = _closed_form_start(flange_to_base, target_to_camera)

    def cost(p: np.ndarray) -> np.ndarray:
        x, y = _unpack(p)
        t_res, r_res = _pose_residuals(x, y, flange_to_base, target_to_camera)
        return np.concatenate([t_res.reshape(-1), ROTATION_RESIDUAL_LEVER_MM * r_res.reshape(-1)])

    result = least_squares(cost, _pack(x0, y0), xtol=LEAST_SQUARES_TOLERANCE, ftol=LEAST_SQUARES_TOLERANCE,
                           gtol=LEAST_SQUARES_TOLERANCE, max_nfev=LEAST_SQUARES_MAX_EVALUATIONS)
    x, y = _unpack(result.x)
    t_res, r_res = _pose_residuals(x, y, flange_to_base, target_to_camera)
    rms_mm = float(np.sqrt(np.mean(np.sum(t_res ** 2, axis=1))))
    rms_deg = float(np.degrees(np.sqrt(np.mean(np.sum(r_res ** 2, axis=1)))))
    return Registration(camera_to_base=y, target_to_flange=x, residual_rms_mm=rms_mm,
                        rotation_residual_rms_deg=rms_deg, pose_count=len(flange_to_base), method=METHOD_FIDUCIAL,
                        accepted=None if accept_rms_mm is None else bool(rms_mm <= accept_rms_mm))


def plane_of_pose(target_to_camera: RigidTransform) -> PlaneObservation:
    """The target's front plane (z = 0 of the target frame) in the camera frame,
    normal toward the camera (minus the target z axis)."""
    normal = -target_to_camera.rotation[:, 2]
    return PlaneObservation(normal=normal, distance_mm=float(-normal @ target_to_camera.translation))


def solve_from_planes(flange_to_base: Sequence[RigidTransform], planes: Sequence[PlaneObservation],
                      initial: tuple[RigidTransform, RigidTransform] | None = None,
                      accept_rms_mm: float | None = None) -> Registration:
    """Registration from depth-plane observations only (Step 4.6, second bullet).

    Each observation constrains the predicted front plane of Y^-1 A_i X (normal
    and distance) to the observed one; the residual per observation is the
    normal difference (times ROTATION_RESIDUAL_LEVER_MM) and the distance
    difference (mm). The unobservable parts of X (in-plane translation and
    rotation about the target normal) are held at zero: X is parametrized by a
    translation along the flange z axis and two tilt angles. The reported RMS is
    the RMS plane-distance residual. ``initial`` gives (X, Y) start values; by
    default X is the identity and Y is estimated by aligning the mean normals."""
    if len(flange_to_base) != len(planes):
        raise ValueError("flange_to_base and planes must have the same length")
    if len(flange_to_base) < MIN_OBSERVATIONS_PLANES:
        raise ValueError(f"the plane-only solve needs at least {MIN_OBSERVATIONS_PLANES} observations")
    if initial is None:
        x_init = RigidTransform.identity()
        # Y start: the rotation taking the observed normals (camera frame) onto the flange z axes in base.
        flange_z_base = np.asarray([-a.rotation[:, 2] for a in flange_to_base])  # target normal toward camera = -z
        observed = np.asarray([p.normal for p in planes])
        covariance = flange_z_base.T @ observed
        u, _, vt = np.linalg.svd(covariance)
        rotation_y = u @ np.diag([1.0, 1.0, np.sign(np.linalg.det(u @ vt))]) @ vt
        centroid_base = np.mean([a.translation for a in flange_to_base], axis=0)
        # Place the camera so that the mean plane distance is reproduced along the mean normal.
        mean_distance = float(np.mean([p.distance_mm for p in planes]))
        mean_normal_base = rotation_y @ np.mean(observed, axis=0)
        translation_y = centroid_base + mean_normal_base * mean_distance
        y_init = RigidTransform(rotation_y, translation_y)
    else:
        x_init, y_init = initial

    def unpack_reduced(p: np.ndarray) -> tuple[RigidTransform, RigidTransform]:
        x = RigidTransform(Rotation.from_rotvec([p[0], p[1], 0.0]).as_matrix(), np.array([0.0, 0.0, p[2]]))
        y = RigidTransform(Rotation.from_rotvec(p[3:6]).as_matrix(), p[6:9])
        return x, y

    def cost(p: np.ndarray) -> np.ndarray:
        x, y = unpack_reduced(p)
        residuals = []
        y_inverse = y.inverse()
        for a, observed_plane in zip(flange_to_base, planes):
            predicted = plane_of_pose(y_inverse.compose(a).compose(x))
            residuals.append(ROTATION_RESIDUAL_LEVER_MM * (predicted.normal - observed_plane.normal))
            residuals.append([predicted.distance_mm - observed_plane.distance_mm])
        return np.concatenate([np.ravel(r) for r in residuals])

    x_rotvec = Rotation.from_matrix(x_init.rotation).as_rotvec()
    p0 = np.concatenate([x_rotvec[:2], [x_init.translation[2]], Rotation.from_matrix(y_init.rotation).as_rotvec(),
                         y_init.translation])
    result = least_squares(cost, p0, xtol=LEAST_SQUARES_TOLERANCE, ftol=LEAST_SQUARES_TOLERANCE,
                           gtol=LEAST_SQUARES_TOLERANCE, max_nfev=LEAST_SQUARES_MAX_EVALUATIONS)
    x, y = unpack_reduced(result.x)
    y_inverse = y.inverse()
    distance_residuals, angle_residuals = [], []
    for a, observed_plane in zip(flange_to_base, planes):
        predicted = plane_of_pose(y_inverse.compose(a).compose(x))
        distance_residuals.append(predicted.distance_mm - observed_plane.distance_mm)
        angle_residuals.append(np.arccos(np.clip(predicted.normal @ observed_plane.normal, -1.0, 1.0)))
    rms_mm = float(np.sqrt(np.mean(np.square(distance_residuals))))
    rms_deg = float(np.degrees(np.sqrt(np.mean(np.square(angle_residuals)))))
    return Registration(camera_to_base=y, target_to_flange=x, residual_rms_mm=rms_mm,
                        rotation_residual_rms_deg=rms_deg, pose_count=len(flange_to_base),
                        method=METHOD_DEPTH_PLANES,
                        accepted=None if accept_rms_mm is None else bool(rms_mm <= accept_rms_mm),
                        notes={"limitation": "a constant depth offset is absorbed into camera_to_base (Section 15)"})
