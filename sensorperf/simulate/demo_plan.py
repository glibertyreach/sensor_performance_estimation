"""
A small but complete demonstration plan for the synthetic session writer (Sections 4 to 9 of the
procedure, drastically reduced), a plausible registration to render it through, and the reduced
sensor geometry of the ``--quick`` option.

Why a local plan. ``acquisition/plan.py`` owns the real planning functions (full station lists,
randomized order, sentinel clock). This module is deliberately independent of them so that the
simulator CLI works on its own: it builds, for every series, only as many stations and poses as are
needed to exercise every analysis once. It follows the same rules the real planner states:

- lateral phase-jitter offsets are uniform over +/- PHASE_JITTER_SPAN_PX / 2 converted to mm at the
  station depth with p(Z) = Z / f_x (Section 6.1, Step 3; Section 7, Step 2),
- targets are fronto-parallel except the tilt poses of the A sub-series (Section 5, Step 5) and the
  registration poses (Section 4, Step 6),
- pose indices are unique within (procedure, target, gap, station, field),
- every capture carries the seed of the draw that produced its offsets.

The demonstration plan covers
    R  registration: T2 (the noise plate; plane-only solve) at several tilted poses spread over the volume
    A  noise: T2 at three stations (center field), two tilt poses, and a T2 drift sentinel before and after
       (the plan ends with a sentinel on the target mounted there)
    B  edges: T3a and T3b at two shape stations (566 and 800 mm), one nominal pose and four jitter poses each, small gap
    Z  Z-step: T2 at one station, a ladder of three step sizes (rungs of the station's expected quantum) with two ABAB
       cycles each, a ramp (T2 tilted about H) at each of the three A stations, and the optional staircase
    C  area: T4 (disks) and T5 (cutouts) at the five shape stations (400, 566, 800, 1131, 1600 mm), a few
       jitter poses each, small gap; the stations make the three features of a plate overlap in D_px
    D  detection: T4 and T5 at the same five stations, single-frame jitter poses per station

Units: millimeters, degrees, pixels. Poses are target -> camera (see sensorperf.geometry.targets).
"""
from __future__ import annotations

from dataclasses import replace
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from sensorperf.acquisition.plan import (
    PlannedCapture, ramp_pose, ramp_tilt_deg, staircase_step_mm, z_step_rungs_mm,
)
from sensorperf.geometry.registration import Registration
from sensorperf.geometry.targets import fronto_parallel_pose, make_noise_plate, tilted_pose
from sensorperf.geometry.transforms import RigidTransform
from sensorperf.io.manifest import (
    SUBSERIES_JITTER, SUBSERIES_LADDER, SUBSERIES_MAIN, SUBSERIES_NOMINAL, SUBSERIES_RAMP, SUBSERIES_SENTINEL,
    SUBSERIES_STAIRCASE, SUBSERIES_TILT, TILT_AXIS_H, TILT_AXIS_V, VISIT_A, VISIT_B,
)
from sensorperf.parameters import (
    CharacterizationParameters, FIELD_POSITION_CENTER, PROCEDURE_AREA, PROCEDURE_DETECTION, PROCEDURE_EDGES,
    PROCEDURE_NOISE, PROCEDURE_REGISTRATION, PROCEDURE_SENTINEL, PROCEDURE_ZSTEP, SensorGeometry,
    TARGET_CUTOUTS, TARGET_DISKS, TARGET_NOISE_PLATE, TARGET_RAISED_SQUARE, TARGET_SQUARE_WINDOW,
)
from sensorperf.simulate.sensor_model import INDICATIVE_DISPARITY_QUANTUM_PX

# ---------------------------------------------------------------------------
# Registration used by the demonstration (plausible, not measured)
# ---------------------------------------------------------------------------
DEMO_CAMERA_TO_BASE_ROTVEC_DEG = (172.0, 8.0, -15.0)
"""Rotation vector (degrees) of the demonstration camera -> robot-base transform: the camera looks
back toward the robot, roughly opposite to the base z axis."""
DEMO_CAMERA_TO_BASE_TRANSLATION_MM = (1200.0, -350.0, 900.0)
"""Translation (mm) of the demonstration camera -> robot-base transform."""
DEMO_TARGET_TO_FLANGE_ROTVEC_DEG = (0.6, -0.4, 1.1)
"""Rotation vector (degrees) of the demonstration target -> flange transform (a slightly
imperfect adapter mounting)."""
DEMO_TARGET_TO_FLANGE_TRANSLATION_MM = (1.5, -2.0, 95.0)
"""Translation (mm) of the demonstration target -> flange transform: the target reference
point is about 95 mm from the flange along the tool axis."""

# ---------------------------------------------------------------------------
# Size of the demonstration plan
# ---------------------------------------------------------------------------
DEMO_REGISTRATION_POSES = 12
"""Registration poses of the full demonstration plan (the real procedure uses 30)."""
DEMO_REGISTRATION_POSES_QUICK = 6
"""Registration poses in quick mode."""
DEMO_REGISTRATION_LATERAL_FRACTION = 0.15
"""Registration poses are spread laterally over this fraction of the half field at their depth."""
DEMO_REGISTRATION_DEPTH_SPAN_FRACTION = 0.3
"""Registration depths are Z_REFERENCE plus or minus this fraction of the working range."""
DEMO_TILT_POSES_A = ((TILT_AXIS_H, 15.0), (TILT_AXIS_V, 30.0))
"""(axis, degrees) of the two A tilt poses."""
DEMO_EDGE_JITTER_POSES = 4
"""Random-offset poses per B edge station (after the nominal pose)."""
DEMO_AREA_JITTER_POSES = 3
"""Random-offset poses per C configuration (a plate, a station)."""
DEMO_DETECTION_POSES = 24
"""Single-frame random-offset poses per D configuration (a plate, a station)."""
DEMO_ZSTEP_LADDER_INDICES = (1, 3, 5)
"""Indices into the station's ladder rungs (``plan.z_step_rungs_mm``: 0.25, 0.5, 1, 2, 4 and 8 expected quanta) of the
three demonstration step sizes: half, two and eight quanta (0.78, 3.1 and 12.4 mm at 800 mm)."""
DEMO_ZSTEP_CYCLES = 2
"""ABAB cycles per step size."""
DEMO_STAIRCASE_STEPS = 30
"""Fine-staircase steps of the demonstration (the optional second pass of Section 6.2): the full Z_STAIRCASE_QUANTA x
Z_STAIRCASE_SUBDIVISION sweep (3 quanta at 10 steps each), so the staircase analysis of Analysis B-Z has data and its
quantum can be set beside the ramp's."""
SEED_UPPER_BOUND = 2 ** 31 - 1
"""Exclusive upper bound of the per-capture seeds drawn for the plan (fits a signed 32-bit integer)."""

ALL_SERIES = (PROCEDURE_REGISTRATION, PROCEDURE_NOISE, PROCEDURE_EDGES, PROCEDURE_ZSTEP, PROCEDURE_AREA,
              PROCEDURE_DETECTION)
"""Series letters a demonstration plan can cover, in acquisition order."""


# ---------------------------------------------------------------------------
# Geometry and registration helpers
# ---------------------------------------------------------------------------
def scaled_geometry(geometry: SensorGeometry, divisor: int) -> SensorGeometry:
    """The geometry with the image size and the pixel intrinsics (fx, fy, cx, cy) divided by
    ``divisor``, so that the field of view is unchanged and each pixel is ``divisor`` times larger
    (``--quick`` uses divisor 4: 640 x 480 becomes 160 x 120). Baseline, projector, depth LSB and frame
    rate are unchanged."""
    if divisor < 1:
        raise ValueError("divisor must be at least 1")
    return replace(
        geometry,
        sensor_fx_px=geometry.require("sensor_fx_px") / divisor,
        sensor_fy_px=geometry.require("sensor_fy_px") / divisor,
        sensor_cx_px=geometry.require("sensor_cx_px") / divisor,
        sensor_cy_px=geometry.require("sensor_cy_px") / divisor,
        image_width_px=int(geometry.require("image_width_px")) // divisor,
        image_height_px=int(geometry.require("image_height_px")) // divisor)


def demo_registration() -> Registration:
    """The demonstration registration: a plausible camera -> base and target -> flange (module
    constants above), marked as simulated in its notes."""
    return Registration(
        camera_to_base=RigidTransform.from_rotation_vector_degrees(DEMO_CAMERA_TO_BASE_ROTVEC_DEG,
                                                                   DEMO_CAMERA_TO_BASE_TRANSLATION_MM),
        target_to_flange=RigidTransform.from_rotation_vector_degrees(DEMO_TARGET_TO_FLANGE_ROTVEC_DEG,
                                                                     DEMO_TARGET_TO_FLANGE_TRANSLATION_MM),
        residual_rms_mm=0.0, rotation_residual_rms_deg=0.0, accepted=True,
        notes={"origin": "simulated by sensorperf.simulate.demo_plan; plausible values, not measured"})


# ---------------------------------------------------------------------------
# The plan
# ---------------------------------------------------------------------------
class _PlanBuilder:
    """Accumulates captures, hands out pose indices that are unique within a configuration and
    the acquisition order, and draws the seeds."""

    def __init__(self, params: CharacterizationParameters, geometry: SensorGeometry,
                 rng: np.random.Generator, disparity_quantum_px: float = INDICATIVE_DISPARITY_QUANTUM_PX) -> None:
        self.params, self.geometry, self.rng = params, geometry, rng
        self.disparity_quantum_px = disparity_quantum_px
        """The disparity quantum the staircase is sized with (the rendering model's, so that the
        fine steps really are fractions of the quantum the frames carry)."""
        self.captures: list[PlannedCapture] = []
        self._next_pose_index: dict[tuple, int] = {}

    def add(self, procedure: str, target_id: str, gap_mm: float | None, station_z_mm: float, field: int,
            frames: int, subseries: str, pose: RigidTransform, **details) -> PlannedCapture:
        """Append one capture; its pose index is the next free one of its configuration."""
        key = (procedure, target_id, gap_mm, station_z_mm, field)
        pose_index = self._next_pose_index.get(key, 0)
        self._next_pose_index[key] = pose_index + 1
        seed = int(self.rng.integers(0, SEED_UPPER_BOUND))
        capture = PlannedCapture(procedure=procedure, target_id=target_id, gap_mm=gap_mm,
                                 station_z_mm=station_z_mm, field=field, pose_index=pose_index, frames=frames,
                                 subseries=subseries, target_to_camera=pose, seed=seed,
                                 order=len(self.captures), **details)
        self.captures.append(capture)
        return capture

    def jitter_offsets_mm(self, station_z_mm: float) -> tuple[float, float]:
        """A random lateral offset (H, V), uniform over +/- PHASE_JITTER_SPAN_PX / 2 converted to mm at
        the station depth (Section 6.1, Step 3)."""
        half_span_mm = self.params.phase_jitter_span_px / 2.0 * self.geometry.pixel_footprint_mm(station_z_mm)
        offset_h, offset_v = self.rng.uniform(-half_span_mm, half_span_mm, size=2)
        return float(offset_h), float(offset_v)

    def add_jitter_poses(self, procedure: str, target_id: str, gap_mm: float | None, station_z_mm: float,
                         count: int, frames: int, include_nominal: bool = False) -> None:
        """A configuration of ``count`` random-offset poses (preceded by a zero-offset pose when
        ``include_nominal``) at a station, fronto-parallel."""
        if include_nominal:
            self.add(procedure, target_id, gap_mm, station_z_mm, FIELD_POSITION_CENTER, frames, SUBSERIES_NOMINAL,
                     fronto_parallel_pose(0.0, 0.0, station_z_mm))
        for _ in range(count):
            offset_h, offset_v = self.jitter_offsets_mm(station_z_mm)
            self.add(procedure, target_id, gap_mm, station_z_mm, FIELD_POSITION_CENTER, frames, SUBSERIES_JITTER,
                     fronto_parallel_pose(offset_h, offset_v, station_z_mm),
                     offset_h_mm=offset_h, offset_v_mm=offset_v)

    def add_sentinel(self) -> None:
        """A drift sentinel (Section 5, Step 3): the target mounted at this point of the plan (that of the latest capture;
        the noise plate T2 at the start of the plan) at the reference station, centered, as ``insert_sentinels`` plans it."""
        z_reference = self.params.z_reference_mm
        mounted = self.captures[-1] if self.captures else None
        target_id = TARGET_NOISE_PLATE if mounted is None else mounted.target_id
        gap_mm = None if mounted is None else mounted.gap_mm
        self.add(PROCEDURE_SENTINEL, target_id, gap_mm, z_reference, FIELD_POSITION_CENTER,
                 self.params.sentinel_frames, SUBSERIES_SENTINEL, fronto_parallel_pose(0.0, 0.0, z_reference))


def _plan_registration(builder: _PlanBuilder, quick: bool) -> None:
    """Section 4, Step 6: T2 at poses spread over the volume, each tilted about both axes within
    +/- REGISTRATION_TILT_RANGE_DEG (the hand-eye solve needs rotation about two axes)."""
    params, geometry, rng = builder.params, builder.geometry, builder.rng
    count = DEMO_REGISTRATION_POSES_QUICK if quick else DEMO_REGISTRATION_POSES
    depth_span = DEMO_REGISTRATION_DEPTH_SPAN_FRACTION * (params.z_max_mm - params.z_min_mm)
    for _ in range(count):
        z_mm = float(rng.uniform(params.z_reference_mm - depth_span, params.z_reference_mm + depth_span))
        half_field_h, half_field_v = geometry.half_field_mm(z_mm)
        h_mm = float(rng.uniform(-1.0, 1.0)) * DEMO_REGISTRATION_LATERAL_FRACTION * half_field_h
        v_mm = float(rng.uniform(-1.0, 1.0)) * DEMO_REGISTRATION_LATERAL_FRACTION * half_field_v
        tilt_h, tilt_v = rng.uniform(-params.registration_tilt_range_deg, params.registration_tilt_range_deg, 2)
        # Rotation about the target's H axis, then about its V axis (both through the reference point).
        rotation = (tilted_pose(h_mm, v_mm, z_mm, TILT_AXIS_H, float(tilt_h)).rotation
                    @ tilted_pose(h_mm, v_mm, z_mm, TILT_AXIS_V, float(tilt_v)).rotation)
        builder.add(PROCEDURE_REGISTRATION, TARGET_NOISE_PLATE, None, params.z_reference_mm,
                    FIELD_POSITION_CENTER, params.frames_per_registration_pose, SUBSERIES_MAIN,
                    RigidTransform(rotation, np.array([h_mm, v_mm, z_mm])),
                    notes={"tilt_h_deg": float(tilt_h), "tilt_v_deg": float(tilt_v)})


def _plan_noise(builder: _PlanBuilder) -> None:
    """Section 5: T2 at Z_MIN, Z_REFERENCE and Z_MAX (center field), two tilt poses at the reference
    station, and a drift sentinel before it (a real plan inserts one per hour of clock; the sentinel after series A is
    added by ``demo_plan``)."""
    params = builder.params
    builder.add_sentinel()
    for station in (params.z_min_mm, params.z_reference_mm, params.z_max_mm):
        builder.add(PROCEDURE_NOISE, TARGET_NOISE_PLATE, None, station, FIELD_POSITION_CENTER,
                    params.frames_per_noise_station, SUBSERIES_MAIN, fronto_parallel_pose(0.0, 0.0, station))
    for axis, degrees in DEMO_TILT_POSES_A:
        builder.add(PROCEDURE_NOISE, TARGET_NOISE_PLATE, None, params.z_reference_mm, FIELD_POSITION_CENTER,
                    params.frames_per_tilt_pose, SUBSERIES_TILT,
                    tilted_pose(0.0, 0.0, params.z_reference_mm, axis, degrees), tilt_axis=axis, tilt_deg=degrees)


def _plan_edges(builder: _PlanBuilder) -> None:
    """Section 6.1: T3a and T3b at two stations, a nominal pose and jitter poses each, small gap."""
    params = builder.params
    # The two shape stations around the reference: nearer than Z_MAX so that the 160 mm square stays wide enough on the
    # coarse pixels of the --quick sensor (35 and 49 px at 800 and 566 mm; 17 px at 1600 mm).
    for target_id in (TARGET_RAISED_SQUARE, TARGET_SQUARE_WINDOW):
        for station in params.z_shape_stations_mm()[1:3]:
            builder.add_jitter_poses(PROCEDURE_EDGES, target_id, params.gap_small_mm, station,
                                     DEMO_EDGE_JITTER_POSES, params.frames_per_edge_pose, include_nominal=True)


def _plan_zstep(builder: _PlanBuilder) -> None:
    """Section 6.2: T2 at the reference station. The ladder moves the plate between Z0 (visit A) and
    Z0 + step (visit B) in ABAB cycles for three step sizes (rungs of the expected depth quantum at Z0). The ramp, one
    pose of T2 tilted about H per station with the tilt of ``plan.ramp_tilt_deg``, is planned at the stations of series A
    of the demonstration (Z_MIN, Z_REFERENCE and Z_MAX), where Analysis A supplies the fixed-pattern map the ramp
    analysis subtracts. The staircase (optional second pass) then moves the plate in fractions of the expected depth
    quantum. ``station_z_mm`` is Z0 for every pose; the plate pose carries the move."""
    params, geometry = builder.params, builder.geometry
    z0 = params.z_reference_mm
    quantum_mm = geometry.depth_quantum_mm(builder.disparity_quantum_px, z0)
    rungs = z_step_rungs_mm(params, quantum_mm)
    for ladder_index in DEMO_ZSTEP_LADDER_INDICES:
        step = rungs[ladder_index]
        for _ in range(DEMO_ZSTEP_CYCLES):
            for visit, offset in ((VISIT_A, 0.0), (VISIT_B, step), (VISIT_A, 0.0), (VISIT_B, step)):
                builder.add(PROCEDURE_ZSTEP, TARGET_NOISE_PLATE, None, z0, FIELD_POSITION_CENTER,
                            params.frames_per_zstep_pose, SUBSERIES_LADDER,
                            fronto_parallel_pose(0.0, 0.0, z0 + offset), step_mm=step, visit=visit)
    # The ramp at the A stations (the plate does not depend on the pixel size of the --quick sensor).
    plate = make_noise_plate(params)
    for station in (params.z_min_mm, params.z_reference_mm, params.z_max_mm):
        station_quantum_mm = geometry.depth_quantum_mm(builder.disparity_quantum_px, station)
        tilt = ramp_tilt_deg(params, geometry, plate, station, station_quantum_mm)
        pose, notes = ramp_pose(params, geometry, plate, station, station_quantum_mm, tilt)
        builder.add(PROCEDURE_ZSTEP, TARGET_NOISE_PLATE, None, station, FIELD_POSITION_CENTER,
                    params.frames_per_ramp_pose, SUBSERIES_RAMP, pose, tilt_axis=TILT_AXIS_H, tilt_deg=tilt, notes=notes)
    # The optional staircase divides one expected quantum at Z0 into Z_STAIRCASE_SUBDIVISION fine steps (never below
    # the smallest move the robot is trusted to make).
    fine_step_mm = staircase_step_mm(params, quantum_mm)
    for step_index in range(1, DEMO_STAIRCASE_STEPS + 1):
        offset = step_index * fine_step_mm
        builder.add(PROCEDURE_ZSTEP, TARGET_NOISE_PLATE, None, z0, FIELD_POSITION_CENTER, params.z_staircase_frames,
                    SUBSERIES_STAIRCASE, fronto_parallel_pose(0.0, 0.0, z0 + offset), step_mm=offset)


def _plan_area(builder: _PlanBuilder) -> None:
    """Section 7: T4 and T5 at the shape stations, jitter poses each, small gap."""
    params = builder.params
    for target_id in (TARGET_DISKS, TARGET_CUTOUTS):
        for station in params.z_shape_stations_mm():
            builder.add_jitter_poses(PROCEDURE_AREA, target_id, params.gap_small_mm, station,
                                     DEMO_AREA_JITTER_POSES, params.frames_per_area_pose)


def _plan_detection(builder: _PlanBuilder) -> None:
    """Section 8: T4 and T5 at the shape stations, single-frame jitter poses."""
    params = builder.params
    for target_id in (TARGET_DISKS, TARGET_CUTOUTS):
        for station in params.z_shape_stations_mm():
            builder.add_jitter_poses(PROCEDURE_DETECTION, target_id, params.gap_small_mm, station,
                                     DEMO_DETECTION_POSES, params.frames_per_detection_trial)


def demo_plan(params: CharacterizationParameters, geometry: SensorGeometry, rng: np.random.Generator,
              quick: bool, series: Sequence[str] | None = None,
              disparity_quantum_px: float = INDICATIVE_DISPARITY_QUANTUM_PX) -> list[PlannedCapture]:
    """The demonstration plan (module docstring) of the requested series (all of ALL_SERIES by default),
    in acquisition order R, A, B, Z, C, D with ``order`` set 0..N-1. ``geometry`` is the sensor the plan
    will be rendered with (it converts the jitter span from pixels to mm); ``quick`` uses fewer
    registration poses. When A is included, a drift sentinel follows series A and another follows the last series,
    each on the target mounted there (T2 after A, so that the T2 sentinels bracket A; the target of the last series at
    the end), as ``insert_sentinels`` plans them. Raises ValueError for an unknown series letter."""
    chosen = tuple(ALL_SERIES if series is None else series)
    unknown = [letter for letter in chosen if letter not in ALL_SERIES]
    if unknown:
        raise ValueError(f"unknown series {unknown}; expected a subset of {ALL_SERIES}")
    builder = _PlanBuilder(params, geometry, rng, disparity_quantum_px)
    if PROCEDURE_REGISTRATION in chosen:
        _plan_registration(builder, quick)
    if PROCEDURE_NOISE in chosen:
        _plan_noise(builder)
        builder.add_sentinel()                  # series boundary: T2 is still mounted
    if PROCEDURE_EDGES in chosen:
        _plan_edges(builder)
    if PROCEDURE_ZSTEP in chosen:
        _plan_zstep(builder)
    if PROCEDURE_AREA in chosen:
        _plan_area(builder)
    if PROCEDURE_DETECTION in chosen:
        _plan_detection(builder)
    if PROCEDURE_NOISE in chosen and builder.captures[-1].procedure != PROCEDURE_SENTINEL:
        builder.add_sentinel()                  # the end of the last series, on the target mounted there
    return builder.captures
