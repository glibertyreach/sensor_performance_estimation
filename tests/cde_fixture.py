"""
Synthetic-session builder shared by the tests of Analyses C, D and E
(tests/test_analysis_cde.py) and by the development scripts.

Why a custom plan. The standard target set is laid out for the FULL-size sensor (640 x 480 px); the 160 x 120 px
test sensor sees its small arrays (T4-S, T5-S) with every feature smaller than one pixel, so nothing is detectable
and no area can be measured. This module therefore replaces T4-S and T5-S by arrays whose diameter ladder is
expressed in pixels of the TEST sensor (a geometric ladder of ratio sqrt(2) from LADDER_FIRST_D_PX to
LADDER_LEVELS rungs at the reference station), keeping every other target of the standard set, and plans

    B  edges   T3a and T3b at 750 and 1000 mm, one nominal pose and EDGE_JITTER_POSES random-offset poses each
    C  area    the two arrays at 750 and 1000 mm (center field), AREA_JITTER_POSES jitter poses each; at 750 mm one
               field position (code 2) and, for the cutouts, the open-background variant (no back plate)
    D  detect  cutouts and disks at 750 mm with DETECTION_POSES_* single-frame jitter poses, and the cutouts at
               1000 mm with DETECTION_POSES_FAR poses

All offsets are uniform over +/- PHASE_JITTER_SPAN_PX / 2 (the planner's own rule, via the demo plan builder).
"""
from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import numpy as np

from sensorperf.geometry.targets import (
    TARGET_KIND_CUTOUT_ARRAY, TARGET_KIND_DISK_ARRAY, fronto_parallel_pose, make_feature_array,
    make_standard_target_set,
)
from sensorperf.io.manifest import SUBSERIES_FIELD, SUBSERIES_OPEN
from sensorperf.parameters import (
    CharacterizationParameters, PROCEDURE_AREA, PROCEDURE_DETECTION, PROCEDURE_EDGES, SensorGeometry,
    TARGET_CUTOUTS_SMALL, TARGET_DISKS_SMALL, TARGET_RAISED_SQUARE, TARGET_SQUARE_WINDOW,
)
from sensorperf.simulate.demo_plan import _PlanBuilder, demo_registration, scaled_geometry
from sensorperf.simulate.sensor_model import SyntheticSensorModel
from sensorperf.simulate.session import write_synthetic_session

QUICK_DIVISOR = 4
"""The test sensor is the indicative one divided by this (640 x 480 -> 160 x 120)."""
LADDER_FIRST_D_PX = 1.4
"""Smallest diameter of the test arrays in pixels of the test sensor at the reference station."""
LADDER_LEVELS = 8
"""Rungs of the test ladder (1.4 to 15.9 px at the reference station, ratio sqrt(2))."""
LADDER_RATIO = math.sqrt(2.0)
"""Ratio of successive diameters."""
ISOLATION_MM = 40.0
"""Edge-to-edge spacing of the test features (about 9 px at the reference station)."""
EDGE_JITTER_POSES = 6
EDGE_FRAMES = 4
AREA_JITTER_POSES = 8
AREA_FRAMES = 2
FIELD_POSES = 4
FIELD_CODE = 2
FIELD_FRACTION_OF_HALF_FIELD = 0.3
OPEN_POSES = 4
DETECTION_POSES_CUTOUT = 300
DETECTION_POSES_DISK = 100
DETECTION_POSES_FAR = 60
EDGE_STATIONS_MM = (750.0, 1000.0)
AREA_STATIONS_MM = (750.0, 1000.0)
FAR_STATION_MM = 1000.0
ROBOT_REPEATABILITY_MM = 0.05
SESSION_SEED = 7


def test_geometry() -> SensorGeometry:
    """The 160 x 120 px sensor of the tests."""
    return scaled_geometry(SensorGeometry.indicative(), QUICK_DIVISOR)


def test_model(geometry: SensorGeometry, seed: int) -> SyntheticSensorModel:
    """The indicative renderer with the noise scaled to the coarser pixels, as sensorperf.cli.simulate does."""
    indicative = SyntheticSensorModel.indicative(geometry)
    return replace(indicative, fixed_pattern_seed=seed,
                   disparity_noise_px=indicative.disparity_noise_px / QUICK_DIVISOR,
                   disparity_quantum_px=indicative.disparity_quantum_px / QUICK_DIVISOR)


def ladder_diameters_mm(params: CharacterizationParameters, geometry: SensorGeometry) -> list[float]:
    """The test ladder in mm: LADDER_LEVELS diameters from LADDER_FIRST_D_PX pixels at the reference station."""
    return [geometry.pixel_footprint_mm(params.z_reference_mm) * LADDER_FIRST_D_PX * LADDER_RATIO ** k
            for k in range(LADDER_LEVELS)]


def build_targets(params: CharacterizationParameters, geometry: SensorGeometry):
    """The standard target set with T4-S and T5-S replaced by the test-ladder arrays."""
    full = SensorGeometry.indicative()
    targets = make_standard_target_set(params, full)
    diameters = ladder_diameters_mm(params, geometry)
    targets.add(make_feature_array(TARGET_DISKS_SMALL, TARGET_KIND_DISK_ARRAY, diameters, ISOLATION_MM,
                                   params.gap_small_mm))
    targets.add(make_feature_array(TARGET_CUTOUTS_SMALL, TARGET_KIND_CUTOUT_ARRAY, diameters, ISOLATION_MM,
                                   params.gap_small_mm))
    return targets


def build_plan(params: CharacterizationParameters, geometry: SensorGeometry, rng: np.random.Generator,
               series: str = "BCD"):
    """The plan of the module docstring (only the requested series letters)."""
    builder = _PlanBuilder(params, geometry, rng)
    gap = params.gap_small_mm
    if "B" in series:
        for target_id in (TARGET_RAISED_SQUARE, TARGET_SQUARE_WINDOW):
            for station in EDGE_STATIONS_MM:
                builder.add_jitter_poses(PROCEDURE_EDGES, target_id, gap, station, EDGE_JITTER_POSES, EDGE_FRAMES,
                                         include_nominal=True)
    if "C" in series:
        for target_id in (TARGET_DISKS_SMALL, TARGET_CUTOUTS_SMALL):
            for station in AREA_STATIONS_MM:
                builder.add_jitter_poses(PROCEDURE_AREA, target_id, gap, station, AREA_JITTER_POSES, AREA_FRAMES)
            # Field sub-series (Section 7, Step 3) at the reference station: field code 2 = (+, -).
            station = params.z_reference_mm
            half_h, half_v = geometry.half_field_mm(station)
            h0, v0 = FIELD_FRACTION_OF_HALF_FIELD * half_h, -FIELD_FRACTION_OF_HALF_FIELD * half_v
            for _ in range(FIELD_POSES):
                dh, dv = builder.jitter_offsets_mm(station)
                builder.add(PROCEDURE_AREA, target_id, gap, station, FIELD_CODE, AREA_FRAMES, SUBSERIES_FIELD,
                            fronto_parallel_pose(h0 + dh, v0 + dv, station), offset_h_mm=dh, offset_v_mm=dv)
        # Open-background variant of the cutouts (Section 7, Step 4): no back plate.
        station = params.z_reference_mm
        for _ in range(OPEN_POSES):
            dh, dv = builder.jitter_offsets_mm(station)
            builder.add(PROCEDURE_AREA, TARGET_CUTOUTS_SMALL, None, station, 0, AREA_FRAMES, SUBSERIES_OPEN,
                        fronto_parallel_pose(dh, dv, station), offset_h_mm=dh, offset_v_mm=dv)
    if "D" in series:
        builder.add_jitter_poses(PROCEDURE_DETECTION, TARGET_CUTOUTS_SMALL, gap, params.z_reference_mm,
                                 DETECTION_POSES_CUTOUT, 1)
        builder.add_jitter_poses(PROCEDURE_DETECTION, TARGET_DISKS_SMALL, gap, params.z_reference_mm,
                                 DETECTION_POSES_DISK, 1)
        builder.add_jitter_poses(PROCEDURE_DETECTION, TARGET_CUTOUTS_SMALL, gap, FAR_STATION_MM,
                                 DETECTION_POSES_FAR, 1)
    return builder.captures


def build_session(root: str | Path, series: str = "BCD", seed: int = SESSION_SEED,
                  params: CharacterizationParameters | None = None) -> Path:
    """Render the test session of the requested series into ``root`` and return the folder."""
    params = CharacterizationParameters() if params is None else params
    geometry = test_geometry()
    rng = np.random.default_rng(seed)
    targets = build_targets(params, geometry)
    plan = build_plan(params, geometry, rng, series)
    return write_synthetic_session(root, params, geometry, test_model(geometry, seed), demo_registration(),
                                   targets, plan, rng, frame_scale=1.0,
                                   robot_repeatability_mm=ROBOT_REPEATABILITY_MM)
