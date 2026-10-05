"""
Synthetic-session builder shared by the tests of Analyses C, D and E
(tests/test_analysis_cde.py) and by the development scripts.

The session is rendered with the INDICATIVE sensor of the 160 x 120 px test geometry (the indicative geometry with
the pixels four times larger and the same field of view) and the STANDARD target set of the full-size geometry
(``make_standard_target_set``: T4 disks and T5 cutouts of 7.0, 19.7 and 55.8 mm, each with three blank sites and, for
the disks, a post site). The test sensor sees those features with one quarter of the pixels the full-size sensor would
(0.75 to 3, 2.1 to 8.5 and 6 to 24 px across the station ladder), and its minimum feature diameter is one quarter of the
indicative 10 px (2.5 px), so the detection transition falls inside the ladder: the smallest feature is resolved only at
the nearest station and the largest at every station.

The plan covers the five shape stations of the redesign (400, 566, 800, 1131 and 1600 mm; the standard shape stride
makes neighboring features coincide in D_px after six ladder steps, and these stations give each pair of neighboring
features two points in their shared D_px range) and

    B  edges   T3a and T3b at EDGE_STATIONS_MM, one nominal pose and EDGE_JITTER_POSES random-offset poses each
    C  area    both plates at every station (center field), AREA_JITTER_POSES jitter poses each; at the reference
               station one field position (code 2) and, for the cutouts, the open-background variant (no back plate)
    D  detect  both plates at every station with DETECTION_POSES single-frame jitter poses each

All offsets are uniform over +/- PHASE_JITTER_SPAN_PX / 2 (the planner's own rule, via the demo plan builder).
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from sensorperf.geometry.targets import fronto_parallel_pose, make_standard_target_set
from sensorperf.io.manifest import SUBSERIES_FIELD, SUBSERIES_OPEN
from sensorperf.parameters import (
    CharacterizationParameters, PROCEDURE_AREA, PROCEDURE_DETECTION, PROCEDURE_EDGES, SensorGeometry,
    TARGET_CUTOUTS, TARGET_DISKS, TARGET_RAISED_SQUARE, TARGET_SQUARE_WINDOW,
)
from sensorperf.simulate.demo_plan import _PlanBuilder, demo_registration, scaled_geometry
from sensorperf.simulate.sensor_model import SyntheticSensorModel
from sensorperf.simulate.session import write_synthetic_session

QUICK_DIVISOR = 4
"""The test sensor is the indicative one divided by this (640 x 480 -> 160 x 120)."""
STATIONS_MM = CharacterizationParameters().z_shape_stations_mm()
"""The stations of the C and D parts of the test session (the five shape stations)."""
EDGE_JITTER_POSES = 6
EDGE_FRAMES = 4
AREA_JITTER_POSES = 8
AREA_FRAMES = 2
FIELD_POSES = 4
FIELD_CODE = 2
FIELD_FRACTION_OF_HALF_FIELD = 0.3
OPEN_POSES = 4
DETECTION_POSES = 60
"""Single-frame poses per plate and station."""
DETECTION_POSES_FAR = 300
"""Single-frame poses per plate at the farthest station (Z_MAX): the extended trials of the low point (D_5) of the
redesign, DETECTION_LOW_TRIALS = 300, which resolve a 5 percent detection probability to about +/- 2.5 percent."""
EDGE_STATIONS_MM = (566.0, 800.0)
"""The B stations: the square must stay wide enough on the test sensor for the one-edge-near rule (49 and 35 px)."""
ROBOT_REPEATABILITY_MM = 0.05
SESSION_SEED = 7
"""Seed of the synthetic session. No test may depend on a lucky draw: the low-point test checks the rule
that ties the D_5 bracket to the per-level counts, not the counts themselves."""


def test_geometry() -> SensorGeometry:
    """The 160 x 120 px sensor of the tests."""
    return scaled_geometry(SensorGeometry.indicative(), QUICK_DIVISOR)


def test_model(geometry: SensorGeometry, seed: int) -> SyntheticSensorModel:
    """The indicative renderer with the noise, the quantum and the minimum feature size scaled to the coarser pixels,
    as sensorperf.cli.simulate does."""
    from dataclasses import replace
    return replace(SyntheticSensorModel.indicative_scaled(geometry, QUICK_DIVISOR), fixed_pattern_seed=seed)


def build_targets(params: CharacterizationParameters):
    """The standard target set of the full-size indicative geometry (physical targets do not depend on the sensor)."""
    return make_standard_target_set(params, SensorGeometry.indicative())


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
        for target_id in (TARGET_DISKS, TARGET_CUTOUTS):
            for station in STATIONS_MM:
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
            builder.add(PROCEDURE_AREA, TARGET_CUTOUTS, None, station, 0, AREA_FRAMES, SUBSERIES_OPEN,
                        fronto_parallel_pose(dh, dv, station), offset_h_mm=dh, offset_v_mm=dv)
    if "D" in series:
        for target_id in (TARGET_CUTOUTS, TARGET_DISKS):
            for station in STATIONS_MM:
                count = DETECTION_POSES_FAR if station == params.z_max_mm else DETECTION_POSES
                builder.add_jitter_poses(PROCEDURE_DETECTION, target_id, gap, station, count, 1)
    return builder.captures


def build_session(root: str | Path, series: str = "BCD", seed: int = SESSION_SEED,
                  params: CharacterizationParameters | None = None) -> Path:
    """Render the test session of the requested series into ``root`` and return the folder."""
    params = CharacterizationParameters() if params is None else params
    geometry = test_geometry()
    rng = np.random.default_rng(seed)
    targets = build_targets(params)
    plan = build_plan(params, geometry, rng, series)
    return write_synthetic_session(root, params, geometry, test_model(geometry, seed), demo_registration(),
                                   targets, plan, rng, frame_scale=1.0,
                                   robot_repeatability_mm=ROBOT_REPEATABILITY_MM)
