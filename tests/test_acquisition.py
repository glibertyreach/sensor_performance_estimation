"""
Tests of the acquisition-side tools: the station planner (Sections 4 to 9), the
pose log -> manifest builder (Section 9), the registration command line
(Section 4, Steps 6 and 7), and the quick-look check with the D pilot post check
(Section 8, Step 1; Section 13, Step 2).

Every test names the step of the procedure it exercises and its acceptance
criterion. Everything runs on the indicative sensor geometry; the check tests
write a tiny synthetic stack (160 x 120 px, the indicative intrinsics divided
by four) with ``sensorperf.io.matcloud.write_matcloud``, so no renderer is
needed.
"""
from __future__ import annotations

import csv
import dataclasses
import json
from collections import Counter
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from sensorperf.acquisition import check as check_module
from sensorperf.acquisition.check import (
    FLAG_FRONT_OFFSET, FLAG_FRONT_TILT, FLAG_LOW_VALID, check_session, pilot_post_check,
)
from sensorperf.acquisition.plan import (
    DOCUMENT_ESTIMATE_FRAMES, DOCUMENT_ESTIMATE_HOURS, DOCUMENT_ESTIMATE_POSES, FILTERS_OFF_POSE_INDEX_BASE, PLAN_CSV_NAME, PLAN_FIGURE_NAME, PLAN_SUMMARY_NAME, PlanDiagnostics,
    PlannedCapture, SERIES_ORDER, TIER_A_DISPARITY_QUANTUM_PX, budget_total, capture_budget, capture_duration_s,
    camera_of, fit_violation_px, format_budget_table, insert_sentinels, jitter_offset_mm, place_in_field,
    plan_detection_series, plan_edge_series, plan_full_session, plan_noise_series, plan_registration,
    plan_zstep_series, read_plan_csv, write_plan, APPROACH_FROM_BELOW, MIN_POSE_LOG_DECIMALS,
    count_sentinel_remounts, tilt_is_feasible, tilt_near_edge_mm,
)
from sensorperf.acquisition.pose_log import PoseLogError, build_manifest, build_manifest_with_report
from sensorperf.cli import check_captures as check_cli
from sensorperf.cli import make_manifest as manifest_cli
from sensorperf.cli import plan_stations as plan_cli
from sensorperf.cli import register as register_cli
from sensorperf.geometry.registration import Registration, plane_of_pose
from sensorperf.geometry.targets import fronto_parallel_pose, make_standard_target_set
from sensorperf.geometry.transforms import RigidTransform
from sensorperf.io.manifest import (
    FIELD_FRACTION_ACHIEVED_KEY, FrameRecord, SUBSERIES_FIELD, SUBSERIES_JITTER, VISIT_A, VISIT_B, write_manifest_csv,
)
from sensorperf.features.planes import plane_depth_image
from sensorperf.io.matcloud import write_matcloud
from sensorperf.io.session import PARAMETERS_FILE_NAME, SENSOR_CONFIG_FILE_NAME, Session, SensorConfig
from sensorperf.parameters import (
    CharacterizationParameters, FIELD_POSITION_CODES, PROCEDURE_AREA, PROCEDURE_DETECTION, PROCEDURE_EDGES,
    PROCEDURE_NOISE, PROCEDURE_REGISTRATION, PROCEDURE_SENTINEL, SensorGeometry, TARGET_DISKS,
    TARGET_NOISE_PLATE, TARGET_CUTOUTS,
)

PARAMS = CharacterizationParameters()
GEOMETRY = SensorGeometry.indicative()
MASTER_SEED = 1
"""Master seed of the plans of these tests."""
SMALL_PARAMS = dataclasses.replace(
    PARAMS, phase_jitter_poses_edge=2, phase_jitter_poses_area=2, field_subseries_poses_area=2,
    detection_trials_per_level=3, detection_zero_trials=5, z_step_repeats=2, z_step_ladder_mm=(0.1, 1.0),
    frames_per_zstep_pose=3, z_staircase_frames=2, frames_per_noise_station=4, frames_per_tilt_pose=2)
"""Reduced parameters for the tests that write files (few poses, few frames)."""

HOURS_TOLERANCE = 0.05
"""Absolute tolerance (hours) of the comparison of the stored budget estimate with the budget formula."""
JITTER_TOLERANCE_MM = 1.0e-9
"""Numerical slack when comparing a logged offset with the half span."""
POSE_TOLERANCE = 1.0e-9
"""Agreement of poses that went through a CSV round trip or a registration composition (mm and degrees)."""

SCALE_DOWN = 4.0
"""Image size and intrinsics are divided by this for the synthetic check stacks (160 x 120 px)."""
CHECK_STATION_MM = PARAMS.z_reference_mm
CHECK_FRAMES = 3
DISPLACEMENT_MM = 5.0
"""A plane displaced by this much from the registered one must be flagged (the thresholds are 3 mm)."""
TILT_DEG = 6.0
"""A plane tilted by this much from the registered one must be flagged (the threshold is 2 degrees)."""
PILOT_POSES = 10
"""C poses of the synthetic post-check test."""


@pytest.fixture(scope="module")
def full_plan() -> tuple[list[PlannedCapture], PlanDiagnostics]:
    """The default plan of every series (Sections 4 to 8) with sentinels, for the structural tests."""
    diagnostics = PlanDiagnostics()
    plan = plan_full_session(PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED), diagnostics=diagnostics)
    return plan, diagnostics


def random_registration(seed: int = 3) -> Registration:
    """A plausible registration (camera looking along the robot's x axis) for the tests."""
    rng = np.random.default_rng(seed)
    camera_to_base = RigidTransform.from_rotation_vector_degrees(rng.normal(0.0, 30.0, 3), [900.0, 50.0, 400.0])
    target_to_flange = RigidTransform.from_rotation_vector_degrees(rng.normal(0.0, 5.0, 3), [5.0, -3.0, 120.0])
    return Registration(camera_to_base, target_to_flange, residual_rms_mm=0.05, pose_count=30, accepted=True)


# ---------------------------------------------------------------------------
# Planning (Sections 4 to 9)
# ---------------------------------------------------------------------------
def test_full_plan_has_unique_pose_keys_and_file_names(full_plan):
    """Section 9 (file-name rule): every pose has its own key and every frame its own file name, all series are
    present, and ``order`` runs 0..N-1."""
    plan, _ = full_plan
    keys = [c.pose_key() for c in plan]
    assert len(set(keys)) == len(keys)
    names = [c.file_name(f) for c in plan for f in range(c.frames)]
    assert len(set(names)) == len(names)
    assert {c.procedure for c in plan} == set(SERIES_ORDER)
    assert [c.order for c in plan] == list(range(len(plan)))
    labels = {(c.procedure, c.subseries) for c in plan}
    for expected in [("A", "main"), ("A", "tilt"), ("A", "remount"), ("B", "nominal"), ("B", "jitter"),
                     ("Z", "ladder"), ("Z", "staircase"), ("C", "jitter"), ("C", "field"), ("D", "jitter"),
                     ("D", "extended"), ("S", "sentinel"), ("R", "main")]:
        assert expected in labels


def test_optional_variants_keep_keys_unique():
    """Section 5 Step 7 (filters off) and Section 7 Step 4 (open background): the optional repeats add poses with
    their own pose indices, so keys and file names stay unique."""
    plan = plan_full_session(PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED), filters_off=True,
                             open_background=True, extended=False)
    assert len({c.pose_key() for c in plan}) == len(plan)
    off = [c for c in plan if c.subseries == "filters_off"]
    assert all(c.pose_index >= FILTERS_OFF_POSE_INDEX_BASE for c in off)
    # Section 4, Step 4.2: the repeat covers A, B-HV and B-Z (A: 47 station poses + 16 tilt poses).
    off_by_series = {p: sum(1 for c in off if c.procedure == p) for p in ("A", "B", "Z")}
    assert len(off) == sum(off_by_series.values())
    assert off_by_series["A"] == 47 + 16
    assert off_by_series["B"] == sum(1 for c in plan if c.procedure == "B" and c.subseries in ("nominal", "jitter"))
    assert off_by_series["Z"] == sum(1 for c in plan if c.procedure == "Z" and c.subseries in ("ladder", "staircase"))
    assert off_by_series["B"] > 0 and off_by_series["Z"] > 0
    open_poses = [c for c in plan if c.subseries == "open"]
    assert len(open_poses) == PARAMS.phase_jitter_poses_area
    assert all(c.gap_mm is None and c.station_z_mm == PARAMS.z_reference_mm for c in open_poses)
    assert {c.target_id for c in open_poses} == {"T5"}


def test_plan_is_reproducible_from_the_master_seed():
    """Section 5 Step 2 (logged seed): the same master seed gives the same plan, another seed another order."""
    one = plan_full_session(SMALL_PARAMS, GEOMETRY, np.random.default_rng(5))
    again = plan_full_session(SMALL_PARAMS, GEOMETRY, np.random.default_rng(5))
    other = plan_full_session(SMALL_PARAMS, GEOMETRY, np.random.default_rng(6))
    assert [c.pose_key() for c in one] == [c.pose_key() for c in again]
    assert all(np.allclose(a.target_to_camera.as_matrix(), b.target_to_camera.as_matrix()) for a, b in zip(one, again))
    assert [c.pose_key() for c in one] != [c.pose_key() for c in other]


def test_registration_poses_span_the_volume():
    """Section 4 Step 6 and redesign note Section 3: REGISTRATION_POSES poses of REGISTRATION_FRAMES frames of the noise
    plate T2 (no pattern plate any more) spanning Z_MIN = 400 to Z_MAX = 1600 mm, with tilts within
    REGISTRATION_TILT_RANGE_DEG about H and V."""
    poses = plan_registration(PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED))
    assert len(poses) == PARAMS.registration_poses
    assert all(c.frames == PARAMS.frames_per_registration_pose and c.target_id == TARGET_NOISE_PLATE
               for c in poses)
    depths = [c.notes["depth_mm"] for c in poses]
    assert min(depths) == pytest.approx(PARAMS.z_min_mm) and max(depths) == pytest.approx(PARAMS.z_max_mm)
    assert (PARAMS.z_min_mm, PARAMS.z_max_mm) == (400.0, 1600.0)
    for c in poses:
        assert abs(c.notes["tilt_h_deg"]) <= PARAMS.registration_tilt_range_deg
        assert abs(c.notes["tilt_v_deg"]) <= PARAMS.registration_tilt_range_deg
        assert c.target_to_camera.translation[2] == pytest.approx(c.notes["depth_mm"])


def test_noise_station_order_is_a_permutation_of_the_station_list(full_plan):
    """Section 5 Steps 1 and 2 and redesign note Section 1: the nine ladder stations at the five field positions plus the two
    legacy depths (700 and 1000 mm) at the center only, 9 x 5 + 2 = 47 main poses, are all visited once, in the order of the
    logged seed; the tilt sub-series (16 poses: the 800 and 1600 mm stations, every tilt at 400 mm being infeasible) and the
    repeat-mount check follow."""
    plan, _ = full_plan
    main = [c for c in plan if c.procedure == PROCEDURE_NOISE and c.subseries == "main"]
    stations = [(z, code) for z in PARAMS.z_stations_mm() for code in FIELD_POSITION_CODES]
    stations += [(z, 0) for z in PARAMS.legacy_extra_stations_mm()]
    assert len(main) == len(stations) == 47
    assert {c.station_z_mm for c in main} == set(PARAMS.z_stations_mm()) | set(PARAMS.legacy_metric_depths_mm)
    assert sorted((c.station_z_mm, c.field) for c in main) == sorted(stations)
    assert [(c.station_z_mm, c.field) for c in main] != sorted(stations)
    seeds = {c.seed for c in main}
    assert len(seeds) == 1 and seeds == {c.notes["order_seed"] for c in main}
    permutation = np.random.default_rng(seeds.pop()).permutation(len(stations))
    assert [(c.station_z_mm, c.field) for c in main] == [stations[int(i)] for i in permutation]
    assert all(c.frames == PARAMS.frames_per_noise_station for c in main)
    tilt = [c for c in plan if c.procedure == PROCEDURE_NOISE and c.subseries == "tilt"]
    assert len(tilt) == 2 * 2 * len(PARAMS.tilt_angles_deg) == 16
    assert Counter((c.tilt_axis, c.tilt_deg) for c in tilt)[("V", 15.0)] == 2
    assert all(c.frames == PARAMS.frames_per_tilt_pose and c.field == 0 for c in tilt)
    remount = [c for c in plan if c.subseries == "remount"]
    assert len(remount) == 1 and remount[0].station_z_mm == PARAMS.z_reference_mm and remount[0].field == 0


def test_tilt_feasibility_skips_tilts_that_bring_the_plate_edge_inside_z_min():
    """Section 5, Step 5 (tilt feasibility): a tilt is planned only where Z - h sin(tilt) >= Z_MIN, h the half extent of the
    400 x 400 mm plate across the tilt axis (200 mm). At the indicative geometry every tilt at 400 mm is skipped, with its
    reason listed in the diagnostics, and the tilt sub-series keeps the 800 and 1600 mm stations in both axes and all angles."""
    plate = make_standard_target_set(PARAMS, GEOMETRY).get(TARGET_NOISE_PLATE)
    assert (plate.half_width_mm, plate.half_height_mm) == (200.0, 200.0)
    # The rule itself, including its equality case (Z = 500 mm, 30 deg: the edge is exactly at Z_MIN).
    assert tilt_near_edge_mm(plate, 800.0, "V", 30.0) == pytest.approx(700.0)
    for axis in ("H", "V"):
        assert not any(tilt_is_feasible(PARAMS, plate, 400.0, axis, angle) for angle in (15.0, 30.0, 45.0))
        assert all(tilt_is_feasible(PARAMS, plate, z, axis, angle) for z in (800.0, 1600.0)
                   for angle in PARAMS.tilt_angles_deg)
    assert tilt_is_feasible(PARAMS, plate, PARAMS.z_min_mm + 200.0 * np.sin(np.radians(30.0)), "V", 30.0)
    assert not tilt_is_feasible(PARAMS, plate, PARAMS.z_min_mm + 200.0 * np.sin(np.radians(30.0)) - 0.01, "V", 30.0)
    diagnostics = PlanDiagnostics()
    plan = plan_noise_series(PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED), with_sentinels=False,
                             diagnostics=diagnostics)
    tilt = [c for c in plan if c.subseries == "tilt"]
    assert {c.station_z_mm for c in tilt} == {800.0, 1600.0}
    assert Counter((c.station_z_mm, c.tilt_axis) for c in tilt) == {
        (z, axis): len(PARAMS.tilt_angles_deg) for z in (800.0, 1600.0) for axis in ("H", "V")}
    assert all(tilt_near_edge_mm(plate, c.station_z_mm, c.tilt_axis, c.tilt_deg) >= PARAMS.z_min_mm for c in tilt)
    # The skipped poses are listed with the reason, per axis.
    assert len(diagnostics.skipped) == 2
    for axis, message in zip(("V", "H"), diagnostics.skipped):
        assert f"tilt about {axis} at Z = 400 mm: skipped 0, 15, 30, 45 deg" in message and "Z_MIN" in message
    from sensorperf.acquisition.plan import plan_summary_text
    text = plan_summary_text(plan, PARAMS, GEOMETRY, diagnostics)
    assert "Skipped poses (left out of the plan): 2" in text and "A tilt about V at Z = 400 mm" in text


def test_tilt_feasibility_follows_the_plate_size_and_z_min():
    """A smaller plate lets a tilt through at 400 mm: with a 100 mm half extent a 15 degree tilt keeps the near edge at 374 mm
    < 400 (skipped), while with Z_MIN lowered to 300 mm the 400 mm station is feasible up to 45 degrees (329 mm)."""
    small_plate = dataclasses.replace(PARAMS, noise_plate_size_mm=(200.0, 200.0))
    plate = make_standard_target_set(small_plate, GEOMETRY).get(TARGET_NOISE_PLATE)
    assert not tilt_is_feasible(small_plate, plate, 400.0, "V", 15.0)
    low = dataclasses.replace(small_plate, z_min_mm=300.0)
    assert all(tilt_is_feasible(low, plate, 400.0, "H", angle) for angle in low.tilt_angles_deg)


def test_legacy_depths_are_captured_at_the_center_only(full_plan):
    """Redesign note Section 1 (specification review): the legacy depths 700 and 1000 mm are extra A stations at the center
    field position only, so the main sub-series has 9 x 5 + 2 = 47 poses; the nine ladder stations keep all five positions."""
    plan, _ = full_plan
    main = [c for c in plan if c.procedure == PROCEDURE_NOISE and c.subseries == "main"]
    assert len(main) == 9 * len(FIELD_POSITION_CODES) + len(PARAMS.legacy_extra_stations_mm()) == 47
    for z in PARAMS.legacy_extra_stations_mm():
        assert [c.field for c in main if c.station_z_mm == z] == [0]
    for z in PARAMS.z_stations_mm():
        assert sorted(c.field for c in main if c.station_z_mm == z) == sorted(FIELD_POSITION_CODES)
    assert PARAMS.legacy_extra_stations_mm() == (700.0, 1000.0)


def test_field_fraction_achieved_is_recorded_in_the_pose_notes(full_plan):
    """Section 5, Step 1: every pose placed at a field position records the achieved fraction of the requested offset in its
    notes; it is 1 for a position that fits (the center always) and equals the fraction kept of a pulled-in pose, which is
    the one listed as "kept N%" in plan_summary.txt. The margin of the fit is BOUNDARY_BAND_HALF_WIDTH_PX."""
    plan, _ = full_plan
    main = [c for c in plan if c.procedure == PROCEDURE_NOISE and c.subseries == "main"]
    assert all(FIELD_FRACTION_ACHIEVED_KEY in c.notes for c in main)
    assert all(c.notes[FIELD_FRACTION_ACHIEVED_KEY] == 1.0 for c in main if c.field == 0 and "field_placement" not in c.notes)
    pulled = [c for c in main if "field_placement" in c.notes and c.field != 0]
    assert pulled
    for c in pulled:
        assert c.notes[FIELD_FRACTION_ACHIEVED_KEY] == c.notes["field_placement"]["fraction_kept"]
        assert 0.0 <= c.notes[FIELD_FRACTION_ACHIEVED_KEY] <= 1.0
    assert any(c.notes[FIELD_FRACTION_ACHIEVED_KEY] < 1.0 for c in pulled)
    # The manifest carries it as string metadata; poses without a field placement carry nothing.
    assert float(pulled[0].manifest_metadata()[FIELD_FRACTION_ACHIEVED_KEY]) == pulled[0].notes[FIELD_FRACTION_ACHIEVED_KEY]
    assert [c for c in plan if c.subseries == "tilt"][0].manifest_metadata() == {}
    # The fit margin is the ROI shrink of Analysis A: moving the margin changes the fraction kept.
    plate = make_standard_target_set(PARAMS, GEOMETRY).get(TARGET_NOISE_PLATE)
    at_band = place_in_field(PARAMS, GEOMETRY, plate, 1000.0, (1.0, -1.0), PARAMS.boundary_band_half_width_px)
    wider = place_in_field(PARAMS, GEOMETRY, plate, 1000.0, (1.0, -1.0), 2.0 * PARAMS.boundary_band_half_width_px)
    assert wider.fraction_kept < at_band.fraction_kept


def test_field_positions_are_pulled_inward_until_the_target_fits():
    """Section 5 Step 1: the off-axis positions are the sign x FIELD_OFFSET_FRACTION x half field, pulled inward along
    the field direction when the plate would not fit; the center is never moved."""
    plate = make_standard_target_set(PARAMS, GEOMETRY).get(TARGET_NOISE_PLATE)
    margin = PARAMS.boundary_band_half_width_px
    camera = camera_of(GEOMETRY)
    far = place_in_field(PARAMS, GEOMETRY, plate, 1000.0, (1.0, -1.0), margin)
    half_h, half_v = GEOMETRY.half_field_mm(1000.0)
    assert far.requested_h_mm == pytest.approx(PARAMS.field_offset_fraction * half_h)
    assert far.requested_v_mm == pytest.approx(-PARAMS.field_offset_fraction * half_v)
    assert 0.0 < far.fraction_kept < 1.0 and far.fits
    assert far.h_mm / far.requested_h_mm == pytest.approx(far.fraction_kept)
    assert far.v_mm / far.requested_v_mm == pytest.approx(far.fraction_kept)
    assert fit_violation_px(camera, plate, fronto_parallel_pose(far.h_mm, far.v_mm, 1000.0), margin) == 0.0
    assert fit_violation_px(camera, plate, fronto_parallel_pose(far.requested_h_mm, far.requested_v_mm, 1000.0),
                            margin) > 0.0
    center = place_in_field(PARAMS, GEOMETRY, plate, 800.0, (0.0, 0.0), margin)
    assert (center.h_mm, center.v_mm, center.fraction_kept) == (0.0, 0.0, 1.0)
    # The adjustments are recorded for the summary.
    main = plan_noise_series(PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED), with_sentinels=False)
    adjusted = [c for c in main if "field_placement" in c.notes]
    assert adjusted and all(c.field != 0 for c in adjusted if c.notes["field_placement"]["fits"])


def test_sentinels_follow_the_budget_clock(full_plan):
    """Section 5 Step 3: a sentinel before the first station, one every DRIFT_SENTINEL_INTERVAL_MIN of estimated clock or
    earlier at a series boundary, one after the last; each is centered at Z_REFERENCE_MM with SENTINEL_FRAMES frames on the
    mounted target (see test_sentinels_are_captured_on_the_mounted_target)."""
    plan, _ = full_plan
    seconds_per_frame = 1.0 / GEOMETRY.frame_rate_hz
    interval = PARAMS.drift_sentinel_interval_min * 60.0
    non_registration = [c for c in plan if c.procedure != PROCEDURE_REGISTRATION]
    assert non_registration[0].procedure == PROCEDURE_SENTINEL and non_registration[-1].procedure == PROCEDURE_SENTINEL
    assert non_registration[0].target_id == TARGET_NOISE_PLATE            # series A comes first: its T2 is mounted
    clock, starts = 0.0, []
    longest = 0.0
    for c in non_registration:
        duration = capture_duration_s(c, seconds_per_frame, PARAMS.move_and_settle_time_s)
        longest = max(longest, duration)
        if c.procedure == PROCEDURE_SENTINEL:
            starts.append(clock)
            assert (c.field, c.frames) == (0, PARAMS.sentinel_frames)
            assert c.station_z_mm == PARAMS.z_reference_mm
        clock += duration
    gaps = np.diff(starts)
    assert len(starts) >= int(clock // interval) + 1                     # at least one per full interval, plus the last
    # No gap exceeds one interval by more than the longest single capture (a boundary sentinel only shortens a gap).
    assert np.all(gaps <= interval + longest + 1e-6)
    # Inserting again changes nothing: existing sentinels are replaced, not duplicated.
    again = insert_sentinels(plan, PARAMS, GEOMETRY, seconds_per_frame, PARAMS.move_and_settle_time_s)
    assert [c.pose_key() for c in again] == [c.pose_key() for c in plan]


def test_sentinels_are_captured_on_the_mounted_target(full_plan):
    """Section 5 Step 3 (specification review): a sentinel is captured on the front plane of the target mounted at that point of
    the plan, with the gap as mounted, so no sentinel row carries T2 while another target is mounted; the first sentinel after
    each mount is that target's reference; T2 sentinels bracket A (before and after) and sit at the end of B-Z; the plan
    summary reports zero sentinel re-mounts and no warning about re-mounts."""
    plan, diagnostics = full_plan
    captures = [c for c in plan if c.procedure not in (PROCEDURE_SENTINEL,)]
    mounted_of_position = {}
    last = None
    for index, c in enumerate(plan):
        if c.procedure != PROCEDURE_SENTINEL:
            last = c
        elif last is not None:
            mounted_of_position[index] = last
    sentinels = [(i, c) for i, c in enumerate(plan) if c.procedure == PROCEDURE_SENTINEL]
    assert len(sentinels) >= 8
    mount_serial_of = {}
    serial, previous_target = 0, None
    for c in plan:                                           # mount number of every capture, as the analysis counts them
        if c.procedure != PROCEDURE_SENTINEL:
            if previous_target is not None and c.target_id != previous_target:
                serial += 1
            previous_target = c.target_id
        mount_serial_of[id(c)] = serial
    referenced = set()
    for index, c in sentinels:
        mounted = mounted_of_position.get(index)
        if mounted is None:                                   # before everything: the first capture after it is mounted
            mounted = next(x for x in plan[index:] if x.procedure not in (PROCEDURE_SENTINEL, PROCEDURE_REGISTRATION))
        assert (c.target_id, c.gap_mm) == (mounted.target_id, mounted.gap_mm)
        # In particular T2 only while T2 (the plate of A and B-Z) is what is mounted.
        assert c.target_id != TARGET_NOISE_PLATE or mounted.target_id == TARGET_NOISE_PLATE
        assert c.notes["sentinel_target"] == c.target_id and "mounted target" in c.notes["sentinel_note"]
        assert (c.field, c.station_z_mm, c.frames) == (0, PARAMS.z_reference_mm, PARAMS.sentinel_frames)
        # The reference note marks the first sentinel of each mount and only that one.
        mount = mount_serial_of[id(mounted)] if index in mounted_of_position else 0
        assert c.notes["mount_reference"] == (mount not in referenced)
        referenced.add(mount)
    # No sentinel sits between captures of another target: its neighbors in the plan include its own target.
    for index, c in sentinels:
        around = [x.target_id for x in plan[max(index - 1, 0):index + 2] if x.procedure != PROCEDURE_SENTINEL]
        assert c.target_id in around
    targets_with_sentinels = {c.target_id for _, c in sentinels}
    assert targets_with_sentinels >= {TARGET_NOISE_PLATE, TARGET_DISKS, TARGET_CUTOUTS}
    # T2 sentinels: before and after A (T2 still mounted at the end of A) and at the end of B-Z.
    t2_positions = [i for i, c in sentinels if c.target_id == TARGET_NOISE_PLATE]
    first_a = next(i for i, c in enumerate(plan) if c.procedure == PROCEDURE_NOISE)
    last_a = max(i for i, c in enumerate(plan) if c.procedure == PROCEDURE_NOISE)
    assert t2_positions[0] == first_a - 1 and t2_positions[1] == last_a + 1
    assert count_sentinel_remounts(plan) == 0 and len(captures) + len(sentinels) == len(plan)
    assert any("0 sentinel re-mounts" in note for note in diagnostics.notes)
    assert not any("re-mount" in warning for warning in diagnostics.warnings)
    # The check really detects a T2 sentinel inside another series.
    broken = [dataclasses.replace(c, target_id=TARGET_NOISE_PLATE, gap_mm=None) if c.procedure == PROCEDURE_SENTINEL
              and c.target_id != TARGET_NOISE_PLATE else c for c in plan]
    assert count_sentinel_remounts(broken) > 0


def test_jitter_offsets_are_uniform_within_the_span_and_logged(full_plan):
    """Sections 6.1, 7 and 8: every random lateral offset lies within +/- PHASE_JITTER_SPAN_PX / 2 (in mm at the station
    depth), comes from its logged seed, and is added to H and V of the target pose."""
    plan, _ = full_plan
    jittered = [c for c in plan if c.procedure in (PROCEDURE_EDGES, PROCEDURE_AREA, PROCEDURE_DETECTION)
                and c.subseries != "nominal"]
    assert len(jittered) > 5000
    for c in jittered:
        half = PARAMS.phase_jitter_span_mm(GEOMETRY, c.station_z_mm) / 2.0
        assert abs(c.offset_h_mm) <= half + JITTER_TOLERANCE_MM and abs(c.offset_v_mm) <= half + JITTER_TOLERANCE_MM
        assert (c.offset_h_mm, c.offset_v_mm) == jitter_offset_mm(c.seed, PARAMS, GEOMETRY, c.station_z_mm)
        base_h, base_v = c.notes.get("field_h_mm", 0.0), c.notes.get("field_v_mm", 0.0)
        assert c.target_to_camera.translation[0] == pytest.approx(base_h + c.offset_h_mm, abs=1e-9)
        assert c.target_to_camera.translation[1] == pytest.approx(base_v + c.offset_v_mm, abs=1e-9)
    offsets = np.array([[c.offset_h_mm / PARAMS.phase_jitter_span_mm(GEOMETRY, c.station_z_mm),
                         c.offset_v_mm / PARAMS.phase_jitter_span_mm(GEOMETRY, c.station_z_mm)] for c in jittered])
    assert offsets.min() < -0.45 and offsets.max() > 0.45          # the whole span is used
    assert abs(offsets.mean()) < 0.02                              # and the draw is centered
    seeds = [c.seed for c in jittered]
    assert len(set(seeds)) > 0.99 * len(seeds)                     # every pose has its own draw


def test_series_counts_follow_the_procedure(full_plan):
    """Sections 6.1, 6.2, 7 and 8 and redesign note Sections 1 and 2: pose counts per configuration. B uses the five
    shape stations, C and D all nine ladder stations of the two plates T4 and T5, D has 300 poses per configuration at
    the three farthest stations and 60 elsewhere."""
    plan, _ = full_plan

    def count(procedure, **match):
        """Number of planned poses of a series whose attributes equal ``match``."""
        return sum(1 for c in plan if c.procedure == procedure
                   and all(getattr(c, k) == v for k, v in match.items()))

    n_shape = len(PARAMS.z_shape_stations_mm())
    n_all = len(PARAMS.z_stations_mm())
    assert (n_shape, n_all) == (5, 9)
    assert count(PROCEDURE_EDGES) == 2 * 2 * n_shape * (1 + PARAMS.phase_jitter_poses_edge)
    assert count(PROCEDURE_EDGES, subseries="nominal") == 2 * 2 * n_shape
    assert {c.station_z_mm for c in plan if c.procedure == PROCEDURE_EDGES} == set(PARAMS.z_shape_stations_mm())
    assert count(PROCEDURE_AREA, subseries="jitter") == 2 * 2 * n_all * PARAMS.phase_jitter_poses_area
    assert count(PROCEDURE_AREA, subseries=SUBSERIES_FIELD) == 2 * 4 * PARAMS.field_subseries_poses_area
    assert count(PROCEDURE_AREA, target_id="T4", gap_mm=15.0, station_z_mm=800.0, subseries=SUBSERIES_JITTER) == 30
    assert {c.station_z_mm for c in plan if c.procedure == PROCEDURE_AREA} == set(PARAMS.z_stations_mm())
    assert {c.station_z_mm for c in plan if c.procedure == PROCEDURE_AREA and c.subseries == SUBSERIES_FIELD} \
        == {PARAMS.z_reference_mm}
    assert count(PROCEDURE_DETECTION, subseries="jitter") == 2 * 2 * n_all * PARAMS.detection_trials_per_level
    # The extended trials: DETECTION_ZERO_TRIALS in all at the three farthest stations, DETECTION_TRIALS_PER_LEVEL elsewhere.
    zero = PARAMS.detection_zero_stations_mm()
    assert zero == (1131.0, 1345.0, 1600.0)
    for z in PARAMS.z_stations_mm():
        expected = PARAMS.detection_zero_trials if z in zero else PARAMS.detection_trials_per_level
        assert count(PROCEDURE_DETECTION, target_id="T5", gap_mm=60.0, station_z_mm=z) == expected
    assert all(c.frames == PARAMS.frames_per_detection_trial and c.level_index is None
               for c in plan if c.procedure == PROCEDURE_DETECTION)
    # One disk plate and one cutout plate, both gaps; no pilot, level selection or continuous-angle sub-series remains.
    assert {c.target_id for c in plan if c.procedure == PROCEDURE_DETECTION} == {"T4", "T5"}
    assert {c.subseries for c in plan if c.procedure == PROCEDURE_DETECTION} == {"jitter", "extended"}


def test_zstep_ladder_alternates_with_the_right_displacement():
    """Section 6.2 Step 2: for each delta, Z_STEP_REPEATS cycles of A (Z0), B (Z0 + delta), alternating; step_mm is the
    delta for both visits; the displacement is 0 for A and delta for B. Step 3: the staircase from Z0 to Z0 + 3 dZ_q in
    steps of dZ_q / Z_STAIRCASE_SUBDIVISION."""
    plan = plan_zstep_series(PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED))
    ladder = [c for c in plan if c.subseries == "ladder"]
    assert len(ladder) == len(PARAMS.z_reduced_stations_mm()) * len(PARAMS.z_step_ladder_mm) * PARAMS.z_step_repeats * 2
    for z0 in PARAMS.z_reduced_stations_mm():
        for delta in PARAMS.z_step_ladder_mm:
            visits = [c for c in ladder if c.station_z_mm == z0 and c.step_mm == delta]
            assert [c.visit for c in visits] == [VISIT_A, VISIT_B] * PARAMS.z_step_repeats
            for c in visits:
                expected = 0.0 if c.visit == VISIT_A else delta
                assert c.notes["displacement_mm"] == expected
                assert c.target_to_camera.translation[2] == pytest.approx(z0 + expected)
                assert c.frames == PARAMS.frames_per_zstep_pose and c.field == 0
    staircase = [c for c in plan if c.subseries == "staircase"]
    steps = int(round(PARAMS.z_staircase_quanta * PARAMS.z_staircase_subdivision))
    assert len(staircase) == len(PARAMS.z_reduced_stations_mm()) * (steps + 1)
    for z0 in PARAMS.z_reduced_stations_mm():
        quantum = GEOMETRY.depth_quantum_mm(TIER_A_DISPARITY_QUANTUM_PX, z0)
        sweep = [c for c in staircase if c.station_z_mm == z0]
        displacement = np.array([c.step_mm for c in sweep])
        assert displacement[0] == 0.0 and displacement[-1] == pytest.approx(PARAMS.z_staircase_quanta * quantum)
        assert np.allclose(np.diff(displacement), quantum / PARAMS.z_staircase_subdivision)
        assert all(c.frames == PARAMS.z_staircase_frames for c in sweep)
    # A measured quantum replaces the Tier-A one.
    fixed = plan_zstep_series(PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED), expected_quantum_mm=2.0)
    last = [c for c in fixed if c.subseries == "staircase" and c.station_z_mm == PARAMS.z_min_mm][-1]
    assert last.step_mm == pytest.approx(6.0)


def test_zstep_visits_are_approached_from_below():
    """Section 8, series Z: every visit of the ladder and of the staircase carries the note that it is approached from
    below, with the overshoot of ``z_step_approach_overshoot_mm``, and plan_summary.txt says so."""
    plan = plan_zstep_series(SMALL_PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED))
    assert plan and all(c.notes["approach"] == APPROACH_FROM_BELOW == "from below" for c in plan)
    assert all(c.notes["approach_overshoot_mm"] == SMALL_PARAMS.z_step_approach_overshoot_mm == 2.0 for c in plan)
    from sensorperf.acquisition.plan import plan_summary_text
    text = plan_summary_text(plan, SMALL_PARAMS, GEOMETRY)
    assert "approach: every visit from below" in text and "at least 2 decimals" in text


def test_filters_off_repeat_is_outside_the_main_budget(full_plan):
    """Section 4, Step 4.2: the filters-off repeat (A, B-HV, B-Z) is listed on its own and leaves every series row of
    the main budget unchanged (the drift sentinels follow the plan's total duration, so their row is not compared)."""
    from sensorperf.acquisition.plan import filters_off_budget, plan_summary_text
    base_plan, _ = full_plan
    plan = plan_full_session(PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED), filters_off=True)
    args = (GEOMETRY.frame_rate_hz, PARAMS.move_and_settle_time_s)
    main = {r.procedure: r for r in capture_budget(plan, *args) if r.procedure != "S"}
    reference = {r.procedure: r for r in capture_budget(base_plan, *args) if r.procedure != "S"}
    assert main.keys() == reference.keys()
    assert all((main[k].poses, main[k].frames) == (reference[k].poses, reference[k].frames) for k in main)
    assert {r.procedure for r in filters_off_budget(plan, *args)} == {"A", "B", "Z"}
    assert filters_off_budget(base_plan, *args) == []
    text = plan_summary_text(plan, PARAMS, GEOMETRY)
    assert "Outside the main budget" in text and "outside the main budget" not in plan_summary_text(base_plan, PARAMS, GEOMETRY).lower()


def test_budget_totals_are_the_stored_estimate(full_plan, capsys):
    """Section 9 and redesign note Section 6: poses, frames and robot hours per series ("10 frames/s and 3 s per move
    plus settle"), recomputed from the parameters. The stored DOCUMENT_ESTIMATE_* constants are the totals of the default
    plan (so plan_summary.txt compares a plan with them), and the per-series structure follows from the parameters."""
    plan, _ = full_plan
    rows = capture_budget(plan, GEOMETRY.frame_rate_hz, PARAMS.move_and_settle_time_s)
    total = budget_total(rows)
    print()
    print(format_budget_table(rows))
    print(f"stored estimate: {DOCUMENT_ESTIMATE_POSES} poses, {DOCUMENT_ESTIMATE_FRAMES} frames, "
          f"{DOCUMENT_ESTIMATE_HOURS} h; plan: {total.poses} poses, {total.frames} frames, {total.robot_hours:.2f} h")
    assert (total.poses, total.frames) == (DOCUMENT_ESTIMATE_POSES, DOCUMENT_ESTIMATE_FRAMES)
    assert total.robot_hours == pytest.approx(DOCUMENT_ESTIMATE_HOURS, abs=HOURS_TOLERANCE)
    # The formula on the stored numbers.
    hours = (DOCUMENT_ESTIMATE_POSES * PARAMS.move_and_settle_time_s
             + DOCUMENT_ESTIMATE_FRAMES / GEOMETRY.frame_rate_hz) / 3600.0
    assert hours == pytest.approx(DOCUMENT_ESTIMATE_HOURS, abs=HOURS_TOLERANCE)
    # A hand-checkable plan: 2 poses of 10 frames and 1 pose of 30 frames at 10 frames/s and 3 s each.
    toy = [dataclasses.replace(plan[0], procedure="A", frames=10), dataclasses.replace(plan[0], procedure="A", frames=10),
           dataclasses.replace(plan[0], procedure="C", frames=30)]
    toy_rows = {r.procedure: r for r in capture_budget(toy, 10.0, 3.0)}
    assert toy_rows["A"].robot_hours == pytest.approx((2 * 3.0 + 20 / 10.0) / 3600.0)
    assert (toy_rows["C"].poses, toy_rows["C"].frames) == (1, 30)
    # Per-series structure from the parameters.
    by_series = {r.procedure: r for r in rows}
    assert (by_series["R"].poses, by_series["R"].frames) == (PARAMS.registration_poses,
                                                             PARAMS.registration_poses * PARAMS.frames_per_registration_pose)
    n_all = len(PARAMS.z_stations_mm())
    n_zero = PARAMS.detection_zero_station_count
    d_poses = 2 * 2 * ((n_all - n_zero) * PARAMS.detection_trials_per_level + n_zero * PARAMS.detection_zero_trials)
    assert by_series["D"].poses == d_poses
    c_poses = 2 * 2 * n_all * PARAMS.phase_jitter_poses_area + 2 * 4 * PARAMS.field_subseries_poses_area
    assert by_series["C"].poses == c_poses
    assert by_series["A"].poses == 47 + 2 * len(PARAMS.tilt_angles_deg) * 2 + 1


def test_write_plan_round_trip(tmp_path: Path):
    """Section 9: poses.csv, plan_summary.txt and plan.png are written; poses.csv reads back to the same plan, and with a
    registration it also holds the flange pose that realizes each target pose."""
    registration = random_registration()
    diagnostics = PlanDiagnostics(master_seed=5)
    plan = plan_full_session(SMALL_PARAMS, GEOMETRY, np.random.default_rng(5), registration=registration,
                             diagnostics=diagnostics, open_background=True)
    written = write_plan(tmp_path, plan, registration, SMALL_PARAMS, GEOMETRY, diagnostics)
    assert (tmp_path / PLAN_CSV_NAME).exists() and (tmp_path / PLAN_SUMMARY_NAME).exists()
    try:
        import matplotlib  # noqa: F401
        assert (tmp_path / PLAN_FIGURE_NAME).stat().st_size > 0 and tmp_path / PLAN_FIGURE_NAME in written
    except ModuleNotFoundError:
        pass
    loaded = read_plan_csv(tmp_path / PLAN_CSV_NAME)
    assert len(loaded) == len(plan)
    for a, b in zip(plan, loaded):
        assert a.pose_key() == b.pose_key() and a.order == b.order and a.frames == b.frames
        assert (a.subseries, a.seed, a.tilt_axis, a.visit, a.step_mm, a.level_index) == \
               (b.subseries, b.seed, b.tilt_axis, b.visit, b.step_mm, b.level_index)
        assert (a.offset_h_mm, a.offset_v_mm, a.tilt_deg) == (b.offset_h_mm, b.offset_v_mm, b.tilt_deg)
        assert a.notes == b.notes
        distance, angle = a.target_to_camera.difference_from(b.target_to_camera)
        assert distance < POSE_TOLERANCE and angle < POSE_TOLERANCE
    with (tmp_path / PLAN_CSV_NAME).open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert {"base_x_mm", "base_rz_deg", "r00", "r22", "quat_w", "quat_z"} <= set(rows[0])
    row = rows[len(rows) // 2]
    flange = RigidTransform(np.array([[float(row[f"r{i}{j}"]) for j in range(3)] for i in range(3)]),
                            [float(row[c]) for c in ("base_x_mm", "base_y_mm", "base_z_mm")])
    wanted = loaded[len(rows) // 2].target_to_camera
    distance, angle = registration.target_to_camera(flange).difference_from(wanted)
    assert distance < POSE_TOLERANCE and angle < POSE_TOLERANCE
    quaternion = np.array([float(row[c]) for c in ("quat_w", "quat_x", "quat_y", "quat_z")])
    assert np.allclose(Rotation.from_quat(np.roll(quaternion, -1)).as_matrix(), flange.rotation, atol=1e-9)
    summary = (tmp_path / PLAN_SUMMARY_NAME).read_text()
    assert "Robot time h" in summary and "Document estimate" in summary and "Field-offset adjustments" in summary
    assert "Warnings" in summary and "Master random seed: 5" in summary


def test_plan_stations_cli(tmp_path: Path, capsys):
    """The plan_stations command line writes the plan, targets.json and parameters.json, accepts a series subset, prints
    the nine-station ladder and the budget with the stored estimate, and reports a bad input with exit code 2."""
    registration = random_registration()
    registration.save(tmp_path / "registration.json")
    out = tmp_path / "plan"
    code = plan_cli.main(["--out", str(out), "--seed", "3", "--series", "A", "Z", "D", "--no-extended",
                          "--registration", str(tmp_path / "registration.json")])
    assert code == 0
    for name in (PLAN_CSV_NAME, PLAN_SUMMARY_NAME, "targets.json", PARAMETERS_FILE_NAME):
        assert (out / name).exists()
    plan = read_plan_csv(out / PLAN_CSV_NAME)
    assert {c.procedure for c in plan} == {"A", "Z", "D", "S"}
    assert sum(1 for c in plan if c.procedure == "D") == 2 * 2 * 9 * PARAMS.detection_trials_per_level
    captured = capsys.readouterr()
    assert "INDICATIVE" in captured.err
    assert "Station ladder (9 stations" in captured.out and "400, 476, 566, 673, 800, 951, 1131, 1345, 1600" in captured.out
    assert "Document estimate" in captured.out
    assert plan_cli.main(["--out", str(out), "--series", "Q"]) == 2
    with pytest.raises(SystemExit):                       # the pilot options of the former level selection are gone
        plan_cli.main(["--out", str(out), "--pilot-d50-mm", "1"])


# ---------------------------------------------------------------------------
# Pose log -> manifest (Section 9)
# ---------------------------------------------------------------------------
def write_empty_captures(root: Path, plan: list[PlannedCapture], series_dirs: bool = True) -> None:
    """Create empty capture files named per the Section 9 rule (the builder must not need to read them)."""
    from sensorperf.io.session import SERIES_DIRS
    for c in plan:
        folder = root / SERIES_DIRS[c.procedure] if series_dirs else root
        folder.mkdir(parents=True, exist_ok=True)
        for f in range(c.frames):
            (folder / c.file_name(f)).write_bytes(b"")


def quaternion_wxyz(rotation: np.ndarray) -> list[float]:
    """The unit quaternion (w, x, y, z) of a rotation matrix."""
    return [float(v) for v in np.roll(Rotation.from_matrix(rotation).as_quat(), 1)]


def per_frame_log(path: Path, plan: list[PlannedCapture], registration: Registration, noise_mm: float = 0.0) -> None:
    """A hand-made pose log, one row per frame, the robot pose read as quaternion_wxyz; the read-back pose is the
    commanded flange pose, shifted by ``noise_mm`` along x when asked."""
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["file", "x_mm", "y_mm", "z_mm", "rotation_type", "r1", "r2", "r3", "r4", "timestamp"])
        for c in plan:
            flange = registration.flange_to_base_for(c.target_to_camera)
            for f in range(c.frames):
                writer.writerow([c.file_name(f), *(flange.translation + [noise_mm, 0.0, 0.0]), "quaternion_wxyz",
                                 *quaternion_wxyz(flange.rotation), f"2026-10-05T10:00:{f:02d}"])


def test_build_manifest_from_a_per_frame_log(tmp_path: Path):
    """Section 9: with the read-back pose equal to the commanded one the manifest's target pose equals the planned pose;
    the plan supplies sub-series, seed, offsets, tilt, step and visit; the optional columns come from the log; the
    capture files stay unread (they are empty)."""
    registration = random_registration()
    plan = plan_zstep_series(SMALL_PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED))[:14]
    plan += plan_edge_series(SMALL_PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED))[:3]
    for position, capture in enumerate(plan):               # two series were concatenated: one acquisition order
        capture.order = position
    root = tmp_path / "session"
    write_empty_captures(root, plan)
    write_plan(tmp_path / "plan", plan, None, SMALL_PARAMS, GEOMETRY)
    per_frame_log(tmp_path / "pose_log.csv", plan, registration)
    records = build_manifest(tmp_path / "pose_log.csv", root, tmp_path / "plan" / PLAN_CSV_NAME, registration, "cfgA",
                             strict=True)
    assert len(records) == sum(c.frames for c in plan)
    by_key = {c.pose_key(): c for c in plan}
    for record in records:
        planned = by_key[record.pose_key()]
        distance, angle = record.target_pose_camera.difference_from(planned.target_to_camera)
        assert distance < 1e-6 and angle < 1e-6
        assert (record.subseries, record.seed, record.visit, record.step_mm, record.tilt_axis) == \
               (planned.subseries, planned.seed, planned.visit, planned.step_mm, planned.tilt_axis)
        assert (record.offset_h_mm, record.offset_v_mm) == (planned.offset_h_mm, planned.offset_v_mm)
        assert record.sensor_config_id == "cfgA" and record.timestamp.startswith("2026-10-05")
        assert record.path.name == planned.file_name(record.frame_index)
    assert [(r.pose_key(), r.frame_index) for r in records] == \
           [(c.pose_key(), f) for c in plan for f in range(c.frames)]
    # The manifest it writes loads back through io.manifest (the path of the whole toolchain).
    write_manifest_csv(root / "manifest.csv", records)
    from sensorperf.io.manifest import load_manifest
    assert len(load_manifest(root / "manifest.csv")) == len(records)


def test_manifest_carries_the_achieved_field_fraction(tmp_path: Path):
    """Section 5, Step 1: the achieved fraction of the requested field offset travels from the plan notes (poses.csv) through
    the pose-log manifest builder into the manifest column ``field_fraction_achieved`` (string metadata) and back through
    ``load_manifest``; poses without it (tilt) have no such column value, and Analysis A reads it as a number."""
    from sensorperf.analysis.noise import _metadata_float
    from sensorperf.io.manifest import load_manifest
    registration = random_registration()
    plan = [c for c in plan_noise_series(SMALL_PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED), with_sentinels=False)
            if c.subseries == "main"][:6] + [c for c in plan_noise_series(
                SMALL_PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED), with_sentinels=False) if c.subseries == "tilt"][:2]
    for position, capture in enumerate(plan):
        capture.order = position
    assert any(c.notes[FIELD_FRACTION_ACHIEVED_KEY] < 1.0 for c in plan if c.subseries == "main")
    root = tmp_path / "session"
    write_empty_captures(root, plan)
    write_plan(tmp_path / "plan", plan, None, SMALL_PARAMS, GEOMETRY)
    per_frame_log(tmp_path / "pose_log.csv", plan, registration)
    records = build_manifest(tmp_path / "pose_log.csv", root, tmp_path / "plan" / PLAN_CSV_NAME, registration, strict=True)
    write_manifest_csv(root / "manifest.csv", records)
    by_key = {c.pose_key(): c for c in plan}
    for record in load_manifest(root / "manifest.csv"):
        planned = by_key[record.pose_key()]
        if planned.subseries == "main":
            assert _metadata_float(record, FIELD_FRACTION_ACHIEVED_KEY) == planned.notes[FIELD_FRACTION_ACHIEVED_KEY]
        else:
            assert FIELD_FRACTION_ACHIEVED_KEY not in record.metadata
            assert np.isnan(_metadata_float(record, FIELD_FRACTION_ACHIEVED_KEY))


def test_build_manifest_warns_when_the_series_z_log_is_rounded(tmp_path: Path):
    """Section 11 (pose log): the read-back pose is the step truth of series Z, so a log whose z_mm values all have fewer
    than two decimals draws a warning that names the problem; the same log with full resolution does not."""
    registration = random_registration()
    plan = plan_zstep_series(SMALL_PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED))[:8]
    root = tmp_path / "session"
    write_empty_captures(root, plan)
    write_plan(tmp_path / "plan", plan, None, SMALL_PARAMS, GEOMETRY)
    per_frame_log(tmp_path / "full.csv", plan, registration)
    _, messages = build_manifest_with_report(tmp_path / "full.csv", root, tmp_path / "plan" / PLAN_CSV_NAME, registration)
    assert not any("pose log resolution" in w for w in messages.warnings)
    with (tmp_path / "full.csv").open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        row["z_mm"] = f"{float(row['z_mm']):.{MIN_POSE_LOG_DECIMALS - 1}f}"          # 0.1 mm rounding
    with (tmp_path / "rounded.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    _, messages = build_manifest_with_report(tmp_path / "rounded.csv", root, tmp_path / "plan" / PLAN_CSV_NAME, registration)
    assert any("pose log resolution" in w and "fewer than 2 decimals" in w for w in messages.warnings)


def test_build_manifest_from_a_per_pose_log_and_unmatched_items(tmp_path: Path):
    """Section 9: a per-pose log (identity columns and frames) works too; unmatched files and plan rows are listed by
    name as warnings, and --strict turns them into errors."""
    registration = random_registration()
    plan = plan_zstep_series(SMALL_PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED))[:6]
    root = tmp_path / "session"
    write_empty_captures(root, plan[:5])                                   # the sixth pose has no files
    stray = root / "A_T2_G0_Z0500_F0_P077_f00.mc"                          # a file nobody planned or logged
    stray.write_bytes(b"")
    write_plan(tmp_path / "plan", plan, None, SMALL_PARAMS, GEOMETRY)
    with (tmp_path / "pose_log.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["procedure", "target_id", "gap_mm", "station_z_mm", "field", "pose_index", "frames", "x_mm",
                         "y_mm", "z_mm", "rotation_type", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9"])
        for c in plan:
            flange = registration.flange_to_base_for(c.target_to_camera)
            writer.writerow([c.procedure, c.target_id, "" if c.gap_mm is None else c.gap_mm, c.station_z_mm, c.field,
                             c.pose_index, c.frames, *flange.translation, "matrix", *flange.rotation.reshape(-1)])
    records, messages = build_manifest_with_report(tmp_path / "pose_log.csv", root, tmp_path / "plan" / PLAN_CSV_NAME,
                                                   registration, "cfg")
    assert not messages.errors
    assert len(records) == sum(c.frames for c in plan[:5])
    joined = " ".join(messages.warnings)
    assert "A_T2_G0_Z0500_F0_P077_f00.mc" in joined                        # the stray file, by name
    assert plan[5].file_name(0)[:-len("_f00.mc")] in joined                # the planned pose without files, by name
    _, strict = build_manifest_with_report(tmp_path / "pose_log.csv", root, tmp_path / "plan" / PLAN_CSV_NAME,
                                           registration, "cfg", strict=True)
    assert strict.errors and not strict.warnings
    with pytest.raises(PoseLogError):
        build_manifest(tmp_path / "pose_log.csv", root, tmp_path / "plan" / PLAN_CSV_NAME, registration, strict=True)
    # The command line: exit code 0 with warnings, 2 with --strict, and the manifest it writes loads.
    plan_csv, log = str(tmp_path / "plan" / PLAN_CSV_NAME), str(tmp_path / "pose_log.csv")
    registration.save(tmp_path / "registration.json")
    out = tmp_path / "manifest.csv"
    common = ["--pose-log", log, "--captures", str(root), "--plan", plan_csv, "--registration",
              str(tmp_path / "registration.json"), "--out", str(out)]
    assert manifest_cli.main(common) == 0 and out.exists()
    out.unlink()
    assert manifest_cli.main(common + ["--strict"]) == 2 and not out.exists()


def test_pose_log_rotation_conventions_and_poses_csv_as_log(tmp_path: Path):
    """Section 9 (rotation conventions of the calibration repository's make_manifest): every rotation_type reads the
    same rotation; a poses.csv written with a registration is accepted as a log in which the robot reached the
    commanded pose; a read-back far from the plan is reported."""
    from sensorperf.acquisition.pose_log import Messages, rotation_from_values
    rotation = Rotation.from_euler("ZYX", [10.0, -20.0, 30.0], degrees=True)
    matrix = rotation.as_matrix()
    messages = Messages()
    q_xyzw = rotation.as_quat()
    cases = {"matrix": list(matrix.reshape(-1)), "rotvec_deg": list(np.degrees(rotation.as_rotvec())),
             "quaternion_xyzw": list(q_xyzw), "quaternion_wxyz": list(np.roll(q_xyzw, 1)),
             "euler_zyx_deg": [10.0, -20.0, 30.0],
             "euler_xyz_deg": list(rotation.as_euler("XYZ", degrees=True)),
             "fixed_xyz_deg": list(rotation.as_euler("xyz", degrees=True))}
    for rotation_type, values in cases.items():
        assert np.allclose(rotation_from_values(rotation_type, values, "test", messages), matrix, atol=1e-9)
    assert np.allclose(rotation_from_values("none", [], "test", messages), np.eye(3))
    assert not messages.errors and not messages.warnings
    rotation_from_values("quaternion_xyzw", [0.0, 0.0, 0.0, 2.0], "test", messages)     # not unit: normalized, warned
    assert messages.warnings and not messages.errors
    # poses.csv with flange poses as the pose log.
    registration = random_registration()
    plan = plan_edge_series(SMALL_PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED))[:3]
    root = tmp_path / "s"
    write_empty_captures(root, plan)
    write_plan(tmp_path / "plan", plan, registration, SMALL_PARAMS, GEOMETRY)
    records, report = build_manifest_with_report(tmp_path / "plan" / PLAN_CSV_NAME, root,
                                                 tmp_path / "plan" / PLAN_CSV_NAME, registration, "c", strict=True)
    assert not report.errors and len(records) == sum(c.frames for c in plan)
    # Shift the log by 3 mm: the poses are flagged as far from the plan.
    per_frame_log(tmp_path / "shifted.csv", plan, registration, noise_mm=3.0)
    _, shifted = build_manifest_with_report(tmp_path / "shifted.csv", root, tmp_path / "plan" / PLAN_CSV_NAME,
                                            registration, "c")
    assert any("farther than" in w for w in shifted.warnings)


# ---------------------------------------------------------------------------
# Registration command line (Section 4, Steps 6 and 7)
# ---------------------------------------------------------------------------
OBSERVATION_POSES = 24
OBSERVATION_NOISE_MM = 0.05
OBSERVATION_NOISE_DEG = 0.02
EXACT_RESIDUAL_MM = 1.0e-6
NOISY_RESIDUAL_LIMIT_MM = 0.15
HEAVY_NOISE_MM = 2.0


def synthetic_observations(path: Path, rng: np.random.Generator, noise_mm: float, noise_deg: float,
                           columns: str = "pose") -> tuple[RigidTransform, RigidTransform, list[RigidTransform]]:
    """Write registration observations for a known X (target -> flange) and Y (camera -> base); returns X, Y and the
    noise-free target poses. ``columns`` is "pose" (tx_mm..) or "plane" (nx..distance_mm)."""
    x_true = RigidTransform.from_rotation_vector_degrees([1.0, 2.0, 3.0], [10.0, 20.0, 150.0])
    y_true = RigidTransform.from_rotation_vector_degrees([170.0, 10.0, -20.0], [800.0, -100.0, 600.0])
    flange = [RigidTransform.from_rotation_vector_degrees(rng.normal(0.0, 25.0, 3),
                                                          rng.normal(0.0, 200.0, 3) + [800.0, 0.0, 500.0])
              for _ in range(OBSERVATION_POSES)]
    exact = [y_true.inverse().compose(a).compose(x_true) for a in flange]
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        extra = ["tx_mm", "ty_mm", "tz_mm", "trx_deg", "try_deg", "trz_deg"] if columns == "pose" \
            else ["nx", "ny", "nz", "distance_mm"]
        writer.writerow(["pose_id", "x_mm", "y_mm", "z_mm", "rotation_type"] + [f"r{i}" for i in range(1, 10)] + extra)
        for i, (a, b) in enumerate(zip(flange, exact)):
            noisy = RigidTransform(Rotation.from_rotvec(np.radians(rng.normal(0.0, noise_deg, 3))).as_matrix() @ b.rotation,
                                   b.translation + rng.normal(0.0, noise_mm, 3))
            if columns == "pose":
                observed = [repr(float(v)) for v in list(noisy.translation) + list(noisy.rotation_vector_degrees())]
            else:
                plane = plane_of_pose(noisy)
                observed = [repr(float(v)) for v in list(plane.normal) + [plane.distance_mm]]
            writer.writerow([f"p{i:02d}", *(repr(float(v)) for v in a.translation), "matrix",
                             *(repr(float(v)) for v in a.rotation.reshape(-1)), *observed])
    return x_true, y_true, exact


def test_register_cli_round_trip_exact_and_noisy(tmp_path: Path, capsys):
    """Section 4 Steps 6-7: the register command line solves the hand-eye problem from exact observations (residual ~0,
    accepted, the truth recovered) and from noisy ones (residual of the order of the noise, accepted below
    REGISTRATION_RESIDUAL_ACCEPT_MM); heavy noise is not accepted (exit code 1); bad input gives exit code 2."""
    rng = np.random.default_rng(11)
    x_true, y_true, exact = synthetic_observations(tmp_path / "exact.csv", rng, 0.0, 0.0)
    out = tmp_path / "registration.json"
    assert register_cli.main(["--observations", str(tmp_path / "exact.csv"), "--method", "fiducial",
                              "--out", str(out)]) == 0
    solved = Registration.load(out)
    assert solved.residual_rms_mm < EXACT_RESIDUAL_MM and solved.accepted and solved.pose_count == OBSERVATION_POSES
    assert solved.camera_to_base.difference_from(y_true)[0] < 1e-5
    assert solved.target_to_flange.difference_from(x_true)[0] < 1e-5
    assert "ACCEPTED" in capsys.readouterr().out
    # Noisy: the residual is the noise level (about sqrt(3) x 0.05 mm) and the registration is still good.
    synthetic_observations(tmp_path / "noisy.csv", np.random.default_rng(12), OBSERVATION_NOISE_MM, OBSERVATION_NOISE_DEG)
    assert register_cli.main(["--observations", str(tmp_path / "noisy.csv"), "--method", "fiducial",
                              "--out", str(out)]) == 0
    noisy = Registration.load(out)
    assert 0.5 * OBSERVATION_NOISE_MM < noisy.residual_rms_mm < NOISY_RESIDUAL_LIMIT_MM
    assert noisy.camera_to_base.difference_from(y_true)[0] < NOISY_RESIDUAL_LIMIT_MM
    # The same noisy observations with a tighter acceptance limit are solved but not accepted.
    assert register_cli.main(["--observations", str(tmp_path / "noisy.csv"), "--method", "fiducial", "--out",
                              str(out), "--accept-mm", "0.001"]) == 1
    assert Registration.load(out).accepted is False
    synthetic_observations(tmp_path / "heavy.csv", np.random.default_rng(13), HEAVY_NOISE_MM, 0.0)
    assert register_cli.main(["--observations", str(tmp_path / "heavy.csv"), "--method", "fiducial",
                              "--out", str(out)]) == 1
    # Depth planes only (Step 6, second bullet; the default since T1 is gone): the plane residual of exact observations
    # is zero, with or without naming the method.
    synthetic_observations(tmp_path / "planes.csv", np.random.default_rng(14), 0.0, 0.0, columns="plane")
    assert register_cli.main(["--observations", str(tmp_path / "planes.csv"), "--method", "planes", "--out",
                              str(out)]) == 0
    assert Registration.load(out).residual_rms_mm < 1.0e-4
    assert register_cli.main(["--observations", str(tmp_path / "planes.csv"), "--out", str(out)]) == 0
    assert Registration.load(out).method == "depth_planes"
    # Input errors.
    assert register_cli.main(["--observations", str(tmp_path / "missing.csv")]) == 2
    assert register_cli.main(["--observations", str(tmp_path / "planes.csv"), "--method", "fiducial"]) == 2


# ---------------------------------------------------------------------------
# Quick-look check and the D pilot post check (Section 8 Step 1; Section 13 Step 2)
# ---------------------------------------------------------------------------
def scaled_geometry() -> SensorGeometry:
    """The indicative geometry at 160 x 120 px: intrinsics divided by SCALE_DOWN, same field of view."""
    g = GEOMETRY
    return dataclasses.replace(
        g, sensor_fx_px=g.sensor_fx_px / SCALE_DOWN, sensor_fy_px=g.sensor_fy_px / SCALE_DOWN,
        sensor_cx_px=g.sensor_cx_px / SCALE_DOWN, sensor_cy_px=g.sensor_cy_px / SCALE_DOWN,
        image_width_px=int(g.image_width_px / SCALE_DOWN), image_height_px=int(g.image_height_px / SCALE_DOWN))


def write_depth_frame(path: Path, geometry: SensorGeometry, depth: np.ndarray) -> None:
    """Write a depth image (NaN = no read) as a ``.mc`` file: header fx, fy, cx, cy and the float32 XYZ matrix, with
    zeros where nothing was read (the sensor's no-read sentinel)."""
    camera = camera_of(geometry)
    u, v = camera.pixel_grid()
    xyz = np.nan_to_num(camera.back_project(u, v, depth), nan=0.0).astype(np.float32)
    header = {"fx": geometry.sensor_fx_px, "fy": geometry.sensor_fy_px, "cx": geometry.sensor_cx_px,
              "cy": geometry.sensor_cy_px, "h": int(depth.shape[1]), "v": int(depth.shape[0])}
    write_matcloud(path, header, {"XYZ": xyz})


def plane_session(tmp_path: Path, plane_offset_mm: float = 0.0, plane_tilt_deg: float = 0.0,
                  read_nothing: bool = False) -> Session:
    """A session of one pose: T2 fronto-parallel at Z_REFERENCE_MM (registered), and a flat plane read over the whole image at
    Z_REFERENCE_MM + plane_offset_mm, tilted about the image V axis by plane_tilt_deg."""
    geometry = scaled_geometry()
    camera = camera_of(geometry)
    normal = np.array([np.sin(np.radians(plane_tilt_deg)), 0.0, -np.cos(np.radians(plane_tilt_deg))])
    depth = plane_depth_image(camera, [0.0, 0.0, CHECK_STATION_MM + plane_offset_mm], normal)
    if read_nothing:
        depth = np.full_like(depth, np.nan)
    pose = fronto_parallel_pose(0.0, 0.0, CHECK_STATION_MM)
    records = []
    for frame in range(CHECK_FRAMES):
        path = tmp_path / f"A_T2_G0_Z0800_F0_P000_f{frame:02d}.mc"
        write_depth_frame(path, geometry, depth)
        records.append(FrameRecord(path=path, procedure="A", target_id="T2", gap_mm=None, station_z_mm=CHECK_STATION_MM,
                                   field=0, pose_index=0, frame_index=frame, robot_pose=RigidTransform.identity(),
                                   target_pose_camera=pose))
    return Session(root=tmp_path, params=PARAMS, sensor=SensorConfig(config_id="test", geometry=geometry),
                   registration=None, targets=make_standard_target_set(PARAMS, GEOMETRY), records=records)


def test_check_session_passes_a_flat_plane_and_flags_a_displaced_one(tmp_path: Path):
    """Section 4 Step 8 / quick look: a flat plane at Z_REFERENCE_MM (160 x 120 px stack, intrinsics / 4) matches the registered
    T2 and flags nothing; the same plane displaced by 5 mm, tilted by 6 degrees, or not read at all is flagged."""
    (tmp_path / "ok").mkdir()
    ok = check_session(plane_session(tmp_path / "ok"))
    pose = ok.poses[0]
    assert not pose.flags and ok.verdict().startswith("VERDICT: all 1 poses passed")
    assert pose.frames == CHECK_FRAMES and pose.valid_fraction == pytest.approx(1.0)
    assert pose.front.pixels > 1000 and pose.front.rms_mm < 1e-3 and abs(pose.front.offset_mm) < 1e-3
    assert pose.front.angle_deg < 1e-3 and np.isnan(pose.back.rms_mm)         # T2 has no back plate
    (tmp_path / "displaced").mkdir()
    displaced = check_session(plane_session(tmp_path / "displaced", plane_offset_mm=DISPLACEMENT_MM))
    assert any(FLAG_FRONT_OFFSET in f for f in displaced.poses[0].flags)
    assert displaced.poses[0].front.offset_mm == pytest.approx(-DISPLACEMENT_MM, abs=1e-3)     # farther = negative
    (tmp_path / "tilted").mkdir()
    tilted = check_session(plane_session(tmp_path / "tilted", plane_tilt_deg=TILT_DEG))
    assert any(FLAG_FRONT_TILT in f for f in tilted.poses[0].flags)
    assert tilted.poses[0].front.angle_deg == pytest.approx(TILT_DEG, abs=0.1)
    (tmp_path / "blank").mkdir()
    blank = check_session(plane_session(tmp_path / "blank", read_nothing=True))
    assert any(FLAG_LOW_VALID in f for f in blank.poses[0].flags)
    assert displaced.to_dict()["n_flagged"] == 1 and "flags" in displaced.format_table().splitlines()[0]


def write_session_folder(root: Path, session: Session) -> None:
    """Write the files Session.load reads (sensor_config.json, targets.json, parameters.json, manifest.csv)."""
    session.sensor.save(root / SENSOR_CONFIG_FILE_NAME)
    session.targets.save(root / "targets.json")
    session.params.to_json(root / PARAMETERS_FILE_NAME)
    write_manifest_csv(root / "manifest.csv", session.records)


def test_check_captures_cli_exit_codes(tmp_path: Path, capsys):
    """check_captures: exit code 0 when nothing is flagged, 1 when a pose is flagged (JSON report written), 2 when the
    session cannot be read."""
    good = tmp_path / "good"
    good.mkdir()
    write_session_folder(good, plane_session(good))
    assert check_cli.main(["--session", str(good), "--out", str(good / "check.json")]) == 0
    report = json.loads((good / "check.json").read_text())
    assert report["n_flagged"] == 0 and report["poses"][0]["frames"] == CHECK_FRAMES
    bad = tmp_path / "bad"
    bad.mkdir()
    write_session_folder(bad, plane_session(bad, plane_offset_mm=DISPLACEMENT_MM))
    assert check_cli.main(["--session", str(bad), "--out", str(bad / "check.json")]) == 1
    assert json.loads((bad / "check.json").read_text())["n_flagged"] == 1
    assert "VERDICT: 1 of 1 poses flagged" in capsys.readouterr().out
    assert check_cli.main(["--session", str(tmp_path / "nowhere")]) == 2


def pilot_session(tmp_path: Path) -> Session:
    """Section 8 Step 1: ten C poses of the disk plate T4 at G = 15 mm and Z = Z_REFERENCE_MM with random lateral
    offsets, rendered noise-free at 160 x 120 px by ray casting the registered geometry (front disks at their depth, the
    back plate behind them, nothing elsewhere)."""
    geometry = scaled_geometry()
    camera = camera_of(geometry)
    targets = make_standard_target_set(PARAMS, GEOMETRY)
    target = targets.get(TARGET_DISKS).with_gap(PARAMS.gap_small_mm)
    records = []
    for index in range(PILOT_POSES):
        offset = jitter_offset_mm(100 + index, PARAMS, geometry, CHECK_STATION_MM)
        pose = fronto_parallel_pose(offset[0], offset[1], CHECK_STATION_MM)
        hit = target.intersect_rays(pose, np.zeros(3), camera.ray_directions())
        depth = hit.point_camera[..., 2]                      # NaN where no surface is hit
        path = tmp_path / f"C_T4_G15_Z0800_F0_P{index:03d}_f00.mc"
        write_depth_frame(path, geometry, depth)
        records.append(FrameRecord(path=path, procedure="C", target_id="T4", gap_mm=PARAMS.gap_small_mm,
                                   station_z_mm=CHECK_STATION_MM, field=0, pose_index=index, frame_index=0,
                                   robot_pose=RigidTransform.identity(), target_pose_camera=pose,
                                   subseries=SUBSERIES_JITTER))
    return Session(root=tmp_path, params=PARAMS, sensor=SensorConfig(config_id="test", geometry=geometry),
                   registration=None, targets=targets, records=records)


def test_pilot_post_check_finds_bare_posts_undetected(tmp_path: Path, capsys):
    """Section 8 Step 1 / Section 13 Step 2 and redesign note Section 2: the D pilot keeps only the post check. On a
    noise-free 160 x 120 px stack of the disk plate T4 the threshold set by the blank sites is zero, and the post-only
    site (0.5 mm, not rendered) is never detected. The pilot D_50 / D_0 level selection no longer exists. The quick-look
    check of the same stack flags nothing."""
    session = pilot_session(tmp_path)
    results = pilot_post_check(session, PARAMS, session.geometry)                  # default station: Z_REFERENCE_MM
    result = results[("T4", PARAMS.gap_small_mm)]
    assert result.station_z_mm == PARAMS.z_reference_mm
    assert result.poses == PILOT_POSES and result.tau_mm is not None and result.tau_mm < 1.0e-2
    assert result.blank_pixels > 0
    assert result.post_trials == PILOT_POSES * PARAMS.post_sites_per_plate and result.post_fraction == 0.0
    assert not hasattr(check_module, "interpolate_d50") and not hasattr(check_module, "pilot_detection_counts")
    assert results == pilot_post_check(session, PARAMS, session.geometry, CHECK_STATION_MM)           # deterministic
    assert pilot_post_check(session, PARAMS, session.geometry, 500.0) == {}
    assert pilot_post_check(session, PARAMS, session.geometry, CHECK_STATION_MM, subseries=("field",)) == {}
    assert not check_session(session).flagged
    # The command line prints the same numbers.
    write_session_folder(tmp_path, session)
    assert check_cli.main(["--session", str(tmp_path), "--pilot", "800"]) == 0
    assert "post-site detection fraction = 0.00" in capsys.readouterr().out
    assert check_cli.main(["--session", str(tmp_path), "--pilot", "500"]) == 2
    assert check_module.EXIT_FLAGGED == 1


def test_check_session_flags_an_unreadable_capture(tmp_path: Path):
    """Quick look: an empty or damaged capture file flags its pose instead of stopping the check."""
    session = plane_session(tmp_path)
    session.records[1].path.write_bytes(b"")
    session.records[2].path.write_bytes(b"not a capture file")
    report = check_session(session)
    assert report.poses[0].flags and report.poses[0].flags[0].startswith(check_module.FLAG_UNREADABLE)


def test_plan_png_is_skipped_with_a_warning_without_matplotlib(tmp_path: Path, monkeypatch, capsys):
    """Section 9 (plan.png): when matplotlib is missing, poses.csv and plan_summary.txt are still written and the
    missing figure is reported as a warning (on stderr and in the summary)."""
    import sys
    monkeypatch.setitem(sys.modules, "matplotlib", None)
    plan = plan_registration(PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED))
    diagnostics = PlanDiagnostics()
    written = write_plan(tmp_path, plan, None, PARAMS, GEOMETRY, diagnostics)
    assert not (tmp_path / PLAN_FIGURE_NAME).exists() and (tmp_path / PLAN_CSV_NAME) in written
    assert "matplotlib" in capsys.readouterr().err
    assert PLAN_FIGURE_NAME in (tmp_path / PLAN_SUMMARY_NAME).read_text()


def test_detection_plan_has_no_pilot_level_selection():
    """Section 8 and redesign note Section 2: the D plan takes no pilot D_50 / D_0 (the levels are the fixed (feature,
    station) pairs), and ``extended=False`` leaves DETECTION_TRIALS_PER_LEVEL poses at every configuration, all
    in sub-series "jitter"."""
    diagnostics = PlanDiagnostics()
    base = plan_detection_series(SMALL_PARAMS, GEOMETRY, np.random.default_rng(2), extended=False,
                                 diagnostics=diagnostics)
    assert len(base) == 2 * 2 * 9 * SMALL_PARAMS.detection_trials_per_level
    assert {c.subseries for c in base} == {"jitter"} and not any("pilot" in n for n in diagnostics.notes)
    with_zero = plan_detection_series(SMALL_PARAMS, GEOMETRY, np.random.default_rng(2))
    assert len(with_zero) == len(base) + 2 * 2 * SMALL_PARAMS.detection_zero_station_count * (
        SMALL_PARAMS.detection_zero_trials - SMALL_PARAMS.detection_trials_per_level)
    import inspect
    assert "pilot_d50_mm" not in inspect.signature(plan_detection_series).parameters
    assert "pilot_d50_mm" not in inspect.signature(plan_full_session).parameters


def test_registration_cli_defaults_to_the_plane_only_solve(tmp_path: Path):
    """Redesign note Section 3: with no pattern plate the command line registers by plane correspondence
    (solve_from_planes) unless told otherwise."""
    assert register_cli.build_parser().get_default("method") == register_cli.METHOD_PLANES
