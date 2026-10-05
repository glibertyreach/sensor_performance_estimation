"""
Tests of the acquisition-side tools: the station planner (Sections 4 to 9), the
pose log -> manifest builder (Section 9), the registration command line
(Section 4, Steps 6 and 7), and the quick-look check with the D pilot detection
counts (Section 8, Step 1; Section 13, Step 2).

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
    FLAG_FRONT_OFFSET, FLAG_FRONT_TILT, FLAG_LOW_VALID, check_session, interpolate_d50,
    pilot_detection_counts, zero_detection_diameter,
)
from sensorperf.acquisition.plan import (
    FILTERS_OFF_POSE_INDEX_BASE, PLAN_CSV_NAME, PLAN_FIGURE_NAME, PLAN_SUMMARY_NAME, PlanDiagnostics,
    PlannedCapture, SERIES_ORDER, TIER_A_DISPARITY_QUANTUM_PX, budget_total, capture_budget, capture_duration_s,
    camera_of, fit_violation_px, format_budget_table, insert_sentinels, jitter_offset_mm, place_in_field,
    plan_detection_series, plan_edge_series, plan_full_session, plan_noise_series, plan_registration,
    plan_zstep_series, read_plan_csv, write_plan,
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
    FrameRecord, SUBSERIES_FIELD, SUBSERIES_JITTER, VISIT_A, VISIT_B, write_manifest_csv,
)
from sensorperf.features.planes import plane_depth_image
from sensorperf.io.matcloud import write_matcloud
from sensorperf.io.session import PARAMETERS_FILE_NAME, SENSOR_CONFIG_FILE_NAME, Session, SensorConfig
from sensorperf.parameters import (
    CharacterizationParameters, FIELD_POSITION_CODES, PROCEDURE_AREA, PROCEDURE_DETECTION, PROCEDURE_EDGES,
    PROCEDURE_NOISE, PROCEDURE_REGISTRATION, PROCEDURE_SENTINEL, SensorGeometry, TARGET_DISKS_LARGE,
    TARGET_NOISE_PLATE,
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

DOCUMENT_POSES = 6710
DOCUMENT_FRAMES = 46910
DOCUMENT_HOURS = 6.9
"""The Section 9 estimate: 6,710 poses, 46,910 frames, 6.9 h."""
BUDGET_TOLERANCE = 0.03
"""Relative tolerance of the comparison with the document's totals, with the document's accounting of D (one array
per kind, see test_budget_matches_the_document). The document gives its totals to three or four digits as estimates;
the plan differs from the accounting that reproduces them by a few dozen poses (the staircase end points, the 0
degree tilt poses captured about both axes, the sentinels), under 1 percent, so 3 percent leaves a margin without
hiding a missing series."""
JITTER_TOLERANCE_MM = 1.0e-9
"""Numerical slack when comparing a logged offset with the half span."""
POSE_TOLERANCE = 1.0e-9
"""Agreement of poses that went through a CSV round trip or a registration composition (mm and degrees)."""

SCALE_DOWN = 4.0
"""Image size and intrinsics are divided by this for the synthetic check stacks (160 x 120 px)."""
CHECK_STATION_MM = 750.0
CHECK_FRAMES = 3
DISPLACEMENT_MM = 5.0
"""A plane displaced by this much from the registered one must be flagged (the thresholds are 3 mm)."""
TILT_DEG = 6.0
"""A plane tilted by this much from the registered one must be flagged (the threshold is 2 degrees)."""
PILOT_POSES = 10
"""C poses of the synthetic pilot test."""


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
    assert len(off) == 55 + 24 and all(c.pose_index >= FILTERS_OFF_POSE_INDEX_BASE for c in off)
    open_poses = [c for c in plan if c.subseries == "open"]
    assert len(open_poses) == 2 * PARAMS.phase_jitter_poses_area
    assert all(c.gap_mm is None and c.station_z_mm == PARAMS.z_reference_mm for c in open_poses)
    assert {c.target_id for c in open_poses} == {"T5-S", "T5-L"}


def test_plan_is_reproducible_from_the_master_seed():
    """Section 5 Step 2 (logged seed): the same master seed gives the same plan, another seed another order."""
    one = plan_full_session(SMALL_PARAMS, GEOMETRY, np.random.default_rng(5))
    again = plan_full_session(SMALL_PARAMS, GEOMETRY, np.random.default_rng(5))
    other = plan_full_session(SMALL_PARAMS, GEOMETRY, np.random.default_rng(6))
    assert [c.pose_key() for c in one] == [c.pose_key() for c in again]
    assert all(np.allclose(a.target_to_camera.as_matrix(), b.target_to_camera.as_matrix()) for a, b in zip(one, again))
    assert [c.pose_key() for c in one] != [c.pose_key() for c in other]


def test_registration_poses_span_the_volume():
    """Section 4 Step 6: REGISTRATION_POSES poses of REGISTRATION_FRAMES frames spanning Z_MIN to Z_MAX, with tilts
    within REGISTRATION_TILT_RANGE_DEG about H and V."""
    poses = plan_registration(PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED))
    assert len(poses) == PARAMS.registration_poses
    assert all(c.frames == PARAMS.frames_per_registration_pose and c.target_id == "T1" for c in poses)
    depths = [c.notes["depth_mm"] for c in poses]
    assert min(depths) == pytest.approx(PARAMS.z_min_mm) and max(depths) == pytest.approx(PARAMS.z_max_mm)
    for c in poses:
        assert abs(c.notes["tilt_h_deg"]) <= PARAMS.registration_tilt_range_deg
        assert abs(c.notes["tilt_v_deg"]) <= PARAMS.registration_tilt_range_deg
        assert c.target_to_camera.translation[2] == pytest.approx(c.notes["depth_mm"])


def test_noise_station_order_is_a_permutation_of_the_station_list(full_plan):
    """Section 5 Steps 1 and 2: the 11 Z stations x 5 field positions are all visited once, in the order of the
    logged seed; tilt sub-series (24 poses) and repeat-mount check follow."""
    plan, _ = full_plan
    main = [c for c in plan if c.procedure == PROCEDURE_NOISE and c.subseries == "main"]
    stations = [(z, code) for z in PARAMS.noise_stations_mm() for code in FIELD_POSITION_CODES]
    assert len(main) == len(stations) == 55
    assert sorted((c.station_z_mm, c.field) for c in main) == sorted(stations)
    assert [(c.station_z_mm, c.field) for c in main] != sorted(stations)
    seeds = {c.seed for c in main}
    assert len(seeds) == 1 and seeds == {c.notes["order_seed"] for c in main}
    permutation = np.random.default_rng(seeds.pop()).permutation(len(stations))
    assert [(c.station_z_mm, c.field) for c in main] == [stations[int(i)] for i in permutation]
    assert all(c.frames == PARAMS.frames_per_noise_station for c in main)
    tilt = [c for c in plan if c.procedure == PROCEDURE_NOISE and c.subseries == "tilt"]
    assert len(tilt) == len(PARAMS.z_reduced_stations_mm) * 2 * len(PARAMS.tilt_angles_deg)
    assert Counter((c.tilt_axis, c.tilt_deg) for c in tilt)[("V", 15.0)] == len(PARAMS.z_reduced_stations_mm)
    assert all(c.frames == PARAMS.frames_per_tilt_pose and c.field == 0 for c in tilt)
    remount = [c for c in plan if c.subseries == "remount"]
    assert len(remount) == 1 and remount[0].station_z_mm == PARAMS.z_reference_mm and remount[0].field == 0


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
    center = place_in_field(PARAMS, GEOMETRY, plate, 750.0, (0.0, 0.0), margin)
    assert (center.h_mm, center.v_mm, center.fraction_kept) == (0.0, 0.0, 1.0)
    # The adjustments are recorded for the summary.
    main = plan_noise_series(PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED), with_sentinels=False)
    adjusted = [c for c in main if "field_placement" in c.notes]
    assert adjusted and all(c.field != 0 for c in adjusted if c.notes["field_placement"]["fits"])


def test_sentinels_follow_the_budget_clock(full_plan):
    """Section 5 Step 3: a sentinel before the first station, one every DRIFT_SENTINEL_INTERVAL_MIN of estimated clock,
    one after the last; each is T2, center, Z_REFERENCE_MM, SENTINEL_FRAMES frames."""
    plan, _ = full_plan
    seconds_per_frame = 1.0 / GEOMETRY.frame_rate_hz
    interval = PARAMS.drift_sentinel_interval_min * 60.0
    non_registration = [c for c in plan if c.procedure != PROCEDURE_REGISTRATION]
    assert non_registration[0].procedure == PROCEDURE_SENTINEL and non_registration[-1].procedure == PROCEDURE_SENTINEL
    clock, starts = 0.0, []
    longest = 0.0
    for c in non_registration:
        duration = capture_duration_s(c, seconds_per_frame, PARAMS.move_and_settle_time_s)
        longest = max(longest, duration)
        if c.procedure == PROCEDURE_SENTINEL:
            starts.append(clock)
            assert (c.target_id, c.gap_mm, c.field, c.frames) == ("T2", None, 0, PARAMS.sentinel_frames)
            assert c.station_z_mm == PARAMS.z_reference_mm
        clock += duration
    gaps = np.diff(starts)
    assert len(starts) >= int(clock // interval) + 1                     # at least one per full interval, plus the last
    # Every gap but the last is at least one interval and overshoots it by at most the longest single capture.
    assert np.all(gaps[:-1] >= interval - 1e-6) and np.all(gaps[:-1] <= interval + longest + 1e-6)
    assert gaps[-1] <= interval + longest
    # Inserting again changes nothing: existing sentinels are replaced, not duplicated.
    again = insert_sentinels(plan, PARAMS, GEOMETRY, seconds_per_frame, PARAMS.move_and_settle_time_s)
    assert [c.pose_key() for c in again] == [c.pose_key() for c in plan]


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
    """Sections 6.1, 6.2, 7 and 8: pose counts per configuration."""
    plan, _ = full_plan

    def count(procedure, **match):
        """Number of planned poses of a series whose attributes equal ``match``."""
        return sum(1 for c in plan if c.procedure == procedure
                   and all(getattr(c, k) == v for k, v in match.items()))

    n_z = len(PARAMS.z_shape_stations_mm)
    assert count(PROCEDURE_EDGES) == 2 * 2 * n_z * (1 + PARAMS.phase_jitter_poses_edge)
    assert count(PROCEDURE_EDGES, subseries="nominal") == 2 * 2 * n_z
    assert count(PROCEDURE_AREA, subseries="jitter") == 4 * 2 * n_z * PARAMS.phase_jitter_poses_area
    assert count(PROCEDURE_AREA, subseries=SUBSERIES_FIELD) == 4 * 4 * PARAMS.field_subseries_poses_area
    assert count(PROCEDURE_AREA, target_id="T4-S", gap_mm=15.0, station_z_mm=750.0, subseries=SUBSERIES_JITTER) == 30
    assert count(PROCEDURE_DETECTION, subseries="jitter") == 4 * 2 * n_z * PARAMS.detection_trials_per_level
    # Section 8 Step 4: every configuration at the reduced stations has DETECTION_ZERO_TRIALS poses in all.
    for z in PARAMS.z_reduced_stations_mm:
        assert count(PROCEDURE_DETECTION, target_id="T5-L", gap_mm=60.0, station_z_mm=z) == PARAMS.detection_zero_trials
    assert count(PROCEDURE_DETECTION, target_id="T5-L", gap_mm=60.0, station_z_mm=625.0) == PARAMS.detection_trials_per_level
    assert all(c.frames == PARAMS.frames_per_detection_trial and c.level_index is None
               for c in plan if c.procedure == PROCEDURE_DETECTION)
    # Both the small and the large array of each kind are planned.
    assert {c.target_id for c in plan if c.procedure == PROCEDURE_DETECTION} == {"T4-S", "T4-L", "T5-S", "T5-L"}


def test_zstep_ladder_alternates_with_the_right_displacement():
    """Section 6.2 Step 2: for each delta, Z_STEP_REPEATS cycles of A (Z0), B (Z0 + delta), alternating; step_mm is the
    delta for both visits; the displacement is 0 for A and delta for B. Step 3: the staircase from Z0 to Z0 + 3 dZ_q in
    steps of dZ_q / Z_STAIRCASE_SUBDIVISION."""
    plan = plan_zstep_series(PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED))
    ladder = [c for c in plan if c.subseries == "ladder"]
    assert len(ladder) == len(PARAMS.z_reduced_stations_mm) * len(PARAMS.z_step_ladder_mm) * PARAMS.z_step_repeats * 2
    for z0 in PARAMS.z_reduced_stations_mm:
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
    assert len(staircase) == len(PARAMS.z_reduced_stations_mm) * (steps + 1)
    for z0 in PARAMS.z_reduced_stations_mm:
        quantum = GEOMETRY.depth_quantum_mm(TIER_A_DISPARITY_QUANTUM_PX, z0)
        sweep = [c for c in staircase if c.station_z_mm == z0]
        displacement = np.array([c.step_mm for c in sweep])
        assert displacement[0] == 0.0 and displacement[-1] == pytest.approx(PARAMS.z_staircase_quanta * quantum)
        assert np.allclose(np.diff(displacement), quantum / PARAMS.z_staircase_subdivision)
        assert all(c.frames == PARAMS.z_staircase_frames for c in sweep)
    # A measured quantum replaces the Tier-A one.
    fixed = plan_zstep_series(PARAMS, GEOMETRY, np.random.default_rng(MASTER_SEED), expected_quantum_mm=2.0)
    last = [c for c in fixed if c.subseries == "staircase" and c.station_z_mm == 500.0][-1]
    assert last.step_mm == pytest.approx(6.0)


def test_budget_matches_the_document(full_plan, capsys):
    """Section 9: poses, frames and robot hours per series ("10 frames/s and 3 s per move plus settle"). The formula
    reproduces the document's 6.9 h from its own 6,710 poses and 46,910 frames. The plan's totals are compared with the
    document's within BUDGET_TOLERANCE using the document's accounting of series D, which the 6,710 evidently
    counts as one plate per kind ({disk, cutout} x 2 gaps x 5 stations, Section 8 Step 3, plus the extended series);
    this plan plans both the small and the large array of each kind, so its D series is exactly twice as large."""
    plan, _ = full_plan
    rows = capture_budget(plan, GEOMETRY.frame_rate_hz, PARAMS.move_and_settle_time_s)
    total = budget_total(rows)
    print()
    print(format_budget_table(rows))
    print(f"document: {DOCUMENT_POSES} poses, {DOCUMENT_FRAMES} frames, {DOCUMENT_HOURS} h; plan: {total.poses} poses, "
          f"{total.frames} frames, {total.robot_hours:.2f} h")
    # The formula on the document's own numbers.
    hours = (DOCUMENT_POSES * PARAMS.move_and_settle_time_s + DOCUMENT_FRAMES / GEOMETRY.frame_rate_hz) / 3600.0
    assert hours == pytest.approx(DOCUMENT_HOURS, abs=0.05)
    # A hand-checkable plan: 2 poses of 10 frames and 1 pose of 30 frames at 10 frames/s and 3 s each.
    toy = [dataclasses.replace(plan[0], procedure="A", frames=10), dataclasses.replace(plan[0], procedure="A", frames=10),
           dataclasses.replace(plan[0], procedure="C", frames=30)]
    toy_rows = {r.procedure: r for r in capture_budget(toy, 10.0, 3.0)}
    assert toy_rows["A"].robot_hours == pytest.approx((2 * 3.0 + 20 / 10.0) / 3600.0)
    assert (toy_rows["C"].poses, toy_rows["C"].frames) == (1, 30)
    # Per-series structure.
    by_series = {r.procedure: r for r in rows}
    assert (by_series["R"].poses, by_series["R"].frames) == (PARAMS.registration_poses,
                                                             PARAMS.registration_poses * PARAMS.frames_per_registration_pose)
    d = by_series["D"]
    one_array_d_poses = 2 * 2 * (len(PARAMS.z_shape_stations_mm) - 3) * PARAMS.detection_trials_per_level \
        + 2 * 2 * 3 * PARAMS.detection_zero_trials
    assert d.poses == 2 * one_array_d_poses
    document_accounting_poses = total.poses - d.poses + one_array_d_poses
    document_accounting_frames = total.frames - d.frames + one_array_d_poses * PARAMS.frames_per_detection_trial
    document_accounting_hours = (document_accounting_poses * PARAMS.move_and_settle_time_s
                                 + document_accounting_frames / GEOMETRY.frame_rate_hz) / 3600.0
    print(f"document accounting (one plate per kind in D): {document_accounting_poses} poses "
          f"({document_accounting_poses / DOCUMENT_POSES:.3f} x), {document_accounting_frames} frames "
          f"({document_accounting_frames / DOCUMENT_FRAMES:.3f} x), {document_accounting_hours:.2f} h "
          f"({document_accounting_hours / DOCUMENT_HOURS:.3f} x)")
    assert document_accounting_poses == pytest.approx(DOCUMENT_POSES, rel=BUDGET_TOLERANCE)
    assert document_accounting_frames == pytest.approx(DOCUMENT_FRAMES, rel=BUDGET_TOLERANCE)
    assert document_accounting_hours == pytest.approx(DOCUMENT_HOURS, rel=BUDGET_TOLERANCE)
    assert 1.4 < total.poses / DOCUMENT_POSES < 1.8                 # the factor-two D series explains the rest


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
    """The plan_stations command line writes the plan, targets.json and parameters.json, accepts a series subset and
    pilot values, and reports a bad input with exit code 2."""
    registration = random_registration()
    registration.save(tmp_path / "registration.json")
    out = tmp_path / "plan"
    code = plan_cli.main(["--out", str(out), "--seed", "3", "--series", "A", "Z", "D", "--no-extended",
                          "--registration", str(tmp_path / "registration.json"), "--pilot-d50-mm", "T4-S@15=1.2",
                          "--pilot-d0-mm", "0.5"])
    assert code == 0
    for name in (PLAN_CSV_NAME, PLAN_SUMMARY_NAME, "targets.json", PARAMETERS_FILE_NAME):
        assert (out / name).exists()
    plan = read_plan_csv(out / PLAN_CSV_NAME)
    assert {c.procedure for c in plan} == {"A", "Z", "D", "S"}
    assert sum(1 for c in plan if c.procedure == "D") == 4 * 2 * 5 * PARAMS.detection_trials_per_level
    assert "INDICATIVE" in capsys.readouterr().err
    assert plan_cli.main(["--out", str(out), "--series", "Q"]) == 2
    assert plan_cli.main(["--out", str(out), "--pilot-d50-mm", "abc"]) == 2


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
        writer.writerow(["file", "x_mm", "y_mm", "z_mm", "rotation_type", "r1", "r2", "r3", "r4", "timestamp",
                         "indicator_mm"])
        for c in plan:
            flange = registration.flange_to_base_for(c.target_to_camera)
            for f in range(c.frames):
                writer.writerow([c.file_name(f), *(flange.translation + [noise_mm, 0.0, 0.0]), "quaternion_wxyz",
                                 *quaternion_wxyz(flange.rotation), f"2026-10-05T10:00:{f:02d}", 1.25 + f])


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
        assert record.indicator_mm == pytest.approx(1.25 + record.frame_index)
        assert record.path.name == planned.file_name(record.frame_index)
    assert [(r.pose_key(), r.frame_index) for r in records] == \
           [(c.pose_key(), f) for c in plan for f in range(c.frames)]
    # The manifest it writes loads back through io.manifest (the path of the whole toolchain).
    write_manifest_csv(root / "manifest.csv", records)
    from sensorperf.io.manifest import load_manifest
    assert len(load_manifest(root / "manifest.csv")) == len(records)


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
    assert register_cli.main(["--observations", str(tmp_path / "exact.csv"), "--out", str(out)]) == 0
    solved = Registration.load(out)
    assert solved.residual_rms_mm < EXACT_RESIDUAL_MM and solved.accepted and solved.pose_count == OBSERVATION_POSES
    assert solved.camera_to_base.difference_from(y_true)[0] < 1e-5
    assert solved.target_to_flange.difference_from(x_true)[0] < 1e-5
    assert "ACCEPTED" in capsys.readouterr().out
    # Noisy: the residual is the noise level (about sqrt(3) x 0.05 mm) and the registration is still good.
    synthetic_observations(tmp_path / "noisy.csv", np.random.default_rng(12), OBSERVATION_NOISE_MM, OBSERVATION_NOISE_DEG)
    assert register_cli.main(["--observations", str(tmp_path / "noisy.csv"), "--out", str(out)]) == 0
    noisy = Registration.load(out)
    assert 0.5 * OBSERVATION_NOISE_MM < noisy.residual_rms_mm < NOISY_RESIDUAL_LIMIT_MM
    assert noisy.camera_to_base.difference_from(y_true)[0] < NOISY_RESIDUAL_LIMIT_MM
    # The same noisy observations with a tighter acceptance limit are solved but not accepted.
    assert register_cli.main(["--observations", str(tmp_path / "noisy.csv"), "--out", str(out), "--accept-mm",
                              "0.001"]) == 1
    assert Registration.load(out).accepted is False
    synthetic_observations(tmp_path / "heavy.csv", np.random.default_rng(13), HEAVY_NOISE_MM, 0.0)
    assert register_cli.main(["--observations", str(tmp_path / "heavy.csv"), "--out", str(out)]) == 1
    # Depth planes only (Step 6, second bullet): the plane residual of exact observations is zero.
    synthetic_observations(tmp_path / "planes.csv", np.random.default_rng(14), 0.0, 0.0, columns="plane")
    assert register_cli.main(["--observations", str(tmp_path / "planes.csv"), "--method", "planes", "--out",
                              str(out)]) == 0
    assert Registration.load(out).residual_rms_mm < 1.0e-4
    # Input errors.
    assert register_cli.main(["--observations", str(tmp_path / "missing.csv")]) == 2
    assert register_cli.main(["--observations", str(tmp_path / "planes.csv"), "--method", "fiducial"]) == 2


# ---------------------------------------------------------------------------
# Quick-look check and pilot counts (Section 8 Step 1; Section 13 Step 2)
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
    """A session of one pose: T2 fronto-parallel at 750 mm (registered), and a flat plane read over the whole image at
    750 mm + plane_offset_mm, tilted about the image V axis by plane_tilt_deg."""
    geometry = scaled_geometry()
    camera = camera_of(geometry)
    normal = np.array([np.sin(np.radians(plane_tilt_deg)), 0.0, -np.cos(np.radians(plane_tilt_deg))])
    depth = plane_depth_image(camera, [0.0, 0.0, CHECK_STATION_MM + plane_offset_mm], normal)
    if read_nothing:
        depth = np.full_like(depth, np.nan)
    pose = fronto_parallel_pose(0.0, 0.0, CHECK_STATION_MM)
    records = []
    for frame in range(CHECK_FRAMES):
        path = tmp_path / f"A_T2_G0_Z0750_F0_P000_f{frame:02d}.mc"
        write_depth_frame(path, geometry, depth)
        records.append(FrameRecord(path=path, procedure="A", target_id="T2", gap_mm=None, station_z_mm=CHECK_STATION_MM,
                                   field=0, pose_index=0, frame_index=frame, robot_pose=RigidTransform.identity(),
                                   target_pose_camera=pose))
    return Session(root=tmp_path, params=PARAMS, sensor=SensorConfig(config_id="test", geometry=geometry),
                   registration=None, targets=make_standard_target_set(PARAMS, GEOMETRY), records=records)


def test_check_session_passes_a_flat_plane_and_flags_a_displaced_one(tmp_path: Path):
    """Section 4 Step 8 / quick look: a flat plane at 750 mm (160 x 120 px stack, intrinsics / 4) matches the registered
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
    """Section 8 Step 1: ten C poses of the large disk array T4-L at G = 15 mm and Z = 750 mm with random lateral
    offsets, rendered noise-free at 160 x 120 px by ray casting the registered geometry (front disks at their depth, the
    back plate behind them, nothing elsewhere)."""
    geometry = scaled_geometry()
    camera = camera_of(geometry)
    targets = make_standard_target_set(PARAMS, GEOMETRY)
    target = targets.get(TARGET_DISKS_LARGE).with_gap(PARAMS.gap_small_mm)
    records = []
    for index in range(PILOT_POSES):
        offset = jitter_offset_mm(100 + index, PARAMS, geometry, CHECK_STATION_MM)
        pose = fronto_parallel_pose(offset[0], offset[1], CHECK_STATION_MM)
        hit = target.intersect_rays(pose, np.zeros(3), camera.ray_directions())
        depth = hit.point_camera[..., 2]                      # NaN where no surface is hit
        path = tmp_path / f"C_T4L_G15_Z0750_F0_P{index:03d}_f00.mc"
        write_depth_frame(path, geometry, depth)
        records.append(FrameRecord(path=path, procedure="C", target_id="T4-L", gap_mm=PARAMS.gap_small_mm,
                                   station_z_mm=CHECK_STATION_MM, field=0, pose_index=index, frame_index=0,
                                   robot_pose=RigidTransform.identity(), target_pose_camera=pose,
                                   subseries=SUBSERIES_JITTER))
    return Session(root=tmp_path, params=PARAMS, sensor=SensorConfig(config_id="test", geometry=geometry),
                   registration=None, targets=targets, records=records)


def test_pilot_detection_counts_find_the_small_diameters_undetected(tmp_path: Path, capsys):
    """Section 8 Step 1 / Section 13 Step 2: with disks of 3.5 to 56 mm on a 160 x 120 px stack (p = 4.4 mm), the
    smallest levels are never detected and the largest always are; the detection fraction is nondecreasing in
    practice, D_0 lies below D_50, D_50 is a diameter between two levels, and the post sites (0.5 mm) are not
    detected. The check of the same stack flags nothing."""
    session = pilot_session(tmp_path)
    results = pilot_detection_counts(session, PARAMS, session.geometry, CHECK_STATION_MM)
    result = results[("T4-L", PARAMS.gap_small_mm)]
    assert result.poses == PILOT_POSES and result.tau_mm is not None and result.tau_mm < 1.0e-2
    assert result.fractions[0] == 0.0 and result.fractions[-1] == 1.0
    assert result.d0_mm is not None and result.d50_mm is not None and result.d0_mm < result.d50_mm
    assert result.diameters_mm[0] < result.d50_mm < result.diameters_mm[-1]
    assert result.post_fraction == 0.0 and result.post_trials > 0
    footprint = session.geometry.pixel_footprint_mm(CHECK_STATION_MM)
    assert 0.5 * footprint < result.d50_mm < 6.0 * footprint
    assert results == pilot_detection_counts(session, PARAMS, session.geometry, CHECK_STATION_MM)      # deterministic
    assert pilot_detection_counts(session, PARAMS, session.geometry, 500.0) == {}
    assert pilot_detection_counts(session, PARAMS, session.geometry, CHECK_STATION_MM, subseries=("field",)) == {}
    assert not check_session(session).flagged
    # The command line prints the same numbers.
    write_session_folder(tmp_path, session)
    assert check_cli.main(["--session", str(tmp_path), "--pilot", "750"]) == 0
    assert "pilot D_50" in capsys.readouterr().out
    assert check_cli.main(["--session", str(tmp_path), "--pilot", "500"]) == 2


def test_pilot_summary_numbers():
    """Section 13 Step 2: the 0.5 crossing is interpolated linearly in ln D, and D_0 is the largest diameter below the
    first detection."""
    diameters = [1.0, 2.0, 4.0, 8.0]
    assert interpolate_d50(diameters, [0.0, 0.25, 0.75, 1.0]) == pytest.approx(np.sqrt(2.0 * 4.0))
    assert interpolate_d50(diameters, [0.0, 0.0, 0.0, 0.4]) is None           # never reaches 0.5
    assert interpolate_d50(diameters, [0.6, 0.7, 0.9, 1.0]) is None           # already above at the smallest level
    assert zero_detection_diameter(diameters, [0, 0, 3, 9]) == 2.0
    assert zero_detection_diameter(diameters, [1, 0, 3, 9]) is None
    assert zero_detection_diameter(diameters, [0, 0, 0, 0]) == 8.0
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


def test_detection_plan_warns_when_few_levels_lie_below_the_pilot_d0():
    """Section 8 Step 2: with a pilot D_0 that only one ladder level of T4-S lies below, the plan warns that the zero
    point will have too few levels; with a D_0 above three levels it does not. The pilot values only change the
    summary, never the poses."""
    diagnostics = PlanDiagnostics()
    ladder = PARAMS.diameter_ladder_mm(GEOMETRY)
    base = plan_detection_series(SMALL_PARAMS, GEOMETRY, np.random.default_rng(2), extended=False)
    few = plan_detection_series(SMALL_PARAMS, GEOMETRY, np.random.default_rng(2),
                                {("T4-S", 15.0): 1.0}, {("T4-S", 15.0): 0.5 * (ladder[0] + ladder[1])}, False,
                                diagnostics=diagnostics)
    assert any("only 1 ladder level(s) of T4-S" in w for w in diagnostics.warnings)
    assert any("pilot T4-S" in n or "D pilot T4-S" in n for n in diagnostics.notes)
    assert [c.pose_key() for c in few] == [c.pose_key() for c in base]
    ok = PlanDiagnostics()
    plan_detection_series(SMALL_PARAMS, GEOMETRY, np.random.default_rng(2), None, {("T4-S", 15.0): 0.5 * (ladder[3] + ladder[4])},
                          False, diagnostics=ok)
    assert not ok.warnings
