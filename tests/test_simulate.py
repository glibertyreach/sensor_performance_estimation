"""
Tests of the synthetic renderer, the synthetic session writer and the demonstration plan
(sensorperf.simulate; design document, Section 5 and Section 6). Every test names the step of
the renderer's pipeline or of the procedure it exercises. All tests use the 160 x 120 geometry
(the indicative one scaled so that the field of view is unchanged), seeded generators and small plans.
"""
from __future__ import annotations

import time
import warnings
from dataclasses import replace

import numpy as np
import pytest
from scipy.ndimage import binary_erosion

from sensorperf.acquisition.plan import PlannedCapture
from sensorperf.geometry.targets import (
    SURFACE_BACK, SURFACE_FRONT, SURFACE_NONE, TARGET_KIND_CUTOUT_ARRAY, TARGET_KIND_RAISED_SQUARE,
    StereoGeometry, fronto_parallel_pose, make_edge_target, make_feature_array, make_noise_plate,
    make_standard_target_set,
)
from sensorperf.io.capture_set import load_stack
from sensorperf.io.manifest import APPROACH_DIRECTION_KEY, APPROACH_STANDARD, FIELD_FRACTION_ACHIEVED_KEY, SUBSERIES_JITTER, SUBSERIES_MAIN, SUBSERIES_SENTINEL
from sensorperf.io.matcloud import read_matcloud
from sensorperf.io.session import SERIES_DIRS, Session
from sensorperf.parameters import (
    CharacterizationParameters, PROCEDURE_EDGES, PROCEDURE_NOISE, PROCEDURE_SENTINEL, SensorGeometry,
    TARGET_CUTOUTS, TARGET_DISKS, TARGET_NOISE_PLATE, TARGET_RAISED_SQUARE,
)
from sensorperf.simulate.demo_plan import ALL_SERIES, demo_plan, demo_registration, scaled_geometry
from sensorperf.simulate.sensor_model import (
    SyntheticSensorModel, render_frame, write_frame,
)
from sensorperf.simulate.session import write_synthetic_session

QUICK_DIVISOR = 4
"""The test geometry is the indicative one divided by this (640 x 480 -> 160 x 120)."""
STATION_MM = CharacterizationParameters().z_reference_mm
"""The mid-range station of the tests (Z_REFERENCE_MM)."""
MIN_FEATURE_PX = 10.0
"""The minimum feature diameter of the indicative synthetic matcher, pixels of the full-size sensor."""
NOISE_FRAMES = 20
"""Frames of the temporal statistics in test (a)."""
CENTER_ROI_HALF_PX = 14
"""Half side of the central pixel region used for plate statistics, pixels."""
MEAN_DEPTH_TOLERANCE_MM = 0.5
"""Test (a): allowed difference between the mean rendered depth and the true plate depth."""
SIGMA_FACTOR = 2.0
"""Test (a): the measured temporal sigma must lie within this factor of the predicted one."""
GAP_LARGE_MM = 60.0
"""The large gap of tests (b) and (c)."""
DEPTH_NEAR_TOLERANCE_MM = 3.0
"""Test (b): 'near' for mean depths of whole regions (about two noise sigma of one pixel)."""
STRICT_FILL = 0.9
"""Test (b): a stricter min_window_fill. With the indicative 0.5 the matcher fills a small hole from the
front-surface pixels around it (they are readable), so it reads the front depth instead of failing; with 0.9
the occlusion shadows inside a small hole show up as no-reads."""
CUTOUT_DIAMETERS_MM = (12.0, 40.0, 120.0)
"""Test (b): cutout diameters, about 4, 14 and 41 pixels at Z_MIN in the test geometry."""
CUTOUT_ISOLATION_MM = 40.0
"""Test (b): edge-to-edge spacing of the cutouts."""
FRONT_PREFERENCE_STRONG = 2.0
"""Test (c): the fattening weight."""
HALF_HEIGHT = 0.5
"""Normalized height of the edge crossing in test (c)."""
RISE_LOW, RISE_HIGH = 0.1, 0.9
"""Normalized heights between which the edge rise distance is measured in test (c)."""
PROFILE_ROWS = 5
"""Rows averaged around the image center for the edge profile of test (c)."""
PERFORMANCE_LIMIT_S = 0.5
"""A full-size frame must render faster than this (contract)."""
FULL_SIZE_DIVISOR = 1
"""No reduction: the 640 x 480 geometry."""
ROBOT_REPEATABILITY_MM = 0.2
"""Robot repeatability of test (e)."""
CORRELATION_LAG_NEAR_PX = 1
"""Test: lag at which the noise is still correlated (block size is 4 px)."""
CORRELATION_LAG_FAR_PX = 12
"""Test: lag at which the noise is no longer correlated."""
MIN_NEAR_CORRELATION = 0.5
"""Test: minimum noise correlation at the near lag."""
MAX_FAR_CORRELATION = 0.2
"""Test: maximum noise correlation at the far lag."""
FIXED_PATTERN_TEST_AMPLITUDE_MM = 10.0
"""Test (f): a fixed pattern large compared with the noise averaged over the test frames."""
PATTERN_RECOVERY_FRAMES = 30
"""Test (f): frames averaged to recover the fixed pattern."""
MIN_PATTERN_CORRELATION = 0.9
"""Test (f): minimum correlation between the frame mean and the noise-free pattern render."""
TEST_SEED = 12345
"""Seed of the test random generators."""


@pytest.fixture(scope="module")
def params() -> CharacterizationParameters:
    return CharacterizationParameters()


@pytest.fixture(scope="module")
def geometry() -> SensorGeometry:
    return scaled_geometry(SensorGeometry.indicative(), QUICK_DIVISOR)


@pytest.fixture(scope="module")
def model(geometry) -> SyntheticSensorModel:
    """The indicative matcher on the test geometry with the minimum-feature-size rule off, so that the tests of the
    matcher mechanics (windows, fill, noise) may use any feature size; the rule has its own test below."""
    return replace(SyntheticSensorModel.indicative(geometry), min_feature_diameter_px=0.0)


def _ideal(model: SyntheticSensorModel) -> SyntheticSensorModel:
    """The model with every random or quantizing effect off: only the geometry and the matcher remain."""
    return replace(model, disparity_noise_px=0.0, disparity_quantum_px=0.0, output_lsb_mm=0.0,
                   fixed_pattern_amplitude_mm=0.0)


def _center_roi(model: SyntheticSensorModel) -> tuple[slice, slice]:
    """Rows and columns of the central region around the principal point."""
    geometry = model.geometry
    row, column = int(geometry.sensor_cy_px), int(geometry.sensor_cx_px)
    return (slice(row - CENTER_ROI_HALF_PX, row + CENTER_ROI_HALF_PX),
            slice(column - CENTER_ROI_HALF_PX, column + CENTER_ROI_HALF_PX))


# ---------------------------------------------------------------------------
# (a) Step 4, 5 of the renderer / Section 10: noise plate depth and temporal sigma
# ---------------------------------------------------------------------------
def test_noise_plate_mean_depth_and_temporal_sigma(params, model):
    """A fronto-parallel T2 at 750 mm: mean depth within 0.5 mm of the truth, temporal sigma within a
    factor 2 of sigma_Z = sigma_d Z^2 / k (Section 10, Step 1 quantities)."""
    rng = np.random.default_rng(TEST_SEED)
    plate = make_noise_plate(params)
    pose = fronto_parallel_pose(0.0, 0.0, STATION_MM)
    rows, columns = _center_roi(model)
    stack = np.stack([render_frame(model, plate, pose, rng).depth[rows, columns] for _ in range(NOISE_FRAMES)])
    assert np.all(np.isfinite(stack))
    assert abs(np.mean(stack) - STATION_MM) < MEAN_DEPTH_TOLERANCE_MM
    predicted = model.disparity_noise_px * STATION_MM ** 2 / model.geometry.disparity_constant_mm_px()
    measured = float(np.median(np.std(stack, axis=0, ddof=1)))
    assert predicted / SIGMA_FACTOR < measured < predicted * SIGMA_FACTOR


def test_noise_is_correlated_over_about_one_block(params, model):
    """Step 4: noise drawn per block and interpolated is correlated at small lags and not at large ones
    (the correlation length Analysis A measures is about the block size)."""
    rng = np.random.default_rng(TEST_SEED)
    plate = make_noise_plate(params)
    pose = fronto_parallel_pose(0.0, 0.0, STATION_MM)
    ideal = render_frame(_ideal(model), plate, pose, rng).depth
    frames = np.stack([render_frame(replace(model, fixed_pattern_amplitude_mm=0.0, disparity_quantum_px=0.0,
                                            output_lsb_mm=0.0), plate, pose, rng).depth - ideal
                       for _ in range(NOISE_FRAMES)])
    rows, columns = _center_roi(model)
    noise = frames[:, rows, columns]

    def correlation(lag: int) -> float:
        return float(np.corrcoef(noise[:, :, :-lag].ravel(), noise[:, :, lag:].ravel())[0, 1])

    assert correlation(CORRELATION_LAG_NEAR_PX) > MIN_NEAR_CORRELATION
    assert abs(correlation(CORRELATION_LAG_FAR_PX)) < MAX_FAR_CORRELATION


# ---------------------------------------------------------------------------
# (b) Steps 2, 3: cutout array with the large gap
# ---------------------------------------------------------------------------
def test_cutout_array_depths_and_no_reads(params, model):
    """T5-like array, gap 60 mm (Section 7 targets): large-cutout pixels read near Z + G, plate pixels
    near Z, and with a strict matcher the smallest cutouts have more no-reads than the largest (occlusion
    shadows fill a small hole completely, a large one only along one edge)."""
    rng = np.random.default_rng(TEST_SEED)
    target = make_feature_array(TARGET_CUTOUTS, TARGET_KIND_CUTOUT_ARRAY, list(CUTOUT_DIAMETERS_MM),
                                CUTOUT_ISOLATION_MM, GAP_LARGE_MM)
    station = params.z_min_mm                      # the near station has the widest occlusion shadows
    pose = fronto_parallel_pose(0.0, 0.0, station)
    stereo = StereoGeometry.from_sensor_geometry(model.geometry)
    strict = replace(model, min_window_fill=STRICT_FILL)
    frame = render_frame(strict, target, pose, rng)
    xy = target.pixel_xy_on_front_plane(stereo.camera, pose)
    window = strict.matching_window_px

    # Plate pixels (front surface far from any cutout) read near Z.
    plate_core = binary_erosion(frame.true_surface == SURFACE_FRONT, structure=np.ones((window, window)))
    plate_depths = frame.depth[plate_core]
    assert plate_depths.size > 0
    assert abs(np.nanmean(plate_depths) - station) < DEPTH_NEAR_TOLERANCE_MM

    # The largest cutout's visible interior reads near Z + G.
    largest = max(target.features_of_kind("cutout"), key=lambda f: f.diameter_mm)
    smallest = min(target.features_of_kind("cutout"), key=lambda f: f.diameter_mm)
    inside_largest = largest.inside(xy[..., 0], xy[..., 1])
    visible_back = (frame.true_surface == SURFACE_BACK) & frame.visibility
    core = binary_erosion(visible_back & inside_largest, structure=np.ones((window, window)))
    assert np.count_nonzero(core) > 0
    assert abs(np.nanmean(frame.depth[core]) - (station + GAP_LARGE_MM)) < DEPTH_NEAR_TOLERANCE_MM

    # No-read fraction inside the smallest versus the largest cutout.
    inside_smallest = smallest.inside(xy[..., 0], xy[..., 1]) & (frame.true_surface == SURFACE_BACK)
    inside_largest = inside_largest & (frame.true_surface == SURFACE_BACK)
    assert np.count_nonzero(inside_smallest) > 0
    no_read_small = float(np.mean(np.isnan(frame.depth[inside_smallest])))
    no_read_large = float(np.mean(np.isnan(frame.depth[inside_largest])))
    assert no_read_small > no_read_large


# ---------------------------------------------------------------------------
# (c) Step 3: raised-square edge profile and foreground fattening
# ---------------------------------------------------------------------------
def _left_edge_profile(model, params, front_preference, rng):
    """(normalized height h along the center rows, column of the true left edge) for a T3a render without noise."""
    target = make_edge_target(params, TARGET_KIND_RAISED_SQUARE, GAP_LARGE_MM, geometry=SensorGeometry.indicative())
    pose = fronto_parallel_pose(0.0, 0.0, STATION_MM)
    ideal = replace(_ideal(model), front_preference=front_preference)
    frame = render_frame(ideal, target, pose, rng)
    row = int(model.geometry.sensor_cy_px)
    rows = slice(row - PROFILE_ROWS // 2, row + PROFILE_ROWS // 2 + 1)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)         # columns without any read are NaN, by design
        depth = np.nanmean(frame.depth[rows], axis=0)
    height = (STATION_MM + GAP_LARGE_MM - depth) / GAP_LARGE_MM            # 1 on the front, 0 on the back
    front_columns = np.flatnonzero(np.all(frame.true_surface[rows] == SURFACE_FRONT, axis=0))
    return height, int(front_columns.min())


def _crossing(height: np.ndarray, level: float, start: int, stop: int) -> float:
    """Column (fractional) where the rising profile first reaches ``level`` between ``start`` and ``stop``."""
    segment = height[start:stop]
    index = int(np.flatnonzero(segment >= level)[0])
    previous, current = segment[index - 1], segment[index]
    return start + index - 1 + (level - previous) / (current - previous)


def test_edge_profile_rises_within_window_and_fattens_the_front(params, model):
    """T3a left edge (Section 6.1): the depth rises from back to front within about matching_window_px
    pixels, and front_preference > 1 moves the half-height crossing toward the back-plate side."""
    window = model.matching_window_px
    rng = np.random.default_rng(TEST_SEED)
    neutral, edge_column = _left_edge_profile(model, params, 1.0, rng)
    fat, _ = _left_edge_profile(model, params, FRONT_PREFERENCE_STRONG, rng)
    start, stop = edge_column - window, edge_column + window + 1
    # The profile starts on the back plate and ends on the front.
    assert neutral[start] < RISE_LOW and neutral[stop - 1] > RISE_HIGH
    for profile in (neutral, fat):
        rise = _crossing(profile, RISE_HIGH, start, stop) - _crossing(profile, RISE_LOW, start, stop)
        assert 0.0 < rise <= window
    # Fattening: the crossing of the fattened edge lies at a smaller column, i.e. on the back-plate side
    # of the left edge, by a clearly measurable amount.
    shift = _crossing(neutral, HALF_HEIGHT, start, stop) - _crossing(fat, HALF_HEIGHT, start, stop)
    assert shift > HALF_HEIGHT


# ---------------------------------------------------------------------------
# (d) write_frame / read_matcloud
# ---------------------------------------------------------------------------
def test_write_frame_round_trip(tmp_path, params, model):
    """The .mc file carries the XYZ matrix exactly and the header of the contract plus the extra entries."""
    rng = np.random.default_rng(TEST_SEED)
    pose = fronto_parallel_pose(0.0, 0.0, STATION_MM)
    frame = render_frame(model, make_edge_target(params, TARGET_KIND_RAISED_SQUARE, GAP_LARGE_MM, geometry=SensorGeometry.indicative()), pose, rng)
    path = tmp_path / "frame.mc"
    write_frame(path, frame, model.geometry, {"note": "round trip", "index": 7})
    capture = read_matcloud(path)
    assert np.array_equal(capture.matrices["XYZ"], frame.xyz)
    assert capture.matrices["XYZ"].dtype == np.float32
    geometry = model.geometry
    height, width = frame.xyz.shape[:2]
    header = capture.header
    assert (header["fx"], header["fy"], header["cx"], header["cy"]) == pytest.approx(
        (geometry.sensor_fx_px, geometry.sensor_fy_px, geometry.sensor_cx_px, geometry.sensor_cy_px))
    assert (header["h"], header["v"]) == (width, height)
    assert header["cameraName"] == "synthetic" and header["version"] == 2
    assert header["note"] == "round trip" and header["index"] == 7
    # No-reads are the all-zero point and NaN in the depth array; reads have z equal to the depth.
    read = np.isfinite(frame.depth)
    assert np.all(frame.xyz[~read] == 0.0)
    assert np.allclose(frame.xyz[read][:, 2], frame.depth[read], atol=1.0e-3)


# ---------------------------------------------------------------------------
# (e) The session writer
# ---------------------------------------------------------------------------
def _tiny_plan(params, geometry) -> list[PlannedCapture]:
    """Three poses: a sentinel, a noise pose and a raised-square pose with an offset."""
    offset_h, offset_v = 3.0, -2.0
    return [
        PlannedCapture(PROCEDURE_SENTINEL, TARGET_NOISE_PLATE, None, STATION_MM, 0, 0, 3, SUBSERIES_SENTINEL,
                       fronto_parallel_pose(0.0, 0.0, STATION_MM), seed=1, order=1),
        PlannedCapture(PROCEDURE_NOISE, TARGET_NOISE_PLATE, None, STATION_MM, 0, 0, 4, SUBSERIES_MAIN,
                       fronto_parallel_pose(0.0, 0.0, STATION_MM), seed=2, order=0,
                       notes={FIELD_FRACTION_ACHIEVED_KEY: 0.5}),
        PlannedCapture(PROCEDURE_EDGES, TARGET_RAISED_SQUARE, params.gap_small_mm, STATION_MM, 0, 0, 2,
                       SUBSERIES_JITTER, fronto_parallel_pose(offset_h, offset_v, STATION_MM), seed=3,
                       offset_h_mm=offset_h, offset_v_mm=offset_v, order=2),
    ]


def test_write_synthetic_session_round_trip(tmp_path, params, geometry, model):
    """Section 9: manifest, every frame file and the JSON records are written; Session.load reads them;
    load_stack of one pose gives the frame count; the manifest's target pose is the realized one."""
    rng = np.random.default_rng(TEST_SEED)
    registration = demo_registration()
    targets = make_standard_target_set(params, SensorGeometry.indicative())
    plan = _tiny_plan(params, geometry)
    calls = []
    root = write_synthetic_session(tmp_path / "session", params, geometry, model, registration, targets, plan, rng,
                                   progress=lambda index, total: calls.append((index, total)),
                                   robot_repeatability_mm=ROBOT_REPEATABILITY_MM)
    assert calls == [(1, 3), (2, 3), (3, 3)]
    for name in ("manifest.csv", "sensor_config.json", "registration.json", "targets.json", "parameters.json",
                 "targets_asbuilt.csv", "session_log.md"):
        assert (root / name).is_file(), name
    session = Session.load(root, require_registration=True)
    assert len(session.records) == sum(capture.frames for capture in plan)
    assert session.sensor.config_id == "synthetic"
    assert session.geometry.image_width_px == geometry.image_width_px
    for record in session.records:
        assert record.path.is_file()
        assert record.path.parent.name == SERIES_DIRS[record.procedure]
    # Records are in acquisition order and the clock advances monotonically.
    stamps = [record.timestamp for record in session.records]
    assert stamps == sorted(stamps)
    assert [record.procedure for record in session.records][0] == PROCEDURE_NOISE
    # One pose loads as a stack of the right size.
    noise_records = [r for r in session.records if r.procedure == PROCEDURE_NOISE]
    stack = load_stack(noise_records)
    assert stack.frame_count == 4
    assert stack.depth.shape == (4, geometry.image_height_px, geometry.image_width_px)
    # The manifest's target pose is registration.target_to_camera(read-back), not the planned pose.
    record = noise_records[0]
    # The plan's field fraction reaches the manifest, and so does the approach direction that every pose carries.
    assert record.metadata == {FIELD_FRACTION_ACHIEVED_KEY: "0.5", APPROACH_DIRECTION_KEY: APPROACH_STANDARD}
    assert all(r.metadata == {APPROACH_DIRECTION_KEY: APPROACH_STANDARD} for r in session.records if r.procedure != PROCEDURE_NOISE)
    realized = session.registration.target_to_camera(record.robot_pose)
    translation_error, rotation_error = realized.difference_from(record.target_pose_camera)
    assert translation_error < 1.0e-6 and rotation_error < 1.0e-6
    planned_error, _ = plan[1].target_to_camera.difference_from(record.target_pose_camera)
    assert 0.0 < planned_error < 10.0 * ROBOT_REPEATABILITY_MM
    # The jitter offsets are recorded as planned.
    edge_record = [r for r in session.records if r.procedure == PROCEDURE_EDGES][0]
    assert (edge_record.offset_h_mm, edge_record.offset_v_mm) == (3.0, -2.0)


def test_frame_scale_and_open_background(tmp_path, params, geometry, model):
    """frame_scale reduces the frames per pose by ceil (at least one); a capture without a gap of a target
    that has a back plate is the open-background variant (nothing behind the raised square)."""
    rng = np.random.default_rng(TEST_SEED)
    targets = make_standard_target_set(params, SensorGeometry.indicative())
    open_capture = PlannedCapture(PROCEDURE_EDGES, TARGET_RAISED_SQUARE, None, STATION_MM, 0, 0, 7, SUBSERIES_MAIN,
                                  fronto_parallel_pose(0.0, 0.0, STATION_MM), seed=1, order=0)
    root = write_synthetic_session(tmp_path / "open", params, geometry, model, demo_registration(), targets,
                                  [open_capture], rng, frame_scale=0.2)
    session = Session.load(root)
    assert len(session.records) == 2                                  # ceil(7 * 0.2)
    stack = load_stack(session.records)
    # Only the 160 mm square is solid: far fewer reads than the back plate would give.
    square_fraction = (params.edge_square_size_mm ** 2) / (
        4.0 * session.geometry.half_field_mm(STATION_MM)[0] * session.geometry.half_field_mm(STATION_MM)[1])
    assert float(stack.valid.mean()) < 2.0 * square_fraction


# ---------------------------------------------------------------------------
# (f) Step 5: fixed pattern identical between frames, noise not
# ---------------------------------------------------------------------------
def test_fixed_pattern_is_static_and_noise_is_not(params, model):
    """Step 5: the fixed pattern is the same in every frame (noise off: identical frames), the noise differs
    (pattern off: different frames), and the mean of many noisy frames recovers the pattern."""
    plate = make_noise_plate(params)
    pose = fronto_parallel_pose(0.0, 0.0, STATION_MM)
    rng = np.random.default_rng(TEST_SEED)
    patterned = replace(model, fixed_pattern_amplitude_mm=FIXED_PATTERN_TEST_AMPLITUDE_MM, disparity_noise_px=0.0,
                        disparity_quantum_px=0.0, output_lsb_mm=0.0)
    first, second = render_frame(patterned, plate, pose, rng).depth, render_frame(patterned, plate, pose, rng).depth
    assert np.array_equal(first, second, equal_nan=True)
    assert np.nanstd(first) > 0.0                                      # the pattern is really there

    noisy = replace(patterned, fixed_pattern_amplitude_mm=0.0, disparity_noise_px=model.disparity_noise_px)
    one, two = render_frame(noisy, plate, pose, rng).depth, render_frame(noisy, plate, pose, rng).depth
    assert not np.array_equal(one, two, equal_nan=True)

    both = replace(patterned, disparity_noise_px=model.disparity_noise_px)
    mean_depth = np.mean([render_frame(both, plate, pose, rng).depth for _ in range(PATTERN_RECOVERY_FRAMES)], axis=0)
    valid = np.isfinite(mean_depth) & np.isfinite(first)
    assert np.corrcoef(mean_depth[valid], first[valid])[0, 1] > MIN_PATTERN_CORRELATION


# ---------------------------------------------------------------------------
# Renderer details: truth arrays, determinism, performance, the demonstration plan
# ---------------------------------------------------------------------------
def test_truth_arrays_and_determinism(params, model):
    """The truth arrays follow the ideal ray cast; equal seeds give equal frames."""
    target = make_edge_target(params, TARGET_KIND_RAISED_SQUARE, GAP_LARGE_MM, geometry=SensorGeometry.indicative())
    pose = fronto_parallel_pose(0.0, 0.0, STATION_MM)
    frame = render_frame(model, target, pose, np.random.default_rng(TEST_SEED))
    again = render_frame(model, target, pose, np.random.default_rng(TEST_SEED))
    assert np.array_equal(frame.depth, again.depth, equal_nan=True)
    surface = frame.true_surface
    assert np.all(np.isnan(frame.true_depth[surface == SURFACE_NONE]))
    assert np.allclose(frame.true_depth[surface == SURFACE_FRONT], STATION_MM)
    assert np.allclose(frame.true_depth[surface == SURFACE_BACK], STATION_MM + GAP_LARGE_MM)
    assert np.all(~np.isfinite(frame.depth[surface == SURFACE_NONE]))      # a ray that hits nothing is a no-read
    assert frame.visibility.shape == surface.shape


def test_drift_adds_linearly_with_elapsed_time(params, model):
    """Step 6: drift_mm_per_hour times the elapsed hours is added to every read."""
    drifting = replace(_ideal(model), drift_mm_per_hour=2.0)
    plate = make_noise_plate(params)
    pose = fronto_parallel_pose(0.0, 0.0, STATION_MM)
    rng = np.random.default_rng(TEST_SEED)
    start = render_frame(drifting, plate, pose, rng).depth
    later = render_frame(drifting, plate, pose, rng, elapsed_hours=1.5).depth
    assert np.allclose(later[np.isfinite(start)] - start[np.isfinite(start)], 3.0)


def test_full_size_frame_renders_fast(params):
    """A 640 x 480 frame of a feature array renders in well under 0.5 s (vectorized numpy only)."""
    geometry = SensorGeometry.indicative()
    model = SyntheticSensorModel.indicative(geometry)
    target = make_standard_target_set(params, geometry).get(TARGET_CUTOUTS, params.gap_small_mm)
    pose = fronto_parallel_pose(0.0, 0.0, STATION_MM)
    rng = np.random.default_rng(TEST_SEED)
    render_frame(model, target, pose, rng)                               # warm the caches
    best = min(_timed(lambda: render_frame(model, target, pose, rng)) for _ in range(3))
    assert best < PERFORMANCE_LIMIT_S


def _timed(call) -> float:
    started = time.perf_counter()
    call()
    return time.perf_counter() - started


def test_demo_plan_covers_every_series(params, geometry):
    """The demonstration plan has every series, unique pose indices per configuration, seeds, and the
    jitter offsets inside +/- PHASE_JITTER_SPAN_PX / 2. Registration uses the noise plate T2 (T1 is gone), and C and D
    cover the five shape stations of both feature plates T4 and T5."""
    plan = demo_plan(params, geometry, np.random.default_rng(TEST_SEED), quick=True)
    assert {capture.procedure for capture in plan} >= set(ALL_SERIES) | {PROCEDURE_SENTINEL}
    assert {c.target_id for c in plan if c.procedure == "R"} == {TARGET_NOISE_PLATE}
    for series in ("C", "D"):
        assert {c.target_id for c in plan if c.procedure == series} == {TARGET_DISKS, TARGET_CUTOUTS}
        assert {c.station_z_mm for c in plan if c.procedure == series} == set(params.z_shape_stations_mm())
    assert [capture.order for capture in plan] == list(range(len(plan)))
    keys = [capture.pose_key() for capture in plan]
    assert len(keys) == len(set(keys))
    assert all(capture.seed is not None for capture in plan)
    for capture in plan:
        limit = params.phase_jitter_span_px / 2.0 * geometry.pixel_footprint_mm(capture.station_z_mm)
        assert abs(capture.offset_h_mm) <= limit and abs(capture.offset_v_mm) <= limit
    subset = demo_plan(params, geometry, np.random.default_rng(TEST_SEED), quick=True, series=["D"])
    assert {capture.procedure for capture in subset} == {"D"}
    with pytest.raises(ValueError):
        demo_plan(params, geometry, np.random.default_rng(TEST_SEED), quick=True, series=["Q"])


def test_minimum_feature_diameter_is_a_named_parameter_of_the_matcher(params):
    """Redesign note Section 5 and the simulator: the imitation matcher resolves no disk or cutout below
    ``min_feature_diameter_px`` (indicative 10 px, inside the 8 to 12 px asked for). On the full-size sensor at Z_MAX
    the 7.0 mm disk is 3 px and the 19.7 mm disk 8.5 px: both are left out (the back plate is read where they are), the
    55.8 mm disk (24 px) is rendered; at Z_MIN the 19.7 mm disk (34 px) is rendered. A model without the rule renders
    all of them."""
    geometry = SensorGeometry.indicative()
    model = replace(SyntheticSensorModel.indicative(geometry), disparity_noise_px=0.0, disparity_quantum_px=0.0,
                    output_lsb_mm=0.0, fixed_pattern_amplitude_mm=0.0)
    assert 8.0 <= model.min_feature_diameter_px <= 12.0 and model.min_feature_diameter_px == MIN_FEATURE_PX
    target = make_standard_target_set(params, geometry).get(TARGET_DISKS, params.gap_small_mm)
    disks = sorted(target.features_of_kind("disk"), key=lambda f: f.diameter_mm)
    rng = np.random.default_rng(TEST_SEED)

    def rendered_front_fraction(feature, station, rule_model):
        """1.0 when the pixel at the feature's center sees the front surface in the rendered scene, else 0.0."""
        pose = fronto_parallel_pose(0.0, 0.0, station)
        frame = render_frame(rule_model, target, pose, rng)
        camera = StereoGeometry.from_sensor_geometry(geometry).camera
        u, v, _ = camera.project(target.feature_center_camera(pose, feature))
        return float(frame.true_surface[int(round(v)), int(round(u))] == SURFACE_FRONT)

    small, middle, large = disks
    unconstrained = replace(model, min_feature_diameter_px=0.0)
    for feature, station, expected in ((small, params.z_max_mm, 0.0), (middle, params.z_max_mm, 0.0),
                                       (large, params.z_max_mm, 1.0), (middle, params.z_min_mm, 1.0)):
        assert rendered_front_fraction(feature, station, model) == expected, (feature.site_id, station)
    assert rendered_front_fraction(small, params.z_max_mm, unconstrained) == 1.0
    scaled = SyntheticSensorModel.indicative_scaled(scaled_geometry(geometry, QUICK_DIVISOR), QUICK_DIVISOR)
    assert scaled.min_feature_diameter_px == pytest.approx(MIN_FEATURE_PX / QUICK_DIVISOR)
    indicative = SyntheticSensorModel.indicative(geometry)
    assert scaled.disparity_noise_px == pytest.approx(indicative.disparity_noise_px / QUICK_DIVISOR)
    assert scaled.disparity_quantum_px == pytest.approx(indicative.disparity_quantum_px / QUICK_DIVISOR)
    with pytest.raises(ValueError):
        replace(model, min_feature_diameter_px=-1.0)


def test_demo_plan_has_the_ramp_and_the_ladder_in_quanta(params, geometry):
    """Section 6.2: the demonstration plan has a ramp (T2 tilted about H by the tilt of ``plan.ramp_tilt_deg``) at each of the
    stations of its A series, a ladder whose rungs are multiples of the expected quantum at its station (half, two and eight
    quanta) and the optional staircase, whose step is one tenth of the quantum."""
    from sensorperf.acquisition.plan import ramp_tilt_deg
    from sensorperf.io.manifest import SUBSERIES_RAMP
    quantum_model = SyntheticSensorModel.indicative_scaled(geometry, QUICK_DIVISOR).disparity_quantum_px
    plan = demo_plan(params, geometry, np.random.default_rng(TEST_SEED), quick=True, series=["A", "Z"],
                     disparity_quantum_px=quantum_model)
    ramp = [c for c in plan if c.procedure == "Z" and c.subseries == SUBSERIES_RAMP]
    assert [c.station_z_mm for c in ramp] == [params.z_min_mm, params.z_reference_mm, params.z_max_mm]
    plate = make_noise_plate(params)
    for c in ramp:
        quantum = geometry.depth_quantum_mm(quantum_model, c.station_z_mm)
        assert c.tilt_axis == "H" and c.frames == params.frames_per_ramp_pose
        assert c.tilt_deg == pytest.approx(ramp_tilt_deg(params, geometry, plate, c.station_z_mm, quantum))
    quantum = geometry.depth_quantum_mm(quantum_model, params.z_reference_mm)
    steps = sorted({c.step_mm for c in plan if c.subseries == "ladder"})
    assert steps == pytest.approx([0.5 * quantum, 2.0 * quantum, 8.0 * quantum])
    stairs = [c.step_mm for c in plan if c.subseries == "staircase"]
    assert np.diff(stairs) == pytest.approx(quantum / params.z_staircase_subdivision)


def test_quantizer_steps_the_rows_of_a_ramp_at_the_depth_quantum(params, geometry):
    """Renderer steps 4 and 5 on the ramp pose: with the noise, the fixed pattern and the output LSB off, the disparity of
    every pixel of a tilted T2 is a whole multiple of the disparity quantum q (the matcher quantizes in disparity, so its
    depth is k / (n q)), and a column of the image steps through the levels with plateaus of about the depth quantum
    q Z^2 / k (in rows: the quantum over the true depth change per row). This is why the ramp analysis finds plateaus, and why
    they are visible only where the noise stays below about a fifth of the quantum (with the indicative noise of 0.64 of the
    quantum the average over frames is smooth)."""
    from sensorperf.acquisition.plan import ramp_pose, ramp_tilt_deg
    model = replace(SyntheticSensorModel.indicative_scaled(geometry, QUICK_DIVISOR), disparity_noise_px=0.0,
                    output_lsb_mm=0.0, fixed_pattern_amplitude_mm=0.0)
    plate = make_noise_plate(params)
    quantum = geometry.depth_quantum_mm(model.disparity_quantum_px, STATION_MM)
    tilt = ramp_tilt_deg(params, geometry, plate, STATION_MM, quantum)
    pose, _ = ramp_pose(params, geometry, plate, STATION_MM, quantum, tilt)
    frame = render_frame(model, make_noise_plate(params), pose, np.random.default_rng(TEST_SEED))
    k = geometry.disparity_constant_mm_px()
    read = np.isfinite(frame.depth)
    levels = k / frame.depth[read] / model.disparity_quantum_px
    assert np.allclose(levels, np.round(levels), atol=1e-3)                       # disparity = n q (float32 xyz)
    column = int(geometry.sensor_cx_px)
    rows = np.flatnonzero(read[:, column])
    values = frame.depth[rows, column]
    jumps = np.flatnonzero(np.abs(np.diff(values)) > 0.5 * quantum)
    assert len(jumps) >= 2                                                       # two quanta over the plate (RAMP_QUANTA)
    ideal = frame.true_depth[rows, column]
    row_step = float(np.median(np.diff(ideal)))
    assert float(np.median(np.diff(jumps))) * row_step == pytest.approx(quantum, rel=0.1)
    assert np.allclose(frame.true_depth[rows[0]:rows[-1], column], frame.true_depth[rows[0]:rows[-1], column - 5], atol=1e-3)
