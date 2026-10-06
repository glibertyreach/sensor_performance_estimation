"""
Tests of Analyses C, D and E (sensorperf.analysis.area, detection, boundary; procedure document Sections 12, 13, 14).

Every test names the step of the procedure it checks. Two sessions are rendered with the INDICATIVE synthetic sensor
(160 x 120 px; tests/cde_fixture.py describes the plan): a custom session with B edges, C arrays (field and
open-background variants included) and D trials at the five shape stations of the Z-sweep design, shared by the analysis
tests, and the standard ``sensorperf.cli.simulate --quick`` session for the command-line check.

In the Z-sweep design a plate carries only three features, so the transfer and detection curves are POOLED over the
features and the stations against D_px = D f_x / Z (the features overlap in D_px), and the overlap (scaling) test
compares neighboring features where they overlap. The synthetic matcher's minimum feature diameter is 10 px of the
full-size sensor, 2.5 px of the 160 x 120 px test sensor: a feature below it is not resolved at all.

What the synthetic sensor does, and why the expected signs follow (front_preference w = 2 > 1). The imitation matcher
reports, for every pixel, the weighted mean depth of the readable pixels in its 7 x 7 window with weight w for
front-surface pixels and 1 for back-surface pixels. A pixel is therefore closer to the front plane than to the back
plane (h >= 0.5) as soon as the front fraction f of its window satisfies w f / (w f + 1 - f) >= 1/2, that is
f >= 1 / (1 + w) = 1/3. Front material is "read" a third of a window beyond its true boundary: foreground fattening.
Consequences measured below: disks grow (b > 0), holes in the front plate shrink (b < 0), a hole smaller than about
the window is filled in by the front plate (its window is more than a third front everywhere), and a disk smaller than
about the window is erased (its window is never a third front). The matcher also reads back-plate points that are
geometrically invisible to the right camera or projector whenever their window has readable neighbors (fabricated reads).
"""
from __future__ import annotations

import csv
import dataclasses
import json
import math
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from sensorperf.analysis import area, boundary, detection, resolution_lateral
from sensorperf.analysis.detection import (
    RULE_INCLUSIVE, RULE_PRIMARY, PooledThreshold, StationWindows, calibrate_station_thresholds, calibrate_tau,
    detected_at, outcomes, window_statistic,
)
from sensorperf.geometry.camera import PinholeCamera
from sensorperf.geometry.targets import (
    FEATURE_CUTOUT, FEATURE_DISK, StereoGeometry, fronto_parallel_pose,
)
from sensorperf.io.manifest import SUBSERIES_FIELD, SUBSERIES_JITTER, SUBSERIES_OPEN
from sensorperf.io.session import FORWARD_MODEL_FILE_NAME, Session
from sensorperf.parameters import AREA_BIAS_FIT_D0_FACTOR, CharacterizationParameters, TARGET_CUTOUTS
from sensorperf.stats.intervals import clopper_pearson
from sensorperf.stats.psychometric import PsychometricFitParameters, best_fit, fit_all_curves

import cde_fixture

PARAMS = CharacterizationParameters()
"""The default procedure parameters (the thresholds the synthetic-count tests apply)."""
REPO_ROOT = Path(__file__).resolve().parents[1]
"""The repository root (working directory of the command-line test)."""
STATION_NEAR_MM = 400.0
STATION_MID_MM = 800.0
STATION_FAR_MM = 1600.0
"""The near, reference and far stations of the test sessions (Z_MIN, Z_REFERENCE, Z_MAX)."""
STATION_EDGE_MM = cde_fixture.EDGE_STATIONS_MM[0]
"""The nearer of the two B stations of the test session."""
UNRESOLVED_D_PX = 2.5
"""The synthetic sensor's minimum feature diameter on the test sensor (indicative 10 px of the full-size sensor / 4)."""
RESOLVED_CUTOUT_D_PX = 12.0
"""Cutouts at least this many pixels across are resolved and keep a good part of their area."""
DETECTION_BOOTSTRAP_RESAMPLES = 60
BOUNDARY_BOOTSTRAP_RESAMPLES = 200
"""Reduced resample counts of the tests (the analyses default to BOOTSTRAP_RESAMPLES = 2000)."""
LARGE_CUTOUT_RATIO_RANGE = (0.5, 1.1)
"""Test C1: the sensed fraction of cutouts of 12 px and more (b is about -1 to -1.5 px, so a 12 to 24 px hole keeps 55 to
90 percent)."""
SMALL_CUTOUT_D_PX = UNRESOLVED_D_PX
"""Test C2: cutouts up to this many pixels across are filled in completely."""
SMALL_CUTOUT_RATIO_MAX = 0.05
"""Test C2: their sensed area is practically zero."""
CONTOUR_AGREEMENT = 0.25
"""Test C4: allowed relative difference between the contour and the pixel-count areas of the largest features."""
D50_PX_RANGE = (0.5, 8.0)
"""Test D2: a few pixels."""
ISOTONIC_FACTOR = 1.5
"""Test D5: allowed ratio between the isotonic and the fitted D_50."""
SMALL_FEATURE_D_PX = UNRESOLVED_D_PX
LARGE_FEATURE_D_PX = 12.0
"""Test E5: features up to / from this many pixels across are compared."""
NOISE_REGRESSION_SIGMA_EXPONENT = 2.0
"""Test D8: the fake Analysis A reports sigma_tot proportional to Z to this power (the stereo depth noise law)."""


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def cde_session(tmp_path_factory) -> Session:
    """The custom B, C, D session (rendered once)."""
    root = cde_fixture.build_session(tmp_path_factory.mktemp("cde_session") / "session", series="BCD")
    return Session.load(root)


@pytest.fixture(scope="module")
def area_result(cde_session, tmp_path_factory):
    """Analysis C on the custom session, with its outputs written."""
    result = area.run_area(cde_session, tmp_path_factory.mktemp("c_out"), {})
    out = tmp_path_factory.mktemp("c_files")
    area.write_outputs(result, out)
    result.out_dir = out
    return result


@pytest.fixture(scope="module")
def detection_result(cde_session, tmp_path_factory):
    """Analysis D on the custom session (reduced bootstrap), with its outputs written."""
    options = detection.DetectionOptions(bootstrap_resamples=DETECTION_BOOTSTRAP_RESAMPLES)
    result = detection.run_detection(cde_session, tmp_path_factory.mktemp("d_out"), {}, options)
    out = tmp_path_factory.mktemp("d_files")
    detection.write_outputs(result, out)
    result.out_dir = out
    return result


@pytest.fixture(scope="module")
def detection_with_a_result(cde_session, tmp_path_factory):
    """Analysis D on the cutout trials of the custom session with sigma_tot(Z) from a stand-in for Analysis A, so that the
    stations with few blank windows pool their neighbors (Section 13, Threshold). No reused C trials, so that the
    counts are those of the plan: DETECTION_TRIALS_PER_LEVEL per main station, DETECTION_LOW_TRIALS at the farthest."""
    cutout_trials = [r for r in cde_session.records if r.procedure == "D" and r.target_id == "T5"]
    session = Session(root=cde_session.root, params=cde_session.params, sensor=cde_session.sensor,
                      registration=cde_session.registration, targets=cde_session.targets, records=cutout_trials)
    options = detection.DetectionOptions(bootstrap_resamples=DETECTION_BOOTSTRAP_RESAMPLES, include_reused=False)
    result = detection.run_detection(session, tmp_path_factory.mktemp("d_a_out"),
                                     {"A": FakeNoiseResult(cde_fixture.STATIONS_MM)}, options)
    out = tmp_path_factory.mktemp("d_a_files")
    detection.write_outputs(result, out)
    result.out_dir = out
    return result


@pytest.fixture(scope="module")
def boundary_result(cde_session, tmp_path_factory):
    """Analysis E on the custom session (reduced bootstrap), with its outputs written."""
    options = boundary.BoundaryOptions(bootstrap_resamples=BOUNDARY_BOOTSTRAP_RESAMPLES)
    result = boundary.run_boundary_bias(cde_session, tmp_path_factory.mktemp("e_out"), {}, options)
    out = tmp_path_factory.mktemp("e_files")
    boundary.write_outputs(result, out)
    result.out_dir = out
    return result


@pytest.fixture(scope="module")
def b_result(cde_session, tmp_path_factory):
    """Analysis B (lateral resolution) on the custom session, with its outputs (B_lsf.csv among them) written."""
    out = tmp_path_factory.mktemp("b_files")
    result = resolution_lateral.run_lateral_resolution(cde_session, out, {})
    resolution_lateral.write_outputs(result, out)
    result.out_dir = out
    return result


@pytest.fixture(scope="module")
def area_with_b_result(cde_session, b_result, tmp_path_factory):
    """Analysis C on the custom session with Analysis B's result in ``previous`` (its line spread functions)."""
    return area.run_area(cde_session, tmp_path_factory.mktemp("c_with_b_out"), {"B": b_result})


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _main_rows(result, kind, station):
    return sorted((r for r in result.rows if r["kind"] == kind and r["station_z_mm"] == station
                   and r["subseries"] == SUBSERIES_JITTER and r["field"] == 0), key=lambda r: r["d_px"])


# ---------------------------------------------------------------------------
# Analysis C (Section 12)
# ---------------------------------------------------------------------------
def test_c_step8_cutout_transfer_curve_is_pooled_over_stations_and_falls_toward_small_diameters(area_result):
    """Step 8: the cutout transfer curve against D_px, pooled over features and stations, rises from 0 for the cutouts
    the matcher fills in (up to 2.5 px) to between 50 and 110 percent for the cutouts of 12 px and more, whatever the
    station and the feature that produced the point; the rows keep the feature identity."""
    rows = sorted((r for r in area_result.rows if r["kind"] == FEATURE_CUTOUT and r["subseries"] == SUBSERIES_JITTER
                   and r["field"] == 0), key=lambda r: r["d_px"])
    assert {r["site_id"] for r in rows} == {"cutout_00", "cutout_01", "cutout_02"}
    assert len({r["station_z_mm"] for r in rows}) == len(cde_fixture.STATIONS_MM)
    assert len(rows) == 3 * len(cde_fixture.STATIONS_MM)
    resolved = [r["ratio_true"] for r in rows if r["d_px"] >= RESOLVED_CUTOUT_D_PX]
    assert resolved and all(LARGE_CUTOUT_RATIO_RANGE[0] <= v <= LARGE_CUTOUT_RATIO_RANGE[1] for v in resolved)
    # Below the minimum feature size the cutout is filled in. (At the farthest station the depth noise, 4 mm against the
    # 15 mm step, makes the connected "sensed" area of such a feature meaningless: that is what the overlap test below
    # attributes to sigma_tot(Z), so the fill-in check uses the stations up to the reference station.)
    assert all(r["ratio_true"] < SMALL_CUTOUT_RATIO_MAX for r in rows
               if r["d_px"] <= UNRESOLVED_D_PX and r["station_z_mm"] <= STATION_MID_MM)
    near = [r["ratio_true"] for r in rows if r["station_z_mm"] <= STATION_MID_MM]
    third = len(near) // 3
    assert np.mean(near[-third:]) > np.mean(near[:third])


def test_c_step9_overlap_test_compares_neighboring_features(area_result):
    """Section 12, Step 9: for each (kind, gap, ratio) the scaling test compares the curves of neighboring
    features over their shared D_px range (2 to 3 points of each curve there) and reports the mean difference, its
    bootstrap interval and whether zero lies inside; the curves can differ where the depth noise of the far stations
    matters, and a disagreement carries the sigma_tot(Z) attribution note."""
    tests = area_result.details["overlap_tests"]
    assert {(t["kind"], t["ratio"]) for t in tests} == {
        (FEATURE_DISK, "A_sensed / A_true"), (FEATURE_CUTOUT, "A_sensed / A_true"),
        (FEATURE_CUTOUT, "A_sensed / A_geo (cameras + projector)")}
    assert len(tests) == 3 * 2                                                       # 3 curves of 2 neighbor pairs
    assert {(t["feature_small"], t["feature_large"]) for t in tests if t["kind"] == FEATURE_CUTOUT} == {
        ("T5:cutout_00", "T5:cutout_01"), ("T5:cutout_01", "T5:cutout_02")}
    for t in tests:
        assert t["status"] == "ok" and t["points_small"] >= 2 and t["points_large"] >= 2
        assert t["shared_low_px"] < t["shared_high_px"]
        assert t["interval_lower"] <= t["interval_upper"] and isinstance(t["agrees"], bool)
        assert t["agrees"] == (t["interval_lower"] <= 0.0 <= t["interval_upper"])
        if not t["agrees"]:
            assert "sigma_tot" in t["note"]
    assert all(math.isfinite(t["mean_difference"]) for t in tests)


def test_c_step4_small_cutouts_read_as_front(area_result):
    """Steps 3 and 4: a hole below the matcher's minimum feature size (2.5 px on the test sensor) is not resolved, and
    with front_preference > 1 a hole of up to about 3 px is more than a third front material in every window, so it
    reads as front plate: no back reads, sensed area near 0 and edge bias b = -D / 2."""
    small = [r for r in area_result.rows if r["kind"] == FEATURE_CUTOUT and r["subseries"] == SUBSERIES_JITTER
             and r["d_px"] <= SMALL_CUTOUT_D_PX and r["station_z_mm"] <= STATION_MID_MM]
    assert small
    assert all(r["ratio_true"] < SMALL_CUTOUT_RATIO_MAX for r in small)
    assert all(abs(r["b_mm"] + r["diameter_mm"] / 2.0) < 0.1 * r["diameter_mm"] for r in small)


def test_c_step10_edge_bias_signs(area_result):
    """Step 10: front_preference > 1 grows the front material (a window pixel reads front when only a third of its
    window is front), so large disks are bigger than their outline (b > 0) and large cutouts smaller (b < 0). The
    fit is made on the largest feature of the plate (selection stated in the details)."""
    fits = {f["kind"]: f for f in area_result.details["bias_fits"] if f.get("points")}
    assert fits[FEATURE_DISK]["b_px_mean"] > 0.0
    assert fits[FEATURE_CUTOUT]["b_px_mean"] < 0.0
    assert "largest" in fits[FEATURE_DISK]["selection"]
    # C keeps the bias in its own outputs (Section 12, Step 10) ...
    terms = area_result.edge_bias_terms()
    assert terms["edge_bias_disk_px"] > 0.0 > terms["edge_bias_cutout_px"]
    assert terms["edge_bias_px"] > 0.0                       # positive = the front material grows
    # ... but no term of C enters forward_model_parameters.json (Section 12, Step 13).
    assert area_result.forward_model_terms() == {}


def test_c_step5_contour_area_matches_pixel_count_for_largest_features(area_result):
    """Step 5: the iso-contour polygon area agrees with the pixel-count area within 25 percent for the largest
    feature of each kind at the near and the reference station (12 and 24 px across)."""
    for kind in (FEATURE_DISK, FEATURE_CUTOUT):
        for station in (STATION_NEAR_MM, STATION_MID_MM):
            largest = _main_rows(area_result, kind, station)[-1]
            assert abs(largest["ratio_contour_pixelcount"] - 1.0) < CONTOUR_AGREEMENT, (kind, station, largest)


def test_c_step7_geometric_areas_are_ordered(area_result):
    """Step 7: A_geo with the projector <= A_geo with the cameras only <= A_true, for every cutout with a back
    plate (the projector adds a viewpoint, which can only remove visible area); a disk face has A_geo = A_true."""
    cutouts = [r for r in area_result.rows if r["kind"] == FEATURE_CUTOUT and math.isfinite(r["a_geo_cameras_mm2"])]
    assert cutouts
    for r in cutouts:
        assert r["a_geo_projector_mm2"] <= r["a_geo_cameras_mm2"] * (1 + 1e-9)
        assert r["a_geo_cameras_mm2"] <= r["a_true_mm2"] * (1 + 1e-9)
        assert r["a_geo_projector_mm2"] >= 0.0
    assert any(r["a_geo_cameras_mm2"] < r["a_true_mm2"] for r in cutouts)           # the shadow is real
    for r in (r for r in area_result.rows if r["kind"] == FEATURE_DISK):
        assert r["a_geo_projector_mm2"] == pytest.approx(r["a_true_mm2"])
    comparison = area_result.details["geo_version_comparison"]
    assert comparison["cameras_only"]["n"] > 0


def test_c_step13_csv_has_one_row_per_configuration_and_feature(area_result, cde_session):
    """Step 13: C_area_summary.csv has exactly one row per configuration x measured feature (all features of the
    on-axis configurations), the documented columns, and the figures and the details exist."""
    rows = _read_csv(area_result.out_dir / area.SUMMARY_FILE_NAME)
    assert len(rows) == len(area_result.rows)
    assert list(rows[0].keys()) == list(area.SUMMARY_COLUMNS)
    keys = [(r["target_id"], r["gap_mm"], r["station_z_mm"], r["field"], r["subseries"], r["site_id"]) for r in rows]
    assert len(keys) == len(set(keys))
    configurations = {(r.target_id, r.gap_mm, r.station_z_mm, r.field, r.subseries) for r in cde_session.records
                      if r.procedure == "C"}
    for target_id, gap, station, field_code, subseries in configurations:
        if field_code != 0 or subseries != SUBSERIES_JITTER:
            continue
        target = cde_session.targets.get(target_id, gap)
        measured = [f for f in target.features if f.kind in (FEATURE_DISK, FEATURE_CUTOUT)]
        found = [r for r in area_result.rows if (r["target_id"], r["gap_mm"], r["station_z_mm"], r["field"],
                                                 r["subseries"]) == (target_id, gap, station, field_code, subseries)]
        assert len(found) == len(measured)
    for name in ("C_transfer_true_G15.png", "C_edge_bias_vs_z.png", "C_phase_spread_vs_dpx.png",
                 "C_predicted_vs_measured.png", area.DETAILS_FILE_NAME, area.OVERLAP_FILE_NAME):
        assert (area_result.out_dir / name).exists(), name


def test_c_step12_field_and_open_background(area_result):
    """Step 12: the field sub-series is compared with the on-axis transfer curves, and for the open-background
    cutouts the no-read area is compared with A_true (no back plane exists there, so nothing else is defined)."""
    field_rows = [r for r in area_result.rows if r["subseries"] == SUBSERIES_FIELD]
    assert field_rows and area_result.details["field_comparison"]
    differences = [c["difference"] for c in area_result.details["field_comparison"] if math.isfinite(c["difference"])]
    assert differences and max(abs(d) for d in differences) < 0.6                  # the same sensor model off axis
    open_rows = [r for r in area_result.rows if r["subseries"] == SUBSERIES_OPEN]
    assert open_rows and all(r["gap_mm"] is None for r in open_rows)
    ratios = {r["site_id"]: r["ratio_noread_true"] for r in open_rows if r["d_px"] > 8.0}
    assert ratios and all(0.8 < v < 1.25 for v in ratios.values())                 # large holes stay holes
    assert area_result.details["open_background"]


def test_c_step11_consistency_with_blur_prediction(area_result):
    """Step 11: without Analysis B the kernel comes from the own radial ESF (stated in the details); the prediction
    including the edge offset reproduces the area of the large disks within 15 percent, and the sign of b agrees with
    the sign expected from s_50 (front grows by -s_50: b_disk ~ -s_50, b_cutout ~ +s_50)."""
    kernel = area_result.details["kernel"]
    assert kernel is not None and "own radial ESF" in kernel["source"]
    sign = area_result.details["sign_check"]
    assert sign[FEATURE_DISK]["agrees"] is True and sign[FEATURE_CUTOUT]["agrees"] is True
    large = [c for c in area_result.details["consistency_with_b"] if c["kind"] == FEATURE_DISK and c["d_px"] > 10.0]
    assert large
    for c in large:
        assert abs(c["ratio_measured_over_blur_offset"] - 1.0) < 0.15
    # Disagreement for small D: the blur predicts a vanishing disk long before the matcher erases it.
    pure_ratio = [c["ratio_measured_over_blur"] for c in area_result.details["consistency_with_b"]
                  if c["kind"] == FEATURE_DISK and c["d_px"] > 10.0]
    assert all(math.isfinite(v) and v > 1.0 for v in pure_ratio)                  # pure blur under-predicts growth


def test_c_kernel_uses_previous_b_when_present(area_result, cde_session):
    """Step 11, Gaussian fallback: rise distances of Analysis B (rise_h_px, rise_v_px, edge_offset_px) define a Gaussian kernel with the
    10-90 percent rise (stated assumption)."""
    class FakeB:
        def forward_model_terms(self):
            return {"rise_h_px": 5.0, "rise_v_px": 4.0, "edge_offset_px": -1.0}

    factor = area._gaussian_rise_factor(0.1, 0.9)
    kernel = area._kernel_from_b({"B": FakeB()}, factor)
    assert kernel["sigma_h_px"] == pytest.approx(5.0 / factor) and kernel["sigma_v_px"] == pytest.approx(4.0 / factor)
    assert kernel["edge_offset_px"] == -1.0
    assert area._kernel_from_b({}, factor) is None


def test_c_step10_bias_fit_threshold_is_a_parameter_tied_to_the_expected_d0():
    """Section 12, Step 10: the threshold of the b fit is CharacterizationParameters.area_bias_fit_min_d_px, twice the 7 px
    expected minimum detectable diameter, and no longer an option of the analysis."""
    params = CharacterizationParameters()
    assert params.area_bias_fit_min_d_px == pytest.approx(AREA_BIAS_FIT_D0_FACTOR * params.expected_d0_px)
    assert params.area_bias_fit_min_d_px == pytest.approx(14.0)
    assert not hasattr(area.AreaOptions(), "bias_fit_min_d_px")


def test_c_step10_bias_fit_uses_the_largest_feature_where_it_is_at_least_the_threshold(area_result):
    """Step 10: per plate, only the largest feature enters the fit, and only at the stations where its D_px is at least
    AREA_BIAS_FIT_MIN_D_PX (14 px); the details state the threshold."""
    threshold = PARAMS.area_bias_fit_min_d_px
    assert area_result.details["bias_fit_min_d_px"] == threshold
    fits = [f for f in area_result.details["bias_fits"] if f.get("points")]
    assert {f["kind"] for f in fits} == {FEATURE_DISK, FEATURE_CUTOUT}
    for fit in fits:
        assert len(fit["sites"]) == 1 and fit["sites"][0][1].endswith("_02")        # the largest feature of the plate
        features = [r for r in area_result.rows if r["kind"] == fit["kind"] and r["site_id"] == fit["sites"][0][1]
                    and r["subseries"] == SUBSERIES_JITTER and r["field"] == 0]
        expected_stations = sorted(r["station_z_mm"] for r in features if r["d_px"] >= threshold)
        assert fit["stations_mm"] == expected_stations and 0 < len(expected_stations) < len(features)
        assert "AREA_BIAS_FIT_MIN_D_PX" in fit["selection"]


def test_c_step9_overlap_file_carries_pair_kind_gap_ratio_difference_and_zero_test(area_result):
    """Step 9: C_overlap_test.csv has one row per feature pair, kind, gap and ratio, with the mean difference over the shared
    range, its bootstrap interval and whether zero lies inside it."""
    rows = _read_csv(area_result.out_dir / area.OVERLAP_FILE_NAME)
    assert list(rows[0].keys()) == list(area.OVERLAP_COLUMNS)
    assert {"kind", "gap_mm", "ratio", "feature_small", "feature_large", "mean_difference", "interval_lower",
            "interval_upper", "agrees"} <= set(rows[0])
    keys = [(r["kind"], r["gap_mm"], r["ratio"], r["feature_small"], r["feature_large"]) for r in rows]
    assert len(keys) == len(set(keys)) == len(area_result.details["overlap_tests"])
    for r in rows:
        inside = float(r["interval_lower"]) <= 0.0 <= float(r["interval_upper"])
        assert (r["agrees"] == "True") == inside


def test_c_step7_both_a_geo_versions_are_equal_for_a_circular_cutout_with_a_midway_projector(cde_session):
    """Step 7: with the projector midway between the cameras the three-center overlap equals the two-camera one for a convex
    outline (a circle), for every cutout of the plate; with the as-built offset of the test sensor (40 mm against the 37.5 mm
    midpoint) the projector version can only be smaller."""
    camera = PinholeCamera(cde_session.geometry.image_width_px, cde_session.geometry.image_height_px,
                           cde_session.geometry.sensor_fx_px, cde_session.geometry.sensor_fy_px,
                           cde_session.geometry.sensor_cx_px, cde_session.geometry.sensor_cy_px)
    baseline = cde_session.geometry.sensor_baseline_mm
    midway = StereoGeometry(camera, (baseline, 0.0, 0.0), (baseline / 2.0, 0.0, 0.0))
    as_built = StereoGeometry.from_sensor_geometry(cde_session.geometry)
    target = cde_session.targets.get(TARGET_CUTOUTS, PARAMS.gap_small_mm)
    pose = fronto_parallel_pose(0.0, 0.0, STATION_MID_MM)
    cutouts = [f for f in target.features if f.kind == FEATURE_CUTOUT]
    assert cutouts
    for feature in cutouts:
        cameras = area.geometric_visible_area_mm2(target, pose, feature, midway, False)
        projector = area.geometric_visible_area_mm2(target, pose, feature, midway, True)
        assert 0.0 < cameras < feature.true_area_mm2()                              # the shadow is real
        assert projector == pytest.approx(cameras, rel=1e-9)
        assert area.geometric_visible_area_mm2(target, pose, feature, as_built, True) <= cameras * (1.0 + 1e-9)


def test_c_step7_the_as_built_projector_position_is_used_and_reported(area_result, cde_session):
    """Step 7: the A_geo rays start at PROJECTOR_OFFSET_MM of the sensor configuration (not an assumed midpoint), and the
    details record it with the midway test."""
    from sensorperf.analysis.common import pose_geometry
    from sensorperf.io.capture_set import load_stack
    record = next(r for r in cde_session.records if r.procedure == "C" and r.target_id == TARGET_CUTOUTS)
    stereo = pose_geometry(cde_session, record, load_stack([record]).camera).stereo
    assert tuple(stereo.projector_center_mm) == tuple(cde_session.geometry.projector_offset_mm)
    projector = area_result.details["projector"]
    assert projector["projector_offset_mm"] == list(cde_session.geometry.projector_offset_mm)
    assert projector["projector_midway"] is False                                    # 40 mm against 37.5 mm
    columns = {"a_geo_cameras_mm2", "a_geo_projector_mm2"}
    assert columns <= set(area.SUMMARY_COLUMNS)


def test_c_step12_field_comparison_carries_the_achieved_field_fraction_per_pose(area_result, cde_session):
    """Step 12: the field comparison has one entry per field pose and feature, each with the pose's achieved field fraction
    (the manifest column) as a factor; the open-background comparison holds the reference station only."""
    comparison = area_result.details["field_comparison"]
    poses = [r for r in cde_session.records if r.procedure == "C" and r.subseries == SUBSERIES_FIELD
             and r.frame_index == 0]
    features = sum(1 for r in comparison if r["pose_index"] == 0) // len({r["target_id"] for r in comparison})
    assert len(comparison) == len(poses) * features
    assert {c["field_fraction_achieved"] for c in comparison} == set(cde_fixture.FIELD_FRACTIONS_ACHIEVED)
    for c in comparison:
        assert c["field_fraction_achieved"] == cde_fixture.FIELD_FRACTIONS_ACHIEVED[
            c["pose_index"] % len(cde_fixture.FIELD_FRACTIONS_ACHIEVED)]
    assert all(r["station_z_mm"] == PARAMS.z_reference_mm for r in area_result.details["open_background"])
    # A summary row of the field sub-series reports the mean achieved fraction of its poses.
    field_rows = [r for r in area_result.rows if r["subseries"] == SUBSERIES_FIELD]
    assert all(math.isfinite(r["field_fraction_achieved"]) for r in field_rows)
    assert all(math.isnan(r["field_fraction_achieved"]) for r in area_result.rows if r["subseries"] != SUBSERIES_FIELD)


def test_c_step12_open_background_leaves_out_other_stations():
    """Step 12: an open-background row away from the reference station is not compared, and the notes say so."""
    def row(station):
        return {"target_id": "T5", "station_z_mm": station, "site_id": "cutout_00", "d_px": 10.0, "a_true_mm2": 100.0,
                "a_noread_mean_mm2": 95.0, "ratio_noread_true": 0.95}
    notes: list[str] = []
    kept = area._open_background([row(800.0), row(1131.0)], 800.0, notes)
    assert [k["station_z_mm"] for k in kept] == [800.0] and len(notes) == 1


def test_b_step16_lsf_file_has_one_row_per_edge_and_bin(b_result):
    """Section 11, Step 16: B_lsf.csv has one row per edge, Z, orientation, polarity, gap and signed-distance bin, with the
    smoothed LSF and its s."""
    rows = _read_csv(b_result.out_dir / resolution_lateral.LSF_CSV_NAME)
    assert resolution_lateral.LSF_CSV_NAME == "B_lsf.csv"
    assert list(rows[0].keys()) == list(resolution_lateral.LSF_COLUMNS)
    bins = len(b_result.edges[0].bin_centers_px)
    assert len(rows) == len(b_result.edges) * bins == len(b_result.lsf_rows())
    keys = {(r["target_id"], r["polarity"], r["gap_mm"], r["station_z_mm"], r["edge"], r["orientation"], r["bin_index"])
            for r in rows}
    assert len(keys) == len(rows)
    edge = b_result.edges[0]
    first = [r for r in rows if (r["target_id"], r["edge"], r["station_z_mm"]) == (
        edge.target_id, edge.edge, repr(edge.station_z_mm)) and r["polarity"] == edge.polarity
        and float(r["gap_mm"]) == edge.gap_mm]
    assert [float(r["s_px"]) for r in first] == pytest.approx(list(edge.bin_centers_px))
    assert np.all(np.isfinite(edge.lsf))                                            # a well-sampled edge has an LSF
    assert [float(r["lsf_per_px"]) for r in first] == pytest.approx(list(edge.lsf), abs=1e-9)
    assert {r["orientation"] for r in rows} == {resolution_lateral.ORIENTATION_H, resolution_lateral.ORIENTATION_V}


def test_c_step11_line_spread_from_lsf_centers_symmetrizes_and_normalizes():
    """Step 11: a skewed, noisy LSF with a negative tail and an offset median becomes a nonnegative, symmetric kernel of unit
    sum centered on its median; the offset and the rise are reported."""
    s = np.arange(-8.0, 8.0 + 1e-9, 0.25)
    offset, sigma = -0.8, 1.2
    lsf = np.exp(-0.5 * ((s - offset) / sigma) ** 2) / (sigma * math.sqrt(2.0 * math.pi))
    lsf[:4] = -0.002                                                                # ringing in the tail
    made = area.line_spread_from_lsf(s, lsf, 0.1, 0.9)
    assert made is not None
    spread, info = made
    assert info["median_px"] == pytest.approx(offset, abs=0.05)
    assert info["rise_px"] == pytest.approx(sigma * area._gaussian_rise_factor(0.1, 0.9), rel=0.03)
    assert np.all(spread.weights >= 0.0) and spread.weights.sum() == pytest.approx(1.0)
    assert spread.weights == pytest.approx(spread.weights[::-1])
    assert float(np.sum(spread.s_px * spread.weights)) == pytest.approx(0.0, abs=1e-9)
    # A kernel measured as a Gaussian predicts the same area as the Gaussian itself.
    for kind in (FEATURE_DISK, FEATURE_CUTOUT):
        measured = area.predicted_area_from_line_spreads_px2(12.0, spread, spread, 0.0, 8, kind)
        gaussian = area.predicted_area_px2(12.0, sigma, sigma, 0.0, 8, kind)
        assert measured == pytest.approx(gaussian, rel=0.03)
    assert area.line_spread_from_lsf(s, np.zeros_like(s), 0.1, 0.9) is None


def test_c_step11_kernel_is_built_from_the_measured_line_spread_functions_of_b(area_with_b_result, b_result):
    """Step 11: with Analysis B's line spread functions the kernel is the measured, separable one; the details say so; both
    predictions (blur alone, blur plus the s_50 growth) are made, the growth reproduces the large disks, and the sign of b
    agrees with s_50 (b_disk ~ -s_50, b_cutout ~ +s_50)."""
    details = area_with_b_result.details
    kernel = details["kernel"]
    assert kernel["kind"] == area.KERNEL_KIND_MEASURED and "separable" in kernel["source"]
    assert {e["b_station_z_mm"] for e in kernel["stations"]} == set(cde_fixture.EDGE_STATIONS_MM)
    assert not any("Gaussian" in note for note in area_with_b_result.notes)
    comparison = details["consistency_with_b"]
    assert comparison and all(math.isfinite(c["a_predicted_blur_mm2"]) and math.isfinite(c["a_predicted_blur_offset_mm2"])
                              for c in comparison)
    large = [c for c in comparison if c["kind"] == FEATURE_DISK and c["d_px"] > 10.0]
    assert large
    for c in large:
        assert abs(c["ratio_measured_over_blur_offset"] - 1.0) < 0.15
        assert c["a_predicted_blur_offset_mm2"] > c["a_predicted_blur_mm2"]        # the growth enlarges a disk
    sign = details["sign_check"]
    assert sign[FEATURE_DISK]["agrees"] is True and sign[FEATURE_CUTOUT]["agrees"] is True
    assert sign["s50_px"] < 0.0                                                      # the front material is fattened


def test_c_step11_kernel_falls_back_to_a_gaussian_and_says_so_when_there_are_no_line_spread_functions(area_result):
    """Step 11: without B_lsf.csv (no B result, nothing in the output folder) the kernel is the Gaussian of equal 10-90
    percent rise, and the details and notes record that."""
    kernel = area_result.details["kernel"]
    assert kernel["kind"] == area.KERNEL_KIND_GAUSSIAN and "10-90" in kernel["note"]
    assert any("Gaussian" in note for note in area_result.notes)


def test_c_step11_kernel_is_read_from_the_b_output_folder(cde_session, b_result, tmp_path):
    """Step 11: when Analysis B did not run in this process, B_lsf.csv in the output folder provides the kernel."""
    out = tmp_path / "analysis"
    out.mkdir()
    (out / resolution_lateral.LSF_CSV_NAME).write_bytes(
        (b_result.out_dir / resolution_lateral.LSF_CSV_NAME).read_bytes())
    options = area.AreaOptions(make_geo_areas=False)
    result = area.run_area(cde_session, out, {}, options)
    kernel = result.details["kernel"]
    assert kernel["kind"] == area.KERNEL_KIND_MEASURED and resolution_lateral.LSF_CSV_NAME in kernel["source"]


# ---------------------------------------------------------------------------
# Analysis D (Section 13)
# ---------------------------------------------------------------------------
def _d_row(result, kind, station, rule):
    """The summary row of one configuration (a station) and rule."""
    return next(r for r in result.rows if r["kind"] == kind and r["station_z_mm"] == station and r["rule"] == rule)


def _pooled_row(result, kind, rule):
    """The pooled row (all stations of the small gap) of a kind and rule."""
    return next(r for r in result.pooled_rows if r["kind"] == kind and r["rule"] == rule and r["field"] == 0)


class FakeNoiseResult:
    """A stand-in for Analysis A: sigma_tot proportional to Z squared, as the stereo depth noise law says (1 mm at
    1 m), reported at the stations of the test session."""

    def __init__(self, stations):
        self.rows = [{"station_z_mm": z, "sigma_tot_mm": (z / 1000.0) ** NOISE_REGRESSION_SIGMA_EXPONENT,
                      "field": 0, "subseries": "main"} for z in stations]


def test_d_step2_window_statistic_equals_labeling():
    """Step 2: the detection statistic S of a window (the largest tau at which the rule detects) satisfies
    'detected at tau  <=>  S > tau' against the direct 8-connected labeling, for 2 and 3 connected pixels, for
    the primary and the no-read-inclusive rule."""
    rng = np.random.default_rng(3)
    for trial in range(40):
        delta = rng.normal(0.0, 1.0, (14, 14))
        delta[rng.random((14, 14)) < 0.15] = np.nan
        yy, xx = np.mgrid[:14, :14]
        window = (xx - 7) ** 2 + (yy - 7) ** 2 <= 36
        plate = rng.random((14, 14)) < 0.9
        for minimum in (1, 2, 3):
            for inclusive in (False, True):
                candidates = plate if inclusive else None
                statistic = window_statistic(delta, window, minimum, candidates)
                for tau in (-1.0, 0.0, 0.5, 1.2, 2.0):
                    assert (statistic > tau) == detected_at(delta, window, tau, minimum, candidates), \
                        (trial, minimum, inclusive, tau)


def test_d_step3_threshold_calibration_meets_false_alarm_target(cde_session):
    """Step 3: per-level tau is the (1 - target) quantile of the blank statistics ('higher'), so the false-alarm
    fraction on the calibration blanks does not exceed DETECTION_FALSE_ALARM_TARGET."""
    target = cde_session.params.detection_false_alarm_target
    rng = np.random.default_rng(5)
    blanks = rng.normal(size=(500, 3))
    taus = calibrate_tau(blanks, target)
    fraction = (blanks > taus[None, :]).mean(axis=0)
    assert np.all(fraction <= target + 1e-12) and np.all(fraction >= target - 2.0 / 500)


def test_d_step3_false_alarm_rate_matches_target(detection_result, cde_session):
    """Step 3: the measured gamma of a cutout station (from its blank sites) is consistent with the 0.01 target: the
    target lies inside the Clopper-Pearson interval of gamma, or gamma does not exceed the target (0 of n, bound
    reported); the pooled gamma of the group pools the blank sites of all its stations."""
    target = cde_session.params.detection_false_alarm_target
    row = _d_row(detection_result, FEATURE_CUTOUT, STATION_MID_MM, RULE_PRIMARY)
    assert row["gamma_blank_trials"] >= 1.0 / target
    pooled = _pooled_row(detection_result, FEATURE_CUTOUT, RULE_PRIMARY)
    assert pooled["gamma_blank_trials"] == sum(r["gamma_blank_trials"] for r in detection_result.rows
                                               if r["kind"] == FEATURE_CUTOUT and r["rule"] == RULE_PRIMARY)
    assert row["gamma_lower"] <= target <= row["gamma_upper"] or row["gamma"] <= target
    assert row["gamma_lower"] <= row["gamma"] <= row["gamma_upper"]
    # With fewer than 1 / target trials per level the quantile is the maximum: 0 of n, upper bound reported.
    row_far = _d_row(detection_result, FEATURE_CUTOUT, STATION_FAR_MM, RULE_PRIMARY)
    assert row_far["gamma"] <= target
    assert row_far["gamma_upper"] == pytest.approx(clopper_pearson(int(round(row_far["gamma"]
                                                                              * row_far["gamma_blank_trials"])),
                                                                   int(row_far["gamma_blank_trials"]),
                                                                   cde_session.params.confidence_level)[1])


# ---------------------------------------------------------------------------
# Step 3: the threshold pooled over blank sites and, where short, over neighboring stations
# ---------------------------------------------------------------------------
THRESHOLD_STATIONS_MM = (566.0, 800.0, 1131.0)
"""Stations of the synthetic threshold tests: the middle one is the station under test; 566 mm is its nearer neighbor
(234 mm away) and 1131 mm the farther (331 mm)."""
OWN_STATION_INDEX = 1


def _blank_windows(rng, counts, sigmas):
    """Synthetic blank windows of one window size at the stations of THRESHOLD_STATIONS_MM: ``counts`` windows each,
    normal with a spread proportional to the station's sigma_tot (the window statistic is in mm)."""
    return [StationWindows(z, sigma, rng.normal(0.0, sigma, count))
            for z, sigma, count in zip(THRESHOLD_STATIONS_MM, sigmas, counts)]


def _single_level(stations: list[StationWindows]) -> list[StationWindows]:
    """Stations of one window size in the per-level layout of StationWindows (one array per level)."""
    return [StationWindows(s.station_z_mm, s.sigma_tot_mm, [s.windows]) for s in stations]


def test_d_step3_a_station_with_too_few_blank_windows_pools_its_nearest_neighbor_until_the_minimum_is_reached():
    """Section 13, Threshold: 180 blank windows (3 sites x 60 trials) are fewer than DETECTION_MIN_BLANK_WINDOWS (300), so
    the nearest neighboring station is pooled, after dividing by sigma_tot(Z), and the pooling stops as soon as the count
    reaches the minimum: here after the one neighbor (360 windows), not the second. The count and the stations pooled are
    recorded, and tau is the quantile of the normalized pool times the station's own sigma_tot."""
    target, minimum = PARAMS.detection_false_alarm_target, PARAMS.detection_min_blank_windows
    own_count = 180
    assert own_count < minimum
    assert minimum == round(3.0 / target)                      # three windows per false alarm at the target
    rng = np.random.default_rng(11)
    sigmas = (0.5, 1.0, 2.0)
    stations = _single_level(_blank_windows(rng, (own_count,) * 3, sigmas))
    result = calibrate_station_thresholds(stations, OWN_STATION_INDEX, target, minimum)[0]
    assert result.pooled and not result.short
    assert result.windows == 2 * own_count and result.stations_mm == (566.0, 800.0)      # the nearer neighbor only
    normalized = np.concatenate([stations[0].windows[0] / sigmas[0], stations[1].windows[0] / sigmas[1]])
    assert result.tau_mm == pytest.approx(np.quantile(normalized, 1.0 - target, method="higher") * sigmas[1])


def test_d_step3_pooling_continues_to_the_second_neighbor_when_the_first_is_not_enough():
    """With 100 windows at each of three stations, the station and its nearest neighbor give 200 (< 300), so the next
    nearest is added as well (300): all three stations, in increasing Z, are recorded."""
    target, minimum = PARAMS.detection_false_alarm_target, PARAMS.detection_min_blank_windows
    per_station = minimum // 3
    stations = _single_level(_blank_windows(np.random.default_rng(12), (per_station,) * 3, (1.0, 1.0, 1.0)))
    result = calibrate_station_thresholds(stations, OWN_STATION_INDEX, target, minimum)[0]
    assert result.windows == 3 * per_station == minimum
    assert result.stations_mm == THRESHOLD_STATIONS_MM and result.pooled and not result.short


def test_d_step3_a_station_with_enough_blank_windows_does_not_pool():
    """900 blank windows (3 sites x 300 trials, an extended station) reach the minimum on their own: tau is the plain
    quantile of the station's windows, nothing is divided by sigma_tot, and only the station itself is recorded."""
    target, minimum = PARAMS.detection_false_alarm_target, PARAMS.detection_min_blank_windows
    own_count = 3 * PARAMS.detection_low_trials
    stations = _single_level(_blank_windows(np.random.default_rng(13), (180, own_count, 180), (1.0, 2.0, 4.0)))
    result = calibrate_station_thresholds(stations, OWN_STATION_INDEX, target, minimum)[0]
    assert not result.pooled and not result.short
    assert result.windows == own_count and result.stations_mm == (800.0,)
    assert result.tau_mm == np.quantile(stations[1].windows[0], 1.0 - target, method="higher")


def test_d_step3_normalized_pooling_equals_direct_calibration_when_sigma_tot_is_equal():
    """With the same sigma_tot at the pooled stations, dividing by it and multiplying back changes nothing: tau equals the
    direct quantile (calibrate_tau) of the windows of the stations pooled."""
    target, minimum = PARAMS.detection_false_alarm_target, PARAMS.detection_min_blank_windows
    sigma = 1.7
    stations = _single_level(_blank_windows(np.random.default_rng(14), (180, 180, 180), (sigma,) * 3))
    result = calibrate_station_thresholds(stations, OWN_STATION_INDEX, target, minimum)[0]
    assert result.pooled and result.stations_mm == (566.0, 800.0)
    direct = calibrate_tau(np.concatenate([stations[0].windows[0], stations[1].windows[0]])[:, None], target)[0]
    assert result.tau_mm == pytest.approx(direct, rel=1e-12)


def test_d_step3_without_sigma_tot_a_short_station_keeps_its_own_windows_and_says_so():
    """No sigma_tot(Z) (Analysis A did not run) means no common scale to pool on: the station keeps its own windows, tau is
    their plain quantile, and the result is flagged short of the minimum. A neighbor without sigma_tot is skipped."""
    target, minimum = PARAMS.detection_false_alarm_target, PARAMS.detection_min_blank_windows
    rng = np.random.default_rng(15)
    no_sigma = _single_level(_blank_windows(rng, (180, 180, 180), (1.0, 1.0, 1.0)))
    no_sigma = [StationWindows(s.station_z_mm, None, s.windows) for s in no_sigma]
    result = calibrate_station_thresholds(no_sigma, OWN_STATION_INDEX, target, minimum)[0]
    assert not result.pooled and result.short and result.windows == 180 and result.stations_mm == (800.0,)
    assert result.tau_mm == np.quantile(no_sigma[1].windows[0], 1.0 - target, method="higher")
    # The nearer neighbor has no sigma_tot: the next one is used instead.
    partly = [no_sigma[0], StationWindows(800.0, 1.0, no_sigma[1].windows),
              StationWindows(1131.0, 1.0, no_sigma[2].windows)]
    result = calibrate_station_thresholds(partly, OWN_STATION_INDEX, target, minimum)[0]
    assert result.stations_mm == (800.0, 1131.0) and result.windows == 360 and not result.short
    # Every neighbor used and the count still short: the station is pooled as far as it can be and flagged short.
    few = _single_level(_blank_windows(rng, (60, 60, 60), (1.0, 1.0, 1.0)))
    result = calibrate_station_thresholds(few, OWN_STATION_INDEX, target, minimum)[0]
    assert result.pooled and result.short and result.windows == 180 and result.stations_mm == THRESHOLD_STATIONS_MM


def test_d_step3_each_window_size_has_its_own_threshold_and_count():
    """tau is per window size: two sizes with different blank-window counts at the same station are pooled independently,
    the one with enough windows not at all."""
    target, minimum = PARAMS.detection_false_alarm_target, PARAMS.detection_min_blank_windows
    rng = np.random.default_rng(16)
    stations = []
    for z, sigma in zip(THRESHOLD_STATIONS_MM, (1.0, 1.0, 1.0)):
        stations.append(StationWindows(z, sigma, [rng.normal(size=3 * 100), rng.normal(size=100)]))
    small, large = calibrate_station_thresholds(stations, OWN_STATION_INDEX, target, minimum)
    assert not small.pooled and small.windows == 300                 # three blank sites hold the smallest window
    assert large.pooled and large.windows == 300                     # one blank site holds the largest: three stations
    assert large.stations_mm == THRESHOLD_STATIONS_MM


def test_d_step3_blank_sites_are_pooled_per_window_size_in_the_collected_trials(detection_result):
    """A blank site holds the window of the feature it serves and of every smaller one, so the smallest feature's window
    is taken on all three blank sites, the middle one's on two, the largest's on one; the own-site column of the pool is
    the statistic Analysis E reads (s_blank)."""
    for config in detection_result.configs:
        for rule in config.rules():
            pool = config.blank_pool(rule)
            assert pool.shape[1:] == (len(config.levels), len(config.levels))
            sites_per_size = [int((~np.isnan(pool[:, level, :])).any(axis=0).sum()) for level in range(pool.shape[1])]
            assert sites_per_size == [3, 2, 1]
            own = np.stack([pool[:, level, level] for level in range(pool.shape[1])], axis=1)
            np.testing.assert_array_equal(own, config.s_blank[rule])


def test_d_step3_the_summary_records_windows_stations_and_pooling_per_station(detection_with_a_result):
    """With sigma_tot(Z) the main stations (60 trials per blank site: 180 windows for the smallest feature, 60 for the
    largest) pool their neighbors, and the summary says how many blank windows were used and which stations; the farthest
    station (300 trials per site) reaches the minimum on its own and is not pooled. gamma and its Clopper-Pearson interval
    stand beside tau."""
    minimum = PARAMS.detection_min_blank_windows
    mid = _d_row(detection_with_a_result, FEATURE_CUTOUT, STATION_MID_MM, RULE_PRIMARY)
    assert mid["tau_pooled"] and mid["blank_windows"] >= minimum and mid["tau_note"]
    stations = [float(z) for z in mid["blank_windows_stations"].split(",")]
    assert STATION_MID_MM in stations and len(stations) > 1
    far = _d_row(detection_with_a_result, FEATURE_CUTOUT, STATION_FAR_MM, RULE_PRIMARY)
    assert not far["tau_pooled"] and far["tau_note"] == ""
    assert far["blank_windows"] == PARAMS.detection_low_trials             # the largest feature: one blank site
    assert far["blank_windows_stations"] == f"{STATION_FAR_MM:g}"
    for row in (mid, far):
        assert row["gamma_lower"] <= row["gamma"] <= row["gamma_upper"] and math.isfinite(row["tau_mm"])
    detail = next(d for d in detection_with_a_result.details if d["station_z_mm"] == STATION_MID_MM
                  and d["rule"] == RULE_PRIMARY)
    assert detail["min_blank_windows"] == minimum
    assert all(level["blank_windows"] >= minimum and level["tau_pooled"] for level in detail["levels"])
    columns = _read_csv(detection_with_a_result.out_dir / detection.SUMMARY_FILE_NAME)[0]
    assert {"blank_windows", "blank_windows_stations", "tau_pooled", "gamma", "gamma_lower", "gamma_upper",
            "tau_mm"} <= set(columns)


def test_d_step3_without_analysis_a_the_short_stations_are_flagged(detection_result):
    """The shared session is run without Analysis A: the main stations cannot be pooled, keep their own windows, say so in
    ``tau_note`` and the notes; the farthest station does not need pooling."""
    mid = _d_row(detection_result, FEATURE_CUTOUT, STATION_MID_MM, RULE_PRIMARY)
    assert not mid["tau_pooled"] and "fewer than" in mid["tau_note"] and mid["blank_windows"] < PARAMS.detection_min_blank_windows
    far = _d_row(detection_result, FEATURE_CUTOUT, STATION_FAR_MM, RULE_PRIMARY)
    assert far["tau_note"] == "" and far["blank_windows"] >= PARAMS.detection_min_blank_windows
    assert any("DETECTION_MIN_BLANK_WINDOWS" in note for note in detection_result.notes)


def test_d_step9_the_bootstrap_rederives_tau_from_the_resampled_pooled_windows():
    """Step 9: a bootstrap resample that draws every pose once reproduces the data's thresholds and gamma, pooled over
    the same stations; the statistic's median tau is the median of the pooled thresholds, which differ from the plain
    per-station ones, so the resample really pools."""
    params = dataclasses.replace(PARAMS, detection_min_blank_windows=100)
    target, minimum = params.detection_false_alarm_target, params.detection_min_blank_windows
    rng = np.random.default_rng(17)
    poses, stations_mm, sigmas = 40, THRESHOLD_STATIONS_MM, (0.5, 1.0, 2.0)
    diameters = (4.0, 9.0)
    levels = [detection.LevelSpec("T", i, f"f{i}", f"b{i}", d, d + 3.0) for i, d in enumerate(diameters)]
    members = []
    for z, sigma in zip(stations_mm, sigmas):
        pool = np.full((poses, 2, 2), np.nan)
        pool[:, 0, :] = rng.normal(0.0, sigma, (poses, 2))             # the small window: both blank sites
        pool[:, 1, 1] = rng.normal(0.0, sigma, poses)                  # the large window: its own blank site only
        s_blank = np.stack([pool[:, 0, 0], pool[:, 1, 1]], axis=1)
        s_feature = rng.normal(2.0 * sigma, sigma, (poses, 2))
        config = detection.ConfigTrials(
            kind=FEATURE_CUTOUT, gap_mm=15.0, station_z_mm=z, field=0, levels=levels,
            pose_keys=[(k,) for k in range(poses)], sources=[detection.SOURCE_D] * poses,
            s_feature={RULE_PRIMARY: s_feature}, s_blank={RULE_PRIMARY: s_blank}, s_blank_pool={RULE_PRIMARY: pool})
        members.append(SimpleNamespace(config=config, rule=RULE_PRIMARY, sigma_tot_mm=sigma))
    windows = [detection.station_windows(m.config, RULE_PRIMARY, m.sigma_tot_mm) for m in members]
    thresholds = [calibrate_station_thresholds(windows, i, target, minimum) for i in range(len(members))]
    assert any(t.pooled for station in thresholds for t in station)
    plain = [calibrate_tau(m.config.s_blank[RULE_PRIMARY], target) for m in members]
    pooled_taus = [detection.thresholds_to_taus(station) for station in thresholds]
    assert not np.allclose(np.concatenate(plain), np.concatenate(pooled_taus))
    # One level per (station, feature) pair, with the counts that the pooled thresholds give.
    level_px = np.geomspace(2.0, 40.0, 2 * len(members))
    lookup = {(m, c): 2 * m + c for m in range(len(members)) for c in range(2)}
    successes = np.zeros(level_px.size)
    trials = np.zeros(level_px.size)
    hits = blank_trials = 0
    for m, member in enumerate(members):
        sf, sb = member.config.s_feature[RULE_PRIMARY], member.config.s_blank[RULE_PRIMARY]
        fd, bd, fv, bv = outcomes(sf, sb, pooled_taus[m])
        s, n, _, h, b = detection.counts_from_outcomes(fd, bd, fv, bv)
        for c in range(2):
            successes[lookup[(m, c)]], trials[lookup[(m, c)]] = s[c], n[c]
        hits, blank_trials = hits + h, blank_trials + b
    best = detection.fit_psychometric(level_px, np.maximum(successes, 1.0), trials, hits / blank_trials,
                                      detection._fit_parameters(params))
    statistic = detection._make_bootstrap_statistic(members, level_px, lookup, params, best,
                                                    wanted={slot: False for slot in range(4)})
    drawn = [(m, k) for m in range(len(members)) for k in range(poses)]
    values = statistic(drawn)
    gamma_slot, tau_slot = detection.BOOTSTRAP_STAT_NAMES.index("gamma"), detection.BOOTSTRAP_STAT_NAMES.index("tau_mm")
    assert values[gamma_slot] == pytest.approx(hits / blank_trials)
    assert values[tau_slot] == pytest.approx(float(np.median(np.concatenate(pooled_taus))))


def test_d_step4_pooled_d50_is_a_few_pixels_and_converts_to_mm_per_station(detection_result, cde_session):
    """Step 4 (pooled): D_50 of the cutouts (primary rule) is fitted on ln D_px over the 15 (feature, station) pairs, is a
    few pixels (0.5 to 8 px) and its bootstrap interval brackets it; every station reports the same D_px and the mm
    value D_px Z / f_x, inside the feature ladder at the reference station."""
    pooled = _pooled_row(detection_result, FEATURE_CUTOUT, RULE_PRIMARY)
    assert pooled["pairs"] == 3 * len(cde_fixture.STATIONS_MM)
    assert D50_PX_RANGE[0] <= pooled["d50_px"] <= D50_PX_RANGE[1]
    assert pooled["d50_lower_px"] <= pooled["d50_px"] <= pooled["d50_upper_px"]
    assert pooled["d50_status"] == "ok" and pooled["best_curve"] in ("logistic", "normal", "weibull")
    fx = cde_session.geometry.sensor_fx_px
    diameters = [f.diameter_mm for f in cde_session.targets.get("T5", 15.0).features if f.kind == FEATURE_CUTOUT]
    for station in cde_fixture.STATIONS_MM:
        row = _d_row(detection_result, FEATURE_CUTOUT, station, RULE_PRIMARY)
        assert row["d50_px"] == pytest.approx(pooled["d50_px"])
        assert row["d50_mm"] == pytest.approx(pooled["d50_px"] * station / fx)
        assert row["d50_mrad"] == pytest.approx(1000.0 * row["d50_mm"] / station)               # Step 10 units
        assert row["d50_px"] == pytest.approx(cde_session.geometry.diameter_in_pixels(row["d50_mm"], station))
        assert row["d50_lower_mm"] <= row["d50_mm"] <= row["d50_upper_mm"]
    assert min(diameters) < _d_row(detection_result, FEATURE_CUTOUT, STATION_MID_MM, RULE_PRIMARY)["d50_mm"] < max(diameters)


def test_d_step4_the_pooled_curve_uses_the_blank_sites_of_the_stations(detection_result):
    """Step 4: the pooled psychometric fit has gamma fixed at the group's blank-site false-alarm rate, the detail record
    lists the merged levels (neighboring features coincide in D_px after six stations), and the disks, which the
    matcher erases below 2.5 px just as it fills the cutouts, have a smaller D_50 in D_px than the cutouts here."""
    pooled = _pooled_row(detection_result, FEATURE_CUTOUT, RULE_PRIMARY)
    detail = next(d for d in detection_result.pooled_details if d["kind"] == FEATURE_CUTOUT and d["rule"] == RULE_PRIMARY)
    assert detail["gamma"] == pytest.approx(pooled["gamma"])
    levels = detail["levels_px"]
    assert 3 <= len(levels) < pooled["pairs"]                                  # coinciding pairs were merged
    assert all(a["d_px"] < b["d_px"] for a, b in zip(levels, levels[1:]))
    assert sum(level["trials"] for level in levels) == pooled["trials"]
    assert _pooled_row(detection_result, FEATURE_DISK, RULE_PRIMARY)["d50_px"] < pooled["d50_px"]


def _pooled_detail(result, kind, rule):
    """The pooled detail that belongs to the pooled row of a kind and rule (the details are appended in the same
    order as the rows; a detail that carries its own keys is matched by them)."""
    for detail in result.pooled_details:
        if detail.get("kind") == kind and detail.get("rule") == rule and detail.get("field", 0) == 0:
            return detail
    index = next(i for i, r in enumerate(result.pooled_rows)
                 if r["kind"] == kind and r["rule"] == rule and r["field"] == 0)
    return result.pooled_details[index]


def test_d5_measured_on_synthetic_counts():
    """Step 6 on fixed counts: D_5 is the largest level such that it and every smaller level have a corrected
    one-sided 95 percent upper bound at or below DETECTION_LOW_PROBABILITY (0.05). With gamma = 0.01 and 300 trials,
    0 detections pass (bound about 0.01) and 30 detections (10 percent) do not; the bracket is [D_5, next level]."""
    levels = np.array([1.0, 2.0, 4.0]); trials = np.array([300, 300, 300])
    bound = PARAMS.detection_low_probability
    assert bound == pytest.approx(0.05)
    result = detection.empirical_d5(levels, np.array([0, 0, 30]), trials, gamma=0.01, confidence=0.95, bound=bound)
    assert result["d5_empirical"] == pytest.approx(2.0) and result["d5_empirical_next"] == pytest.approx(4.0)
    # Each passing level has its corrected bound within 0.05, the failing one above it.
    assert [r["within_bound"] for r in result["levels"]] == [True, True, False]
    assert all(r["p_star_upper"] <= bound for r in result["levels"] if r["within_bound"])
    assert result["levels"][2]["p_star_upper"] > bound
    # A raw detection rate of 3 of 300 (the false-alarm rate, 1 percent) is within the 5 percent bound; 3 in 300
    # at the smallest level does not make the smaller levels fail.
    result = detection.empirical_d5(levels, np.array([3, 0, 30]), trials, gamma=0.01, confidence=0.95, bound=bound)
    assert result["d5_empirical"] == pytest.approx(2.0)
    # The smallest level already above the bound: D_5 is not demonstrated (NaN) and the bracket starts at that level.
    result = detection.empirical_d5(levels, np.array([30, 0, 0]), trials, gamma=0.01, confidence=0.95, bound=bound)
    assert math.isnan(result["d5_empirical"]) and result["d5_empirical_next"] == pytest.approx(1.0)
    # Every level within the bound: D_5 is the largest level, with no level above it.
    result = detection.empirical_d5(levels, np.array([0, 0, 0]), trials, gamma=0.01, confidence=0.95, bound=bound)
    assert result["d5_empirical"] == pytest.approx(4.0) and math.isnan(result["d5_empirical_next"])
    # The smallest level whose corrected probability is at or below 5 percent is reported, whatever the input order.
    shuffled = detection.empirical_d5(levels[::-1], np.array([30, 0, 0]), trials, gamma=0.01, confidence=0.95,
                                      bound=bound)
    assert shuffled["d5_empirical"] == pytest.approx(2.0)


def test_d0_is_predicted_from_the_fitted_curve():
    """Step 7: the predicted D_0 is the best fitted curve inverted at DETECTION_ZERO_PREDICTION_LEVEL (0.01), flagged
    as a prediction with a note; with no fit it is NaN and the note says so; a fit that did not converge gives NaN."""
    levels = np.geomspace(5.0, 80.0, 9)
    gamma, lapse, alpha, beta = 0.01, 0.0, math.log(20.0), 0.6
    corrected = 1.0 / (1.0 + np.exp(-(np.log(levels) - alpha) / beta))
    trials = np.full(levels.shape, 400)
    successes = np.rint(trials * (gamma + (1.0 - gamma - lapse) * corrected))
    with np.errstate(over="ignore"):                      # the Weibull shape overflows in far tails; still valid
        fits = fit_all_curves(levels, successes, trials, gamma, PsychometricFitParameters(lapse_rate_max=0.05))
    best = best_fit(fits)
    level = PARAMS.detection_zero_prediction_level
    assert level == pytest.approx(0.01)
    result = detection.predict_d0(fits, best, level, PARAMS.detection_low_probability)
    assert result["is_prediction"] is True
    assert result["d0"] == pytest.approx(best.threshold(level))             # the inverse of the fitted curve at 0.01
    assert result["d0"] < best.threshold(PARAMS.detection_low_probability) < best.threshold(0.5)
    assert result["model_min"] <= result["d0"] <= result["model_max"]
    assert "PREDICTION" in result["note"] and "not a measurement" in result["note"] and "extrapolation" in result["note"]
    # The curve really sits at the prediction level there (corrected probability 0.01).
    assert best.corrected_probability(result["d0"]) == pytest.approx(level, abs=1e-9)
    # No fit: NaN with the note saying so.
    missing = detection.predict_d0(None, None, level, PARAMS.detection_low_probability)
    assert math.isnan(missing["d0"]) and missing["is_prediction"] is True and "no psychometric fit" in missing["note"]
    # A fit that did not converge is not extrapolated.
    unconverged = dataclasses.replace(best, converged=False)
    failed = detection.predict_d0(fits, unconverged, level, PARAMS.detection_low_probability)
    assert math.isnan(failed["d0"]) and "did not converge" in failed["note"]


def test_d_step5_to_7_minimums_are_ordered(detection_result):
    """Steps 5 to 7: D_10 <= D_50; the measured D_5 (the 300 trials at the farthest station demonstrate at most 5 percent
    detection of the smallest feature there) is a D_px bracket [D_5, next level] converted to mm per station and follows
    the rule of Step 6; the predicted D_0 is the fitted curve at 0.01, flagged as a prediction, below D_10; the model
    range of D_10 contains the best-fit D_10; the isotonic D_50 is within a factor 1.5 of the fitted one; the floor
    model gives a D_0 below D_10."""
    for rule in (RULE_PRIMARY, RULE_INCLUSIVE):
        pooled = _pooled_row(detection_result, FEATURE_CUTOUT, rule)
        assert pooled["d10_px"] <= pooled["d50_px"]
        # The measured 5 percent point follows the rule of Step 6, not a lucky draw: D_5 is the largest level such that
        # it and every smaller level have a corrected upper bound within DETECTION_LOW_PROBABILITY.
        levels = _pooled_detail(detection_result, FEATURE_CUTOUT, rule)["d5_empirical"]["levels"]
        assert levels, "the far-station trials must give at least one level with trials"
        passing = []
        for level in sorted(levels, key=lambda r: r["d_px"]):
            if not level["within_bound"]:
                break
            passing.append(level["d_px"])
        if passing:
            assert pooled["d5_empirical_px"] == pytest.approx(max(passing))
            assert pooled["d5_empirical_px"] <= pooled["d10_px"]
            assert pooled["d5_empirical_px"] < pooled["d5_empirical_next_px"]                           # the bracket
        else:
            assert math.isnan(pooled["d5_empirical_px"])
            assert pooled["d5_empirical_next_px"] == pytest.approx(min(r["d_px"] for r in levels))
        # The predicted 0 percent point: the best fitted curve at DETECTION_ZERO_PREDICTION_LEVEL, flagged.
        assert pooled["d0_is_prediction"] is True and "PREDICTION" in pooled["d0_predicted_note"]
        assert math.isfinite(pooled["d0_predicted_px"]) and pooled["d0_predicted_px"] < pooled["d10_px"]
        assert pooled["d0_predicted_model_min_px"] <= pooled["d0_predicted_px"] <= pooled["d0_predicted_model_max_px"]
        assert pooled["d10_model_min_px"] <= pooled["d10_px"] + 1e-9 <= pooled["d10_model_max_px"] + 2e-9
        ratio = pooled["d50_isotonic_px"] / pooled["d50_px"]
        assert 1.0 / ISOTONIC_FACTOR < ratio < ISOTONIC_FACTOR
        assert pooled["d0_model_status"] in ("ok", "not estimable")
        if pooled["d0_model_status"] == "ok":
            assert pooled["d0_model_px"] <= pooled["d10_px"]
        for station in cde_fixture.STATIONS_MM:
            row = _d_row(detection_result, FEATURE_CUTOUT, station, rule)
            assert row["d10_mm"] <= row["d50_mm"]
            assert row["d0_is_prediction"] is True and row["d0_predicted_note"] == pooled["d0_predicted_note"]
            assert row["d0_predicted_px"] == pytest.approx(pooled["d0_predicted_px"])
            assert row["d0_predicted_mm"] == pytest.approx(pooled["d0_predicted_px"] * station / cde_fixture_fx())
            if passing:
                assert row["d5_empirical_px"] == pytest.approx(pooled["d5_empirical_px"])
                assert row["d5_empirical_mm"] == pytest.approx(pooled["d5_empirical_px"] * station / cde_fixture_fx())
                assert row["d5_empirical_mm"] < row["d5_empirical_next_mm"]
            else:
                assert math.isnan(row["d5_empirical_px"])


def test_fitted_d5_against_the_empirical_bracket_on_synthetic_counts():
    """On a fixed count table (300 trials per level, a logistic with gamma = 0.01 whose expected counts are used) the
    fitted D_5 is the inverse of the best fitted curve at 0.05 (the corrected probability there is 0.05), lies in the
    tested range, and is at or above the empirical level, which is the largest level whose corrected one-sided upper
    bound is at most 0.05. The empirical bracket is CONSERVATIVE: the upper bound is above the point estimate, so the
    first level that fails the bound can lie below the fitted D_5 (the next level is not an upper limit of it)."""
    levels = np.geomspace(0.8, 30.0, 14)
    gamma, alpha, beta = 0.01, math.log(6.0), 0.5
    corrected = 1.0 / (1.0 + np.exp(-(np.log(levels) - alpha) / beta))
    trials = np.full(levels.shape, PARAMS.detection_low_trials)
    successes = np.rint(trials * (gamma + (1.0 - gamma) * corrected))
    with np.errstate(all="ignore"):
        fits = fit_all_curves(levels, successes, trials, gamma, PsychometricFitParameters(lapse_rate_max=0.05))
    best = best_fit(fits)
    fitted = best.threshold(PARAMS.detection_low_probability)
    assert best.corrected_probability(fitted) == pytest.approx(PARAMS.detection_low_probability, abs=1e-9)
    assert levels[0] < fitted < levels[-1]
    empirical = detection.empirical_d5(levels, successes, trials, gamma, PARAMS.confidence_level,
                                       PARAMS.detection_low_probability)
    assert empirical["d5_empirical"] <= fitted
    assert empirical["d5_empirical"] < empirical["d5_empirical_next"]
    # The model range over the three shapes brackets the best fit's value.
    shapes = [f.threshold(PARAMS.detection_low_probability) for f in fits.values()]
    assert min(shapes) <= fitted <= max(shapes)


def test_d5_fitted_sits_above_the_empirical_level_and_has_intervals(detection_result):
    """Section 13: D_5 is read from the best fitted curve at DETECTION_LOW_PROBABILITY (with its model range over the
    three shapes and a bootstrap interval) and the empirical bracket is reported beside it: the fitted D_5 is at or above
    the empirical level (a demonstrated at-most-5-percent level); the predicted D_0 lies below D_5; both bootstrap
    intervals are finite and contain their point estimates when the fit converges."""
    for rule in (RULE_PRIMARY, RULE_INCLUSIVE):
        pooled = _pooled_row(detection_result, FEATURE_CUTOUT, rule)
        assert pooled["d5_status"] == "ok" and math.isfinite(pooled["d5_px"])
        low = pooled["d5_empirical_px"]
        if math.isfinite(low):
            assert low <= pooled["d5_px"]
        assert pooled["d5_model_min_px"] <= pooled["d5_px"] <= pooled["d5_model_max_px"]
        assert pooled["d0_predicted_px"] < pooled["d5_px"] < pooled["d10_px"]
        for name in ("d5", "d0_predicted"):
            lower, upper, value = pooled[f"{name}_lower_px"], pooled[f"{name}_upper_px"], pooled[f"{name}_px"]
            assert math.isfinite(lower) and math.isfinite(upper) and lower < upper, name
            assert lower <= value <= upper, name
        # The per-station rows carry the same values converted to mm.
        for station in cde_fixture.STATIONS_MM:
            row = _d_row(detection_result, FEATURE_CUTOUT, station, rule)
            assert row["d5_mm"] == pytest.approx(pooled["d5_px"] * station / cde_fixture_fx())
            assert row["d5_lower_mm"] < row["d5_mm"] < row["d5_upper_mm"]
            assert row["d0_predicted_lower_mm"] < row["d0_predicted_mm"] < row["d0_predicted_upper_mm"]
    names = _pooled_detail(detection_result, FEATURE_CUTOUT, RULE_PRIMARY)["bootstrap"]["names"]
    assert names == list(detection.BOOTSTRAP_STAT_NAMES) and "d5_px" in names and "d0_predicted_px" in names


def cde_fixture_fx() -> float:
    """f_x of the test sensor (pixels)."""
    return cde_fixture.test_geometry().sensor_fx_px


def test_d_step1_independence_is_computed(detection_result):
    """Step 1: the lag-1 autocorrelation of each feature's outcome sequence and the neighbor phi coefficients are
    computed against +/- 2 / sqrt(n); the flag is a bool and the tests are listed (a station where every feature is
    always or never detected has nothing to test)."""
    for row, detail in zip(detection_result.rows, detection_result.details):
        assert isinstance(row["independence_ok"], bool)
        independence = detail["independence"]
        for test in independence["lag1"]:
            assert test["bound"] == pytest.approx(2.0 / math.sqrt(test["n"]))
        if row["station_z_mm"] == STATION_MID_MM and row["kind"] == FEATURE_CUTOUT:
            assert independence["tested"] > 0 and independence["lag1"]
    assert _d_row(detection_result, FEATURE_CUTOUT, STATION_MID_MM, RULE_PRIMARY)["independence_ok"]


def test_d_step2_no_read_inclusive_rule_is_not_worse(detection_result):
    """Step 2: counting no-reads as candidates can only add detections, so the inclusive D_50 is at most the primary
    one for cutouts (the synthetic matcher fills holes with front values instead of leaving no-reads, so here the two
    rules coincide up to fit noise; the inequality is the property)."""
    primary_pooled = _pooled_row(detection_result, FEATURE_CUTOUT, RULE_PRIMARY)
    inclusive_pooled = _pooled_row(detection_result, FEATURE_CUTOUT, RULE_INCLUSIVE)
    assert inclusive_pooled["d50_px"] <= primary_pooled["d50_px"] * (1.0 + 1e-9)
    for station in (STATION_MID_MM, STATION_FAR_MM):
        primary = _d_row(detection_result, FEATURE_CUTOUT, station, RULE_PRIMARY)
        inclusive = _d_row(detection_result, FEATURE_CUTOUT, station, RULE_INCLUSIVE)
        assert inclusive["d50_mm"] <= primary["d50_mm"] * (1.0 + 1e-9)
        assert inclusive["gamma"] >= primary["gamma"] - 1e-12
    assert not any(r["kind"] == FEATURE_DISK and r["rule"] == RULE_INCLUSIVE for r in detection_result.rows)


def test_d_step8_geometric_limit_and_reuse(detection_result):
    """Step 8 and Section 8 Step 6: the geometric limit (A_geo reaching zero) is far below D_50, the projector version
    is at least the cameras-only one, and the first frames of the C poses of the matching configuration are counted as
    reused trials."""
    row = _d_row(detection_result, FEATURE_CUTOUT, STATION_MID_MM, RULE_PRIMARY)
    assert 0.0 < row["d0_geometric_mm"] < row["d50_mm"]
    assert row["d0_geometric_mm"] >= row["d0_geometric_cameras_mm"] - 1e-9
    assert row["trials_reused"] > 0 and row["trials_new"] == cde_fixture.DETECTION_POSES
    far = _d_row(detection_result, FEATURE_CUTOUT, STATION_FAR_MM, RULE_PRIMARY)
    assert far["trials_new"] == cde_fixture.DETECTION_POSES_FAR
    disk = _d_row(detection_result, FEATURE_DISK, STATION_MID_MM, RULE_PRIMARY)
    assert math.isnan(disk["d0_geometric_mm"])                                      # cutouts only


def test_d_step11_csv_and_figures(detection_result):
    """Step 11: one summary row per configuration and rule (cutouts at five stations with both rules, disks at five with
    the primary rule), one pooled row per (kind, gap, field, rule), the documented columns, the details and the figures."""
    rows = _read_csv(detection_result.out_dir / detection.SUMMARY_FILE_NAME)
    assert len(rows) == 3 * len(cde_fixture.STATIONS_MM)
    assert list(rows[0].keys()) == list(detection.SUMMARY_COLUMNS)
    for name in ("d50_mm", "d50_px", "d50_mrad", "d10_px", "d5_mm", "d5_px", "d5_mrad", "d5_lower_px", "d5_upper_px", "d5_empirical_mm", "d5_empirical_next_px", "d0_predicted_lower_px", "d0_predicted_upper_mm", "d5_status", "d0_predicted_mm", "d0_predicted_px", "d0_is_prediction",
                 "d0_predicted_note", "d0_model_mrad",
                 "d0_geometric_px", "gamma_lower", "tau_mm", "independence_ok"):
        assert name in rows[0]
    pooled = _read_csv(detection_result.out_dir / detection.POOLED_FILE_NAME)
    assert len(pooled) == 3 and list(pooled[0].keys()) == list(detection.POOLED_COLUMNS)
    assert all(r["d0_is_prediction"] == "True" and "PREDICTION" in r["d0_predicted_note"] for r in pooled)
    assert all(r["d0_is_prediction"] == "True" for r in rows)
    document = json.loads((detection_result.out_dir / detection.DETAILS_FILE_NAME).read_text())
    assert len(document["configurations"]) == 3 * len(cde_fixture.STATIONS_MM) and len(document["pooled"]) == 3
    assert "levels" in document["configurations"][0] and "bootstrap" in document["pooled"][0]
    assert document["pooled"][0]["bootstrap"]["stratified_by"] == "station"
    for name in ("D_psychometric_cutout_G15_F0.png", "D_psychometric_disk_G15_F0.png", "D_minimum_vs_z.png",
                 detection.OVERLAP_FILE_NAME):
        assert (detection_result.out_dir / name).exists(), name
    terms = detection_result.forward_model_terms()
    assert set(terms) == {"d50_px", "d10_px"} and terms["d10_px"] < terms["d50_px"]


def test_d_step5_d50_d10_and_d5_carry_a_model_range_over_the_three_shapes(detection_result, cde_session):
    """Section 13, Model dependence: D_50, D_10 and D_5 all come from the best of the three shapes by deviance, and each
    has the range across the shapes as its model uncertainty (``*_model_min`` / ``*_model_max``), in px (pooled rows) and
    in mm, px and mrad per station (summary rows). The best shape's value lies inside its range."""
    fx = cde_session.geometry.sensor_fx_px
    pooled = [r for r in detection_result.pooled_rows if r["d50_status"] == "ok" and r["d10_status"] == "ok"]
    assert pooled
    checked_d5 = 0
    for row in pooled:
        names = ["d50", "d10"] + (["d5"] if row["d5_status"] == "ok" else [])
        checked_d5 += row["d5_status"] == "ok"
        for name in names:
            low, high, value = row[f"{name}_model_min_px"], row[f"{name}_model_max_px"], row[f"{name}_px"]
            assert math.isfinite(low) and math.isfinite(high)
            assert low <= value * (1.0 + 1e-9) and value <= high * (1.0 + 1e-9), (row["kind"], name)
    assert checked_d5
    row = pooled[0]
    station_rows = [r for r in detection_result.rows if (r["kind"], r["gap_mm"], r["field"], r["rule"])
                    == (row["kind"], row["gap_mm"], row["field"], row["rule"])]
    for station_row in station_rows:
        for name in ("d50", "d10"):
            assert station_row[f"{name}_model_min_mm"] == pytest.approx(
                row[f"{name}_model_min_px"] * station_row["station_z_mm"] / fx)
            assert station_row[f"{name}_model_max_px"] == pytest.approx(row[f"{name}_model_max_px"])
    header = _read_csv(detection_result.out_dir / detection.POOLED_FILE_NAME)[0]
    assert {f"{name}_model_{end}_px" for name in ("d50", "d10", "d5") for end in ("min", "max")} <= set(header)


def test_d_prior_expectation_columns_give_d50_and_the_predicted_d0_over_the_expected_minimum(detection_result):
    """Section 13, intro: the fitted D_50 and the predicted D_0 are reported against the expected minimum
    (``expected_d0_px``, the laser-pencil model of Section 3.2) as ``d50_over_expected`` and
    ``d0_predicted_over_expected``, per pooled configuration and (the same value) per station; NaN where the minimum is
    not estimated. The details say what the prior is."""
    expected = PARAMS.expected_d0_px
    assert expected == pytest.approx(7.0)
    seen = 0
    for row in detection_result.pooled_rows:
        for ratio, minimum in (("d50_over_expected", "d50_px"), ("d0_predicted_over_expected", "d0_predicted_px")):
            if math.isfinite(row[minimum]):
                assert row[ratio] == pytest.approx(row[minimum] / expected)
                seen += 1
            else:
                assert math.isnan(row[ratio])
    assert seen
    for row in detection_result.rows:
        pooled = next(p for p in detection_result.pooled_rows if (p["kind"], p["gap_mm"], p["field"], p["rule"])
                      == (row["kind"], row["gap_mm"], row["field"], row["rule"]))
        for ratio in detection.EXPECTED_RATIO_COLUMNS:
            assert row[ratio] == pytest.approx(pooled[ratio], nan_ok=True)
    document = json.loads((detection_result.out_dir / detection.DETAILS_FILE_NAME).read_text())
    assert "laser-pencil" in document["prior_expectation"] and f"{expected:g} px" in document["prior_expectation"]
    assert all(p["prior_expectation"]["expected_d0_px"] == expected for p in document["pooled"])
    for name in ("pooled", "summary"):
        file_name = detection.POOLED_FILE_NAME if name == "pooled" else detection.SUMMARY_FILE_NAME
        assert set(detection.EXPECTED_RATIO_COLUMNS) <= set(_read_csv(detection_result.out_dir / file_name)[0])


def test_d_step10_overlap_file_carries_kind_gap_field_rule_and_the_pair(detection_result):
    """Section 13, Units: D_overlap_test.csv has one row per pair of neighboring features, with the kind, the gap, the field
    position and the rule beside the pair."""
    rows = _read_csv(detection_result.out_dir / detection.OVERLAP_FILE_NAME)
    assert len(rows) == len(detection_result.overlap_rows) > 0
    assert {"kind", "gap_mm", "field", "rule", "feature_small", "feature_large"} <= set(rows[0])
    assert {r["kind"] for r in rows} == {FEATURE_DISK, FEATURE_CUTOUT}
    assert {r["rule"] for r in rows} == {RULE_PRIMARY, RULE_INCLUSIVE}
    assert all(r["gap_mm"] and r["field"] != "" and r["feature_small"] and r["feature_large"] for r in rows)


def test_forward_model_file_has_the_d_terms_and_no_term_of_c(area_result, detection_result):
    """Section 12, Step 13: no term of C enters forward_model_parameters.json (the edge bias stays in C's own outputs and
    in E's cross-check), while D's ``d50_px`` and ``d10_px`` do (Section 13, Outputs)."""
    from sensorperf.analysis import forward_model
    document = forward_model.assemble({"C": area_result, "D": detection_result})
    assert not any(key.startswith("edge_bias") for key in document["terms"])
    assert {"d50_px", "d10_px"} <= set(document["terms"])
    assert document["provenance"]["d50_px"] == "analysis D"
    assert "analysis C" not in document["provenance"].values()


def test_d_step12_overlap_test_between_neighboring_features(detection_result):
    """Redesign note Section 5 (D): the overlap test compares the corrected detection curves of neighboring features over
    the D_px they share: two pairs per group, each with points of both curves in the range, a mean difference, a bootstrap
    interval and the agreement flag. In the synthetic data the depth noise grows with Z, and at the same D_px the larger
    feature is seen from farther away, so it detects less than the smaller one: the curves disagree (mean difference of
    larger minus smaller below zero) and the note says why."""
    tests = detection_result.overlap_rows
    assert len(tests) == 3 * 2
    for t in tests:
        assert t["status"] == "ok" and t["points_small"] >= 2 and t["points_large"] >= 2
        assert t["interval_lower"] <= t["mean_difference"] <= t["interval_upper"] or not t["agrees"]
        assert t["agrees"] == (t["interval_lower"] <= 0.0 <= t["interval_upper"])
    disagreeing = [t for t in tests if not t["agrees"]]
    assert disagreeing and all("sigma_tot" in t["note"] for t in disagreeing)
    assert all(t["mean_difference"] < 0.0 for t in disagreeing)                        # the larger feature detects less


def test_d_step13_noise_covariate_regression(cde_session, tmp_path):
    """Redesign note Section 5 (D): with sigma_tot per station from Analysis A the pooled counts are regressed on ln D_px
    and ln sigma_tot(Z) (logistic); the size term is positive (larger features are detected more) and finite
    coefficients, standard errors and a likelihood-ratio p-value are reported. Without A the regression is skipped with a
    note, and the overlap test then carries no sigma_tot values."""
    options = detection.DetectionOptions(bootstrap_resamples=2, include_reused=False)
    cutout_trials = [r for r in cde_session.records if r.procedure == "D" and r.target_id == "T5"]
    session = Session(root=cde_session.root, params=cde_session.params, sensor=cde_session.sensor,
                      registration=cde_session.registration, targets=cde_session.targets, records=cutout_trials)
    without = detection.run_detection(session, Path("."), {}, options)
    assert all(r["regression_status"].startswith("no sigma_tot") for r in without.pooled_rows)
    assert any("noise covariate" in note for note in without.notes)
    with_a = detection.run_detection(session, Path("."), {"A": FakeNoiseResult(cde_fixture.STATIONS_MM)}, options)
    row = _pooled_row(with_a, FEATURE_CUTOUT, RULE_PRIMARY)
    assert row["regression_status"] in ("ok",) or "separate" in row["regression_status"]
    assert row["regression_observations"] == 3 * len(cde_fixture.STATIONS_MM)
    if row["regression_status"] == "ok":
        assert row["regression_b_ln_dpx"] > 0.0 and row["regression_se_ln_dpx"] > 0.0
        assert math.isfinite(row["regression_b_ln_sigma"]) and row["regression_se_ln_sigma"] > 0.0
        assert 0.0 <= row["regression_p_noise"] <= 1.0
    sigma = [t for t in with_a.overlap_rows if t["sigma_tot_small_mm"] is not None]
    assert sigma and all(t["sigma_tot_large_mm"] > t["sigma_tot_small_mm"] for t in sigma)  # the larger one is seen from afar


def test_d_step9_bootstrap_is_reduced_for_few_poses(cde_session):
    """Step 9: a group with fewer than full_bootstrap_min_poses poses uses the reduced resample count and the details say
    so; a group with too few trials per pair reports 'too few trials' without failing. (Run on the two farthest
    stations only, to keep the test short.)"""
    far = [r for r in cde_session.records if r.procedure == "D" and r.station_z_mm in cde_fixture.STATIONS_MM[-2:]]
    small = Session(root=cde_session.root, params=cde_session.params, sensor=cde_session.sensor,
                    registration=cde_session.registration, targets=cde_session.targets, records=far)
    options = detection.DetectionOptions(reduced_bootstrap_resamples=7, full_bootstrap_min_poses=10 ** 6,
                                         include_reused=False)
    result = detection.run_detection(small, Path("."), {}, options)
    row = next(r for r in result.pooled_rows if r["rule"] == RULE_PRIMARY)
    assert row["bootstrap_resamples"] == 7
    assert result.pooled_details[0]["bootstrap"]["reduced"] is True
    strict = detection.DetectionOptions(min_trials_per_level=10 ** 6, bootstrap_resamples=5, include_reused=False)
    result = detection.run_detection(small, Path("."), {}, strict)
    assert all(r["d50_status"] == "too few trials" and math.isnan(r["d50_mm"]) for r in result.rows)
    assert all(r["d50_status"] == "too few trials" and math.isnan(r["d50_px"]) for r in result.pooled_rows)


# ---------------------------------------------------------------------------
# Analysis E (Section 14)
# ---------------------------------------------------------------------------
def _e_rows(result, **criteria):
    return [r for r in result.rows if all(r[k] == v for k, v in criteria.items())]


def test_e_step4_fabricated_reads_exist_and_beta_is_computed(boundary_result):
    """Step 4: where geometry says no stereo read is possible (V = 0: the back-plate sliver hidden from the right camera
    or the projector) the matcher still reads (fabricated reads, R_fab > 0) and it never drops out where V = 1
    (R_drop = 0), so beta_read is computed and positive; the widths are non-negative and below the band width."""
    band = 2.0 * boundary_result.details["bins"]["band_px"]
    pooled = _e_rows(boundary_result, source="B", target_id="all", visibility_rule="projector")
    assert pooled
    for row in pooled:
        assert row["r_fab"] > 0.0
        assert math.isfinite(row["beta_read"]) and row["beta_read"] > 0.0
    for row in boundary_result.rows:
        for key in ("w_fab_px", "w_drop_px"):
            assert 0.0 <= row[key] < band
    projector = _e_rows(boundary_result, source="B", target_id="T3a", orientation="along", gap_mm=15.0,
                        station_z_mm=STATION_EDGE_MM, visibility_rule="projector")[0]
    cameras = _e_rows(boundary_result, source="B", target_id="T3a", orientation="along", gap_mm=15.0,
                      station_z_mm=STATION_EDGE_MM, visibility_rule="cameras")[0]
    assert projector["w_fab_px"] > 0.0 and cameras["w_fab_px"] > 0.0                # both rules reported


def test_e_step5_near_surface_preference(boundary_result):
    """Step 5: front_preference > 1 makes the matcher read the front surface beyond the true edge (front reads at
    s < 0 outnumber back reads at s > 0): pi_near > 0, the half-height crossing s_50 < 0 (the edge appears shifted onto
    the back side) and the mean h of intermediate reads is between 0 and 1."""
    rows = _e_rows(boundary_result, source="B", target_id="all", visibility_rule="projector")
    assert any(math.isfinite(r["pi_near"]) for r in rows)
    for row in rows:
        if math.isfinite(row["pi_near"]):        # no misread pixels at all (as at the nearest station) leaves pi undefined
            assert row["pi_near"] > 0.5 and row["pi_lower"] <= row["pi_near"] <= row["pi_upper"]
        assert row["s50_px"] < 0.0
        assert 0.0 < row["h_mid_mean"] < 1.0
    terms = boundary_result.forward_model_terms()
    assert set(terms) == {"w_fab_px", "w_drop_px", "pi_near", "beta_read"}
    assert terms["pi_near"] > 0.0 and terms["w_fab_px"] > 0.0
    checks = boundary_result.details["cross_checks"]["own_s50_vs_pi"]
    assert checks and all(c["signs_consistent"] for c in checks if math.isfinite(c["pi_near"]))


def test_e_step3_profiles_sum_to_one(boundary_result):
    """Step 3: in every bin with data the four outcome profiles P_front, P_back, P_mid and P_none sum to 1."""
    profiles = boundary_result.details["profiles"]
    assert profiles
    for profile in profiles:
        counts = np.asarray(profile["count"])
        total = sum(np.nan_to_num(np.asarray(profile[name], dtype=float)) for name in ("front", "back", "mid", "none"))
        assert np.allclose(total[counts > 0], 1.0)
        assert (counts > 0).sum() > 10


def test_e_step7_breakdowns_and_csv(boundary_result):
    """Step 7 and Step 8: the CSV breaks the result down by source, target (polarity), orientation, Z, G and visibility
    rule, with bootstrap intervals; both orientations and both polarities are present."""
    rows = _read_csv(boundary_result.out_dir / boundary.SUMMARY_FILE_NAME)
    assert list(rows[0].keys()) == list(boundary.SUMMARY_COLUMNS)
    assert {r["orientation"] for r in boundary_result.rows} >= {"along", "across", "all", "circular"}
    assert {r["polarity"] for r in boundary_result.rows if r["target_id"] != "all"} == {"raised", "window"}
    assert {r["visibility_rule"] for r in boundary_result.rows} == {"projector", "cameras"}
    assert {r["source"] for r in boundary_result.rows} == {"B", "C"}
    along = _e_rows(boundary_result, source="B", target_id="T3b", orientation="along", station_z_mm=STATION_MID_MM,
                    visibility_rule="projector")[0]
    assert along["poses"] == cde_fixture.EDGE_JITTER_POSES + 1 and along["pixels"] > 1000
    assert math.isfinite(along["beta_lower"]) or along["beta_read"] == 1.0
    for name in ("E_outcome_profiles.png", "E_beta_pi_vs_z.png", "E_feature_scale_vs_dpx.png",
                 boundary.DETAILS_FILE_NAME):
        assert (boundary_result.out_dir / name).exists(), name


def test_e_step6_fill_in_is_larger_for_the_smallest_cutouts(boundary_result):
    """Step 6: P(fill-in) (the majority outcome inside the true outline is a front read) is larger for the smallest
    cutouts than for the largest ones, which are reported correctly as back reads; disks mirror this with erasure; and
    the two detection rules are reported per diameter."""
    table = boundary_result.details["feature_scale"]
    cutouts = sorted((r for r in table if r["kind"] == FEATURE_CUTOUT), key=lambda r: r["d_px"])
    assert {r["station_z_mm"] for r in cutouts} == set(cde_fixture.STATIONS_MM)           # pooled over all stations
    small = np.mean([r["p_fill_in"] for r in cutouts if r["d_px"] <= SMALL_FEATURE_D_PX])
    large = np.mean([r["p_fill_in"] for r in cutouts if r["d_px"] >= LARGE_FEATURE_D_PX])
    assert small > large and small > 0.9 and large < 0.2
    assert cutouts[-1]["p_correct"] > cutouts[0]["p_correct"]
    for r in cutouts:
        assert r["p_correct"] + r["p_wrong_surface"] + r["p_no_read"] == pytest.approx(1.0)
        assert math.isfinite(r["detect_primary"]) and math.isfinite(r["detect_inclusive"])
        assert r["rule_difference"] >= -1e-12
    disks = sorted((r for r in table if r["kind"] == FEATURE_DISK), key=lambda r: r["d_px"])
    assert disks[0]["p_erased"] > disks[-1]["p_erased"]


def test_e_sigma_source_is_stated_and_a_is_preferred(cde_session, tmp_path):
    """Step 2: without Analysis A sigma_tot is estimated locally (stated); when A offers rows with sigma_tot_mm at a
    station, those are used."""
    options = boundary.BoundaryOptions(bootstrap_resamples=2, feature_scale=False)
    local = boundary.run_boundary_bias(cde_session, tmp_path, {}, options)
    assert all("local robust std" in s["origin"] for s in local.details["sigma_tot"])

    class FakeA:
        summary_rows = [{"station_z_mm": STATION_MID_MM, "sigma_tot_mm": 1.1, "field": 0, "subseries": "main"},
                        {"station_z_mm": STATION_FAR_MM, "sigma_tot_mm": 2.0, "field": 0, "subseries": "main"}]

    with_a = boundary.run_boundary_bias(cde_session, tmp_path, {"A": FakeA()}, options)
    assert all(s["origin"] == "analysis A" for s in with_a.details["sigma_tot"])
    assert {s["sigma_tot_mm"] for s in with_a.details["sigma_tot"]} == {1.1, 2.0}       # the nearest of A's stations


def test_e_returns_none_without_frames(cde_session, tmp_path):
    """The analyses return None when the session has no frames of their procedure."""
    stripped = Session(root=cde_session.root, params=cde_session.params, sensor=cde_session.sensor,
                       registration=cde_session.registration, targets=cde_session.targets, records=[])
    assert boundary.run_boundary_bias(stripped, tmp_path, {}) is None
    assert detection.run_detection(stripped, tmp_path, {}) is None
    assert area.run_area(stripped, tmp_path, {}) is None


# ---------------------------------------------------------------------------
# Command line on the standard quick session
# ---------------------------------------------------------------------------
def test_cli_analyze_c_d_e_on_the_standard_quick_session(tmp_path):
    """Entry-point contract: `python3 -m sensorperf.cli.analyze --only C D E` exits 0 on the standard
    `simulate --quick --seed 1` session (whose small arrays are sub-pixel, so D reports 'too few trials' or an
    unbracketed threshold rather than crashing) and forward_model_parameters.json carries w_fab_px and pi_near."""
    session = tmp_path / "quick"
    simulate = subprocess.run([sys.executable, "-m", "sensorperf.cli.simulate", "--out", str(session), "--quick",
                               "--seed", "1"], cwd=REPO_ROOT, capture_output=True, text=True)
    assert simulate.returncode == 0, simulate.stderr
    analyze = subprocess.run([sys.executable, "-m", "sensorperf.cli.analyze", "--session", str(session),
                              "--only", "C", "D", "E"], cwd=REPO_ROOT, capture_output=True, text=True)
    assert analyze.returncode == 0, analyze.stdout + analyze.stderr
    document = json.loads((session / "analysis" / FORWARD_MODEL_FILE_NAME).read_text())
    assert "w_fab_px" in document["terms"] and "pi_near" in document["terms"]
    assert not any(key.startswith("edge_bias") for key in document["terms"])     # no term of C (Section 12, Step 13)
    for name in ("C_area_summary.csv", "D_detect_summary.csv", "E_boundary_bias.csv"):
        assert (session / "analysis" / name).exists(), name
    rows = _read_csv(session / "analysis" / "D_detect_summary.csv")
    assert rows and all(r["d50_status"] in ("too few trials", "ok", "never reached in the tested range",
                                            "already exceeded at the smallest level") for r in rows)
