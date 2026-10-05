"""
Tests of Analyses C, D and E (sensorperf.analysis.area, detection, boundary; procedure document Sections 12, 13, 14).

Every test names the step of the procedure it checks. Two sessions are rendered with the INDICATIVE synthetic sensor
(160 x 120 px; tests/cde_fixture.py explains why the arrays are re-laid out in pixels of the test sensor):
a custom session with B edges, C arrays (field and open-background variants included) and D trials, shared by the
analysis tests, and the standard ``sensorperf.cli.simulate --quick`` session for the command-line check.

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
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from sensorperf.analysis import area, boundary, detection
from sensorperf.analysis.detection import (
    RULE_INCLUSIVE, RULE_PRIMARY, calibrate_tau, detected_at, outcomes, window_statistic,
)
from sensorperf.geometry.targets import FEATURE_CUTOUT, FEATURE_DISK
from sensorperf.io.manifest import SUBSERIES_FIELD, SUBSERIES_JITTER, SUBSERIES_OPEN
from sensorperf.io.session import FORWARD_MODEL_FILE_NAME, Session
from sensorperf.stats.intervals import clopper_pearson

import cde_fixture

REPO_ROOT = Path(__file__).resolve().parents[1]
"""The repository root (working directory of the command-line test)."""
STATION_MID_MM = 750.0
STATION_FAR_MM = 1000.0
"""The stations of the test sessions."""
DETECTION_BOOTSTRAP_RESAMPLES = 60
BOUNDARY_BOOTSTRAP_RESAMPLES = 200
"""Reduced resample counts of the tests (the analyses default to BOOTSTRAP_RESAMPLES = 2000)."""
LARGEST_CUTOUT_RATIO_RANGE = (0.5, 1.1)
"""Test C1: the sensed fraction of the largest cutouts (b is about -1 to -1.5 px, so a 12 to 16 px hole keeps 55 to 75 percent)."""
SMALL_CUTOUT_D_PX = 3.0
"""Test C2: cutouts up to this many pixels across are filled in completely."""
SMALL_CUTOUT_RATIO_MAX = 0.05
"""Test C2: their sensed area is practically zero."""
CONTOUR_AGREEMENT = 0.25
"""Test C4: allowed relative difference between the contour and the pixel-count areas of the largest features."""
D50_PX_RANGE = (0.5, 6.0)
"""Test D2: a few pixels."""
ISOTONIC_FACTOR = 1.5
"""Test D5: allowed ratio between the isotonic and the fitted D_50."""
SMALLEST_FIELD_CUTOUT_LEVELS = 2
"""Test E5: how many of the smallest and of the largest cutouts are compared."""


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
def boundary_result(cde_session, tmp_path_factory):
    """Analysis E on the custom session (reduced bootstrap), with its outputs written."""
    options = boundary.BoundaryOptions(bootstrap_resamples=BOUNDARY_BOOTSTRAP_RESAMPLES)
    result = boundary.run_boundary_bias(cde_session, tmp_path_factory.mktemp("e_out"), {}, options)
    out = tmp_path_factory.mktemp("e_files")
    boundary.write_outputs(result, out)
    result.out_dir = out
    return result


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _main_rows(result, kind, station):
    return sorted((r for r in result.rows if r["kind"] == kind and r["station_z_mm"] == station
                   and r["subseries"] == SUBSERIES_JITTER and r["field"] == 0), key=lambda r: r["d_px"])


# ---------------------------------------------------------------------------
# Analysis C (Section 12)
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("station", [STATION_MID_MM, STATION_FAR_MM])
def test_c_step8_cutout_transfer_curve_falls_toward_small_diameters(area_result, station):
    """Step 8: the largest cutouts keep between 50 and 110 percent of the true area and the ratio falls toward the
    smallest cutouts, which the matcher fills in (sensed area near 0)."""
    rows = _main_rows(area_result, FEATURE_CUTOUT, station)
    ratios = [r["ratio_true"] for r in rows]
    assert LARGEST_CUTOUT_RATIO_RANGE[0] <= ratios[-1] <= LARGEST_CUTOUT_RATIO_RANGE[1]
    assert ratios[-1] > ratios[0]
    assert np.mean(ratios[-3:]) > np.mean(ratios[:3])
    assert all(b <= a + 0.25 for a, b in zip(ratios[::-1], ratios[::-1][1:]))      # no large rise toward small D


def test_c_step4_small_cutouts_read_as_front(area_result):
    """Steps 3 and 4: with front_preference > 1 a hole of up to about 3 px is more than a third front material
    in every window, so it reads as front plate: no back reads, sensed area near 0 and edge bias b = -D / 2."""
    small = [r for r in area_result.rows if r["kind"] == FEATURE_CUTOUT and r["subseries"] == SUBSERIES_JITTER
             and r["d_px"] <= SMALL_CUTOUT_D_PX]
    assert small
    assert all(r["ratio_true"] < SMALL_CUTOUT_RATIO_MAX for r in small)
    assert all(abs(r["b_mm"] + r["diameter_mm"] / 2.0) < 0.1 * r["diameter_mm"] for r in small)


def test_c_step9_edge_bias_signs(area_result):
    """Step 9: front_preference > 1 grows the front material (a window pixel reads front when only a third of its
    window is front), so large disks are bigger than their outline (b > 0) and large cutouts smaller (b < 0). The
    fit is made on the largest third of the ladder (selection stated in the details)."""
    fits = {f["kind"]: f for f in area_result.details["bias_fits"] if f.get("points")}
    assert fits[FEATURE_DISK]["b_px_mean"] > 0.0
    assert fits[FEATURE_CUTOUT]["b_px_mean"] < 0.0
    assert "largest" in fits[FEATURE_DISK]["selection"]
    terms = area_result.forward_model_terms()
    assert terms["edge_bias_disk_px"] > 0.0 > terms["edge_bias_cutout_px"]
    assert terms["edge_bias_px"] > 0.0                       # positive = the front material grows


def test_c_step5_contour_area_matches_pixel_count_for_largest_features(area_result):
    """Step 5: the iso-contour polygon area agrees with the pixel-count area within 25 percent for the largest
    feature of each kind at each station."""
    for kind in (FEATURE_DISK, FEATURE_CUTOUT):
        for station in (STATION_MID_MM, STATION_FAR_MM):
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


def test_c_step12_csv_has_one_row_per_configuration_and_feature(area_result, cde_session):
    """Step 12: C_area_summary.csv has exactly one row per configuration x measured feature (all features of the
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
                 "C_predicted_vs_measured.png", area.DETAILS_FILE_NAME):
        assert (area_result.out_dir / name).exists(), name


def test_c_step11_field_and_open_background(area_result):
    """Step 11: the field sub-series is compared with the on-axis transfer curves, and for the open-background
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


def test_c_step10_consistency_with_blur_prediction(area_result):
    """Step 10: without Analysis B the kernel comes from the own radial ESF (stated in the details); the prediction
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
    """Step 10: rise distances of Analysis B (rise_h_px, rise_v_px, edge_offset_px) define a Gaussian kernel with the
    10-90 percent rise (stated assumption)."""
    class FakeB:
        def forward_model_terms(self):
            return {"rise_h_px": 5.0, "rise_v_px": 4.0, "edge_offset_px": -1.0}

    factor = area._gaussian_rise_factor(0.1, 0.9)
    kernel = area._kernel_from_b({"B": FakeB()}, factor)
    assert kernel["sigma_h_px"] == pytest.approx(5.0 / factor) and kernel["sigma_v_px"] == pytest.approx(4.0 / factor)
    assert kernel["edge_offset_px"] == -1.0
    assert area._kernel_from_b({}, factor) is None


# ---------------------------------------------------------------------------
# Analysis D (Section 13)
# ---------------------------------------------------------------------------
def _d_row(result, kind, station, rule):
    return next(r for r in result.rows if r["kind"] == kind and r["station_z_mm"] == station and r["rule"] == rule)


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
    """Step 3: the measured gamma of the 300-trial cutout configuration is consistent with the 0.01 target: the target
    lies inside the Clopper-Pearson interval of gamma, or gamma does not exceed the target (0 of n, bound reported)."""
    target = cde_session.params.detection_false_alarm_target
    row = _d_row(detection_result, FEATURE_CUTOUT, STATION_MID_MM, RULE_PRIMARY)
    assert row["gamma_blank_trials"] >= 1.0 / target
    assert row["gamma_lower"] <= target <= row["gamma_upper"] or row["gamma"] <= target
    assert row["gamma_lower"] <= row["gamma"] <= row["gamma_upper"]
    # With fewer than 1 / target trials per level the quantile is the maximum: 0 of n, upper bound reported.
    row_far = _d_row(detection_result, FEATURE_CUTOUT, STATION_FAR_MM, RULE_PRIMARY)
    assert row_far["gamma"] <= target
    assert row_far["gamma_upper"] == pytest.approx(clopper_pearson(int(round(row_far["gamma"]
                                                                              * row_far["gamma_blank_trials"])),
                                                                   int(row_far["gamma_blank_trials"]),
                                                                   cde_session.params.confidence_level)[1])


def test_d_step4_d50_is_a_few_pixels_and_inside_the_ladder(detection_result, cde_session):
    """Step 4: D_50 of the cutouts (primary rule, 300 trials) lies between the smallest and largest diameter on the
    plate and is a few pixels (0.5 to 6 px); its bootstrap interval brackets it."""
    row = _d_row(detection_result, FEATURE_CUTOUT, STATION_MID_MM, RULE_PRIMARY)
    diameters = [f.diameter_mm for f in cde_session.targets.get("T5-S", 15.0).features if f.kind == FEATURE_CUTOUT]
    assert min(diameters) < row["d50_mm"] < max(diameters)
    assert D50_PX_RANGE[0] <= row["d50_px"] <= D50_PX_RANGE[1]
    assert row["d50_lower_px"] <= row["d50_px"] <= row["d50_upper_px"]
    assert row["d50_status"] == "ok" and row["best_curve"] in ("logistic", "normal", "weibull")
    assert row["d50_mrad"] == pytest.approx(1000.0 * row["d50_mm"] / STATION_MID_MM)        # Step 10 units
    assert row["d50_px"] == pytest.approx(cde_session.geometry.diameter_in_pixels(row["d50_mm"], STATION_MID_MM))


def test_d_step5_to_7_minimums_are_ordered(detection_result):
    """Steps 5 to 7: D_10 <= D_50; D_0,emp <= D_10 (or not demonstrated, with the bracket reported); the model range
    of D_10 contains the best-fit D_10; the isotonic D_50 is within a factor 1.5 of the fitted one; the floor model
    gives a D_0 below D_10."""
    for rule in (RULE_PRIMARY, RULE_INCLUSIVE):
        row = _d_row(detection_result, FEATURE_CUTOUT, STATION_MID_MM, rule)
        assert row["d10_mm"] <= row["d50_mm"]
        if math.isfinite(row["d0_emp_mm"]):
            assert row["d0_emp_mm"] <= row["d10_mm"]
        else:
            assert math.isfinite(row["d0_emp_next_mm"])                            # the bracket is reported
        assert row["d10_model_min_mm"] <= row["d10_mm"] + 1e-9 <= row["d10_model_max_mm"] + 2e-9
        ratio = row["d50_isotonic_mm"] / row["d50_mm"]
        assert 1.0 / ISOTONIC_FACTOR < ratio < ISOTONIC_FACTOR
        assert row["d0_model_status"] in ("ok", "not estimable")
        if row["d0_model_status"] == "ok":
            assert row["d0_model_mm"] <= row["d10_mm"]


def test_d_step1_independence_is_computed(detection_result):
    """Step 1: the lag-1 autocorrelation of each feature's outcome sequence and the neighbor phi coefficients are
    computed against +/- 2 / sqrt(n); the flag is a bool and the tests are listed."""
    for row, detail in zip(detection_result.rows, detection_result.details):
        assert isinstance(row["independence_ok"], bool)
        independence = detail["independence"]
        assert independence["tested"] > 0 and independence["lag1"]
        for test in independence["lag1"]:
            assert test["bound"] == pytest.approx(2.0 / math.sqrt(test["n"]))
    assert _d_row(detection_result, FEATURE_CUTOUT, STATION_MID_MM, RULE_PRIMARY)["independence_ok"]


def test_d_step2_no_read_inclusive_rule_is_not_worse(detection_result):
    """Step 2: counting no-reads as candidates can only add detections, so the inclusive D_50 is at most the primary
    one for cutouts (the synthetic matcher fills holes with front values instead of leaving no-reads, so here the two
    rules coincide up to fit noise; the inequality is the property)."""
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
    assert row["trials_reused"] > 0 and row["trials_new"] == cde_fixture.DETECTION_POSES_CUTOUT
    disk = _d_row(detection_result, FEATURE_DISK, STATION_MID_MM, RULE_PRIMARY)
    assert math.isnan(disk["d0_geometric_mm"])                                      # cutouts only


def test_d_step11_csv_and_figures(detection_result):
    """Step 11: one row per configuration and rule (cutouts at two stations with both rules, disks at one with the
    primary rule), the documented columns, the details and the figures."""
    rows = _read_csv(detection_result.out_dir / detection.SUMMARY_FILE_NAME)
    assert len(rows) == 5
    assert list(rows[0].keys()) == list(detection.SUMMARY_COLUMNS)
    for name in ("d50_mm", "d50_px", "d50_mrad", "d10_px", "d0_emp_mm", "d0_emp_next_px", "d0_model_mrad",
                 "d0_geometric_px", "gamma_lower", "tau_mm", "independence_ok"):
        assert name in rows[0]
    document = json.loads((detection_result.out_dir / detection.DETAILS_FILE_NAME).read_text())
    assert len(document["configurations"]) == 5
    assert "levels" in document["configurations"][0] and "bootstrap" in document["configurations"][0]
    for name in ("D_psychometric_cutout_G15_Z750_F0.png", "D_theta_vs_z.png"):
        assert (detection_result.out_dir / name).exists(), name
    terms = detection_result.forward_model_terms()
    assert set(terms) == {"d50_px", "d10_px"} and terms["d10_px"] < terms["d50_px"]


def test_d_step9_bootstrap_is_reduced_for_few_poses(cde_session):
    """Step 9: a configuration with fewer than full_bootstrap_min_poses poses uses the reduced resample count and the
    details say so; a configuration with too few trials per level reports 'too few trials' without failing. (Run on
    the 60-pose far-station cutout configuration only, to keep the test short.)"""
    far = [r for r in cde_session.records if r.procedure == "D" and r.station_z_mm == STATION_FAR_MM]
    small = Session(root=cde_session.root, params=cde_session.params, sensor=cde_session.sensor,
                    registration=cde_session.registration, targets=cde_session.targets, records=far)
    options = detection.DetectionOptions(reduced_bootstrap_resamples=7, full_bootstrap_min_poses=10 ** 6,
                                         include_reused=False)
    result = detection.run_detection(small, Path("."), {}, options)
    row = next(r for r in result.rows if r["rule"] == RULE_PRIMARY)
    assert row["bootstrap_resamples"] == 7
    assert result.details[0]["bootstrap"]["reduced"] is True
    strict = detection.DetectionOptions(min_trials_per_level=10 ** 6, bootstrap_resamples=5, include_reused=False)
    result = detection.run_detection(small, Path("."), {}, strict)
    assert all(r["d50_status"] == "too few trials" and math.isnan(r["d50_mm"]) for r in result.rows)


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
                        station_z_mm=STATION_FAR_MM, visibility_rule="projector")[0]
    cameras = _e_rows(boundary_result, source="B", target_id="T3a", orientation="along", gap_mm=15.0,
                      station_z_mm=STATION_FAR_MM, visibility_rule="cameras")[0]
    assert projector["w_fab_px"] > 0.0 and cameras["w_fab_px"] > 0.0                # both rules reported


def test_e_step5_near_surface_preference(boundary_result):
    """Step 5: front_preference > 1 makes the matcher read the front surface beyond the true edge (front reads at
    s < 0 outnumber back reads at s > 0): pi_near > 0, the half-height crossing s_50 < 0 (the edge appears shifted onto
    the back side) and the mean h of intermediate reads is between 0 and 1."""
    for row in _e_rows(boundary_result, source="B", target_id="all", visibility_rule="projector"):
        assert row["pi_near"] > 0.5 and row["pi_lower"] <= row["pi_near"] <= row["pi_upper"]
        assert row["s50_px"] < 0.0
        assert 0.0 < row["h_mid_mean"] < 1.0
    terms = boundary_result.forward_model_terms()
    assert set(terms) == {"w_fab_px", "w_drop_px", "pi_near", "beta_read"}
    assert terms["pi_near"] > 0.0 and terms["w_fab_px"] > 0.0
    checks = boundary_result.details["cross_checks"]["own_s50_vs_pi"]
    assert checks and all(c["signs_consistent"] for c in checks)


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
    cutouts = sorted((r for r in table if r["kind"] == FEATURE_CUTOUT and r["station_z_mm"] == STATION_MID_MM),
                     key=lambda r: r["d_px"])
    small = np.mean([r["p_fill_in"] for r in cutouts[:SMALLEST_FIELD_CUTOUT_LEVELS]])
    large = np.mean([r["p_fill_in"] for r in cutouts[-SMALLEST_FIELD_CUTOUT_LEVELS:]])
    assert small > large and small > 0.9 and large < 0.2
    assert cutouts[-1]["p_correct"] > cutouts[0]["p_correct"]
    for r in cutouts:
        assert r["p_correct"] + r["p_wrong_surface"] + r["p_no_read"] == pytest.approx(1.0)
        assert math.isfinite(r["detect_primary"]) and math.isfinite(r["detect_inclusive"])
        assert r["rule_difference"] >= -1e-12
    disks = sorted((r for r in table if r["kind"] == FEATURE_DISK and r["station_z_mm"] == STATION_MID_MM),
                   key=lambda r: r["d_px"])
    assert disks[0]["p_erased"] > disks[-1]["p_erased"]


def test_e_sigma_source_is_stated_and_a_is_preferred(cde_session, tmp_path):
    """Step 2: without Analysis A sigma_tot is estimated locally (stated); when A offers rows with sigma_tot_mm at a
    station, those are used."""
    options = boundary.BoundaryOptions(bootstrap_resamples=2, feature_scale=False)
    local = boundary.run_boundary_bias(cde_session, tmp_path, {}, options)
    assert all("local robust std" in s["origin"] for s in local.details["sigma_tot"])

    class FakeA:
        summary_rows = [{"station_z_mm": 750.0, "sigma_tot_mm": 1.1, "field": 0, "subseries": "main"},
                        {"station_z_mm": 1000.0, "sigma_tot_mm": 2.0, "field": 0, "subseries": "main"}]

    with_a = boundary.run_boundary_bias(cde_session, tmp_path, {"A": FakeA()}, options)
    assert all(s["origin"] == "analysis A" for s in with_a.details["sigma_tot"])
    assert {s["sigma_tot_mm"] for s in with_a.details["sigma_tot"]} == {1.1, 2.0}


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
    for name in ("C_area_summary.csv", "D_detect_summary.csv", "E_boundary_bias.csv"):
        assert (session / "analysis" / name).exists(), name
    rows = _read_csv(session / "analysis" / "D_detect_summary.csv")
    assert rows and all(r["d50_status"] in ("too few trials", "ok", "never reached in the tested range",
                                            "already exceeded at the smallest level") for r in rows)
