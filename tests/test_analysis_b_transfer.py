"""
Tests of the edge position transfer of Analysis B-HV (specification Section 11.1, "Edge position transfer"; module
``sensorperf.analysis.resolution_lateral``, Step 9).

Two kinds of test. The numerical ones feed :func:`analyze_edge_transfer` with synthetic per-pose points whose gain, pixel-
locking bias, dot pitch bias and approach hysteresis are known and check that each is recovered. The session ones render a
small synthetic edge series (T3a, the small gap, the reference station: a nominal pose, jitter poses and the optional lateral
sweep) and check the wiring: the sweep poses stay out of the pooled ESF, the approach direction is derived as the plan
planned it, the new columns and the figure are written, and Analysis A's correlation length becomes the dot pitch.
"""
from __future__ import annotations

import csv
import math
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from sensorperf.acquisition.plan import plan_edge_series
from sensorperf.analysis import resolution_lateral
from sensorperf.analysis.resolution_lateral import (
    LateralOptions, PoseBins, TransferPoint, analyze_edge_transfer, approach_hysteresis, fit_edge_transfer, pose_s50,
    sweep_axis_and_approach,
)
from sensorperf.cli import simulate as simulate_cli
from sensorperf.geometry.targets import make_standard_target_set
from sensorperf.io.manifest import APPROACH_DIRECTION_KEY, SUBSERIES_LATERAL_SWEEP
from sensorperf.io.session import Session
from sensorperf.parameters import CharacterizationParameters, SensorGeometry
from sensorperf.simulate.demo_plan import demo_registration, scaled_geometry
from sensorperf.simulate.sensor_model import SyntheticSensorModel
from sensorperf.simulate.session import write_synthetic_session

# ---------------------------------------------------------------------------
# Synthetic truth of the numerical tests
# ---------------------------------------------------------------------------
TRUE_GAIN = 0.98
"""Lateral gain of the synthetic sensor: it reports 98 percent of the edge motion."""
EDGE_BIAS_PX = -1.1
"""Edge bias (s_50 at the nominal pose) of the synthetic sensor, px (a matcher that fattens the front surface)."""
PIXEL_LOCK_AMPLITUDE_PX = 0.05
"""Amplitude of the 1 px periodic bias, px."""
PIXEL_LOCK_PHASE_RAD = 0.7
DOT_PITCH_PX = 2.3
"""Projector dot pitch offered to the fit (the correlation length of Analysis A), px."""
DOT_PITCH_AMPLITUDE_PX = 0.04
DOT_PITCH_PHASE_RAD = 1.9
HYSTERESIS_PX = 0.1
"""Approach hysteresis of the synthetic sensor: s_50 of a pose approached from the negative side minus that of a pose
approached from the positive side, px."""
S50_NOISE_PX = 0.01
"""Noise of a pose's own s_50, px."""
RECOVERY_TOLERANCE = 0.2
"""Every known quantity is recovered within this fraction of itself (for the gain: of its distance from 1)."""
JITTER_POSES = 60
JITTER_SPAN_PX = 8.0
"""The phase-jitter poses: 60 offsets uniform over +/- 4 px (the span of PHASE_JITTER_SPAN_PX)."""
SWEEP_STEP_PX = 0.1
SWEEP_POSES = 20
"""The lateral sweep: 20 poses per axis at 0.1 px spacing, alternately approached from the negative and the positive side."""


def _synthetic_s50(offset: np.ndarray, from_negative: np.ndarray | None, rng: np.random.Generator) -> np.ndarray:
    """s_50 of each pose in the frame of its own true edge: the bias, the gain error, the two periodic biases, the approach
    hysteresis (half of it either way) and noise."""
    s50 = (EDGE_BIAS_PX + (TRUE_GAIN - 1.0) * offset
           + PIXEL_LOCK_AMPLITUDE_PX * np.sin(2.0 * math.pi * offset + PIXEL_LOCK_PHASE_RAD)
           + DOT_PITCH_AMPLITUDE_PX * np.sin(2.0 * math.pi * offset / DOT_PITCH_PX + DOT_PITCH_PHASE_RAD)
           + rng.normal(0.0, S50_NOISE_PX, offset.size))
    if from_negative is not None:
        s50 = s50 + np.where(from_negative, 0.5 * HYSTERESIS_PX, -0.5 * HYSTERESIS_PX)
    return s50


def _jitter_points(rng: np.random.Generator) -> list[TransferPoint]:
    offset = rng.uniform(-JITTER_SPAN_PX / 2.0, JITTER_SPAN_PX / 2.0, JITTER_POSES)
    return [TransferPoint(float(o), float(s)) for o, s in zip(offset, _synthetic_s50(offset, None, rng))]


def _sweep_points(rng: np.random.Generator) -> list[TransferPoint]:
    offset = SWEEP_STEP_PX * np.arange(1, SWEEP_POSES + 1)
    from_negative = np.arange(SWEEP_POSES) % 2 == 0                 # the 1st, 3rd, ... pose from the negative side
    return [TransferPoint(float(o), float(s), bool(n))
            for o, s, n in zip(offset, _synthetic_s50(offset, from_negative, rng), from_negative)]


def test_transfer_recovers_the_gain_and_both_periodic_biases():
    """Section 11.1, Edge position transfer: on synthetic jitter poses with a known gain (0.98), a known 1 px periodic bias
    (0.05 px) and a known dot pitch bias, the regression returns each within 20 percent (the gain: of its distance from 1),
    with the edge bias as the intercept and a standard error that covers the truth."""
    rng = np.random.default_rng(11)
    transfer = analyze_edge_transfer(_jitter_points(rng), [], DOT_PITCH_PX, LateralOptions())
    fit = transfer.fit
    assert fit.poses == JITTER_POSES == len(transfer.points)
    assert abs(fit.gain - TRUE_GAIN) <= RECOVERY_TOLERANCE * abs(1.0 - TRUE_GAIN)
    assert abs(fit.gain - TRUE_GAIN) <= 3.0 * fit.gain_se and 0.0 < fit.gain_se < 0.01
    assert fit.intercept_px == pytest.approx(EDGE_BIAS_PX, abs=0.02)
    assert fit.pixel_lock_amplitude_px == pytest.approx(PIXEL_LOCK_AMPLITUDE_PX, rel=RECOVERY_TOLERANCE)
    assert fit.dot_pitch_amplitude_px == pytest.approx(DOT_PITCH_AMPLITUDE_PX, rel=RECOVERY_TOLERANCE)
    assert fit.dot_pitch_px == DOT_PITCH_PX and transfer.dot_pitch_px == DOT_PITCH_PX
    assert fit.residual_rms_px == pytest.approx(S50_NOISE_PX, rel=0.5)
    # Without sweep poses there is no hysteresis and no sweep refit.
    assert math.isnan(transfer.hysteresis_px) and transfer.sweep_fit is None


def test_transfer_gain_of_one_means_s50_does_not_depend_on_the_offset():
    """The sign convention: s_50 is measured against the true edge of each pose, so a sensor that follows the edge exactly (gain
    1) gives a flat s_50 against the offset; the gain is one plus the slope, and the intercept the edge bias."""
    offset = np.linspace(-4.0, 4.0, 40)
    fit = fit_edge_transfer(offset, np.full(offset.size, EDGE_BIAS_PX), None, LateralOptions())
    assert fit.gain == pytest.approx(1.0) and fit.intercept_px == pytest.approx(EDGE_BIAS_PX)
    fit = fit_edge_transfer(offset, EDGE_BIAS_PX + (TRUE_GAIN - 1.0) * offset, None, LateralOptions())
    assert fit.gain == pytest.approx(TRUE_GAIN)


def test_transfer_sweep_recovers_the_hysteresis_and_refits_the_periodic_terms():
    """Section 11.1, Edge position transfer (sweep): on synthetic sweep poses at 0.1 px spacing alternately approached from
    the negative and the positive side, the mean difference of s_50 recovers the known hysteresis (0.1 px, negative minus
    positive) within 20 percent, and the periodic terms refitted on the sweep poses alone recover the 1 px bias within 20
    percent; the jitter fit of the same edge is untouched by the sweep."""
    rng = np.random.default_rng(12)
    jitter, sweep = _jitter_points(rng), _sweep_points(rng)
    with_sweep = analyze_edge_transfer(jitter, sweep, DOT_PITCH_PX, LateralOptions())
    without = analyze_edge_transfer(jitter, [], DOT_PITCH_PX, LateralOptions())
    assert with_sweep.hysteresis_px == pytest.approx(HYSTERESIS_PX, rel=RECOVERY_TOLERANCE)
    assert with_sweep.sweep_fit.pixel_lock_amplitude_px == pytest.approx(PIXEL_LOCK_AMPLITUDE_PX, rel=RECOVERY_TOLERANCE)
    assert with_sweep.sweep_fit.hysteresis_coefficient_px == pytest.approx(HYSTERESIS_PX, rel=RECOVERY_TOLERANCE)
    assert with_sweep.fit.gain == without.fit.gain and with_sweep.fit.pixel_lock_amplitude_px == \
        without.fit.pixel_lock_amplitude_px
    assert len(with_sweep.points) == JITTER_POSES and len(with_sweep.sweep_points) == SWEEP_POSES


def test_approach_hysteresis_sign_and_missing_sides():
    """Hysteresis is the mean s_50 approached from the negative side minus that from the positive side; NaN unless both sides
    have poses, and poses whose approach is unknown are left out."""
    s50 = np.array([1.0, 0.0, 3.0, 2.0, 99.0])
    assert approach_hysteresis(s50, [True, False, True, False, None]) == pytest.approx(1.0)
    assert math.isnan(approach_hysteresis(s50[:3], [True, True, None]))
    assert math.isnan(approach_hysteresis(s50[:0], []))


def test_transfer_skips_what_the_poses_cannot_support_and_says_so():
    """Without a dot pitch the dot pitch term is skipped with a note (amplitude NaN); a dot pitch of exactly 1 px is not
    separable from pixel locking and is skipped; with five poses (the demonstration series) only the line is fitted; with two
    poses nothing is. The notes name the reason."""
    rng = np.random.default_rng(13)
    points = _jitter_points(rng)
    no_pitch = analyze_edge_transfer(points, [], None, LateralOptions())
    assert math.isnan(no_pitch.fit.dot_pitch_amplitude_px) and math.isnan(no_pitch.fit.dot_pitch_px)
    assert np.isfinite(no_pitch.fit.pixel_lock_amplitude_px)
    assert any("no Analysis A correlation length" in note for note in no_pitch.notes)
    one_pixel = analyze_edge_transfer(points, [], 1.0, LateralOptions())
    assert math.isnan(one_pixel.fit.dot_pitch_amplitude_px) and any("not separable" in n for n in one_pixel.notes)
    few = analyze_edge_transfer(points[:5], [], DOT_PITCH_PX, LateralOptions())
    assert np.isfinite(few.fit.gain) and math.isnan(few.fit.pixel_lock_amplitude_px)
    assert any("degrees of freedom" in note for note in few.notes)
    two = analyze_edge_transfer(points[:2], [], DOT_PITCH_PX, LateralOptions())
    assert math.isnan(two.fit.gain) and any("only 2 poses" in note for note in two.notes)


def _logistic_pose_bins(center_px: float, reads_per_bin: int, bins: int = 64, width_px: float = 0.25) -> tuple:
    """(centers, PoseBins) of a single pose whose ESF is a logistic step centered at ``center_px``."""
    edges = -width_px * bins / 2.0 + width_px * np.arange(bins + 1)
    centers = (edges[:-1] + edges[1:]) / 2.0
    esf = 1.0 / (1.0 + np.exp(-(centers - center_px) / 0.5))
    n_read = np.full(bins, float(reads_per_bin))
    return centers, PoseBins(sum_h=esf * n_read, n_read=n_read, n_noread=np.zeros(bins))


def test_pose_s50_is_the_half_height_crossing_of_one_pose_and_skips_thin_poses():
    """Each pose gets its own s_50 from its own binned ESF (the same half-height crossing as the pooled ESF); a pose with too
    few valid bins (here: 2 reads in a bin, below the 3 required) is skipped, and so is a pose whose valid bins are too few."""
    params, options = CharacterizationParameters(), LateralOptions()
    centers, bins = _logistic_pose_bins(0.3, reads_per_bin=20)
    assert pose_s50(bins, centers, params, options) == pytest.approx(0.3, abs=0.05)
    centers, thin = _logistic_pose_bins(0.3, reads_per_bin=options.transfer_min_reads_per_bin - 1)
    assert pose_s50(thin, centers, params, options) is None
    sparse = PoseBins(sum_h=bins.sum_h.copy(), n_read=bins.n_read.copy(), n_noread=bins.n_noread)
    keep = int(options.transfer_min_valid_bin_fraction * sparse.n_read.size) - 1
    sparse.sum_h[keep:], sparse.n_read[keep:] = 0.0, 0.0
    assert pose_s50(sparse, centers, params, options) is None


# ---------------------------------------------------------------------------
# A rendered edge series with the lateral sweep
# ---------------------------------------------------------------------------
JITTER_POSES_RENDERED = 12
"""Jitter poses of the rendered series (plus the nominal pose: 13 poses per edge)."""
FRAMES_PER_RENDERED_POSE = 2
SESSION_SEED = 5
DOT_PITCH_H_PX, DOT_PITCH_V_PX = 2.3, 1.9
"""Correlation lengths of the stand-in for Analysis A: the dot pitch offered to H edges and to V edges."""
RENDERED_GAIN_RANGE = (0.7, 1.3)
"""The gain of the rendered sensor (a geometric imitation matcher: gain 1) is within this range with 13 two-frame poses."""
SWEEP_AXIS_POSES = 20
"""Sweep poses per axis."""
SWEEP_ACROSS_EDGE_FRACTION_RANGE = (0.98, 1.0)
"""The fraction of a sweep step that lies across the (slanted, about 5 degrees) edge."""


@pytest.fixture(scope="module")
def sweep_session(tmp_path_factory):
    """(session, plan of the rendered poses): T3a with the small gap at Z_REFERENCE, a nominal pose, jitter poses and the
    optional lateral sweep (20 poses in H and 20 in V), rendered on the quick geometry with 2 frames per pose."""
    root = tmp_path_factory.mktemp("sweep_session") / "session"
    params = replace(CharacterizationParameters(), phase_jitter_poses_edge=JITTER_POSES_RENDERED,
                     frames_per_edge_pose=FRAMES_PER_RENDERED_POSE)
    full = SensorGeometry.indicative()
    geometry = scaled_geometry(full, simulate_cli.QUICK_PIXEL_DIVISOR)
    model = SyntheticSensorModel.indicative_scaled(geometry, simulate_cli.QUICK_PIXEL_DIVISOR)
    rng = np.random.default_rng(SESSION_SEED)
    targets = make_standard_target_set(params, full)
    plan = [c for c in plan_edge_series(params, geometry, rng, targets, lateral_sweep=True)
            if c.target_id == "T3a" and c.gap_mm == params.gap_small_mm and c.station_z_mm == params.z_reference_mm]
    write_synthetic_session(root, params, geometry, model, demo_registration(), targets, plan, rng)
    return Session.load(root), plan


@pytest.fixture(scope="module")
def sweep_result(sweep_session, tmp_path_factory):
    session, _ = sweep_session
    out = tmp_path_factory.mktemp("sweep_analysis")
    a_stand_in = SimpleNamespace(main_rows=lambda: [SimpleNamespace(
        station_z_mm=session.params.z_reference_mm, corr_len_h_px=DOT_PITCH_H_PX, corr_len_v_px=DOT_PITCH_V_PX)])
    result = resolution_lateral.run_lateral_resolution(session, out, {"A": a_stand_in})
    resolution_lateral.write_outputs(result, out)
    return result, out


def test_sweep_poses_are_used_only_by_the_transfer(sweep_session, sweep_result):
    """Section 11.1: the lateral sweep covers +/- 1 px and would bias the pooled ESF, so it stays out of it (13 poses and 26
    frames per edge: the nominal and the jitter poses only), while the transfer reads the 20 sweep poses along the axis that
    moves the edge (H sweep for the left and right edges, V sweep for the top and bottom)."""
    result, _ = sweep_result
    assert [e.edge for e in result.edges] == ["left", "right", "top", "bottom"]
    for e in result.edges:
        assert e.poses == JITTER_POSES_RENDERED + 1 and e.frames == (JITTER_POSES_RENDERED + 1) * FRAMES_PER_RENDERED_POSE
        assert len(e.transfer.points) + e.transfer.skipped_poses == e.poses
        assert len(e.transfer.sweep_points) + e.transfer.sweep_skipped_poses == SWEEP_AXIS_POSES
        assert math.isfinite(e.transfer.hysteresis_px) and e.transfer.sweep_fit is not None
        # The nominal pose is the origin of the offsets.
        assert any(p.offset_px == 0.0 for p in e.transfer.points)
        # The sweep offsets are the read-back lateral shifts of 0.1 px steps (0.1 to 2.0 px along the axis), taken across the
        # edge: the square is slanted, so the component perpendicular to the edge is the same fraction (the cosine of the
        # slant) of every step.
        magnitudes = np.array(sorted(abs(p.offset_px) for p in e.transfer.sweep_points))
        fractions = magnitudes / (SWEEP_STEP_PX * np.arange(1, magnitudes.size + 1))
        assert np.allclose(fractions, fractions[0], rtol=1.0e-3) and SWEEP_ACROSS_EDGE_FRACTION_RANGE[0] < fractions[0] <= 1.0


def test_sweep_approach_direction_round_trips_through_the_manifest(sweep_session):
    """The plan's ``approach_direction`` note reaches the manifest as a metadata column (empty for other poses) and the
    analysis reads it: for every sweep pose the column equals the planned direction and the analysis returns that axis and
    side. Only for a manifest without the column does it derive the same answer from the logged offset by the plan's rule."""
    session, plan = sweep_session
    planned = {c.pose_index: c.notes[APPROACH_DIRECTION_KEY] for c in plan if c.subseries == SUBSERIES_LATERAL_SWEEP}
    assert len(planned) == 2 * SWEEP_AXIS_POSES
    pitch = session.geometry.pixel_footprint_mm(session.params.z_reference_mm)
    step = session.params.lateral_sweep_step_px
    checked = 0
    for record in session.records:
        if record.subseries != SUBSERIES_LATERAL_SWEEP:
            assert APPROACH_DIRECTION_KEY not in record.metadata          # empty cell for every other pose
            continue
        direction = planned[record.pose_index]
        assert record.metadata[APPROACH_DIRECTION_KEY] == direction
        assert sweep_axis_and_approach(record, pitch, step) == (direction[1], direction[0] == "-")
        stripped = replace(record, metadata={})                          # a manifest without the column: the fallback rule
        assert sweep_axis_and_approach(stripped, pitch, step) == (direction[1], direction[0] == "-")
        told = replace(record, metadata={APPROACH_DIRECTION_KEY: "+V"})  # the column wins over the offset
        assert sweep_axis_and_approach(told, pitch, step) == ("V", False)
        checked += record.frame_index == 0
    assert checked == 2 * SWEEP_AXIS_POSES


def test_transfer_columns_figure_and_dot_pitch_of_the_rendered_series(sweep_result):
    """The summary carries the nine new columns; the gain of the geometric imitation matcher is near 1; Analysis A's
    correlation length along H (V) is the dot pitch of the H (V) edges; the transfer figure exists in PNG and SVG; the
    details JSON keeps the per-pose points; the forward-model terms are unchanged."""
    result, out = sweep_result
    rows = list(csv.DictReader((out / resolution_lateral.SUMMARY_CSV_NAME).open(encoding="utf-8")))
    new_columns = ("lateral_gain", "lateral_gain_se", "pixel_lock_amplitude_px", "dot_pitch_amplitude_px", "dot_pitch_px",
                   "transfer_poses", "hysteresis_px", "sweep_pixel_lock_amplitude_px", "sweep_dot_pitch_amplitude_px")
    assert set(new_columns) <= set(rows[0]) and all(c in resolution_lateral.SUMMARY_COLUMNS for c in new_columns)
    for row in rows:
        assert RENDERED_GAIN_RANGE[0] < float(row["lateral_gain"]) < RENDERED_GAIN_RANGE[1]
        assert float(row["lateral_gain_se"]) > 0.0 and int(row["transfer_poses"]) == JITTER_POSES_RENDERED + 1
        assert float(row["dot_pitch_px"]) == (DOT_PITCH_H_PX if row["orientation"] == "H" else DOT_PITCH_V_PX)
        for column in ("pixel_lock_amplitude_px", "dot_pitch_amplitude_px", "hysteresis_px",
                       "sweep_pixel_lock_amplitude_px", "sweep_dot_pitch_amplitude_px"):
            assert math.isfinite(float(row[column])), column
    for extension in ("png", "svg"):
        assert (out / f"{resolution_lateral.FIGURE_STEMS['transfer']}.{extension}").stat().st_size > 0
    details = (out / resolution_lateral.DETAILS_JSON_NAME).read_text(encoding="utf-8")
    assert '"edge_position_transfer"' in details and '"sweep_points"' in details
    assert set(result.forward_model_terms()) == {"rise_h_px", "rise_v_px", "edge_offset_px"}


def test_transfer_without_analysis_a_skips_the_dot_pitch_with_a_note(sweep_session, tmp_path):
    """Without ``previous["A"]`` the dot pitch term is skipped, NaN in the CSV, and the row note says why."""
    session, _ = sweep_session
    result = resolution_lateral.run_lateral_resolution(session, tmp_path, {})
    for e in result.edges:
        row = e.csv_row()
        assert math.isnan(row["dot_pitch_px"]) and math.isnan(row["dot_pitch_amplitude_px"])
        assert math.isfinite(row["pixel_lock_amplitude_px"])
        assert "no Analysis A correlation length" in row["note"]
