"""
Tests of the limits of the quick-look capture check (``sensorperf.acquisition.check``): the residual and
distance limits scale with the registered target depth, the tilt limit follows the precision of each plane fit,
a tilt that cannot be resolved is reported instead of flagged, and a gross error is still caught.

The tests use a small synthetic session (``sensorperf.cli.simulate --quick --series C D``, 160 x 120 px, seeded).
"""
from __future__ import annotations

import dataclasses
import json
import math
import time
from pathlib import Path

import numpy as np
import pytest

from sensorperf.acquisition.check import (
    CORRELATION_MAX_LAG_PX, FLAG_BACK_OFFSET, FLAG_FRONT_OFFSET, CheckParameters, check_session, depth_limit_scale,
    residual_correlation_area, tilt_standard_error_deg,
)
from sensorperf.cli import check_captures as check_cli
from sensorperf.cli import simulate as simulate_cli
from sensorperf.geometry.transforms import RigidTransform
from sensorperf.io.session import Session
from sensorperf.parameters import CharacterizationParameters

SIMULATION_SEED = 3
"""Seed of the synthetic session."""
GROSS_SHIFT_MM = 50.0
"""The registered target of the corrupted pose is moved this far along the camera z axis."""
CORRUPTED_STATION_MM = 800.0
"""Station of the pose that the test corrupts."""
LOW_CAP_DEG = 2.5
"""A tilt-limit cap low enough that some planes of the quick session (a few hundred pixels at 1,600 mm) cannot be tested."""
FAR_Z_MM = 1500.0
"""Targets registered beyond this distance are at the 1,600 mm station."""
SECONDS_PER_TEST = 60.0
"""The test must stay well under this."""
NOISE_POINTS = 4000
"""Points of the noisy planes of the standard-error test."""
NOISE_TRIALS = 200
"""Planes fitted in the standard-error test."""
NOISE_SIGMA_MM = 1.0
"""Independent depth noise of the standard-error test."""
PLANE_HALF_WIDTH_MM = 100.0
"""Half width of the square patch of the standard-error test (the minor in-plane spread is 100 / sqrt(3) mm)."""


@pytest.fixture(scope="module")
def quick_session(tmp_path_factory) -> Session:
    """A quick synthetic session of the series C and D (about 270 poses, a few seconds)."""
    root = tmp_path_factory.mktemp("quick_cd")
    assert simulate_cli.main(["--out", str(root), "--quick", "--series", "C", "D", "--seed", str(SIMULATION_SEED)]) == 0
    return Session.load(root)


def test_depth_limit_scale_follows_the_stereo_noise_law_and_never_tightens():
    """The scale is (Z / reference) ** exponent beyond the reference, 1 at and inside it, and 1 without a usable Z."""
    params = CheckParameters()
    reference = CharacterizationParameters().z_reference_mm
    assert params.limit_reference_z_mm == reference and params.limit_depth_exponent == 2.0
    assert depth_limit_scale(reference, params) == 1.0
    assert depth_limit_scale(reference / 2.0, params) == 1.0                       # never tighter than stated
    assert depth_limit_scale(2.0 * reference, params) == pytest.approx(4.0)        # 1,600 mm: four times the limit
    assert depth_limit_scale(float("nan"), params) == 1.0
    assert depth_limit_scale(2.0 * reference, dataclasses.replace(params, limit_depth_exponent=0.0)) == 1.0


def test_tilt_standard_error_matches_the_scatter_of_fitted_slopes():
    """The standard error is rms / (s_minor sqrt(N)): on independent noise it equals the observed spread of the
    slope of fitted planes (here about the y axis, the minor axis of a rectangular patch)."""
    rng = np.random.default_rng(SIMULATION_SEED)
    x = rng.uniform(-2.0 * PLANE_HALF_WIDTH_MM, 2.0 * PLANE_HALF_WIDTH_MM, NOISE_POINTS)       # major axis
    y = rng.uniform(-PLANE_HALF_WIDTH_MM, PLANE_HALF_WIDTH_MM, NOISE_POINTS)                   # minor axis
    slopes, errors = [], []
    for _ in range(NOISE_TRIALS):
        z = rng.normal(0.0, NOISE_SIGMA_MM, NOISE_POINTS)
        coefficients, *_ = np.linalg.lstsq(np.column_stack([x, y, np.ones(NOISE_POINTS)]), z, rcond=None)
        slopes.append(coefficients[1])
        errors.append(tilt_standard_error_deg(np.column_stack([x, y, z]), float(np.std(z))))
    expected_deg = math.degrees(math.atan(np.std(slopes)))
    assert np.mean(errors) == pytest.approx(expected_deg, rel=0.15)
    assert math.isinf(tilt_standard_error_deg(np.column_stack([x[:2], y[:2], x[:2]]), 1.0))   # too few points
    line = np.column_stack([x, np.zeros(NOISE_POINTS), np.zeros(NOISE_POINTS)])
    assert math.isinf(tilt_standard_error_deg(line, 1.0))                                      # collinear: no plane


BLOCK_PX = 4
"""Side of the blocks of the correlated-noise residual map (the indicative sensor's noise block)."""
MAP_SIDE_PX = 120
"""Side of the square residual maps of the correlation-area test."""


def test_correlation_area_is_one_for_independent_pixels_and_grows_with_the_noise_block():
    """The correlation area of white residuals is about 1 (a little above, from the positive noise of the 288 other lags);
    residuals constant over 4 x 4 px blocks give about the block area, 16; a gap in the fitted pixels does not matter."""
    rng = np.random.default_rng(SIMULATION_SEED)
    white = rng.normal(0.0, 1.0, (MAP_SIDE_PX, MAP_SIDE_PX))
    blocks = np.kron(rng.normal(0.0, 1.0, (MAP_SIDE_PX // BLOCK_PX,) * 2), np.ones((BLOCK_PX, BLOCK_PX)))
    assert 1.0 <= residual_correlation_area(white) < 2.5
    area = residual_correlation_area(blocks)
    assert 0.6 * BLOCK_PX ** 2 < area < 1.6 * BLOCK_PX ** 2
    gapped = blocks.copy()
    gapped[:, MAP_SIDE_PX // 3: MAP_SIDE_PX // 2] = np.nan
    assert residual_correlation_area(gapped) == pytest.approx(area, rel=0.3)
    assert CORRELATION_MAX_LAG_PX == 8


def corrupted(session: Session, shift_mm: float) -> tuple[Session, str]:
    """The session with the registered target of one pose at the reference station moved by shift_mm along z in every
    frame of that pose (a wrong registration); returns it and the name of the corrupted pose."""
    victim = next(r for r in session.records
                  if r.target_id == "T4" and r.station_z_mm == CORRUPTED_STATION_MM and r.procedure == "D")
    records = []
    for record in session.records:
        if record.pose_key() == victim.pose_key():
            pose = record.target_pose_camera
            record = dataclasses.replace(
                record, target_pose_camera=RigidTransform(pose.rotation, pose.translation + [0.0, 0.0, shift_mm]))
        records.append(record)
    return dataclasses.replace(session, records=records), victim.path.name.rsplit("_f", 1)[0]


def test_clean_session_passes_and_a_gross_error_is_still_flagged(quick_session: Session, tmp_path: Path):
    """A clean synthetic session flags nothing (and check.json records the limits and the new parameters); moving the
    registered target of one pose by 50 mm along z, or swapping its target, flags that pose and only that pose."""
    started = time.time()
    clean = quick_session
    report = check_session(clean)
    assert report.to_dict()["n_flagged"] == 0, [(p.name, p.flags) for p in report.flagged]
    parameters = report.to_dict()["parameters"]
    assert parameters["limit_reference_z_mm"] == CharacterizationParameters().z_reference_mm
    assert {"limit_depth_exponent", "normal_sigma_factor", "normal_limit_cap_deg"} <= set(parameters)
    # The limits applied are recorded per pose and plane.
    fitted = next(p for p in report.poses if p.front.pixels)
    entry = next(e for e in report.to_dict()["poses"] if e["pose"] == fitted.name)["front"]
    assert entry["residual_limit_mm"] >= CheckParameters().plane_residual_warn_mm
    assert entry["offset_limit_mm"] >= CheckParameters().pose_residual_warn_mm
    assert entry["normal_limit_deg"] >= CheckParameters().normal_warn_deg and entry["normal_untestable"] is False
    assert entry["correlation_area_px"] >= 1.0 and entry["effective_pixels"] <= fitted.front.pixels

    broken, name = corrupted(clean, GROSS_SHIFT_MM)
    broken_report = check_session(broken)
    assert [p.name for p in broken_report.flagged] == [name]
    flagged = broken_report.flagged[0]
    plane = next(c for c in (flagged.front, flagged.back) if np.isfinite(c.offset_mm) and abs(c.offset_mm) > c.offset_limit_mm)
    assert any(flag in flagged.flags for flag in (FLAG_FRONT_OFFSET, FLAG_BACK_OFFSET))
    assert abs(plane.offset_mm) == pytest.approx(GROSS_SHIFT_MM, rel=0.2)
    assert broken_report.verdict().startswith(f"VERDICT: 1 of {len(broken_report.poses)} poses flagged")
    assert time.time() - started < SECONDS_PER_TEST


def test_far_poses_are_judged_by_scaled_limits_and_untestable_tilts_are_noted(quick_session: Session, tmp_path: Path, capsys):
    """At 1,600 mm the residual limit is four times the stated one; a plane whose tilt limit exceeds the cap raises no
    tilt flag and carries the note 'tilt not testable' (printed on the pose line and written to check.json)."""
    out = tmp_path / "check.json"
    cap_deg = LOW_CAP_DEG
    check_cli.main(["--session", str(quick_session.root), "--out", str(out), "--normal-limit-cap-deg", str(cap_deg)])
    report = json.loads(out.read_text())
    far = [p for p in report["poses"] if p["target_z_mm"] > FAR_Z_MM and p["front"]["pixels"]]
    assert far and all(p["front"]["residual_limit_mm"] == pytest.approx(4.0 * 3.0, rel=0.01) for p in far)
    untestable = [(p, c) for p in report["poses"] for c in (p["front"], p["back"]) if c["normal_untestable"]]
    assert untestable
    for pose, plane in untestable:
        assert plane["normal_limit_deg"] is None and max(2.0, 4.0 * plane["sigma_angle_deg"]) > cap_deg
        assert any("tilt not testable" in note for note in pose["notes"])
    testable = [c for p in report["poses"] for c in (p["front"], p["back"]) if c["normal_limit_deg"] is not None]
    assert testable and all(c["normal_limit_deg"] <= cap_deg for c in testable)
    assert "tilt not testable" in capsys.readouterr().out


def test_cli_options_set_the_limit_parameters(quick_session: Session, tmp_path: Path):
    """The four new options reach CheckParameters (the report's parameters block); the reference distance defaults to the
    session's Z_REFERENCE_MM."""
    out = tmp_path / "check.json"
    check_cli.main(["--session", str(quick_session.root), "--out", str(out), "--limit-reference-z-mm", "700",
                    "--limit-depth-exponent", "1.5", "--normal-sigma-factor", "3", "--normal-limit-cap-deg", "8"])
    parameters = json.loads(out.read_text())["parameters"]
    assert (parameters["limit_reference_z_mm"], parameters["limit_depth_exponent"], parameters["normal_sigma_factor"],
            parameters["normal_limit_cap_deg"]) == (700.0, 1.5, 3.0, 8.0)
    help_text = check_cli.build_parser().format_help()
    for option in ("--limit-reference-z-mm", "--limit-depth-exponent", "--normal-sigma-factor", "--normal-limit-cap-deg"):
        assert option in help_text
