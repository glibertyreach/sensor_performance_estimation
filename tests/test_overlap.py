"""
Tests of the overlap (scaling) test of the Z-sweep design (sensorperf.analysis.overlap; redesign note Section 5), used by
Analyses C and D.

Every test names what it checks and its acceptance criterion. All random draws come from seeded generators; the curves
are synthetic functions of D_px, so the expected answer of the test is known.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from sensorperf.analysis.overlap import (
    STATUS_FEW_POINTS, STATUS_NO_OVERLAP, STATUS_OK, OverlapOptions, TransferCurve, binomial_standard_error,
    overlap_test, overlap_tests, sigma_tot_at, sigma_tot_by_station,
)
from sensorperf.parameters import CharacterizationParameters, SensorGeometry

PARAMS = CharacterizationParameters()
GEOMETRY = SensorGeometry.indicative()
RESAMPLES = 400
"""Bootstrap resamples of these tests (the analyses use BOOTSTRAP_RESAMPLES)."""
SEED = 20261005
STANDARD_ERROR = 0.02
"""Standard error assigned to every point of the synthetic curves."""
SHIFT = 0.15
"""The shift of the disagreeing curve (7.5 standard errors)."""
HALF_SATURATION_PX = 10.0
"""D_px at which the synthetic transfer function is 1/2."""


def transfer_function(d_px):
    """A smooth increasing function of D_px between 0 and 1."""
    return 1.0 / (1.0 + (HALF_SATURATION_PX / np.asarray(d_px, dtype=float)) ** 2)


def feature_curve(label: str, feature_index: int, shift: float = 0.0) -> TransferCurve:
    """One feature's curve over the nine stations: D_px = D_k f_x / Z with the real ladder, the transfer function (plus an
    optional shift) as its value."""
    diameter = PARAMS.feature_diameters_mm(GEOMETRY)[feature_index]
    stations = np.array(PARAMS.z_stations_mm())
    d_px = np.array([GEOMETRY.diameter_in_pixels(diameter, z) for z in stations])
    return TransferCurve(label, d_px, transfer_function(d_px) + shift, np.full(d_px.size, STANDARD_ERROR), stations)


def test_identical_curves_agree():
    """Two copies of the same function of D_px over the shared range: the mean difference is 0 and zero lies inside the
    bootstrap interval, for every neighboring pair of the real feature ladder."""
    results = overlap_tests([feature_curve("f0", 0), feature_curve("f1", 1), feature_curve("f2", 2)],
                            PARAMS.confidence_level, RESAMPLES)
    assert [(r.label_small, r.label_large) for r in results] == [("f0", "f1"), ("f1", "f2")]
    for result in results:
        assert result.status == STATUS_OK and result.points_small >= 2 and result.points_large >= 2
        assert abs(result.mean_difference) < 5.0e-3                       # the grid is shared, only the point spacing differs
        assert result.agrees is True and result.interval_lower < 0.0 < result.interval_upper
        assert result.resamples == RESAMPLES


def test_exactly_identical_curves_have_zero_difference():
    """Two curves with the same points and no uncertainty: mean difference exactly 0, interval [0, 0], agreement."""
    x = np.geomspace(3.0, 20.0, 9)
    curve = TransferCurve("a", x, transfer_function(x), np.zeros(x.size))
    other = TransferCurve("b", x.copy(), transfer_function(x), np.zeros(x.size))
    result = overlap_test(curve, other, PARAMS.confidence_level, RESAMPLES)
    assert result.mean_difference == 0.0 and (result.interval_lower, result.interval_upper) == (0.0, 0.0)
    assert result.agrees is True


def test_shifted_curve_disagrees():
    """A curve shifted by 7.5 standard errors differs from its neighbor by the shift over the shared range: the mean
    difference is the shift and zero lies outside the bootstrap interval; the sigma_tot attribution note is set when A's
    values are given."""
    small, large = feature_curve("f0", 0), feature_curve("f1", 1, shift=SHIFT)
    sigma = {z: (z / 1000.0) ** 2 for z in PARAMS.z_stations_mm()}
    result = overlap_test(small, large, PARAMS.confidence_level, RESAMPLES, sigma_tot_mm=sigma)
    assert result.status == STATUS_OK
    assert result.mean_difference == pytest.approx(SHIFT, abs=5.0e-3)
    assert result.agrees is False and result.interval_lower > 0.0
    assert "sigma_tot" in result.note
    # At the same D_px the larger feature is seen from the farther stations, where the depth noise is larger.
    assert result.sigma_tot_large_mm > result.sigma_tot_small_mm > 0.0
    without = overlap_test(small, large, PARAMS.confidence_level, RESAMPLES)
    assert without.agrees is False and without.sigma_tot_small_mm is None and "not available" in without.note


def test_shared_range_is_the_overlap_of_the_two_d_px_ranges():
    """The shared range runs from the larger minimum to the smaller maximum D_px: for the real ladder the half-octave
    overlap (8.5 to 12 px and 24 to 34 px) of the redesign note."""
    first, second, third = (feature_curve("f0", 0), feature_curve("f1", 1), feature_curve("f2", 2))
    one = overlap_test(first, second, PARAMS.confidence_level, RESAMPLES)
    two = overlap_test(second, third, PARAMS.confidence_level, RESAMPLES)
    assert (one.shared_low_px, one.shared_high_px) == pytest.approx((8.5, 12.0), abs=0.1)
    assert (two.shared_low_px, two.shared_high_px) == pytest.approx((24.0, 34.0), abs=0.2)


def test_untestable_pairs_say_why():
    """No shared D_px range, or fewer points than OverlapOptions.min_points_per_curve inside it, is reported by status,
    not raised."""
    low = TransferCurve("low", np.array([1.0, 2.0, 3.0]), np.array([0.0, 0.1, 0.2]), np.full(3, 0.01))
    high = TransferCurve("high", np.array([10.0, 20.0, 30.0]), np.array([0.5, 0.7, 0.9]), np.full(3, 0.01))
    result = overlap_test(low, high, PARAMS.confidence_level, RESAMPLES)
    assert result.status == STATUS_NO_OVERLAP and result.agrees is None and math.isnan(result.mean_difference)
    touching = TransferCurve("touching", np.array([2.9, 10.0, 20.0]), np.array([0.2, 0.5, 0.7]), np.full(3, 0.01))
    result = overlap_test(low, touching, PARAMS.confidence_level, RESAMPLES)
    assert result.status == STATUS_FEW_POINTS and result.agrees is None
    single = TransferCurve("single", np.array([5.0]), np.array([0.3]), np.array([0.01]))
    assert overlap_test(single, high, PARAMS.confidence_level, RESAMPLES).status == STATUS_FEW_POINTS


def test_bootstrap_interval_widens_with_the_standard_errors():
    """The parametric bootstrap uses the point standard errors: ten times larger errors give an interval about ten times
    as wide; points without a known error (NaN) count as exact."""
    small, large = feature_curve("f0", 0), feature_curve("f1", 1)
    base = overlap_test(small, large, PARAMS.confidence_level, RESAMPLES, np.random.default_rng(SEED))
    noisy_small = TransferCurve(small.label, small.d_px, small.value, 10.0 * small.std_error, small.station_z_mm)
    noisy_large = TransferCurve(large.label, large.d_px, large.value, 10.0 * large.std_error, large.station_z_mm)
    wide = overlap_test(noisy_small, noisy_large, PARAMS.confidence_level, RESAMPLES, np.random.default_rng(SEED))
    ratio = (wide.interval_upper - wide.interval_lower) / (base.interval_upper - base.interval_lower)
    assert 8.0 < ratio < 12.0
    unknown = TransferCurve(small.label, small.d_px, small.value, np.full(small.d_px.size, np.nan), small.station_z_mm)
    exact = overlap_test(unknown, TransferCurve(large.label, large.d_px, large.value, np.zeros(large.d_px.size),
                                                large.station_z_mm), PARAMS.confidence_level, RESAMPLES)
    assert exact.interval_lower == exact.interval_upper


def test_binomial_standard_error_is_finite_at_the_extremes():
    """Detection proportions of 0 or 1 keep a nonzero standard error (half-count shrinkage); no trials give NaN."""
    error = binomial_standard_error(np.array([0, 30, 60, 5]), np.array([60, 60, 60, 0]))
    assert error[0] > 0.0 and error[2] > 0.0 and error[1] == pytest.approx(0.0645, abs=1.0e-3)
    assert math.isnan(error[3])


def test_sigma_tot_helpers():
    """sigma_tot per station is read from Analysis A's rows (objects or dicts, main center-field rows only) and
    interpolated log-log between A's stations (a power law is reproduced exactly) and clamped at its ends."""
    class Row:
        def __init__(self, z, sigma, field=0, subseries="main", tilt_deg=0.0):
            self.station_z_mm, self.sigma_tot_mm, self.field, self.subseries, self.tilt_deg = z, sigma, field, subseries, tilt_deg

    class FakeA:
        rows = [Row(400.0, 0.16), Row(800.0, 0.64), Row(1600.0, 2.56), Row(800.0, 9.0, field=2), Row(800.0, 9.0, tilt_deg=15.0),
                Row(800.0, 9.0, subseries="tilt")]

    table = sigma_tot_by_station({"A": FakeA()})
    assert table == {400.0: 0.16, 800.0: 0.64, 1600.0: 2.56}
    assert sigma_tot_at(table, 566.0) == pytest.approx((566.0 / 1000.0) ** 2, rel=1.0e-9)       # sigma = (Z / 1 m)^2
    assert sigma_tot_at(table, 800.0) == 0.64
    assert sigma_tot_by_station({}) == {} and sigma_tot_by_station({"A": object()}) == {}
    assert sigma_tot_at({}, 800.0) is None and sigma_tot_at({800.0: 1.0}, 400.0) is None
