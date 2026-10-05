"""
The overlap (scaling) test of the Z-sweep design (redesign note, Section 5), shared by Analysis C (area transfer
curves) and Analysis D (detection curves).

Why it exists. The design treats the subtended size in pixels, D_px = D f_x / Z, as the governing variable: a feature
seen from a different distance should give the same result as a feature of another size seen at the matching
distance. The three features of a plate are spaced by FEATURE_LADDER_RATIO = 2 sqrt(2) so that neighboring features
cover overlapping D_px ranges (a half octave when the stations span two octaves). Where two features overlap in D_px
their curves are two independent measurements of the same function of D_px if the scaling hypothesis holds. The test
compares them:

    1. the shared D_px range of two neighboring curves is [max of the two minimums, min of the two maximums];
    2. both curves are interpolated linearly in ln D_px onto OverlapOptions.grid_points evenly spaced (in ln D_px)
       points of that range, and the difference (curve of the larger feature minus curve of the smaller one) is averaged:
       the MEAN DIFFERENCE over the shared range;
    3. a parametric bootstrap redraws every curve point from a normal distribution with its own standard error
       (the curves are independent measurements) and repeats 2; the percentile interval of the mean difference at
       the confidence level is the BOOTSTRAP INTERVAL.
    The curves AGREE when zero lies inside the bootstrap interval, equivalently when the observed mean difference lies
    within the band that sampling noise alone can produce about zero. Agreement supports D_px as the governing
    variable; disagreement means something else that changes with Z (the depth noise sigma_tot(Z), which is larger at
    the far stations where the small features are seen) matters, and the result then reports the mean sigma_tot of the
    stations that contribute to the shared range for each feature, so that the difference can be attributed to it.

Conventions: D_px in pixels; curve values are dimensionless (area ratios, detection probabilities); the standard
errors are in the units of the values.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

import numpy as np

from sensorperf.io.manifest import SUBSERIES_MAIN
from sensorperf.parameters import FIELD_POSITION_CENTER

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
PERCENT = 100.0
"""Percent per unit fraction (the percentile arguments of numpy)."""
TWO_TAILS = 2.0
"""A two-sided interval holds (1 - confidence) / TWO_TAILS in each tail."""
HALF_COUNT = 0.5
"""Binomial standard errors use the proportion (s + 1/2) / (n + 1) so that a curve point with no or all successes
still has a nonzero standard error (the Jeffreys-type shrinkage; used by :func:`binomial_standard_error`)."""
RANGE_TOLERANCE = 5.0e-3
"""Relative tolerance of the test that a curve point lies inside the shared D_px range. The ends of the shared range of
two neighboring features are D_px values of the same station shift, which coincide only up to the 1 mm rounding of
the stations (about 0.1 percent), so a point within this tolerance of a range end counts as inside."""
STATUS_OK = "ok"
STATUS_NO_OVERLAP = "no shared D_px range"
STATUS_FEW_POINTS = "too few points in the shared range"
"""Values of :attr:`OverlapResult.status`."""


@dataclass(frozen=True)
class OverlapOptions:
    """Settings of the overlap test that are not procedure parameters."""

    grid_points: int = 25
    """Points of the common ln D_px grid on which the two curves are compared."""
    min_points_per_curve: int = 2
    """A curve needs at least this many of its own points inside the shared range for the pair to be tested."""
    difference_tolerance: float = 1.0e-9
    """Zero counts as inside the interval when it lies within this distance of it (floating-point guard: two
    identical curves must agree)."""
    seed: int = 20261006
    """Seed of the bootstrap generator (used when the caller passes no generator)."""


@dataclass
class TransferCurve:
    """One feature's curve: values against D_px over the stations at which the feature was measured."""

    label: str
    """Feature identity (for example the site id, 'disk_00')."""
    d_px: np.ndarray
    """Subtended size D_px = D f_x / Z of each point, pixels."""
    value: np.ndarray
    """Curve value of each point (area ratio, corrected detection probability, ...)."""
    std_error: np.ndarray
    """Standard error of each value (NaN counts as zero: no uncertainty is known for the point)."""
    station_z_mm: np.ndarray = field(default_factory=lambda: np.zeros(0))
    """Station depth of each point (mm), for the sigma_tot attribution; may be empty."""

    def sorted(self) -> "TransferCurve":
        """The curve with its points in increasing D_px, points with a non-finite D_px or value removed."""
        keep = np.isfinite(self.d_px) & np.isfinite(self.value) & (self.d_px > 0.0)
        order = np.argsort(self.d_px[keep], kind="stable")
        stations = self.station_z_mm[keep][order] if self.station_z_mm.size == self.d_px.size else self.station_z_mm
        return TransferCurve(self.label, self.d_px[keep][order], self.value[keep][order],
                             np.nan_to_num(self.std_error[keep][order], nan=0.0), stations)


@dataclass
class OverlapResult:
    """The overlap test of two neighboring curves."""

    label_small: str
    """The feature with the smaller D_px range (the smaller feature)."""
    label_large: str
    """The feature with the larger D_px range."""
    status: str
    """STATUS_OK, or why the pair could not be tested."""
    shared_low_px: float = math.nan
    shared_high_px: float = math.nan
    points_small: int = 0
    """Points of the small feature's curve inside the shared range."""
    points_large: int = 0
    mean_difference: float = math.nan
    """Mean over the shared range of (curve of the larger feature) - (curve of the smaller feature)."""
    interval_lower: float = math.nan
    interval_upper: float = math.nan
    """Bootstrap percentile interval of the mean difference."""
    agrees: bool | None = None
    """True when zero lies inside the bootstrap interval, False when not, None when the pair was not tested."""
    resamples: int = 0
    sigma_tot_small_mm: float | None = None
    """Mean sigma_tot(Z) of the stations of the small feature's points in the shared range (None if unknown)."""
    sigma_tot_large_mm: float | None = None
    note: str = ""

    def as_row(self) -> dict[str, Any]:
        """The CSV/JSON row of this result."""
        return {"feature_small": self.label_small, "feature_large": self.label_large, "status": self.status,
                "shared_low_px": self.shared_low_px, "shared_high_px": self.shared_high_px,
                "points_small": self.points_small, "points_large": self.points_large,
                "mean_difference": self.mean_difference, "interval_lower": self.interval_lower,
                "interval_upper": self.interval_upper, "agrees": self.agrees, "resamples": self.resamples,
                "sigma_tot_small_mm": self.sigma_tot_small_mm, "sigma_tot_large_mm": self.sigma_tot_large_mm,
                "note": self.note}


def binomial_standard_error(successes, trials) -> np.ndarray:
    """Standard error of a detection proportion s / n, from the shrunk proportion (s + 1/2) / (n + 1) so that
    proportions of 0 or 1 keep a nonzero error. Arrays in, array out (NaN where there are no trials)."""
    s = np.asarray(successes, dtype=float)
    n = np.asarray(trials, dtype=float)
    p = (s + HALF_COUNT) / (n + 1.0)
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(n > 0, np.sqrt(p * (1.0 - p) / (n + 1.0)), np.nan)


def _interpolate_log(curve: TransferCurve, grid_px: np.ndarray, values: np.ndarray | None = None) -> np.ndarray:
    """The curve (or the given values at its points) interpolated linearly in ln D_px onto ``grid_px``."""
    return np.interp(np.log(grid_px), np.log(curve.d_px), curve.value if values is None else values)


def _row_value(row: Any, name: str, default: Any = None) -> Any:
    """A field of an analysis row that is either a dict or a dataclass."""
    return row.get(name, default) if isinstance(row, Mapping) else getattr(row, name, default)


def sigma_tot_by_station(previous: Mapping[str, Any] | None) -> dict[float, float]:
    """sigma_tot (mm) per station of the fronto-parallel center-field main rows of Analysis A, from
    ``previous["A"]``: its ``main_rows()`` (a NoiseResult), else its ``summary_rows`` / ``rows`` (dict or dataclass rows
    with ``station_z_mm`` and ``sigma_tot_mm``; rows of other fields and sub-series are skipped). An empty dict when A
    did not run or offers no rows."""
    result = None if not previous else previous.get("A")
    if result is None:
        return {}
    rows: list[Any] = []
    if callable(getattr(result, "main_rows", None)):
        rows = list(result.main_rows())
    else:
        for attribute in ("summary_rows", "rows", "summary"):
            candidate = getattr(result, attribute, None)
            if isinstance(candidate, (list, tuple)):
                rows = [r for r in candidate
                        if _row_value(r, "field", FIELD_POSITION_CENTER) in (FIELD_POSITION_CENTER, None)
                        and _row_value(r, "subseries", SUBSERIES_MAIN) in (SUBSERIES_MAIN, None, "")
                        and not _row_value(r, "tilt_deg", 0.0)]
                break
    table: dict[float, list[float]] = {}
    for row in rows:
        station, sigma = _row_value(row, "station_z_mm"), _row_value(row, "sigma_tot_mm")
        if station is not None and sigma is not None and math.isfinite(float(sigma)):
            table.setdefault(float(station), []).append(float(sigma))
    return {z: float(np.mean(values)) for z, values in table.items()}


def sigma_tot_at(table: Mapping[float, float], station_z_mm: float) -> float | None:
    """sigma_tot (mm) at a station from A's per-station values: the exact station when A measured it, else a
    log-log interpolation between A's stations (sigma_tot is close to a power law of Z), clamped at A's ends; None
    when A offers no station, or a single station other than this one."""
    if not table:
        return None
    exact = [v for z, v in table.items() if math.isclose(z, station_z_mm, abs_tol=0.5)]
    if exact:
        return float(exact[0])
    if len(table) < 2:
        return None
    z = np.array(sorted(table))
    sigma = np.array([table[v] for v in z])
    return float(math.exp(np.interp(math.log(station_z_mm), np.log(z), np.log(sigma))))


def overlap_test(small: TransferCurve, large: TransferCurve, confidence: float, resamples: int,
                 rng: np.random.Generator | None = None, options: OverlapOptions = OverlapOptions(),
                 sigma_tot_mm: Mapping[float, float] | None = None) -> OverlapResult:
    """The overlap test (module docstring) of two curves; ``small`` is the feature with the smaller D_px range.

    Returns an :class:`OverlapResult` whose status says when the pair could not be tested (no shared range, or too few
    points of a curve inside it). ``sigma_tot_mm`` (station -> mm, from Analysis A) adds the mean sigma_tot of the
    contributing stations to the result."""
    a, b = small.sorted(), large.sorted()
    result = OverlapResult(label_small=small.label, label_large=large.label, status=STATUS_OK)
    if a.d_px.size < 2 or b.d_px.size < 2:
        result.status = STATUS_FEW_POINTS
        result.note = "a curve has fewer than two points"
        return result
    low, high = max(a.d_px[0], b.d_px[0]), min(a.d_px[-1], b.d_px[-1])
    result.shared_low_px, result.shared_high_px = float(low), float(high)
    if not high > low:
        result.status = STATUS_NO_OVERLAP
        return result
    inside_a = (a.d_px >= low * (1.0 - RANGE_TOLERANCE)) & (a.d_px <= high * (1.0 + RANGE_TOLERANCE))
    inside_b = (b.d_px >= low * (1.0 - RANGE_TOLERANCE)) & (b.d_px <= high * (1.0 + RANGE_TOLERANCE))
    result.points_small, result.points_large = int(inside_a.sum()), int(inside_b.sum())
    if min(result.points_small, result.points_large) < options.min_points_per_curve:
        result.status = STATUS_FEW_POINTS
        return result

    grid = np.exp(np.linspace(math.log(low), math.log(high), options.grid_points))
    result.mean_difference = float(np.mean(_interpolate_log(b, grid) - _interpolate_log(a, grid)))
    generator = np.random.default_rng(options.seed) if rng is None else rng
    draws = np.empty(resamples)
    for k in range(resamples):
        # Independent normal redraws of every point of both curves, then the same mean difference.
        value_a = a.value + a.std_error * generator.standard_normal(a.value.size)
        value_b = b.value + b.std_error * generator.standard_normal(b.value.size)
        draws[k] = np.mean(_interpolate_log(b, grid, value_b) - _interpolate_log(a, grid, value_a))
    tail = PERCENT * (1.0 - confidence) / TWO_TAILS
    result.interval_lower = float(np.percentile(draws, tail))
    result.interval_upper = float(np.percentile(draws, PERCENT - tail))
    result.resamples = resamples
    result.agrees = bool(result.interval_lower - options.difference_tolerance <= 0.0
                         <= result.interval_upper + options.difference_tolerance)
    if sigma_tot_mm:
        for curve, inside, name in ((a, inside_a, "sigma_tot_small_mm"), (b, inside_b, "sigma_tot_large_mm")):
            values = [sigma_tot_at(sigma_tot_mm, float(z)) for z in curve.station_z_mm[inside]] \
                if curve.station_z_mm.size == curve.d_px.size else []
            values = [v for v in values if v is not None]
            if values:
                setattr(result, name, float(np.mean(values)))
    if not result.agrees:
        result.note = ("the curves disagree over the shared D_px range: D_px alone does not govern the result; "
                       "attributed to sigma_tot(Z), which differs between the stations that see each feature there"
                       if result.sigma_tot_small_mm is not None and result.sigma_tot_large_mm is not None
                       else "the curves disagree over the shared D_px range: D_px alone does not govern the result "
                            "(sigma_tot(Z) from Analysis A was not available to attribute the difference)")
    return result


def overlap_tests(curves: Sequence[TransferCurve], confidence: float, resamples: int,
                  options: OverlapOptions = OverlapOptions(),
                  sigma_tot_mm: Mapping[float, float] | None = None) -> list[OverlapResult]:
    """The overlap test of every pair of neighboring curves, curves ordered by the median D_px of their points
    (so the feature ladder 0, 1, 2 gives the pairs (0, 1) and (1, 2))."""
    usable = [c.sorted() for c in curves]
    usable = [c for c in usable if c.d_px.size]
    usable.sort(key=lambda c: float(np.median(c.d_px)))
    rng = np.random.default_rng(options.seed)
    return [overlap_test(small, large, confidence, resamples, rng, options, sigma_tot_mm)
            for small, large in zip(usable[:-1], usable[1:])]
