"""
Psychometric curves, model-free isotonic thresholds and the threshold (floor)
model of the detectability analysis.

Serves Analysis D (Section 13 of the characterization procedure, Steps 4, 5
and 7) and the Z-resolution detection of Section 11.2.

Model (Step 4). The raw detection probability at diameter D (or step size)
is

    psi(D) = gamma + (1 - gamma - lambda) F((ln D - alpha) / beta),

with gamma the guess (false-alarm) rate fixed from the blank sites, lambda
the lapse rate, free but bounded to [0, LAPSE_RATE_MAX], and F one of three
sigmoid shapes in x = ln D: logistic, cumulative normal, or Weibull
(F = 1 - exp(-exp(z))). The false-alarm-corrected probability is P* = F.
Parameters (alpha, beta, lambda) are fitted by maximum binomial likelihood;
gamma is not fitted. beta is carried through its logarithm so it stays
positive.

Conventions: levels are positive sizes (millimeters, or D_px for the pooled
detection fit; the natural logarithm is taken inside); probabilities are in [0, 1]; ``successes`` and ``trials``
are counts per level; ``deviance`` is 2 (saturated log-likelihood minus
fitted log-likelihood) with the saturated model psi_i = s_i / n_i at every
level.

Threshold model (Step 7). P* = 0 for D <= D_0 and
1 - exp(-((D - D_0) / s)^k) above it. The fit works on corrected
proportions times trials; those products are rounded to whole counts and
treated as effective successes, an approximation that ignores the extra
variance the false-alarm correction adds. D_0 gets a profile-likelihood
interval.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np
from scipy.optimize import minimize, minimize_scalar
from scipy.special import expit, ndtr, ndtri, xlogy
from scipy.stats import chi2

CURVE_LOGISTIC, CURVE_NORMAL, CURVE_WEIBULL = "logistic", "normal", "weibull"
CURVE_NAMES = (CURVE_LOGISTIC, CURVE_NORMAL, CURVE_WEIBULL)
"""All supported curve families, in the order fit_all_curves reports them."""

# ---------------------------------------------------------------------------
# Numerical constants of the psychometric fit
# ---------------------------------------------------------------------------
PROBABILITY_EPSILON = 1.0e-9
"""Probabilities are clipped to [eps, 1 - eps] inside the log-likelihood so a
fitted probability of exactly 0 or 1 cannot produce log(0)."""

DEFAULT_MAX_ITERATIONS = 500
"""Default optimizer iteration budget of one L-BFGS-B run (PsychometricFitParameters.max_iterations)."""

HALF_PROBABILITY = 0.5
"""P* = 0.5: the point whose level seeds alpha (the D_50 of the logistic and normal curves)."""

LOGISTIC_SPAN_PROBABILITY = 0.9
"""A logistic rising from 10 to 90 percent covers a z range of 2 ln 9 = 4.39 in units of beta."""

INITIAL_BETA_SPAN_DIVISOR = 2.0 * math.log(LOGISTIC_SPAN_PROBABILITY / (1.0 - LOGISTIC_SPAN_PROBABILITY))
"""Initial beta = (range of ln level) / this. It assumes the tested levels span
about the 10 to 90 percent rise of the curve, which the feature and station
ladders of the Z-sweep design arrange (the levels span 3 to 96 px around the
expected 10 to 15 px threshold)."""

INITIAL_BETA_MULTIPLIERS = (0.5, 1.0, 2.0)
"""The fit is started from each of these multiples of the initial beta and the
best optimum is kept; a single start can stall on a flat likelihood ridge."""

INITIAL_LAPSE_FRACTION = 0.25
"""The starting lapse rate is this fraction of its upper bound."""

MIN_LOG_SPAN = 1.0e-3
"""Guard: the range of ln level used for bounds and starts is at least this, so
a nearly one-level data set does not collapse the bounds."""

ALPHA_MARGIN_SPANS = 3.0
"""alpha may leave the range of tested ln levels by this multiple of the range
(thresholds far outside the data are not identifiable anyway)."""

LOG_BETA_LOWER_BOUND = math.log(1.0e-3)
"""Lower bound of ln beta (an almost vertical step in ln D)."""

LOG_BETA_UPPER_BOUND = math.log(1.0e2)
"""Upper bound of ln beta (a curve flat over many decades of D)."""

# ---------------------------------------------------------------------------
# Numerical constants of the isotonic fit and the threshold model
# ---------------------------------------------------------------------------
THRESHOLD_MODEL_GRID_POINTS = 61
"""Number of D_0 values of the profile-likelihood grid, spaced evenly over [0, D_0 upper bound]."""

THRESHOLD_MODEL_MAX_ITERATIONS = 300
"""Iteration budget of one L-BFGS-B run of the threshold-model fit."""

THRESHOLD_SHAPE_STARTS = (1.0, 2.0, 4.0)
"""Starting Weibull shapes k of the threshold-model fit (1 = exponential rise, larger = steeper)."""

WEIBULL_CHARACTERISTIC_PROBABILITY = 1.0 - math.exp(-1.0)
"""P* reached at D = D_0 + s (where the Weibull exponent equals 1); seeds the scale s."""

THRESHOLD_SCALE_MIN_FRACTION = 1.0e-3
"""Lower bound of s as a fraction of the largest tested level."""

THRESHOLD_SCALE_MAX_MULTIPLE = 1.0e3
"""Upper bound of s as a multiple of the largest tested level."""

THRESHOLD_LOG_SHAPE_BOUNDS = (math.log(0.1), math.log(50.0))
"""Bounds of ln k: shapes from a very gradual (0.1) to a nearly step-like (50) rise."""

PROFILE_DEGREES_OF_FREEDOM = 1
"""The profile likelihood on D_0 is a one-parameter likelihood-ratio test (chi-square, 1 dof)."""

LIKELIHOOD_RATIO_FACTOR = 2.0
"""Likelihood-ratio statistic = 2 (NLL(D_0) - NLL_min)."""


# ---------------------------------------------------------------------------
# Curve families
# ---------------------------------------------------------------------------
def _sigmoid(curve: str, z: np.ndarray) -> np.ndarray:
    """F(z) of the named family: logistic, cumulative normal, or Weibull in ln D."""
    if curve == CURVE_LOGISTIC:
        return expit(z)
    if curve == CURVE_NORMAL:
        return ndtr(z)
    if curve == CURVE_WEIBULL:
        return -np.expm1(-np.exp(z))
    raise ValueError(f"unknown curve {curve!r}; expected one of {CURVE_NAMES}")


def _inverse_sigmoid(curve: str, probability: float) -> float:
    """z with F(z) = probability, for the named family (probability strictly in (0, 1))."""
    if not 0.0 < probability < 1.0:
        raise ValueError(f"p_star must lie strictly between 0 and 1, got {probability}")
    if curve == CURVE_LOGISTIC:
        return math.log(probability / (1.0 - probability))
    if curve == CURVE_NORMAL:
        return float(ndtri(probability))
    if curve == CURVE_WEIBULL:
        return math.log(-math.log(1.0 - probability))
    raise ValueError(f"unknown curve {curve!r}; expected one of {CURVE_NAMES}")


def corrected_rate(raw_rate, guess_rate):
    """False-alarm-corrected detection probability (psi - gamma) / (1 - gamma).

    The correction removes the share of detections that the blank-site false
    alarms alone would produce (Section 13, Step 4). Negative values, which
    sampling noise produces when psi < gamma, are clipped at 0. Accepts
    scalars (returns a float) or arrays (returns an array); dimensionless.
    """
    if not 0.0 <= np.max(guess_rate) < 1.0:
        raise ValueError(f"guess_rate must lie in [0, 1), got {guess_rate}")
    corrected = np.maximum((np.asarray(raw_rate, dtype=float) - guess_rate) / (1.0 - guess_rate), 0.0)
    return float(corrected) if corrected.ndim == 0 else corrected


# ---------------------------------------------------------------------------
# Psychometric fit
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class PsychometricFitParameters:
    """Settings of one psychometric fit."""

    lapse_rate_max: float
    """Upper bound of the lapse rate lambda (params.detection_lapse_rate_max)."""
    curve: str = CURVE_LOGISTIC
    """Curve family: CURVE_LOGISTIC, CURVE_NORMAL or CURVE_WEIBULL."""
    max_iterations: int = DEFAULT_MAX_ITERATIONS
    """Iteration budget of each optimizer run."""
    warm_start: tuple[float, float, float] | None = None
    """(alpha, beta, lapse rate) of an earlier fit to the same kind of data. When given, the optimizer runs once from
    that start instead of from each of the INITIAL_BETA_MULTIPLIERS heuristic starts: a bootstrap resample is close to
    the original data, so the original optimum is a good start and the fit is several times faster."""


@dataclass
class PsychometricFit:
    """A fitted psychometric curve and the data it was fitted to."""

    curve: str
    """Curve family of F."""
    alpha: float
    """Location in ln(level), the argument of F at which the (z = 0) midpoint lies."""
    beta: float
    """Scale in ln(level) units, positive."""
    guess_rate: float
    """gamma, fixed from the blank sites."""
    lapse_rate: float
    """lambda, fitted within [0, lapse_rate_max]."""
    deviance: float
    """2 (saturated log-likelihood - fitted log-likelihood); smaller is a better fit."""
    converged: bool
    """Whether the best optimizer run reported success."""
    levels: np.ndarray
    """Tested levels in mm."""
    successes: np.ndarray
    """Detections per level."""
    trials: np.ndarray
    """Trials per level."""

    def corrected_probability(self, x) -> np.ndarray:
        """P*(x) = F((ln x - alpha) / beta), the false-alarm-corrected probability."""
        z = (np.log(np.asarray(x, dtype=float)) - self.alpha) / self.beta
        return _sigmoid(self.curve, z)

    def probability(self, x) -> np.ndarray:
        """Raw detection probability psi(x) = gamma + (1 - gamma - lambda) P*(x)."""
        return self.guess_rate + (1.0 - self.guess_rate - self.lapse_rate) * self.corrected_probability(x)

    def threshold(self, p_star: float) -> float:
        """Level in mm at which the corrected probability equals p_star.

        Inverts F: logistic z = ln(p / (1 - p)); normal z = Phi^-1(p);
        Weibull in ln x z = ln(-ln(1 - p)); then x = exp(alpha + beta z). The
        logistic D_50 is exp(alpha) and D_10 is exp(alpha - beta ln 9).
        """
        return float(math.exp(self.alpha + self.beta * _inverse_sigmoid(self.curve, p_star)))


def _validate_counts(levels, successes, trials, guess_rate: float):
    """Convert inputs to float arrays and reject impossible counts."""
    levels = np.asarray(levels, dtype=float)
    successes = np.asarray(successes, dtype=float)
    trials = np.asarray(trials, dtype=float)
    if not (levels.ndim == successes.ndim == trials.ndim == 1 and levels.size == successes.size == trials.size):
        raise ValueError("levels, successes and trials must be one-dimensional arrays of equal length")
    if np.any(levels <= 0.0):
        raise ValueError("levels must be positive (the fit works in ln level)")
    if np.any(trials < 0) or np.any(successes < 0) or np.any(successes > trials):
        raise ValueError("counts must satisfy 0 <= successes <= trials")
    if np.sum(trials) <= 0:
        raise ValueError("at least one trial is needed")
    if not 0.0 <= guess_rate < 1.0:
        raise ValueError(f"guess_rate must lie in [0, 1), got {guess_rate}")
    return levels, successes, trials


def _binomial_log_likelihood(successes: np.ndarray, trials: np.ndarray, probability: np.ndarray) -> float:
    """Sum of s ln p + (n - s) ln(1 - p) with p clipped to [eps, 1 - eps]."""
    clipped = np.clip(probability, PROBABILITY_EPSILON, 1.0 - PROBABILITY_EPSILON)
    return float(np.sum(successes * np.log(clipped) + (trials - successes) * np.log1p(-clipped)))


def _saturated_log_likelihood(successes: np.ndarray, trials: np.ndarray) -> float:
    """Log-likelihood of the saturated model p_i = s_i / n_i (xlogy makes 0 ln 0 = 0)."""
    with np.errstate(divide="ignore", invalid="ignore"):
        proportion = np.where(trials > 0, successes / np.where(trials > 0, trials, 1.0), 0.0)
    return float(np.sum(xlogy(successes, proportion) + xlogy(trials - successes, 1.0 - proportion)))


def _initial_half_level(levels: np.ndarray, successes: np.ndarray, trials: np.ndarray, guess_rate: float) -> float:
    """ln level at which the corrected proportion is about 0.5, interpolated.

    The corrected proportions are made monotone with a running maximum so
    sampling noise cannot send the interpolation backwards; if they never
    reach 0.5 the level whose proportion is nearest 0.5 is used instead.
    """
    order = np.argsort(levels)
    log_levels = np.log(levels[order])
    observed = np.where(trials[order] > 0, successes[order] / np.maximum(trials[order], 1.0), 0.0)
    corrected = np.asarray(corrected_rate(observed, guess_rate))
    running = np.maximum.accumulate(corrected)
    if running[-1] >= HALF_PROBABILITY and running[0] <= HALF_PROBABILITY:
        # np.interp needs a non-decreasing abscissa: the running maximum is one.
        return float(np.interp(HALF_PROBABILITY, running, log_levels))
    return float(log_levels[int(np.argmin(np.abs(corrected - HALF_PROBABILITY)))])


def fit_psychometric(levels, successes, trials, guess_rate: float,
                     params: PsychometricFitParameters) -> PsychometricFit:
    """Maximum binomial likelihood fit of alpha, beta and lambda with gamma fixed.

    The optimizer works on theta = (alpha, ln beta, lambda) with L-BFGS-B
    bounds (alpha within the data range extended by ALPHA_MARGIN_SPANS ranges,
    ln beta within named bounds, lambda within [0, lapse_rate_max]). It is
    started from several beta values (INITIAL_BETA_MULTIPLIERS) around a
    heuristic initial guess and the best optimum is kept. Section 13, Steps 4
    and 5.
    """
    levels, successes, trials = _validate_counts(levels, successes, trials, guess_rate)
    if not 0.0 <= params.lapse_rate_max < 1.0 - guess_rate:
        raise ValueError("lapse_rate_max must lie in [0, 1 - guess_rate)")
    if params.curve not in CURVE_NAMES:
        raise ValueError(f"unknown curve {params.curve!r}; expected one of {CURVE_NAMES}")

    log_levels = np.log(levels)
    span = max(float(np.ptp(log_levels)), MIN_LOG_SPAN)

    def negative_log_likelihood(theta: np.ndarray) -> float:
        alpha, log_beta, lapse = theta
        z = (log_levels - alpha) / math.exp(log_beta)
        psi = guess_rate + (1.0 - guess_rate - lapse) * _sigmoid(params.curve, z)
        return -_binomial_log_likelihood(successes, trials, psi)

    bounds = [(float(np.min(log_levels)) - ALPHA_MARGIN_SPANS * span,
               float(np.max(log_levels)) + ALPHA_MARGIN_SPANS * span),
              (LOG_BETA_LOWER_BOUND, LOG_BETA_UPPER_BOUND),
              (0.0, params.lapse_rate_max)]
    half_level = _initial_half_level(levels, successes, trials, guess_rate)
    z_half = _inverse_sigmoid(params.curve, HALF_PROBABILITY)

    starts = []
    if params.warm_start is not None:
        alpha_w, beta_w, lapse_w = params.warm_start
        starts.append(np.array([alpha_w, math.log(beta_w), lapse_w]))
    else:
        for multiplier in INITIAL_BETA_MULTIPLIERS:
            beta0 = multiplier * span / INITIAL_BETA_SPAN_DIVISOR
            # Place alpha so that P* = 0.5 falls at the observed half level for this curve family.
            alpha0 = half_level - beta0 * z_half
            starts.append(np.array([alpha0, math.log(beta0), INITIAL_LAPSE_FRACTION * params.lapse_rate_max]))
    best = None
    for start in starts:
        start = np.clip(start, [b[0] for b in bounds], [b[1] for b in bounds])
        result = minimize(negative_log_likelihood, start, method="L-BFGS-B", bounds=bounds,
                          options={"maxiter": params.max_iterations})
        if best is None or result.fun < best.fun:
            best = result

    alpha, log_beta, lapse = best.x
    deviance = LIKELIHOOD_RATIO_FACTOR * (_saturated_log_likelihood(successes, trials) + float(best.fun))
    return PsychometricFit(curve=params.curve, alpha=float(alpha), beta=math.exp(log_beta),
                           guess_rate=float(guess_rate), lapse_rate=float(lapse),
                           deviance=max(deviance, 0.0), converged=bool(best.success),
                           levels=levels, successes=successes, trials=trials)


def fit_all_curves(levels, successes, trials, guess_rate,
                   params: PsychometricFitParameters) -> dict[str, PsychometricFit]:
    """Fit the logistic, normal and Weibull shapes (Section 13, Step 5).

    ``params.curve`` is ignored; the lapse bound and iteration budget are
    used. The dict is keyed by curve name in the order logistic, normal,
    weibull; ``best_fit`` picks the smallest deviance. All three have the same
    number of free parameters, so deviance is a fair comparison.
    """
    fits = {}
    for curve in CURVE_NAMES:
        shape_params = PsychometricFitParameters(lapse_rate_max=params.lapse_rate_max, curve=curve,
                                                 max_iterations=params.max_iterations)
        fits[curve] = fit_psychometric(levels, successes, trials, guess_rate, shape_params)
    return fits


def best_fit(fits: dict[str, PsychometricFit]) -> PsychometricFit:
    """The fit with the smallest deviance (the one whose D_10 is reported)."""
    return min(fits.values(), key=lambda fit: fit.deviance)


# ---------------------------------------------------------------------------
# Isotonic regression
# ---------------------------------------------------------------------------
def pool_adjacent_violators(values: Sequence[float], weights: Sequence[float] | None = None) -> np.ndarray:
    """Weighted non-decreasing least-squares fit by pool-adjacent-violators.

    Neighboring points that violate monotonicity are merged into a block
    whose value is the weighted mean of its members; merging repeats until
    block values increase. Returns an array of the input length.
    """
    values = np.asarray(values, dtype=float)
    weights = np.ones_like(values) if weights is None else np.asarray(weights, dtype=float)
    block_value: list[float] = []
    block_weight: list[float] = []
    block_count: list[int] = []
    for value, weight in zip(values, weights):
        block_value.append(float(value))
        block_weight.append(float(weight))
        block_count.append(1)
        while len(block_value) > 1 and block_value[-2] > block_value[-1]:
            total = block_weight[-2] + block_weight[-1]
            merged = (block_value[-2] * block_weight[-2] + block_value[-1] * block_weight[-1]) / total if total > 0 \
                else 0.5 * (block_value[-2] + block_value[-1])
            block_value[-2:] = [merged]
            block_weight[-2:] = [total]
            block_count[-2:] = [block_count[-2] + block_count[-1]]
    return np.repeat(block_value, block_count)


def isotonic_threshold(levels, corrected_proportions, p_star: float, trials=None) -> float:
    """Model-free threshold: level where a monotone fit first reaches p_star (Step 5).

    The corrected proportions, ordered by ln(level), are fitted with a
    non-decreasing curve (pool-adjacent-violators, weighted by ``trials`` when
    given), and the first crossing of p_star is located by linear
    interpolation of the fitted values against ln(level), then returned as a
    level in mm. Returns NaN when the fit never reaches p_star, or when it is
    already at or above p_star at the lowest level (the crossing then lies
    outside the tested range and is not extrapolated).
    """
    levels = np.asarray(levels, dtype=float)
    proportions = np.asarray(corrected_proportions, dtype=float)
    if levels.shape != proportions.shape or levels.ndim != 1:
        raise ValueError("levels and corrected_proportions must be one-dimensional and of equal length")
    order = np.argsort(levels, kind="stable")
    log_levels = np.log(levels[order])
    weights = None if trials is None else np.asarray(trials, dtype=float)[order]
    fitted = pool_adjacent_violators(proportions[order], weights)

    reaching = np.nonzero(fitted >= p_star)[0]
    if reaching.size == 0 or reaching[0] == 0:
        return math.nan
    upper = int(reaching[0])
    lower = upper - 1
    rise = fitted[upper] - fitted[lower]  # positive: fitted[lower] < p_star <= fitted[upper]
    fraction = (p_star - fitted[lower]) / rise
    return float(math.exp(log_levels[lower] + fraction * (log_levels[upper] - log_levels[lower])))


# ---------------------------------------------------------------------------
# Threshold (floor) model
# ---------------------------------------------------------------------------
@dataclass
class ThresholdFit:
    """Weibull-with-floor fit of the corrected detection probability (Section 13, Step 7)."""

    d0: float
    """Maximum likelihood floor D_0 in mm: P* = 0 at and below it."""
    scale: float
    """Weibull scale s in mm."""
    shape: float
    """Weibull shape k (dimensionless)."""
    d0_lower: float
    """Lower limit of the profile-likelihood interval of D_0, mm."""
    d0_upper: float
    """Upper limit of the profile-likelihood interval of D_0, mm."""
    deviance: float
    """2 (saturated - fitted) log-likelihood of the effective counts."""
    converged: bool = True
    """Whether the optimizer runs behind the point estimate reported success."""
    profile_d0: np.ndarray | None = None
    """D_0 values of the profile grid, mm."""
    profile_statistic: np.ndarray | None = None
    """2 (minNLL(D_0) - minNLL) at each grid value; the interval is where it is <= the chi-square quantile."""

    def probability(self, x) -> np.ndarray:
        """P*(x): 0 at and below D_0, 1 - exp(-((x - D_0) / s)^k) above."""
        return _floor_weibull(np.asarray(x, dtype=float), self.d0, self.scale, self.shape)


def _floor_weibull(levels: np.ndarray, d0: float, scale: float, shape: float) -> np.ndarray:
    """Weibull rise starting at the floor d0: 0 for levels <= d0."""
    excess = np.maximum(levels - d0, 0.0) / scale
    return -np.expm1(-np.power(excess, shape))


def fit_threshold_model(levels, corrected_proportions, trials, confidence: float) -> ThresholdFit:
    """MLE and profile-likelihood interval for the floor D_0 (Section 13, Step 7).

    Corrected proportion times trials, rounded to whole counts, serves as the
    effective number of successes in a binomial likelihood (probabilities
    clipped to [eps, 1 - eps]). D_0 is restricted to [0, D_0 max], where
    D_0 max is the smallest level with a nonzero effective count: a model
    with P* = 0 at a level that shows detections would have essentially zero
    likelihood. For a grid of D_0 values (THRESHOLD_MODEL_GRID_POINTS, plus
    the MLE itself) the scale and shape are re-optimized; the interval is the
    set of D_0 with 2 (minNLL(D_0) - minNLL) <= chi2.ppf(confidence, 1). The
    interval ends are linearly interpolated between the last grid value
    inside the set and the first one outside; a grid end that is inside the
    set (D_0 = 0 or D_0 max) is itself the interval end, because D_0 cannot
    leave the allowed range.
    """
    levels = np.asarray(levels, dtype=float)
    proportions = np.asarray(corrected_proportions, dtype=float)
    trials_arr = np.asarray(trials, dtype=float)
    if not (levels.ndim == 1 and levels.shape == proportions.shape == trials_arr.shape):
        raise ValueError("levels, corrected_proportions and trials must be one-dimensional and of equal length")
    if np.any(levels <= 0.0) or np.any(trials_arr < 0):
        raise ValueError("levels must be positive and trials non-negative")
    if not 0.0 < confidence < 1.0:
        raise ValueError(f"confidence must lie strictly between 0 and 1, got {confidence}")

    order = np.argsort(levels, kind="stable")
    levels, proportions, trials_arr = levels[order], proportions[order], trials_arr[order]
    # Effective successes: corrected proportion x trials, rounded to whole counts.
    successes = np.minimum(np.round(np.clip(proportions, 0.0, 1.0) * trials_arr), trials_arr)
    detected = successes > 0
    if not np.any(detected):
        raise ValueError("every effective count is zero: the floor D_0 is not bounded above by the data; "
                         "use the empirical bound of Step 6 instead")
    d0_max = float(np.min(levels[detected]))
    top = float(np.max(levels))
    saturated = _saturated_log_likelihood(successes, trials_arr)

    log_scale_bounds = (math.log(THRESHOLD_SCALE_MIN_FRACTION * top), math.log(THRESHOLD_SCALE_MAX_MULTIPLE * top))
    bounds = [log_scale_bounds, THRESHOLD_LOG_SHAPE_BOUNDS]
    low_bound = np.array([b[0] for b in bounds])
    high_bound = np.array([b[1] for b in bounds])

    def minimum_nll(d0: float, warm_start: np.ndarray | None):
        """Smallest NLL over (ln s, ln k) at fixed d0, with the optimizer result's parameters."""
        def nll(theta: np.ndarray) -> float:
            return -_binomial_log_likelihood(successes, trials_arr,
                                             _floor_weibull(levels, d0, math.exp(theta[0]), math.exp(theta[1])))
        # Scale seed: first level whose corrected proportion reaches 1 - 1/e, measured from d0.
        reached = np.nonzero(proportions >= WEIBULL_CHARACTERISTIC_PROBABILITY)[0]
        reference = float(levels[reached[0]]) if reached.size else top
        scale0 = max(reference - d0, THRESHOLD_SCALE_MIN_FRACTION * top * math.e)
        starts = [np.array([math.log(scale0), math.log(k0)]) for k0 in THRESHOLD_SHAPE_STARTS]
        if warm_start is not None:
            starts.append(warm_start)
        best = None
        for start in starts:
            result = minimize(nll, np.clip(start, low_bound, high_bound), method="L-BFGS-B", bounds=bounds,
                              options={"maxiter": THRESHOLD_MODEL_MAX_ITERATIONS})
            if best is None or result.fun < best.fun:
                best = result
        return best

    # Profile grid over d0, warm-starting each fit from its neighbor's optimum.
    grid = list(np.linspace(0.0, d0_max, THRESHOLD_MODEL_GRID_POINTS))
    results = []
    warm = None
    for d0 in grid:
        fit = minimum_nll(float(d0), warm)
        warm = fit.x
        results.append(fit)
    best_index = int(np.argmin([r.fun for r in results]))
    # Refine the point estimate by also fitting between the neighbors of the best grid value.
    neighbors = grid[max(best_index - 1, 0):best_index + 2]
    refined_d0, refined = float(grid[best_index]), results[best_index]
    if len(neighbors) > 1 and neighbors[0] < neighbors[-1]:
        outcome = minimize_scalar(lambda d0: minimum_nll(float(d0), refined.x).fun,
                                  bounds=(neighbors[0], neighbors[-1]), method="bounded")
        if outcome.fun < refined.fun:
            refined_d0 = float(outcome.x)
            refined = minimum_nll(refined_d0, refined.x)
    # Insert the MLE into the grid so the interval always contains it.
    if refined_d0 not in grid:
        position = int(np.searchsorted(grid, refined_d0))
        grid.insert(position, refined_d0)
        results.insert(position, refined)
    grid_arr = np.array(grid)
    nll_arr = np.array([r.fun for r in results])
    minimum = float(min(nll_arr.min(), refined.fun))
    statistic = LIKELIHOOD_RATIO_FACTOR * (nll_arr - minimum)
    critical = float(chi2.ppf(confidence, PROFILE_DEGREES_OF_FREEDOM))

    inside = np.nonzero(statistic <= critical)[0]
    first, last = int(inside[0]), int(inside[-1])  # never empty: the MLE row has statistic 0

    def crossing(inner: int, outer: int) -> float:
        """D_0 where the statistic equals ``critical``, between grid indices inner (inside) and outer (outside)."""
        fraction = (critical - statistic[inner]) / (statistic[outer] - statistic[inner])
        return float(grid_arr[inner] + fraction * (grid_arr[outer] - grid_arr[inner]))

    d0_lower = float(grid_arr[0]) if first == 0 else crossing(first, first - 1)
    d0_upper = float(grid_arr[-1]) if last == len(grid_arr) - 1 else crossing(last, last + 1)

    scale, shape = math.exp(refined.x[0]), math.exp(refined.x[1])
    deviance = LIKELIHOOD_RATIO_FACTOR * (saturated + minimum)
    return ThresholdFit(d0=refined_d0, scale=scale, shape=shape, d0_lower=d0_lower, d0_upper=d0_upper,
                        deviance=max(deviance, 0.0), converged=bool(refined.success),
                        profile_d0=grid_arr, profile_statistic=statistic)
