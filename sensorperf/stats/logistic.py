"""
Binomial logistic regression with a few covariates, for the noise-covariate model of the detectability analysis
(procedure document, Section 13; redesign note, Section 5).

Model. For grouped binomial data (counts of detections ``s_i`` in ``n_i`` trials at design point i) the detection
probability is

    logit p_i = b_0 + b_1 x_i1 + b_2 x_i2 + ...

with covariates x such as ln D_px and ln sigma_tot(Z). The fit maximizes the binomial likelihood by iteratively
reweighted least squares (Newton steps). Two safeguards make it usable on detection data, where whole groups of
design points detect nothing or everything:

- the covariates are centered and scaled to unit standard deviation before the fit (so the ridge below acts evenly and
  the optimizer is well conditioned); the coefficients are converted back to the original units afterward,
- a small ridge penalty on the slope coefficients (never on the intercept) keeps the estimate finite under complete or
  quasi-complete separation, and the result says when that happened (``separated``).

Conventions: covariates are dimensionless numbers (natural logarithms of sizes and noise levels taken by the caller);
probabilities are in [0, 1]; the deviance is 2 (saturated log-likelihood minus fitted log-likelihood) with the
saturated model p_i = s_i / n_i.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.special import expit, xlogy
from scipy.stats import chi2

# ---------------------------------------------------------------------------
# Numerical constants
# ---------------------------------------------------------------------------
RIDGE_PENALTY = 1.0e-4
"""Ridge weight on the standardized slope coefficients (intercept excluded): negligible for data that identify the
slopes, but enough to keep them finite when the data are separated."""
MAX_ITERATIONS = 100
"""Newton iteration budget."""
CONVERGENCE_TOLERANCE = 1.0e-9
"""The fit has converged when the largest coefficient change of a step is below this (standardized units)."""
SEPARATION_COEFFICIENT = 15.0
"""A standardized slope above this in magnitude means the likelihood has run into the ridge: the data separate
detections from misses (the logistic curve is a step), and the slope and its standard error are not meaningful."""
MIN_STANDARD_DEVIATION = 1.0e-12
"""A covariate whose standard deviation is below this carries no information (it is constant)."""
PROBABILITY_EPSILON = 1.0e-12
"""Probabilities are clipped to [eps, 1 - eps] inside logarithms."""
HALF = 0.5
"""The quadratic ridge penalty is HALF ridge |beta|^2, so that its gradient is ridge beta."""
LIKELIHOOD_RATIO_FACTOR = 2.0
"""Likelihood-ratio statistic = 2 (log L_full - log L_reduced)."""
HESSIAN_CONDITION_LIMIT = 1.0e12
"""A Hessian whose condition number exceeds this is treated as singular (no standard errors)."""


@dataclass
class LogisticFit:
    """A fitted grouped logistic regression."""

    coefficients: np.ndarray
    """Intercept first, then one slope per covariate column, in the units of the covariates."""
    standard_errors: np.ndarray
    """Asymptotic standard errors (inverse observed information, ridge included); NaN when the information matrix
    is singular or the fit separated."""
    deviance: float
    """2 (saturated log-likelihood - fitted log-likelihood)."""
    log_likelihood: float
    """Binomial log-likelihood of the fit (without the ridge term)."""
    converged: bool
    """Whether the Newton iteration converged within the budget."""
    separated: bool
    """True when a standardized slope reached SEPARATION_COEFFICIENT (see that constant)."""
    observations: int
    """Number of design points used."""

    def z_values(self) -> np.ndarray:
        """Wald z = coefficient / standard error (NaN where the error is NaN or zero)."""
        with np.errstate(invalid="ignore", divide="ignore"):
            return self.coefficients / self.standard_errors

    def predict(self, covariates: np.ndarray) -> np.ndarray:
        """Detection probability at rows of covariates (shape (m, k))."""
        design = np.column_stack([np.ones(len(covariates)), covariates])
        return expit(design @ self.coefficients)


def _log_likelihood(successes: np.ndarray, trials: np.ndarray, probability: np.ndarray) -> float:
    """Binomial log-likelihood without the combinatorial constant, probabilities clipped away from 0 and 1."""
    p = np.clip(probability, PROBABILITY_EPSILON, 1.0 - PROBABILITY_EPSILON)
    return float(np.sum(xlogy(successes, p) + xlogy(trials - successes, 1.0 - p)))


def _saturated_log_likelihood(successes: np.ndarray, trials: np.ndarray) -> float:
    """Log-likelihood of the saturated model p_i = s_i / n_i."""
    p = np.where(trials > 0, successes / np.maximum(trials, 1.0), 0.0)
    return float(np.sum(xlogy(successes, p) + xlogy(trials - successes, 1.0 - p)))


def fit_logistic(covariates, successes, trials, ridge: float = RIDGE_PENALTY) -> LogisticFit:
    """Fit logit p = b_0 + sum_j b_j x_j to grouped binomial counts.

    ``covariates`` is an (m, k) array (k >= 0; k = 0 fits the intercept only), ``successes`` and ``trials`` are
    length-m counts (design points with no trials are dropped). Returns a :class:`LogisticFit` in the original units
    of the covariates. Raises ValueError for inconsistent shapes or counts."""
    x = np.asarray(covariates, dtype=float)
    if x.ndim == 1:
        x = x[:, None]
    s = np.asarray(successes, dtype=float)
    n = np.asarray(trials, dtype=float)
    if x.shape[0] != s.size or s.size != n.size:
        raise ValueError("covariates, successes and trials must have the same number of design points")
    if np.any(s < 0) or np.any(s > n):
        raise ValueError("successes must lie between 0 and trials")
    keep = n > 0
    x, s, n = x[keep], s[keep], n[keep]
    if s.size == 0:
        raise ValueError("no design point has any trials")
    columns = x.shape[1]
    # Standardize: centered, unit-variance covariates (constant columns are left centered and unscaled).
    mean = x.mean(axis=0) if columns else np.zeros(0)
    scale = x.std(axis=0) if columns else np.zeros(0)
    scale = np.where(scale > MIN_STANDARD_DEVIATION, scale, 1.0)
    design = np.column_stack([np.ones(s.size), (x - mean) / scale])
    penalty = np.diag([0.0] + [ridge] * columns)

    beta = np.zeros(columns + 1)
    start_rate = float(np.clip(s.sum() / n.sum(), PROBABILITY_EPSILON, 1.0 - PROBABILITY_EPSILON))
    beta[0] = math.log(start_rate / (1.0 - start_rate))
    converged = False
    for _ in range(MAX_ITERATIONS):
        p = expit(design @ beta)
        weights = n * p * (1.0 - p)
        gradient = design.T @ (s - n * p) - penalty @ beta
        hessian = design.T @ (design * weights[:, None]) + penalty
        try:
            step = np.linalg.solve(hessian, gradient)
        except np.linalg.LinAlgError:
            break
        # Halve the step while the penalized likelihood would fall (guards the near-separated case).
        current = _log_likelihood(s, n, expit(design @ beta)) - HALF * beta @ penalty @ beta
        factor = 1.0
        while factor > CONVERGENCE_TOLERANCE:
            trial_beta = beta + factor * step
            if _log_likelihood(s, n, expit(design @ trial_beta)) - HALF * trial_beta @ penalty @ trial_beta >= current:
                break
            factor /= 2.0
        beta = beta + factor * step
        if float(np.max(np.abs(factor * step))) < CONVERGENCE_TOLERANCE:
            converged = True
            break

    p = expit(design @ beta)
    weights = n * p * (1.0 - p)
    information = design.T @ (design * weights[:, None]) + penalty
    separated = bool(columns and np.max(np.abs(beta[1:])) > SEPARATION_COEFFICIENT)
    with np.errstate(all="ignore"):
        if np.linalg.cond(information) < HESSIAN_CONDITION_LIMIT and not separated:
            covariance = np.linalg.inv(information)
        else:
            covariance = np.full(information.shape, np.nan)
    # Back to the original covariate units: b_j = beta_j / scale_j, b_0 = beta_0 - sum_j beta_j mean_j / scale_j.
    slopes = beta[1:] / scale
    intercept = beta[0] - float(np.sum(beta[1:] * mean / scale))
    conversion = np.zeros((columns + 1, columns + 1))
    conversion[0, 0] = 1.0
    conversion[0, 1:] = -mean / scale
    for j in range(columns):
        conversion[j + 1, j + 1] = 1.0 / scale[j]
    covariance_original = conversion @ covariance @ conversion.T
    log_likelihood = _log_likelihood(s, n, p)
    deviance = LIKELIHOOD_RATIO_FACTOR * (_saturated_log_likelihood(s, n) - log_likelihood)
    return LogisticFit(coefficients=np.concatenate([[intercept], slopes]),
                       standard_errors=np.sqrt(np.maximum(np.diag(covariance_original), 0.0)) if np.all(
                           np.isfinite(covariance_original)) else np.full(columns + 1, np.nan),
                       deviance=max(float(deviance), 0.0), log_likelihood=log_likelihood, converged=converged,
                       separated=separated, observations=int(s.size))


def likelihood_ratio_pvalue(full: LogisticFit, reduced: LogisticFit, extra_parameters: int = 1) -> float:
    """p-value of the likelihood-ratio test that the ``extra_parameters`` covariates dropped from ``full`` to get
    ``reduced`` carry no information (chi-square with that many degrees of freedom)."""
    statistic = max(LIKELIHOOD_RATIO_FACTOR * (full.log_likelihood - reduced.log_likelihood), 0.0)
    return float(chi2.sf(statistic, extra_parameters))
