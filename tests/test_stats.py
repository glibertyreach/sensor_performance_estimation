"""
Tests of sensorperf.stats (intervals and psychometric fits).

Each test names the step of the characterization procedure it exercises and
its acceptance criterion. All random draws come from seeded generators.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from sensorperf.parameters import CharacterizationParameters
from sensorperf.stats.intervals import bootstrap_statistic, clopper_pearson, clopper_pearson_upper
from sensorperf.stats.logistic import fit_logistic, likelihood_ratio_pvalue
from sensorperf.stats.psychometric import (CURVE_NAMES, PsychometricFitParameters, best_fit, corrected_rate,
                                           fit_all_curves, fit_psychometric, fit_threshold_model,
                                           isotonic_threshold)

PARAMS = CharacterizationParameters()
SEED = 20261005
TRUE_D50_MM = 2.0
TRUE_BETA = 0.35
TRUE_GAMMA = 0.01
TRUE_LAMBDA = 0.02
D50_TOLERANCE = 0.10
D10_TOLERANCE = 0.25
EXACT_TOLERANCE = 1.0e-12
RULE_OF_THREE_CEILING = 0.0105
TEST_LEVELS = 9
"""Levels of the synthetic psychometric data sets of these tests."""
TEST_LEVEL_LOW_FACTOR = 0.3
TEST_LEVEL_HIGH_FACTOR = 2.5
"""The synthetic levels run from 0.3 to 2.5 times the true D_50."""


def simulate_logistic_counts(rng: np.random.Generator):
    """Nine log-spaced levels (0.3 to 2.5 times D_50), 60 trials each, from a known logistic curve."""
    levels = TRUE_D50_MM * np.geomspace(TEST_LEVEL_LOW_FACTOR, TEST_LEVEL_HIGH_FACTOR, TEST_LEVELS)
    z = (np.log(levels) - math.log(TRUE_D50_MM)) / TRUE_BETA
    psi = TRUE_GAMMA + (1.0 - TRUE_GAMMA - TRUE_LAMBDA) / (1.0 + np.exp(-z))
    trials = np.full(levels.shape, PARAMS.detection_trials_per_level)
    return levels, rng.binomial(trials, psi), trials


def test_clopper_pearson_bounds():
    """Section 13, Step 6 and Section 8, Step 4: exact bounds; 0 of 300 gives the rule-of-three bound."""
    upper = clopper_pearson_upper(0, PARAMS.detection_zero_trials, PARAMS.confidence_level)
    expected = 1.0 - (1.0 - PARAMS.confidence_level) ** (1.0 / PARAMS.detection_zero_trials)
    assert abs(upper - expected) < EXACT_TOLERANCE
    assert upper < RULE_OF_THREE_CEILING
    assert clopper_pearson_upper(20, 20, PARAMS.confidence_level) == 1.0
    lower, upper2 = clopper_pearson(5, 20, PARAMS.confidence_level)
    assert lower < 0.25 < upper2
    assert clopper_pearson(0, 20, PARAMS.confidence_level)[0] == 0.0
    assert clopper_pearson(20, 20, PARAMS.confidence_level)[1] == 1.0


def test_bootstrap_recovers_mean_and_counts_failures():
    """Bootstrap over poses (Section 13): the interval covers the truth; failing resamples are counted."""
    rng = np.random.default_rng(SEED)
    true_mean, group_count = 3.0, 50
    groups = [rng.normal(true_mean, 1.0, size=5) for _ in range(group_count)]
    sample_mean = float(np.mean(np.concatenate(groups)))

    def mean_statistic(chosen):
        return float(np.mean(np.concatenate(chosen)))

    result = bootstrap_statistic(groups, mean_statistic, PARAMS.bootstrap_resamples, PARAMS.confidence_level, rng)
    assert result.estimate == pytest.approx(sample_mean)
    assert result.lower < true_mean < result.upper
    assert result.failures == 0
    assert result.samples.shape == (PARAMS.bootstrap_resamples,)

    calls = {"count": 0}
    failing_every = 4

    def sometimes_fails(chosen):
        calls["count"] += 1
        if calls["count"] > 1 and calls["count"] % failing_every == 0:
            raise RuntimeError("no crossing in this resample")
        return mean_statistic(chosen)

    resamples = 100
    flaky = bootstrap_statistic(groups, sometimes_fails, resamples, PARAMS.confidence_level, rng)
    assert flaky.failures > 0
    assert flaky.samples.shape[0] + flaky.failures == resamples


def test_psychometric_fit_recovers_logistic_truth():
    """Section 13, Steps 4-5: D_50 within 10 percent, D_10 within 25 percent, lambda inside its bounds."""
    rng = np.random.default_rng(SEED)
    levels, successes, trials = simulate_logistic_counts(rng)
    fit_params = PsychometricFitParameters(lapse_rate_max=PARAMS.detection_lapse_rate_max)
    fit = fit_psychometric(levels, successes, trials, TRUE_GAMMA, fit_params)
    true_d10 = TRUE_D50_MM * math.exp(-TRUE_BETA * math.log(9.0))
    assert abs(fit.threshold(0.5) / TRUE_D50_MM - 1.0) < D50_TOLERANCE
    assert abs(fit.threshold(0.1) / true_d10 - 1.0) < D10_TOLERANCE
    assert 0.0 <= fit.lapse_rate <= PARAMS.detection_lapse_rate_max
    assert fit.threshold(0.1) == pytest.approx(math.exp(fit.alpha - fit.beta * math.log(9.0)), rel=1e-12)
    assert fit.threshold(0.5) == pytest.approx(math.exp(fit.alpha), rel=1e-12)
    assert fit.corrected_probability(fit.threshold(0.1)) == pytest.approx(0.1)

    fits = fit_all_curves(levels, successes, trials, TRUE_GAMMA, fit_params)
    assert set(fits) == set(CURVE_NAMES) and len(fits) == 3
    winner = best_fit(fits)
    assert winner.deviance == min(f.deviance for f in fits.values())
    for fit_i in fits.values():
        assert fit_i.deviance >= 0.0
        assert 0.0 <= fit_i.lapse_rate <= PARAMS.detection_lapse_rate_max


def test_isotonic_threshold():
    """Section 13, Step 5 check: monotone data interpolate linearly (in ln level); noisy data stay bracketed."""
    levels = np.array([1.0, 2.0, 4.0, 8.0])
    monotone = np.array([0.0, 0.2, 0.8, 1.0])
    # The 0.5 crossing lies halfway in value between 2 and 4, hence halfway in ln level.
    assert isotonic_threshold(levels, monotone, 0.5) == pytest.approx(math.sqrt(2.0 * 4.0))
    assert math.isnan(isotonic_threshold(levels, np.array([0.0, 0.1, 0.2, 0.3]), 0.5))

    noisy_levels = np.geomspace(1.0, 9.0, 9)
    noisy = np.array([0.0, 0.1, 0.05, 0.4, 0.3, 0.7, 0.6, 0.95, 1.0])
    crossing = isotonic_threshold(noisy_levels, noisy, 0.5)
    # Raw proportions bracket 0.5 between levels 5 (index 4, 0.3) and 6 (index 5, 0.7).
    assert noisy_levels[4] <= crossing <= noisy_levels[5]
    assert isotonic_threshold(noisy_levels, noisy, 0.5, trials=np.full(9, 60)) == pytest.approx(crossing, rel=0.5)


def test_threshold_model_recovers_floor():
    """Section 13, Step 7: D_0 = 1.0 mm lies inside a profile-likelihood interval of positive width."""
    rng = np.random.default_rng(SEED)
    true_d0, true_scale, true_shape = 1.0, 1.2, 2.0
    levels = np.linspace(0.5, 4.0, TEST_LEVELS)
    trials = np.full(levels.shape, PARAMS.detection_trials_per_level)
    probability = -np.expm1(-np.power(np.maximum(levels - true_d0, 0.0) / true_scale, true_shape))
    successes = rng.binomial(trials, probability)
    fit = fit_threshold_model(levels, successes / trials, trials, PARAMS.confidence_level)
    assert fit.d0_lower <= true_d0 <= fit.d0_upper
    assert fit.d0_upper > fit.d0_lower
    assert fit.d0_lower <= fit.d0 <= fit.d0_upper
    assert fit.scale > 0.0 and fit.shape > 0.0


def test_corrected_rate():
    """Section 13, Step 4: P* = (psi - gamma) / (1 - gamma), clipped at 0."""
    assert corrected_rate(TRUE_GAMMA, TRUE_GAMMA) == 0.0
    assert corrected_rate(1.0, TRUE_GAMMA) == 1.0
    assert corrected_rate(0.0, TRUE_GAMMA) == 0.0
    assert np.allclose(corrected_rate(np.array([0.01, 0.505]), TRUE_GAMMA), [0.0, 0.5])


def test_stratified_bootstrap_keeps_the_group_count_of_every_stratum():
    """Section 13 (pooled design): with strata, every resample draws as many groups from each stratum as it has, so a
    station is never over- or under-represented."""
    groups = [(stratum, index) for stratum in range(3) for index in range(10 + 5 * stratum)]
    strata = [g[0] for g in groups]
    seen_counts = []

    def statistic(drawn):
        seen_counts.append(tuple(sum(1 for g in drawn if g[0] == stratum) for stratum in range(3)))
        return float(len(drawn))

    bootstrap_statistic(groups, statistic, 20, PARAMS.confidence_level, np.random.default_rng(SEED), strata=strata)
    assert set(seen_counts) == {(10, 15, 20)}
    with pytest.raises(ValueError):
        bootstrap_statistic(groups, statistic, 5, PARAMS.confidence_level, np.random.default_rng(SEED), strata=[0])


def test_logistic_regression_recovers_the_noise_covariate():
    """Section 13 (noise covariate): grouped logistic regression on ln D_px and ln sigma recovers known coefficients
    within a few standard errors; the likelihood-ratio test rejects dropping a true covariate and keeps a useless one."""
    rng = np.random.default_rng(SEED)
    true = np.array([-3.4, 1.5, -1.0])
    d_px = np.tile(np.geomspace(3.0, 96.0, 9), 3)
    log_sigma = rng.normal(0.0, 0.5, d_px.size)
    design = np.column_stack([np.log(d_px), log_sigma])
    p = 1.0 / (1.0 + np.exp(-(true[0] + design @ true[1:])))
    trials = np.full(d_px.size, PARAMS.detection_trials_per_level)
    successes = rng.binomial(trials, p)
    full = fit_logistic(design, successes, trials)
    assert full.converged and not full.separated
    assert np.all(np.abs(full.coefficients - true) < 4.0 * full.standard_errors)
    assert likelihood_ratio_pvalue(full, fit_logistic(design[:, :1], successes, trials)) < 1.0e-3
    useless = np.column_stack([np.log(d_px), rng.normal(0.0, 0.5, d_px.size)])
    p_useless = 1.0 / (1.0 + np.exp(-(true[0] + true[1] * useless[:, 0])))
    successes_useless = rng.binomial(trials, p_useless)
    assert likelihood_ratio_pvalue(fit_logistic(useless, successes_useless, trials),
                                   fit_logistic(useless[:, :1], successes_useless, trials)) > 1.0e-3


def test_logistic_regression_flags_separation():
    """A step in the data (all misses below a size, all detections above it) is reported as separated, with finite
    coefficients (the ridge) and no standard errors."""
    d_px = np.geomspace(3.0, 96.0, 10)
    trials = np.full(10, 20)
    successes = np.where(d_px > 10.0, 20, 0)
    fit = fit_logistic(np.log(d_px)[:, None], successes, trials)
    assert fit.separated and np.all(np.isfinite(fit.coefficients)) and np.all(np.isnan(fit.standard_errors))
