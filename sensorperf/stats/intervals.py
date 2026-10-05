"""
Binomial and bootstrap confidence intervals.

Serves Analysis D (Section 13 of the characterization procedure, Step 6: the
empirical D_0 bound from one-sided Clopper-Pearson upper bounds, and the rule
of three of Section 8, Step 4) and every analysis that bootstraps over poses
(``BOOTSTRAP_RESAMPLES``; Analyses C, D and E).

Conventions: probabilities and proportions are dimensionless numbers in
[0, 1]; ``confidence`` is the coverage of the interval or bound, for example
0.95 (``CONFIDENCE_LEVEL``).

The Clopper-Pearson ("exact") interval inverts the binomial test through the
beta distribution. It is conservative: the actual coverage is at least the
nominal one for every true probability, which is the property wanted for a
"practically zero detection" claim.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Sequence

import numpy as np
from scipy.stats import beta

TAIL_COUNT_TWO_SIDED = 2.0
"""A two-sided interval at confidence c leaves (1 - c) / 2 in each tail."""

UNIT_PROBABILITY = 1.0
"""Upper end of the probability scale (certain detection)."""

ZERO_PROBABILITY = 0.0
"""Lower end of the probability scale (no detection)."""


@dataclass
class BootstrapResult:
    """Outcome of ``bootstrap_statistic``."""

    estimate: float | np.ndarray
    """The statistic evaluated on the original, unresampled groups."""
    lower: float | np.ndarray
    """Lower percentile limit, at (1 - confidence) / 2 of the resampled statistics (NaN when none succeeded)."""
    upper: float | np.ndarray
    """Upper percentile limit, at 1 - (1 - confidence) / 2 of the resampled statistics (NaN when none succeeded)."""
    samples: np.ndarray
    """Statistic values of the successful resamples, shape (n_successful, ...) in resample order."""
    failures: int
    """Number of resamples skipped because the statistic raised an exception or returned NaN."""


def _check_binomial_counts(successes: int, trials: int, confidence: float) -> None:
    """Reject counts and confidence levels that do not describe a binomial experiment."""
    if trials < 1:
        raise ValueError(f"trials must be at least 1, got {trials}")
    if not 0 <= successes <= trials:
        raise ValueError(f"successes must lie in [0, trials]; got {successes} of {trials}")
    if not 0.0 < confidence < 1.0:
        raise ValueError(f"confidence must lie strictly between 0 and 1, got {confidence}")


def clopper_pearson_upper(successes: int, trials: int, confidence: float) -> float:
    """One-sided exact upper bound on a binomial probability (Section 13, Step 6).

    With s successes in n trials, the bound p_U is the probability at which
    observing s or fewer successes has probability 1 - confidence, which is
    the ``confidence`` quantile of Beta(s + 1, n - s). It equals 1 when
    s == n. For s = 0 it reduces to 1 - (1 - c)^(1/n), about 3/n at 95 percent
    (the rule of three). Returns a probability in [0, 1].
    """
    _check_binomial_counts(successes, trials, confidence)
    if successes == trials:
        return UNIT_PROBABILITY
    return float(beta.ppf(confidence, successes + 1, trials - successes))


def clopper_pearson(successes: int, trials: int, confidence: float) -> tuple[float, float]:
    """Two-sided exact binomial interval (lower, upper), probabilities in [0, 1].

    Each tail holds (1 - confidence) / 2. The lower limit is the
    (1 - c) / 2 quantile of Beta(s, n - s + 1) and is 0 when s == 0; the upper
    limit is the 1 - (1 - c) / 2 quantile of Beta(s + 1, n - s) and is 1 when
    s == n.
    """
    _check_binomial_counts(successes, trials, confidence)
    tail = (UNIT_PROBABILITY - confidence) / TAIL_COUNT_TWO_SIDED
    lower = ZERO_PROBABILITY if successes == 0 else float(beta.ppf(tail, successes, trials - successes + 1))
    upper = UNIT_PROBABILITY if successes == trials else float(
        beta.ppf(UNIT_PROBABILITY - tail, successes + 1, trials - successes))
    return lower, upper


def bootstrap_statistic(groups: Sequence[Any], statistic: Callable[[Sequence[Any]], float | np.ndarray],
                        resamples: int, confidence: float, rng: np.random.Generator) -> BootstrapResult:
    """Percentile bootstrap over groups (poses) with replacement.

    Whole groups are resampled, never the items inside them: frames or trials
    of one pose share that pose's sub-pixel phase and are not independent, so
    the pose is the unit of replication (Section 13; Analyses C, D, E). Each
    resample draws len(groups) groups with replacement, hands the list of
    drawn groups to ``statistic`` and records the result. A resample whose
    statistic raises an exception or returns NaN (for example a resample that
    contains no level crossing) is skipped and counted in ``failures``, so the
    caller can judge how far the interval is trustworthy. The interval is the
    (1 - confidence) / 2 and 1 - (1 - confidence) / 2 percentiles of the
    successful samples; ``estimate`` is the statistic on the original groups
    (an exception there propagates, since no meaningful result exists).
    """
    if resamples < 1:
        raise ValueError(f"resamples must be at least 1, got {resamples}")
    if not 0.0 < confidence < 1.0:
        raise ValueError(f"confidence must lie strictly between 0 and 1, got {confidence}")
    group_list = list(groups)
    if not group_list:
        raise ValueError("bootstrap needs at least one group")

    estimate = statistic(group_list)
    kept: list[np.ndarray] = []
    failures = 0
    for _ in range(resamples):
        chosen = rng.integers(0, len(group_list), size=len(group_list))
        try:
            value = np.asarray(statistic([group_list[index] for index in chosen]), dtype=float)
        except Exception:  # noqa: BLE001 - any statistic failure is counted, not hidden: see BootstrapResult.failures
            failures += 1
            continue
        if np.any(np.isnan(value)):
            failures += 1
            continue
        kept.append(value)

    tail_percent = 100.0 * (UNIT_PROBABILITY - confidence) / TAIL_COUNT_TWO_SIDED
    if kept:
        samples = np.stack(kept, axis=0)
        lower = np.percentile(samples, tail_percent, axis=0)
        upper = np.percentile(samples, 100.0 - tail_percent, axis=0)
    else:
        samples = np.empty((0,) + np.shape(estimate), dtype=float)
        lower = np.full(np.shape(estimate), np.nan)
        upper = np.full(np.shape(estimate), np.nan)
    if np.ndim(lower) == 0:
        lower, upper = float(lower), float(upper)
    return BootstrapResult(estimate=estimate, lower=lower, upper=upper, samples=samples, failures=failures)
