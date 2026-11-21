"""Chi-square thresholds for the NIS tests (cached: scipy's ppf is slow per sample).

If the filter's model is right, the NIS of a measurement with k values follows a chi-square
distribution with k degrees of freedom. chi2_threshold(0.999, k) is the value that a correct
measurement stays below 99.9 % of the time: anything above it is suspicious.
"""
from functools import lru_cache
from scipy.stats import chi2


@lru_cache(maxsize=None)
def chi2_threshold(prob: float, dof: int) -> float:
    return float(chi2.ppf(prob, dof))
