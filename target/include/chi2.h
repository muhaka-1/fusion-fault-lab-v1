#ifndef CHI2_H
#define CHI2_H
/* Inverse chi-square CDF (same as scipy.stats.chi2.ppf), for gate thresholds. */
double chi2_ppf(double p, int dof);
double chi2_cdf(double x, int dof);
#endif
