#include "chi2.h"
#include <math.h>

/* Regularized lower incomplete gamma P(a, x): series for x < a + 1, Lentz continued fraction otherwise. */
static double gammp(double a, double x)
{
    if (x <= 0.0) return 0.0;
    const double gln = lgamma(a);
    if (x < a + 1.0) {
        double ap = a, sum = 1.0 / a, del = sum;
        for (int n = 0; n < 1000; ++n) {
            ap += 1.0;
            del *= x / ap;
            sum += del;
            if (fabs(del) < fabs(sum) * 1e-17) break;
        }
        return sum * exp(-x + a * log(x) - gln);
    }
    const double tiny = 1e-300;
    double b = x + 1.0 - a, c = 1.0 / tiny, d = 1.0 / b, h = d;
    for (int i = 1; i < 1000; ++i) {
        const double an = -i * (i - a);
        b += 2.0;
        d = an * d + b; if (fabs(d) < tiny) d = tiny;
        c = b + an / c; if (fabs(c) < tiny) c = tiny;
        d = 1.0 / d;
        const double del = d * c;
        h *= del;
        if (fabs(del - 1.0) < 1e-17) break;
    }
    return 1.0 - exp(-x + a * log(x) - gln) * h;
}

double chi2_cdf(double x, int dof) { return gammp(0.5 * dof, 0.5 * x); }

double chi2_ppf(double p, int dof)
{
    if (dof <= 0 || !(p > 0.0)) return 0.0;
    if (p >= 1.0) return INFINITY;
    if (dof == 2) return -2.0 * log1p(-p);       /* closed form */
    double lo = 0.0, hi = dof + 10.0;
    while (chi2_cdf(hi, dof) < p) hi *= 2.0;
    for (int i = 0; i < 200 && hi - lo > 1e-15 * hi; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (chi2_cdf(mid, dof) < p) lo = mid; else hi = mid;
    }
    return 0.5 * (lo + hi);
}
