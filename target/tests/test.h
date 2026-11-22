/* A minimal unit-test framework: no dependencies, works on any C11 compiler.
 *   CHECK(cond)          the condition must be true
 *   CHECK_EQ(a, b)       integers must be equal (prints both on failure)
 *   CHECK_NEAR(a, b, t)  doubles within tolerance t
 *   RUN(fn)              run one test function, count it as failed if any check failed
 */
#ifndef TEST_H
#define TEST_H
#include <math.h>
#include <stdio.h>

extern int t_run, t_fail, t_checks;
extern const char *t_name;
extern int t_cur_fail;

#define CHECK(cond) do { t_checks++; if (!(cond)) { t_cur_fail = 1; \
    printf("  FAIL %s:%d %s: %s\n", __FILE__, __LINE__, t_name, #cond); } } while (0)
#define CHECK_EQ(a, b) do { long long _a = (long long)(a), _b = (long long)(b); t_checks++; if (_a != _b) { t_cur_fail = 1; \
    printf("  FAIL %s:%d %s: %s == %s (%lld vs %lld)\n", __FILE__, __LINE__, t_name, #a, #b, _a, _b); } } while (0)
#define CHECK_NEAR(a, b, tol) do { double _a = (a), _b = (b); t_checks++; if (!(fabs(_a - _b) <= (tol))) { t_cur_fail = 1; \
    printf("  FAIL %s:%d %s: %s ~ %s (%.10g vs %.10g, tol %g)\n", __FILE__, __LINE__, t_name, #a, #b, _a, _b, (double)(tol)); } } while (0)
#define RUN(fn) do { t_name = #fn; t_cur_fail = 0; t_run++; fn(); if (t_cur_fail) t_fail++; } while (0)

void suite_e2e(void);   /* M7 */
void suite_msg(void);   /* M8 */
void suite_core(void);  /* M9, M10 */
void suite_nav(void);   /* M12 */
#endif
