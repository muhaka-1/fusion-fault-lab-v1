/* Unit-test runner for the C target. Each module adds its suite here.
 * M7: E2E protection. Later: messages (M8), cluster core (M9, M10), nav (M12, M13). */
#include "test.h"
int t_run, t_fail, t_checks, t_cur_fail;
const char *t_name;

int main(void)
{
    suite_e2e();
    suite_msg();
    printf("%d/%d tests passed (%d checks)\n", t_run - t_fail, t_run, t_checks);
    return t_fail ? 1 : 0;
}
