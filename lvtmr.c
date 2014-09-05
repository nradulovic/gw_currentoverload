
#include "lvtmr.h"
#include "main.h"

#define TMR_IS_FREE             (lvtmr_tick)-1

static lvtmr_tick g_lvtmr[LVTMR_MAX_NUM_OF_TIMERS];

void lvtmr_init(void)
{
    lvtmr               cnt;

    cnt = LVTMR_MAX_NUM_OF_TIMERS;

    while (cnt != 0u) {
        cnt--;
        g_lvtmr[cnt] = TMR_IS_FREE;
    }
}

lvtmr lvtmr_create_i(
    lvtmr_tick          tick)
{
    lvtmr               cnt;

    cnt = LVTMR_MAX_NUM_OF_TIMERS;

    while (cnt != 0u) {
        cnt--;

        if (g_lvtmr[cnt] == TMR_IS_FREE) {
            g_lvtmr[cnt] = tick;

            return (cnt);
        }
    }

    return ((lvtmr)-1);
}

bool lvtmr_is_done_i(
    lvtmr               tmr)
{
    if (g_lvtmr[tmr] != 0) {

        return (false);
    } else {
        g_lvtmr[tmr] = TMR_IS_FREE;

        return (true);
    }
}

void lvtmr_delete_i(
    lvtmr               tmr)
{
    g_lvtmr[tmr] = TMR_IS_FREE;
}

void lvtmr_eval_i(void)
{
    lvtmr               cnt;

    cnt = LVTMR_MAX_NUM_OF_TIMERS;

    while (cnt != 0) {
        cnt--;

        if ((g_lvtmr[cnt] != 0) && (g_lvtmr[cnt] != TMR_IS_FREE)) {
            g_lvtmr[cnt]--;
        }
    }
}

lvtmr lvtmr_create(
    lvtmr_tick                  tick)
{
    lvtmr                       ret;

    critical_enter();
    ret = lvtmr_create_i(tick);
    critical_exit();

    return (ret);
}

bool lvtmr_is_done(
    lvtmr                       tmr)
{
    bool                        ret;

    critical_enter();
    ret = lvtmr_is_done_i(tmr);
    critical_exit();

    return (ret);
}

void lvtmr_delete(
    lvtmr                       tmr)
{
    critical_enter();
    lvtmr_delete_i(tmr);
    critical_exit();
}
