/*
 * This file is part of Current Monitor.
 *
 * web site:    http://github.com/nradulovic
 * e-mail  :    nenad.b.radulovic@gmail.com
 *//***********************************************************************//**
 * @file
 * @author      Nenad Radulovic
 *********************************************************************//** @{ */

#ifndef LVTMR_H__
#define	LVTMR_H__

/*=========================================================  INCLUDE FILES  ==*/

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/*===============================================================  MACRO's  ==*/

#define LVTMR_MAX_NUM_OF_TIMERS         8

#define LVTMR_MS_TO_TICK(ms)                                                    \
  (((uint32_t)(ms) * (uint32_t)LVTMR_TICK_FREQUENCY) / (uint32_t)1000u)

/*------------------------------------------------------  C++ extern begin  --*/
#ifdef __cplusplus
extern "C" {
#endif

/*============================================================  DATA TYPES  ==*/

typedef uint8_t lvtmr;
typedef uint16_t lvtmr_tick;

/*======================================================  GLOBAL VARIABLES  ==*/
/*===================================================  FUNCTION PROTOTYPES  ==*/

void lvtmr_init(void);

lvtmr lvtmr_create_i(
    lvtmr_tick                  tick);

bool lvtmr_is_done_i(
    lvtmr                       tmr);

void lvtmr_delete_i(
    lvtmr                       tmr);

lvtmr lvtmr_create(
    lvtmr_tick                  tick);

bool lvtmr_is_done(
    lvtmr                       tmr);

void lvtmr_delete(
    lvtmr                       tmr);

void lvtmr_eval_i(void);

/*--------------------------------------------------------  C++ extern end  --*/
#ifdef __cplusplus
}
#endif

/*================================*//** @cond *//*==  CONFIGURATION ERRORS  ==*/
/** @endcond *//** @} *//******************************************************
 * END of lvtmr.h
 ******************************************************************************/
#endif /* LVTMR_H__ */

