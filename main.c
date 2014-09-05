/* 
 * File:   main.c
 * Author: Nenad Radulovic
 *
 * Created on 13.12.2013., 12.30
 */

/*=========================================================  INCLUDE FILES  ==*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <xc.h>

#include "hw_config.h"
#include "config.h"
#include "lvtmr.h"
#include "main.h"

/*=========================================================  LOCAL MACRO's  ==*/

/**@brief       Calculate log2 for value @c x during the compilation
 */
#define BIT_UINT8_LOG2(x)                                                       \
    ((x) <   2u ? 0u :                                                          \
     ((x) <   4u ? 1u :                                                         \
      ((x) <   8u ? 2u :                                                        \
       ((x) <  16u ? 3u :                                                       \
        ((x) <  32u ? 4u :                                                      \
         ((x) <  64u ? 5u :                                                     \
          ((x) < 128u ? 6u : 7u)))))))

#define SIGNAL_FROM_MA(current)                                                 \
    ((signal)((int32_t)(current) * (int32_t)CFG_SENS_UNIT_INT / (int32_t)1000l))

#define QSIGNAL_FROM_MA(current)                                                \
    ((signal)((int32_t)(current) * (int32_t)CFG_SENS_UNIT_INT * (int32_t)CFG_SENS_UNIT_INT / (int32_t)1000l))

#define ABS(x)                              ((x < 0) ? (-x) : x)

#define RELAY_CCW_SET_ACTIVE()                                                  \
  do                                                                            \
  {                                                                             \
    RELAY_CCW_LAT |= RELAY_CCW_PIN_Msk;                                         \
  }                                                                             \
  while (0)

#define RELAY_CCW_SET_INACTIVE()                                                \
  do                                                                            \
  {                                                                             \
    RELAY_CCW_LAT &= ~RELAY_CCW_PIN_Msk;                                        \
  }                                                                             \
  while (0)

#define RELAY_CW_SET_ACTIVE()                                                   \
  do                                                                            \
  {                                                                             \
    RELAY_CW_LAT |= RELAY_CW_PIN_Msk;                                           \
  }                                                                             \
  while (0)

#define RELAY_CW_SET_INACTIVE()                                                 \
  do                                                                            \
  {                                                                             \
    RELAY_CW_LAT &= ~RELAY_CW_PIN_Msk;                                          \
  }                                                                             \
  while (0)

#define RELAY_OL_SET_ACTIVE()                                                   \
  do                                                                            \
  {                                                                             \
    RELAY_OL_LAT |= RELAY_OL_PIN_Msk;                                           \
  }                                                                             \
  while (0)

#define RELAY_OL_SET_INACTIVE()                                                 \
  do                                                                            \
  {                                                                             \
    RELAY_OL_LAT &= ~RELAY_OL_PIN_Msk;                                          \
  }                                                                             \
  while (0)

#define LED_MOT_SET_ACTIVE()                                                    \
  do                                                                            \
  {                                                                             \
    LED_MOT_LAT &= ~LED_MOT_PIN_Msk;                                            \
  }                                                                             \
  while (0)

#define LED_MOT_SET_INACTIVE()                                                  \
  do                                                                            \
  {                                                                             \
    LED_MOT_LAT |= LED_MOT_PIN_Msk;                                             \
  }                                                                             \
  while (0)

#define LED_OL_SET_ACTIVE()                                                     \
  do                                                                            \
  {                                                                             \
    LED_OL_LAT &= ~LED_OL_PIN_Msk;                                              \
  }                                                                             \
  while (0)

#define LED_OL_SET_INACTIVE()                                                   \
  do                                                                            \
  {                                                                             \
    LED_OL_LAT |= LED_OL_PIN_Msk;                                               \
  }                                                                             \
  while (0)

#if !defined(NDEBUG)
#define DEBUG_PIN_SET_ACTIVE()                                                  \
  do                                                                            \
  {                                                                             \
    DEBUG_PIN_LAT |= DEBUG_PIN_PIN_Msk;                                         \
  }                                                                             \
  while (0)

#define DEBUG_PIN_SET_INACTIVE()                                                \
  do                                                                            \
  {                                                                             \
    DEBUG_PIN_LAT &= ~DEBUG_PIN_PIN_Msk;                                        \
  }                                                                             \
  while (0)
#else
#define DEBUG_PIN_SET_ACTIVE()                                                  \
  (void)0

#define DEBUG_PIN_SET_INACTIVE()                                                \
  (void)0
#endif

#define TASK_COMPUTE                        (0x1u << 0)
#define TASK_TICK                           (0x1u << 1)

/*======================================================  LOCAL DATA TYPES  ==*/

enum display_cmd
{
    LED_ON,
    LED_OFF,
    LED_BLINK
};

typedef int16_t signal;
typedef int32_t signal_diff;

/*=============================================  LOCAL FUNCTION PROTOTYPES  ==*/

static void hw_init(void);

#if !defined(NDEBUG)
static void debug_pin_init(void);
#endif
static void osc_init(void);
static void relay_cw_init(void);
static void relay_ccw_init(void);
static void relay_ol_init(void);
static void sens_init(void);
static void sens_trigger_init(void);
static void sens_set_zero(int16_t zero);
static void sens_isr(void);
static void signal_insert_sample(void);
static void sys_tick_init(void);
static void sys_tick_isr(void);
static void led_ol_init(void);
static void led_mot_init(void);
static void led_ol_fsm(
  enum display_cmd       cmd);
static void led_mot_fsm(
  enum display_cmd       cmd);
void interrupt hi_isr(
  void);
void low_priority interrupt lo_isr(
  void);

/*=======================================================  LOCAL VARIABLES  ==*/

static volatile uint8_t         g_scheduled;
static signal                   g_signal;
static signal                   g_signal_abs;
static signal_diff              g_signal_diff1;
static signal_diff              g_signal_diff2;
static int16_t                  g_sens_zero = SENS_INITIAL_ZERO;

static enum display_cmd         g_led_mot_state;
static enum display_cmd         g_led_ol_state;

/*======================================================  GLOBAL VARIABLES  ==*/
/*============================================  LOCAL FUNCTION DEFINITIONS  ==*/

#define is_scheduled_i(task)                (g_scheduled & (task))
#define sched_ready_i(task)                  g_scheduled |= (task)
#define sched_block_i(task)                  g_scheduled &= ~(task)
#define scheduled()                          g_scheduled

static void hw_init(void)
{
  INTCONbits.GIEH       = 0;                                                    /* Disable all high prio interrupts                         */
  INTCONbits.GIEL       = 0;                                                    /* Disable all low  prio interrupts                         */
  RCONbits.IPEN         = 1;                                                    /* Enable ISR priorities                                    */
  PIE1                  = 0;
  PIE2                  = 0;
  osc_init();
#if !defined(NDEBUG)
  debug_pin_init();
#endif
  relay_cw_init();
  relay_ccw_init();
  relay_ol_init();
  led_ol_init();
  led_mot_init();
  sens_init();
  sens_trigger_init();
  sys_tick_init();
  INTCONbits.GIEH       = 1;                                                    /* Enable all high prio interrupts                          */
  INTCONbits.GIEL       = 1;                                                    /* Enable all low  prio interrupts                          */
}

/*-- Debug pin ---------------------------------------------------------------*/
#if !defined(NDEBUG)
static void debug_pin_init(void)
{
    DEBUG_PIN_LAT  &= ~DEBUG_PIN_PIN_Msk;
    DEBUG_PIN_PORT &= ~DEBUG_PIN_PIN_Msk;
    DEBUG_PIN_TRIS &= ~DEBUG_PIN_PIN_Msk;
}
#endif

/*-- Osc ---------------------------------------------------------------------*/
static void osc_init(void)
{
  OSCCONbits.IDLEN      = 0;                                                    /* Run mode enabled                                         */
#if (CPU_FREQUENCY == 8000000ul)
    {
        uint8_t             retry;

        retry = UINT8_MAX;

        while ((OSCCONbits.OSTS == 0x0) && (retry-- != 0));

        if (OSCCONbits.OSTS == 1) {
            OSCCONbits.SCS0 = 0;                                                /* Set to internal oscilator                                */
            OSCCONbits.SCS1 = 1;
        } else {
            STAT_SET_FAIL(STAT_OBJ_OSC);
        }
    }
#elif (CPU_FREQUENCY == 32000000ul)
    {
        uint8_t             retry;

        retry = UINT8_MAX;

        while ((OSCCONbits.OSTS == 0x0) && (retry-- != 0));

        if (OSCCONbits.OSTS == 1) {
            OSCCONbits.SCS0 = 0;                                                /* Set to external oscilator                                */
            OSCCONbits.SCS1 = 0;
        } else {
            STAT_SET_FAIL(STAT_OBJ_OSC);
        }
    }
#else
    {
        uint8_t                 retry;

        retry = UINT8_MAX;

        while ((OSCCONbits.OSTS == 0x0) && (retry-- != 0));

        if (OSCCONbits.OSTS == 1) {
            OSCCONbits.SCS0 = 0;                                                /* Set to external oscilator                                */
            OSCCONbits.SCS1 = 0;
        } else {
            /* Panic here */
            while (1);
        }
    }
#endif
}

/*-- Relay functions ---------------------------------------------------------*/
static void relay_cw_init(void)
{
  RELAY_CW_LAT  &= ~RELAY_CW_PIN_Msk;
  RELAY_CW_PORT &= ~RELAY_CW_PIN_Msk;
  RELAY_CW_TRIS &= ~RELAY_CW_PIN_Msk;
}

static void relay_ccw_init(void)
{
  RELAY_CCW_LAT  &= ~RELAY_CCW_PIN_Msk;
  RELAY_CCW_PORT &= ~RELAY_CCW_PIN_Msk;
  RELAY_CCW_TRIS &= ~RELAY_CCW_PIN_Msk;
}

static void relay_ol_init(void)
{
  RELAY_OL_LAT  &= ~RELAY_OL_PIN_Msk;
  RELAY_OL_PORT &= ~RELAY_OL_PIN_Msk;
  RELAY_OL_TRIS &= ~RELAY_OL_PIN_Msk;
}

/*-- Sensor functions --------------------------------------------------------*/

#define ADC_CLOCK_SOURCE_2Tosc          0x00u
#define ADC_CLOCK_SOURCE_4Tosc          0x04u
#define ADC_CLOCK_SOURCE_8Tosc          0x01u
#define ADC_CLOCK_SOURCE_16Tosc         0x05u
#define ADC_CLOCK_SOURCE_32Tosc         0x02u
#define ADC_CLOCK_SOURCE_64Tosc         0x06u
#define ADC_CLOCK_SOURCE_Frc            0x03u

#define ADC_ACQUISITION_0Tad            0x00u
#define ADC_ACQUISITION_2Tad            0x01u
#define ADC_ACQUISITION_4Tad            0x02u
#define ADC_ACQUISITION_6Tad            0x03u
#define ADC_ACQUISITION_8Tad            0x04u
#define ADC_ACQUISITION_12Tad           0x05u
#define ADC_ACQUISITION_16Tad           0x06u
#define ADC_ACQUISITION_20Tad           0x07u

static void sens_init(
  void)
{
  ADCON0bits.ADON       = 0;                                                    /* A/D converter module is off                              */
  SENS_LAT             &= ~SENS_PIN_Msk;
  SENS_PORT            &= ~SENS_PIN_Msk;
  SENS_TRIS            |=  SENS_PIN_Msk;
  ADCON2bits.ADCS       = ADC_CLOCK_SOURCE_64Tosc;                              /* A/D conversion clock selection                           */
  ADCON2bits.ACQT       = ADC_ACQUISITION_20Tad;                                /* A/D conversion acquisition time selection                */
  ADCON2bits.ADFM       = 1;                                                    /* A/D result right justified                               */
  ADCON1bits.PCFG      &= ~SENS_PIN_Msk;                                        /* Configure pins as analog                                 */
  ADCON0bits.VCFG       = 0;                                                    /* Voltage regerence set to AVdd and AVss                   */
  ADCON0bits.CHS        = SENS_PIN_Pos;                                         /* Select the current channel                               */
  ADCON0bits.GO         = 0;                                                    /* ADC is stopped                                           */
  ADCON0bits.ADON       = 1;                                                    /* A/D converter module is on                               */
  IPR1bits.ADIP         = 1;                                                    /* High ISR                                                 */
  PIR1bits.ADIF         = 0;
  PIE1bits.ADIE         = 1;
}

static void sens_trigger_init(
  void)
{
  T3CONbits.TMR3ON      = 0;                                                    /* Stop if enabled                                          */
  T3CONbits.T3CCP1      = 0;                                                    /* TMR1 is the clock source for CCP                         */
  T1CONbits.TMR1ON      = 0;
  T1CONbits.RD16        = 0;
  T1CONbits.T1RUN       = 0;
  T1CONbits.T1CKPS      = SENS_TRIG_PRESCALER;
  T1CONbits.T1OSCEN     = 0;
  T1CONbits.nT1SYNC     = 0;
  T1CONbits.TMR1CS      = 0;                                                    /* Use internal clock                                       */
  T3CONbits.TMR3CS      = 0;
  TMR1L                 = 0;
  TMR1H                 = 0;
  CCP1CONbits.CCP1M     = 0x0b;                                                 /* Compare mode, set ECCP1IF, reset TMR1 or TMR3, start ADC */
  CCPR1H                = ((SENS_TRIG_PERIOD_COUNTER) >> 8) & 0xff;
  CCPR1L                = ((SENS_TRIG_PERIOD_COUNTER) >> 0) & 0xff;
  IPR1bits.CCP1IP       = 1;                                                    /* High ISR                                                 */
  PIR1bits.CCP1IF       = 0;
  PIE1bits.CCP1IE       = 0;
  IPR1bits.TMR1IP       = 1;                                                    /* High ISR                                                 */
  PIR1bits.TMR1IF       = 0;
  PIE1bits.TMR1IE       = 0;
  T1CONbits.TMR1ON      = 1;
}

static void sens_set_zero(
    int16_t                     zero)
{
    g_sens_zero  = zero / CFG_SIGNAL_SAMPLES_PER_POINT;
    g_sens_zero += SENS_INITIAL_ZERO;

    if (g_sens_zero > (SENS_INITIAL_ZERO + CFG_SIGNAL_MAX_ZERO_DEVIATION)) {
        g_sens_zero =  SENS_INITIAL_ZERO + CFG_SIGNAL_MAX_ZERO_DEVIATION;
    } else if (g_sens_zero < (SENS_INITIAL_ZERO - CFG_SIGNAL_MAX_ZERO_DEVIATION)) {
        g_sens_zero = SENS_INITIAL_ZERO - CFG_SIGNAL_MAX_ZERO_DEVIATION;
    }
}

#define sens_read()                         ((int16_t)ADRES - g_sens_zero)

static void sens_isr(void)
{
    if (PIR1bits.ADIF == 1) {
        PIR1bits.ADIF = 0;
        signal_insert_sample();
    }
}

/*-- Signal functions --------------------------------------------------------*/

static void signal_insert_sample(void)
{
    static signal               signal_acc;
    static uint8_t              signal_index;
    
    signal_acc += (signal)sens_read();
    signal_index++;

    if (signal_index == CFG_SIGNAL_SAMPLES_PER_POINT) {
        signal_index = 0;

        g_signal   = (int24_t)signal_acc
            * (int24_t)(10l * CFG_SENS_UNIT_INT / CFG_SIGNAL_SAMPLES_PER_POINT)
            / (int24_t)(10l * CFG_SENS_UV_PER_AMPER / CFG_ADC_UV_QUANTUM);
        signal_acc = 0;
        sched_ready_i(TASK_COMPUTE);
    }
}

static signal_diff signal_compute_diff1(
    signal                      signal_sample)
{
    signal_diff                 ret;
    
    static signal               signal_buff[CFG_SIGNAL_POINTS_PER_DIFF];
    static uint8_t              signal_idx;

    signal_buff[signal_idx++] = signal_sample;
    
    if (signal_idx == CFG_SIGNAL_POINTS_PER_DIFF) {
        signal_idx = 0;
    }
    ret = (signal_diff)signal_sample - (signal_diff)signal_buff[signal_idx];

    return (ret);
}

static signal_diff signal_compute_diff2(
    signal_diff                 diff_sample)
{
    signal_diff                 ret;
    
    static signal_diff          diff_buff[CFG_SIGNAL_POINTS_PER_DIFF];
    static uint8_t              diff_idx;

    diff_buff[diff_idx++] = diff_sample;
    
    if (diff_idx == CFG_SIGNAL_POINTS_PER_DIFF) {
        diff_idx = 0;
    }
    ret = diff_sample - diff_buff[diff_idx];

    return (ret);
}

static void signal_process(void)
{
    g_signal_abs   = ABS(g_signal);
    g_signal_diff1 = signal_compute_diff1(g_signal_abs);

    if (g_signal_diff1 < 0) {
        g_signal_diff1 = 0;
    }
    g_signal_diff2 = signal_compute_diff2(g_signal_diff1);
}

/*-- Systick functions -------------------------------------------------------*/

static void sys_tick_init(void)
{
#if (SYSTICK_USE_DEDICATED_TMR == 1)
  T0CONbits.TMR0ON      = 0;                                                    /* Turn TMR0 off if it was on                               */
  TMR0L                 = 0;
  TMR0H                 = 0;
  T0CONbits.T08BIT      = 0;                                                    /* 16bit mode                                               */
  T0CONbits.T0CS        = 0;                                                    /* Internal instruction clock CLK0                          */
  T0CONbits.T0SE        = 1;                                                    /* Increment on lot-to-high transition                      */
  T0CONbits.PSA         = 1;                                                    /* Prescaler is used                                        */
  T0CONbits.T0PS        = 0;                                                    /* 1/2                                                      */
  INTCON2bits.TMR0IP    = 0;                                                    /* Low ISR                                                  */
  INTCONbits.TMR0IF     = 0;
  INTCONbits.TMR0IE     = 1;
  T0CONbits.TMR0ON      = 1;                                                    /* Start timer                                              */
#else
#endif
}

static void sys_tick_isr(void)
{
#if (SYSTICK_USE_DEDICATED_TMR == 1)
    if (INTCONbits.TMR0IF == 0x1) {
        INTCONbits.TMR0IF = 0;
        lvtmr_eval_i();
    }
#else
    lvtmr_eval_i();
#endif
}

static void led_ol_init(void)
{
    LED_OL_LAT  &= ~LED_OL_PIN_Msk;
    LED_OL_PORT &= ~LED_OL_PIN_Msk;
    LED_OL_TRIS &= ~LED_OL_PIN_Msk;
}

static void led_mot_init(void)
{
    LED_MOT_LAT  &= ~LED_MOT_PIN_Msk;
    LED_MOT_PORT &= ~LED_MOT_PIN_Msk;
    LED_MOT_TRIS &= ~LED_MOT_PIN_Msk;
}

static void led_ol_fsm(
    enum display_cmd            cmd)
{
    enum display_state {
        DISP_ON,
        DISP_OFF,
        DISP_BLINK_ON,
        DISP_BLINK_OFF
    };
    static enum display_state   state = DISP_OFF;
    static lvtmr                led_tmr;

    switch (state) {
        case DISP_ON : {
            LED_OL_SET_ACTIVE();

            if (cmd == LED_OFF) {
                state = DISP_OFF;
            } else if (cmd == LED_BLINK) {
                led_tmr = lvtmr_create(LVTMR_MS_TO_TICK(CFG_TIME_LED_BLINK));
                state   = DISP_BLINK_OFF;
            }
            break;
        }
        case DISP_OFF : {
            LED_OL_SET_INACTIVE();

            if (cmd == LED_ON) {
                state = DISP_ON;
            } else if (cmd == LED_BLINK) {
                led_tmr = lvtmr_create(LVTMR_MS_TO_TICK(CFG_TIME_LED_BLINK));
                state   = DISP_BLINK_ON;
            }
            break;
        }
        case DISP_BLINK_ON : {
            LED_OL_SET_ACTIVE();

            if (cmd == LED_ON) {
                lvtmr_delete(led_tmr);
                state = DISP_ON;
            } else if (cmd == LED_OFF) {
                lvtmr_delete(led_tmr);
                state = DISP_OFF;
            } else if (lvtmr_is_done(led_tmr)) {
                led_tmr = lvtmr_create(LVTMR_MS_TO_TICK(CFG_TIME_LED_BLINK));
                state   = DISP_BLINK_OFF;
            }
            break;
        }
        case DISP_BLINK_OFF : {
            LED_OL_SET_INACTIVE();

            if (cmd == LED_ON) {
                lvtmr_delete(led_tmr);
                state = DISP_ON;
            } else if (cmd == LED_OFF) {
                lvtmr_delete(led_tmr);
                state = DISP_OFF;
            } else if (lvtmr_is_done(led_tmr)) {
                led_tmr = lvtmr_create(LVTMR_MS_TO_TICK(CFG_TIME_LED_BLINK));
                state   = DISP_BLINK_ON;
            }
            break;
        }
    }
}

static void led_mot_fsm(
    enum display_cmd            cmd)
{
    enum display_state {
        DISP_ON,
        DISP_OFF,
        DISP_BLINK_ON,
        DISP_BLINK_OFF
    };
    static enum display_state   state = DISP_OFF;
    static lvtmr                led_tmr;

    switch (state) {
        case DISP_ON : {
            LED_MOT_SET_ACTIVE();

            if (cmd == LED_OFF) {
                state = DISP_OFF;
            } else if (cmd == LED_BLINK) {
                led_tmr = lvtmr_create(LVTMR_MS_TO_TICK(CFG_TIME_LED_BLINK));
                state   = DISP_BLINK_OFF;
            }
            break;
        }
        case DISP_OFF : {
            LED_MOT_SET_INACTIVE();

            if (cmd == LED_ON) {
                state = DISP_ON;
            } else if (cmd == LED_BLINK) {
                led_tmr = lvtmr_create(LVTMR_MS_TO_TICK(CFG_TIME_LED_BLINK));
                state   = DISP_BLINK_ON;
            }
            break;
        }
        case DISP_BLINK_ON : {
            LED_MOT_SET_ACTIVE();

            if (cmd == LED_ON) {
                lvtmr_delete(led_tmr);
                state = DISP_ON;
            } else if (cmd == LED_OFF) {
                lvtmr_delete(led_tmr);
                state = DISP_OFF;
            } else if (lvtmr_is_done(led_tmr)) {
                led_tmr = lvtmr_create(LVTMR_MS_TO_TICK(CFG_TIME_LED_BLINK));
                state   = DISP_BLINK_OFF;
            }
            break;
        }
        case DISP_BLINK_OFF : {
            LED_MOT_SET_INACTIVE();

            if (cmd == LED_ON) {
                lvtmr_delete(led_tmr);
                state = DISP_ON;
            } else if (cmd == LED_OFF) {
                lvtmr_delete(led_tmr);
                state = DISP_OFF;
            } else if (lvtmr_is_done(led_tmr)) {
                led_tmr = lvtmr_create(LVTMR_MS_TO_TICK(CFG_TIME_LED_BLINK));
                state   = DISP_BLINK_ON;
            }
            break;
        }
    }
}


static void main_fsm(void)
{
    enum main_state {
        STATE_INIT,
        STATE_PRE_WAIT,
        STATE_CALIBRATE,
        STATE_POST_WAIT,
        STATE_SEARCH_STARTUP,
        STATE_INITIAL_PEAK,
        STATE_PROCESS_SIGNAL,
        STATE_OVERLOAD
    };
    static enum main_state      state = STATE_INIT;
    static lvtmr                main_tmr;
    static int32_t              signal_acc;
    static uint8_t              signal_idx;

    switch (state) {
        case STATE_INIT : {
            main_tmr        = lvtmr_create(LVTMR_MS_TO_TICK(CFG_TIME_INITIAL_WAIT));
            g_led_mot_state = LED_OFF;
            g_led_ol_state  = LED_OFF;
            RELAY_CCW_SET_INACTIVE();
            RELAY_CW_SET_INACTIVE();
            RELAY_OL_SET_INACTIVE();
            state           = STATE_PRE_WAIT;
            break;
        }
        case STATE_PRE_WAIT : {
            if (lvtmr_is_done(main_tmr)) {
                main_tmr   = lvtmr_create(LVTMR_MS_TO_TICK(CFG_TIME_FIND_ZERO));
                signal_acc = 0;
                signal_idx = 0;
                state      = STATE_CALIBRATE;
            }
            break;
        }
        case STATE_CALIBRATE : {
            signal_acc += (int32_t)g_signal;
            signal_idx++;

            if (lvtmr_is_done(main_tmr)) {
                int16_t         new_zero;

                main_tmr = lvtmr_create(LVTMR_MS_TO_TICK(CFG_TIME_POST_ZERO));
                new_zero = (int16_t)(signal_acc / (int32_t)signal_idx);
                sens_set_zero(new_zero);
                state    = STATE_POST_WAIT;
            }
            break;
        }
        case STATE_POST_WAIT : {
            if (lvtmr_is_done(main_tmr)) {
                state = STATE_SEARCH_STARTUP;
            }
            break;
        }
        case STATE_SEARCH_STARTUP : {

            if (g_signal_abs > SIGNAL_FROM_MA(CFG_CURRENT_INITIAL_MA)) {
                main_tmr = lvtmr_create(LVTMR_MS_TO_TICK(CFG_TIME_INITIAL_PEAK));
                state    = STATE_INITIAL_PEAK;
            }
            break;
        }
        case STATE_INITIAL_PEAK : {
            
            if (lvtmr_is_done(main_tmr)) {
                state = STATE_PROCESS_SIGNAL;
            }
            break;
        }
        case STATE_PROCESS_SIGNAL : {
            int32_t             mul;

            if (g_signal > SIGNAL_FROM_MA(CFG_CURRENT_MINIMAL_MA)) {
                RELAY_CCW_SET_ACTIVE();
                RELAY_CW_SET_INACTIVE();
                g_led_mot_state = LED_BLINK;
            } else if (g_signal < -SIGNAL_FROM_MA(CFG_CURRENT_MINIMAL_MA)) {
                RELAY_CCW_SET_INACTIVE();
                RELAY_CW_SET_ACTIVE();
                g_led_mot_state = LED_BLINK;
            } else {
                RELAY_CCW_SET_INACTIVE();
                RELAY_CW_SET_INACTIVE();
                g_led_mot_state = LED_OFF;
            }
            mul = (int32_t)g_signal_diff1 * (int32_t)g_signal_diff2;

            if (mul >= (CFG_MUL_THRESHOLD * CFG_SENS_UNIT_INT * CFG_SENS_UNIT_INT)) {
                RELAY_OL_SET_ACTIVE();
                g_led_ol_state = LED_ON;
            }

            break;
        }
        default: {
            break;
        }
    }
}

/*===================================  GLOBAL PRIVATE FUNCTION DEFINITIONS  ==*/
/*====================================  GLOBAL PUBLIC FUNCTION DEFINITIONS  ==*/

int main(
    int                         argc,
    char **                     argv)
{
    (void)argc;
    (void)argv;
    lvtmr_init();
    hw_init();

    while (true) {
        uint8_t                 tasks;

        tasks = scheduled();


        if (tasks) {
            DEBUG_PIN_SET_ACTIVE();

            if (tasks & TASK_COMPUTE) {
                signal_process();
                main_fsm();
                critical_enter();
                sched_block_i(TASK_COMPUTE);
                critical_exit();
            }

            if (tasks & TASK_TICK) {
                led_mot_fsm(g_led_mot_state);
                led_ol_fsm(g_led_ol_state);
                critical_enter();
                sched_block_i(TASK_TICK);
                critical_exit();
            }
            DEBUG_PIN_SET_INACTIVE();
        }
    }

    return (EXIT_SUCCESS);
}

void interrupt hi_isr(void)
{
    sens_isr();
    sys_tick_isr();
    sched_ready_i(TASK_TICK);
}

void low_priority interrupt lo_isr(void)
{
    sys_tick_isr();
}

/*================================*//** @cond *//*==  CONFIGURATION ERRORS  ==*/
/** @endcond *//** @} *//** @} *//*********************************************
 * END of main.c
 ******************************************************************************/
