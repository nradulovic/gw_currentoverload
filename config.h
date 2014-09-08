/* 
 * File:   config.h
 * Author: Nenad Radulovic
 *
 * Created on 13.12.2013., 12.37
 */

#ifndef CONFIG_H
#define	CONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <p18f1220.h>

#define CFG_TIME_INITIAL_WAIT           50u

#define CFG_TIME_FIND_ZERO              100u

#define CFG_TIME_POST_ZERO              20u

#define CFG_TIME_INITIAL_PEAK           200u

#define CFG_TIME_LED_BLINK              60u

#define CFG_CURRENT_MINIMAL_MA          1000l

#define CFG_CURRENT_INITIAL_MA          1000l

#define CFG_MUL_THRESHOLD               18l

#define CFG_SENS_UV_PER_AMPER           66000l
#define CFG_ADC_UV_QUANTUM              4882l
#define CFG_SENS_UNIT_INT               100l
 
#define RELAY_CW_PORT                   PORTB
#define RELAY_CW_TRIS                   TRISB
#define RELAY_CW_LAT                    LATB
#define RELAY_CW_PIN_Pos                1
#define RELAY_CW_PIN_Msk                (0x01u << RELAY_CW_PIN_Pos)

#define RELAY_CCW_PORT                  PORTB
#define RELAY_CCW_TRIS                  TRISB
#define RELAY_CCW_LAT                   LATB
#define RELAY_CCW_PIN_Pos               0
#define RELAY_CCW_PIN_Msk               (0x01u << RELAY_CCW_PIN_Pos)

#define RELAY_OL_PORT                   PORTA
#define RELAY_OL_TRIS                   TRISA
#define RELAY_OL_LAT                    LATA
#define RELAY_OL_PIN_Pos                3
#define RELAY_OL_PIN_Msk                (0x01u << RELAY_OL_PIN_Pos)

#define DEBUG_PIN_PORT                  PORTB
#define DEBUG_PIN_TRIS                  TRISB
#define DEBUG_PIN_LAT                   LATB
#define DEBUG_PIN_PIN_Pos               4
#define DEBUG_PIN_PIN_Msk               (0x01u << DEBUG_PIN_PIN_Pos)
  
#define SENS_PORT                       PORTA
#define SENS_TRIS                       TRISA
#define SENS_LAT                        LATA
#define SENS_PIN_Pos                    0
#define SENS_PIN_Msk                    (0x01u << SENS_PIN_Pos)

#define LED_MOT_PORT                    PORTA
#define LED_MOT_TRIS                    TRISA
#define LED_MOT_LAT                     LATA
#define LED_MOT_PIN_Pos                 4
#define LED_MOT_PIN_Msk                 (0x01u << LED_MOT_PIN_Pos)

#define LED_OL_PORT                     PORTA
#define LED_OL_TRIS                     TRISA
#define LED_OL_LAT                      LATA
#define LED_OL_PIN_Pos                  1
#define LED_OL_PIN_Msk                  (0x01u << LED_OL_PIN_Pos)

#define SENS_FREQUENCY                  1000l
#define SENS_INITIAL_ZERO               (1024l / 2l)

#define CFG_SIGNAL_SAMPLES_PER_POINT    16

#define CFG_SIGNAL_POINTS_PER_DIFF      8

#define CFG_SIGNAL_MAX_ZERO_DEVIATION   10

#define LVTMR_TICK_FREQUENCY            SENS_FREQUENCY
  
#define CPU_FREQUENCY                   40000000ul
#define SYSTICK_USE_DEDICATED_TMR       0

#define SENS_TRIG_DIVISOR               (CPU_FREQUENCY / (4ul * SENS_FREQUENCY))
  
#if   (SENS_TRIG_DIVISOR < 65534)
#define SENS_TRIG_PRESCALER             0x00
#define SENS_TRIG_PERIOD_COUNTER        SENS_TRIG_DIVISOR
#elif (SENS_TRIG_DIVISOR < 131070)
#define SENS_TRIG_PRESCALER             0x01
#define SENS_TRIG_PERIOD_COUNTER        (SENS_TRIG_DIVISOR / 2)
#elif (SENS_TRIG_DIVISOR < 262142)
#define SENS_TRIG_PRESCALER             0x02
#define SENS_TRIG_PERIOD_COUNTER        (SENS_TRIG_DIVISOR / 4)
#elif (SENS_TRIG_DIVISOR < 524286)
#define SENS_TRIG_PRESCALER             0x04
#define SENS_TRIG_PERIOD_COUNTER        (SENS_TRIG_DIVISOR / 8)
#else
# error "Invalid setting for sensor trigger frequency"
#endif

#if (CFG_SIGNAL_SAMPLES_PER_POINT != 2) && (CFG_SIGNAL_SAMPLES_PER_POINT != 4) &&                     \
    (CFG_SIGNAL_SAMPLES_PER_POINT != 8) && (CFG_SIGNAL_SAMPLES_PER_POINT != 16)
#error "Invalid settting for number of samples per data point"
#endif
            
#ifdef	__cplusplus
}
#endif

#endif	/* CONFIG_H */

