/* 
 * File:   main.h
 * Author: nenad
 *
 * Created on August 27, 2014, 7:15 PM
 */

#ifndef MAIN_H
#define	MAIN_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

#define INTCON_GIEH                         (0x1u << 7)
#define INTCON_GIEL                         (0x1u << 6)

#define critical_enter()                    INTCON &= ~(INTCON_GIEH | INTCON_GIEL)
#define critical_exit()                     INTCON |=   INTCON_GIEH | INTCON_GIEL

#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */

