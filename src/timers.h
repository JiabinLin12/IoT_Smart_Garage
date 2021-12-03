/***********************************************************************
 *@file        irq.h
 *
 *@version     0.0.1
 *
 *@brief       letimer header file.
 *
 *@author      Jiabin Lin, jili9036@Colorado.edu
 *
 *@date        Nov 13th, 2021
 *
 *@institution University of Colorado Boulder (UCB)
 *
 *@course      ECEN 5823-001: IoT Embedded Firmware (Fall 2021)
 *
 *@instructor  David Sluiter
 *
 *@assignment  ecen5823-assignment2-JiabinLin12
 *
 *@due         Dec 8th, 2020
 *
 *@resources   Utilized Silicon Labs' EMLIB peripheral libraries to
 *             implement functionality.
 *
 *
 *@copyright   All rights reserved. Distribution allowed only for the
 *             use of assignment grading. Use of code excerpts allowed at the
 *discretion of author. Contact for permission.  */

#ifndef SRC_TIMERS_H_
#define SRC_TIMERS_H_

#include "em_letimer.h"

#define MS_TO_US      1000
#define US_TO_MS      1000
#define S_TO_US       1000000
#define US_TO_S       1000000
#define S_TO_MS       1000

//Function prototype
void letimer_init();
void timerWaitUs_polled(uint32_t us_wait);
void timerWaitUs_irq(uint32_t us_wait);

void delay_ms(uint32_t ms);

#endif /* SRC_TIMERS_H_ */
