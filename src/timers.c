/***********************************************************************
 *@file        timer.c
 *
 *@version     0.0.1
 *
 *@brief       letimer setup function file.
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
 *             discretion of author. Contact for permission.  */

#include "timers.h"
#include "app.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"



/**
 * This function initialize LE-TIMER0
 * Enable comp1 for letimer0
 * @param: void
 * @return void
 */
void letimer_init(){
  LETIMER_Init_TypeDef letimer_init_args = LETIMER_INIT_DEFAULT;
//  letimer_init_args.topValue = ACTUAL_COMP0_LOAD;         //Set value to 3000ms
//  letimer_init_args.comp0Top = true;                      //Reset CNT when underflow
  LETIMER_Init(LETIMER0,&letimer_init_args);              // DOS moved this up to here

//  LETIMER_CompareSet(LETIMER0,0,ACTUAL_COMP0_LOAD);       //Set comparator0 period
//  LETIMER_IntClear(LETIMER0,IF_UF);                       //Clear UF Interrupt
//  LETIMER_IntEnable(LETIMER0,IF_UF);                      //Enable UF Interrupt
//
  NVIC_ClearPendingIRQ(LETIMER0_IRQn);                    //Clear IRQ
  NVIC_EnableIRQ(LETIMER0_IRQn);                          //Enable IRQ

  LETIMER_Enable(LETIMER0,true);                          //Enable LETIMER
}




/**
 * sleep amount of time based on the user input. Noticed in timerWaitUs_irq
 * CPU is not polling, it only wakes up when the comparator 1 values matches
 * the value we set to.
 * @param[in]: millisecond to wait
 *             input range [1000, 3000000]
 *             1000:
 *             we are using ULFRCO(1kHz) in EM3
 *             it is not possible to reach to microsecond level
 *
 *             LETIMER_PERIOD_MS:
 *             Period is 3 second, busying polling more than
 *             3 second would be bad for power consumption +
 *             polling more than 3 second CPU will spend all
 *             its time waiting
 * */
void timerWaitUs_irq(uint32_t us_wait){

  if((us_wait < (1*MS_TO_US))){
       LOG_ERROR("ULFRCO only has precision of millisecond");
       us_wait = 80000;
   }
  //Convert from us time request to the number of ticks required
  float clk_period = (float)1/(float)(ACTUAL_CLK_FREQ);
  float s_wait = (float)us_wait/(float)US_TO_S;
  uint16_t tik_req = (uint16_t)(s_wait/clk_period);

  //Calculate the final tik needed
  uint16_t cur_tik = LETIMER_CounterGet(LETIMER0);
  uint16_t final_tik = (cur_tik - tik_req);         // target COMP1 value

  //Enable Comparator 1 interrupts
  LETIMER_CompareSet(LETIMER0,1,final_tik);
  LETIMER_IntClear(LETIMER0,IF_COMP1);
  LETIMER0->IEN |= IF_COMP1;
}

void delay_ms(uint32_t ms) {
  // 32khz
  uint32_t cnt = ms * 32;

  while (cnt--) {
      ;
  }
}

