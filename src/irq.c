/***********************************************************************
 *@file        irq.c
 *
 *@version     0.0.1
 *
 *@brief       interrupt function file.
 *
 *@author      Jiabin Lin, jili9036@Colorado.edu
 *
 *@date        Sep 3, 2021
 *
 *@institution University of Colorado Boulder (UCB)
 *
 *@course      ECEN 5823-001: IoT Embedded Firmware (Fall 2021)
 *
 *@instructor  David Sluiter
 *
 *@assignment  ecen5823-assignment2-JiabinLin12
 *
 *@due         Sep 10, 2020
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

// Include logging for this file
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"


#include <src/sml_state_machine.h>
#include <src/sml_adc.h>
#include <src/vl53_state_machine.h>

/***
 * Handle LETIMER0 interrupt
 * @param: void
 * */
void LETIMER0_IRQHandler(void){

  int letimer_flag = LETIMER_IntGetEnabled(LETIMER0);  //determine source of IRQ

  if(letimer_flag & LETIMER_IF_COMP1){
    LETIMER_IntDisable(LETIMER0, LETIMER_IF_COMP1);
    schedulerSetEventWaitUs();
  }

}



void GPIO_EVEN_IRQHandler(void){
  uint32_t gpio_flag = GPIO_IntGetEnabled();
  GPIO_IntClear(gpio_flag);
  //Either PB0 or bream beam
  if (!GPIO_PinInGet(PB0_PORT,PB0_PIN)
      && (gpio_flag & (1<<PB0_PIN))){
     pb_pressed_external_signal();
  }else if ((gpio_flag & (1<<BREAK_BEAM_PIN))){
      schedulerSetEventBreakBeam();
  }

}

void GPIO_ODD_IRQHandler(void)
{
  uint32_t gpio_flag = GPIO_IntGetEnabled();

  // Clear all odd pin interrupt flags
  GPIO_IntClear(0xAAAA);

  if (gpio_flag & (1U<<VL53L0X_GPIO1_PIN)) {
      vl_set_flag_measure_ready(true);
  }
}


void ADC0_IRQHandler(void)
{
  //Disable ADC interrupt
  IntDisable_ADC();
  // Get ADC result
  schedulerSetEventADCDone();
}



