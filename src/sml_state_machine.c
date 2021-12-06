/*
 * smart_light_state_machine.c
 *
 *  Created on: Nov 11, 2021
 *      Author: jiabinlin
 */

#include "sl_bt_api.h"
#include "em_adc.h"
#include "em_core.h"

// Include logging for this file
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"


#include <src/sml_state_machine.h>
#include <src/sml_adc.h>
#include "scheduler.h" //letimer_flag
#include "gpio.h"
#include "timers.h"

void smart_light_state_machine(sl_bt_msg_t *evt_struct){
  state_smart_light  current_state;
  static   state_smart_light next_state = STATE_SL_IDLE;
  uint32_t signal;

  uint16_t adcSample;

  current_state = next_state;

  if (SL_BT_MSG_ID(evt_struct->header) != sl_bt_evt_system_external_signal_id){
       return;
  }
  signal = evt_struct->data.evt_system_external_signal.extsignals;

  if(signal == break_beam_event)
    current_state = STATE_SL_IDLE;

  switch(current_state){
    case STATE_SL_IDLE:
      next_state = STATE_SL_IDLE;
      if(signal==break_beam_event){
          light_sensor_enable();
          motion_to_client_indication(0x1);
          timerWaitUs_irq(SENSOR_WAKE_UP_TIME);
          next_state = STATE_SL_SENSOR_WAKE;
      }
      break;

    case STATE_SL_SENSOR_WAKE:
      next_state = STATE_SL_SENSOR_WAKE;
      if(signal ==letimer_comp1_expired){
          motion_to_client_indication(0x0);
          IntEnable_ADC();
          ADC_Start(ADC0, adcStartSingle);
          next_state = STATE_SL_ADC_WORK;
      }
      break;

    case STATE_SL_ADC_WORK:
      next_state = STATE_SL_ADC_WORK;
      if(signal == adc_trigger_event){
          adcSample = ADC_DataSingleGet(ADC0);
          light_sensor_disable();
          LOG_INFO("%d", adcSample);
           if(adcSample < LIGHT_SENSOR_THRESHOLD){
              gpioLed0SetOn();
              light_to_client_indication(0x1);
              timerWaitUs_irq(LIGHT_MAINTAIN_TIME);
              next_state = STATE_SL_DOWNCOUNT;
          }else{
              next_state = STATE_SL_IDLE;
          }
      }
      break;

    case STATE_SL_DOWNCOUNT:
      next_state = STATE_SL_DOWNCOUNT;
      if(signal ==letimer_comp1_expired){
          gpioLed0SetOff();
          light_to_client_indication(0x0);
          next_state = STATE_SL_IDLE;
      }
      break;

    default:
      break;
  }
}


void schedulerSetEventBreakBeam(){
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();                                            //Enter critical section while processing the queue
  sl_bt_external_signal(break_beam_event);
  CORE_EXIT_CRITICAL();
}

void schedulerSetEventADCDone(){
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();                                            //Enter critical section while processing the queue
  sl_bt_external_signal(adc_trigger_event);
  CORE_EXIT_CRITICAL();
}
