/*
 * smart_light_state_machine.h
 *
 *  Created on: Nov 11, 2021
 *      Author: jiabinlin
 */

#ifndef SRC_SML_STATE_MACHINE_H_
#define SRC_SML_STATE_MACHINE_H_

#define LIGHT_SENSOR_THRESHOLD  2500
#define SENSOR_WAKE_UP_TIME     5000
#define LIGHT_MAINTAIN_TIME     5000000
typedef enum {
  STATE_SL_IDLE=100,      //start with 100 prevent collision from teammates
  STATE_SL_SENSOR_WAKE,
  STATE_SL_ADC_WORK,
  STATE_SL_DOWNCOUNT
}state_smart_light;


typedef enum {
  break_beam_event =100,
  adc_trigger_event,
}event_smart_light;
void schedulerSetEventBreakBeam();
void schedulerSetEventADCDone();
void smart_light_state_machine(sl_bt_msg_t *evt_struct);

#endif /* SRC_SML_STATE_MACHINE_H_ */
