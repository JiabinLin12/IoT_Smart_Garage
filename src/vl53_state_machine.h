/*
 * ridar_test.h
 *
 *  Created on: Oct 5, 2021
 *      Author: han16
 */

#ifndef RIDAR_FSM_H_
#define RIDAR_FSM_H_

#include "app.h"

#include <stdint.h>
#include <stdbool.h>

/**
 * vl53l0x state
 * */
typedef enum {
  vlst_IDLE                = 0,
  vlst_Boot,
  vlst_MeasureRequest,
  vlst_WaitForStartBit,
  vlst_WaitForMeasure,
  vlst_MeasureCompleted,
  vlst_Error,
}vl_state_t;


/**
 * @brief
 *  FLAG - Enable
 * */
void vl_set_flag_enable(bool is_enable);
bool vl_get_flag_enable();

/**
 * @brief
 *  FLAG - Chip Measurement Completed
 * @details
 *  flag will be set in irq.c, clear in ridar_fsm.c
 * */
void vl_set_flag_measure_ready(bool is_enable);
bool vl_get_flag_measure_ready();

/**
 * @brief
 *  FLAG - Chip Data Ready
 * @details
 *  flag will be clear in ble.c, set in ridar_fsm.c
 * */
void vl_set_flag_data_ready(bool is_enable);
bool vl_get_flag_data_ready();

/**
 * @brief
 *  vl53l0x range result (mm)
 * */
void vl_set_result(uint16_t val);
uint16_t vl_get_result();


void ridar_fsm(sl_bt_msg_t *evt_struct);

#endif
