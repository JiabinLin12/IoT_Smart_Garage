/***********************************************************************
 *@file        ble.h
 *
 *@version     0.0.1
 *
 *@brief       ble related function declaration
 *
 *@author      Jiabin Lin, jili9036@Colorado.edu
 *
 *@date        Sep 26th, 2021
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

#ifndef SRC_BLE_H_
#define SRC_BLE_H_
#include <stdbool.h>
#include "sl_bt_api.h"

#include "app.h"

#define UINT8_TO_BITSTREAM(p,n)   {*(p)++ = (uint8_t)(n);}
#define UINT32_TO_BITSTREAM(p,n)  {*(p)++ = (uint8_t)(n); *(p)++ = (uint8_t)((n)>>8); \
                                   *(p)++ = (uint8_t)((n)>>16); *(p)++=(uint8_t)((n)>>24);}
#define UINT32_TO_FLOAT(m, e)     (((uint32_t)(m) & 0x00FFFFFFU) | (uint32_t)((int32_t)(e) << 24))

//advertiser set timing
#define MIN_ADV_INTERVAL    400    // min. adv. interval (milliseconds * 1.6 = 250*1.6)
#define MAX_ADV_INTERVAL    400    // max. adv. interval (milliseconds * 1.6 = 250*1.6)
#define ADV_DURATION        0
#define MAX_ADV_EVENT       0

//connection events parameters
#define MIN_CNT_INTERVAL    60       //min. cnt. interval (75ms / 1.25ms = 60)
#define MAX_CNT_INVERVAL    60       //max. cnt. interval (75ms / 1.25ms = 60)
#define SLAVE_LATENCY       4        //slave latency 4*75ms = 300ms
#define SUPERVISON_TIMEOUT  800      //Supervision timeout((1 + latency) * max_interval * 2) = 45150us = 0.4515s ~ 5s
                                     //700 accommodate the real latency of 90
#define MIN_CNT_EVT_LENGTH  0
#define MAX_CNT_EVT_LENGTH  0xffff

#define SM_CONFIG_FLAG      0x0F

#define PRINT_PARAMS        0
typedef struct {
  //value that are common to servers and clients
  bool connection_enable;

  //value unique for server
  bool indication_in_flight;

  bool htm_indication_enable;
  bool motion_indication_enable;
  bool light_indication_enable;
  bool car_lot_indication_enable;

  bool smart_garage_bonded;

  bool smart_garage_confirmation_require;

  //value unique for client
}ble_data_struct_t;


ble_data_struct_t *get_ble_data();
uint8_t *get_connection_handle();

void handle_ble_event(sl_bt_msg_t *evt);
#endif /* SRC_BLE_H_ */
