/***********************************************************************
 *@file        scheduler.c
 *
 *@version     0.0.1
 *
 *@brief       implement scheduler for interrupt events and its
 *@brief       helper funtions
 *
 *@author      Jiabin Lin, jili9036@Colorado.edu
 *
 *@date        Nov 13rd, 2021
 *
 *@institution University of Colorado Boulder (UCB)
 *
 *@course      ECEN 5823-001: IoT Embedded Firmware (Fall 2021)
 *
 *@instructor  David Sluiter
 *
 *@assignment  final-project
 *
 *@due         Dec 8th, 2021
 *
 *@resources   Utilized Silicon Labs' EMLIB peripheral libraries to
 *             implement functionality.
 *
 *
 *@copyright   All rights reserved. Distribution allowed only for the
 *             use of assignment grading. Use of code excerpts allowed at the
 *             discretion of author. Contact for permission.  */

#include "scheduler.h"
#include "em_core.h"
#include "gpio.h"
// Include logging for this file
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

#include "ble.h"
#include "gatt_db.h"

#include "lcd.h"
CirQueue_t PushButtonQ;

CirQueue_t *getPbQ(){
  return (&PushButtonQ);
}


/**
 * This function initialize the circular queue struct
 * @param: void
 * return void
 */

void PbCirQ_init(){
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  PushButtonQ.writeptr = 0;
  PushButtonQ.readptr = 0;
  PushButtonQ.isfull=false;
  CORE_EXIT_CRITICAL();
}
/**
 * helper function check if the queue is empty
 * **/
static inline bool isEmpty(){
  CirQueue_t *PbQ = getPbQ();
  return ((*PbQ).writeptr==(*PbQ).readptr) && (!(*PbQ).isfull);
}

/**
 * enqueue the circular queue
 * @param[in]: element that needs to be enqueue
 * @return[out]: true if enqueue successfully
 *               false otherwise
 * */

static bool PbCirQ_EnQ(QueueElement_t element){
  CORE_DECLARE_IRQ_STATE;

  CirQueue_t *PbQ = getPbQ();

  //Case 1: FIFO is full
  if((*PbQ).isfull){
      return false;
  }
  else{
    CORE_ENTER_CRITICAL();
    (*PbQ).elements[(*PbQ).writeptr] = element;
    (*PbQ).writeptr = ((*PbQ).writeptr+1==CIR_QUEUE_SIZE)? 0:(*PbQ).writeptr+1;
    (*PbQ).isfull = ((*PbQ).writeptr==(*PbQ).readptr);
    CORE_EXIT_CRITICAL();
  }
  return true;
}


/**
 * dequeue the event
 * @return: return the oldest element in the queue
 * */

static QueueElement_t PbCirQ_DeQ(){
  CORE_DECLARE_IRQ_STATE;
  CirQueue_t *PbQ = getPbQ();

  QueueElement_t element;
  element.invalid_element = true;

  //Empty Queue
  if(isEmpty()){
      return element;
  }
  //get element
  element = (*PbQ).elements[(*PbQ).readptr];
  element.invalid_element = false;

  if ((*PbQ).isfull==true){
    (*PbQ).isfull = false;
  }
  CORE_ENTER_CRITICAL();
  PushButtonQ.readptr = ((*PbQ).readptr+1==CIR_QUEUE_SIZE)? 0:(*PbQ).readptr+1;     //Move head to the next element
  CORE_EXIT_CRITICAL();
  return element;
}


/**
 * Set button press event
 * */
void pb_pressed_external_signal(){
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();                                            //Enter critical section while processing the queue
  sl_bt_external_signal(bt_ext_sig_pb0_pressed);
  CORE_EXIT_CRITICAL();
}


/**
 * set WaitUs event to the queue
 * */
void schedulerSetEventWaitUs(){
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();                                            //Enter critical section while processing the queue
  sl_bt_external_signal(letimer_comp1_expired);
  CORE_EXIT_CRITICAL();
}


/**
 * check if the light gatt is ok to indicate
 * */
static inline bool light_ok_to_indicate(){
  ble_data_struct_t *ble_data_loc = get_ble_data();
  return (ble_data_loc->light_indication_enable &&
          ble_data_loc->connection_enable &&
         !ble_data_loc->indication_in_flight &&
          ble_data_loc->smart_garage_bonded);
}

/**
 * check if the motion gatt is ok to indicate
 * */
static inline bool motion_ok_to_indicate(){
  ble_data_struct_t *ble_data_loc = get_ble_data();
  return (ble_data_loc->motion_indication_enable &&
          ble_data_loc->connection_enable &&
         !ble_data_loc->indication_in_flight &&
          ble_data_loc->smart_garage_bonded);
}


/**
 * Update local gatt attribute
 *  @param[in]: element contains all the gatt information to store
 *  in local gatt database
 *
 * */
void update_gatt_db(QueueElement_t element){
  sl_status_t sc;
  sc = sl_bt_gatt_server_write_attribute_value( element.charactristic,              // handle from gatt_db.h
                                                0,                      // offset
                                                element.bufferLength,   // length
                                                &element.buffer[0] );   // pointer to buffer where data is

  if(sc!=SL_STATUS_OK){
    LOG_ERROR("pb gatt write attribute failed %x", sc);
  }
}


/**
 * Function to sent indication to server
 * @param[in]: element contains all the gatt information to sent
 * **/
void send_indication(QueueElement_t element){
  sl_status_t sc;
  ble_data_struct_t *ble_data_loc = get_ble_data();

  if(!element.invalid_element){
    sc=sl_bt_gatt_server_send_indication(element.charHandle,
                                         element.charactristic,
                                         element.bufferLength,
                                         &element.buffer[0]);
    if(sc!=SL_STATUS_OK){
      LOG_ERROR("pb gatt send indication failed %x", sc);
    }else{
      ble_data_loc->indication_in_flight = true;
    }
  }
}
/**
 * light server to sent indication to client
 * @param: light state on(0x01) or off(0x00)
 * **/
void light_to_client_indication(uint8_t light_state){
  uint8_t *connection_handle_loc = get_connection_handle();
  ble_data_struct_t *ble_data_loc = get_ble_data();
  QueueElement_t element;
  element.buffer[0] = 0x00;
  element.buffer[1] = light_state;
  element.bufferLength = 2;
  element.charHandle=*connection_handle_loc;
  element.invalid_element = false;
  element.charactristic = gattdb_light_state;

  //update_pb_gatt_db(element);
  update_gatt_db(element);

  //No connection detected
  if(!(*ble_data_loc).connection_enable)
    return;

  //good to send indication
  if(light_ok_to_indicate()){
      send_indication(element);

  //enqueue if it is not ok to send indication
  }else{
      PbCirQ_EnQ(element);
  }
}
/**
 * motion server to sent indication to client
 * @param: motion state on(0x01) or off(0x00)
 * **/
void motion_to_client_indication(uint8_t motion_state){
  uint8_t *connection_handle_loc = get_connection_handle();
  ble_data_struct_t *ble_data_loc = get_ble_data();
  QueueElement_t element;
  element.buffer[0] = 0x00;
  element.buffer[1] = motion_state;
  element.bufferLength = 2;
  element.charHandle=*connection_handle_loc;
  element.invalid_element = false;
  element.charactristic = gattdb_motion_state;
  //update_pb_gatt_db(element);
  update_gatt_db(element);

  //No connection detected
  if(!(*ble_data_loc).connection_enable)
    return;

  //good to send indication
  if(motion_ok_to_indicate()){
      send_indication(element);

  //enqueue if it is not ok to send indication
  }else{
      PbCirQ_EnQ(element);
  }
}


/**
 * Push button for bonding confirmation
 * */
void pushbutton0_response(){
  sl_status_t sc;
  uint8_t *connection_handle_loc = get_connection_handle();
  ble_data_struct_t *ble_data_loc = get_ble_data();

  //No connection detected
  if(!(*ble_data_loc).connection_enable)
    return;

  //confirmation needed
  if(((*ble_data_loc).smart_garage_confirmation_require) && (!(*ble_data_loc).smart_garage_bonded)){
      sc = sl_bt_sm_passkey_confirm(*connection_handle_loc,1);
      if(sc!=SL_STATUS_OK){
          LOG_ERROR("passkey confirm failed %d", sc);
       }
      (*ble_data_loc).smart_garage_confirmation_require = false;
  }
}


/**
 * Dequeue and send indication based on a soft timer
 * */
void soft_timer_deq_indication(){
  QueueElement_t element;

  if(light_ok_to_indicate()&& !isEmpty()){
      element = PbCirQ_DeQ();
      send_indication(element);
  }
}


