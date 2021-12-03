/***********************************************************************
 *@file        ble.c
 *
 *@version     0.0.1
 *
 *@brief       function to handle ble event
 *@brief
 *
 *@author      Jiabin Lin, jili9036@Colorado.edu
 *
 *@date        Sep 26, 2021
 *
 *@institution University of Colorado Boulder (UCB)
 *
 *@course      ECEN 5823-001: IoT Embedded Firmware (Fall 2021)
 *
 *@instructor  David Sluiter
 *
 *@assignment  final project
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
#include "ble.h"
#include "app_assert.h"

// Include logging for this file
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

#include "autogen/gatt_db.h"

#include "ble_device_type.h"

ble_data_struct_t ble_data;

static uint8_t advertising_set_handle = 0xff;
uint8_t connection_handle = 0xff;
/**Get private ble data*/
ble_data_struct_t *get_ble_data(){
  return (&ble_data);
}
/**Get private connection handle*/
uint8_t *get_connection_handle(){
  return (&connection_handle);
}

/**private ble data initialization*/
void ble_data_init(){
  ble_data_struct_t *ble_data_loc = get_ble_data();
  ble_data_loc->htm_indication_enable = false;
  ble_data_loc->connection_enable = false;

  ble_data_loc->indication_in_flight = false;
  ble_data_loc->motion_indication_enable = false;
  ble_data_loc->light_indication_enable = false;
  ble_data_loc->car_lot_indication_enable = false;

  ble_data_loc->smart_garage_bonded = false;
  ble_data_loc->smart_garage_confirmation_require = false;
}

void server_char_status_flag_set(sl_bt_msg_t *evt, uint16_t chardb,bool *indication_enable, bool *indication_in_flight){
  if((evt->data.evt_gatt_server_characteristic_status.characteristic ==chardb)){
    if(evt->data.evt_gatt_server_characteristic_status.status_flags ==sl_bt_gatt_server_client_config){            //Configuration changed
      if(evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_server_indication) {     //Indication enable
         *indication_enable = true;
      } else if(evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_server_disable) { // Indication disable
         *indication_enable = false;
      }
    }else if(evt->data.evt_gatt_server_characteristic_status.status_flags ==sl_bt_gatt_server_confirmation){       //confirmation from client
         *indication_in_flight = false;
    }
  }
}

/**
 * Handle ble event based on the ble stack signal
 * @param[in] ble stack signal
 * */
void handle_ble_event(sl_bt_msg_t *evt){
  ble_data_struct_t *ble_data_loc = get_ble_data();
  sl_status_t sc;
  bd_addr addr;
  uint8_t address_type;
  switch(SL_BT_MSG_ID(evt->header)){
    //Events common to both servers and clients

    case sl_bt_evt_system_boot_id:
        ble_data_init();
        displayInit();
        sc = sl_bt_advertiser_create_set(&advertising_set_handle);
        if(sc!=SL_STATUS_OK){
          LOG_ERROR("Advertiser handle create failed %d", sc);
        }


        sc = sl_bt_advertiser_set_timing( advertising_set_handle,
                                          MIN_ADV_INTERVAL,
                                          MAX_ADV_INTERVAL,
                                          ADV_DURATION,
                                          MAX_ADV_EVENT);
        if(sc!=SL_STATUS_OK){
          LOG_ERROR("Advertiser set timing failed %d", sc);
        }


        sc = sl_bt_advertiser_start(advertising_set_handle,
                                    sl_bt_advertiser_general_discoverable,
                                    sl_bt_advertiser_connectable_scannable);
        if(sc!=SL_STATUS_OK){
          LOG_ERROR("Advertiser start failed %d", sc);
        }

        sc = sl_bt_sm_delete_bondings();
        if(sc!=SL_STATUS_OK){
          LOG_ERROR("delete bondings failed %d", sc);
        }

        sc = sl_bt_sm_configure(SM_CONFIG_FLAG,sl_bt_sm_io_capability_displayyesno);
        if(sc!=SL_STATUS_OK){
          LOG_ERROR("security configure failed %d", sc);
        }

        sc = sl_bt_system_get_identity_address(&addr, &address_type);
        if(sc!=SL_STATUS_OK){
          LOG_ERROR("Get identity address failed %d", sc);
        }
        displayPrintf(DISPLAY_ROW_NAME, "Server");
        displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising");
        displayPrintf(DISPLAY_ROW_ASSIGNMENT, "Final Project");
        displayPrintf(DISPLAY_ROW_BTADDR, "%02X:%02X:%02X:%02X:%02X:%02X",addr.addr[0],
                      addr.addr[1],addr.addr[2],addr.addr[3],addr.addr[4], addr.addr[5]);
      break;

      /**Call when client connected**/
    case sl_bt_evt_connection_opened_id:
      ble_data_loc->connection_enable = true;
      connection_handle = evt->data.evt_connection_opened.connection;
      sc = sl_bt_connection_set_parameters( connection_handle,
                                            MIN_CNT_INTERVAL,
                                            MAX_CNT_INVERVAL,
                                            SLAVE_LATENCY,
                                            SUPERVISON_TIMEOUT,
                                            MIN_CNT_EVT_LENGTH,
                                            MAX_CNT_EVT_LENGTH);
      if(sc!=SL_STATUS_OK){
        LOG_ERROR("connection set parameters failed %d", sc);
      }

      sc = sl_bt_advertiser_stop(advertising_set_handle);

      if(sc!=SL_STATUS_OK){
        LOG_ERROR("advertiser stop failed %d", sc);
      }

      displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");


      break;

      /**Call when a connection closed**/
    case sl_bt_evt_connection_closed_id:
      ble_data_init();
      ble_data_loc->connection_enable = false;
      sc = sl_bt_sm_delete_bondings();
      if(sc!=SL_STATUS_OK){
        LOG_ERROR("delete bondings failed %d", sc);
      }

      sc = sl_bt_advertiser_start(
             advertising_set_handle,
             sl_bt_advertiser_general_discoverable,
             sl_bt_advertiser_connectable_scannable);
      if(sc!=SL_STATUS_OK){
        LOG_ERROR("connection enable failed %d", sc);
      }
      displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising");
      displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
      PbCirQ_init();

      // run vl53l0x extra time
      sl_bt_system_set_soft_timer(32768, 2, true); // Deprecated

      break;


    /**Call when parameters are set**/
    case sl_bt_evt_connection_parameters_id:
#if (PRINT_PARAMS) // DOS: compiler error in formatting these parameters, included with ( ) and typecast to int
      LOG_INFO("interval=%d, latency=%d, timeout=%d",
               (int) (evt->data.evt_connection_parameters.interval*1.25),
               (int) (evt->data.evt_connection_parameters.latency),
               (int) (evt->data.evt_connection_parameters.timeout) );
#endif
      break;

      //call when user push the read characteristic
     case sl_bt_evt_sm_confirm_bonding_id:
        if(evt->data.evt_sm_confirm_bonding.connection==connection_handle){
          sc = sl_bt_sm_bonding_confirm(connection_handle,1);
        }else{
          sc = sl_bt_sm_bonding_confirm(connection_handle,0);
        }

        if(sc!=SL_STATUS_OK){
          LOG_ERROR("bonding confirm failed %d", sc);
        }
        break;

    case sl_bt_evt_sm_confirm_passkey_id:
       displayPrintf(DISPLAY_ROW_PASSKEY,"%d",evt->data.evt_sm_confirm_passkey.passkey);
       displayPrintf(DISPLAY_ROW_ACTION,"Confirm with PB0");
       if(evt->data.evt_sm_confirm_bonding.connection==connection_handle &&
           evt->data.evt_sm_confirm_bonding.bonding_handle!=SL_BT_INVALID_BONDING_HANDLE){
           ble_data_loc->smart_garage_confirmation_require = true;
       }
       break;

    case sl_bt_evt_sm_bonded_id:
       displayPrintf(DISPLAY_ROW_CONNECTION, "Bonded");
       displayPrintf(DISPLAY_ROW_PASSKEY,"");
       displayPrintf(DISPLAY_ROW_ACTION,"");
       if(evt->data.evt_sm_bonded.connection==connection_handle&&
          evt->data.evt_sm_confirm_bonding.bonding_handle!=SL_BT_INVALID_BONDING_HANDLE){
         ble_data_loc->smart_garage_bonded  = true;
       }

       // start 1 sec period timer for distence sensor
       sl_status_t timer_response = sl_bt_system_set_soft_timer(32768, 2, false); // Deprecated

       if (timer_response != SL_STATUS_OK)
       {
           LOG_ERROR("LEDC - sl_bt_system_set_soft_timer");
       }

       break;

     case  sl_bt_evt_sm_bonding_failed_id:
       ble_data_loc->smart_garage_bonded  = false;
       LOG_ERROR("Bonding Failed");
       break;


    case sl_bt_evt_system_external_signal_id:
      switch(evt->data.evt_system_external_signal.extsignals){
        case bt_ext_sig_pb0_pressed: {
          pushbutton0_response();
          break;
        }
        case bt_ext_sig_ridar_result_ready: {
          uint16_t dist_mm = vl_get_result();
          // disable ridar state machine, wait for next result
          vl_set_flag_enable(false);
          vl_set_flag_data_ready(false);

          if (dist_mm > 200) {
              // car is not in the position
              carlot_to_client_indication(false);
          }
          else {
              carlot_to_client_indication(true);
          }
          LOG_INFO("distance = %d mm \r\n", dist_mm);
          break;
        }

        default:
          break;
      }
      break;

      /**Call when change of CCCD or indication confirmation received by client**/
    case sl_bt_evt_gatt_server_characteristic_status_id:
      server_char_status_flag_set(evt, gattdb_motion_state,&(ble_data_loc->light_indication_enable), &(ble_data_loc->indication_in_flight));
      server_char_status_flag_set(evt, gattdb_light_state, &(ble_data_loc->motion_indication_enable), &(ble_data_loc->indication_in_flight));
      server_char_status_flag_set(evt, gattdb_carLotState, &(ble_data_loc->car_lot_indication_enable), &(ble_data_loc->indication_in_flight));
      break;

    case sl_bt_evt_system_soft_timer_id:
        displayUpdate();
        soft_timer_deq_indication();

        if (evt->data.evt_system_soft_timer.handle == 2) {
            if (!vl_get_flag_enable()) {
                vl_set_flag_enable(true);
            }
        }

        break;

      /**Should never reach here**/
    case sl_bt_evt_gatt_server_indication_timeout_id:
      LOG_ERROR("Client did not respond");
      break;
  }
}
