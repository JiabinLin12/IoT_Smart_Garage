
#include "vl53_state_machine.h"
#include "ridar_test.h"
#include "app.h"

//extern void timerWaitms(uint32_t time_ms);

static uint8_t curr_state = 0;
static uint8_t next_state = 0;

extern uint8_t vl_stop_variable;

bool flag_vl_en = false;
bool flag_vl_measure_ready = false;
bool flag_vl_data_ready = false;

uint16_t vl_result = 0;

void ridar_fsm(sl_bt_msg_t *evt_struct) {

  if (evt_struct) {
      ;
  }

  curr_state = next_state;

  switch(curr_state) {
    case vlst_IDLE: {
      if (vl_get_flag_enable()) {
          /* This "enable" flag will be set in ble.c*/
          /* Single-read */
          vl_set_flag_data_ready(false);
          next_state = vlst_MeasureRequest;
      }
      break;
    }

    case vlst_MeasureRequest: {
      vl_set_flag_measure_ready(false);

      vl_writeReg(0x80, 0x01);
      vl_writeReg(0xFF, 0x01);
      vl_writeReg(0x00, 0x00);
      vl_writeReg(0x91, vl_stop_variable);
      vl_writeReg(0x00, 0x01);
      vl_writeReg(0xFF, 0x00);
      vl_writeReg(0x80, 0x00);
      vl_writeReg(SYSRANGE_START, 0x01);

      next_state = vlst_WaitForStartBit;
      break;
    }

    case vlst_WaitForStartBit: {
      if ((vl_readReg(SYSRANGE_START) & 0x01) == 0x00) {
          next_state = vlst_WaitForMeasure;
      }
//      timerWaitms_polled(5);
      delay_ms(5);

      break;
    }

    case vlst_WaitForMeasure: {
      if (vl_get_flag_measure_ready()) {
          vl_set_flag_measure_ready(false);
          next_state = vlst_MeasureCompleted;
      }
//      timerWaitms_polled(5);
      delay_ms(5);
      break;
    }

    case vlst_MeasureCompleted: {
      uint16_t range = vl_readReg16Bit(RESULT_RANGE_STATUS + 10);
      vl_writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

      vl_set_result(range);
      vl_set_flag_data_ready(true);
      sl_bt_external_signal(bt_ext_sig_ridar_result_ready);
      vl_set_flag_enable(false);

      next_state = vlst_IDLE;
      break;
    }

    case vlst_Error: {
      break;
    }

    default:
      break;
  }
}


void vl_set_flag_enable(bool is_enable) {
  flag_vl_en = is_enable;
}

bool vl_get_flag_enable() {
  return flag_vl_en;
}


void vl_set_flag_measure_ready(bool is_enable) {
  flag_vl_measure_ready = is_enable;
}

bool vl_get_flag_measure_ready() {
  return flag_vl_measure_ready;
}


void vl_set_flag_data_ready(bool is_enable) {
  flag_vl_data_ready = is_enable;
}

bool vl_get_flag_data_ready() {
  return flag_vl_data_ready;
}

void vl_set_result(uint16_t val) {
  vl_result = val;
}

uint16_t vl_get_result() {
  return vl_result;
}

