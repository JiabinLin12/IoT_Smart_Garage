/***********************************************************************
 *@file        oscillators.c
 *
 *@version     0.0.1
 *
 *@brief       clock management unit setup function file.
 *
 *@author      Fanghao Han, faha9923@Colorado.edu
 *
 *@date        Nov 13, 2021
 *
 *@institution University of Colorado Boulder (UCB)
 *
 *@course      ECEN 5823-001: IoT Embedded Firmware (Fall 2021)
 *
 *@instructor  David Sluiter
 *
 *@assignment  final-project
 *
 *@resources   Utilized Silicon Labs' EMLIB peripheral libraries to
 *             implement functionality.
 *
 *
 *@copyright   All rights reserved. Distribution allowed only for the
 *             use of assignment grading. Use of code excerpts allowed at the
 *             discretion of author. Contact for permission.  */


#include "ridar_test.h"

#include "log.h"

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

uint8_t vl_stop_variable;
uint32_t measurement_timing_budget_us;




typedef struct SequenceStepEnables_t
    {
      bool tcc, msrc, dss, pre_range, final_range;
    }SequenceStepEnables;

typedef struct SequenceStepTimeouts_t
    {
      uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

      uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
      uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
    }SequenceStepTimeouts;


bool getSpadInfo(uint8_t * count, bool * type_is_aperture);

void getSequenceStepEnables(SequenceStepEnables * enables);
void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);

bool performSingleRefCalibration(uint8_t vhv_init_byte);

uint16_t decodeTimeout(uint16_t reg_val);
uint16_t encodeTimeout(uint16_t timeout_mclks);

static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);



uint8_t vl53l0x_init(void)
{
  vl_stop_variable = 0;

  // VL53L0X_DataInit() begin
  vl_writeReg(0x89, vl_readReg(0x89) | 0x01); // set bit 0

  // sensor uses 1V8 mode for I/O by default

  // "Set I2C standard mode"
  vl_writeReg(0x88, 0x00);

  vl_writeReg(0x80, 0x01);
  vl_writeReg(0xFF, 0x01);
  vl_writeReg(0x00, 0x00);
  vl_stop_variable = vl_readReg(0x91);//stop_var = 0x3c
  vl_writeReg(0x00, 0x01);
  vl_writeReg(0xFF, 0x00);
  vl_writeReg(0x80, 0x00);

  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  vl_writeReg(MSRC_CONFIG_CONTROL, vl_readReg(MSRC_CONFIG_CONTROL) | 0x12); // 0|0x12

  // set final range signal rate limit to 0.25 MCPS (million counts per second)
  setSignalRateLimit(0.25); // 0x00, 0x20

  vl_writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

  // VL53L0X_DataInit() end

  // VL53L0X_StaticInit() begin

  uint8_t spad_count;
  bool spad_type_is_aperture;
  if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  uint8_t ref_spad_map[6];
  //readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
  VL_Read_nByte(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6, ref_spad_map);
  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

  vl_writeReg(0xFF, 0x01);
  vl_writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  vl_writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  vl_writeReg(0xFF, 0x00);
  vl_writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++)
  {
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
    {
      spads_enabled++;
    }
  }

  //writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
  VL_Read_nByte(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6, ref_spad_map);
  // -- VL53L0X_set_reference_spads() end

  // -- VL53L0X_load_tuning_settings() begin
  // DefaultTuningSettings from vl53l0x_tuning.h

  vl_writeReg(0xFF, 0x01);
  vl_writeReg(0x00, 0x00);

  vl_writeReg(0xFF, 0x00);
  vl_writeReg(0x09, 0x00);
  vl_writeReg(0x10, 0x00);
  vl_writeReg(0x11, 0x00);

  vl_writeReg(0x24, 0x01);
  vl_writeReg(0x25, 0xFF);
  vl_writeReg(0x75, 0x00);

  vl_writeReg(0xFF, 0x01);
  vl_writeReg(0x4E, 0x2C);
  vl_writeReg(0x48, 0x00);
  vl_writeReg(0x30, 0x20);

  vl_writeReg(0xFF, 0x00);
  vl_writeReg(0x30, 0x09);
  vl_writeReg(0x54, 0x00);
  vl_writeReg(0x31, 0x04);
  vl_writeReg(0x32, 0x03);
  vl_writeReg(0x40, 0x83);
  vl_writeReg(0x46, 0x25);
  vl_writeReg(0x60, 0x00);
  vl_writeReg(0x27, 0x00);
  vl_writeReg(0x50, 0x06);
  vl_writeReg(0x51, 0x00);
  vl_writeReg(0x52, 0x96);
  vl_writeReg(0x56, 0x08);
  vl_writeReg(0x57, 0x30);
  vl_writeReg(0x61, 0x00);
  vl_writeReg(0x62, 0x00);
  vl_writeReg(0x64, 0x00);
  vl_writeReg(0x65, 0x00);
  vl_writeReg(0x66, 0xA0);

  vl_writeReg(0xFF, 0x01);
  vl_writeReg(0x22, 0x32);
  vl_writeReg(0x47, 0x14);
  vl_writeReg(0x49, 0xFF);
  vl_writeReg(0x4A, 0x00);

  vl_writeReg(0xFF, 0x00);
  vl_writeReg(0x7A, 0x0A);
  vl_writeReg(0x7B, 0x00);
  vl_writeReg(0x78, 0x21);

  vl_writeReg(0xFF, 0x01);
  vl_writeReg(0x23, 0x34);
  vl_writeReg(0x42, 0x00);
  vl_writeReg(0x44, 0xFF);
  vl_writeReg(0x45, 0x26);
  vl_writeReg(0x46, 0x05);
  vl_writeReg(0x40, 0x40);
  vl_writeReg(0x0E, 0x06);
  vl_writeReg(0x20, 0x1A);
  vl_writeReg(0x43, 0x40);

  vl_writeReg(0xFF, 0x00);
  vl_writeReg(0x34, 0x03);
  vl_writeReg(0x35, 0x44);

  vl_writeReg(0xFF, 0x01);
  vl_writeReg(0x31, 0x04);
  vl_writeReg(0x4B, 0x09);
  vl_writeReg(0x4C, 0x05);
  vl_writeReg(0x4D, 0x04);

  vl_writeReg(0xFF, 0x00);
  vl_writeReg(0x44, 0x00);
  vl_writeReg(0x45, 0x20);
  vl_writeReg(0x47, 0x08);
  vl_writeReg(0x48, 0x28);
  vl_writeReg(0x67, 0x00);
  vl_writeReg(0x70, 0x04);
  vl_writeReg(0x71, 0x01);
  vl_writeReg(0x72, 0xFE);
  vl_writeReg(0x76, 0x00);
  vl_writeReg(0x77, 0x00);

  vl_writeReg(0xFF, 0x01);
  vl_writeReg(0x0D, 0x01);

  vl_writeReg(0xFF, 0x00);
  vl_writeReg(0x80, 0x01);
  vl_writeReg(0x01, 0xF8);

  vl_writeReg(0xFF, 0x01);
  vl_writeReg(0x8E, 0x01);
  vl_writeReg(0x00, 0x01);
  vl_writeReg(0xFF, 0x00);
  vl_writeReg(0x80, 0x00);

  // -- VL53L0X_load_tuning_settings() end

  // "Set interrupt config to new sample ready"
  // -- VL53L0X_SetGpioConfig() begin

  vl_writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  vl_writeReg(GPIO_HV_MUX_ACTIVE_HIGH, vl_readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
  vl_writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  // -- VL53L0X_SetGpioConfig() end

  measurement_timing_budget_us = getMeasurementTimingBudget();

  // "Disable MSRC and TCC by default"
  // MSRC = Minimum Signal Rate Check
  // TCC = Target CentreCheck
  // -- VL53L0X_SetSequenceStepEnable() begin

  vl_writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // -- VL53L0X_SetSequenceStepEnable() end

  // "Recalculate timing budget"
  setMeasurementTimingBudget(measurement_timing_budget_us);

  // VL53L0X_StaticInit() end

  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

  // -- VL53L0X_perform_vhv_calibration() begin

  vl_writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!performSingleRefCalibration(0x40)) { return false; }

  // -- VL53L0X_perform_vhv_calibration() end

  // -- VL53L0X_perform_phase_calibration() begin

  vl_writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!performSingleRefCalibration(0x00)) { return false; }

  // -- VL53L0X_perform_phase_calibration() end

  // "restore the previous Sequence Config"
  vl_writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // VL53L0X_PerformRefCalibration() end

  // start 1 sec period timer
  sl_status_t timer_response = sl_bt_system_set_soft_timer(32768, 2, false); // Deprecated

  if (timer_response != SL_STATUS_OK)
  {
      LOG_ERROR("LEDC - sl_bt_system_set_soft_timer");
  }

  return true;
}

//VL53L0X�����Գ���
void vl53l0x_test(void)
{
  if(!vl53l0x_init())
   {
    LOG_INFO("VL53L0X Error!!!\r\n");
    delay_ms(500);
   }
   LOG_INFO("VL53L0X OK\r\n");

   while(1)
   {
       LOG_INFO("%d mm\n\r", readRangeSingleMillimeters());
       delay_ms(1000);

   }
}


bool setSignalRateLimit(float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  vl_writeReg16Bit(0x44, limit_Mcps * (1 << 7));
  return true;
}

bool getSpadInfo(uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;
  uint16_t timeout = 0x3333;

  vl_writeReg(0x80, 0x01);
  vl_writeReg(0xFF, 0x01);
  vl_writeReg(0x00, 0x00);

  vl_writeReg(0xFF, 0x06);
  vl_writeReg(0x83, vl_readReg(0x83) | 0x04);
  vl_writeReg(0xFF, 0x07);
  vl_writeReg(0x81, 0x01);

  vl_writeReg(0x80, 0x01);

  vl_writeReg(0x94, 0x6b);
  vl_writeReg(0x83, 0x00);

  while (timeout--)
  {
      if (vl_readReg(0x83) != 0x00)
        break;

      if (timeout == 1)
        return false;
  }
  vl_writeReg(0x83, 0x01);
  tmp = vl_readReg(0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  vl_writeReg(0x81, 0x00);
  vl_writeReg(0xFF, 0x06);
  vl_writeReg(0x83, vl_readReg( 0x83  & ~0x04));
  vl_writeReg(0xFF, 0x01);
  vl_writeReg(0x00, 0x01);

  vl_writeReg(0xFF, 0x00);
  vl_writeReg(0x80, 0x00);

  return true;
}

void getSequenceStepEnables(SequenceStepEnables * enables)
{
  uint8_t sequence_config = vl_readReg(SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

uint8_t getVcselPulsePeriod(vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  {
    return decodeVcselPeriod(vl_readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(vl_readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; }
}

void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = vl_readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
    timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    decodeTimeout(vl_readReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
    timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);

  timeouts->final_range_mclks =
    decodeTimeout(vl_readReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t encodeTimeout(uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

bool setMeasurementTimingBudget(uint32_t budget_us)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return false; }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return false;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    vl_writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return true;
}

uint32_t getMeasurementTimingBudget(void)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}


bool performSingleRefCalibration(uint8_t vhv_init_byte)
{
  uint16_t timeout = 0x3333;

  vl_writeReg(SYSRANGE_START, 0x01 | vhv_init_byte); // SYSRANGE_MODE_START_STOP

  while (timeout--)
 {
     if ((vl_readReg(RESULT_INTERRUPT_STATUS) & 0x07) != 0x00)
       break;

     if (timeout == 1)
       return false;
 }

  vl_writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  vl_writeReg(SYSRANGE_START, 0x00);

  return true;
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t readRangeContinuousMillimeters(void)
{
  uint16_t timeout = 0x3333;

  while (timeout--)
 {
     if ((vl_readReg(RESULT_INTERRUPT_STATUS) & 0x07) != 0x00)
       break;

     if (timeout == 1)
       return 65535;
 }
  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
  uint16_t range = vl_readReg16Bit(RESULT_RANGE_STATUS + 10);

  vl_writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  return range;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t readRangeSingleMillimeters(void)
{
  uint16_t timeout = 0x3333;

  vl_writeReg(0x80, 0x01);
  vl_writeReg(0xFF, 0x01);
  vl_writeReg(0x00, 0x00);
  vl_writeReg(0x91, vl_stop_variable);
  vl_writeReg(0x00, 0x01);
  vl_writeReg(0xFF, 0x00);
  vl_writeReg(0x80, 0x00);

  vl_writeReg(SYSRANGE_START, 0x01);

  // "Wait until start bit has been cleared"
  while (timeout--)
 {
     if ((vl_readReg(SYSRANGE_START) & 0x01) == 0x00)
       break;

     delay_ms(8);

     if (timeout == 1)
       return 65535;
 }

  return readRangeContinuousMillimeters();
}
