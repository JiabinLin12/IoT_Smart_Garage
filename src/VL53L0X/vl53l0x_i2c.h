#ifndef __VL53L0_I2C_H
#define __VL53L0_I2C_H


#include "app.h"

#include "em_cmu.h"
#include "sl_i2cspm.h"

#include "vl53l0x_types.h"


#define VL53L0X_Addr    0x29
#define APDS9960_addr   0x39

#define STATUS_OK       0x00
#define STATUS_FAIL     0x01


bool vl_writeReg(uint8_t reg, uint8_t value);
uint8_t vl_readReg(uint8_t reg);

bool vl_writeReg16Bit(uint8_t reg, uint16_t value);
bool vl_writeReg32Bit(uint8_t reg, uint32_t value);

uint16_t vl_readReg16Bit(uint8_t reg);
uint32_t vl_readReg32Bit(uint8_t reg);

void writeMulti(uint8_t reg, uint8_t const * src, uint8_t count);
void readMulti(uint8_t reg, uint8_t * dst, uint8_t count);

uint8_t VL_Write_nByte(uint8_t REG_Address,uint16_t len, uint8_t *buf);
uint8_t VL_Read_nByte(uint8_t REG_Address,uint16_t len,uint8_t *buf);

#endif 


