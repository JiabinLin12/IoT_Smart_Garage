#include "vl53l0x_i2c.h"
#include "i2c.h"

#include <string.h>

#define VL_SLAVE_ADRR 0x29

I2C_TransferSeq_TypeDef vl_i2c_seq = { 0 };
uint8_t vl_i2c_seq_data_buf[64] = { 0 };


uint8_t VL_Write_nByte(uint8_t REG_Address,uint16_t len, uint8_t *buf)
{
  // 1. Device addr + register addr + data
  I2C_TransferReturn_TypeDef ret;
  uint16_t i = 0;

  memset(&vl_i2c_seq_data_buf, 0, sizeof(vl_i2c_seq_data_buf));

  vl_i2c_seq_data_buf[0] = REG_Address;

  for(i=1; i<(len+1); i++)
  {
      vl_i2c_seq_data_buf[i] = *buf++;
  }

  ret = i2c0Write_poll(VL_SLAVE_ADRR, vl_i2c_seq_data_buf, len + 1);

  if (ret != i2cTransferDone)
  {
      return 1;
  }

  return 0;
}

uint8_t VL_Read_nByte(uint8_t REG_Address,uint16_t len,uint8_t *buf)
{
  uint8_t write_data[1] = {0};

  // 1. Device addr + register addr + data
  I2C_TransferReturn_TypeDef ret;

  ret = VL_Write_nByte(REG_Address, 0, write_data);
  if (ret != i2cTransferDone)
  {
      return 1;
  }

  memset(&vl_i2c_seq_data_buf, 0, sizeof(vl_i2c_seq_data_buf));

  ret = i2c0Read_poll(VL_SLAVE_ADRR, vl_i2c_seq_data_buf, len);

  if (ret != i2cTransferDone)
  {
      return 1;
  }

  memcpy(buf, vl_i2c_seq_data_buf, len);

  return 0;
}

bool vl_writeReg(uint8_t reg, uint8_t data)
{
  uint8_t buf = data;
  if (VL_Write_nByte(reg, 1, &buf) != i2cTransferDone)
    return false;

  return true;
}

bool vl_writeReg16Bit(uint8_t reg, uint16_t data)
{
  uint8_t buf[2] = {
      ((data >> 8) & 0xFF),
      (data & 0xFF)
  };

  if (VL_Write_nByte(reg, 2, buf) != i2cTransferDone)
    return false;

  return true;
}

uint8_t vl_readReg(uint8_t reg)
{
  uint8_t data = 0;

  if (VL_Read_nByte(reg, 1, &data) != i2cTransferDone)
    return 0;

  return data;
}

uint16_t vl_readReg16Bit(uint8_t reg)
{
  uint8_t data[2] = {0};
  uint16_t ret = 0;


  if (VL_Read_nByte(reg, 2, data) != i2cTransferDone)
    return 0;

  ret = *(data + 1) + (((*data) << 8) & 0xff00);

  return ret;
}
