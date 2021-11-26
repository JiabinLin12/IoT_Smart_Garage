/*
 * i2c.c
 *
 *  Created on: Sep 15, 2021
 *      Author: han16
 */

#include "i2c.h"
#include "log.h"

#include <stdint.h>

si7021_data_t si7021_data = {0};
I2C_TransferSeq_TypeDef seq_irq = {0};

/**
 * @brief
 *  Initialize I2C0
 */
void i2cInit(void)
{
  I2CSPM_Init_TypeDef i2c_config = {
      .port             = I2C0,
      .sclPort          = I2C0_PORT,
      .sclPin           = I2C0_SCL_PIN,
      .sdaPort          = I2C0_PORT,
      .sdaPin           = I2C0_SDA_PIN,
      .portLocationScl  = I2C0_SCL_LOC,
      .portLocationSda  = I2C0_SDA_LOC,
      .i2cRefFreq       = 0,
      .i2cMaxFreq       = I2C_FREQ_STANDARD_MAX,
      .i2cClhr          = i2cClockHLRStandard
  };

  I2CSPM_Init(&i2c_config);
}

/**
 * @brief
 *  I2C0 write sequence
 */
I2C_TransferReturn_TypeDef i2c0Write_poll(uint8_t slave_addr,
                                     void * read_data,
                                     uint16_t read_len)
{
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;

  seq.addr = slave_addr << 1;
  seq.flags = I2C_FLAG_WRITE;
  seq.buf[0].data = (uint8_t *) read_data;
  seq.buf[0].len = (uint16_t) read_len;

  ret = I2CSPM_Transfer( I2C0, &seq );

  return ret;
}

/**
 * @brief
 *  I2C0 read sequence
 */
I2C_TransferReturn_TypeDef i2c0Read_poll(uint8_t slave_addr,
                                    void * write_data,
                                    uint16_t write_len)
{
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;

  seq.addr = slave_addr << 1;
  seq.flags = I2C_FLAG_READ;
  seq.buf[0].data = (uint8_t *) write_data;
  seq.buf[0].len = (uint16_t) write_len;

  ret = I2CSPM_Transfer(I2C0, &seq);

  return ret;
}


/**
 * @brief
 *  I2C0 write sequence
 */
I2C_TransferReturn_TypeDef i2c0Write_irq(uint8_t slave_addr,
                                    void * writedata,
                                    uint16_t data_len,
                                    I2C_TransferSeq_TypeDef * seq)
{
  seq->addr = slave_addr << 1;
  seq->flags = I2C_FLAG_WRITE;
  seq->buf[0].data = (uint8_t *) writedata;
  seq->buf[0].len = (uint16_t) data_len;

  return I2C_TransferInit(I2C0, &seq_irq);
}

/**
 * @brief
 *  I2C read sequence
 */
I2C_TransferReturn_TypeDef i2c0Read_irq(uint8_t slave_addr,
                                   void * readdata,
                                   uint16_t data_len,
                                   I2C_TransferSeq_TypeDef * seq)
{
  seq->addr = slave_addr << 1;
  seq->flags = I2C_FLAG_READ;
  seq->buf[0].data = (uint8_t *) readdata;
  seq->buf[0].len = (uint16_t) data_len;

  return I2C_TransferInit(I2C0, &seq_irq);
}
