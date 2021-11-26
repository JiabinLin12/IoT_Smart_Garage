/*
 * i2c.h
 *
 *  Created on: Sep 15, 2021
 *      Author: han16
 */

#ifndef SRC_I2C_H_
#define SRC_I2C_H_


#include "app.h"
#include "gpio.h"
#include "timers.h"
#include "em_cmu.h"
#include "sl_i2cspm.h"

#define I2C0_SCL_LOC      14
#define I2C0_SDA_LOC      16

#define SI7021_ADDR       0x40
#define CMD_READ_HUMD     0
#define CMD_MEASURE_TEMP  0xF3


typedef struct {
  uint16_t len;
  uint8_t data[2];
} i2c_msg_t;


typedef struct si7021_data_s {
  float temperature;
}si7021_data_t;

/**
 * @brief
 *  Initialize I2C
 */
void i2cInit(void);

/**
 * @brief
 *  I2C0 write sequence
 */
I2C_TransferReturn_TypeDef i2c0Write_poll(uint8_t slave_addr,
                                     void * read_data,
                                     uint16_t read_len);

I2C_TransferReturn_TypeDef i2c0Read_poll(uint8_t slave_addr,
                                    void * write_data,
                                    uint16_t write_len);

I2C_TransferReturn_TypeDef i2c0Write_irq(uint8_t slave_addr,
                                    void * writedata,
                                    uint16_t data_len,
                                    I2C_TransferSeq_TypeDef * seq);

I2C_TransferReturn_TypeDef i2c0Read_irq(uint8_t slave_addr,
                                   void * readdata,
                                   uint16_t data_len,
                                   I2C_TransferSeq_TypeDef * seq);

#endif /* SRC_I2C_H_ */
