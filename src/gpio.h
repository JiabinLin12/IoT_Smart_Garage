/*
   gpio.h
  
    Created on: Dec 12, 2018
        Author: Dan Walkes

    Updated by Dave Sluiter Sept 7, 2020. moved #defines from .c to .h file.
    Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_

#include <stdbool.h>
#include "em_gpio.h"
#include <string.h>


// Student Edit: Define these, 0's are placeholder values.
// See the radio board user guide at https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf
// and GPIO documentation at https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__GPIO.html
// to determine the correct values for these.

// Distance Sensor VL53L0X
#define VL53L0X_GPIO1_PORT          gpioPortD
#define VL53L0X_GPIO1_PIN           11           // Interrupt, EXP9

#define VL53L0X_XSHUT_PORT          gpioPortF
#define VL53L0X_XSHUT_PIN           3           // Power control, EXP13

// I2C0 Peripheral
#define I2C0_PORT                   gpioPortC
#define I2C0_SCL_PIN                10
#define I2C0_SDA_PIN                11

// Smart Light Macro Define
#define BREAK_BEAM_PORT             gpioPortD   //EXP_HEADER7
#define BREAK_BEAM_PIN              10
#define LIGHT_SENSOR_POWER_PORT     gpioPortD   //External Pin 11
#define LIGHT_SENSOR_POWER_PIN      12

// LCD display
#define DISP_PORT                   gpioPortD
#define DISP_PIN                    13

// Thermal
#define SI7021_PORT                 gpioPortD
#define SI7021_PIN                  15

// LEDs
#define	LED0_port                   gpioPortF
#define LED0_pin                    4
#define LED1_port                   gpioPortF
#define LED1_pin                    5

// Buttons
#define PB0_PORT                    gpioPortF
#define PB0_PIN                     6
#define PB1_PORT                    gpioPortF
#define PB1_PIN                     7


// Function prototypes
void gpioInit();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();
void si7021_enable();
void si7021_disable();
void gpioSensorEnSetOn();
void gpioSetDisplayExtcomin(bool status);
void light_sensor_enable();
void light_sensor_disable();
void gpioVL53SetOn();
void gpioVL53SetOff();

//DOS
void ToggleLED0 (void);


#endif /* SRC_GPIO_H_ */
