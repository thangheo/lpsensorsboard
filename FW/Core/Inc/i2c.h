#ifndef __I2C_H
#define __I2C_H
#include "stm32l0xx_hal.h"

void i2c_init(void);
void i2c_send(uint16_t dev_addr,uint8_t* data,uint16_t len);
void i2c_rx(uint16_t dev_addr,uint8_t* data,uint16_t len);

#endif
