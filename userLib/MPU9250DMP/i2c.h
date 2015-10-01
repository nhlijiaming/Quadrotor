/*
 * i2c.h
 *
 *  Created on: 2014Äê7ÔÂ24ÈÕ
 *      Author: Ljm
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"

void Delay(unsigned long num_ms);

int stm32l_get_clock_ms(unsigned long *count);

void SPI_Init();

uint8_t Sensors_I2C_WriteRegister_swap(uint8_t saddr, uint8_t addr, uint8_t len, uint8_t *data);

uint8_t Sensors_I2C_ReadRegister_swap(uint8_t saddr, uint8_t addr, uint8_t len, uint8_t *data);


#endif /* I2C_H_ */
