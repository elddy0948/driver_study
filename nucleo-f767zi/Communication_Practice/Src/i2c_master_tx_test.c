/*
 * i2c_master_tx_test.c
 *
 *  Created on: Jun 4, 2024
 *      Author: HoJoon
 */

#include "stm32f767xx.h"

/**
 * ALTERNATE FUNCTION 4
 * I2C1
 * PB6 : I2C1_SCL
 * PB7 : I2C1_SDA
 */
void I2C1_GPIO_initialize(void)
{
	GPIO_Handle_t i2c_pins;

	i2c_pins.pGPIOx = GPIOB;
	i2c_pins.PinConfig.PinMode = GPIO_MODE_AF;
	i2c_pins.PinConfig.PinOPType = GPIO_OUTPUT_TYPE_OD;
	i2c_pins.PinConfig.PinPuPdControl = GPIO_PUPD_PU;
	i2c_pins.PinConfig.PinAltFunMode = 4;
	i2c_pins.PinConfig.PinSpeed = GPIO_SPEED_FAST;

	i2c_pins.PinConfig.PinNumber = GPIO_PIN_NUMBER_6;
	GPIO_Init(&i2c_pins);

	i2c_pins.PinConfig.PinNumber = GPIO_PIN_NUMBER_7;
	GPIO_Init(&i2c_pins);
}
