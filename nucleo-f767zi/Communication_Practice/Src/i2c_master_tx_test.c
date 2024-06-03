/*
 * i2c_master_tx_test.c
 *
 *  Created on: Jun 4, 2024
 *      Author: HoJoon
 */

#include <string.h>
#include "stm32f767xx.h"

#define SLAVE_ADDR		0x68

I2C_Handle_t i2c1_handle;

uint8_t dummy_data[] = "We are testing I2C master Tx\n";

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

void I2C1_initialize(void)
{
	i2c1_handle.pI2Cx = I2C1;
	i2c1_handle.I2CConfig.ACKControl = I2C_ACK_ENABLE;
	i2c1_handle.I2CConfig.DeviceAddress = 0x61;
	i2c1_handle.I2CConfig.SCLSpeed = I2C_SCL_SPEED_STANDARD;
	I2C_Initialize(&i2c1_handle);
}

int main(void)
{
	I2C1_GPIO_initialize();
	I2C1_initialize();

	I2C_Peripheral_control(I2C1, ENABLE);

	while (1) {
		// Some button or delay

		I2C_Master_send_data(&i2c1_handle, dummy_data, strlen((char*)dummy_data), SLAVE_ADDR);
	}

	return 0;
}
