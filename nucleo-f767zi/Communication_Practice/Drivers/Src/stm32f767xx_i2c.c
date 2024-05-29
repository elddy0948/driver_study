/*
 * stm32f767xx_i2c.c
 *
 *  Created on: May 29, 2024
 *      Author: HoJoon
 */

#include "stm32f767xx_i2c.h"

void I2C_Peripheral_clock_control(I2C_RegDef_t* pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		} else if (pI2Cx == I2C4) {
			I2C4_PCLK_EN();
		}
	} else {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		} else if (pI2Cx == I2C4) {
			I2C4_PCLK_DI();
		}
	}
}

void I2C_Peripheral_control(I2C_RegDef_t* pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}
