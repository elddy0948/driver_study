/*
 * stm32f767xx_i2c.c
 *
 *  Created on: May 29, 2024
 *      Author: HoJoon
 */

#include "stm32f767xx_i2c.h"

uint16_t ahb_prescalers[8] = { 2, 4, 8, 16, 64, 128, 256, 512 };
uint8_t apb1_prescalers[4] = { 2, 4, 8, 16 };

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

uint32_t RCC_Get_PLL_output_clock()
{
	return 0;
}

uint32_t RCC_Get_PCLK1_value(void)
{
	uint32_t pclk1 = 0, system_clock = 0;
	uint8_t clk_src = 0, ahb_prescaler = 0, temp = 0, apb1_prescaler = 0;

	clk_src = ((RCC->CFGR >> 2) & 0x3);

	/*
	 * SWS Register
	 * 00 : HSI
	 * 01 : HSE
	 * 10 : PLL
	 */
	if (clk_src == 0) {
		system_clock = 16000000;
	} else if (clk_src == 1) {
		system_clock = 4260000;
	} else if (clk_src == 2) {
		system_clock = RCC_Get_PLL_output_clock();
	}

	temp = ((RCC->CFGR >> 4) & 0xF);

	if (temp < 8) {
		ahb_prescaler = 1;
	} else {
		ahb_prescaler = ahb_prescalers[temp - 8];
	}

	temp = ((RCC->CFGR >> 10) & 0x7);

	if (temp < 4) {
		apb1_prescaler = 1;
	} else {
		apb1_prescaler = apb1_prescalers[temp - 4];
	}

	pclk1 = (system_clock / ahb_prescaler) / apb1_prescaler;

	return pclk1;
}

void I2C_Initialize(I2C_Handle_t* pI2CHandle)
{
	uint32_t temp = 0;

	if (pI2CHandle->I2CConfig.SCLSpeed == I2C_SCL_SPEED_STANDARD) {
		temp |= (I2C_STANDARD_MODE_16MHZ_PRESC << I2C_TIMINGR_PRESC);
		temp |= (I2C_STANDARD_MODE_16MHZ_SCLH << I2C_TIMINGR_SCLH);
		temp |= (I2C_STANDARD_MODE_16MHZ_SCLL << I2C_TIMINGR_SCLL);
		temp |= (I2C_STANDARD_MODE_16MHZ_SDADEL << I2C_TIMINGR_SDADEL);
		temp |= (I2C_STANDARD_MODE_16MHZ_SCLDEL << I2C_TIMINGR_SCLDEL);
	} else if (pI2CHandle->I2CConfig.SCLSpeed == I2C_SCL_SPEED_FAST) {
		temp |= (I2C_FAST_MODE_16MHZ_PRESC << I2C_TIMINGR_PRESC);
		temp |= (I2C_FAST_MODE_16MHZ_SCLH << I2C_TIMINGR_SCLH);
		temp |= (I2C_FAST_MODE_16MHZ_SCLL << I2C_TIMINGR_SCLL);
		temp |= (I2C_FAST_MODE_16MHZ_SDADEL << I2C_TIMINGR_SDADEL);
		temp |= (I2C_FAST_MODE_16MHZ_SCLDEL << I2C_TIMINGR_SCLDEL);
	} else if (pI2CHandle->I2CConfig.SCLSpeed == I2C_SCL_SPEED_FAST_PLUS) {
		temp |= (I2C_FAST_MODE_PLUS_16MHZ_PRESC << I2C_TIMINGR_PRESC);
		temp |= (I2C_FAST_MODE_PLUS_16MHZ_SCLH << I2C_TIMINGR_SCLH);
		temp |= (I2C_FAST_MODE_PLUS_16MHZ_SCLL << I2C_TIMINGR_SCLL);
		temp |= (I2C_FAST_MODE_PLUS_16MHZ_SDADEL << I2C_TIMINGR_SDADEL);
		temp |= (I2C_FAST_MODE_PLUS_16MHZ_SCLDEL << I2C_TIMINGR_SCLDEL);
	}

	pI2CHandle->pI2Cx->TIMINGR = temp;

	temp = 0;

	temp |= pI2CHandle->I2CConfig.DeviceAddress;
	pI2CHandle->pI2Cx->OAR1 |= (temp << 1);
}

void I2C_Peripheral_control(I2C_RegDef_t* pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}
