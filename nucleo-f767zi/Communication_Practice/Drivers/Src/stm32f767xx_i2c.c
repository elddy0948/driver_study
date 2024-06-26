/*
 * stm32f767xx_i2c.c
 *
 *  Created on: May 29, 2024
 *      Author: HoJoon
 */

#include "stm32f767xx_i2c.h"

uint16_t ahb_prescalers[8] = { 2, 4, 8, 16, 64, 128, 256, 512 };
uint8_t apb1_prescalers[4] = { 2, 4, 8, 16 };

static void I2C_Execute_address_phase(I2C_RegDef_t *pI2Cx, uint8_t slave_address)
{
	uint32_t temp = 0;

	slave_address = slave_address << 1;

	temp |= (slave_address << I2C_CR2_SADD);
	temp &= ~(1 << I2C_CR2_RDWRN);

	pI2Cx->CR2 |= temp;
}

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

	I2C_Peripheral_clock_control(pI2CHandle->pI2Cx, ENABLE);

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

void I2C_DeInitialize(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1) {
		I2C1_REG_RESET();
	} else if (pI2Cx == I2C2) {
		I2C2_REG_RESET();
	} else if (pI2Cx == I2C3) {
		I2C3_REG_RESET();
	} else if (pI2Cx == I2C4) {
		I2C4_REG_RESET();
	}
}

uint8_t I2C_Get_flag_status(I2C_RegDef_t *pI2Cx, uint32_t flag_name)
{
	if (pI2Cx->ISR & flag_name) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_Master_send_data(I2C_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slave_address)
{
	pHandle->pI2Cx->CR2 |= (1 << I2C_CR2_START);

	while (!I2C_Get_flag_status(pHandle->pI2Cx, I2C_FLAG_BUSY));

	I2C_Execute_address_phase(pHandle->pI2Cx, slave_address);

	while (!I2C_Get_flag_status(pHandle->pI2Cx, I2C_FLAG_ADDR));

	while (length > 0) {
		while (!I2C_Get_flag_status(pHandle->pI2Cx, I2C_FLAG_TXE));
		pHandle->pI2Cx->TXDR = *pTxBuffer;
		pTxBuffer++;
		length--;
	}

	while (!I2C_Get_flag_status(pHandle->pI2Cx, I2C_FLAG_TXE));

	pHandle->pI2Cx->CR2 |= (1 << I2C_CR2_STOP);
}

/**
 * I2C Master receiver for N <= 255 bytes
 * Reference Manual p.1203 Figure 369
 */
void I2C_Master_receive_data(I2C_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t slave_address)
{
	uint32_t temp = 0;

	temp |= (length << I2C_CR2_NBYTES);
//	temp |= (1 << I2C_CR2_AUTOEND);
	temp |= (slave_address << I2C_CR2_SADD);
	temp |= (1 << I2C_CR2_RDWRN);

	pHandle->pI2Cx->CR2 = temp;

	pHandle->pI2Cx->CR2 |= (1 << I2C_CR2_START);

	while (length != 0) {
		while (!(pHandle->pI2Cx->ISR & (1 << I2C_ISR_RXNE)));
		*(pRxBuffer) = pHandle->pI2Cx->RXDR;
		length--;
	}

	while (!(pHandle->pI2Cx->ISR & (1 << I2C_ISR_TC)));

	return;
}
