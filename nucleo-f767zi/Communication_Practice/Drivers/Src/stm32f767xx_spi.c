/*
 * stm32f767xx_spi.c
 *
 *  Created on: May 2, 2024
 *      Author: HoJoon
 */

#include "stm32f767xx_spi.h"

void SPI_PeripheralClockControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1) { SPI1_PCLK_EN(); }
		else if (pSPIx == SPI2) { SPI2_PCLK_EN(); }
		else if (pSPIx == SPI3) { SPI3_PCLK_EN(); }
		else if (pSPIx == SPI4) { SPI4_PCLK_EN(); }
	}
	else
	{
		if (pSPIx == SPI1) { SPI1_PCLK_DI(); }
		else if (pSPIx == SPI2) { SPI2_PCLK_DI(); }
		else if (pSPIx == SPI3) { SPI3_PCLK_DI(); }
		else if (pSPIx == SPI4) { SPI4_PCLK_DI(); }
	}
}

void SPI_Init(SPI_Handle_t* pSPIHandle)
{

}

void SPI_DeInit(SPI_RegDef_t* pSPIx)
{

}

void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t length)
{

}

void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t length)
{

}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

void SPI_IRQHandling(SPI_Handle_t* pHandle)
{

}
