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
	uint32_t tempRegister = 0;

	SPI_PeripheralClockControl(pSPIHandle->pSPIx, ENABLE);

	tempRegister |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);	// Configure device mode

	// Bus configure
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_FULL_DUPLEX)
	{
		// BIDI Mode clear
		tempRegister &= ~(1 << SPI_CR1_BIDI_MODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_HALF_DUPLEX)
	{
		// BIDI Mode set
		tempRegister |= (1 << SPI_CR1_BIDI_MODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_SIMPLEX_RX)
	{
		// BIDI Mode clear, RXONLY set
		tempRegister &= ~(1 << SPI_CR1_BIDI_MODE);
		tempRegister |= (1 << SPI_CR1_RX_ONLY);
	}

	tempRegister |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_MSTR);	// Configure baud rate
	tempRegister |= (pSPIHandle->SPIConfig.SPI_CRCL << SPI_CR1_CRCL);
	tempRegister |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);
	tempRegister |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	pSPIHandle->pSPIx->CR1 = tempRegister;
}

void SPI_DeInit(SPI_RegDef_t* pSPIx)
{

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t FlagName)
{
	if (pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/* This is polling based code ( blocking API call ) */
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t length)
{
	while (length > 0)
	{
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		if (pSPIx->CR1 & (1 << SPI_CR1_CRCL))
		{
			// 16 Bit
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			length -= 2;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// 8 Bit
			pSPIx->DR = *(pTxBuffer);
			length--;
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t length)
{
	while (length > 0) {
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		if ((pSPIx->CR1 & (1 << SPI_CR1_CRCL))) {
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			length -= 2;
			(uint16_t*)pRxBuffer++;
		} else {
			*(pRxBuffer) = pSPIx->DR;
			length--;
			pRxBuffer++;
		}
	}
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

void SPI_PeripheralControl(SPI_RegDef_t* pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t* pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t* pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}
