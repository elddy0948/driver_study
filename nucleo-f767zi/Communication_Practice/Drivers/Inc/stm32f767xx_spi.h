/*
 * stm32f767xx_spi.h
 *
 *  Created on: May 2, 2024
 *      Author: HoJoon
 */

#ifndef INC_STM32F767XX_SPI_H_
#define INC_STM32F767XX_SPI_H_

#include "stm32f767xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode;		/* @SPI_DEVICE_MODE */
	uint8_t SPI_BusConfig;		/* @SPI_BUS_CONFIG */
	uint8_t SPI_SclkSpeed;		/* @SPI_CLK_SPEED */
	uint8_t SPI_CRCL;			/* @SPI_CRCL */
	uint8_t SPI_CPOL;			/* @SPI_CPOL */
	uint8_t SPI_CPHA;			/* @SPI_CPHA */
	uint8_t SPI_SSM;			/* @SPI_SSM */
} SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
} SPI_Handle_t;

/* @SPI_DEVICE_MODE */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/* @SPI_BUS_CONFIG */
#define SPI_BUS_FULL_DUPLEX		1
#define SPI_BUS_HALF_DUPLEX		2
#define SPI_BUS_SIMPLEX_RX		3

/* @SPI_CLK_SPEED */
#define SPI_PCLK_SPEED_DIV2		0
#define SPI_PCLK_SPEED_DIV4		1
#define SPI_PCLK_SPEED_DIV8		2
#define SPI_PCLK_SPEED_DIV16	3
#define SPI_PCLK_SPEED_DIV32	4
#define SPI_PCLK_SPEED_DIV64	5
#define SPI_PCLK_SPEED_DIV128	6
#define SPI_PCLK_SPEED_DIV256	7

/* @SPI_CRCL */
#define SPI_CRCL_8BITS		0
#define SPI_CRCL_16BITS		1

/* @SPI_CPOL */
#define SPI_CPOL_HIGH		1
#define SPI_CPOL_LOW		0

/* @SPI_CPHA */
#define SPI_CPHA_HIGH		1
#define SPI_CPHA_LOW		0

/* @SPI_SSM */
#define SPI_SSM_EN			1
#define SPI_SSM_DI			0

#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_BUSY_FALG		(1 << SPI_SR_BSY)

/* Peripheral Clock setup */
void SPI_PeripheralClockControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi);

/* Init and De-init */
void SPI_Init(SPI_Handle_t* pSPIHandle);
void SPI_DeInit(SPI_RegDef_t* pSPIx);

/* Data send and receive */
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t length);
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t length);

/* IRQ Configuration and ISR handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t* pHandle);

void SPI_PeripheralControl(SPI_RegDef_t* pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t* pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t* pSPIx, uint8_t EnOrDi);

#endif /* INC_STM32F767XX_SPI_H_ */
