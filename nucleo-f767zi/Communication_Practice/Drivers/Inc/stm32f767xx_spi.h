/*
 * stm32f767xx_spi.h
 *
 *  Created on: May 2, 2024
 *      Author: HoJoon
 */

#ifndef INC_STM32F767XX_SPI_H_
#define INC_STM32F767XX_SPI_H_

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
} SPI_Handle_t;

#endif /* INC_STM32F767XX_SPI_H_ */
