
#include "stm32f767xx.h"

/*
 * Configure GPIO pins with alternate function for SPI2
 * AF5
 * PB12 --> NSS
 * PB13 --> SCLK
 * PB14 --> MISO
 * PB15 --> MOSI
 */
void SPI_GPIOInit(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;

	SPIPins.PinConfig.PinMode = GPIO_MODE_AF;
	SPIPins.PinConfig.PinAltFunMode = 5;
	SPIPins.PinConfig.PinOPType = GPIO_OUTPUT_TYPE_PP;
	SPIPins.PinConfig.PinPuPdControl = GPIO_PUPD_NO_PUPD;
	SPIPins.PinConfig.PinSpeed = GPIO_SPEED_FAST;


	// NSS
	SPIPins.PinConfig.PinNumber = 12;
	GPIO_Init(&SPIPins);

	// SCLK
	SPIPins.PinConfig.PinNumber = 13;
	GPIO_Init(&SPIPins);

	// MISO
	SPIPins.PinConfig.PinNumber = 14;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.PinConfig.PinNumber = 15;
	GPIO_Init(&SPIPins);
}

void SPI2_Init(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;

	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_FULL_DUPLEX;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_PCLK_SPEED_DIV2;
	SPI2Handle.SPIConfig.SPI_CRCL = SPI_CRCL_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2Handle);
}

int main(void)
{
	SPI_GPIOInit();
	SPI2_Init();
	return 0;
}
