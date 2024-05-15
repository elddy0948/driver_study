#include <string.h>
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
	// SPIPins.PinConfig.PinNumber = 14;
	// GPIO_Init(&SPIPins);

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
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_PCLK_SPEED_DIV8;
	SPI2Handle.SPIConfig.SPI_CRCL = SPI_CRCL_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;	// Hardware Slave Management enabled for NSS pin

	SPI_Init(&SPI2Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t btn;

	btn.pGPIOx = GPIOC;
	btn.PinConfig.PinNumber = GPIO_PIN_NUMBER_7;
	btn.PinConfig.PinMode = GPIO_MODE_INPUT;
	btn.PinConfig.PinSpeed = GPIO_SPEED_FAST;
	btn.PinConfig.PinPuPdControl = GPIO_PUPD_NO_PUPD;

	GPIO_Init(&btn);
}

int main(void)
{
	char userData[] = "Hello World";

	SPI_GPIOInit();
	GPIO_ButtonInit();
	SPI2_Init();

	SPI_SSOEConfig(SPI2, ENABLE);
	SPI_SSIConfig(SPI2, DISABLE);

	for(;;)
	{
		while (!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUMBER_7));

		// Some Delay
		SPI_PeripheralControl(SPI2, ENABLE);

		SPI_SendData(SPI2, (uint8_t*)userData, strlen(userData));

		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
