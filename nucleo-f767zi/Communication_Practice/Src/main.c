
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

int main(void)
{
	SPI_GPIOInit();
	return 0;
}
