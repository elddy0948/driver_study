#include <string.h>
#include "stm32f767xx.h"

/*
 * SPI command helper macros
 * ----------------------------------------------
 */
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON		1
#define LED_OFF		0

#define ARDUINO_ANALOG_PIN0		0
#define ARDUINO_ANALOG_PIN1		1
#define ARDUINO_ANALOG_PIN2		2
#define ARDUINO_ANALOG_PIN3		3
#define ARDUINO_ANALOG_PIN4		4
#define ARDUINO_ANALOG_PIN5		5

#define ARDUINO_LED_PIN			9
/*
 * --------------------------------------------
 */

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

uint8_t SPI_VerifyResponse(uint8_t response)
{
	if (response == 0xF5) {
		return 1;
	}

	return 0;
}

int main(void)
{
	uint8_t dummy_data = 0xFF;
	uint8_t dummy_read = 0;

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

		// Send command
		uint8_t command_code = COMMAND_LED_CTRL;
		uint8_t response = 0;
		uint8_t args[2] = {0, 0};

		SPI_SendData(SPI2, &command_code, 1);
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// Send dummy bits(1 byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_data, 1);
		SPI_ReceiveData(SPI2, &response, 1);

		if (SPI_VerifyResponse(response)) {
			args[0] = ARDUINO_LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);
		}

		// Command Sensor read
		while (!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUMBER_7));
		// Delay()
		command_code = COMMAND_SENSOR_READ;
		SPI_SendData(SPI2, &command_code, 1);
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		SPI_SendData(SPI2, &dummy_data, 1);
		SPI_ReceiveData(SPI2, &response, 1);

		uint8_t analog_read = 0;
		if (SPI_VerifyResponse(response)) {
			args[0] = ARDUINO_ANALOG_PIN0;
			SPI_SendData(SPI2, args, 1);

			SPI_ReceiveData(SPI2, &dummy_read, 1);

			// insert some delay
			SPI_SendData(SPI2, &dummy_data, 1);
			SPI_ReceiveData(SPI2, &analog_read, 1);
		}

		// Check SPI is not in BUSY
		SPI_GetFlagStatus(SPI2, SPI_BUSY_FALG);

		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
