/*
 * spi_message_int.c
 *
 *  Created on: May 27, 2024
 *      Author: HoJoon
 */

#include <string.h>
#include "stm32f767xx.h"

#define MAX_LENGTH		500

/*
 * Global variables
 */
SPI_Handle_t spi2_handle;

char receive_buffer[MAX_LENGTH];
uint8_t read_byte = 0;
volatile uint8_t receive_stop = 0;
volatile uint8_t data_available = 0;

void Slave_GPIO_interrupt_init(void)
{
	GPIO_Handle_t spi_interrupt_pin;
	memset(&spi_interrupt_pin, 0, sizeof(spi_interrupt_pin));

	spi_interrupt_pin.pGPIOx = GPIOD;
	spi_interrupt_pin.PinConfig.PinNumber = GPIO_PIN_NUMBER_6;
	spi_interrupt_pin.PinConfig.PinMode = GPIO_MODE_IT_FALLING;
	spi_interrupt_pin.PinConfig.PinSpeed = GPIO_SPEED_LOW;
	spi_interrupt_pin.PinConfig.PinPuPdControl = GPIO_PUPD_NO_PUPD;

	GPIO_Init(&spi_interrupt_pin);

	GPIO_IRQPriorityConfig(IRQ_NUMBER_EXTI9_5, NVIC_IRQ_PRIORITY_15);
	GPIO_IRQInterruptConfig(IRQ_NUMBER_EXTI9_5, ENABLE);
}

void SPI2_GPIO_init(void)
{
	GPIO_Handle_t spi2_gpio;

	spi2_gpio.pGPIOx = GPIOB;
	spi2_gpio.PinConfig.PinMode = GPIO_MODE_AF;
	spi2_gpio.PinConfig.PinAltFunMode = 5;
	spi2_gpio.PinConfig.PinOPType = GPIO_OUTPUT_TYPE_PP;
	spi2_gpio.PinConfig.PinPuPdControl = GPIO_PUPD_PU;
	spi2_gpio.PinConfig.PinSpeed = GPIO_SPEED_FAST;

	spi2_gpio.PinConfig.PinNumber = GPIO_PIN_NUMBER_13;
	GPIO_Init(&spi2_gpio);
	spi2_gpio.PinConfig.PinNumber = GPIO_PIN_NUMBER_15;
	GPIO_Init(&spi2_gpio);
	spi2_gpio.PinConfig.PinNumber = GPIO_PIN_NUMBER_14;
	GPIO_Init(&spi2_gpio);
	spi2_gpio.PinConfig.PinNumber = GPIO_PIN_NUMBER_12;
	GPIO_Init(&spi2_gpio);
}

void SPI2_Init(void)
{
	spi2_handle.pSPIx = SPI2;
	spi2_handle.SPIConfig.SPI_BusConfig = SPI_BUS_FULL_DUPLEX;
	spi2_handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	spi2_handle.SPIConfig.SPI_SclkSpeed = SPI_PCLK_SPEED_DIV32;
	spi2_handle.SPIConfig.SPI_CRCL = SPI_CRCL_8BITS;
	spi2_handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	spi2_handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	spi2_handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&spi2_handle);
}

int main(void)
{
	uint8_t dummy = 0xFF;

	Slave_GPIO_interrupt_init();

	SPI2_GPIO_init();
	SPI2_Init();

	SPI_SSOEConfig(SPI2, ENABLE);
	SPI_IRQInterruptConfig(IRQ_NUMBER_SPI2, ENABLE);

	while(1) {
		receive_stop = 0;
		while(!data_available);

		GPIO_IRQInterruptConfig(IRQ_NUMBER_EXTI9_5, DISABLE);
		SPI_PeripheralControl(SPI2, ENABLE);

		while(!receive_stop) {
			while(SPI_SendData_IT(&spi2_handle, &dummy, 1) == SPI_BUSY_IN_TX);
			while(SPI_ReceiveData_IT(&spi2_handle, &read_byte, 1) == SPI_BUSY_IN_RX);
		}

		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		SPI_PeripheralControl(SPI2, DISABLE);

		data_available = 0;

		GPIO_IRQInterruptConfig(IRQ_NUMBER_EXTI9_5, ENABLE);
	}

	return 0;
}

void SPI2_IRQHandle(void)
{
	SPI_IRQHandling(&spi2_handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t* pHandle, uint8_t event)
{
	static uint32_t buffer_index = 0;
	if (event == SPI_EVENT_RX_COMPLETE) {
		receive_buffer[buffer_index++] = read_byte;
		if (read_byte == '\0' || (buffer_index == MAX_LENGTH)) {
			receive_stop = 0;
			receive_buffer[buffer_index - 1] = '\0';
			buffer_index = 0;
		}
	}
}

void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NUMBER_6);
	data_available = 1;
}
