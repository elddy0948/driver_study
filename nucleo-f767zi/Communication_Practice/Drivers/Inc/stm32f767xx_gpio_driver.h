/*
 * stm32f767xx_gpio_driver.h
 *
 *  Created on: Mar 18, 2024
 *      Author: hojoon
 */

#ifndef DRIVERS_STM32F767XX_GPIO_DRIVER_H_
#define DRIVERS_STM32F767XX_GPIO_DRIVER_H_

#include "stm32f767xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;			// @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;			// @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			// @GPIO_SPEED
	uint8_t GPIO_PinPuPdControl;	// @GPIO_PULLUP_PULLDOWN
	uint8_t GPIO_PinOPType;			// @GPIO_OUTPUT_TYPE
	uint8_t GPIO_PinAltFunMode;		// @GPIO_ALTERNATE_FUNCTION
} GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t* pGPIOx;	// this holds the base address of the GPIO port
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;


// @GPIO_PIN_NUMBERS
#define GPIO_PIN_NUMBER_0	0
#define GPIO_PIN_NUMBER_1	1
#define GPIO_PIN_NUMBER_2	2
#define GPIO_PIN_NUMBER_3	3
#define GPIO_PIN_NUMBER_4	4
#define GPIO_PIN_NUMBER_5	5
#define GPIO_PIN_NUMBER_6	6
#define GPIO_PIN_NUMBER_7	7
#define GPIO_PIN_NUMBER_8	8
#define GPIO_PIN_NUMBER_9	9
#define GPIO_PIN_NUMBER_10	10
#define GPIO_PIN_NUMBER_11	11
#define GPIO_PIN_NUMBER_12	12
#define GPIO_PIN_NUMBER_13	13
#define GPIO_PIN_NUMBER_14	14
#define GPIO_PIN_NUMBER_15	15

// @GPIO_PIN_MODES
#define GPIO_MODE_INPUT			0
#define GPIO_MODE_OUTPUT		1
#define GPIO_MODE_AF			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FALLING	4
#define GPIO_MODE_IT_RISING		5
#define GPIO_MODE_IT_RFTRIGGER	6

// @GPIO_OUTPUT_TYPE
#define GPIO_OUTPUT_TYPE_PP	0
#define	GPIO_OUTPUT_TYPE_OD	1

// @GPIO_SPEED
#define GPIO_SPEED_LOW		0
#define	GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

// @GPIO_PULLUP_PULLDOWN
#define GPIO_PUPD_NO_PUPD	0
#define GPIO_PUPD_PU		1
#define GPIO_PUPD_PD		2

// @GPIO_ALTERNATE_FUNCTION
#define GPIO_ALTFUNC0	0
#define GPIO_ALTFUNC1	1
#define GPIO_ALTFUNC2	2
#define GPIO_ALTFUNC3	3
#define	GPIO_ALTFUNC4	4
#define GPIO_ALTFUNC5	5
#define GPIO_ALTFUNC6	6
#define GPIO_ALTFUNC7	7
#define	GPIO_ALTFUNC8	8
#define GPIO_ALTFUNC9	9
#define GPIO_ALTFUNC10	10
#define GPIO_ALTFUNC11	11
#define GPIO_ALTFUNC12	12
#define	GPIO_ALTFUNC13	13
#define GPIO_ALTFUNC14	14
#define GPIO_ALTFUNC15	15

// APIs
void GPIO_PCLKControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi);

void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* DRIVERS_STM32F767XX_GPIO_DRIVER_H_ */
