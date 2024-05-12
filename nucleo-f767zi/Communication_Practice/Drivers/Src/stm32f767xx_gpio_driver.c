/*
 * stm32f767xx_gpio_driver.c
 *
 *  Created on: Mar 18, 2024
 *      Author: hojoon
 */


#include "stm32f767xx_gpio_driver.h"


/************************
 *	@fn			- GPIO_PCLKControl
 *
 *	@brief		- This function enables or disables peripheral clock for the given GPIO port
 *
 *	@param[in]	-
 *
 *	@return		- none
 *
 *	@note		- none
 */
void GPIO_PCLKControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
		else if(pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
		else if(pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_DI();
		}
	}
}

void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{
	uint32_t temp = 0;

	// 1. configure the mode of gpio pin
	if(pGPIOHandle->PinConfig.PinMode <= GPIO_MODE_ANALOG)
	{
		// non-interrupt mode
		temp = (pGPIOHandle->PinConfig.PinMode << (2 * pGPIOHandle->PinConfig.PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->PinConfig.PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		if(pGPIOHandle->PinConfig.PinMode == GPIO_MODE_IT_FALLING)
		{
			// Falling edge
			EXTI->FTSR |= (1 << pGPIOHandle->PinConfig.PinNumber);

			// Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->PinConfig.PinNumber);
		}
		else if(pGPIOHandle->PinConfig.PinMode == GPIO_MODE_IT_RISING)
		{
			// Rising Edge
			EXTI->RTSR |= (1 << pGPIOHandle->PinConfig.PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->PinConfig.PinNumber);
		}
		else if(pGPIOHandle->PinConfig.PinMode == GPIO_MODE_IT_RFTRIGGER)
		{
			// Rising Falling Trigger
			EXTI->RTSR |= (1 << pGPIOHandle->PinConfig.PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->PinConfig.PinNumber);
		}

		uint8_t temp1 = pGPIOHandle->PinConfig.PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->PinConfig.PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= (portcode << (temp2 * 4));

		EXTI->IMR |= (1 << pGPIOHandle->PinConfig.PinNumber);
	}

	temp = 0;

	// 2. configure the speed
	temp = (pGPIOHandle->PinConfig.PinSpeed << (2 * pGPIOHandle->PinConfig.PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// 3. configure the pupd settings
	temp = (pGPIOHandle->PinConfig.PinPuPdControl << (2 * pGPIOHandle->PinConfig.PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	// 4. configure the outputtype
	temp = (pGPIOHandle->PinConfig.PinOPType << pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->PinConfig.PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	// 5. configure the alternate functionality
	if(pGPIOHandle->PinConfig.PinMode == GPIO_MODE_AF)
	{
		// configure the alt function registers.
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->PinConfig.PinNumber / 8;
		temp2 = pGPIOHandle->PinConfig.PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->PinConfig.PinAltFunMode << (4 * temp2));
	}
}

void GPIO_DeInit(GPIO_RegDef_t* pGPIOx)
{
	// Using reset register
	// RCC_AHB1RSTR
	if(pGPIOx == GPIOA) { GPIOA_REG_RESET(); }
	else if(pGPIOx == GPIOB) { GPIOB_REG_RESET(); }
	else if(pGPIOx == GPIOC) { GPIOC_REG_RESET(); }
	else if(pGPIOx == GPIOD) { GPIOD_REG_RESET(); }
	else if(pGPIOx == GPIOE) { GPIOE_REG_RESET(); }
	else if(pGPIOx == GPIOF) { GPIOF_REG_RESET(); }
	else if(pGPIOx == GPIOG) { GPIOG_REG_RESET(); }
	else if(pGPIOx == GPIOH) { GPIOH_REG_RESET(); }
	else if(pGPIOx == GPIOI) { GPIOI_REG_RESET(); }
	else if(pGPIOx == GPIOJ) { GPIOJ_REG_RESET(); }
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx)
{
	uint16_t value;
	value = pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value)
{ 
	pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	// Use Exclusive OR
	pGPIOx->ODR ^= (1 << PinNumber);
}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_IPR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}
