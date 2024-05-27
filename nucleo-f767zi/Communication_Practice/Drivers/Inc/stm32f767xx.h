/*
 * stm32f767xx.h
 *
 *  Created on: Mar 16, 2024
 *      Author: hojoon
 */

#ifndef DRIVERS_STM32F767XX_H_
#define DRIVERS_STM32F767XX_H_

#include <stddef.h>
#include <stdint.h>

#define __weak		__attribute__((weak))

#define FLASH_AXIM_BASEADDR	0x08000000U
#define FLASH_ICTM_BASEADDR	0x00200000U
#define ROM_BASEADDR		0x00100000U
#define SRAM1_BASEADDR		0x20020000U
#define SRAM2_BASEADDR		0x2007C000U
#define SRAM				SRAM1_BASEADDR

#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U
#define AHB3PERIPH_BASEADDR		0x60000000U
#define INTERNALPERIPH_BASEADDR	0xE0000000U

// NVIC ISERx register
#define NVIC_ICTR		((volatile uint32_t*)0xE000E004)
#define NVIC_ISER0		((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1		((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2		((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3		((volatile uint32_t*)0xE000E10C)
#define NVIC_ISER4		((volatile uint32_t*)0xE000E110)
#define NVIC_ISER5		((volatile uint32_t*)0xE000E114)
#define NVIC_ISER6		((volatile uint32_t*)0xE000E118)
#define NVIC_ISER7		((volatile uint32_t*)0xE000E11C)

// NVIC ICERx register
#define NVIC_ICER0		((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1		((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2		((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3		((volatile uint32_t*)0xE000E18C)
#define NVIC_ICER4		((volatile uint32_t*)0xE000E190)
#define NVIC_ICER5		((volatile uint32_t*)0xE000E194)
#define NVIC_ICER6		((volatile uint32_t*)0xE000E198)
#define NVIC_ICER7		((volatile uint32_t*)0xE000E19C)

// NVIC Interrupt Priority Register
#define NVIC_IPR_BASE_ADDR		((volatile uint32_t*)0xE000E400)

#define NO_IPR_BITS_IMPLEMENTED	4

// GPIO BASE ADDRESS
#define GPIOA_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1000U)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1400U)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1800U)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1C00U)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASEADDR + 0x2000U)
#define GPIOJ_BASEADDR		(AHB1PERIPH_BASEADDR + 0x2400U)
#define GPIOK_BASEADDR		(AHB1PERIPH_BASEADDR + 0x2800U)
#define CRC_BASEADDR		(AHB1PERIPH_BASEADDR + 0x3000U)
#define RCC_BASEADDR		(AHB1PERIPH_BASEADDR + 0x3800U)

// APB1 PERIPHRALS
#define SPI2_BASEADDR		(APB1PERIPH_BASEADDR + 0x3800U)
#define SPI3_BASEADDR		(APB1PERIPH_BASEADDR + 0x3C00U)
#define USART2_BASEADDR		(APB1PERIPH_BASEADDR + 0x4400U)
#define USART3_BASEADDR		(APB1PERIPH_BASEADDR + 0x4800U)
#define UART4_BASEADDR		(APB1PERIPH_BASEADDR + 0x4C00U)
#define UART5_BASEADDR		(APB1PERIPH_BASEADDR + 0x5000U)
#define I2C1_BASEADDR		(APB1PERIPH_BASEADDR + 0x5400U)
#define I2C2_BASEADDR		(APB1PERIPH_BASEADDR + 0x5800U)
#define I2C3_BASEADDR		(APB1PERIPH_BASEADDR + 0x5C00U)
#define I2C4_BASEADDR		(APB1PERIPH_BASEADDR + 0x6000U)

// APB2 PERIPHRALS
#define USART1_BASEADDR		(APB2PERIPH_BASEADDR + 0x1000U)
#define USART6_BASEADDR		(APB2PERIPH_BASEADDR + 0x1400U)
#define SPI1_BASEADDR		(APB2PERIPH_BASEADDR + 0x3000U)
#define SPI4_BASEADDR		(APB2PERIPH_BASEADDR + 0x3400U)
#define SYSCFG_BASEADDR		(APB2PERIPH_BASEADDR + 0x3800U)
#define EXTI_BASEADDR		(APB2PERIPH_BASEADDR + 0x3C00U)

// Peripheral register definition structures

//GPIO
typedef struct
{
	volatile uint32_t MODER;	// ADDRESS OFFSET : 0x00
	volatile uint32_t OTYPER;	// ADDRESS OFFSET : 0x04
	volatile uint32_t OSPEEDR;	// ADDRESS OFFSET : 0x08
	volatile uint32_t PUPDR;	// ADDRESS OFFSET : 0x0C
	volatile uint32_t IDR;		// ADDRESS OFFSET : 0x10
	volatile uint32_t ODR;		// ADDRESS OFFSET : 0x14
	volatile uint32_t BSRR;		// ADDRESS OFFSET : 0x18
	volatile uint32_t LCKR;		// ADDRESS OFFSET : 0x1C
	volatile uint32_t AFR[2];	// ADDRESS OFFSET : 0x20 ~ 0x24
} GPIO_RegDef_t;

// RCC
typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED1;
	uint32_t RESERVED2;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED3;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED4;
	uint32_t RESERVED5;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED6;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED7;
	uint32_t RESERVED8;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED9;
	uint32_t RESERVED10;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR1;
	volatile uint32_t DCKCFGR2;
} RCC_RegDef_t;

// SYSCFG
typedef struct
{
	volatile uint32_t MEMRMP;		// Address offset : 0x00
	volatile uint32_t PMC;			// Address offset : 0x04
	volatile uint32_t EXTICR[4];	// Address offset : 0x08 ~ 0x14
	uint32_t RESERVED1;				// Address offset : 0x18
	volatile uint32_t CBR;			// Address offset : 0x1C
	volatile uint32_t CMPCR;		// Address offset : 0x20
} SYSCFG_RegDef_t;

// EXTI
typedef struct
{
	volatile uint32_t IMR;			// Address offset : 0x00
	volatile uint32_t EMR;			// Address offset : 0x04
	volatile uint32_t RTSR;			// Address offset : 0x08
	volatile uint32_t FTSR;			// Address offset : 0x0C
	volatile uint32_t SWIER;		// Address offset : 0x10
	volatile uint32_t PR;			// Address offset : 0x14
} EXTI_RegDef_t;

/* SPI */
typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
} SPI_RegDef_t;

#define GPIOA	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF	((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG	((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH	((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI	((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ	((GPIO_RegDef_t*)GPIOJ_BASEADDR)

#define RCC		((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI	((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1	((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2	((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3	((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4	((SPI_RegDef_t*)SPI4_BASEADDR)

// Clock enable macros for GPIOx peripherals
#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()	(RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN()	(RCC->AHB1ENR |= (1 << 9))

// Clock disable macros for GPIOx peripherals
#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 9))

// Reset GPIOx peripherals
#define GPIOA_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); } while(0)
#define GPIOJ_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 9)); (RCC->AHB1RSTR &= ~(1 << 9)); } while(0)

#define GPIO_BASEADDR_TO_CODE(x)	( (x == GPIOA) ? 0 : \
									  (x == GPIOB) ? 1 : \
									  (x == GPIOC) ? 2 : \
									  (x == GPIOD) ? 3 : \
									  (x == GPIOE) ? 4 : \
									  (x == GPIOF) ? 5 : \
									  (x == GPIOC) ? 6 : \
									  (x == GPIOH) ? 7 : \
									  (x == GPIOI) ? 8 : \
									  (x == GPIOJ) ? 9 : 0 )

/*
 * IRQ Number based on Reference Manual vector table
 * Page 313
 */
#define IRQ_NUMBER_EXTI0		6
#define IRQ_NUMBER_EXTI1		7
#define IRQ_NUMBER_EXTI2		8
#define IRQ_NUMBER_EXTI3		9
#define IRQ_NUMBER_EXIT4		10
#define IRQ_NUMBER_EXTI9_5		23
#define IRQ_NUMBER_EXIT15_10	40

#define IRQ_NUMBER_SPI1			35
#define IRQ_NUMBER_SPI2			36
#define IRQ_NUMBER_SPI3			51
#define IRQ_NUMBER_SPI4			84

// priority levels
#define NVIC_IRQ_PRIORITY_0		0
#define NVIC_IRQ_PRIORITY_15	15

// SYSCFG peripheral
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))

/* SPI Peripheral */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))

#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13))

// Some generic macros
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET

/* Bit position definitions of SPI peripheral */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSB_FIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RX_ONLY		10
#define SPI_CR1_CRCL		11
#define SPI_CR1_CRC_NEXT	12
#define SPI_CR1_CRC_EN		13
#define SPI_CR1_BIDI_OE		14
#define SPI_CR1_BIDI_MODE	15

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_NSSP		3
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7
#define SPI_CR2_DS			8
#define SPI_CR2_FRXTH		12
#define SPI_CR2_LDMA_RX		13
#define SPI_CR2_LDMA_TX		14

#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8
#define SPI_SR_FRLVL		9
#define SPI_SR_FTLVL		11

#include "stm32f767xx_gpio_driver.h"
#include "stm32f767xx_spi.h"

#endif /* DRIVERS_STM32F767XX_H_ */
