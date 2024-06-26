/*
 * stm32f767xx_i2c.h
 *
 *  Created on: May 29, 2024
 *      Author: HoJoon
 */

#ifndef INC_STM32F767XX_I2C_H_
#define INC_STM32F767XX_I2C_H_

#include "stm32f767xx.h"

typedef struct {
	uint32_t SCLSpeed;
	uint8_t DeviceAddress;
	uint8_t ACKControl;
} I2C_Config_t;

typedef struct {
	I2C_RegDef_t* pI2Cx;
	I2C_Config_t I2CConfig;
} I2C_Handle_t;

/*
 * SCL Speed
 * Standard-mode	: up to 100kHz
 * Fast-mode		: up to 400kHz
 * Fast-mode Plus	: up to 1MHz
 */
#define I2C_SCL_SPEED_STANDARD		100000
#define I2C_SCL_SPEED_FAST			400000
#define I2C_SCL_SPEED_FAST_PLUS		1000000

/*
 * Mode Setting helper macros
 * Reference manual p.1206
 */
#define I2C_STANDARD_MODE_16MHZ_PRESC			3
#define I2C_STANDARD_MODE_16MHZ_SCLL			0x13
#define I2C_STANDARD_MODE_16MHZ_SCLH			0xF
#define I2C_STANDARD_MODE_16MHZ_SDADEL			0x2
#define I2C_STANDARD_MODE_16MHZ_SCLDEL			0x4

#define I2C_FAST_MODE_16MHZ_PRESC				1
#define I2C_FAST_MODE_16MHZ_SCLL				0x9
#define I2C_FAST_MODE_16MHZ_SCLH				0x3
#define I2C_FAST_MODE_16MHZ_SDADEL				0x2
#define I2C_FAST_MODE_16MHZ_SCLDEL				0x3

#define I2C_FAST_MODE_PLUS_16MHZ_PRESC			0
#define I2C_FAST_MODE_PLUS_16MHZ_SCLL			0x4
#define I2C_FAST_MODE_PLUS_16MHZ_SCLH			0x2
#define I2C_FAST_MODE_PLUS_16MHZ_SDADEL			0x0
#define I2C_FAST_MODE_PLUS_16MHZ_SCLDEL			0x2

/*
 * I2C Flags
 */
#define I2C_FLAG_BUSY		(1 << I2C_ISR_BUSY)
#define I2C_FLAG_OVR		(1 << I2C_ISR_OVR)
#define I2C_FLAG_STOPF		(1 << I2C_ISR_STOPF)
#define I2C_FLAG_NACKF		(1 << I2C_ISR_NACKF)
#define I2C_FLAG_TXE		(1 << I2C_ISR_TXE)
#define I2C_FLAG_ADDR		(1 << I2C_ISR_ADDR)

/*
 * ACK Control
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

void I2C_Peripheral_clock_control(I2C_RegDef_t* pI2Cx, uint8_t EnOrDi);
void I2C_Peripheral_control(I2C_RegDef_t* pI2Cx, uint8_t EnOrDi);

void I2C_Initialize(I2C_Handle_t* pI2CHandle);
void I2C_DeInitialize(I2C_RegDef_t* pI2Cx);

uint8_t I2C_Get_flag_status(I2C_RegDef_t *pI2Cx, uint32_t flag_name);
void I2C_Master_send_data(I2C_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slave_address);
void I2C_Master_receive_data(I2C_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t slave_address);

/*
 * Interrupt APIs
 */
void I2C_IRQ_interrupt_config(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQ_priority_config(uint8_t IRQNumber, uint32_t IRQPriority);

void I2C_Application_event_callback(I2C_Handle_t* pI2CHandle, uint8_t event);

#endif /* INC_STM32F767XX_I2C_H_ */
