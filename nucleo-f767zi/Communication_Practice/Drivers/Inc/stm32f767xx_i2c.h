/*
 * stm32f767xx_i2c.h
 *
 *  Created on: May 29, 2024
 *      Author: HoJoon
 */

#ifndef INC_STM32F767XX_I2C_H_
#define INC_STM32F767XX_I2C_H_

typedef struct {
	uint32_t SCLSpeed;
	uint8_t DeviceAddress;
	uint8_t ACKControl;
	uint16_t FMDutyCycle;
} I2C_Config_t;

typedef struct {
	I2C_RegDef_t* pI2Cx;
	I2C_Config_t I2CConfig;
} I2C_Handle_t;

#endif /* INC_STM32F767XX_I2C_H_ */
