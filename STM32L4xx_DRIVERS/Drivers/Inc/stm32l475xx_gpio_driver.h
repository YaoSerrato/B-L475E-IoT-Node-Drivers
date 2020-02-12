/*
 * stm32l475xx_gpio_driver.h
 *
 *  Created on: Feb 11, 2020
 *      Author: yao
 */

#ifndef INC_STM32L475XX_GPIO_DRIVER_H_
#define INC_STM32L475XX_GPIO_DRIVER_H_

#include <stm32l475xx.h>
#include <stdint.h>

/* Handle structure for a GPIO pin */
typedef struct
{
	uint8_t	GPIO_PinNumber;
	uint8_t	GPIO_PinMode;
	uint8_t	GPIO_PinSpeed;
	uint8_t	GPIO_PinPuPdControl;
	uint8_t	GPIO_PinOType;
	uint8_t	GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t		*pGPIOx;			/* This will hold the base address of the peripheral */
	GPIO_PinConfig_t	GPIO_PinConfig;		/* This is for configuring pin */
}GPIO_Handle_t;

/* APIs supported by GPIO driver */
void GPIO_Init(void);						/* API for pin & port initialization */
void GPIO_DeInit(void);						/* API for pin & port deinitialization */
void GPIO_PeriClkControl(void);				/* API for configure the clock for GPIO peripheral */
void GPIO_ReadPin(void);					/* API for reading input pin */
void GPIO_ReadPort(void);					/* API for reading input port */
void GPIO_WritePin(void);					/* API for writing output pin */
void GPIO_WritePort(void);					/* API for writing output port */
void GPIO_TogglePin(void);					/* API for toggle output port */
void GPIO_IRQConfig(void);					/* API for configuring the IRQ */
void GPIO_IRQHandling(void);				/* API for processing the IRQ */


#endif /* INC_STM32L475XX_GPIO_DRIVER_H_ */
