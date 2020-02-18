/*
 * stm32l475xx_flash_driver.h
 *
 *  Created on: Feb 18, 2020
 *      Author: H369169
 */

#ifndef INC_STM32L475XX_FLASH_DRIVER_H_
#define INC_STM32L475XX_FLASH_DRIVER_H_

/* Here go the system header files */
#include <stdint.h>

/* Here go the project includes */
#include <stm32l475xx.h>

typedef enum
{
	FLASH_STATUS_OK = 0,
	FLASH_STATUS_ERROR = 1
}FLASH_STATUS;

FLASH_STATUS FLASH_SetLatency(uint32_t freq_HCLK);

#endif /* INC_STM32L475XX_FLASH_DRIVER_H_ */
